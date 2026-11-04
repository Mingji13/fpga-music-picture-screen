`timescale 1ns/1ps

module media_player_control #(
    parameter [31:0] DEFAULT_PERIOD_CYCLES = 32'd250000000,
    parameter [31:0] MIN_PERIOD_CYCLES = 32'd50000000,
    parameter [31:0] MAX_PERIOD_CYCLES = 32'd3000000000,
    parameter integer EXTERNAL_AUTO_TRANSITION = 0,
    parameter integer RUNTIME_GEOMETRY = 0
) (
    input  wire        sys_clk,
    input  wire        sys_rst_n,
    input  wire        next_event,
    input  wire        prev_event,
    input  wire        toggle_auto_event,
    input  wire        auto_pause,       // 转场期间暂停轮播；不阻挡转场放行的切图请求
    input  wire        period_cfg_valid,
    input  wire [31:0] period_cfg_cycles,
    input  wire        auto_cfg_valid,
    input  wire        auto_cfg_enabled,
    input  wire [3:0]  image_complete,
    output reg         auto_enabled,
    output reg  [31:0] active_period_cycles,
    output reg  [1:0]  pending_index,
    output reg  [7:0]  control_error,
    output wire        request_busy,
    output wire        manual_switch_ready,
    output wire        auto_next_event, // 自动到期交给转场；转场随后通过 next_event 放行
    input  wire        pix_clk,
    input  wire        pix_rst_n,
    input  wire        frame_start,
    output reg  [1:0]  active_index,
    output reg         index_commit,
    output reg         active_valid,
    // Per bank: {pixel_count[19:0], height[9:0], width[9:0]}.
    // A slot is published only through image_complete, after its metadata is stable.
    input wire [159:0] image_geometry,
    output wire [15:0] active_width,
    output wire [15:0] active_height,
    output wire [21:0] active_word_count
);
    localparam ERR_NONE = 8'h00;
    localparam ERR_PERIOD_RANGE = 8'h51;
    localparam ERR_NO_RESOURCE = 8'h53;

    reg [31:0] auto_counter;
    reg next_req_pending;
    reg prev_req_pending;
    reg request_toggle;
    reg ack_toggle_pix;
    reg ack_sync1, ack_sync2;
    reg [1:0] index_sync1, index_sync2;
    reg req_sync1, req_sync2;
    reg req_seen_pix;
    reg [1:0] requested_index_pix;
    reg commit_pending_pix;
    reg initial_request_done;
    reg startup_armed;
    reg [39:0] pending_geometry;
    reg [39:0] geometry_sync1, geometry_sync2, requested_geometry_pix, active_geometry;
    assign active_width = RUNTIME_GEOMETRY ? {6'd0,active_geometry[9:0]} : 16'd640;
    assign active_height = RUNTIME_GEOMETRY ? {6'd0,active_geometry[19:10]} : 16'd480;
    assign active_word_count = RUNTIME_GEOMETRY ? {2'd0,active_geometry[39:20]} : 22'd307200;

    function [39:0] geometry_for;
        input [1:0] index;
        begin
            case (index)
                0: geometry_for = image_geometry[39:0];
                1: geometry_for = image_geometry[79:40];
                2: geometry_for = image_geometry[119:80];
                default: geometry_for = image_geometry[159:120];
            endcase
        end
    endfunction

    assign request_busy = request_toggle != ack_sync2;

    // 首图已提交、当前没有切图握手，且有另一张已完成图片，才允许先淡出。
    // image_complete 在正常播放期间只增加；清除它的播放器复位也会复位转场。
    wire [3:0] other_complete = image_complete & ~(4'b0001 << pending_index);
    assign manual_switch_ready = initial_request_done && !request_busy &&
                                 image_complete[pending_index] && (|other_complete);

    // 组合事件让转场在计时到期的同一拍占有请求，避免晚一拍的按键抢入。
    // auto_pause 包含当拍手动请求，故手动与自动恰好同时到达时手动优先。
    assign auto_next_event = EXTERNAL_AUTO_TRANSITION && auto_enabled &&
                             manual_switch_ready && !auto_pause &&
                             !toggle_auto_event && !auto_cfg_valid && !period_cfg_valid &&
                             !next_event && !next_req_pending && !prev_event && !prev_req_pending &&
                             (auto_counter >= active_period_cycles - 1'b1);

    function [2:0] next_available;
        input [1:0] current;
        input [3:0] complete;
        integer step;
        reg [1:0] candidate;
        begin
            next_available = {1'b0, current};
            for (step = 1; step <= 4; step = step + 1) begin
                candidate = current + step[1:0];
                if (!next_available[2] && complete[candidate])
                    next_available = {1'b1, candidate};
            end
        end
    endfunction

    function [2:0] prev_available;
        input [1:0] current;
        input [3:0] complete;
        integer step;
        reg [1:0] candidate;
        begin
            prev_available = {1'b0, current};
            for (step = 1; step <= 4; step = step + 1) begin
                candidate = current - step[1:0];
                if (!prev_available[2] && complete[candidate])
                    prev_available = {1'b1, candidate};
            end
        end
    endfunction

    function [2:0] first_available;
        input [3:0] complete;
        begin
            if (complete[0])
                first_available = {1'b1, 2'd0};
            else if (complete[1])
                first_available = {1'b1, 2'd1};
            else if (complete[2])
                first_available = {1'b1, 2'd2};
            else if (complete[3])
                first_available = {1'b1, 2'd3};
            else
                first_available = 3'b000;
        end
    endfunction

    wire [2:0] first_completed = first_available(image_complete);

    // 仅在 request_busy=0 时调用：此时 pending_index 已获视频域确认，
    // 就是当前显示编号。直接用它选下一张，避免额外副本晚更新一拍。
    task issue_switch_request;
        input go_previous;
        reg [2:0] selected;
        begin
            selected = go_previous ? prev_available(pending_index, image_complete) :
                                     next_available(pending_index, image_complete);
            if (!selected[2]) begin
                control_error <= ERR_NO_RESOURCE;
            end else begin
                pending_index <= selected[1:0];
                pending_geometry <= RUNTIME_GEOMETRY ? geometry_for(selected[1:0]) : 40'd0;
                request_toggle <= ~request_toggle;
                auto_counter <= 32'd0;
            end
        end
    endtask

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            auto_enabled <= 1'b0;
            active_period_cycles <= DEFAULT_PERIOD_CYCLES;
            pending_index <= 2'd0;
            pending_geometry <= 40'd0;
            control_error <= ERR_NONE;
            auto_counter <= 32'd0;
            next_req_pending <= 1'b0;
            prev_req_pending <= 1'b0;
            request_toggle <= 1'b0;
            initial_request_done <= 1'b0;
            // 真实上电时loader会先把image_complete清零；测试或复位期间
            // 若输入已经是非零，不能把它误认为“刚完成首图”。
            startup_armed <= 1'b0;
            ack_sync1 <= 1'b0;
            ack_sync2 <= 1'b0;
        end else begin
            ack_sync1 <= ack_toggle_pix;
            ack_sync2 <= ack_sync1;
            // 等待帧边界提交时，每个方向最多保留一次请求。
            if (next_event)
                next_req_pending <= 1'b1;
            if (prev_event)
                prev_req_pending <= 1'b1;

            if (period_cfg_valid) begin
                if (period_cfg_cycles < MIN_PERIOD_CYCLES ||
                    period_cfg_cycles > MAX_PERIOD_CYCLES) begin
                    control_error <= ERR_PERIOD_RANGE;
                end else begin
                    active_period_cycles <= period_cfg_cycles;
                    auto_counter <= 32'd0;
                end
            end

            if (auto_cfg_valid) begin
                auto_enabled <= auto_cfg_enabled;
                auto_counter <= 32'd0;
            end

            // 首张图片完成后也走同一条 CDC/帧边界提交路径。
            // 即使首张正好是 bank0，也要发一次请求，避免 reader 在未准备好
            // 时提前开始读帧；active_valid 会在视频域真正提交后才拉高。
            if (image_complete == 4'd0)
                startup_armed <= 1'b1;

            if (!initial_request_done && startup_armed && first_completed[2] &&
                !request_busy) begin
                pending_index <= first_completed[1:0];
                pending_geometry <= RUNTIME_GEOMETRY ? geometry_for(first_completed[1:0]) : 40'd0;
                request_toggle <= ~request_toggle;
                initial_request_done <= 1'b1;
                startup_armed <= 1'b0;
                auto_counter <= 32'd0;
            end

            if (toggle_auto_event) begin
                auto_enabled <= ~auto_enabled;
                auto_counter <= 32'd0;
            end

            // 优先级沿用原工程：下一张、上一张、自动轮播。
            if (request_busy) begin
                auto_counter <= 32'd0;
            end else if (next_event || next_req_pending) begin
                next_req_pending <= 1'b0;
                issue_switch_request(1'b0);
            end else if (prev_event || prev_req_pending) begin
                prev_req_pending <= 1'b0;
                issue_switch_request(1'b1);
            end else if (auto_pause || toggle_auto_event || auto_cfg_valid || period_cfg_valid) begin
                // 转场结束后重新计时；接收手动转场的当拍也必须暂停，
                // 避免自动计时恰好到期时，在淡出开始前抢先发起切图。
                auto_counter <= 32'd0;
            end else if (auto_enabled) begin
                if (EXTERNAL_AUTO_TRANSITION && !manual_switch_ready)
                    auto_counter <= 32'd0;
                else if (auto_counter >= active_period_cycles - 1'b1) begin
                    if (EXTERNAL_AUTO_TRANSITION)
                        auto_counter <= 32'd0;
                    else
                        issue_switch_request(1'b0);
                end else
                    auto_counter <= auto_counter + 1'b1;
            end else begin
                auto_counter <= 32'd0;
            end
        end
    end

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            index_sync1 <= 2'd0;
            index_sync2 <= 2'd0;
            geometry_sync1 <= 40'd0;
            geometry_sync2 <= 40'd0;
            requested_geometry_pix <= 40'd0;
            active_geometry <= 40'd0;
            req_sync1 <= 1'b0;
            req_sync2 <= 1'b0;
            req_seen_pix <= 1'b0;
            requested_index_pix <= 2'd0;
            commit_pending_pix <= 1'b0;
            active_index <= 2'd0;
            index_commit <= 1'b0;
            active_valid <= 1'b0;
            ack_toggle_pix <= 1'b0;
        end else begin
            index_sync1 <= pending_index;
            index_sync2 <= index_sync1;
            // Same held mailbox and request/ack lifetime as pending_index.
            geometry_sync1 <= pending_geometry;
            geometry_sync2 <= geometry_sync1;
            req_sync1 <= request_toggle;
            req_sync2 <= req_sync1;
            index_commit <= 1'b0;

            if (req_sync2 != req_seen_pix && !commit_pending_pix) begin
                req_seen_pix <= req_sync2;
                requested_index_pix <= index_sync2;
                requested_geometry_pix <= geometry_sync2;
                commit_pending_pix <= 1'b1;
            end

            if (frame_start && commit_pending_pix) begin
                active_index <= requested_index_pix;
                active_geometry <= requested_geometry_pix;
                index_commit <= 1'b1;
                active_valid <= 1'b1;
                commit_pending_pix <= 1'b0;
                ack_toggle_pix <= req_seen_pix;
            end
        end
    end
endmodule


