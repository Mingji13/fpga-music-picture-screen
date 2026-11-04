`timescale 1ns/1ps

// 手动下一张/上一张：淡出 -> 全黑切图 -> 淡入。
// 自动轮播：直接提交新图 -> 黑底从左向右揭示；首图没有转场请求，直接显示。
//
// 接口约定：
// 1. next/prev/auto_event_in 是 ctrl_clk 域的单周期事件；忙时丢弃新请求。
//    同拍优先级 next > prev > auto；手动方向在接收时保存，全黑后才输出。
// 2. player_active、frame_start、index_commit 都属于 pix_clk 域。
// 3. 手动全黑时、自动进入等待状态时，输出一次对应方向事件；
//    上层实际提交切图后返回 index_commit（须在消隐期，早于首个有效像素）。
//    上层接入时需保证请求可执行；没有提交时 HOLD 会一直等待。
// 4. 两个复位来自同一次系统/播放器复位，各自在对应时钟域同步释放。
module video_fade_transition #(
    // 0=旁路；启用时取 1..256 且能整除 256，例如 8、16、32。
    parameter integer FADE_FRAMES = 0,
    parameter integer IMAGE_WIDTH = 640, // 1..1280, displayed image width
    parameter integer RUNTIME_GEOMETRY = 0
) (
    input  wire       ctrl_clk,
    input  wire       ctrl_rst_n,
    input  wire       next_event_in,
    input  wire       prev_event_in,
    input  wire       auto_event_in,
    output wire       next_event_out,
    output wire       prev_event_out,
    output wire       fade_busy,

    input  wire       pix_clk,
    input  wire       pix_rst_n,
    input  wire       player_active,
    input  wire       frame_start,
    input  wire       index_commit,
    output wire [7:0] alpha_out,
    output wire       fade_active,
    output wire       wipe_active,
    output wire [11:0] wipe_boundary, // 图片坐标，边界左侧显示新图
    input wire [11:0] displayed_width
);
    generate
        if (FADE_FRAMES == 0) begin : g_bypass
            assign next_event_out = next_event_in || (auto_event_in && !prev_event_in);
            assign prev_event_out = prev_event_in && !next_event_in;
            assign fade_busy = 1'b0;
            assign alpha_out = 8'd255;
            assign fade_active = 1'b0;
            assign wipe_active = 1'b0;
            assign wipe_boundary = RUNTIME_GEOMETRY ? displayed_width : IMAGE_WIDTH;
        end else begin : g_fade
            localparam integer STEP = 256 / FADE_FRAMES;
            localparam integer CNT_W = (FADE_FRAMES <= 2) ? 2 :
                                      $clog2(FADE_FRAMES + 1);

            localparam [1:0] S_IDLE = 2'd0;
            localparam [1:0] S_OUT  = 2'd1;
            localparam [1:0] S_HOLD = 2'd2;
            localparam [1:0] S_IN   = 2'd3;

            reg req_toggle;
            reg request_previous;
            reg request_auto;
            reg ack_seen;
            reg commit_sync1, commit_sync2;
            reg done_sync1, done_sync2;

            reg [1:0] state;
            reg [CNT_W-1:0] ramp;
            reg req_sync1, req_sync2, req_seen;
            reg commit_toggle;
            reg done_toggle;
            reg wipe_mode;

            wire fade_running = (state != S_IDLE);
            wire [CNT_W+8:0] ramp_alpha = ramp * STEP;
            wire [11:0] image_width = RUNTIME_GEOMETRY ? displayed_width : IMAGE_WIDTH;
            wire [CNT_W+11:0] wipe_scaled = ramp * image_width;
            reg [11:0] wipe_boundary_q;
            // Geometry and ramp change in blanking; register the variable
            // multiplication before it reaches the per-pixel colour mask.
            always @(posedge pix_clk or negedge pix_rst_n) begin
                if (!pix_rst_n) wipe_boundary_q <= 12'd0;
                else wipe_boundary_q <= wipe_active ? (wipe_scaled / FADE_FRAMES) : image_width;
            end
            assign fade_active = fade_running;
            assign wipe_active = (state == S_IN) && wipe_mode;
            // 先乘再除，避免 640/FADE_FRAMES 提前取整（如 FRAMES=256）。
            assign wipe_boundary = RUNTIME_GEOMETRY ? wipe_boundary_q :
                                   (wipe_active ? (wipe_scaled / FADE_FRAMES) : IMAGE_WIDTH);
            // 请求可能在一帧中间到达：ramp=FADE_FRAMES 时仍保持全亮，
            // 直到下一次 frame_start 才开始减小亮度。
            assign alpha_out = (state == S_IDLE) ? 8'd255 :
                               wipe_mode ? 8'd255 :
                               (state == S_HOLD) ? 8'd0 :
                               (ramp == FADE_FRAMES) ? 8'd255 :
                               ramp_alpha[7:0];

            always @(posedge pix_clk or negedge pix_rst_n) begin
                if (!pix_rst_n) begin
                    state <= S_IDLE;
                    ramp <= {CNT_W{1'b0}};
                    req_sync1 <= 1'b0;
                    req_sync2 <= 1'b0;
                    req_seen <= 1'b0;
                    commit_toggle <= 1'b0;
                    done_toggle <= 1'b0;
                    wipe_mode <= 1'b0;
                end else begin
                    req_sync1 <= req_toggle;
                    req_sync2 <= req_sync1;

                    case (state)
                    S_IDLE: begin
                        if (req_sync2 != req_seen) begin
                            req_seen <= req_sync2;
                            // 尚未显示首图的请求直接消耗，不留到出图后补播。
                            if (player_active) begin
                                // request_auto 与 req_toggle 同拍锁存，在整个握手
                                // 完成前保持不变；同步后的请求到达才读取这位属性。
                                wipe_mode <= request_auto;
                                if (request_auto) begin
                                    state <= S_HOLD;
                                    ramp <= {CNT_W{1'b0}};
                                    commit_toggle <= ~commit_toggle;
                                end else begin
                                    state <= S_OUT;
                                    ramp <= FADE_FRAMES;
                                end
                            end else begin
                                done_toggle <= req_sync2;
                            end
                        end
                    end
                    S_OUT: begin
                        if (frame_start) begin
                            if (ramp == 0) begin
                                state <= S_HOLD;
                                commit_toggle <= ~commit_toggle;
                            end else begin
                                ramp <= ramp - 1'b1;
                            end
                        end
                    end
                    S_HOLD: begin
                        // 不是“等固定时间”，而是等图片真正提交完成。
                        // 手动等待期间为黑色；自动等待期间保留旧图全亮。
                        if (index_commit) begin
                            state <= S_IN;
                            ramp <= {CNT_W{1'b0}};
                        end
                    end
                    S_IN: begin
                        if (frame_start) begin
                            if (ramp >= FADE_FRAMES - 1) begin
                                state <= S_IDLE;
                                wipe_mode <= 1'b0;
                                done_toggle <= req_seen;
                            end else
                                ramp <= ramp + 1'b1;
                        end
                    end
                    default: begin
                        state <= S_IDLE;
                        ramp <= {CNT_W{1'b0}};
                        wipe_mode <= 1'b0;
                        done_toggle <= req_seen;
                    end
                    endcase
                end
            end

            // 相对课程原版，忙标志改由“请求是否完成”确定：发请求即忙，
            // 淡入结束才确认完成，避免 busy 跨域返回前连续翻转 req_toggle。
            // 单独同步完成标记，不依赖两组独立同步信号恰好同拍到达。
            assign fade_busy = (req_toggle != done_sync2);
            assign next_event_out = (commit_sync2 != ack_seen) && !request_previous;
            assign prev_event_out = (commit_sync2 != ack_seen) && request_previous;

            always @(posedge ctrl_clk or negedge ctrl_rst_n) begin
                if (!ctrl_rst_n) begin
                    req_toggle <= 1'b0;
                    request_previous <= 1'b0;
                    request_auto <= 1'b0;
                    ack_seen <= 1'b0;
                    commit_sync1 <= 1'b0;
                    commit_sync2 <= 1'b0;
                    done_sync1 <= 1'b0;
                    done_sync2 <= 1'b0;
                end else begin
                    done_sync1 <= done_toggle;
                    done_sync2 <= done_sync1;
                    commit_sync1 <= commit_toggle;
                    commit_sync2 <= commit_sync1;
                    if ((next_event_in || prev_event_in || auto_event_in) && !fade_busy) begin
                        req_toggle <= ~req_toggle;
                        request_previous <= !next_event_in && prev_event_in;
                        request_auto <= !next_event_in && !prev_event_in && auto_event_in;
                    end
                    if (commit_sync2 != ack_seen)
                        ack_seen <= commit_sync2;
                end
            end
        end
    endgenerate
endmodule
