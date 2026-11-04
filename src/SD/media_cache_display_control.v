`timescale 1ns/1ps

// Display half of media_image_cache_scheduler's switch transaction.
// Accept ONLY a ready, reserved target. The scheduler owns keys/auto requests;
// this module never selects a different file or queues a second direction.
// Payload {file ID, physical bank, geometry} stays held until switch_done.
//
// switch_done requires BOTH transition completion AND a tagged reader reload
// acknowledgement: old SDRAM returns have drained and its FIFO was reset.
// A frame boundary/index_commit alone must NEVER release the scheduler locks.
// All participating blocks must share a coordinated reset, locally released.
module media_cache_display_control #(
    parameter integer IMAGE_ID_WIDTH = 4,
    parameter integer FADE_FRAMES = 16
) (
    input wire sys_clk,
    input wire sys_rst_n,
    input wire switch_valid,
    output wire switch_ready,
    input wire [IMAGE_ID_WIDTH-1:0] switch_image_id,
    input wire [1:0] switch_bank,
    input wire switch_initial,
    input wire switch_auto,
    input wire switch_previous,
    // Per bank: {pixel_count[19:0], height[9:0], width[9:0]}.
    input wire [159:0] image_geometry,
    output reg switch_done,
    output wire request_busy,
    output wire [1:0] pending_bank,
    // These describe the actual pixel commit, including during fade-in.
    // Scheduler active_* intentionally updates later, at switch_done.
    output reg display_valid,
    output reg [IMAGE_ID_WIDTH-1:0] display_image_id,
    output reg [1:0] display_bank,
    output reg display_commit,

    input wire pix_clk,
    input wire pix_rst_n,
    input wire frame_start,
    output reg active_valid,
    output reg [IMAGE_ID_WIDTH-1:0] active_image_id,
    output reg [1:0] active_bank,
    output reg index_commit,
    output reg active_epoch,
    output wire [15:0] active_width,
    output wire [15:0] active_height,
    output wire [21:0] active_word_count,
    // Connect to media_frame_reader with DISPLAY_EPOCH_TAG=1. This is NOT
    // frame_read_done or a generic frame pulse; all three are pixel-domain.
    input wire reader_reload_done,
    input wire reader_reload_epoch,
    input wire [1:0] reader_reload_bank,
    output wire fade_busy,
    output wire reveal_active,
    output wire [1:0] reveal_effect,
    output wire [8:0] reveal_progress,
    output wire [4:0] fade_level
);
    localparam [1:0] S_IDLE=0, S_START=1, S_EFFECT=2, S_WAIT=3;
    localparam integer PAYLOAD_WIDTH = IMAGE_ID_WIDTH + 42;
    reg [1:0] state;
    reg [PAYLOAD_WIDTH-1:0] held_payload;
    reg held_initial, held_auto;
    reg [1:0] next_effect;
    reg request_toggle;
    reg commit_sync1, commit_sync2, commit_seen_sys;
    reg retired_sync1, retired_sync2;
    reg [PAYLOAD_WIDTH-1:0] payload_sync1, payload_sync2, requested_payload;
    reg request_sync1, request_sync2, request_seen_pix;
    reg commit_pending;
    reg [39:0] active_geometry;
    reg commit_toggle_pix, retired_toggle_pix, retirement_pending;
    wire effect_commit;
    wire start_effect = state == S_START && !held_initial;

    assign pending_bank = held_payload[41:40];
    assign active_width = {6'd0,active_geometry[9:0]};
    assign active_height = {6'd0,active_geometry[19:10]};
    assign active_word_count = {2'd0,active_geometry[39:20]};
    // Keep a one-cycle completion bubble: the scheduler consumes switch_done
    // before it is allowed to hand over another target.
    assign request_busy = state != S_IDLE || switch_done;
    assign switch_ready = sys_rst_n && !request_busy && !fade_busy;

    function [39:0] geometry_for;
        input [1:0] bank;
        begin
            case (bank)
                0: geometry_for=image_geometry[39:0];
                1: geometry_for=image_geometry[79:40];
                2: geometry_for=image_geometry[119:80];
                default: geometry_for=image_geometry[159:120];
            endcase
        end
    endfunction

    video_reveal_transition #(.FRAMES(FADE_FRAMES)) u_transition (
        .ctrl_clk(sys_clk), .ctrl_rst_n(sys_rst_n),
        .request(start_effect), .automatic_in(held_auto), .effect_in(next_effect),
        .commit_request(effect_commit), .busy(fade_busy),
        .pix_clk(pix_clk), .pix_rst_n(pix_rst_n),
        .player_active(active_valid), .frame_start(frame_start),
        .index_commit(index_commit), .reveal_active(reveal_active),
        .effect_out(reveal_effect), .progress(reveal_progress), .fade_level(fade_level)
    );

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            state<=S_IDLE; held_payload<=0;
            held_initial<=0; held_auto<=0; next_effect<=0;
            request_toggle<=0; switch_done<=0;
            commit_sync1<=0; commit_sync2<=0; commit_seen_sys<=0;
            retired_sync1<=0; retired_sync2<=0;
            display_valid<=0; display_image_id<=0; display_bank<=0;
            display_commit<=0;
        end else begin
            commit_sync1<=commit_toggle_pix; commit_sync2<=commit_sync1;
            retired_sync1<=retired_toggle_pix; retired_sync2<=retired_sync1;
            switch_done<=0; display_commit<=0;
            if (commit_sync2 != commit_seen_sys) begin
                commit_seen_sys<=commit_sync2;
                display_valid<=1;
                display_image_id<=held_payload[PAYLOAD_WIDTH-1:42];
                display_bank<=held_payload[41:40];
                display_commit<=1;
            end
            case (state)
                S_IDLE: if (switch_valid && switch_ready) begin
                    held_payload<={switch_image_id,switch_bank,geometry_for(switch_bank)};
                    held_initial<=switch_initial;
                    held_auto<=switch_auto;
                    state<=S_START;
                end
                S_START: begin
                    // With FADE_FRAMES=0 the pass-through event occurs on this
                    // very cycle. Do not miss it by first waiting in S_EFFECT.
                    if (held_initial || effect_commit) begin
                        request_toggle<=~request_toggle;
                        state<=S_WAIT;
                    end else state<=S_EFFECT;
                end
                S_EFFECT: if (effect_commit) begin
                    request_toggle<=~request_toggle;
                    state<=S_WAIT;
                end
                S_WAIT: if (retired_sync2 == request_toggle && !fade_busy) begin
                    switch_done<=1;
                    if (!held_initial && held_auto) next_effect<=next_effect+1'b1;
                    state<=S_IDLE;
                end
            endcase
        end
    end

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            payload_sync1<=0; payload_sync2<=0; requested_payload<=0;
            request_sync1<=0; request_sync2<=0; request_seen_pix<=0;
            commit_pending<=0; active_valid<=0; active_image_id<=0;
            active_bank<=0; active_geometry<=0; index_commit<=0;
            active_epoch<=0; commit_toggle_pix<=0; retired_toggle_pix<=0;
            retirement_pending<=0;
        end else begin
            // Bundled-data mailbox: sys holds the payload before the request
            // toggle and until retirement returns; capture after synchronization.
            payload_sync1<=held_payload; payload_sync2<=payload_sync1;
            request_sync1<=request_toggle; request_sync2<=request_sync1;
            index_commit<=0;
            if (request_sync2 != request_seen_pix && !commit_pending) begin
                requested_payload<=payload_sync2;
                request_seen_pix<=request_sync2;
                commit_pending<=1;
            end
            if (frame_start && commit_pending) begin
                active_image_id<=requested_payload[PAYLOAD_WIDTH-1:42];
                active_bank<=requested_payload[41:40];
                active_geometry<=requested_payload[39:0];
                active_valid<=1; index_commit<=1;
                active_epoch<=request_seen_pix;
                commit_toggle_pix<=request_seen_pix;
                commit_pending<=0; retirement_pending<=1;
            end else if (retirement_pending && reader_reload_done &&
                         reader_reload_epoch == active_epoch &&
                         reader_reload_bank == active_bank) begin
                retired_toggle_pix<=active_epoch;
                retirement_pending<=0;
            end
        end
    end
endmodule
