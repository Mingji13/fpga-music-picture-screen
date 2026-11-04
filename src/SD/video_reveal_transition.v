`timescale 1ns/1ps

// Shared transaction: manual fade-out/commit/fade-in, or automatic reveal.
// Mode/effect are bundled with request and held until busy drops. The first
// image bypasses this module; reader retirement is checked by its caller.
module video_reveal_transition #(
    parameter integer FRAMES = 16 // 0 bypass, otherwise a power of two <= 256
) (
    input wire ctrl_clk, ctrl_rst_n, request,
    input wire automatic_in,
    input wire [1:0] effect_in,
    output wire commit_request, busy,
    input wire pix_clk, pix_rst_n, player_active, frame_start, index_commit,
    output wire reveal_active,
    output wire [1:0] effect_out,
    output wire [8:0] progress,
    output wire [4:0] fade_level // 0..16, exact black/original at endpoints
);
    generate if (FRAMES == 0) begin : g_bypass
        assign commit_request = request;
        assign busy = 1'b0;
        assign reveal_active = 1'b0;
        assign effect_out = 0;
        assign progress = 9'd256;
        assign fade_level = 5'd16;
    end else begin : g_reveal
        localparam [1:0] IDLE=0, WAIT_COMMIT=1, REVEAL=2, FADE_OUT=3;
        localparam integer COUNT_WIDTH = $clog2(FRAMES+1);
        localparam integer PROGRESS_SHIFT = 8-$clog2(FRAMES);
        reg request_toggle, commit_seen;
        reg [1:0] held_effect;
        reg held_automatic, automatic_q;
        reg commit_sync1, commit_sync2, done_sync1, done_sync2;
        reg request_sync1, request_sync2, request_seen;
        reg commit_toggle, done_toggle;
        reg [1:0] state, effect_q;
        reg [COUNT_WIDTH-1:0] ramp_q;
        wire [8:0] phase_progress = {9'd0,ramp_q} << PROGRESS_SHIFT;
        wire manual_running = state != IDLE && !automatic_q;
        assign busy = request_toggle != done_sync2;
        assign commit_request = commit_sync2 != commit_seen;
        assign reveal_active = state == REVEAL && automatic_q;
        assign effect_out = effect_q;
        assign progress = reveal_active ? phase_progress : 9'd256;
        assign fade_level = manual_running ? phase_progress[8:4] : 5'd16;

        always @(posedge ctrl_clk or negedge ctrl_rst_n) begin
            if (!ctrl_rst_n) begin
                request_toggle<=0; held_effect<=0; held_automatic<=0; commit_seen<=0;
                commit_sync1<=0; commit_sync2<=0; done_sync1<=0; done_sync2<=0;
            end else begin
                commit_sync1<=commit_toggle; commit_sync2<=commit_sync1;
                done_sync1<=done_toggle; done_sync2<=done_sync1;
                if (request && !busy) begin
                    held_effect<=effect_in;
                    held_automatic<=automatic_in;
                    request_toggle<=~request_toggle;
                end
                if (commit_request) commit_seen<=commit_sync2;
            end
        end
        always @(posedge pix_clk or negedge pix_rst_n) begin
            if (!pix_rst_n) begin
                request_sync1<=0; request_sync2<=0; request_seen<=0;
                commit_toggle<=0; done_toggle<=0; state<=IDLE;
                effect_q<=0; ramp_q<=0; automatic_q<=0;
            end else begin
                request_sync1<=request_toggle; request_sync2<=request_sync1;
                case (state)
                    IDLE: if (request_sync2 != request_seen) begin
                        request_seen<=request_sync2;
                        if (player_active) begin
                            effect_q<=held_effect;
                            automatic_q<=held_automatic;
                            ramp_q<=FRAMES;
                            if (held_automatic) begin
                                commit_toggle<=~commit_toggle;
                                state<=WAIT_COMMIT;
                            end else state<=FADE_OUT;
                        end else done_toggle<=request_sync2;
                    end
                    FADE_OUT: if (frame_start) begin
                        if (ramp_q <= 1) begin
                            ramp_q<=0;
                            commit_toggle<=~commit_toggle;
                            state<=WAIT_COMMIT;
                        end else ramp_q<=ramp_q-1'b1;
                    end
                    // Manual stays black until the actual bank commit;
                    // automatic keeps the old picture visible while waiting.
                    WAIT_COMMIT: if (index_commit) begin
                        ramp_q<=0; state<=REVEAL;
                    end
                    REVEAL: if (frame_start) begin
                        if (ramp_q >= FRAMES-1) begin
                            state<=IDLE; done_toggle<=request_seen;
                        end else ramp_q<=ramp_q+1'b1;
                    end
                    default: begin state<=IDLE; done_toggle<=request_seen; end
                endcase
            end
        end
    end endgenerate
endmodule

// A FIVE-clock visibility pipeline runs alongside the scaler's FIVE-clock
// RGB/timing pipeline. Only this single bit is delayed; no second RGB/X/Y
// pipeline or full image buffer is required. Use the SAME raw raster X/Y as
// the scaler. Geometry/progress settle during blanking. Outside the image,
// scaler RGB is already black, so this mask need not duplicate its window.
// 0: left->right; 1: top->bottom; 2: centre->sides; 3: horizontal blinds.
module video_reveal_mask (
    input wire pclk, video_rst_n,
    input wire reveal_active,
    input wire [1:0] effect,
    input wire [8:0] progress,
    input wire [11:0] image_width, image_height, image_left, image_top,
    input wire [11:0] in_x, in_y,
    output reg show_pixel
);
    reg [20:0] width_product, height_product;
    reg [11:0] boundary_x, boundary_y, curtain_left, curtain_right;
    reg [5:0] blind_boundary;
    // Multiplication and the curtain edge add/subtract are separated from
    // per-pixel comparisons. All four modes share these boundary registers.
    always @(posedge pclk or negedge video_rst_n) begin
        if (!video_rst_n) begin
            width_product<=0; height_product<=0;
            boundary_x<=0; boundary_y<=0; curtain_left<=0; curtain_right<=0;
            blind_boundary<=0;
        end else begin
            width_product<=image_width*progress;
            height_product<=image_height*progress;
            boundary_x<=width_product[19:8];
            boundary_y<=height_product[19:8];
            curtain_left<=(image_width-width_product[19:8])>>1;
            curtain_right<=({1'b0,image_width}+{1'b0,width_product[19:8]})>>1;
            blind_boundary<=progress[8:3]; // 32-pixel horizontal strips
        end
    end
    reg [11:0] relative_x, relative_y;
    reg active0, active1;
    reg [1:0] effect0, effect1;
    reg [3:0] visible1;
    reg visible2, visible3;
    always @(posedge pclk or negedge video_rst_n) begin
        if (!video_rst_n) begin
            relative_x<=0; relative_y<=0;
            active0<=0; active1<=0;
            effect0<=0; effect1<=0; visible1<=0;
            visible2<=0; visible3<=0; show_pixel<=0;
        end else begin
            relative_x<=in_x-image_left; relative_y<=in_y-image_top;
            active0<=reveal_active; active1<=active0;
            effect0<=effect; effect1<=effect0;
            visible1[0]<=relative_x<boundary_x;
            visible1[1]<=relative_y<boundary_y;
            visible1[2]<=relative_x>=curtain_left && relative_x<curtain_right;
            visible1[3]<={1'b0,relative_y[4:0]}<blind_boundary;
            visible2<=!active1 || visible1[effect1];
            visible3<=visible2;
            show_pixel<=visible3;
        end
    end
endmodule
