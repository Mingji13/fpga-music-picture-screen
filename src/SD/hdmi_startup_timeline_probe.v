`timescale 1ns/1ps

// Observation only. All event times share the 50 MHz board-clock timer, in
// decimal tenths of a second since board reset/configuration (not wall power).
// A pixel-domain sticky bit catches even a one-pixel image pulse. A per-frame
// toggle crosses SOF without assuming that a 74 MHz pulse survives at 50 MHz.
// SD recovery does NOT reset this recorder. It must retain the cause/history.
// O_video_locked is transmitter status, never a monitor-ready acknowledgement.
module hdmi_startup_timeline_probe #(
    parameter integer BOARD_CLOCK_HZ = 50000000,
    parameter integer GAP_TICKS = 5 // about 40..50 ms without an input SOF
) (
    input wire pix_clk, pix_rst_n,
    input wire image_pixel, input_frame_start,
    input wire hdmi_locked, audio_enabled,
    input wire sd_retry_active,
    input wire board_clk, board_rst_n,
    output wire live_phase,
    output wire [23:0] live_status,
    output wire [23:0] image_status, first_lock_status, last_lock_status,
    output wire [23:0] audio_status, retry_status, gap_status
);
    reg image_seen_pix, frame_toggle_pix;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin image_seen_pix <= 0; frame_toggle_pix <= 0; end
        else begin
            if (image_pixel) image_seen_pix <= 1;
            if (input_frame_start) frame_toggle_pix <= ~frame_toggle_pix;
        end
    end

    // Each bit is an independent level/toggle, not a multi-bit data bus.
    // Existing video->board and SD->board max-delay bounds cover first stages;
    // first->second stages and all board-domain logic remain normally timed.
    (* async_reg = "true" *) reg [4:0] event_meta, event_sync;
    reg [4:0] event_old;
    wire [4:0] events_async = {frame_toggle_pix, sd_retry_active,
                              audio_enabled, hdmi_locked, image_seen_pix};
    always @(posedge board_clk or negedge board_rst_n) begin
        if (!board_rst_n) begin event_meta <= 0; event_sync <= 0; event_old <= 0; end
        else begin
            event_meta <= events_async;
            event_sync <= event_meta;
            event_old <= event_sync;
        end
    end

    localparam integer TICK_CYCLES = BOARD_CLOCK_HZ / 100; // 10 ms
    localparam integer TICK_BITS = TICK_CYCLES < 2 ? 1 : $clog2(TICK_CYCLES);
    localparam integer GAP_BITS = GAP_TICKS < 2 ? 1 : $clog2(GAP_TICKS+1);
    reg [TICK_BITS-1:0] tick_count;
    reg [3:0] tenth_divider;
    reg [15:0] time_bcd;
    wire tick_10ms = tick_count == TICK_CYCLES-1;
    always @(posedge board_clk or negedge board_rst_n) begin
        if (!board_rst_n) begin tick_count <= 0; tenth_divider <= 0; time_bcd <= 0; end
        else if (tick_10ms) begin
            tick_count <= 0;
            if (tenth_divider == 9) begin
                tenth_divider <= 0;
                if (time_bcd != 16'h9999) begin
                    if (time_bcd[3:0] != 9) time_bcd[3:0] <= time_bcd[3:0]+1'b1;
                    else begin
                        time_bcd[3:0] <= 0;
                        if (time_bcd[7:4] != 9) time_bcd[7:4] <= time_bcd[7:4]+1'b1;
                        else begin
                            time_bcd[7:4] <= 0;
                            if (time_bcd[11:8] != 9) time_bcd[11:8] <= time_bcd[11:8]+1'b1;
                            else begin time_bcd[11:8] <= 0; time_bcd[15:12] <= time_bcd[15:12]+1'b1; end
                        end
                    end
                end
            end else tenth_divider <= tenth_divider+1'b1;
        end else tick_count <= tick_count+1'b1;
    end

    reg image_seen, lock_seen, audio_seen;
    reg [15:0] image_time, first_lock_time, last_lock_time;
    reg [15:0] audio_time, retry_time, gap_time;
    reg [3:0] lock_losses, image_lock_losses, retries, gaps;
    reg frame_seen, gap_active;
    reg [GAP_BITS-1:0] gap_ticks;
    wire lock_rise = event_sync[1] && !event_old[1];
    wire lock_fall = !event_sync[1] && event_old[1];
    wire retry_rise = event_sync[3] && !event_old[3];
    wire frame_arrived = event_sync[4] != event_old[4];
    always @(posedge board_clk or negedge board_rst_n) begin
        if (!board_rst_n) begin
            image_seen <= 0; lock_seen <= 0; audio_seen <= 0;
            image_time <= 16'hffff; first_lock_time <= 16'hffff; last_lock_time <= 16'hffff;
            audio_time <= 16'hffff; retry_time <= 16'hffff; gap_time <= 16'hffff;
            lock_losses <= 0; image_lock_losses <= 0; retries <= 0; gaps <= 0;
            frame_seen <= 0; gap_active <= 0; gap_ticks <= 0;
        end else begin
            if (event_sync[0] && !image_seen) begin image_seen <= 1; image_time <= time_bcd; end
            if (event_sync[2] && !audio_seen) begin audio_seen <= 1; audio_time <= time_bcd; end
            if (lock_rise) begin
                lock_seen <= 1; last_lock_time <= time_bcd;
                if (!lock_seen) first_lock_time <= time_bcd;
            end
            if (lock_fall && lock_seen) begin
                if (lock_losses != 15) lock_losses <= lock_losses+1'b1;
                if (image_seen && image_lock_losses != 15) image_lock_losses <= image_lock_losses+1'b1;
            end
            if (retry_rise) begin
                retry_time <= time_bcd;
                if (retries != 15) retries <= retries+1'b1;
            end
            if (frame_arrived) begin
                frame_seen <= 1; gap_ticks <= 0; gap_active <= 0;
            end else if (tick_10ms && frame_seen && !gap_active) begin
                if (gap_ticks == GAP_TICKS-1) begin
                    gap_active <= 1; gap_time <= time_bcd;
                    if (gaps != 15) gaps <= gaps+1'b1;
                end else gap_ticks <= gap_ticks+1'b1;
            end
        end
    end

    // First 15 seconds: 0 S dddd, where S={retry,audio,image-seen,lock}.
    // History: page / count / dddd; dddd is decimal tenths, FFFF means unseen.
    // Counts are hexadecimal and saturate at F. Page 3 counts lock losses
    // AFTER the first image only; page 2 counts all losses since board reset.
    assign live_phase = time_bcd < 16'h0150;
    assign live_status = {4'h0, event_sync[3], event_sync[2], image_seen, event_sync[1], time_bcd};
    assign image_status = {4'h1, 4'h0, image_time};
    assign first_lock_status = {4'h2, lock_losses, first_lock_time};
    assign last_lock_status = {4'h3, image_lock_losses, last_lock_time};
    assign audio_status = {4'h4, 4'h0, audio_time};
    assign retry_status = {4'h5, retries, retry_time};
    assign gap_status = {4'h6, gaps, gap_time};
endmodule
