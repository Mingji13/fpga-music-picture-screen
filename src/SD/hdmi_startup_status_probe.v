`timescale 1ns/1ps

// Observation only. External video supplies the time reference; this circuit
// has no timer, timestamp bank or return path to playback/reset control.
// C I L R A H: image-ever, lock-now, retry-ever, audio-enable-now,
// lock-loss-after-image. All five digits are 0/1, not counters.
// Transmitter lock is NOT a monitor/display-ready acknowledgement.
module hdmi_startup_status_probe (
    input wire pix_clk, pix_rst_n,
    input wire image_pixel, hdmi_locked, audio_enabled,
    input wire sd_retry_active,
    input wire board_clk, board_rst_n,
    output wire [23:0] status
);
    reg image_seen_pix, lock_seen_pix, loss_seen_pix;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            image_seen_pix <= 0;
            lock_seen_pix <= 0;
            loss_seen_pix <= 0;
        end else begin
            if (image_pixel) image_seen_pix <= 1;
            if (hdmi_locked) lock_seen_pix <= 1;
            if (lock_seen_pix && (image_seen_pix || image_pixel) && !hdmi_locked)
                loss_seen_pix <= 1;
        end
    end

    // Independent single-bit levels; narrow pixel/lock-loss events are held
    // in their source domain. SD recovery lasts 100 ms in production.
    // Existing video/SD -> board max-delay bounds cover these crossings.
    (* async_reg = "true" *) reg [4:0] event_meta, event_sync;
    reg image_seen, retry_seen, loss_seen;
    always @(posedge board_clk or negedge board_rst_n) begin
        if (!board_rst_n) begin
            event_meta <= 0;
            event_sync <= 0;
            image_seen <= 0;
            retry_seen <= 0;
            loss_seen <= 0;
        end else begin
            event_meta <= {loss_seen_pix, audio_enabled, sd_retry_active,
                           hdmi_locked, image_seen_pix};
            event_sync <= event_meta;
            image_seen <= image_seen | event_sync[0];
            retry_seen <= retry_seen | event_sync[2];
            loss_seen <= loss_seen | event_sync[4];
        end
    end
    assign status = {4'hc, 3'b0, image_seen, 3'b0, event_sync[1],
                    3'b0, retry_seen, 3'b0, event_sync[3], 3'b0, loss_seen};
endmodule
