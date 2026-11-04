`timescale 1ns/1ps

// Board audio-mode control, in the video/HDMI PCM clock domain.
// 00 = ascending-scale test; 01 = picture-linked melody; 10 = PCM file; 11 = silence.
// Both generators are continuously drained by the Top, including when hidden.
module pcm_audio_mode_select #(
    parameter integer MODE_STABLE_CYCLES = 500000, // 20 ms at 25 MHz
    parameter integer MIN_SAMPLE_CYCLES = 520,
    parameter [1:0] RESET_MODE = 2'b00
) (
    input wire clk,
    input wire rst_n,
    input wire playback_enable,
    input wire [1:0] mode_async,
    input wire test_valid,
    input wire [23:0] test_left,
    input wire [23:0] test_right,
    input wire media_valid,
    input wire [23:0] media_left,
    input wire [23:0] media_right,
    input wire wav_valid,
    input wire [23:0] wav_left,
    input wire [23:0] wav_right,
    // Ask for one file sample; wav_valid/data return one clock later.
    // An accepted request completes even if the mode changes meanwhile.
    output wire wav_sample_request,
    output reg [1:0] mode_active,
    output reg pcm_valid,
    output reg [23:0] pcm_left,
    output reg [23:0] pcm_right
);
    localparam integer STABLE_BITS = (MODE_STABLE_CYCLES < 2) ? 1 :
                                     $clog2(MODE_STABLE_CYCLES);
    localparam integer GAP_BITS = (MIN_SAMPLE_CYCLES < 2) ? 1 :
                                  $clog2(MIN_SAMPLE_CYCLES);
    (* async_reg = "true" *) reg [1:0] mode_meta;
    (* async_reg = "true" *) reg [1:0] mode_sync;
    reg [1:0] mode_candidate;
    reg [STABLE_BITS-1:0] stable_count;
    reg [GAP_BITS-1:0] sample_gap;
    // Before first-picture output, use the continuously running media
    // sample cadence for silent PCM/ACR while the test DDS stays at C4.
    wire selected_valid = (playback_enable && mode_active == 2'b00) ?
                          test_valid :
                          ((playback_enable && mode_active == 2'b10) ?
                           1'b0 : media_valid);
    // Account for the one-clock synchronous RAM read before committing PCM.
    // The other sources may discard a sample during switching; file data may not.
    localparam integer WAV_REQUEST_GAP = (MIN_SAMPLE_CYCLES > 1) ?
                                         MIN_SAMPLE_CYCLES - 2 : 0;
    assign wav_sample_request = playback_enable && mode_active == 2'b10 &&
                                media_valid && sample_gap >= WAV_REQUEST_GAP;

    // Only the first stage is exempted from input timing in the SDC.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode_meta <= RESET_MODE;
            mode_sync <= RESET_MODE;
        end else begin
            mode_meta <= mode_async;
            mode_sync <= mode_meta;
        end
    end

    // Qualify the complete two-bit mode, so contact bounce does not keep
    // switching sources. A new candidate restarts the stability interval.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode_candidate <= RESET_MODE;
            mode_active <= RESET_MODE;
            stable_count <= 0;
        end else if (mode_sync != mode_candidate) begin
            mode_candidate <= mode_sync;
            stable_count <= 0;
        end else if (mode_active == mode_candidate) begin
            stable_count <= 0;
        end else if (stable_count == MODE_STABLE_CYCLES - 1) begin
            mode_active <= mode_candidate;
            stable_count <= 0;
        end else begin
            stable_count <= stable_count + 1'b1;
        end
    end

    // Register data and valid together: changing a switch cannot cut a PCM
    // pulse in half. A source change may discard one too-early sample; the
    // next one is used, preventing back-to-back pulses from different phases.
    // This is not a fade or a promise of pop-free analog output.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pcm_valid <= 1'b0;
            pcm_left <= 24'd0;
            pcm_right <= 24'd0;
            sample_gap <= MIN_SAMPLE_CYCLES - 1;
        end else begin
            pcm_valid <= 1'b0;
            if (sample_gap != MIN_SAMPLE_CYCLES - 1)
                sample_gap <= sample_gap + 1'b1;
            if (wav_valid) begin
                sample_gap <= 0;
                pcm_valid <= 1'b1;
                pcm_left <= wav_left;
                pcm_right <= wav_right;
            end else if (selected_valid && sample_gap == MIN_SAMPLE_CYCLES - 1) begin
                sample_gap <= 0;
                pcm_valid <= 1'b1;
                if (!playback_enable) begin
                    pcm_left <= 24'd0;
                    pcm_right <= 24'd0;
                end else case (mode_active)
                    2'b00: begin
                        pcm_left <= test_left;
                        pcm_right <= test_right;
                    end
                    2'b01: begin
                        pcm_left <= media_left;
                        pcm_right <= media_right;
                    end
                    default: begin
                        // Keep 48 kHz PCM/ACR running in reserved modes.
                        pcm_left <= 24'd0;
                        pcm_right <= 24'd0;
                    end
                endcase
            end
        end
    end
endmodule
