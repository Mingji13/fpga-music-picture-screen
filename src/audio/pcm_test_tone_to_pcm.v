`timescale 1ns/1ps

// Width adapter for the current HDMI transmitter's 24-bit PCM input.
// The test-tone source owns a 16-bit signed sample; HDMI IEC 60958 words
// carry that sample left-justified in 24 bits: {sample[15:0], 8'h00}.
// The valid/ready contract is passed through without adding a buffer.
module pcm_test_tone_to_pcm #(
    parameter integer SAMPLE_DIV = 521,
    parameter integer NOTE_HOLD_SAMPLES = 24000
) (
    input  wire                clk,
    input  wire                rst_n,
    input  wire                enable,
    output wire                pcm_valid,
    input  wire                pcm_ready,
    output wire signed [23:0]  pcm_left,
    output wire signed [23:0]  pcm_right,
    output wire                pcm_accepted,
    output wire [3:0]          rom_index
);
    wire               source_valid;
    wire               source_ready;
    wire signed [15:0] source_left;
    wire signed [15:0] source_right;
    wire               source_accepted;

    pcm_test_tone #(
        .SAMPLE_DIV (SAMPLE_DIV),
        .NOTE_HOLD_SAMPLES (NOTE_HOLD_SAMPLES)
    ) u_pcm_test_tone (
        .clk             (clk),
        .rst_n           (rst_n),
        .enable          (enable),
        .sample_valid    (source_valid),
        .sample_ready    (source_ready),
        .sample_left     (source_left),
        .sample_right    (source_right),
        .rom_index       (rom_index),
        .sample_accepted (source_accepted)
    );

    assign source_ready = pcm_ready;
    assign pcm_valid = source_valid;
    assign pcm_left  = {source_left, 8'h00};
    assign pcm_right = {source_right, 8'h00};
    assign pcm_accepted = source_accepted;
endmodule
