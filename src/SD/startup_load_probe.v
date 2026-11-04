`timescale 1ns/1ps

// Observation only: collect startup state in the SD clock domain and transfer
// a coherent snapshot through the existing diagnostic mailbox implementation.
module startup_load_probe #(
    parameter integer WINDOW_BITS = 21
) (
    input wire sd_clk,
    input wire sd_rst,
    input wire sd_init_done,
    input wire scan_kicked,
    input wire scan_done,
    input wire display_valid,
    input wire load_start,
    input wire pixel_valid,
    input wire frame_commit,
    input wire [3:0] image_complete,
    input wire [7:0] decode_error,
    input wire [7:0] writer_error,
    input wire board_clk,
    input wire board_rst_n,
    output wire [23:0] status
);
    reg [WINDOW_BITS-1:0] count;
    reg [3:0] progress_seen;
    reg [3:0] first_decode_error;
    reg [7:0] first_writer_error;
    wire [3:0] progress_events = {load_start, pixel_valid, frame_commit, |image_complete};
    wire [3:0] stage = !sd_init_done ? 4'h5 :
                        !scan_kicked ? 4'h6 :
                        !scan_done ? 4'h7 :
                        !display_valid ? 4'h8 : 4'h9;
    // A S P D WW: page tag, stage, progress bits, first BMP error, first writer error.
    // Progress is sticky across individual image retries, until SD reset:
    // bit3 load-start seen; bit2 pixel seen; bit1 frame commit seen; bit0 any bank complete.
    wire [23:0] sample = {4'ha, stage, progress_seen | progress_events,
                         first_decode_error, first_writer_error};

    always @(posedge sd_clk or posedge sd_rst) begin
        if (sd_rst) begin
            count <= 0;
            progress_seen <= 4'd0;
            first_decode_error <= 4'd0;
            first_writer_error <= 8'd0;
        end else begin
            count <= count + 1'b1;
            progress_seen <= progress_seen | progress_events;
            if (first_decode_error == 0 && decode_error != 0)
                first_decode_error <= decode_error[3:0];
            if (first_writer_error == 0 && writer_error != 0)
                first_writer_error <= writer_error;
        end
    end

    probe_snapshot_mailbox #(.WIDTH(24)) u_mailbox (
        .src_clk(sd_clk), .src_rst_n(!sd_rst), .capture(&count), .sample(sample),
        .dst_clk(board_clk), .dst_rst_n(board_rst_n), .status(status)
    );
endmodule
