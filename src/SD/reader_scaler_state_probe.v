`timescale 1ns/1ps

// Observation only. Each clock collects its own activity over an independent
// time window; it does NOT use the frame pulse being diagnosed as its timer.
// Cross-domain snapshots are coherent individually, not simultaneous across
// the pixel and memory domains. A blocked mailbox simply drops an observation.
module reader_scaler_state_probe #(
    parameter integer PIX_WINDOW_BITS = 21,
    parameter integer MEM_WINDOW_BITS = 22
) (
    input wire pix_clk, pix_rst_n,
    input wire frame_start, stream_start, source_take, image_pixel,
    input wire scaler_underflow, request_inflight, stream_ready, pixel_valid,
    input wire mem_clk, mem_rst_n,
    input wire [7:0] reader_error,
    input wire reload_pending, inflight_pending, reader_busy, fifo_reset,
    input wire read_command, read_return, return_nonblack,
    input wire init_done, controller_busy, refresh_active, burst_active, burst_grant,
    input wire board_clk, board_rst_n,
    output wire [23:0] status
);
    reg [PIX_WINDOW_BITS-1:0] pix_count;
    reg [MEM_WINDOW_BITS-1:0] mem_count;
    reg [5:0] pix_seen;
    reg [3:0] mem_seen;
    wire pix_capture = &pix_count;
    wire mem_capture = &mem_count;
    wire [5:0] pix_events = {frame_start, stream_start, source_take,
                             image_pixel, scaler_underflow, pixel_valid};
    wire [3:0] mem_events = {read_command, read_return,
                             read_return && return_nonblack, burst_grant};
    wire [5:0] pix_activity = pix_seen | pix_events;
    wire [3:0] mem_activity = mem_seen | mem_events;
    // Error nibble: 0=none, 1=61 frame overrun, 2=62 FIFO overflow,
    // 3=63 underflow, 4=64 short frame; F=some other/unknown code.
    wire [3:0] error_code = (reader_error == 8'h00) ? 4'h0 :
                            (reader_error[7:4] == 4'h6) ? reader_error[3:0] : 4'hf;
    wire [7:0] pix_data = {
        pix_activity[5:2],
        pix_activity[1], request_inflight, stream_ready, pix_activity[0]
    };
    wire [15:0] mem_data = {
        error_code,
        reload_pending, inflight_pending, reader_busy, fifo_reset,
        mem_activity[3:1], init_done,
        controller_busy, refresh_active, burst_active, mem_activity[0]
    };
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin pix_count <= 0; pix_seen <= 0; end
        else begin
            pix_count <= pix_count + 1'b1;
            pix_seen <= pix_capture ? 6'd0 : pix_activity;
        end
    end
    always @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin mem_count <= 0; mem_seen <= 0; end
        else begin
            mem_count <= mem_count + 1'b1;
            mem_seen <= mem_capture ? 4'd0 : mem_activity;
        end
    end
    probe_snapshot_mailbox #(.WIDTH(8)) u_pix_mailbox (
        .src_clk(pix_clk), .src_rst_n(pix_rst_n),
        .capture(pix_capture), .sample(pix_data),
        .dst_clk(board_clk), .dst_rst_n(board_rst_n), .status(status[23:16])
    );
    probe_snapshot_mailbox #(.WIDTH(16)) u_mem_mailbox (
        .src_clk(mem_clk), .src_rst_n(mem_rst_n),
        .capture(mem_capture), .sample(mem_data),
        .dst_clk(board_clk), .dst_rst_n(board_rst_n), .status(status[15:0])
    );
endmodule

module probe_snapshot_mailbox #(parameter integer WIDTH = 8) (
    input wire src_clk, src_rst_n, capture,
    input wire [WIDTH-1:0] sample,
    input wire dst_clk, dst_rst_n,
    output reg [WIDTH-1:0] status
);
    reg [WIDTH-1:0] held_data;
    reg request_toggle, ack_toggle;
    (* async_reg = "true" *) reg ack_meta, ack_sync;
    (* async_reg = "true" *) reg request_meta, request_sync;
    reg [WIDTH-1:0] data_meta, data_sync;
    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            held_data <= 0; request_toggle <= 0; ack_meta <= 0; ack_sync <= 0;
        end else begin
            ack_meta <= ack_toggle;
            ack_sync <= ack_meta;
            if (capture && (request_toggle == ack_sync)) begin
                held_data <= sample;
                request_toggle <= ~request_toggle;
            end
        end
    end
    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            data_meta <= 0; data_sync <= 0; request_meta <= 0;
            request_sync <= 0; ack_toggle <= 0; status <= 0;
        end else begin
            data_meta <= held_data;
            data_sync <= data_meta;
            request_meta <= request_toggle;
            request_sync <= request_meta;
            if (request_sync != ack_toggle) begin
                status <= data_sync;
                ack_toggle <= request_sync;
            end
        end
    end
endmodule
