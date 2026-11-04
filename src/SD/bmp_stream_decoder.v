`timescale 1ns/1ps

// Uncompressed RGB24 BMP byte stream -> pixels in FILE row order.
// start precedes byte 0. Input offsets must be contiguous; byte_valid may pause.
// Drain the file after an error: finished is issued only after file_done.
// config_valid precedes the first pixel even with one input byte every clock.
// The loader uses frame_top_down to normalize SDRAM rows to top-to-bottom.
module bmp_stream_decoder #(
    parameter integer MAX_WIDTH = 960,
    parameter integer MAX_HEIGHT = 540
) (
    input wire clk, rst_n, start,
    input wire [31:0] directory_size,
    input wire byte_valid,
    input wire [7:0] byte_data,
    input wire [31:0] byte_offset,
    input wire file_done,
    input wire [7:0] file_error,
    output reg busy, finished, frame_commit,
    output reg [7:0] error_code,
    output reg config_valid,
    output reg [15:0] frame_width, frame_height,
    output reg [31:0] frame_pixels,
    output reg frame_top_down,
    output reg pixel_valid,
    output reg [23:0] pixel_rgb
);
    localparam [7:0] ERR_NONE=0, ERR_HEADER=1, ERR_TRUNCATED=2,
                     ERR_STREAM=3, ERR_FILE=4;
    reg [7:0] magic0, magic1;
    reg [31:0] declared_size, pixel_offset, dib_size, width, signed_height, compression;
    reg [15:0] planes, bit_count;
    reg [31:0] directory_size_q;
    wire [31:0] abs_height = signed_height[31] ? (~signed_height + 1'b1) : signed_height;
    wire dimensions_ok = width != 0 && width <= MAX_WIDTH &&
                         abs_height != 0 && abs_height <= MAX_HEIGHT;
    // Derive arithmetic widths from the supported source geometry, rather
    // than truncating large BMP dimensions to the old cache's ten bits.
    localparam integer WIDTH_BITS = $clog2(MAX_WIDTH+1);
    localparam integer HEIGHT_BITS = $clog2(MAX_HEIGHT+1);
    localparam integer ROW_BITS = $clog2(MAX_WIDTH*3+4);
    localparam integer PAYLOAD_BITS = $clog2(((MAX_WIDTH*3+3)/4)*4*MAX_HEIGHT+1);
    localparam integer PIXEL_BITS = $clog2(MAX_WIDTH*MAX_HEIGHT+1);
    wire [ROW_BITS-1:0] row_bytes = width[WIDTH_BITS-1:0] * 3;
    reg [ROW_BITS-1:0] row_bytes_q, row_stride_q;
    reg [PAYLOAD_BITS-1:0] payload_bytes_q;
    reg [PIXEL_BITS-1:0] pixel_count_q;
    reg [32:0] pixel_end_q;
    reg header_ok_q, header_accepted, finish_pending;
    reg [32:0] next_offset;
    reg [31:0] pixels_emitted;
    reg [ROW_BITS-1:0] row_offset;
    reg [1:0] rgb_lane;
    reg [7:0] blue, green;

    // Relevant header fields end at byte 33. Pipeline arithmetic while bytes
    // 34..53 arrive; no full-width multiplication/comparison in the pixel path.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_bytes_q <= 0; row_stride_q <= 0; payload_bytes_q <= 0;
            pixel_count_q <= 0; pixel_end_q <= 0; header_ok_q <= 0;
        end else if (start) begin
            row_bytes_q <= 0; row_stride_q <= 0; payload_bytes_q <= 0;
            pixel_count_q <= 0; pixel_end_q <= 0; header_ok_q <= 0;
        end else begin
            row_bytes_q <= row_bytes;
            row_stride_q <= (row_bytes + 3) & ~3;
            payload_bytes_q <= row_stride_q * abs_height[HEIGHT_BITS-1:0];
            pixel_count_q <= width[WIDTH_BITS-1:0] * abs_height[HEIGHT_BITS-1:0];
            pixel_end_q <= {1'b0,pixel_offset} + {{(33-PAYLOAD_BITS){1'b0}},payload_bytes_q};
            header_ok_q <= magic0 == "B" && magic1 == "M" && dimensions_ok &&
                dib_size >= 40 && planes == 1 && bit_count == 24 && compression == 0 &&
                pixel_offset >= 54 && {1'b0,pixel_offset} >= 33'd14+{1'b0,dib_size} &&
                {1'b0,declared_size} >= pixel_end_q && declared_size <= directory_size_q;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            magic0 <= 0; magic1 <= 0; declared_size <= 0; pixel_offset <= 0;
            dib_size <= 0; width <= 0; signed_height <= 0; compression <= 0;
            planes <= 0; bit_count <= 0; directory_size_q <= 0;
            busy <= 0; finished <= 0; frame_commit <= 0; error_code <= ERR_NONE;
            config_valid <= 0; frame_width <= 0; frame_height <= 0; frame_pixels <= 0;
            frame_top_down <= 0; pixel_valid <= 0; pixel_rgb <= 0;
            header_accepted <= 0; finish_pending <= 0; next_offset <= 0;
            pixels_emitted <= 0; row_offset <= 0; rgb_lane <= 0; blue <= 0; green <= 0;
        end else begin
            config_valid <= 0; pixel_valid <= 0; finished <= 0; frame_commit <= 0;
            if (start) begin
                magic0 <= 0; magic1 <= 0; declared_size <= 0; pixel_offset <= 0;
                dib_size <= 0; width <= 0; signed_height <= 0; compression <= 0;
                planes <= 0; bit_count <= 0; directory_size_q <= directory_size;
                busy <= 1; error_code <= ERR_NONE; header_accepted <= 0;
                finish_pending <= 0; next_offset <= 0; pixels_emitted <= 0;
                row_offset <= 0; rgb_lane <= 0; blue <= 0; green <= 0;
                frame_width <= 0; frame_height <= 0; frame_pixels <= 0; frame_top_down <= 0;
            end else if (busy) begin
                if (byte_valid && !finish_pending) begin
                    next_offset <= next_offset + 1'b1;
                    if ({1'b0,byte_offset} != next_offset && error_code == ERR_NONE)
                        error_code <= ERR_STREAM;
                    if (error_code == ERR_NONE && {1'b0,byte_offset} == next_offset) begin
                        case (byte_offset)
                            0: magic0 <= byte_data;
                            1: magic1 <= byte_data;
                            2,3,4,5: declared_size[(byte_offset-2)*8 +: 8] <= byte_data;
                            10,11,12,13: pixel_offset[(byte_offset-10)*8 +: 8] <= byte_data;
                            14,15,16,17: dib_size[(byte_offset-14)*8 +: 8] <= byte_data;
                            18,19,20,21: width[(byte_offset-18)*8 +: 8] <= byte_data;
                            22,23,24,25: signed_height[(byte_offset-22)*8 +: 8] <= byte_data;
                            26,27: planes[(byte_offset-26)*8 +: 8] <= byte_data;
                            28,29: bit_count[(byte_offset-28)*8 +: 8] <= byte_data;
                            30,31,32,33: compression[(byte_offset-30)*8 +: 8] <= byte_data;
                            53: begin
                                if (!header_ok_q) error_code <= ERR_HEADER;
                                else begin
                                    header_accepted <= 1;
                                    config_valid <= 1;
                                    frame_width <= width[15:0];
                                    frame_height <= abs_height[15:0];
                                    frame_pixels <= {{(32-PIXEL_BITS){1'b0}},pixel_count_q};
                                    frame_top_down <= signed_height[31];
                                end
                            end
                        endcase
                        if (header_accepted && byte_offset >= pixel_offset &&
                            {1'b0,byte_offset} < pixel_end_q) begin
                            row_offset <= row_offset == row_stride_q-1'b1 ? {ROW_BITS{1'b0}} : row_offset+1'b1;
                            if (row_offset < row_bytes_q) begin
                                case (rgb_lane)
                                    0: begin blue <= byte_data; rgb_lane <= 1; end
                                    1: begin green <= byte_data; rgb_lane <= 2; end
                                    2: begin
                                        pixel_rgb <= {byte_data,green,blue};
                                        pixel_valid <= 1; rgb_lane <= 0;
                                        pixels_emitted <= pixels_emitted+1'b1;
                                    end
                                    default: rgb_lane <= 0;
                                endcase
                            end
                        end
                    end
                end
                // Wait one cycle so a simultaneous last byte and EOF is counted.
                if (file_done) begin
                    finish_pending <= 1;
                    if (file_error != 0) error_code <= ERR_FILE;
                end
                if (finish_pending) begin
                    finish_pending <= 0; busy <= 0; finished <= 1;
                    if (error_code == ERR_NONE) begin
                        if (!header_accepted || pixels_emitted != frame_pixels ||
                            rgb_lane != 0 || row_offset != 0 || next_offset < pixel_end_q)
                            error_code <= ERR_TRUNCATED;
                        else frame_commit <= 1;
                    end
                end
            end
        end
    end
endmodule
