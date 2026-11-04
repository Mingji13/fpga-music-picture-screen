`timescale 1ns/1ps

// Dual-clock PCM sample buffer between the FAT32 byte stream and HDMI audio.
// The writer admits one complete SD data sector only when 256 samples fit.
module pcm_async_ring_buffer #(
    parameter integer ADDR_WIDTH = 12,
    parameter integer SECTOR_SAMPLES = 256
) (
    input  wire              wr_clk,
    input  wire              wr_rst_n,
    input  wire              wr_byte_valid,
    input  wire [7:0]        wr_byte,
    input  wire              wr_file_done,
    output wire              wr_sector_ready,
    output reg               wr_overflow,
    output reg               wr_format_error,

    input  wire              rd_clk,
    input  wire              rd_rst_n,
    input  wire              rd_sample_tick,
    output reg               rd_sample_valid,
    output wire signed [15:0] rd_sample,
    output wire              rd_empty,
    output wire              rd_playback_done,
    output reg               rd_underflow
);
    localparam integer DEPTH = (1 << ADDR_WIDTH);
    localparam [ADDR_WIDTH:0] READY_USED_LIMIT = DEPTH - SECTOR_SAMPLES;

    reg [15:0] sample_mem [0:DEPTH-1];
    reg [15:0] rd_sample_data;
    reg rd_sample_silent;

    reg [ADDR_WIDTH:0] wr_bin;
    reg [ADDR_WIDTH:0] wr_gray;
    reg [ADDR_WIDTH:0] rd_bin;
    reg [ADDR_WIDTH:0] rd_gray;

    (* async_reg = "true" *) reg [ADDR_WIDTH:0] rd_gray_sync1;
    (* async_reg = "true" *) reg [ADDR_WIDTH:0] rd_gray_sync2;
    (* async_reg = "true" *) reg [ADDR_WIDTH:0] wr_gray_sync1;
    (* async_reg = "true" *) reg [ADDR_WIDTH:0] wr_gray_sync2;
    (* async_reg = "true" *) reg wr_done_sync1;
    (* async_reg = "true" *) reg wr_done_sync2;

    reg [2:0] done_pipeline;
    reg wr_done_latched;
    reg low_byte_pending;
    reg [7:0] low_byte;

    wire [ADDR_WIDTH:0] rd_bin_sync;
    wire [ADDR_WIDTH:0] used_samples;
    wire [ADDR_WIDTH:0] wr_gray_full_compare;
    wire wr_full;
    wire [ADDR_WIDTH:0] rd_gray_empty_compare;
    wire rd_is_empty;

    function [ADDR_WIDTH:0] bin_to_gray;
        input [ADDR_WIDTH:0] binary_value;
        begin
            bin_to_gray = (binary_value >> 1) ^ binary_value;
        end
    endfunction

    function [ADDR_WIDTH:0] gray_to_bin;
        input [ADDR_WIDTH:0] gray_value;
        integer bit_index;
        begin
            gray_to_bin[ADDR_WIDTH] = gray_value[ADDR_WIDTH];
            for (bit_index = ADDR_WIDTH-1; bit_index >= 0; bit_index = bit_index-1)
                gray_to_bin[bit_index] = gray_to_bin[bit_index+1] ^ gray_value[bit_index];
        end
    endfunction

    assign rd_bin_sync = gray_to_bin(rd_gray_sync2);
    assign used_samples = wr_bin - rd_bin_sync;
    assign wr_sector_ready = (used_samples <= READY_USED_LIMIT);

    assign wr_gray_full_compare = {
        ~rd_gray_sync2[ADDR_WIDTH:ADDR_WIDTH-1],
        rd_gray_sync2[ADDR_WIDTH-2:0]
    };
    assign wr_full = (wr_gray == wr_gray_full_compare);

    assign rd_gray_empty_compare = wr_gray_sync2;
    assign rd_is_empty = (rd_gray == rd_gray_empty_compare);
    assign rd_empty = rd_is_empty;
    assign rd_playback_done = (&done_pipeline) && rd_is_empty;
    assign rd_sample = rd_sample_silent ? 16'sd0 : rd_sample_data;

    // Keep the RAM ports free of reset and zero-insertion muxes so TD can
    // infer dual-clock block RAM. Pointers, validity and silence reset below;
    // the stored samples themselves do not need to be cleared.
    always @(posedge wr_clk) begin
        if (wr_rst_n && wr_byte_valid && low_byte_pending && !wr_full)
            sample_mem[wr_bin[ADDR_WIDTH-1:0]] <= {wr_byte, low_byte};
    end
    always @(posedge rd_clk) begin
        if (rd_rst_n && rd_sample_tick && !rd_is_empty)
            rd_sample_data <= sample_mem[rd_bin[ADDR_WIDTH-1:0]];
    end

    // Read pointer synchronization into the SD/FAT32 writer clock domain.
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            rd_gray_sync1 <= 0;
            rd_gray_sync2 <= 0;
        end else begin
            rd_gray_sync1 <= rd_gray;
            rd_gray_sync2 <= rd_gray_sync1;
        end
    end

    // Convert the incoming little-endian byte stream into signed PCM samples.
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_bin <= 0;
            wr_gray <= 0;
            low_byte_pending <= 1'b0;
            low_byte <= 8'd0;
            wr_done_latched <= 1'b0;
            wr_overflow <= 1'b0;
            wr_format_error <= 1'b0;
        end else begin
            if (wr_byte_valid) begin
                if (!low_byte_pending) begin
                    low_byte <= wr_byte;
                    low_byte_pending <= 1'b1;
                end else begin
                    low_byte_pending <= 1'b0;
                    if (wr_full) begin
                        wr_overflow <= 1'b1;
                    end else begin
                        wr_bin <= wr_bin + 1'b1;
                        wr_gray <= bin_to_gray(wr_bin + 1'b1);
                    end
                end
            end

            if (wr_file_done) begin
                wr_done_latched <= 1'b1;
                if (wr_byte_valid ? !low_byte_pending : low_byte_pending)
                    wr_format_error <= 1'b1;
            end
        end
    end

    // Write pointer and completion flag synchronization into the sample clock domain.
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            wr_gray_sync1 <= 0;
            wr_gray_sync2 <= 0;
            wr_done_sync1 <= 1'b0;
            wr_done_sync2 <= 1'b0;
            done_pipeline <= 3'b000;
        end else begin
            wr_gray_sync1 <= wr_gray;
            wr_gray_sync2 <= wr_gray_sync1;
            wr_done_sync1 <= wr_done_latched;
            wr_done_sync2 <= wr_done_sync1;
            done_pipeline <= {done_pipeline[1:0], wr_done_sync2};
        end
    end

    // A sample tick always produces a valid output. Empty reads become silence.
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_bin <= 0;
            rd_gray <= 0;
            rd_sample_valid <= 1'b0;
            rd_sample_silent <= 1'b1;
            rd_underflow <= 1'b0;
        end else begin
            rd_sample_valid <= 1'b0;
            if (rd_sample_tick) begin
                rd_sample_valid <= 1'b1;
                rd_sample_silent <= rd_is_empty;
                if (rd_is_empty) begin
                    rd_underflow <= 1'b1;
                end else begin
                    rd_bin <= rd_bin + 1'b1;
                    rd_gray <= bin_to_gray(rd_bin + 1'b1);
                end
            end
        end
    end
endmodule
