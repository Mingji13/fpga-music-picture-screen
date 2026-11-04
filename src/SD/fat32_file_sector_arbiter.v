`timescale 1ns/1ps

// Shares one sector adapter between two FAT32 file streamers. A grant owns
// the adapter until sector_done, so response bytes can never cross streams.
module fat32_file_sector_arbiter (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire        bmp_req,
    input  wire [31:0] bmp_lba,
    output wire        bmp_accept,
    input  wire        pcm_req,
    input  wire [31:0] pcm_lba,
    output wire        pcm_accept,
    output wire        sector_req,
    output wire [31:0] sector_lba,
    input  wire        sector_busy,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error,
    output wire        bmp_valid,
    output wire [7:0]  bmp_byte,
    output wire [8:0]  bmp_byte_index,
    output wire        bmp_done,
    output wire [7:0]  bmp_error,
    output wire        pcm_valid,
    output wire [7:0]  pcm_byte,
    output wire [8:0]  pcm_byte_index,
    output wire        pcm_done,
    output wire [7:0]  pcm_error
);
    reg owner_active;
    reg owner_pcm;
    reg prefer_pcm;
    wire grant_bmp;
    wire grant_pcm;

    assign grant_bmp = enable && !owner_active && !sector_busy && bmp_req &&
                       (!pcm_req || !prefer_pcm);
    assign grant_pcm = enable && !owner_active && !sector_busy && pcm_req &&
                       (!bmp_req || prefer_pcm);
    assign bmp_accept = grant_bmp;
    assign pcm_accept = grant_pcm;
    assign sector_req = grant_bmp || grant_pcm;
    assign sector_lba = grant_pcm ? pcm_lba : bmp_lba;

    assign bmp_valid = owner_active && !owner_pcm && sector_valid;
    assign bmp_byte = bmp_valid ? sector_byte : 8'd0;
    assign bmp_byte_index = bmp_valid ? sector_byte_index : 9'd0;
    assign bmp_done = owner_active && !owner_pcm && sector_done;
    assign bmp_error = bmp_done ? sector_error : 8'd0;

    assign pcm_valid = owner_active && owner_pcm && sector_valid;
    assign pcm_byte = pcm_valid ? sector_byte : 8'd0;
    assign pcm_byte_index = pcm_valid ? sector_byte_index : 9'd0;
    assign pcm_done = owner_active && owner_pcm && sector_done;
    assign pcm_error = pcm_done ? sector_error : 8'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            owner_active <= 1'b0;
            owner_pcm <= 1'b0;
            prefer_pcm <= 1'b0;
        end else if (owner_active) begin
            if (sector_done)
                owner_active <= 1'b0;
        end else if (grant_bmp || grant_pcm) begin
            owner_active <= 1'b1;
            owner_pcm <= grant_pcm;
            prefer_pcm <= grant_bmp;
        end
    end
endmodule
