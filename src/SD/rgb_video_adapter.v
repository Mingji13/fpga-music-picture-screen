`timescale 1ns / 1ps

module rgb_video_adapter #(
    parameter integer HS_IN_ACTIVE_LOW = 1,
    parameter integer VS_IN_ACTIVE_LOW = 1,
    parameter integer HS_OUT_ACTIVE_LOW = 1,
    parameter integer VS_OUT_ACTIVE_LOW = 1
) (
    input  wire        pclk,
    input  wire        rst_n,
    input  wire        hs_in,
    input  wire        vs_in,
    input  wire        de_in,
    input  wire [23:0] rgb_in,
    output wire        video_clk,
    output reg         video_hs,
    output reg         video_vs,
    output reg         video_de,
    output reg [23:0]  video_rgb
);

    wire hs_active = HS_IN_ACTIVE_LOW ? ~hs_in : hs_in;
    wire vs_active = VS_IN_ACTIVE_LOW ? ~vs_in : vs_in;

    assign video_clk = pclk;

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            video_hs  <= HS_OUT_ACTIVE_LOW ? 1'b1 : 1'b0;
            video_vs  <= VS_OUT_ACTIVE_LOW ? 1'b1 : 1'b0;
            video_de  <= 1'b0;
            video_rgb <= 24'h000000;
        end else begin
            video_hs  <= HS_OUT_ACTIVE_LOW ? ~hs_active : hs_active;
            video_vs  <= VS_OUT_ACTIVE_LOW ? ~vs_active : vs_active;
            video_de  <= de_in;
            video_rgb <= de_in ? rgb_in : 24'h000000;
        end
    end

endmodule

