`timescale 1ns / 1ps

module video_timing_ctrl #(
    parameter integer H_ACT = 640,
    parameter integer H_FP = 16,
    parameter integer H_SYNC = 96,
    parameter integer H_BP = 48,
    parameter integer V_ACT = 480,
    parameter integer V_FP = 10,
    parameter integer V_SYNC = 2,
    parameter integer V_BP = 33,
    parameter integer ACTIVE_LAST = 0
) (
    input  wire       pclk,
    input  wire       rst_n,
    output wire       hsync,
    output wire       vsync,
    output wire       de,
    output reg [11:0] x,
    output reg [11:0] y
);

    localparam integer H_TOTAL = H_ACT + H_FP + H_SYNC + H_BP;
    localparam integer V_TOTAL = V_ACT + V_FP + V_SYNC + V_BP;

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            x <= 12'd0;
            y <= 12'd0;
        end else if (x == H_TOTAL - 1) begin
            x <= 12'd0;
            y <= (y == V_TOTAL - 1) ? 12'd0 : y + 12'd1;
        end else begin
            x <= x + 12'd1;
        end
    end

    assign de = ACTIVE_LAST ?
                ((x >= H_TOTAL - H_ACT) && (y >= V_TOTAL - V_ACT)) :
                ((x < H_ACT) && (y < V_ACT));

    assign hsync = ACTIVE_LAST ?
                   ~((x >= H_FP) && (x < H_FP + H_SYNC)) :
                   ~((x >= H_ACT + H_FP) &&
                     (x < H_ACT + H_FP + H_SYNC));

    assign vsync = ACTIVE_LAST ?
                   ~((y >= V_FP) && (y < V_FP + V_SYNC)) :
                   ~((y >= V_ACT + V_FP) &&
                     (y < V_ACT + V_FP + V_SYNC));

endmodule

