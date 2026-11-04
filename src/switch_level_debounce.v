`timescale 1ns/1ps

// Synchronize a maintained switch, then accept a new level only after it
// remains different from the current level for STABLE_CYCLES samples.
module switch_level_debounce #(
    parameter integer STABLE_CYCLES = 1480000
) (
    input  wire clk,
    input  wire rst_n,
    input  wire level_async,
    output reg  level_stable
);
    localparam integer COUNT_BITS = (STABLE_CYCLES < 2) ? 1 :
                                    $clog2(STABLE_CYCLES);
    (* async_reg = "true" *) reg level_meta;
    (* async_reg = "true" *) reg level_sync;
    reg [COUNT_BITS-1:0] stable_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            level_meta <= 1'b0;
            level_sync <= 1'b0;
        end else begin
            level_meta <= level_async;
            level_sync <= level_meta;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            level_stable <= 1'b0;
            stable_count <= 0;
        end else if (level_sync == level_stable) begin
            stable_count <= 0;
        end else if (stable_count == STABLE_CYCLES - 1) begin
            level_stable <= level_sync;
            stable_count <= 0;
        end else begin
            stable_count <= stable_count + 1'b1;
        end
    end
endmodule
