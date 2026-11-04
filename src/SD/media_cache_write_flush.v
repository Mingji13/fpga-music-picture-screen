`timescale 1ns/1ps

// Retire a failed frame's write path without resetting other cached frames.
// Used only after the file/decoder has stopped; src_writer_idle must mean that
// the loader is inactive AND its final registered command is no longer valid.
// Block new memory burst grants, let a previously granted burst finish, then
// reset both ends of the command CDC (including incomplete, unannounced blocks).
// Do not reset this helper with its own reset_request. The parent resets the
// loader/scheduler/helper together only for a whole-player restart.
module media_cache_write_flush #(
    parameter integer GUARD_CYCLES = 4
) (
    input wire src_clk, src_rst_n,
    input wire flush_request, src_writer_idle,
    output wire flush_done,
    input wire mem_clk, mem_rst_n,
    input wire mem_write_active, mem_write_en,
    input wire controller_busy, refresh_active,
    output wire mem_block_requests,
    output reg reset_request
);
    localparam integer CW = $clog2(GUARD_CYCLES+1);
    localparam [1:0] IDLE=0, DRAIN=1, RESET=2, RELEASE=3;
    reg [1:0] state;
    reg [CW-1:0] count;
    reg req_sync1, req_sync2;
    reg ack_mem, ack_sync1, ack_sync2;
    wire memory_quiet = !mem_write_active && !mem_write_en &&
                        !controller_busy && !refresh_active;
    assign flush_done = ack_sync2;
    assign mem_block_requests = req_sync2 || state != IDLE;

    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin ack_sync1<=0;ack_sync2<=0;end
        else begin ack_sync1<=ack_mem;ack_sync2<=ack_sync1;end
    end
    always @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin
            req_sync1<=0;req_sync2<=0;
            state<=IDLE;count<=0;ack_mem<=0;reset_request<=0;
        end else begin
            req_sync1 <= flush_request && src_writer_idle;
            req_sync2 <= req_sync1;
            case (state)
            IDLE: begin
                count<=0;
                if(req_sync2) state<=DRAIN;
            end
            DRAIN: begin
                if(!memory_quiet) count<=0;
                else if(count==GUARD_CYCLES-1) begin
                    reset_request<=1;count<=0;state<=RESET;
                end else count<=count+1'b1;
            end
            RESET: begin
                if(count<GUARD_CYCLES) count<=count+1'b1;
                else begin
                    ack_mem<=1;
                    if(!req_sync2) begin
                        reset_request<=0;count<=0;state<=RELEASE;
                    end
                end
            end
            RELEASE: begin
                // The CDC reset synchronizers release before done falls.
                if(count==GUARD_CYCLES-1) begin
                    ack_mem<=0;count<=0;state<=IDLE;
                end else count<=count+1'b1;
            end
            default: begin state<=IDLE;reset_request<=0;ack_mem<=0;count<=0;end
            endcase
        end
    end
endmodule
