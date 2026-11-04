`timescale 1ns/1ps

// Prepends optional CUE reading to the existing PCM streamer. The playlist
// holds the PCM FIFO reset until this finishes; BMP keeps using its own port.
module music_cue_loader (
    input wire clk, rst_n, prepare,
    input wire cue_present, query_valid,
    input wire [31:0] query_cluster, query_size,
    output wire query_enable,
    input wire stream_busy, stream_done,
    output reg load_start,
    output reg [31:0] load_cluster, load_size,
    output wire loading,
    output wire ready
);
    localparam IDLE=0, QUERY=1, CHECK=6, START=2, WAIT_DONE=3, DRAIN=4, READY=5;
    reg [2:0] state;
    reg [2:0] drain;
    reg size_high_invalid;
    assign query_enable=state==QUERY;
    assign loading=state==START || state==WAIT_DONE || state==DRAIN;
    assign ready=state==READY;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin state<=IDLE;drain<=0;load_start<=0;load_cluster<=0;load_size<=0;size_high_invalid<=0;end
        else begin
            load_start<=0;
            case(state)
                IDLE:if(prepare) state<=cue_present?QUERY:READY;
                QUERY:if(query_valid) begin
                    load_cluster<=query_cluster;
                    // Only sizes 68..4160 can reach load_start. Hold their
                    // 13-bit value plus one rejection flag for high bits.
                    load_size<={19'd0,query_size[12:0]};
                    size_high_invalid<=|query_size[31:13];
                    state<=CHECK;
                end
                // Check the held descriptor after the synchronous catalogue
                // lookup. Do not put query selection and range arithmetic on
                // the same 100 MHz path. Sizes above 8191 are rejected early.
                CHECK:begin
                    if(size_high_invalid || load_size[12:0]<13'd68 ||
                       load_size[12:0]>13'd4160 || load_size[1:0]!=0) state<=READY;
                    else state<=START;
                end
                START:if(!stream_busy) begin load_start<=1;state<=WAIT_DONE;end
                WAIT_DONE:if(stream_done) begin drain<=5;state<=DRAIN;end
                DRAIN:if(drain==0) state<=READY;else drain<=drain-1'b1;
                READY:if(!prepare) state<=IDLE;
                default:state<=IDLE;
            endcase
        end
    end
endmodule
