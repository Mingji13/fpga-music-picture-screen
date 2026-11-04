`timescale 1ns/1ps

// SD-clock playlist control. One metadata lookup per song, one live streamer,
// one ring. buffer_hold resets BOTH ring domains but never the video/SD bus.
module pcm_playlist_controller #(
    parameter integer MAX_SONGS = 16,
    parameter integer ENABLE_CUES = 0,
    parameter integer ID_WIDTH = MAX_SONGS <= 1 ? 1 : $clog2(MAX_SONGS)
) (
    input wire clk, rst_n, enable,
    input wire [MAX_SONGS-1:0] song_mask,
    input wire repeat_all, next_track, previous_track,
    input wire playback_done,
    input wire stream_busy,
    input wire [7:0] stream_error,
    output wire stream_cancel,
    output reg stream_start,
    output reg [31:0] stream_cluster, stream_size,
    output wire buffer_hold,
    input wire buffer_ready, buffer_cleared,
    output wire query_enable,
    output reg [ID_WIDTH-1:0] query_id,
    input wire query_valid,
    input wire [31:0] query_cluster, query_size,
    output reg [ID_WIDTH-1:0] current_id,
    output wire cue_prepare,
    input wire cue_ready
);
    localparam [3:0] WAIT_MEDIA=0, SELECT=1, QUERY=2, RELEASE=3,
        START=4, PLAY=5, STOP=6, CLEAR=7, FAULT=8, PREPARE=9;
    reg [3:0] state;
    reg search_previous;
    function [ID_WIDTH-1:0] following;
        input [ID_WIDTH-1:0] id;
        begin following=(id==MAX_SONGS-1)?0:id+1'b1; end
    endfunction
    function [ID_WIDTH-1:0] preceding;
        input [ID_WIDTH-1:0] id;
        begin preceding=(id==0)?MAX_SONGS-1:id-1'b1; end
    endfunction
    assign query_enable=(state==QUERY);
    assign cue_prepare=ENABLE_CUES && state==PREPARE;
    assign stream_cancel=(state==STOP);
    assign buffer_hold=(state!=RELEASE && state!=START && state!=PLAY && state!=FAULT);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=WAIT_MEDIA;query_id<=0;current_id<=0;
            search_previous<=0;
            stream_start<=0;stream_cluster<=0;stream_size<=0;
        end else begin
            stream_start<=0;
            case (state)
                WAIT_MEDIA: if(enable && |song_mask) begin
                    query_id<=0;search_previous<=0;state<=SELECT;
                end
                // A serial search avoids a wide priority encoder/mux. Even
                // with 15 missing names this costs well below one microsecond.
                SELECT: if(song_mask[query_id]) state<=QUERY;
                        else query_id<=search_previous?preceding(query_id):following(query_id);
                QUERY: if(query_valid) begin
                    stream_cluster<=query_cluster;stream_size<=query_size;
                    current_id<=query_id;state<=ENABLE_CUES?PREPARE:RELEASE;
                end
                PREPARE:if(cue_ready) state<=RELEASE;
                // Both reset synchronizers must release, and the old done
                // level must cross back LOW before a new file can start.
                RELEASE: if(buffer_ready && !playback_done) state<=START;
                START: begin stream_start<=1;state<=PLAY;end
                PLAY: begin
                    if(next_track || previous_track) begin
                        search_previous<=!next_track;
                        query_id<=next_track?following(current_id):preceding(current_id);
                        state<=STOP;
                    end
                    else if(playback_done && !stream_busy && !stream_start) begin
                        if(stream_error!=0) state<=FAULT;
                        else begin
                            query_id<=repeat_all?following(current_id):current_id;
                            search_previous<=0;
                            state<=CLEAR;
                        end
                    end
                end
                STOP: if(!stream_busy && !stream_start) state<=CLEAR;
                CLEAR: if(buffer_cleared) state<=SELECT;
                // A corrupt file does not create a tight SD retry loop.
                // KEY4 still permits the user to choose another file.
                FAULT: if(next_track || previous_track) begin
                    search_previous<=!next_track;
                    query_id<=next_track?following(current_id):preceding(current_id);
                    state<=CLEAR;
                end
                default: state<=WAIT_MEDIA;
            endcase
        end
    end
endmodule
