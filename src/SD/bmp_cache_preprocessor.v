`timescale 1ns/1ps

// RGB24 BMP <=1920x1080 -> cache geometry <=960x540.
// Small images are unchanged. Larger images use a separable 2x2 box average
// (sum all four channels, round once), then the display scaler enlarges them.
// Only one row of horizontal sums is stored; there is no full-frame buffer.
// Pixels retain FILE row order. For odd bottom-up heights the single edge row
// comes first, so top-down and bottom-up files produce identical cache pixels.
module bmp_cache_preprocessor (
    input wire clk, rst_n, start,
    input wire [31:0] directory_size,
    input wire byte_valid,
    input wire [7:0] byte_data,
    input wire [31:0] byte_offset,
    input wire file_done,
    input wire [7:0] file_error,
    output reg busy, finished, frame_commit,
    output wire [7:0] error_code,
    output reg config_valid,
    output reg [15:0] frame_width, frame_height,
    output reg [31:0] frame_pixels,
    output reg frame_top_down,
    output reg pixel_valid,
    output reg [23:0] pixel_rgb,
    // Original positive BMP dimensions, stable when config_valid is asserted.
    output wire [15:0] source_width, source_height
);
    wire raw_busy, raw_finished, raw_commit, raw_config, raw_pixel;
    wire [15:0] raw_width, raw_height;
    assign source_width = raw_width;
    assign source_height = raw_height;
    wire [31:0] raw_pixels;
    wire raw_top_down;
    wire [23:0] raw_rgb;
    bmp_stream_decoder #(.MAX_WIDTH(1920),.MAX_HEIGHT(1080)) u_decoder (
        .clk(clk),.rst_n(rst_n),.start(start),.directory_size(directory_size),
        .byte_valid(byte_valid),.byte_data(byte_data),.byte_offset(byte_offset),
        .file_done(file_done),.file_error(file_error),.busy(raw_busy),
        .finished(raw_finished),.frame_commit(raw_commit),.error_code(error_code),
        .config_valid(raw_config),.frame_width(raw_width),.frame_height(raw_height),
        .frame_pixels(raw_pixels),.frame_top_down(raw_top_down),
        .pixel_valid(raw_pixel),.pixel_rgb(raw_rgb)
    );
    wire needs_half = raw_width>960 || raw_height>540;
    wire [9:0] cache_width = needs_half ? (({1'b0,raw_width[10:0]}+12'd1)>>1) : raw_width[9:0];
    wire [9:0] cache_height = needs_half ? (({1'b0,raw_height[10:0]}+12'd1)>>1) : raw_height[9:0];
    reg half_size;
    reg [10:0] source_x,source_y;
    reg second_row;
    reg [23:0] previous_rgb;
    wire last_x = source_x==raw_width[10:0]-1'b1;
    wire single_row = raw_height[0] &&
        (raw_top_down ? source_y==raw_height[10:0]-1'b1 : source_y==0);
    wire [23:0] pair_left = source_x[0] ? previous_rgb : raw_rgb;
    reg hvalid,hemit,hsingle;
    reg [9:0] hcol;
    reg [26:0] hsum;
    // No reset on the RAM/read port: infer synchronous block RAM. Every read
    // used for output follows a complete write of the corresponding first row.
    (* ram_style = "block" *) reg [26:0] line_sums [0:959];
    reg [26:0] line_read;
    always @(posedge clk) begin
        line_read <= line_sums[hcol];
        if(rst_n && !start && hvalid && !hemit)
            line_sums[hcol] <= hsum;
    end
    reg vvalid,vsingle;
    reg [26:0] sum_q;
    wire [26:0] other_sum = vsingle ? sum_q : line_read;
    wire [9:0] red_sum = {1'b0,sum_q[26:18]}+{1'b0,other_sum[26:18]}+10'd2;
    wire [9:0] green_sum = {1'b0,sum_q[17:9]}+{1'b0,other_sum[17:9]}+10'd2;
    wire [9:0] blue_sum = {1'b0,sum_q[8:0]}+{1'b0,other_sum[8:0]}+10'd2;
    reg [3:0] finish_pipe,commit_pipe;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            busy<=0;finished<=0;frame_commit<=0;config_valid<=0;
            frame_width<=0;frame_height<=0;frame_pixels<=0;frame_top_down<=0;
            pixel_valid<=0;pixel_rgb<=0;half_size<=0;
            source_x<=0;source_y<=0;second_row<=0;previous_rgb<=0;
            hvalid<=0;hemit<=0;hsingle<=0;hcol<=0;hsum<=0;
            vvalid<=0;vsingle<=0;sum_q<=0;finish_pipe<=0;commit_pipe<=0;
        end else begin
            finished<=0;frame_commit<=0;config_valid<=0;pixel_valid<=0;
            hvalid<=0;vvalid<=hvalid && hemit;
            sum_q<=hsum;vsingle<=hsingle;
            finish_pipe<={finish_pipe[2:0],raw_finished};
            commit_pipe<={commit_pipe[2:0],raw_commit};
            if(start) begin
                busy<=1;half_size<=0;source_x<=0;source_y<=0;second_row<=0;
                hvalid<=0;vvalid<=0;finish_pipe<=0;commit_pipe<=0;
                frame_width<=0;frame_height<=0;frame_pixels<=0;
            end else begin
                if(raw_config) begin
                    half_size<=needs_half;config_valid<=1;
                    frame_width<={6'd0,cache_width};frame_height<={6'd0,cache_height};
                    frame_pixels<=cache_width*cache_height;frame_top_down<=raw_top_down;
                    source_x<=0;source_y<=0;second_row<=0;
                end
                if(raw_pixel && half_size) begin
                    previous_rgb<=raw_rgb;
                    if(source_x[0] || last_x) begin
                        hvalid<=1;hcol<=source_x[10:1];
                        hemit<=second_row || single_row;hsingle<=single_row;
                        hsum<={({1'b0,pair_left[23:16]}+{1'b0,raw_rgb[23:16]}),
                               ({1'b0,pair_left[15:8]}+{1'b0,raw_rgb[15:8]}),
                               ({1'b0,pair_left[7:0]}+{1'b0,raw_rgb[7:0]})};
                    end
                    if(last_x) begin
                        source_x<=0;source_y<=source_y+1'b1;
                        second_row<=single_row ? 1'b0 : !second_row;
                    end else source_x<=source_x+1'b1;
                end
                if(half_size ? vvalid : raw_pixel) begin
                    pixel_valid<=1;
                    pixel_rgb<=half_size ? {red_sum[9:2],green_sum[9:2],blue_sum[9:2]} : raw_rgb;
                end
                // EOF is delayed beyond the final filter/RAM pipeline output.
                // Failed files drain too, with the decoder's original error.
                if(finish_pipe[3]) begin
                    busy<=0;finished<=1;frame_commit<=commit_pipe[3];
                end
            end
        end
    end
endmodule
