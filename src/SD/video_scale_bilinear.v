`timescale 1ns/1ps

// Runtime-size RGB stream (1..960 x 1..540) -> aspect-fit 1280x720.
// The source is row-major, top-to-bottom, one accepted transfer per source pixel.
// Config is sampled one clock AFTER frame_start, allowing active-bank metadata
// to change on that boundary. A fresh source_frame_start must follow the reader
// FIFO flush. Input may stall; video timing never stalls. A missed row blanks
// the remainder of that frame and rearms at the next frame_start.
//
// Horizontal interpolation is done once per source row into three row RAMs;
// vertical interpolation reuses those rows. Adjacent samples are reused, so
// four independent SDRAM reads per output pixel are not required.
// RGB/HS/VS/DE/X/image-valid have the same FIVE-clock latency as video_scale_3_2.
// rst_n clears image/stream state. video_rst_n alone resets the timing/X
// pipeline, so an SD retry blanks pixels without interrupting the raster.
module video_scale_bilinear (
    input wire pclk, rst_n, video_rst_n, frame_start, frame_enable,
    input wire [15:0] source_width, source_height,
    input wire source_frame_start, source_valid,
    input wire [23:0] source_rgb,
    output wire source_ready,
    input wire in_hs, in_vs, in_de,
    input wire [11:0] in_x, in_y,
    output reg out_hs, out_vs, out_de,
    output reg [11:0] out_x,
    output reg [23:0] out_rgb,
    output reg out_image_valid,
    output reg underflow,
    output reg config_error,
    output wire [11:0] image_width,
    output wire [11:0] image_left,
    output wire [11:0] image_height, image_top
);
    wire geometry_done, geometry_error;
    wire [11:0] draw_width, draw_height, left, top;
    assign image_width = draw_width;
    assign image_left = left;
    assign image_height = draw_height;
    assign image_top = top;
    wire [31:0] step_x, step_y;
    reg config_start, geometry_ready;
    reg [9:0] width_q, height_q, last_source_x, last_source_y;
    video_scale_geometry u_geometry (
        .clk(pclk), .rst_n(rst_n), .start(config_start),
        .source_width(source_width), .source_height(source_height),
        .busy(), .done(geometry_done), .error(geometry_error),
        .draw_width(draw_width), .draw_height(draw_height), .left(left), .top(top),
        .step_x(step_x), .step_y(step_y)
    );

    reg [23:0] line0 [0:1279];
    reg [23:0] line1 [0:1279];
    reg [23:0] line2 [0:1279];
    reg [23:0] line0_q, line1_q, line2_q;
    reg [2:0] line_busy, line_full;
    reg [9:0] line0_row, line1_row, line2_row;
    reg frame_active, frame_failed, stream_started, row_good;
    reg [1:0] write_bank, read_bank;
    reg [9:0] write_y;
    reg [10:0] write_column;
    reg [31:0] h_position, v_position;
    reg [23:0] left_rgb, right_rgb;
    reg left_valid, right_valid, row_open;

    function [1:0] next_bank;
        input [1:0] bank;
        begin next_bank = bank == 2 ? 2'd0 : bank + 1'b1; end
    endfunction
    wire [31:0] next_h_position = h_position + step_x;
    wire [31:0] next_v_position = v_position + step_y;
    wire [9:0] read_y = v_position[29:20];
    wire at_last_row = read_y == last_source_y;
    wire [9:0] neighbour_row = read_y+1'b1;
    wire [1:0] bottom_bank = at_last_row ? read_bank : next_bank(read_bank);
    wire [11:0] right_edge = left + draw_width;
    wire [11:0] bottom_edge = top + draw_height;
    wire image_window = geometry_ready && in_de &&
                        in_x >= left && in_x < right_edge && in_y >= top && in_y < bottom_edge;
    wire row_start = image_window && in_x == left;
    wire row_end = image_window && in_x == right_edge-1'b1;
    // Outside the image, RAM read data is discarded by the aligned good
    // pipeline. Keep the window comparator off the RAM address path.
    wire [10:0] read_column = in_x-left;
    wire top_ready = read_bank == 0 ? line_full[0] && line0_row == read_y :
                     read_bank == 1 ? line_full[1] && line1_row == read_y :
                                      line_full[2] && line2_row == read_y;
    // Compare the next row in parallel with the top row. Do not put the
    // height subtraction and row/bank muxes ahead of three tag comparators.
    // The last source row reuses the top row, with identical pixel timing.
    wire neighbour_ready = read_bank == 0 ? line_full[1] && line1_row == neighbour_row :
                           read_bank == 1 ? line_full[2] && line2_row == neighbour_row :
                                            line_full[0] && line0_row == neighbour_row;
    wire row_ready = top_ready && (at_last_row || neighbour_ready);
    wire release_row = row_end && next_v_position[29:20] != read_y;
    wire write_allowed = frame_active && !frame_failed && geometry_ready && stream_started &&
                         write_y < height_q && (row_open || !line_busy[write_bank]) &&
                         !frame_start && !source_frame_start;
    wire last_column = {1'b0,write_column} == draw_width-1'b1;
    wire advance_pair = next_h_position[29:20] != h_position[29:20];
    wire need_next_sample = left_valid && right_valid && !last_column && advance_pair &&
                            next_h_position[29:20] < last_source_x;
    assign source_ready = write_allowed && (!left_valid || !right_valid || need_next_sample);
    wire source_take = source_ready && source_valid;
    wire emit = write_allowed && left_valid && right_valid &&
                (!need_next_sample || source_valid);
    wire [20:0] h_fraction_rounded = {1'b0,h_position[19:0]} + 21'd2048;
    wire [20:0] v_fraction_rounded = {1'b0,v_position[19:0]} + 21'd2048;

    // Horizontal pipeline: differences -> three signed products -> rounded sum
    // -> row RAM. The bank remains reserved until the display releases it,
    // including the cycles between last emit and the final RAM write.
    reg [2:0] h_valid;
    reg [8:0] h_weight0;
    reg signed [8:0] h_dr0, h_dg0, h_db0;
    reg signed [18:0] h_pr1, h_pg1, h_pb1;
    reg [23:0] h_base0, h_base1;
    reg [18:0] h_r2, h_g2, h_b2;
    reg [1:0] h_bank0, h_bank1, h_bank2;
    reg [10:0] h_col0, h_col1, h_col2;
    reg [9:0] h_row0, h_row1, h_row2;
    reg [2:0] h_last;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            h_valid <= 0; h_last <= 0; h_weight0 <= 0;
            h_dr0 <= 0; h_dg0 <= 0; h_db0 <= 0;
            h_pr1 <= 0; h_pg1 <= 0; h_pb1 <= 0;
            h_base0 <= 0; h_base1 <= 0; h_r2 <= 0; h_g2 <= 0; h_b2 <= 0;
            h_bank0 <= 0; h_bank1 <= 0; h_bank2 <= 0;
            h_col0 <= 0; h_col1 <= 0; h_col2 <= 0;
            h_row0 <= 0; h_row1 <= 0; h_row2 <= 0;
        end else begin
            h_valid <= frame_start ? 3'd0 : {h_valid[1:0],emit};
            h_last <= {h_last[1:0],last_column};
            h_weight0 <= h_fraction_rounded[20:12];
            h_dr0 <= $signed({1'b0,right_rgb[23:16]}) - $signed({1'b0,left_rgb[23:16]});
            h_dg0 <= $signed({1'b0,right_rgb[15:8]}) - $signed({1'b0,left_rgb[15:8]});
            h_db0 <= $signed({1'b0,right_rgb[7:0]}) - $signed({1'b0,left_rgb[7:0]});
            h_base0 <= left_rgb; h_base1 <= h_base0;
            h_pr1 <= h_dr0 * $signed({1'b0,h_weight0});
            h_pg1 <= h_dg0 * $signed({1'b0,h_weight0});
            h_pb1 <= h_db0 * $signed({1'b0,h_weight0});
            h_r2 <= $signed({3'd0,h_base1[23:16],8'd0}) + h_pr1 + 19'sd128;
            h_g2 <= $signed({3'd0,h_base1[15:8],8'd0}) + h_pg1 + 19'sd128;
            h_b2 <= $signed({3'd0,h_base1[7:0],8'd0}) + h_pb1 + 19'sd128;
            h_bank0 <= write_bank; h_bank1 <= h_bank0; h_bank2 <= h_bank1;
            h_col0 <= write_column; h_col1 <= h_col0; h_col2 <= h_col1;
            h_row0 <= write_y; h_row1 <= h_row0; h_row2 <= h_row1;
        end
    end
    always @(posedge pclk) begin
        if (h_valid[2] && h_bank2 == 0) line0[h_col2] <= {h_r2[15:8],h_g2[15:8],h_b2[15:8]};
        if (h_valid[2] && h_bank2 == 1) line1[h_col2] <= {h_r2[15:8],h_g2[15:8],h_b2[15:8]};
        if (h_valid[2] && h_bank2 == 2) line2[h_col2] <= {h_r2[15:8],h_g2[15:8],h_b2[15:8]};
        line0_q <= line0[read_column]; line1_q <= line1[read_column]; line2_q <= line2[read_column];
    end

    reg [2:0] timing0, timing1, timing2, timing3;
    reg [11:0] x0, x1, x2, x3;
    reg good0, good1, good2, good3;
    reg [1:0] top_bank0, bottom_bank0;
    reg [8:0] weight0, weight1;
    reg signed [8:0] dr1, dg1, db1;
    reg signed [18:0] pr2, pg2, pb2;
    reg [23:0] base1, base2;
    reg [18:0] vr, vg, vb;
    wire [23:0] top_rgb = top_bank0 == 0 ? line0_q : top_bank0 == 1 ? line1_q : line2_q;
    wire [23:0] bottom_rgb = bottom_bank0 == 0 ? line0_q : bottom_bank0 == 1 ? line1_q : line2_q;
    // Keep timing and canvas coordinates aligned through business resets.
    // Image-valid/RGB below still clear with the reader/cache state.
    always @(posedge pclk or negedge video_rst_n) begin
        if (!video_rst_n) begin
            timing0 <= 0; timing1 <= 0; timing2 <= 0; timing3 <= 0;
            x0 <= 0; x1 <= 0; x2 <= 0; x3 <= 0;
            out_hs <= 0; out_vs <= 0; out_de <= 0; out_x <= 0;
        end else begin
            timing0 <= {in_hs,in_vs,in_de}; timing1 <= timing0; timing2 <= timing1; timing3 <= timing2;
            x0 <= in_x; x1 <= x0; x2 <= x1; x3 <= x2;
            {out_hs,out_vs,out_de} <= timing3; out_x <= x3;
        end
    end
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            good0 <= 0; good1 <= 0; good2 <= 0; good3 <= 0;
            top_bank0 <= 0; bottom_bank0 <= 0; weight0 <= 0; weight1 <= 0;
            dr1 <= 0; dg1 <= 0; db1 <= 0; pr2 <= 0; pg2 <= 0; pb2 <= 0;
            base1 <= 0; base2 <= 0; vr <= 0; vg <= 0; vb <= 0;
            out_rgb <= 0; out_image_valid <= 0;
        end else begin
            good0 <= image_window && frame_active && !frame_failed &&
                     (row_start ? row_ready : row_good) && !frame_start;
            good1 <= good0; good2 <= good1; good3 <= good2;
            top_bank0 <= read_bank; bottom_bank0 <= bottom_bank;
            weight0 <= v_fraction_rounded[20:12]; weight1 <= weight0;
            dr1 <= $signed({1'b0,bottom_rgb[23:16]}) - $signed({1'b0,top_rgb[23:16]});
            dg1 <= $signed({1'b0,bottom_rgb[15:8]}) - $signed({1'b0,top_rgb[15:8]});
            db1 <= $signed({1'b0,bottom_rgb[7:0]}) - $signed({1'b0,top_rgb[7:0]});
            base1 <= top_rgb; base2 <= base1;
            pr2 <= dr1 * $signed({1'b0,weight1});
            pg2 <= dg1 * $signed({1'b0,weight1});
            pb2 <= db1 * $signed({1'b0,weight1});
            vr <= $signed({3'd0,base2[23:16],8'd0}) + pr2 + 19'sd128;
            vg <= $signed({3'd0,base2[15:8],8'd0}) + pg2 + 19'sd128;
            vb <= $signed({3'd0,base2[7:0],8'd0}) + pb2 + 19'sd128;
            out_image_valid <= good3;
            out_rgb <= good3 ? {vr[15:8],vg[15:8],vb[15:8]} : 24'd0;
        end
    end

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            config_start <= 0; geometry_ready <= 0; width_q <= 0; height_q <= 0;
            last_source_x <= 0; last_source_y <= 0;
            frame_active <= 0; frame_failed <= 0; stream_started <= 0;
            line_busy <= 0; line_full <= 0; line0_row <= 0; line1_row <= 0; line2_row <= 0;
            write_bank <= 0; read_bank <= 0; write_y <= 0; write_column <= 0;
            h_position <= 0; v_position <= 0; left_rgb <= 0; right_rgb <= 0;
            left_valid <= 0; right_valid <= 0; row_open <= 0; row_good <= 0;
            underflow <= 0; config_error <= 0;
        end else if (frame_start) begin
            config_start <= frame_enable; geometry_ready <= 0;
            frame_active <= frame_enable; frame_failed <= 0; stream_started <= 0;
            line_busy <= 0; line_full <= 0;
            write_bank <= 0; read_bank <= 0; write_y <= 0; write_column <= 0;
            h_position <= 0; v_position <= 0;
            left_valid <= 0; right_valid <= 0; row_open <= 0; row_good <= 0;
            underflow <= 0; config_error <= 0;
        end else begin
            config_start <= 0;
            if (config_start) begin
                width_q <= source_width[9:0]; height_q <= source_height[9:0];
                last_source_x <= source_width[9:0]-1'b1;
                last_source_y <= source_height[9:0]-1'b1;
            end
            if (geometry_done && frame_active && !config_start) begin
                geometry_ready <= !geometry_error;
                config_error <= geometry_error;
                if (geometry_error) frame_failed <= 1;
            end
            if (frame_active && source_frame_start && !stream_started) stream_started <= 1;
            if (row_start) begin
                row_good <= row_ready;
                if (frame_active && !frame_failed && !row_ready) begin frame_failed <= 1; underflow <= 1; end
            end
            if (row_end) begin
                v_position <= next_v_position;
                if (release_row) begin
                    read_bank <= next_bank(read_bank);
                    line_busy[read_bank] <= 0;
                    line_full[read_bank] <= 0;
                end
            end
            if (h_valid[2] && h_last[2]) begin
                line_full[h_bank2] <= 1;
                case (h_bank2)
                    0: line0_row <= h_row2;
                    1: line1_row <= h_row2;
                    default: line2_row <= h_row2;
                endcase
            end
            if (source_take) begin
                if (!left_valid) begin
                    left_rgb <= source_rgb; left_valid <= 1; row_open <= 1;
                    line_busy[write_bank] <= 1;
                    if (width_q == 1) begin right_rgb <= source_rgb; right_valid <= 1; end
                end else if (!right_valid) begin right_rgb <= source_rgb; right_valid <= 1; end
            end
            if (emit) begin
                if (last_column) begin
                    write_column <= 0; write_y <= write_y+1'b1; write_bank <= next_bank(write_bank);
                    h_position <= 0; left_valid <= 0; right_valid <= 0; row_open <= 0;
                end else begin
                    write_column <= write_column+1'b1;
                    h_position <= next_h_position;
                    if (advance_pair) begin
                        left_rgb <= right_rgb;
                        right_rgb <= need_next_sample ? source_rgb : right_rgb;
                    end
                end
            end
        end
    end
endmodule
