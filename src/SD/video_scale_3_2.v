`timescale 1ns/1ps

// 640x480 row-major RGB -> centered 960x720 on a 1280x720 canvas.
// Separable bilinear: horizontal on stream admission, vertical on display.
// Coordinates are 2*dst/3; Q8 fractions cycle 0,171,85. Round after each axis;
// clamp the right/bottom neighbour at the image edge.
//
// Three 960-pixel synchronous RAMs hold horizontally interpolated rows:
// two feed the display while the third prefetches the next source row.
// Each source pixel is accepted exactly once (307200 transfers/frame).
//
// frame_start is in vertical blanking and latches frame_enable. Only
// source_frame_start, AFTER the reader flushes its previous FIFO epoch,
// admits the fresh stream. Underflow blanks the remainder of the frame,
// without stopping HDMI timing; the next frame can recover.
// RGB, image-valid, HS/VS/DE and active canvas X have FIVE pclk stages.
module video_scale_3_2 (
    input  wire        pclk,
    input  wire        rst_n,
    input  wire        frame_start,
    input  wire        frame_enable,
    input  wire        source_frame_start,
    input  wire        source_valid,
    input  wire [23:0] source_rgb,
    output wire        source_ready,
    input  wire        in_hs,
    input  wire        in_vs,
    input  wire        in_de,
    input  wire [11:0] in_x,
    input  wire [11:0] in_y,
    output reg         out_hs,
    output reg         out_vs,
    output reg         out_de,
    output reg  [11:0] out_x,
    output reg  [23:0] out_rgb,
    output reg         out_image_valid,
    output reg         underflow
);
    reg [23:0] line0 [0:959];
    reg [23:0] line1 [0:959];
    reg [23:0] line2 [0:959];
    reg [23:0] line0_q, line1_q, line2_q;
    reg [2:0] line_full;
    reg [8:0] line0_row, line1_row, line2_row;
    reg frame_active, frame_failed, stream_started;
    reg [9:0] write_x, write_column;
    reg [8:0] write_y, read_y;
    reg [1:0] write_bank, read_bank, y_phase;
    reg [23:0] previous_rgb;
    reg copy_pending, row_good;

    function [1:0] next_bank;
        input [1:0] bank;
        begin next_bank = (bank == 2'd2) ? 2'd0 : bank + 1'b1; end
    endfunction

    // Only constant weights are needed for 3:2, not variable multipliers.
    // 255*256+128 fits in 16 bits. Equal colours and weight=0 remain exact.
    function [15:0] weighted_sum;
        input [7:0] a, b;
        input [1:0] phase;
        reg signed [16:0] difference, times5, times85, base;
        begin
            // 85 = (1+4)*(1+16), 171 = 256-85. Explicit add/shift
            // arithmetic avoids TD mapping twelve constant products to DSPs.
            // Signed extension is essential when the neighbour is darker.
            difference = (phase == 2'd1) ?
                         $signed({9'd0,a}) - $signed({9'd0,b}) :
                         $signed({9'd0,b}) - $signed({9'd0,a});
            base = (phase == 2'd1) ? $signed({1'b0,b,8'd0}) :
                                                    $signed({1'b0,a,8'd0});
            times5 = difference + (difference <<< 2);
            times85 = times5 + (times5 <<< 4);
            weighted_sum = (phase == 2'd0) ? {a,8'd0} :
                                            base + times85 + 17'sd128;
        end
    endfunction

    wire image_window = in_de && (in_x >= 12'd160) &&
                        (in_x < 12'd1120) && (in_y < 12'd720);
    wire row_start = image_window && (in_x == 12'd160);
    wire row_end = image_window && (in_x == 12'd1119);
    wire [9:0] read_column = image_window ? (in_x - 12'd160) : 10'd0;
    wire [1:0] bottom_bank = (read_y == 9'd479) ? read_bank : next_bank(read_bank);
    wire [8:0] bottom_row = (read_y == 9'd479) ? read_y : read_y + 1'b1;
    wire top_ready = (read_bank == 2'd0) ? (line_full[0] && line0_row == read_y) :
                     (read_bank == 2'd1) ? (line_full[1] && line1_row == read_y) :
                                           (line_full[2] && line2_row == read_y);
    wire bottom_ready = (bottom_bank == 2'd0) ? (line_full[0] && line0_row == bottom_row) :
                        (bottom_bank == 2'd1) ? (line_full[1] && line1_row == bottom_row) :
                                              (line_full[2] && line2_row == bottom_row);
    wire row_ready = top_ready && bottom_ready;
    wire this_row_good = row_start ? row_ready : row_good;
    wire release_row = row_end && (y_phase != 2'd0);

    wire write_allowed = frame_active && !frame_failed && stream_started &&
                         (write_y < 9'd480) && !line_full[write_bank] &&
                         !frame_start && !source_frame_start;
    assign source_ready = write_allowed && !copy_pending;
    wire source_take = source_valid && source_ready;
    wire line_write = write_allowed && (source_take || copy_pending);
    wire [1:0] h_phase = write_x[0] ? 2'd1 : 2'd2;
    wire [15:0] h_r = weighted_sum(previous_rgb[23:16], source_rgb[23:16], h_phase);
    wire [15:0] h_g = weighted_sum(previous_rgb[15:8], source_rgb[15:8], h_phase);
    wire [15:0] h_b = weighted_sum(previous_rgb[7:0], source_rgb[7:0], h_phase);
    wire [23:0] horizontal_rgb = copy_pending ? previous_rgb :
                                (write_x == 10'd0) ? source_rgb :
                                {h_r[15:8], h_g[15:8], h_b[15:8]};

    // Unreset synchronous RAMs infer BRAM. Flags/tags own all validity.
    always @(posedge pclk) begin
        if (line_write && write_bank == 2'd0) line0[write_column] <= horizontal_rgb;
        if (line_write && write_bank == 2'd1) line1[write_column] <= horizontal_rgb;
        if (line_write && write_bank == 2'd2) line2[write_column] <= horizontal_rgb;
        line0_q <= line0[read_column];
        line1_q <= line1[read_column];
        line2_q <= line2[read_column];
    end

    // Vertical pipeline: RAM -> signed difference -> times85 -> rounded sum
    // -> output. No RAM-to-output path contains the whole interpolation.
    function signed [8:0] blend_difference;
        input [7:0] a, b;
        input [1:0] phase;
        begin
            blend_difference = (phase == 2'd0) ? 9'sd0 :
                               (phase == 2'd1) ?
                               $signed({1'b0,a}) - $signed({1'b0,b}) :
                               $signed({1'b0,b}) - $signed({1'b0,a});
        end
    endfunction
    function signed [15:0] multiply85;
        input signed [8:0] difference;
        reg signed [11:0] extended, times5;
        reg signed [15:0] extended5;
        begin
            extended = {{3{difference[8]}},difference};
            times5 = extended + (extended <<< 2);
            extended5 = {{4{times5[11]}},times5};
            multiply85 = extended5 + (extended5 <<< 4);
        end
    endfunction
    reg [2:0] timing0, timing1, timing2, timing3;
    reg [11:0] x0, x1, x2, x3;
    reg good0, good1, good2, good3;
    reg [1:0] top_bank0, bottom_bank0, y_phase0;
    reg signed [8:0] diff_r1, diff_g1, diff_b1;
    reg signed [15:0] product_r2, product_g2, product_b2;
    reg [23:0] base1, base2;
    reg [15:0] v_r, v_g, v_b;
    wire [23:0] top_rgb = (top_bank0 == 2'd0) ? line0_q :
                          (top_bank0 == 2'd1) ? line1_q : line2_q;
    wire [23:0] bottom_rgb = (bottom_bank0 == 2'd0) ? line0_q :
                             (bottom_bank0 == 2'd1) ? line1_q : line2_q;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            timing0 <= 3'd0; timing1 <= 3'd0; timing2 <= 3'd0; timing3 <= 3'd0;
            x0 <= 12'd0; x1 <= 12'd0; x2 <= 12'd0; x3 <= 12'd0;
            good0 <= 1'b0; good1 <= 1'b0; good2 <= 1'b0; good3 <= 1'b0;
            top_bank0 <= 2'd0; bottom_bank0 <= 2'd0; y_phase0 <= 2'd0;
            diff_r1 <= 9'sd0; diff_g1 <= 9'sd0; diff_b1 <= 9'sd0;
            product_r2 <= 16'sd0; product_g2 <= 16'sd0; product_b2 <= 16'sd0;
            base1 <= 24'd0; base2 <= 24'd0;
            v_r <= 16'd0; v_g <= 16'd0; v_b <= 16'd0;
            out_hs <= 1'b0; out_vs <= 1'b0; out_de <= 1'b0;
            out_x <= 12'd0; out_rgb <= 24'd0; out_image_valid <= 1'b0;
        end else begin
            timing0 <= {in_hs, in_vs, in_de}; timing1 <= timing0;
            timing2 <= timing1; timing3 <= timing2;
            x0 <= in_x; x1 <= x0; x2 <= x1; x3 <= x2;
            top_bank0 <= read_bank; bottom_bank0 <= bottom_bank; y_phase0 <= y_phase;
            good0 <= image_window && frame_active && !frame_failed &&
                     this_row_good && !frame_start;
            good1 <= good0; good2 <= good1; good3 <= good2;
            diff_r1 <= blend_difference(top_rgb[23:16], bottom_rgb[23:16], y_phase0);
            diff_g1 <= blend_difference(top_rgb[15:8], bottom_rgb[15:8], y_phase0);
            diff_b1 <= blend_difference(top_rgb[7:0], bottom_rgb[7:0], y_phase0);
            base1 <= (y_phase0 == 2'd1) ? bottom_rgb : top_rgb;
            product_r2 <= multiply85(diff_r1);
            product_g2 <= multiply85(diff_g1);
            product_b2 <= multiply85(diff_b1);
            base2 <= base1;
            v_r <= {base2[23:16],8'd0} + product_r2 + 16'd128;
            v_g <= {base2[15:8],8'd0} + product_g2 + 16'd128;
            v_b <= {base2[7:0],8'd0} + product_b2 + 16'd128;
            {out_hs, out_vs, out_de} <= timing3;
            out_x <= x3;
            out_image_valid <= good3;
            out_rgb <= good3 ? {v_r[15:8], v_g[15:8], v_b[15:8]} : 24'd0;
        end
    end

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            frame_active <= 1'b0; frame_failed <= 1'b0; stream_started <= 1'b0;
            line_full <= 3'd0;
            line0_row <= 9'd0; line1_row <= 9'd0; line2_row <= 9'd0;
            write_x <= 10'd0; write_column <= 10'd0; write_y <= 9'd0;
            write_bank <= 2'd0; read_bank <= 2'd0; read_y <= 9'd0; y_phase <= 2'd0;
            previous_rgb <= 24'd0; copy_pending <= 1'b0; row_good <= 1'b0;
            underflow <= 1'b0;
        end else if (frame_start) begin
            frame_active <= frame_enable; frame_failed <= 1'b0; stream_started <= 1'b0;
            line_full <= 3'd0;
            write_x <= 10'd0; write_column <= 10'd0; write_y <= 9'd0;
            write_bank <= 2'd0; read_bank <= 2'd0; read_y <= 9'd0; y_phase <= 2'd0;
            previous_rgb <= 24'd0; copy_pending <= 1'b0; row_good <= 1'b0;
            underflow <= 1'b0;
        end else begin
            if (row_start) begin
                row_good <= row_ready;
                if (frame_active && !frame_failed && !row_ready) begin
                    frame_failed <= 1'b1;
                    underflow <= 1'b1;
                end
            end
            if (row_end) begin
                y_phase <= (y_phase == 2'd2) ? 2'd0 : y_phase + 1'b1;
                if (y_phase != 2'd0) begin
                    read_y <= read_y + 1'b1;
                    read_bank <= next_bank(read_bank);
                end
            end
            if (frame_active && !frame_failed) begin
                if (source_frame_start && !stream_started) stream_started <= 1'b1;
                if (release_row) line_full[read_bank] <= 1'b0;
                if (source_take) begin
                    previous_rgb <= source_rgb;
                    write_x <= (write_x == 10'd639) ? 10'd0 : write_x + 1'b1;
                    // Each even source pixel after x=0 emits a blend followed
                    // by itself. Last odd pixel also emits itself (edge clamp).
                    copy_pending <= (write_x != 10'd0) &&
                                    (!write_x[0] || write_x == 10'd639);
                end else if (line_write) begin
                    copy_pending <= 1'b0;
                end
                if (line_write) begin
                    if (write_column == 10'd959) begin
                        write_column <= 10'd0;
                        write_y <= write_y + 1'b1;
                        write_bank <= next_bank(write_bank);
                        line_full[write_bank] <= 1'b1;
                        case (write_bank)
                            2'd0: line0_row <= write_y;
                            2'd1: line1_row <= write_y;
                            default: line2_row <= write_y;
                        endcase
                    end else write_column <= write_column + 1'b1;
                end
            end
        end
    end
endmodule
