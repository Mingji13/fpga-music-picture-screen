`timescale 1ns/1ps

// 128 根上下对称的幅度柱，叠加在已经缩放、转场处理过的 RGB 上。
// 输入要求：同一像素时钟域，RGB/HS/VS/DE/in_x 已对齐；每行 x 逐像素递增。
// VS 低有效，与本工程 video_timing_ctrl / video_scale_3_2 一致。
// 输出固定经过三级寄存，关闭叠加时也保持相同延迟。
module video_waveform_overlay #(
    parameter integer WAVE_X = 256,
    parameter integer CENTER_Y = 608,
    parameter [23:0] BAR_RGB = 24'h20d0ff,
    parameter [23:0] AXIS_RGB = 24'h406060
) (
    input  wire        pclk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire [2047:0] history_peaks,
    input  wire        in_hs,
    input  wire        in_vs,
    input  wire        in_de,
    input  wire [11:0] in_x,
    input  wire [23:0] in_rgb,
    output reg         out_hs,
    output reg         out_vs,
    output reg         out_de,
    output reg [23:0]  out_rgb,
    output reg         overlay_active
);
    // 固定 128 根柱，每个槽位 6 像素：前 4 像素画柱，后 2 像素留空。
    localparam integer WAVE_WIDTH = 768;
    localparam integer MAX_HALF_HEIGHT = 64;

    reg vs_q, de_q;
    reg [11:0] line_y;
    reg frame_enabled;
    reg [6:0] frame_height [0:127];
    wire frame_start = vs_q && !in_vs;

    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            vs_q <= 1'b1;
            de_q <= 1'b0;
            line_y <= 12'd0;
            frame_enabled <= 1'b0;
        end else begin
            vs_q <= in_vs;
            de_q <= in_de;
            if (!in_vs)
                line_y <= 12'd0;
            else if (de_q && !in_de)
                line_y <= line_y + 1'b1;
            if (frame_start)
                frame_enabled <= enable;
        end
    end

    // 在帧边界一次性保存所有柱高；帧中途的音频更新不会改变当前画面。
    // 柱高 = floor(幅度 / 512)，0..32768 映射到 0..64 像素。
    // 这里只需保留高 7 位，避免另外保存一份完整的 16 位历史。
    genvar b;
    generate for (b = 0; b < 128; b = b + 1) begin : snapshot
        wire [6:0] height = history_peaks[(127-b)*16+9 +: 7];
        always @(posedge pclk or negedge rst_n) begin
            if (!rst_n)
                frame_height[b] <= 7'd0;
            else if (frame_start)
                frame_height[b] <= (height > 7'd64) ? 7'd64 : height;
        end
    end endgenerate

    // 根据顺序到达的像素计数，省去 (in_x-WAVE_X)/6 的组合除法。
    // 进入绘图区的第一个像素使用 index=0, phase=0；每行重新开始。
    wire inside_x = in_de && (in_x >= WAVE_X) && (in_x < WAVE_X+WAVE_WIDTH);
    reg [6:0] bar_index;
    reg [2:0] column_phase;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            bar_index <= 7'd0;
            column_phase <= 3'd0;
        end else if (!inside_x) begin
            bar_index <= 7'd0;
            column_phase <= 3'd0;
        end else if (column_phase == 3'd5) begin
            bar_index <= bar_index + 1'b1;
            column_phase <= 3'd0;
        end else begin
            column_phase <= column_phase + 1'b1;
        end
    end

    wire inside_wave = inside_x && frame_enabled &&
                       (line_y >= CENTER_Y-MAX_HALF_HEIGHT) &&
                       (line_y <= CENTER_Y+MAX_HALF_HEIGHT);
    wire [11:0] center_distance = (line_y >= CENTER_Y) ?
                                  line_y-CENTER_Y : CENTER_Y-line_y;

    // 第一级：把 128 选 1 拆成八组 16 选 1，同时寄存原图与绘图条件。
    reg [6:0] group_height_s0 [0:7];
    reg [2:0] group_select_s0;
    reg [11:0] distance_s0;
    reg area_s0, body_s0, axis_s0;
    reg hs_s0, vs_s0, de_s0;
    reg [23:0] rgb_s0;

    // 第二级：八组选一，继续让像素、同步信号和绘图条件一起前进。
    reg [6:0] height_s1;
    reg [11:0] distance_s1;
    reg area_s1, body_s1, axis_s1;
    reg hs_s1, vs_s1, de_s1;
    reg [23:0] rgb_s1;

    wire bar_pixel = area_s1 && body_s1 && (height_s1 != 7'd0) &&
                     (distance_s1 <= {5'd0, height_s1});
    wire axis_pixel = area_s1 && axis_s1;

    integer g;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            for (g = 0; g < 8; g = g + 1)
                group_height_s0[g] <= 7'd0;
            group_select_s0 <= 3'd0;
            distance_s0 <= 12'd0;
            area_s0 <= 1'b0;
            body_s0 <= 1'b0;
            axis_s0 <= 1'b0;
            hs_s0 <= 1'b1;
            vs_s0 <= 1'b1;
            de_s0 <= 1'b0;
            rgb_s0 <= 24'd0;
            height_s1 <= 7'd0;
            distance_s1 <= 12'd0;
            area_s1 <= 1'b0;
            body_s1 <= 1'b0;
            axis_s1 <= 1'b0;
            hs_s1 <= 1'b1;
            vs_s1 <= 1'b1;
            de_s1 <= 1'b0;
            rgb_s1 <= 24'd0;
            out_hs <= 1'b1;
            out_vs <= 1'b1;
            out_de <= 1'b0;
            out_rgb <= 24'd0;
            overlay_active <= 1'b0;
        end else begin
            for (g = 0; g < 8; g = g + 1)
                group_height_s0[g] <= frame_height[g*16+bar_index[3:0]];
            group_select_s0 <= bar_index[6:4];
            distance_s0 <= center_distance;
            area_s0 <= inside_wave;
            body_s0 <= (column_phase < 3'd4);
            axis_s0 <= (line_y == CENTER_Y);
            hs_s0 <= in_hs;
            vs_s0 <= in_vs;
            de_s0 <= in_de;
            rgb_s0 <= in_rgb;

            height_s1 <= group_height_s0[group_select_s0];
            distance_s1 <= distance_s0;
            area_s1 <= area_s0;
            body_s1 <= body_s0;
            axis_s1 <= axis_s0;
            hs_s1 <= hs_s0;
            vs_s1 <= vs_s0;
            de_s1 <= de_s0;
            rgb_s1 <= rgb_s0;

            // 第三级：仅覆盖柱子/中线像素，其他位置逐像素保留原图。
            out_hs <= hs_s1;
            out_vs <= vs_s1;
            out_de <= de_s1;
            out_rgb <= bar_pixel ? BAR_RGB : axis_pixel ? AXIS_RGB : rgb_s1;
            overlay_active <= bar_pixel || axis_pixel;
        end
    end
endmodule
