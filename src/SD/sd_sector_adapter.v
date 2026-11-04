`timescale 1ns/1ps

// SD卡扇区读接口适配层。
// 将FAT32模块的sector_*接口转换成官方sd_card_top的sd_sec_*接口。
module sd_sector_adapter #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer SD_READ_TIMEOUT_MS = 500
)(
    input  wire        clk,
    input  wire        rst_n,                   // 低有效复位
    // FAT32模块侧：单周期请求脉冲，在sector_busy为0时发出。
    input  wire        sector_req,
    input  wire [31:0] sector_lba,
    output wire        sector_busy,
    output wire        sector_valid,
    output wire [7:0]  sector_byte,
    output wire [8:0]  sector_byte_index,       // 当前有效字节的序号，0~511
    output wire        sector_done,
    output wire [7:0]  sector_error,
    // 无进展超时保持到复位，由顶层统一恢复整条SD读取链路。
    output reg         read_timeout,
    // 官方sd_card_top侧：请求保持到sd_sec_read_end有效。
    output reg         sd_sec_read,
    output reg  [31:0] sd_sec_read_addr,
    input  wire        sd_sec_read_data_valid,
    input  wire [7:0]  sd_sec_read_data,
    input  wire        sd_sec_read_end
);

    reg [8:0] byte_cnt;
    reg       busy_reg;
    reg [31:0] read_wait_cnt;
    localparam integer READ_TIMEOUT_CYCLES =
        ((CLK_FREQ_HZ / 1000) * SD_READ_TIMEOUT_MS > 0) ?
         (CLK_FREQ_HZ / 1000) * SD_READ_TIMEOUT_MS : 1;

    assign sector_busy       = busy_reg;
    assign sector_valid      = busy_reg && !read_timeout && sd_sec_read_data_valid;
    assign sector_byte       = sd_sec_read_data;
    assign sector_byte_index = byte_cnt;
    assign sector_done       = busy_reg && !read_timeout && sd_sec_read_end;
    // 官方接口没有错误输出，仍按文档固定为0。
    // 超时不伪造sector_done，避免底层旧请求尚未结束时上层直接重试。
    assign sector_error      = 8'd0;

    // 只统计一笔扇区读取期间连续无数据、无结束通知的时间。
    // 整张图片可以读很久；每收到一个字节就重新计时。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            read_wait_cnt <= 32'd0;
            read_timeout  <= 1'b0;
        end else if (!read_timeout) begin
            if (!busy_reg || sd_sec_read_data_valid || sd_sec_read_end)
                read_wait_cnt <= 32'd0;
            else if (read_wait_cnt >= READ_TIMEOUT_CYCLES - 1)
                read_timeout <= 1'b1;
            else
                read_wait_cnt <= read_wait_cnt + 32'd1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sd_sec_read      <= 1'b0;
            sd_sec_read_addr <= 32'd0;
            byte_cnt        <= 9'd0;
            busy_reg        <= 1'b0;
        end else if (!read_timeout) begin
            if (sector_req && !busy_reg) begin
                sd_sec_read      <= 1'b1;
                sd_sec_read_addr <= sector_lba;
                busy_reg        <= 1'b1;
                byte_cnt        <= 9'd0;
            end else if (sd_sec_read_end) begin
                sd_sec_read <= 1'b0;
                busy_reg    <= 1'b0;
                byte_cnt    <= 9'd0;
            end else if (sd_sec_read_data_valid) begin
                byte_cnt <= byte_cnt + 9'd1;
            end
        end
    end

endmodule
