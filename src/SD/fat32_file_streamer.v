`timescale 1ns/1ps

module fat32_file_streamer #(
    parameter integer MAX_VISITED_CLUSTERS = 8192,
    parameter integer SUPPORT_CANCEL = 0
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        file_start,
    input  wire        file_cancel,
    input  wire [31:0] first_cluster,
    input  wire [31:0] file_size,
    input  wire [7:0]  sectors_per_cluster,
    input  wire [31:0] fat_start_lba,
    input  wire [31:0] data_start_lba,
    input  wire [31:0] max_cluster,
    // Admission for one complete 512-byte data sector. Sample only before
    // issuing its request; an in-flight sector and FAT reads always drain.
    input  wire        data_sector_ready,
    output wire        file_busy,
    output reg         file_done,
    output reg         file_valid,
    output reg  [7:0]  file_byte,
    output reg  [31:0] file_byte_offset,
    output reg  [7:0]  fs_error,
    output wire        sector_req,
    output reg  [31:0] sector_lba,
    input  wire        sector_accept,
    input  wire        sector_valid,
    input  wire [7:0]  sector_byte,
    input  wire [8:0]  sector_byte_index,
    input  wire        sector_done,
    input  wire [7:0]  sector_error
);
    localparam ERR_NONE              = 8'h00;
    localparam ERR_SECTOR            = 8'h10;
    localparam ERR_FAT_LOOP          = 8'h23;
    localparam ERR_CLUSTER_RANGE     = 8'h24;
    localparam ERR_EARLY_EOF         = 8'h25;
    localparam ERR_VISITED_EXHAUSTED = 8'h26;

    localparam ST_IDLE       = 4'd0;
    localparam ST_DATA_REQ   = 4'd1;
    localparam ST_DATA_WAIT  = 4'd2;
    localparam ST_FAT_REQ    = 4'd3;
    localparam ST_FAT_WAIT   = 4'd4;
    localparam ST_FAT_PARSE  = 4'd5;
    localparam ST_FAT_CHECK  = 4'd6;
    localparam ST_FINISH     = 4'd7;
    localparam ST_CLUSTER_OFFSET = 4'd8;
    localparam ST_CLUSTER_SCALE  = 4'd9;
    localparam ST_CLUSTER_LBA    = 4'd10;
    localparam ST_DATA_ADVANCE   = 4'd11;

    localparam integer CHAIN_COUNT_WIDTH = $clog2(MAX_VISITED_CLUSTERS + 1);

    reg [3:0] state;
    reg [31:0] current_cluster;
    reg [31:0] next_cluster;
    reg next_cluster_eof;
    reg next_cluster_invalid;
    reg next_cluster_self;
    reg [31:0] cluster_offset;
    reg [31:0] cluster_sector_offset;
    reg [31:0] remaining_bytes;
    reg [7:0] sector_in_cluster;
    reg [8:0] fat_entry_offset;
    reg [7:0] fat_byte0, fat_byte1, fat_byte2, fat_byte3;
    reg [CHAIN_COUNT_WIDTH-1:0] chain_count;
    reg final_chain_check;
    reg data_request_armed;

    assign file_busy = (state != ST_IDLE);
    // Register sector admission before requesting the adapter. This separates
    // ring-pointer decoding/capacity arithmetic from arbitration/address enable.
    // Once admitted, hold the reservation until acceptance. Only this streamer
    // writes its buffer, so available capacity cannot shrink while it waits.
    wire cancelling = SUPPORT_CANCEL && file_cancel;
    assign sector_req = !cancelling && ((state == ST_FAT_REQ) ||
                        ((state == ST_DATA_REQ) && data_request_armed));

    task finish_with_error;
        input [7:0] code;
        begin
            fs_error <= code;
            state <= ST_FINISH;
        end
    endtask

    task advance_to_next_cluster;
        input [31:0] cluster_value;
        begin
            chain_count <= chain_count + 1'b1;
            current_cluster <= cluster_value;
            state <= ST_CLUSTER_OFFSET;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            file_done <= 1'b0;
            file_valid <= 1'b0;
            file_byte <= 8'd0;
            file_byte_offset <= 32'd0;
            fs_error <= ERR_NONE;
            sector_lba <= 32'd0;
            current_cluster <= 32'd0;
            next_cluster <= 32'd0;
            next_cluster_eof <= 1'b0;
            next_cluster_invalid <= 1'b0;
            next_cluster_self <= 1'b0;
            cluster_offset <= 32'd0;
            cluster_sector_offset <= 32'd0;
            remaining_bytes <= 32'd0;
            sector_in_cluster <= 8'd0;
            fat_entry_offset <= 9'd0;
            fat_byte0 <= 8'd0;
            fat_byte1 <= 8'd0;
            fat_byte2 <= 8'd0;
            fat_byte3 <= 8'd0;
            chain_count <= 0;
            final_chain_check <= 1'b0;
            data_request_armed <= 1'b0;
        end else begin
            file_done <= 1'b0;
            file_valid <= 1'b0;

            if (!cancelling && state == ST_DATA_WAIT && sector_valid && remaining_bytes != 0) begin
                file_valid <= 1'b1;
                file_byte <= sector_byte;
                file_byte_offset <= file_size - remaining_bytes;
                remaining_bytes <= remaining_bytes - 1'b1;
            end

            if (state == ST_FAT_WAIT && sector_valid) begin
                if (sector_byte_index == fat_entry_offset)
                    fat_byte0 <= sector_byte;
                else if (sector_byte_index == fat_entry_offset + 1'b1)
                    fat_byte1 <= sector_byte;
                else if (sector_byte_index == fat_entry_offset + 2'd2)
                    fat_byte2 <= sector_byte;
                else if (sector_byte_index == fat_entry_offset + 2'd3)
                    fat_byte3 <= sector_byte;
            end

            // Cancel only withdraws requests which have not been accepted.
            // An accepted data/FAT sector retains arbiter ownership and drains
            // completely before file_busy falls. Never reset a live SD read.
            if (cancelling && state != ST_IDLE && state != ST_FINISH) begin
                data_request_armed <= 1'b0;
                if ((state != ST_DATA_WAIT && state != ST_FAT_WAIT) || sector_done)
                    state <= ST_FINISH;
            end else case (state)
                ST_IDLE: if (file_start) begin
                    fs_error <= ERR_NONE;
                    file_byte_offset <= 32'd0;
                    remaining_bytes <= file_size;
                    sector_in_cluster <= 8'd0;
                    chain_count <= 1;
                    final_chain_check <= 1'b0;
                    if (file_size == 0) begin
                        state <= ST_FINISH;
                    end else if (first_cluster < 2 || first_cluster > max_cluster ||
                                 sectors_per_cluster == 0) begin
                        finish_with_error(ERR_CLUSTER_RANGE);
                    end else begin
                        current_cluster <= first_cluster;
                        state <= ST_CLUSTER_OFFSET;
                    end
                end
                // Each arithmetic operation has its own register boundary.
                // Both the first cluster and subsequent FAT links use this path.
                // No request is issued until the complete LBA is registered.
                ST_CLUSTER_OFFSET: begin
                    cluster_offset <= current_cluster - 32'd2;
                    state <= ST_CLUSTER_SCALE;
                end
                ST_CLUSTER_SCALE: begin
                    cluster_sector_offset <= cluster_offset * sectors_per_cluster;
                    state <= ST_CLUSTER_LBA;
                end
                ST_CLUSTER_LBA: begin
                    sector_lba <= data_start_lba + cluster_sector_offset;
                    state <= ST_DATA_REQ;
                end
                ST_DATA_REQ: begin
                    if (sector_accept) begin
                        data_request_armed <= 1'b0;
                        state <= ST_DATA_WAIT;
                    end else if (data_sector_ready) begin
                        data_request_armed <= 1'b1;
                    end
                end
                ST_DATA_WAIT: if (sector_done) begin
                    if (sector_error != 0) begin
                        finish_with_error(ERR_SECTOR);
                    end else begin
                        // 先锁存文件尾判断，再在下一拍选择/更新地址。
                        // sector_valid 与最后一个字节同拍到达时也必须计入。
                        // 避免 SD 返回有效信号一路穿过结束判断和 32 位 LBA 选择。
                        final_chain_check <= (remaining_bytes == 0) ||
                                             (remaining_bytes == 1 && sector_valid);
                        state <= ST_DATA_ADVANCE;
                    end
                end
                ST_DATA_ADVANCE: begin
                    if (final_chain_check ||
                        !(sector_in_cluster + 1'b1 < sectors_per_cluster)) begin
                        sector_in_cluster <= 8'd0;
                        fat_entry_offset <= {current_cluster[6:0], 2'b00};
                        sector_lba <= fat_start_lba + (current_cluster >> 7);
                        state <= ST_FAT_REQ;
                    end else begin
                        sector_in_cluster <= sector_in_cluster + 1'b1;
                        sector_lba <= sector_lba + 1'b1;
                        state <= ST_DATA_REQ;
                    end
                end
                ST_FAT_REQ: if (sector_accept) begin
                    state <= ST_FAT_WAIT;
                end
                ST_FAT_WAIT: if (sector_done) begin
                    if (sector_error != 0)
                        finish_with_error(ERR_SECTOR);
                    else
                        state <= ST_FAT_PARSE;
                end
                ST_FAT_PARSE: begin
                    next_cluster <= {4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0};
                    next_cluster_eof <= {fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} >= 28'hffffff8;
                    next_cluster_invalid <=
                        {4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} < 32'd2 ||
                        {4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} > max_cluster;
                    next_cluster_self <=
                        {4'd0, fat_byte3[3:0], fat_byte2, fat_byte1, fat_byte0} == current_cluster;
                    state <= ST_FAT_CHECK;
                end
                ST_FAT_CHECK: begin
                    if (final_chain_check) begin
                        final_chain_check <= 1'b0;
                        if (next_cluster_eof)
                            state <= ST_FINISH;
                        else if (next_cluster_invalid)
                            finish_with_error(ERR_CLUSTER_RANGE);
                        else
                            finish_with_error(ERR_FAT_LOOP);
                    end else if (next_cluster_eof) begin
                        finish_with_error(ERR_EARLY_EOF);
                    end else if (next_cluster_invalid) begin
                        finish_with_error(ERR_CLUSTER_RANGE);
                    end else if (chain_count >= MAX_VISITED_CLUSTERS) begin
                        finish_with_error(ERR_VISITED_EXHAUSTED);
                    end else if (next_cluster_self) begin
                        finish_with_error(ERR_FAT_LOOP);
                    end else begin
                        advance_to_next_cluster(next_cluster);
                    end
                end
                ST_FINISH: begin
                    file_done <= 1'b1;
                    state <= ST_IDLE;
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule

