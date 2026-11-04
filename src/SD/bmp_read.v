module bmp_read#(
    parameter integer CLK_FREQ_HZ        = 100_000_000,
    parameter integer SD_READ_TIMEOUT_MS = 500
)(
    input                       clk,
    input                       rst,
    output                      ready,

    // 上电扫描：从 scan_start_sector 开始，顺序寻找前 scan_target_count 张 BMP
    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [2:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [2:0]            scan_found_total,

    // 按指定扇区加载一张图到 SDRAM
    input                       load_start,
    input  [31:0]               load_sector,

    input                       sd_init_done,
    output reg [3:0]            state_code,
    output reg [2:0]            bmp_error,
    output reg                  load_error,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,

    output reg                  write_req,
    input                       write_req_ack,

    output reg                  sd_sec_read,
    output reg [31:0]           sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    output reg                  bmp_data_wr_en,
    output reg [23:0]           bmp_data
);

localparam ST_IDLE      = 3'd0;
localparam ST_SCAN      = 3'd1;
localparam ST_LOAD_HDR  = 3'd2;
localparam ST_LOAD_WAIT = 3'd3;
localparam ST_LOAD_DATA = 3'd4;

localparam ERR_NONE      = 3'd0;
localparam ERR_HEADER    = 3'd1;
localparam ERR_TIMEOUT   = 3'd2;
localparam ERR_TRUNCATED = 3'd3;

localparam integer TIMEOUT_CYCLES = (CLK_FREQ_HZ / 1000) * SD_READ_TIMEOUT_MS;
localparam integer EXPECTED_PIXELS = 640 * 480;

reg [2:0]  state;
reg [9:0]  rd_cnt;

reg [7:0]  header_0;
reg [7:0]  header_1;
reg [31:0] file_len;
reg [31:0] pixel_offset;
reg [31:0] dib_size;
reg [31:0] width;
reg [31:0] height;
reg [15:0] planes;
reg [15:0] bit_count;
reg [31:0] compression;

reg [31:0] scan_sector;
reg [31:0] load_sector_latched;
reg [31:0] bmp_len_cnt;
reg [1:0]  bmp_byte_idx;
reg [31:0] pixel_count;

reg [31:0] sd_timeout_cnt;

wire header_match;
wire bmp_signature_match;
wire bmp_data_valid;
wire fast_skip_valid;

wire [31:0] file_sector_count;
wire [31:0] next_scan_sector_if_match;
wire [31:0] next_scan_sector_if_miss;

wire timeout_watch_active;
wire timeout_progress;
wire timeout_expired;//已超时

wire       pixel_output_now;
wire [31:0] pixel_count_with_current;

assign ready = (state == ST_IDLE);
assign bmp_signature_match =(header_0 == "B") && (header_1 == "M");//宽松识别，只要前两个字节是BM就为1
assign header_match = bmp_signature_match &&
                      (dib_size >= 32'd40) &&
                      (width  == {16'd0, bmp_width}) &&
                      (height == {16'd0, bmp_height}) &&
                      (planes == 16'd1) &&
                      (bit_count == 16'd24) &&
                      (compression == 32'd0) &&
                      (pixel_offset >= 32'd54) &&
                      (pixel_offset < file_len) &&
                      (file_len >= pixel_offset +
                       ({16'd0, bmp_width} *
                        {16'd0, bmp_height} *
                        32'd3));

assign fast_skip_valid = bmp_signature_match &&
    (dib_size >= 32'd40) &&
    (file_len >= 32'd54) &&
    (file_len <= 32'hFFFF_FE00) &&
    (scan_sector <= scan_max_sector) &&
    (file_sector_count <=
        (scan_max_sector - scan_sector));

assign bmp_data_valid = (sd_sec_read_data_valid == 1'b1) &&
                        (bmp_len_cnt >= pixel_offset) &&
                        (bmp_len_cnt <  file_len);
assign file_sector_count = (file_len == 32'd0) ? 32'd1 : ((file_len + 32'd511) >> 9);
assign next_scan_sector_if_match = scan_sector + file_sector_count;
assign next_scan_sector_if_miss = fast_skip_valid ? (scan_sector + file_sector_count) : (scan_sector + 32'd1);

assign timeout_watch_active =
    (state == ST_SCAN)      ||
    (state == ST_LOAD_HDR)  ||
    (state == ST_LOAD_WAIT) ||
    (state == ST_LOAD_DATA);

assign timeout_progress =
    sd_sec_read_data_valid ||
    sd_sec_read_end        ||
    ((state == ST_LOAD_WAIT) && write_req_ack);

assign timeout_expired =
    timeout_watch_active &&
    (sd_timeout_cnt >= (TIMEOUT_CYCLES - 1));

assign pixel_output_now =
    bmp_data_valid &&
    (bmp_byte_idx == 2'd2);

assign pixel_count_with_current =
    pixel_count + (pixel_output_now ? 32'd1 : 32'd0);


always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_cnt <= 10'd0;
    end else if ((state == ST_SCAN) || (state == ST_LOAD_HDR)) begin
        if (sd_sec_read_data_valid)
            rd_cnt <= rd_cnt + 10'd1;
        else if (sd_sec_read_end)
            rd_cnt <= 10'd0;
    end else begin
        rd_cnt <= 10'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
            header_0     <= 8'd0;
            header_1     <= 8'd0;
            file_len     <= 32'd0;
            pixel_offset <= 32'd54;
            dib_size     <= 32'd0;
            width        <= 32'd0;
            height       <= 32'd0;
            planes       <= 16'd0;
            bit_count    <= 16'd0;
            compression  <= 32'd0;
    end else if (((state == ST_SCAN) || (state == ST_LOAD_HDR)) && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd0 : header_0 <= sd_sec_read_data;
            10'd1 : header_1 <= sd_sec_read_data;

            10'd2 : file_len[7:0] <= sd_sec_read_data;
            10'd3 : file_len[15:8] <= sd_sec_read_data;
            10'd4 : file_len[23:16] <= sd_sec_read_data;
            10'd5 : file_len[31:24] <= sd_sec_read_data;

            10'd10: pixel_offset[7:0] <= sd_sec_read_data;
            10'd11: pixel_offset[15:8] <= sd_sec_read_data;
            10'd12: pixel_offset[23:16] <= sd_sec_read_data;
            10'd13: pixel_offset[31:24] <= sd_sec_read_data;

            10'd14: dib_size[7:0]   <= sd_sec_read_data;
            10'd15: dib_size[15:8]  <= sd_sec_read_data;
            10'd16: dib_size[23:16] <= sd_sec_read_data;
            10'd17: dib_size[31:24] <= sd_sec_read_data;

            10'd18: width[7:0] <= sd_sec_read_data;
            10'd19: width[15:8] <= sd_sec_read_data;
            10'd20: width[23:16] <= sd_sec_read_data;
            10'd21: width[31:24] <= sd_sec_read_data;

            10'd22: height[7:0] <= sd_sec_read_data;
            10'd23: height[15:8] <= sd_sec_read_data;
            10'd24: height[23:16] <= sd_sec_read_data;
            10'd25: height[31:24] <= sd_sec_read_data;

            10'd26: planes[7:0]  <= sd_sec_read_data;
            10'd27: planes[15:8] <= sd_sec_read_data;

            10'd28: bit_count[7:0] <= sd_sec_read_data;
            10'd29: bit_count[15:8] <= sd_sec_read_data;

            10'd30: compression[7:0] <= sd_sec_read_data;
            10'd31: compression[15:8] <= sd_sec_read_data;
            10'd32: compression[23:16] <= sd_sec_read_data;
            10'd33: compression[31:24] <= sd_sec_read_data;
            default: ;
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_len_cnt <= 32'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (sd_sec_read_data_valid)
            bmp_len_cnt <= bmp_len_cnt + 32'd1;
    end else begin
        bmp_len_cnt <= 32'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_byte_idx <= 2'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid)
            bmp_byte_idx <= (bmp_byte_idx == 2'd2) ? 2'd0 : (bmp_byte_idx + 2'd1);
    end else begin
        bmp_byte_idx <= 2'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_data_wr_en <= 1'b0;
        bmp_data       <= 24'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid) begin
            case (bmp_byte_idx)
                2'd0: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[7:0]  <= sd_sec_read_data;
                end
                2'd1: begin
                    bmp_data_wr_en <= 1'b0;
                    bmp_data[15:8] <= sd_sec_read_data;
                end
                2'd2: begin
                    bmp_data_wr_en  <= 1'b1;
                    bmp_data[23:16] <= sd_sec_read_data;
                end
                default: begin
                    bmp_data_wr_en <= 1'b0;
                end
            endcase
        end else begin
            bmp_data_wr_en <= 1'b0;
        end
    end else begin
        bmp_data_wr_en <= 1'b0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        bmp_error         <= ERR_NONE;
        load_error        <= 1'b0;        
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
        pixel_count <= 32'd0;
    end else if (!sd_init_done) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        bmp_error         <= ERR_NONE;
        load_error        <= 1'b0;        
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
        pixel_count <= 32'd0;
    end else begin
        load_error       <= 1'b0;
        scan_found_valid <= 1'b0;

        if (timeout_expired) begin
            bmp_error    <= ERR_TIMEOUT;
            load_error   <= 1'b1;
            state        <= ST_IDLE;
            sd_sec_read  <= 1'b0;
            write_req    <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    state_code  <= 4'd1;
                    sd_sec_read <= 1'b0;
                    write_req   <= 1'b0;

                    if (scan_start) begin
                        scan_done        <= 1'b0;
                        scan_found_total <= 3'd0;
                        scan_sector      <= scan_start_sector;
                        sd_sec_read_addr <= scan_start_sector;
                        state            <= ST_SCAN;
                    end else if (load_start) begin
                        bmp_error           <= ERR_NONE;
                        pixel_count         <= 32'd0;
                        load_sector_latched <= load_sector;
                        sd_sec_read_addr    <= load_sector;
                        state               <= ST_LOAD_HDR;
                    end
                end

                ST_SCAN: begin
                    state_code  <= 4'd2;
                    sd_sec_read <= 1'b1;

                    if (sd_sec_read_end) begin
                        sd_sec_read <= 1'b0;

                        if (header_match) begin
                            scan_found_valid  <= 1'b1;
                            scan_found_sector <= scan_sector;
                            scan_found_total  <= scan_found_total + 3'd1;

                            if ((scan_found_total + 3'd1 >= scan_target_count) || (next_scan_sector_if_match > scan_max_sector)) begin
                                scan_done        <= 1'b1;
                                state            <= ST_IDLE;
                                sd_sec_read_addr <= next_scan_sector_if_match;
                                scan_sector      <= next_scan_sector_if_match;
                            end else begin
                                sd_sec_read_addr <= next_scan_sector_if_match;
                                scan_sector      <= next_scan_sector_if_match;
                            end
                        end else begin
                            if (scan_sector >= scan_max_sector) begin
                                scan_done <= 1'b1;
                                state     <= ST_IDLE;
                            end else begin
                                sd_sec_read_addr <= next_scan_sector_if_miss;
                                scan_sector      <= next_scan_sector_if_miss;
                            end
                        end
                    end
                end

                ST_LOAD_HDR: begin
                    state_code  <= 4'd2;
                    sd_sec_read <= 1'b1;

                    if (sd_sec_read_end) begin
                        sd_sec_read <= 1'b0;

                        if (header_match) begin
                            write_req        <= 1'b1;
                            sd_sec_read_addr <= load_sector_latched;
                            state            <= ST_LOAD_WAIT;
                        end else begin
                            bmp_error  <= ERR_HEADER;
                            load_error <= 1'b1;
                            state      <= ST_IDLE;
                        end
                    end
                end                 

                ST_LOAD_WAIT: begin
                    state_code <= 4'd3;

                    if (write_req_ack) begin
                        write_req <= 1'b0;
                        state     <= ST_LOAD_DATA;
                    end
                end

                ST_LOAD_DATA: begin
                    state_code  <= 4'd4;
                    sd_sec_read <= 1'b1;

                    if (pixel_output_now)
                        pixel_count <= pixel_count + 32'd1;
                        
                    if (sd_sec_read_end) begin
                        sd_sec_read <= 1'b0;

                        if ((bmp_len_cnt >= file_len) || (sd_sec_read_data_valid && ((bmp_len_cnt + 32'd1) >= file_len))) begin

                            if (pixel_count_with_current < EXPECTED_PIXELS) begin
                                bmp_error  <= ERR_TRUNCATED;
                                load_error <= 1'b1;
                            end

                            state <= ST_IDLE;
                        end else begin
                            sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                        end
                    end                
                end

                default: begin
                    state       <= ST_IDLE;
                    state_code  <= 4'd1;
                    sd_sec_read <= 1'b0;
                    write_req   <= 1'b0;
                end
            endcase
        end
    end
end

//看门狗计数器
always @(posedge clk or posedge rst) begin
    if (rst) begin
        sd_timeout_cnt <= 32'd0;
    end else if (!sd_init_done) begin
        sd_timeout_cnt <= 32'd0;
    end else if (!timeout_watch_active) begin
        sd_timeout_cnt <= 32'd0;
    end else if (timeout_progress) begin
        sd_timeout_cnt <= 32'd0;
    end else if (sd_timeout_cnt < TIMEOUT_CYCLES) begin
        sd_timeout_cnt <= sd_timeout_cnt + 32'd1;
    end
end

endmodule
