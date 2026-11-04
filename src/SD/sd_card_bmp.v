
module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4,
    parameter integer LONG_PRESS_MS      = 1000,   // KEY2长按触发时间，默认1秒
    parameter integer AUTO_PLAY_MS       = 3000,   // 自动轮播间隔，默认3秒
    parameter integer LOAD_WAIT_TIMEOUT_MS = 500, // 写入握手/整帧完成等待的无进展超时
    parameter integer SD_READ_TIMEOUT_MS = 500,   // 扫描/加载共用的扇区读取无进展超时
    parameter integer USE_BANK_WRITER = 0,        // 迁移期间默认仍使用旧写链路
    parameter integer VARIABLE_BMP_SIZE = 0,
    // Enable only with USE_BANK_WRITER=1 and VARIABLE_BMP_SIZE=1. The display
    // side must also use the dynamic scheduler before enabling in the top.
    parameter integer DYNAMIC_CACHE = 0,
    parameter integer MAX_IMAGES = 16,
    parameter integer MULTI_SONG = 0,
    parameter integer MUSIC_CUES = 0,
    parameter integer MAX_SONGS = 16,
    parameter integer IMAGE_ID_WIDTH = (MAX_IMAGES <= 1) ? 1 : $clog2(MAX_IMAGES),
    parameter integer IMAGE_COUNT_WIDTH = $clog2(MAX_IMAGES+1)
)(
    input                       clk,
    input                       rst,
    input                       key_next,
    input                       key_auto,
    input                       Sdr_init_done,
    output reg [3:0]            state_code,     // 数码管显示（复用端口，内容改为图片编号）
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output reg                  display_valid,
    output wire                 debug_sd_init_done,
    output wire                 debug_sd_read_timeout,
    output wire                 debug_scan_kicked,
    output wire                 debug_scan_done,

    input                       write_finish_toggle,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,
    // Four-bank write-path observation ports. These expose the existing
    // pixel stream and load decisions without changing the legacy handshake.
    output wire                 frame_load_start,
    output wire [1:0]           frame_load_index,
    output wire                 frame_commit,
    output wire [7:0]           frame_decode_error,
    // 以下反馈与本模块同属 clk 域，只有 USE_BANK_WRITER=1 时使用。
    input                       frame_writer_busy,
    input                       frame_writer_sector_ready,
    input                       frame_writer_done,
    input [7:0]                 frame_writer_error,
    input [3:0]                 frame_writer_image_complete,
    input                       pcm_sector_ready,
    // All playlist controls/feedback are already synchronized into clk.
    input                       pcm_repeat_all, pcm_next_track, pcm_previous_track,
    input                       pcm_playback_done, pcm_buffer_ready, pcm_buffer_cleared,
    output wire                 pcm_buffer_hold,
    output wire                 pcm_byte_valid,
    output wire [7:0]           pcm_byte,
    output wire                 pcm_file_done,
    output wire [7:0]           pcm_file_error,
    input wire                  pcm_play_clk, pcm_play_rst_n, pcm_sample_take,
    output wire                 pcm_cue_valid, pcm_cue_event,
    // 复用现有按键判断，向四 bank 播放控制器输出单周期事件。
    output reg                  player_next_event,
    output reg                  player_prev_event,
    output reg                  player_toggle_auto_event,
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO,
    // Header metadata is in clk domain, before the first decoded pixel.
    output wire                 frame_config_valid,
    output wire [15:0]          frame_width,
    output wire [15:0]          frame_height,
    output wire [15:0]          source_width, source_height,
    output wire [31:0]          frame_pixels,
    output wire                 frame_top_down,
    // Dynamic-cache job interface, all in clk domain. File ID != physical bank.
    output wire                 cache_catalog_valid,
    output wire [MAX_IMAGES-1:0] cache_catalog_mask,
    output wire [IMAGE_COUNT_WIDTH-1:0] cache_catalog_count,
    output wire [7:0]           cache_catalog_error,
    input wire                  cache_load_valid,
    output wire                 cache_load_ready,
    input wire [IMAGE_ID_WIDTH-1:0] cache_load_image_id,
    input wire [1:0]            cache_load_bank,
    output reg                  cache_load_done,
    output reg                  cache_load_success,
    output reg [7:0]            cache_load_error,
    output reg [IMAGE_ID_WIDTH-1:0] cache_job_image_id,
    // Four-phase cleanup handshake: keep req high until done, then wait for
    // done low before retry/result. The wrapper must stop the writer, retire
    // its memory burst and safely reset queued/partial CDC commands first.
    output reg                  cache_write_flush_req,
    input wire                  cache_write_flush_done,
    // A cleanup that cannot finish requires coordinated whole-player recovery;
    // NEVER release its slot or fabricate a completed load in that situation.
    output reg                  cache_recovery_required
);

wire key_next_press;
wire key_auto_press;

wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;
reg              bmp_data_wr_en;
reg [23:0]       bmp_data;
wire             sd_init_done;
wire             bmp_ready;
reg [2:0]        bmp_error;
reg              load_error;
wire             bmp_source_done;     // 文件读取成功结束，区别于空闲电平bmp_ready
wire             sized_decoder_busy, sized_decoder_finished, sized_commit;
wire             sized_pixel_valid;
wire [23:0]      sized_pixel_rgb;
wire [7:0]       sized_decoder_error;
reg              sized_file_finished_seen;
reg              scan_done;           // 保存扫描已结束状态，直到复位或重新扫描

// === FAT32接入准备：卷扫描结果（由u_scanner输出，上层使用wire）===
wire             scanner_busy;
wire             scanner_done_pulse;  // 单周期结束通知，与原scan_done保持区分
wire [7:0]       scanner_fs_error;
wire [7:0]       scanner_spc;         // 每簇扇区数
wire [31:0]      scanner_fat_start;
wire [31:0]      scanner_data_start;
wire [31:0]      scanner_max_cluster;
wire [3:0]       scan_resource_valid; // bit0~3分别对应PIC1~PIC4
wire             scanner_result_usable;
wire [31:0]      img_cluster0, img_cluster1, img_cluster2, img_cluster3;
wire [31:0]      img_size0, img_size1, img_size2, img_size3;
wire             audio_file_valid;
wire [31:0]      audio_file_cluster;
wire [31:0]      audio_file_size;
localparam integer SONG_ID_WIDTH = MAX_SONGS<=1 ? 1 : $clog2(MAX_SONGS);
wire [MAX_SONGS-1:0] song_valid_mask;
wire song_query_enable, song_query_valid;
wire [SONG_ID_WIDTH-1:0] song_query_id;
wire [31:0] song_query_cluster, song_query_size;
wire [31:0] pcm_selected_cluster, pcm_selected_size;
wire pcm_cancel;
wire [MAX_SONGS-1:0] cue_valid_mask;
wire cue_query_enable, cue_query_valid, cue_prepare, cue_ready;
wire [31:0] cue_query_cluster, cue_query_size, cue_load_cluster, cue_load_size;
wire cue_loading, cue_load_start, cue_player_ready_sd;
wire [MAX_IMAGES-1:0] catalog_image_mask;
wire [IMAGE_COUNT_WIDTH-1:0] catalog_image_count;
wire catalog_query_valid;
wire cache_job_active;
wire [31:0] catalog_query_cluster, catalog_query_size;

// 文件加载参数由本模块保存；文件字节和状态由u_streamer输出。
reg [31:0]       load_cluster;
reg [31:0]       load_file_size;
reg [1:0]        load_prep_state;      // 文件读取前，先完成SDRAM写入请求握手
reg [1:0]        write_ack_sync;       // mem_clk域的应答同步到当前clk域
reg              streamer_start_pulse;
wire             pcm_streamer_start_pulse;
reg              pcm_streamer_started;
reg              bank_load_start_pulse;
reg              writer_error_seen;
wire             frame_write_success;
reg              preload_mode;
reg [3:0]        preload_pending_mask;
wire [3:0]       preload_remaining_mask;
wire             preload_complete;
wire             streamer_busy;
wire             streamer_done_pulse; // 单周期结束通知，不直接作为bmp_ready
wire [7:0]       streamer_fs_error;
wire             streamer_byte_valid;
wire             streamer_data_sector_ready;
wire [7:0]       streamer_byte;
wire [31:0]      streamer_byte_offset;
wire             pcm_streamer_busy;
wire             pcm_streamer_done_pulse;
wire             pcm_streamer_byte_valid;
wire [7:0]       pcm_streamer_byte;
wire [31:0]      pcm_streamer_byte_offset;
wire [7:0]       pcm_streamer_fs_error;

// BMP文件头采集：沿用原bmp_read的字段，供格式校验和像素解析使用。
// bmp_header_complete只表示前54字节已收到，不表示格式已通过校验。
reg [7:0]        header_0, header_1;
reg [31:0]       file_len;
reg [31:0]       data_offset;          // 像素数据在文件中的起始字节位置
reg [31:0]       dib_size;
reg [31:0]       width, height;
reg [15:0]       planes;
reg [15:0]       bit_count;
reg [31:0]       compression;
reg              bmp_header_complete;
wire             header_match;
wire             header_fields_match;
reg              header_fields_match_q;
wire [63:0]      expected_pixel_bytes;
wire [63:0]      pixel_data_end;
reg [63:0]       pixel_data_end_q;
wire [63:0]      dib_header_end;
wire [31:0]      expected_pixels;
wire             pixel_byte_valid;
reg [1:0]        rgb_byte_cnt;         // 0=等待B，1=等待G，2=等待R
reg [31:0]       pixel_count;          // 本次加载已拼装并输出的像素数

// 扫描与加载分时使用SD卡；后续在控制逻辑中切换。
reg              use_streamer;        // 0=扫描器，1=文件流读取器
reg              scan_success;        // 扫描结束后，存在可用图片

// 扫描器的扇区接口，接到仲裁逻辑。
wire             scanner_sector_req;
wire [31:0]      scanner_sector_lba;
wire             scanner_sector_busy;
wire             scanner_sector_valid;
wire [7:0]       scanner_sector_byte;
wire [8:0]       scanner_sector_byte_index;
wire             scanner_sector_done;
wire [7:0]       scanner_sector_error;

// 文件流读取器的扇区接口，接到仲裁逻辑。
wire             streamer_sector_req;
wire [31:0]      streamer_sector_lba;
wire             streamer_sector_accept;
wire             streamer_sector_valid;
wire [7:0]       streamer_sector_byte;
wire [8:0]       streamer_sector_byte_index;
wire             streamer_sector_done;
wire [7:0]       streamer_sector_error;

wire             pcm_streamer_sector_req;
wire [31:0]      pcm_streamer_sector_lba;
wire             pcm_streamer_sector_accept;
wire             pcm_streamer_sector_valid;
wire [7:0]       pcm_streamer_sector_byte;
wire [8:0]       pcm_streamer_sector_byte_index;
wire             pcm_streamer_sector_done;
wire [7:0]       pcm_streamer_sector_error;
wire             file_arb_sector_req;
wire [31:0]      file_arb_sector_lba;
wire             file_arb_bmp_accept;
wire             file_arb_pcm_accept;
wire             file_arb_bmp_valid;
wire [7:0]       file_arb_bmp_byte;
wire [8:0]       file_arb_bmp_byte_index;
wire             file_arb_bmp_done;
wire [7:0]       file_arb_bmp_error;
wire             file_arb_pcm_valid;
wire [7:0]       file_arb_pcm_byte;
wire [8:0]       file_arb_pcm_byte_index;
wire             file_arb_pcm_done;
wire [7:0]       file_arb_pcm_error;

// 仲裁逻辑与sd_sector_adapter之间的接口。
// req和lba将由always @(*)组合逻辑赋值，reg在这里不表示时序寄存器。
reg              arb_sector_req;
reg [31:0]       arb_sector_lba;
wire             arb_sector_busy;
wire             arb_sector_valid;
wire [7:0]       arb_sector_byte;
wire [8:0]       arb_sector_byte_index;
wire             arb_sector_done;
wire [7:0]       arb_sector_error;

reg              scan_start_pulse;
reg              load_start_pulse;
reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
reg [31:0]       auto_cnt;
reg [2:0]        img_found_count;
reg [3:0]        img_valid_mask;       // 保存本次扫描找到的文件编号，避免丢失缺号信息
reg [1:0]        img_idx;              // 当前真正显示中的图片编号
reg [1:0]        load_idx;             // 当前正在写入的图片编号
reg [1:0]        pending_buf_idx;      // 当前正在写入的目标缓冲区
reg              next_req_pending;
reg              load_busy;
reg              skip_on_failure;      //加载失败执行跳过的决策：0：来源为手动按键，失败两次后执行停止，保留当前显示 1：来源为首图或自动轮播，失败两次后执行自动跳到下一张

// 分别保存文件读取成功和SDRAM整帧写完通知，允许两者按任意顺序到达。
reg              source_done_seen;
reg              write_done_seen;

reg [31:0]       load_wait_cnt;
wire             load_wait_active;
wire             load_wait_progress;
wire             load_wait_expired;

reg              load_error_seen;
reg [1:0]        retry_count;

// 同步 mem_clk 域的 write_finish_toggle
reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;

wire [31:0] auto_play_cycles;
wire auto_tick;
wire [1:0] first_valid_idx;
wire [1:0] next_from_current;
wire [1:0] prev_from_current;
wire [1:0] next_from_loading;
wire [2:0] next_preload_selection;

// === 新增：KEY2长按检测相关寄存器 ===
wire        key_auto_stable;      // KEY2消抖后的稳定状态（1=松开，0=按下）
reg         key_auto_stable_old;  // 用于识别松开瞬间
reg [31:0]  long_press_cnt;       // 长按计数器
reg         long_press_marked;     // 长按已触发标记（防止按住不放重复触发）
reg         prev_req_pending;      // 上一张请求挂起标志（和next_req_pending对称）

// 与fat32_volume_scanner的错误码对应；22表示只找到了部分目标文件。
localparam [7:0] FS_ERR_NONE             = 8'h00;
localparam [7:0] FS_ERR_RESOURCE_MISSING = 8'h22;
// 保留原bmp_read的错误编号；新增4表示文件系统/簇链读取失败。
localparam [2:0] ERR_NONE      = 3'd0;
localparam [2:0] ERR_HEADER    = 3'd1;
localparam [2:0] ERR_TIMEOUT   = 3'd2;
localparam [2:0] ERR_TRUNCATED = 3'd3;
localparam [2:0] ERR_FILE      = 3'd4;
localparam [2:0] ERR_WRITE     = 3'd5;
localparam [1:0] PREP_IDLE         = 2'd0;
localparam [1:0] PREP_WAIT_READY   = 2'd1;
localparam [1:0] PREP_WAIT_ACK     = 2'd2;
localparam [1:0] PREP_WAIT_RELEASE = 2'd3;
localparam integer LOAD_WAIT_TIMEOUT_CYCLES =
    ((CLK_FREQ_HZ / 1000) * LOAD_WAIT_TIMEOUT_MS > 0) ?
     (CLK_FREQ_HZ / 1000) * LOAD_WAIT_TIMEOUT_MS : 1;

assign write_req  = !USE_BANK_WRITER && (load_prep_state == PREP_WAIT_ACK);
// After a writer failure, drain the file but stop delivering pixels to the
// inactive loader. This also preserves its original error instead of 0x41.
assign write_en   = (VARIABLE_BMP_SIZE ? sized_pixel_valid : bmp_data_wr_en) &&
                    !(DYNAMIC_CACHE && cache_write_flush_req) &&
                    (!USE_BANK_WRITER ||
                     (frame_writer_busy && frame_writer_error == 8'd0 &&
                      !writer_error_seen));
assign write_data = {VARIABLE_BMP_SIZE ? sized_pixel_rgb : bmp_data, 8'b0};
assign frame_load_start = USE_BANK_WRITER ? bank_load_start_pulse : load_start_pulse;
assign frame_load_index = DYNAMIC_CACHE ? pending_buf_idx : load_idx;
assign frame_commit = bmp_source_done;
assign frame_decode_error = (VARIABLE_BMP_SIZE && sized_decoder_error != 0) ?
                            sized_decoder_error : {5'd0, bmp_error};
// Failed loads must still reach file_done before the existing retry path can
// run. Bypass capacity after a latched error; write_en above discards pixels.
assign streamer_data_sector_ready = !USE_BANK_WRITER ||
                                    frame_writer_sector_ready || writer_error_seen;
assign debug_sd_init_done = sd_init_done;
assign debug_scan_kicked  = scan_kicked;
assign debug_scan_done    = scan_done;
assign auto_play_cycles = (CLK_FREQ_HZ / 1000) * AUTO_PLAY_MS;
assign auto_tick = (auto_cnt == (auto_play_cycles - 1));
assign first_valid_idx = first_index_valid(img_valid_mask);
assign next_from_current = next_index_valid(img_idx, img_valid_mask);
assign prev_from_current = prev_index_valid(img_idx, img_valid_mask);
assign next_from_loading = next_index_valid(load_idx, img_valid_mask);
assign preload_remaining_mask = preload_pending_mask & ~frame_writer_image_complete;
assign next_preload_selection = next_preload_index(load_idx, preload_remaining_mask);
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
// load_done 也可能表示失败退出，必须同时检查错误；成功只接受本次反馈。
assign frame_write_success = USE_BANK_WRITER ?
    (frame_writer_done && (frame_writer_error == 8'd0) && !writer_error_seen) :
    write_finish_pulse;
assign preload_complete = !USE_BANK_WRITER ||
                          (!preload_mode && (preload_pending_mask == 4'd0));
assign scanner_result_usable = DYNAMIC_CACHE ?
    ((|catalog_image_mask) && scanner_fs_error == FS_ERR_NONE) :
    ((scan_resource_valid != 4'd0) &&
     ((scanner_fs_error == FS_ERR_NONE) ||
      (scanner_fs_error == FS_ERR_RESOURCE_MISSING)));
assign cache_catalog_valid = DYNAMIC_CACHE && scan_done;
assign cache_catalog_mask = DYNAMIC_CACHE ? catalog_image_mask : 0;
assign cache_catalog_count = DYNAMIC_CACHE ? catalog_image_count : 0;
assign cache_catalog_error = DYNAMIC_CACHE ? scanner_fs_error : 0;
assign cache_load_ready = DYNAMIC_CACHE && sd_init_done && scan_success &&
    !cache_job_active && !load_busy && !load_start_pulse && !streamer_start_pulse && !streamer_busy &&
    !sized_decoder_busy && (load_prep_state == PREP_IDLE) &&
    !cache_write_flush_req && !cache_write_flush_done &&
    !cache_load_done && !cache_recovery_required && !frame_writer_busy;
// ready只表示可以接收新任务；成功结束必须另外检查文件流和BMP头。
assign bmp_ready = (load_prep_state == PREP_IDLE) && !streamer_start_pulse &&
                   !streamer_busy && !arb_sector_busy && !load_start_pulse &&
                   (!VARIABLE_BMP_SIZE || !sized_decoder_busy);
assign bmp_source_done = VARIABLE_BMP_SIZE ? (load_busy && sized_commit) :
                         (load_busy && streamer_done_pulse && header_match &&
                         (streamer_fs_error == FS_ERR_NONE) &&
                         (pixel_count == expected_pixels) && (rgb_byte_cnt == 2'd0));

generate
    if (VARIABLE_BMP_SIZE) begin : g_sized_bmp
        bmp_cache_preprocessor u_decoder (
            .clk(clk), .rst_n(!rst && sd_init_done &&
                             !(DYNAMIC_CACHE && cache_write_flush_req)),
            // Clear a previous decoder error BEFORE the loader begins a retry.
            .start(load_start_pulse), .directory_size(load_file_size),
            .byte_valid(streamer_byte_valid), .byte_data(streamer_byte),
            .byte_offset(streamer_byte_offset), .file_done(streamer_done_pulse),
            .file_error(streamer_fs_error), .busy(sized_decoder_busy),
            .finished(sized_decoder_finished), .frame_commit(sized_commit),
            .error_code(sized_decoder_error), .config_valid(frame_config_valid),
            .frame_width(frame_width), .frame_height(frame_height),
            .source_width(source_width), .source_height(source_height),
            .frame_pixels(frame_pixels), .frame_top_down(frame_top_down),
            .pixel_valid(sized_pixel_valid), .pixel_rgb(sized_pixel_rgb)
        );
    end else begin : g_fixed_bmp_metadata
        assign sized_decoder_busy = 1'b0;
        assign sized_decoder_finished = 1'b0;
        assign sized_commit = 1'b0;
        assign sized_decoder_error = 8'd0;
        assign sized_pixel_valid = 1'b0;
        assign sized_pixel_rgb = 24'd0;
        assign frame_config_valid = 1'b0;
        assign frame_width = bmp_width;
        assign frame_height = bmp_height;
        assign source_width = bmp_width;
        assign source_height = bmp_height;
        assign frame_pixels = expected_pixels;
        assign frame_top_down = 1'b0;
    end
endgenerate

always @(posedge clk or posedge rst) begin
    if (rst) sized_file_finished_seen <= 1'b0;
    else if (!sd_init_done || load_start_pulse) sized_file_finished_seen <= 1'b0;
    else if (sized_decoder_finished) sized_file_finished_seen <= 1'b1;
end

// 这里只监视没有SD读取任务在执行的两段等待，允许安全退出后重试。
// 读卡中途的超时必须连同底层驱动一起恢复，不能在此直接丢弃请求。
assign load_wait_active = load_busy && !load_start_pulse && !load_error &&
                          !(DYNAMIC_CACHE && cache_write_flush_req) &&
                          !streamer_busy && (DYNAMIC_CACHE || !arb_sector_busy) &&
                          ((load_prep_state != PREP_IDLE) ||
                           (source_done_seen && !write_done_seen));
// 使用本拍能推进状态的条件，保证应答恰好在截止时到达也优先于超时。
assign load_wait_progress =
    ((load_prep_state == PREP_WAIT_READY) && Sdr_init_done &&
     (USE_BANK_WRITER ? !frame_writer_busy : !write_ack_sync[1])) ||
    ((load_prep_state == PREP_WAIT_ACK) &&
     (USE_BANK_WRITER ? frame_writer_busy : write_ack_sync[1])) ||
    (!USE_BANK_WRITER && (load_prep_state == PREP_WAIT_RELEASE) && !write_ack_sync[1]) ||
    streamer_start_pulse || frame_write_success;
assign load_wait_expired = load_wait_active && !load_wait_progress &&
                          (load_wait_cnt >= LOAD_WAIT_TIMEOUT_CYCLES - 1);

// 当前640x480、24位BMP每行640*3=1920字节，无行末填充。
// 先扩展位宽再运算，避免异常偏移量或尺寸使32位加乘法溢出。
assign expected_pixel_bytes = {48'd0, bmp_width} *
                              {48'd0, bmp_height} * 64'd3;
assign expected_pixels = {16'd0, bmp_width} * {16'd0, bmp_height};
assign pixel_data_end = {32'd0, data_offset} + expected_pixel_bytes;
assign dib_header_end = 64'd14 + {32'd0, dib_size};
// 头字段在偏移33后已收齐，偏移34..53不再修改这些字段。
// 利用这段时间提前寄存像素区终点和校验结果，避免每个像素经过
// “宽位加法 -> 大小比较 -> 全头校验 -> 输出使能”的长组合路径。
// 完整头标记仍在偏移53到达时置位，连续到来的偏移54像素字节不会漏掉。
always @(posedge clk or posedge rst) begin
    if (rst) begin
        pixel_data_end_q <= 64'd0;
        header_fields_match_q <= 1'b0;
    end else if (!sd_init_done || load_start_pulse) begin
        pixel_data_end_q <= 64'd0;
        header_fields_match_q <= 1'b0;
    end else begin
        pixel_data_end_q <= pixel_data_end;
        header_fields_match_q <= header_fields_match;
    end
end
// 只接受头校验通过后的像素区；BMP头、扩展头及文件尾附加字节不送入FIFO。
assign pixel_byte_valid = load_busy && header_match && streamer_byte_valid &&
                          (streamer_byte_offset >= data_offset) &&
                          ({32'd0, streamer_byte_offset} < pixel_data_end_q);

// 收齐基础头信息后，恢复原bmp_read的格式要求并核对目录中的文件大小。
// file_len允许小于目录大小（文件末尾可能有附加字节），但必须容纳完整像素。
assign header_match = bmp_header_complete && header_fields_match_q;
assign header_fields_match = (header_0 == "B") && (header_1 == "M") &&
                      (dib_size >= 32'd40) &&
                      (bmp_width != 16'd0) && (bmp_height != 16'd0) &&
                      (width == {16'd0, bmp_width}) &&
                      (height == {16'd0, bmp_height}) &&
                      (planes == 16'd1) &&
                      (bit_count == 16'd24) &&
                      (compression == 32'd0) &&
                      (data_offset >= 32'd54) &&
                      ({32'd0, data_offset} >= dib_header_end) &&
                      (data_offset < file_len) &&
                      ({32'd0, file_len} >= pixel_data_end_q) &&
                      (file_len <= load_file_size);

// 扇区读仲裁：扫描与加载分时使用同一个适配层。
// use_streamer只能在当前扇区读取结束后切换。
always @(*) begin
    if (use_streamer) begin
        arb_sector_req = file_arb_sector_req;
        arb_sector_lba = file_arb_sector_lba;
    end else begin
        arb_sector_req = scanner_sector_req;
        arb_sector_lba = scanner_sector_lba;
    end
end

// 未选中的模块收到busy=1，等待获得SD卡使用权；其余返回信号置0。
assign scanner_sector_busy       = use_streamer ? 1'b1 : arb_sector_busy;
assign scanner_sector_valid      = use_streamer ? 1'b0 : arb_sector_valid;
assign scanner_sector_byte       = use_streamer ? 8'd0 : arb_sector_byte;
assign scanner_sector_byte_index = use_streamer ? 9'd0 : arb_sector_byte_index;
assign scanner_sector_done       = use_streamer ? 1'b0 : arb_sector_done;
assign scanner_sector_error      = use_streamer ? 8'd0 : arb_sector_error;

assign streamer_sector_valid      = use_streamer ? file_arb_bmp_valid : 1'b0;
assign streamer_sector_byte       = use_streamer ? file_arb_bmp_byte : 8'd0;
assign streamer_sector_byte_index = use_streamer ? file_arb_bmp_byte_index : 9'd0;
assign streamer_sector_done       = use_streamer ? file_arb_bmp_done : 1'b0;
assign streamer_sector_error      = use_streamer ? file_arb_bmp_error : 8'd0;
assign streamer_sector_accept     = use_streamer ? file_arb_bmp_accept : 1'b0;

assign pcm_streamer_sector_valid      = use_streamer ? file_arb_pcm_valid : 1'b0;
assign pcm_streamer_sector_byte       = use_streamer ? file_arb_pcm_byte : 8'd0;
assign pcm_streamer_sector_byte_index = use_streamer ? file_arb_pcm_byte_index : 9'd0;
assign pcm_streamer_sector_done       = use_streamer ? file_arb_pcm_done : 1'b0;
assign pcm_streamer_sector_error      = use_streamer ? file_arb_pcm_error : 8'd0;
assign pcm_streamer_sector_accept     = use_streamer ? file_arb_pcm_accept : 1'b0;
assign pcm_byte_valid = pcm_streamer_byte_valid && !cue_loading;
assign pcm_byte = pcm_streamer_byte;
assign pcm_file_done = pcm_streamer_done_pulse && !cue_loading;
assign pcm_file_error = cue_loading ? 8'd0 : pcm_streamer_fs_error;

generate if(MUSIC_CUES && MULTI_SONG && DYNAMIC_CACHE) begin : g_music_cues
    reg prepare_old;
    wire event_toggle, event_sync, player_ready;
    reg event_old;
    always @(posedge clk or posedge rst) begin
        if(rst) begin prepare_old<=0;event_old<=0;end
        else begin prepare_old<=cue_prepare;event_old<=event_sync;end
    end
    music_cue_loader u_loader (
        .clk(clk),.rst_n(~rst),.prepare(cue_prepare),
        .cue_present(cue_valid_mask[song_query_id]),.query_enable(cue_query_enable),
        .query_valid(cue_query_valid),.query_cluster(cue_query_cluster),.query_size(cue_query_size),
        .stream_busy(pcm_streamer_busy),.stream_done(pcm_streamer_done_pulse),
        .load_start(cue_load_start),.load_cluster(cue_load_cluster),.load_size(cue_load_size),
        .loading(cue_loading),.ready(cue_ready)
    );
    music_cue_table u_table (
        .clk(clk),.rst_n(~rst),.prepare(cue_prepare && !prepare_old),
        .load_start(cue_load_start),.file_size(cue_load_size),.pcm_samples(pcm_selected_size>>1),
        .byte_valid(pcm_streamer_byte_valid && cue_loading),.byte_data(pcm_streamer_byte),
        .byte_offset(pcm_streamer_byte_offset),.file_done(pcm_streamer_done_pulse && cue_loading),
        .file_error(pcm_streamer_fs_error),.table_valid(pcm_cue_valid),.cue_count(),
        .play_clk(pcm_play_clk),.play_rst_n(pcm_play_rst_n),.sample_take(pcm_sample_take),
        .event_toggle(event_toggle),.play_ready(player_ready)
    );
    sync_2ff u_event_sync(.clk(clk),.rst_n(~rst),.async_in(event_toggle),.sync_out(event_sync));
    sync_2ff u_ready_sync(.clk(clk),.rst_n(~rst),.async_in(player_ready),.sync_out(cue_player_ready_sd));
    assign pcm_cue_event=(event_sync!=event_old) && pcm_cue_valid && !pcm_buffer_hold;
end else begin : g_no_music_cues
    assign cue_query_enable=0;
    assign cue_load_start=0;
    assign cue_load_cluster=0;
    assign cue_load_size=0;
    assign cue_loading=0;
    assign cue_ready=1;
    assign cue_player_ready_sd=1;
    assign pcm_cue_valid=0;
    assign pcm_cue_event=0;
end endgenerate

// Both policies share the streamer/ring. The playlist waits for audible EOF;
// legacy clients retain one-shot SONG1 playback. A full ring pauses reads at
// sector boundaries without disturbing pictures or an accepted SD transaction.
generate if (MULTI_SONG) begin : g_playlist
    pcm_playlist_controller #(.MAX_SONGS(MAX_SONGS),
        .ENABLE_CUES(MUSIC_CUES && DYNAMIC_CACHE)) u_playlist (
        .clk(clk), .rst_n(~rst), .enable(sd_init_done && use_streamer && scan_done),
        .song_mask(song_valid_mask), .repeat_all(pcm_repeat_all), .next_track(pcm_next_track),
        .previous_track(pcm_previous_track),
        .playback_done(pcm_playback_done), .stream_busy(pcm_streamer_busy),
        .stream_error(pcm_streamer_fs_error), .stream_cancel(pcm_cancel),
        .stream_start(pcm_streamer_start_pulse), .stream_cluster(pcm_selected_cluster),
        .stream_size(pcm_selected_size), .buffer_hold(pcm_buffer_hold),
        .buffer_ready(pcm_buffer_ready && (!pcm_cue_valid || cue_player_ready_sd)), .buffer_cleared(pcm_buffer_cleared),
        .query_enable(song_query_enable), .query_id(song_query_id),
        .query_valid(song_query_valid), .query_cluster(song_query_cluster),
        .query_size(song_query_size), .current_id(), .cue_prepare(cue_prepare),.cue_ready(cue_ready)
    );
end else begin : g_single_song
reg start_pulse;
assign pcm_streamer_start_pulse=start_pulse;
assign pcm_selected_cluster=audio_file_cluster;
assign pcm_selected_size=audio_file_size;
assign pcm_buffer_hold=1'b0;
assign pcm_cancel=1'b0;
assign cue_prepare=1'b0;
assign song_query_enable=1'b0;
assign song_query_id=0;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        start_pulse <= 1'b0;
        pcm_streamer_started <= 1'b0;
    end else begin
        start_pulse <= 1'b0;
        if (!sd_init_done)
            pcm_streamer_started <= 1'b0;
        else if (use_streamer && scan_done && audio_file_valid &&
                 !pcm_streamer_started && !pcm_streamer_busy) begin
            start_pulse <= 1'b1;
            pcm_streamer_started <= 1'b1;
        end
    end
end
end endgenerate

// 文件字节位置从0开始；多字节字段按BMP的小端序分别写入对应位段。
always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_0            <= 8'd0;
        header_1            <= 8'd0;
        file_len            <= 32'd0;
        data_offset         <= 32'd0;
        dib_size            <= 32'd0;
        width               <= 32'd0;
        height              <= 32'd0;
        planes              <= 16'd0;
        bit_count           <= 16'd0;
        compression         <= 32'd0;
        bmp_header_complete <= 1'b0;
    end else if (!sd_init_done || load_start_pulse) begin
        // 每次加载（包括重试）重新采集，避免沿用上一张图片的头信息。
        header_0            <= 8'd0;
        header_1            <= 8'd0;
        file_len            <= 32'd0;
        data_offset         <= 32'd0;
        dib_size            <= 32'd0;
        width               <= 32'd0;
        height              <= 32'd0;
        planes              <= 16'd0;
        bit_count           <= 16'd0;
        compression         <= 32'd0;
        bmp_header_complete <= 1'b0;
    end else if (load_busy && !bmp_header_complete && streamer_byte_valid) begin
        case (streamer_byte_offset)
            32'd0 : header_0 <= streamer_byte;
            32'd1 : header_1 <= streamer_byte;

            32'd2 : file_len[7:0]   <= streamer_byte;
            32'd3 : file_len[15:8]  <= streamer_byte;
            32'd4 : file_len[23:16] <= streamer_byte;
            32'd5 : file_len[31:24] <= streamer_byte;

            32'd10: data_offset[7:0]   <= streamer_byte;
            32'd11: data_offset[15:8]  <= streamer_byte;
            32'd12: data_offset[23:16] <= streamer_byte;
            32'd13: data_offset[31:24] <= streamer_byte;

            32'd14: dib_size[7:0]   <= streamer_byte;
            32'd15: dib_size[15:8]  <= streamer_byte;
            32'd16: dib_size[23:16] <= streamer_byte;
            32'd17: dib_size[31:24] <= streamer_byte;

            32'd18: width[7:0]   <= streamer_byte;
            32'd19: width[15:8]  <= streamer_byte;
            32'd20: width[23:16] <= streamer_byte;
            32'd21: width[31:24] <= streamer_byte;

            32'd22: height[7:0]   <= streamer_byte;
            32'd23: height[15:8]  <= streamer_byte;
            32'd24: height[23:16] <= streamer_byte;
            32'd25: height[31:24] <= streamer_byte;

            32'd26: planes[7:0]  <= streamer_byte;
            32'd27: planes[15:8] <= streamer_byte;

            32'd28: bit_count[7:0]  <= streamer_byte;
            32'd29: bit_count[15:8] <= streamer_byte;

            32'd30: compression[7:0]   <= streamer_byte;
            32'd31: compression[15:8]  <= streamer_byte;
            32'd32: compression[23:16] <= streamer_byte;
            32'd33: compression[31:24] <= streamer_byte;

            32'd53: bmp_header_complete <= 1'b1;
            default: ;
        endcase
    end
end

// BMP的文件字节顺序为B、G、R。每收到第3个字节，输出一个完整RGB像素。
// 无有效字节时保留拼装进度，跨扇区、跨簇读取或字节间停顿都不会打乱颜色。
always @(posedge clk or posedge rst) begin
    if (rst) begin
        rgb_byte_cnt   <= 2'd0;
        pixel_count    <= 32'd0;
        bmp_data       <= 24'd0;
        bmp_data_wr_en <= 1'b0;
    end else if (!sd_init_done || load_start_pulse) begin
        rgb_byte_cnt   <= 2'd0;
        pixel_count    <= 32'd0;
        bmp_data       <= 24'd0;
        bmp_data_wr_en <= 1'b0;
    end else begin
        bmp_data_wr_en <= 1'b0;
        if (pixel_byte_valid) begin
            case (rgb_byte_cnt)
                2'd0: begin
                    bmp_data[7:0] <= streamer_byte;
                    rgb_byte_cnt  <= 2'd1;
                end
                2'd1: begin
                    bmp_data[15:8] <= streamer_byte;
                    rgb_byte_cnt   <= 2'd2;
                end
                2'd2: begin
                    bmp_data[23:16] <= streamer_byte;
                    rgb_byte_cnt    <= 2'd0;
                    bmp_data_wr_en  <= 1'b1;
                    pixel_count     <= pixel_count + 32'd1;
                end
                default: rgb_byte_cnt <= 2'd0;
            endcase
        end
    end
end

// write_req_ack由mem_clk域产生，先同步后再用于当前域的握手判断。
always @(posedge clk or posedge rst) begin
    if (rst)
        write_ack_sync <= 2'b00;
    else
        write_ack_sync <= {write_ack_sync[0], write_req_ack};
end

// 计的是连续无进展时间；状态推进或完成通知到达时重新计时。
always @(posedge clk or posedge rst) begin
    if (rst) begin
        load_wait_cnt <= 32'd0;
    end else if (!sd_init_done) begin
        load_wait_cnt <= 32'd0;
    end else begin
        if (!load_wait_active || load_wait_progress || load_wait_expired)
            load_wait_cnt <= 32'd0;
        else
            load_wait_cnt <= load_wait_cnt + 32'd1;
    end
end

// 文件流只在数据扇区之间暂停，当前扇区的字节必须完整接收。
// 先准备写入FIFO，再启动文件读取；后续扇区由loader容量控制。
// 旧链路：ack=1期间FIFO复位，等req和ack均撤销后才读文件。
// 四 bank：等loader空闲后发load_start，看到load_busy确认接受后才读文件。
// 此处只准备目标缓冲区，后续像素输出仍须通过header_match校验。
always @(posedge clk or posedge rst) begin
    if (rst) begin
        load_prep_state      <= PREP_IDLE;
        streamer_start_pulse <= 1'b0;
        bank_load_start_pulse <= 1'b0;
    end else if (!sd_init_done || load_wait_expired ||
                 (DYNAMIC_CACHE && cache_write_flush_req)) begin
        load_prep_state      <= PREP_IDLE;
        streamer_start_pulse <= 1'b0;
        bank_load_start_pulse <= 1'b0;
    end else begin
        streamer_start_pulse <= 1'b0;
        bank_load_start_pulse <= 1'b0;
        case (load_prep_state)
            PREP_IDLE: begin
                if (load_start_pulse && !streamer_start_pulse &&
                    !streamer_busy && (DYNAMIC_CACHE || !arb_sector_busy))
                    load_prep_state <= PREP_WAIT_READY;
            end
            PREP_WAIT_READY: begin
                if (USE_BANK_WRITER) begin
                    if (Sdr_init_done && !frame_writer_busy) begin
                        bank_load_start_pulse <= 1'b1;
                        load_prep_state <= PREP_WAIT_ACK;
                    end
                end else if (Sdr_init_done && !write_ack_sync[1])
                    load_prep_state <= PREP_WAIT_ACK;
            end
            PREP_WAIT_ACK: begin
                if (USE_BANK_WRITER) begin
                    if (frame_writer_busy) begin
                        streamer_start_pulse <= 1'b1;
                        load_prep_state <= PREP_IDLE;
                    end
                end else if (write_ack_sync[1])
                    load_prep_state <= PREP_WAIT_RELEASE;
            end
            PREP_WAIT_RELEASE: begin
                if (!write_ack_sync[1]) begin
                    streamer_start_pulse <= 1'b1;
                    load_prep_state      <= PREP_IDLE;
                end
            end
            default: load_prep_state <= PREP_IDLE;
        endcase
    end
end

// 文件流不能中途取消。先记住写侧错误，等文件流停下再让原有重试逻辑处理。
// PREP阶段不接收旧错误，loader接受新的load_start时会清除上一次错误码。
always @(posedge clk or posedge rst) begin
    if (rst)
        writer_error_seen <= 1'b0;
    else if (!sd_init_done || load_start_pulse)
        writer_error_seen <= 1'b0;
    else if (USE_BANK_WRITER && load_busy && (load_prep_state == PREP_IDLE) &&
             !streamer_start_pulse && (frame_writer_error != 8'd0))
        writer_error_seen <= 1'b1;
end

// 文件流没有中途取消接口：头错误也要等本次file_done后才通知上层重试，
// 避免流读取器仍忙时收到下一次单周期file_start，漏掉重试请求。
// file_done也可能表示失败，因此不能直接作为加载成功通知。
always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_error  <= ERR_NONE;
        load_error <= 1'b0;
    end else if (!sd_init_done || load_start_pulse) begin
        bmp_error  <= ERR_NONE;
        load_error <= 1'b0;
    end else begin
        load_error <= 1'b0;
        if (load_wait_expired) begin
            bmp_error  <= ERR_TIMEOUT;
            load_error <= 1'b1;
        end else if (VARIABLE_BMP_SIZE && load_busy && sized_decoder_finished &&
                     sized_decoder_error != 0) begin
            case (sized_decoder_error)
                8'd1: bmp_error <= ERR_HEADER;
                8'd2: bmp_error <= ERR_TRUNCATED;
                default: bmp_error <= ERR_FILE;
            endcase
            load_error <= 1'b1;
        end else if (USE_BANK_WRITER && load_busy && writer_error_seen &&
                     (!VARIABLE_BMP_SIZE || sized_file_finished_seen) &&
                     !load_error && !load_error_seen &&
                     !streamer_busy && (DYNAMIC_CACHE || !arb_sector_busy) && !streamer_start_pulse &&
                     (load_prep_state == PREP_IDLE)) begin
            bmp_error  <= ERR_WRITE;
            load_error <= 1'b1;
        end else if (!VARIABLE_BMP_SIZE && load_busy && streamer_done_pulse) begin
            if (streamer_fs_error != FS_ERR_NONE) begin
                bmp_error  <= ERR_FILE;
                load_error <= 1'b1;
            end else if (!bmp_header_complete) begin
                bmp_error  <= ERR_TRUNCATED;
                load_error <= 1'b1;
            end else if (!header_match) begin
                bmp_error  <= ERR_HEADER;
                load_error <= 1'b1;
            end else if ((pixel_count != expected_pixels) || (rgb_byte_cnt != 2'd0)) begin
                bmp_error  <= ERR_TRUNCATED;
                load_error <= 1'b1;
            end
        end
    end
end

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_next),
    .press_pulse(key_next_press),
    .button_stable_out()
);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_auto (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_auto),
    .press_pulse(key_auto_press),
    .button_stable_out(key_auto_stable)   // 新增：连接消抖后的稳定状态
);

// 首图选择编号最小的已找到文件；空掩码返回0，加载条件会阻止启动。
function [1:0] first_index_valid;
    input [3:0] valid_mask;
    begin
        if (valid_mask[0])
            first_index_valid = 2'd0;
        else if (valid_mask[1])
            first_index_valid = 2'd1;
        else if (valid_mask[2])
            first_index_valid = 2'd2;
        else if (valid_mask[3])
            first_index_valid = 2'd3;
        else
            first_index_valid = 2'd0;
    end
endfunction

// 按PIC1~PIC4顺序循环找下一张，跳过缺失编号。
function [1:0] next_index_valid;
    input [1:0] cur;
    input [3:0] valid_mask;
    reg [1:0] next1, next2, next3;
    begin
        // 三个候选直接由 cur 并行算出，避免“查掩码 -> 改候选 -> 再查”串联。
        // 选择顺序、空掩码回到 cur，以及 2 位编号的回绕均与原来一致。
        next1 = cur + 2'd1;
        next2 = cur + 2'd2;
        next3 = cur + 2'd3;
        if (valid_mask[next1])
            next_index_valid = next1;
        else if (valid_mask[next2])
            next_index_valid = next2;
        else if (valid_mask[next3])
            next_index_valid = next3;
        else
            next_index_valid = cur;
    end
endfunction

// 从当前编号之后循环找待加载项；最高位表示是否找到，低两位是编号。
function [2:0] next_preload_index;
    input [1:0] cur;
    input [3:0] pending_mask;
    begin
        next_preload_index = {|pending_mask, next_index_valid(cur, pending_mask)};
    end
endfunction

// FAT32图片选择：同一个图片编号分别查起始簇和文件大小（字节）。
function [31:0] cluster_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: cluster_lut = img_cluster0;
            2'd1: cluster_lut = img_cluster1;
            2'd2: cluster_lut = img_cluster2;
            default: cluster_lut = img_cluster3;
        endcase
    end
endfunction

function [31:0] size_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: size_lut = img_size0;
            2'd1: size_lut = img_size1;
            2'd2: size_lut = img_size2;
            default: size_lut = img_size3;
        endcase
    end
endfunction

function [1:0] next_buf_lut;
    input [1:0] cur_disp_buf;
    input       valid_now;
    begin
        if (!valid_now)
            next_buf_lut = 2'd0;                 // 首图固定写 buffer0
        else if (cur_disp_buf == 2'd0)
            next_buf_lut = 2'd1;
        else
            next_buf_lut = 2'd0;
    end
endfunction

// 与下一张对称，按反方向循环找已找到的文件。
function [1:0] prev_index_valid;
    input [1:0] cur;
    input [3:0] valid_mask;
    reg [1:0] candidate;
    begin
        candidate = cur - 2'd1;
        if (!valid_mask[candidate])
            candidate = candidate - 2'd1;
        if (!valid_mask[candidate])
            candidate = candidate - 2'd1;
        if (!valid_mask[candidate])
            candidate = candidate - 2'd1;
        prev_index_valid = candidate;
    end
endfunction

generate if (DYNAMIC_CACHE) begin : g_cache_control
    localparam [2:0] JOB_IDLE=0, JOB_RUN=1, JOB_FLUSH=2, JOB_RELEASE=3, JOB_QUERY=4;
    reg [2:0] job_state;
    reg [31:0] cleanup_wait;
    assign cache_job_active = job_state != JOB_IDLE;

    always @(posedge clk or posedge rst) begin
        if (rst || !sd_init_done) begin
            job_state <= JOB_IDLE;
            cleanup_wait <= 0;
            cache_load_done <= 0;
            cache_load_success <= 0;
            cache_load_error <= 0;
            cache_job_image_id <= 0;
            cache_write_flush_req <= 0;
            cache_recovery_required <= 0;
            load_cluster <= 0;
            load_file_size <= 0;
            load_idx <= 0;
            pending_buf_idx <= 0;
            write_buf_idx <= 0;
            disp_buf_idx <= 0;
            load_busy <= 0;
            load_start_pulse <= 0;
            use_streamer <= 0;
            scan_success <= 0;
            scan_done <= 0;
            scan_kicked <= 0;
            scan_start_pulse <= 0;
            source_done_seen <= 0;
            write_done_seen <= 0;
            load_error_seen <= 0;
            retry_count <= 0;
            first_image_committed <= 0;
            display_valid <= 0;
            state_code <= 1;
            img_idx <= 0;
            img_found_count <= 0;
            img_valid_mask <= 0;
            // Legacy-only policy state is inactive in this generate branch.
            preload_mode <= 0;
            preload_pending_mask <= 0;
            next_req_pending <= 0;
            prev_req_pending <= 0;
            skip_on_failure <= 0;
            auto_play_en <= 0;
            auto_cnt <= 0;
            wrfin_tgl_sync <= 0;
            player_next_event <= 0;
            player_prev_event <= 0;
            player_toggle_auto_event <= 0;
            key_auto_stable_old <= 1;
            long_press_cnt <= 0;
            long_press_marked <= 0;
        end else begin
            load_start_pulse <= 0;
            scan_start_pulse <= 0;
            cache_load_done <= 0;
            player_next_event <= 0;
            player_prev_event <= 0;
            player_toggle_auto_event <= 0;
            key_auto_stable_old <= key_auto_stable;

            if (!scan_kicked && Sdr_init_done && !scanner_busy &&
                !streamer_busy && !arb_sector_busy) begin
                scan_kicked <= 1;
                scan_start_pulse <= 1;
            end
            if (scanner_done_pulse) begin
                scan_done <= 1;
                scan_success <= scanner_result_usable;
                // A valid audio-only volume can still feed the PCM path.
                use_streamer <= scanner_result_usable ||
                    (scanner_fs_error == FS_ERR_RESOURCE_MISSING && audio_file_valid);
                img_valid_mask <= scan_resource_valid;
                img_found_count <= (catalog_image_count > 7) ? 3'd7 : catalog_image_count;
            end

            // Keys are events only. No preload-complete gate and no second
            // request queue here; the cache scheduler owns request replacement.
            if (key_next_press && scan_done && catalog_image_count > 1)
                player_next_event <= 1;
            if (!key_auto_stable) begin
                if (long_press_cnt < (CLK_FREQ_HZ/1000)*LONG_PRESS_MS-1)
                    long_press_cnt <= long_press_cnt + 1'b1;
                else if (!long_press_marked) begin
                    long_press_marked <= 1;
                    if (scan_done && catalog_image_count > 1) player_prev_event <= 1;
                end
            end else begin
                long_press_cnt <= 0;
                if (!key_auto_stable_old) begin
                    if (!long_press_marked && scan_done && catalog_image_count > 1)
                        player_toggle_auto_event <= 1;
                    long_press_marked <= 0;
                end
            end

            if (!cache_recovery_required) begin
                case (job_state)
                JOB_IDLE: if (cache_load_valid && cache_load_ready) begin
                    cache_job_image_id <= cache_load_image_id;
                    pending_buf_idx <= cache_load_bank;
                    load_idx <= cache_load_bank; // legacy observation only
                    write_buf_idx <= cache_load_bank;
                    source_done_seen <= 0;
                    write_done_seen <= 0;
                    load_error_seen <= 0;
                    retry_count <= 0;
                    cache_load_success <= 0;
                    cache_load_error <= 0;
                    job_state <= JOB_QUERY;
                end
                // The catalogue reads metadata from synchronous RAM. The
                // accepted ID remains held throughout this extra query cycle.
                JOB_QUERY: begin
                    load_cluster <= catalog_query_cluster;
                    load_file_size <= catalog_query_size;
                    if (catalog_query_valid) begin
                        load_busy <= 1;
                        load_start_pulse <= 1;
                        job_state <= JOB_RUN;
                    end else begin
                        // Reject an absent/out-of-range file without touching
                        // the writer or reusing a previous directory query.
                        cache_load_done <= 1;
                        cache_load_error <= 8'h74;
                        load_busy <= 0;
                        job_state <= JOB_IDLE;
                    end
                end
                JOB_RUN: begin
                    if (!load_start_pulse && !streamer_start_pulse &&
                        load_prep_state == PREP_IDLE && !load_error) begin
                        if (bmp_source_done) source_done_seen <= 1;
                        if (frame_write_success) write_done_seen <= 1;
                    end
                    if (load_error && !load_error_seen) begin
                        load_error_seen <= 1;
                        source_done_seen <= 0;
                        write_done_seen <= 0;
                        cache_load_error <= {5'd0,bmp_error};
                        cache_write_flush_req <= 1;
                        cleanup_wait <= 0;
                        job_state <= JOB_FLUSH;
                    end else if (source_done_seen && write_done_seen &&
                                 !writer_error_seen && frame_writer_error == 0 &&
                                 !frame_writer_busy && !streamer_busy) begin
                        // Success already includes destination CDC drain via
                        // frame_writer_done; it needs no destructive cleanup.
                        load_busy <= 0;
                        cache_load_done <= 1;
                        cache_load_success <= 1;
                        cache_load_error <= 0;
                        job_state <= JOB_IDLE;
                        if (!first_image_committed) begin
                            first_image_committed <= 1;
                            display_valid <= 1; // at least one frame has loaded
                            disp_buf_idx <= pending_buf_idx;
                            img_idx <= pending_buf_idx;
                        end
                    end
                end
                JOB_FLUSH: begin
                    if (cache_write_flush_done && !frame_writer_busy &&
                        !streamer_busy && !streamer_start_pulse) begin
                        cache_write_flush_req <= 0;
                        cleanup_wait <= 0;
                        job_state <= JOB_RELEASE;
                    end else if (cleanup_wait >= LOAD_WAIT_TIMEOUT_CYCLES-1)
                        cache_recovery_required <= 1;
                    else cleanup_wait <= cleanup_wait + 1'b1;
                end
                JOB_RELEASE: begin
                    if (!cache_write_flush_done) begin
                        if (retry_count == 0) begin
                            retry_count <= 1;
                            load_start_pulse <= 1;
                            load_error_seen <= 0;
                            job_state <= JOB_RUN;
                        end else begin
                            load_busy <= 0;
                            cache_load_done <= 1;
                            cache_load_success <= 0;
                            job_state <= JOB_IDLE;
                        end
                    end else if (cleanup_wait >= LOAD_WAIT_TIMEOUT_CYCLES-1)
                        cache_recovery_required <= 1;
                    else cleanup_wait <= cleanup_wait + 1'b1;
                end
                default: cache_recovery_required <= 1;
                endcase
            end
        end
    end
end else begin : g_static_control
    assign cache_job_active = 1'b0;
    // Keep new ports benign when compiling the verified static-cache top.
    always @(posedge clk or posedge rst) begin
        cache_load_done <= 0;
        cache_load_success <= 0;
        cache_load_error <= 0;
        cache_job_image_id <= 0;
        cache_write_flush_req <= 0;
        cache_recovery_required <= 0;
    end
always @(posedge clk or posedge rst) begin
    if (rst) begin
        load_cluster          <= 32'd0;
        load_file_size        <= 32'd0;
        use_streamer          <= 1'b0;
        scan_success          <= 1'b0;
        scan_done             <= 1'b0;
        wrfin_tgl_sync        <= 3'b000;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        scan_kicked           <= 1'b0;
        first_image_committed <= 1'b0;
        preload_mode          <= 1'b0;
        preload_pending_mask  <= 4'd0;
        auto_play_en          <= 1'b0;
        auto_cnt              <= 32'd0;
        img_found_count       <= 3'd0;
        img_valid_mask        <= 4'd0;
        img_idx               <= 2'd0;
        load_idx              <= 2'd0;
        pending_buf_idx       <= 2'd0;
        write_buf_idx         <= 2'd0;
        disp_buf_idx          <= 2'd0;
        next_req_pending      <= 1'b0;
        load_busy             <= 1'b0;
        skip_on_failure        <= 1'b0;
        source_done_seen      <= 1'b0;
        write_done_seen       <= 1'b0;
        load_error_seen       <= 1'b0;
        retry_count           <= 2'd0;
        display_valid         <= 1'b0;
        //新增变量加入复位
        key_auto_stable_old <= 1'b1;
        long_press_cnt    <= 32'd0;
        long_press_marked <= 1'b0;
        prev_req_pending  <= 1'b0;
        state_code        <= 4'd1;
        player_next_event        <= 1'b0;
        player_prev_event        <= 1'b0;
        player_toggle_auto_event <= 1'b0;
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;
        player_next_event        <= 1'b0;
        player_prev_event        <= 1'b0;
        player_toggle_auto_event <= 1'b0;
        
        key_auto_stable_old <= key_auto_stable;//位置在这是因为只要系统不复位，每一次时钟沿都应该记录一次状态

        state_code <= {2'b00, img_idx} + 4'd1;

        if (!sd_init_done) begin
            load_cluster          <= 32'd0;
            load_file_size        <= 32'd0;
            use_streamer          <= 1'b0;
            scan_success          <= 1'b0;
            scan_done             <= 1'b0;
            scan_kicked           <= 1'b0;
            first_image_committed <= 1'b0;
            preload_mode          <= 1'b0;
            preload_pending_mask  <= 4'd0;
            auto_play_en          <= 1'b0;
            auto_cnt              <= 32'd0;
            img_found_count       <= 3'd0;
            img_valid_mask        <= 4'd0;
            img_idx               <= 2'd0;
            load_idx              <= 2'd0;
            pending_buf_idx       <= 2'd0;
            write_buf_idx         <= 2'd0;
            disp_buf_idx          <= 2'd0;
            next_req_pending      <= 1'b0;
            load_busy             <= 1'b0;
            skip_on_failure        <= 1'b0;
            source_done_seen      <= 1'b0;
            write_done_seen       <= 1'b0;
            load_error_seen       <= 1'b0;
            retry_count           <= 2'd0;
            display_valid         <= 1'b0;

            // TF卡未初始化时，清除尚未完成的KEY2操作
            long_press_cnt        <= 32'd0;
            long_press_marked     <= 1'b0;
            prev_req_pending      <= 1'b0;
        end else begin
            // 完成位由loader保持；完成或放弃的图片都不再安排读卡。
            if (USE_BANK_WRITER && preload_mode)
                preload_pending_mask <= preload_remaining_mask;

            // load_error是单周期脉冲；脉冲结束后允许识别下一次错误
            if (!load_error)
                load_error_seen <= 1'b0;

            // 只处理正在进行的图片加载错误
            if (load_error && !load_error_seen) begin
                load_error_seen <= 1'b1;

                if (load_busy) begin
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;

                    if (retry_count == 2'd0) begin
                        // 第一次加载失败：保留load_cluster和load_file_size，重试同一张
                        retry_count      <= 2'd1;
                        load_start_pulse <= 1'b1;
                    end else if (USE_BANK_WRITER && preload_mode) begin
                        // 第二次失败：只放弃当前图片，下轮调度继续处理其他项。
                        // 不设置image_complete，播放器不会显示这个坏bank。
                        preload_pending_mask <= preload_remaining_mask &
                                                ~(4'b0001 << load_idx);
                        load_busy       <= 1'b0;
                        retry_count     <= 2'd0;
                        skip_on_failure <= 1'b0;
                    end else if (skip_on_failure) begin
                        // 重试仍失败：跳过当前坏图
                        load_idx       <= next_from_loading;
                        load_cluster   <= cluster_lut(next_from_loading);
                        load_file_size <= size_lut(next_from_loading);

                        retry_count      <= 2'd0;
                        load_start_pulse <= 1'b1;
                        auto_cnt         <= 32'd0;
                    end else begin
                        // 手动加载重试仍失败：保留当前画面
                        load_busy      <= 1'b0;
                        retry_count    <= 2'd0;
                        skip_on_failure <= 1'b0;
                    end

                end else if (scan_kicked && !scan_done) begin
                    // 扫描超时：允许下一周期重新发起扫描
                    scan_kicked  <= 1'b0;
                    retry_count  <= 2'd0;
                end
            end

            // 完成脉冲只出现一拍：保存结果，后续按键/轮播使用保持状态。
            // 扫描器已完成最后一个扇区，接受结果后才把读卡使用权交给文件流读取器。
            if (scanner_done_pulse) begin
                scan_done    <= 1'b1;
                scan_success <= scanner_result_usable;
                use_streamer <= scanner_result_usable;
                if (scanner_result_usable) begin
                    img_valid_mask <= scan_resource_valid;
                    preload_mode <= USE_BANK_WRITER;
                    preload_pending_mask <= USE_BANK_WRITER ? scan_resource_valid : 4'd0;
                    // 搜索从前一个编号开始，第一次就会选中最小有效编号。
                    if (USE_BANK_WRITER)
                        load_idx <= first_index_valid(scan_resource_valid) - 2'd1;
                    // 每一项扩展成3位再相加，确保四张都找到时能表示4。
                    img_found_count <= {2'b00, scan_resource_valid[0]} +
                                       {2'b00, scan_resource_valid[1]} +
                                       {2'b00, scan_resource_valid[2]} +
                                       {2'b00, scan_resource_valid[3]};
                end else begin
                    img_valid_mask  <= 4'd0;
                    img_found_count <= 3'd0;
                    preload_mode <= 1'b0;
                    preload_pending_mask <= 4'd0;
                end
            end

            // 完成通知都只出现一拍，分别记住，避免先到的通知被错过。
            // 准备写入FIFO期间不接收通知，防止上一轮的延迟通知混入新加载。
            if (load_busy && !load_start_pulse && !streamer_start_pulse &&
                (load_prep_state == PREP_IDLE) && !load_error) begin
                if (bmp_source_done)
                    source_done_seen <= 1'b1;
                if (frame_write_success)
                    write_done_seen <= 1'b1;
            end

            // 文件完整有效且写侧成功结束后，才提交新图。四 bank 的成功
            // 反馈目前确认命令已送到SDRAM用户接口；物理时序仍须上板验证。
            if (load_busy && source_done_seen && write_done_seen && !load_error &&
                (!USE_BANK_WRITER || (!writer_error_seen && frame_writer_error == 8'd0))) begin
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
                load_error_seen <= 1'b0;
                retry_count     <= 2'd0;
                skip_on_failure        <= 1'b0;
                // 预加载后面的图片不改变首图；切图交给media_player_control。
                if (!USE_BANK_WRITER || !first_image_committed) begin
                    disp_buf_idx          <= USE_BANK_WRITER ? load_idx : pending_buf_idx;
                    img_idx               <= load_idx;
                    display_valid         <= 1'b1;
                    first_image_committed <= 1'b1;
                end
            end

            // SD卡和SDRAM已初始化，且读卡通路空闲时，自动扫描PIC1~PIC4.BMP。
            if (!scan_kicked && Sdr_init_done && !scanner_busy &&
                !streamer_busy && !arb_sector_busy) begin
                scan_start_pulse      <= 1'b1;
                scan_kicked           <= 1'b1;
                scan_done             <= 1'b0;
                scan_success          <= 1'b0;
                use_streamer          <= 1'b0;
                first_image_committed <= 1'b0;
                preload_mode          <= 1'b0;
                preload_pending_mask  <= 4'd0;
                auto_play_en          <= 1'b0;
                auto_cnt              <= 32'd0;
                img_found_count       <= 3'd0;
                img_valid_mask        <= 4'd0;
                img_idx               <= 2'd0;
                load_idx              <= 2'd0;
                pending_buf_idx       <= 2'd0;
                write_buf_idx         <= 2'd0;
                disp_buf_idx          <= 2'd0;
                next_req_pending      <= 1'b0;
                display_valid         <= 1'b0;
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
            end else begin
                // 忙的时候也只记 1 次“下一张”请求，不会累积成连跳两张。
                if (key_next_press && scan_done && preload_complete && (img_found_count > 3'd1)) begin
                    if (!USE_BANK_WRITER)
                        next_req_pending <= 1'b1;
                    player_next_event <= 1'b1;
                end

                // 替换：KEY2长按检测：长按触发上一张，短按松开时切换自动轮播
                if (key_auto_stable == 1'b0) begin
                    // 按键保持按下时持续计数
                    if (long_press_cnt <
                        ((CLK_FREQ_HZ / 1000) * LONG_PRESS_MS - 1)) begin
                        long_press_cnt <= long_press_cnt + 32'd1;
                    end else if (!long_press_marked) begin
                        // 达到长按时间后只触发一次
                        long_press_marked <= 1'b1;

                        if (scan_done && preload_complete && (img_found_count > 3'd1)) begin
                            if (!USE_BANK_WRITER)
                                prev_req_pending <= 1'b1;
                            player_prev_event <= 1'b1;
                        end
                    end
                end else begin
                    // 按键处于松开状态
                    long_press_cnt <= 32'd0;

                    // 当前为1、上一周期为0，说明刚刚松开
                    if (!key_auto_stable_old) begin
                        // 没有达到长按时间，才把这次操作视为短按
                        if (!long_press_marked &&
                            scan_done && preload_complete &&
                            (img_found_count > 3'd1)) begin
                            if (!USE_BANK_WRITER)
                                auto_play_en <= ~auto_play_en;
                            auto_cnt     <= 32'd0;
                            player_toggle_auto_event <= 1'b1;
                        end

                        // 本次按键操作结束，为下一次操作做准备
                        long_press_marked <= 1'b0;
                    end
                end

                // 自动播放可配置周期计数：只有当前没有写图任务时才计时
                if (!USE_BANK_WRITER && scan_done && auto_play_en && display_valid && !load_busy && first_image_committed && (img_found_count > 3'd1)) begin
                    if (auto_tick)
                        auto_cnt <= 32'd0;
                    else
                        auto_cnt <= auto_cnt + 32'd1;
                end else begin
                    auto_cnt <= 32'd0;
                end

                // 四bank模式：扫描后逐张预加载，之后不再因按键/轮播重新读卡。
                if (USE_BANK_WRITER) begin
                    if (preload_mode && scan_done && bmp_ready && !load_busy && !load_error) begin
                        if (next_preload_selection[2]) begin
                            load_idx         <= next_preload_selection[1:0];
                            load_cluster     <= cluster_lut(next_preload_selection[1:0]);
                            load_file_size   <= size_lut(next_preload_selection[1:0]);
                            // 每张图片固定对应自己的bank，不再交替使用buffer0/1。
                            pending_buf_idx  <= next_preload_selection[1:0];
                            write_buf_idx    <= next_preload_selection[1:0];
                            load_start_pulse <= 1'b1;
                            load_busy        <= 1'b1;
                            skip_on_failure  <= 1'b1;
                            source_done_seen <= 1'b0;
                            write_done_seen  <= 1'b0;
                            load_error_seen  <= 1'b0;
                            retry_count      <= 2'd0;
                        end else begin
                            // 全部完成或已各重试一次后放弃；全坏时也停止调度。
                            preload_mode <= 1'b0;
                        end
                    end
                end
                // 旧双缓冲模式：首图选已找到的最小编号，自动加载到 buffer0。
                else if (scan_done && !first_image_committed && bmp_ready && !load_busy && (img_found_count != 3'd0)) begin
                    load_idx         <= first_valid_idx;
                    load_cluster     <= cluster_lut(first_valid_idx);
                    load_file_size   <= size_lut(first_valid_idx);
                    pending_buf_idx  <= 2'd0;
                    write_buf_idx    <= 2'd0;
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    skip_on_failure   <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    load_error_seen  <= 1'b0;
                    retry_count      <= 2'd0;
                    next_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
                // 手动下一张优先：写到“非当前显示”的另一块 buffer
                else if (scan_done && bmp_ready && display_valid && !load_busy && next_req_pending && (img_found_count > 3'd1)) begin
                    load_idx         <= next_from_current;
                    load_cluster     <= cluster_lut(next_from_current);
                    load_file_size   <= size_lut(next_from_current);
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    skip_on_failure   <= 1'b0;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    load_error_seen <= 1'b0;
                    retry_count     <= 2'd0;
                    next_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
                // KEY2长按上一张：优先级低于手动下一张，高于自动轮播
                else if (scan_done && bmp_ready && display_valid && !load_busy && prev_req_pending && (img_found_count > 3'd1)) begin
                    load_idx       <= prev_from_current;
                    load_cluster   <= cluster_lut(prev_from_current);
                    load_file_size <= size_lut(prev_from_current);
                    pending_buf_idx <= next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx   <= next_buf_lut(disp_buf_idx, display_valid);

                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    skip_on_failure <= 1'b0;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    load_error_seen <= 1'b0;
                    retry_count     <= 2'd0;                    
                    prev_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
                // 自动播放下一张：同样写到“非当前显示”的另一块 buffer
                else if (scan_done && bmp_ready && display_valid && !load_busy && auto_play_en && auto_tick && (img_found_count > 3'd1)) begin
                    load_idx         <= next_from_current;
                    load_cluster     <= cluster_lut(next_from_current);
                    load_file_size   <= size_lut(next_from_current);
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    skip_on_failure <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    load_error_seen <= 1'b0;
                    retry_count     <= 2'd0;                    
                    auto_cnt         <= 32'd0;
                end
            end
        end
    end
end

end endgenerate

// FAT32卷扫描器：输出文件起始簇、大小及卷布局信息。
// 扫描完成是单周期通知，先单独接出，后续由控制逻辑记录扫描结果。
generate if (DYNAMIC_CACHE) begin : g_cache_catalog
    fat32_media_catalog #(.MAX_IMAGES(MAX_IMAGES), .ENABLE_PLAYLIST(MULTI_SONG),
        .MAX_SONGS(MAX_SONGS), .ENABLE_CUES(MUSIC_CUES && MULTI_SONG),
        .RAW_QUERY_DATA(MUSIC_CUES && MULTI_SONG)) u_catalog (
        .clk(clk), .rst_n(~rst), .scan_start(scan_start_pulse),
        .scan_busy(scanner_busy), .scan_done(scanner_done_pulse),
        .fs_error(scanner_fs_error), .partition_lba(),
        .sectors_per_cluster(scanner_spc), .fat_start_lba(scanner_fat_start),
        .data_start_lba(scanner_data_start), .max_cluster(scanner_max_cluster),
        .image_valid_mask(catalog_image_mask), .image_count(catalog_image_count),
        .query_image_id(cache_job_active ? cache_job_image_id : cache_load_image_id), .query_valid(catalog_query_valid),
        .query_cluster(catalog_query_cluster), .query_size(catalog_query_size),
        .audio_valid(audio_file_valid), .audio_cluster(audio_file_cluster),
        .song_valid_mask(song_valid_mask), .query_song_enable(song_query_enable),
        .query_song_id(song_query_id), .query_song_valid(song_query_valid),
        .query_song_cluster(song_query_cluster), .query_song_size(song_query_size),
        .cue_valid_mask(cue_valid_mask), .query_cue_enable(cue_query_enable),
        .query_cue_valid(cue_query_valid), .query_cue_cluster(cue_query_cluster), .query_cue_size(cue_query_size),
        .audio_size(audio_file_size), .sector_req(scanner_sector_req),
        .sector_lba(scanner_sector_lba), .sector_busy(scanner_sector_busy),
        .sector_valid(scanner_sector_valid), .sector_byte(scanner_sector_byte),
        .sector_byte_index(scanner_sector_byte_index), .sector_done(scanner_sector_done),
        .sector_error(scanner_sector_error)
    );
    assign scan_resource_valid = catalog_image_mask;
    assign {img_cluster0,img_cluster1,img_cluster2,img_cluster3} = 128'd0;
    assign {img_size0,img_size1,img_size2,img_size3} = 128'd0;
end else begin : g_static_catalog
    assign cue_valid_mask=0;
    assign cue_query_valid=0;
    assign cue_query_cluster=0;
    assign cue_query_size=0;
    assign song_valid_mask=audio_file_valid ? {{(MAX_SONGS-1){1'b0}},1'b1} : 0;
    assign song_query_valid=audio_file_valid && song_query_id==0;
    assign song_query_cluster=audio_file_cluster;
    assign song_query_size=audio_file_size;
    assign catalog_image_mask = 0;
    assign catalog_image_count = 0;
    assign catalog_query_valid = 0;
    assign catalog_query_cluster = 0;
    assign catalog_query_size = 0;
fat32_volume_scanner u_scanner (
    .clk                 (clk),
    .rst_n               (~rst),
    .scan_start          (scan_start_pulse),
    .scan_busy           (scanner_busy),
    .scan_done           (scanner_done_pulse),
    .fs_error            (scanner_fs_error),
    .partition_lba       (),
    .sectors_per_cluster (scanner_spc),
    .fat_start_lba       (scanner_fat_start),
    .data_start_lba      (scanner_data_start),
    .max_cluster         (scanner_max_cluster),
    .resource_valid      (scan_resource_valid),
    .resource0_cluster   (img_cluster0),
    .resource1_cluster   (img_cluster1),
    .resource2_cluster   (img_cluster2),
    .resource3_cluster   (img_cluster3),
    .resource0_size      (img_size0),
    .resource1_size      (img_size1),
    .resource2_size      (img_size2),
    .resource3_size      (img_size3),
    .audio_valid         (audio_file_valid),
    .audio_cluster       (audio_file_cluster),
    .audio_size          (audio_file_size),
    .sector_req          (scanner_sector_req),
    .sector_lba          (scanner_sector_lba),
    .sector_busy         (scanner_sector_busy),
    .sector_valid        (scanner_sector_valid),
    .sector_byte         (scanner_sector_byte),
    .sector_byte_index   (scanner_sector_byte_index),
    .sector_done         (scanner_sector_done),
    .sector_error        (scanner_sector_error)
);
end endgenerate

// FAT32文件流读取器：沿指定文件的簇链，依次输出文件字节（包含BMP头）。
// file_done是单周期结束通知，错误详情由streamer_fs_error提供。
fat32_file_streamer u_streamer (
    .clk                 (clk),
    .rst_n               (~rst),
    .file_start          (streamer_start_pulse),
    .first_cluster       (load_cluster),
    .file_size           (load_file_size),
    .sectors_per_cluster (scanner_spc),
    .fat_start_lba       (scanner_fat_start),
    .data_start_lba      (scanner_data_start),
    .max_cluster         (scanner_max_cluster),
    .data_sector_ready   (streamer_data_sector_ready),
    .file_busy           (streamer_busy),
    .file_done           (streamer_done_pulse),
    .file_valid          (streamer_byte_valid),
    .file_byte           (streamer_byte),
    .file_byte_offset    (streamer_byte_offset),
    .fs_error            (streamer_fs_error),
    .sector_req          (streamer_sector_req),
    .sector_lba          (streamer_sector_lba),
    .sector_accept       (streamer_sector_accept),
    .sector_valid        (streamer_sector_valid),
    .sector_byte         (streamer_sector_byte),
    .sector_byte_index   (streamer_sector_byte_index),
    .sector_done         (streamer_sector_done),
    .sector_error        (streamer_sector_error)
);

// 65536 clusters permits up to 32 MiB even at the minimum 512-byte cluster;
// unlike the image guard (8192), it covers the current 20,852,736-byte song.
fat32_file_streamer #(.MAX_VISITED_CLUSTERS(65536), .SUPPORT_CANCEL(MULTI_SONG)) u_pcm_streamer (
    .clk                 (clk),
    .rst_n               (~rst),
    .file_start          (pcm_streamer_start_pulse | cue_load_start),
    .file_cancel         (pcm_cancel),
    .first_cluster       (cue_loading ? cue_load_cluster : pcm_selected_cluster),
    .file_size           (cue_loading ? cue_load_size : pcm_selected_size),
    .sectors_per_cluster (scanner_spc),
    .fat_start_lba       (scanner_fat_start),
    .data_start_lba      (scanner_data_start),
    .max_cluster         (scanner_max_cluster),
    .data_sector_ready   (cue_loading ? 1'b1 : pcm_sector_ready),
    .file_busy           (pcm_streamer_busy),
    .file_done           (pcm_streamer_done_pulse),
    .file_valid          (pcm_streamer_byte_valid),
    .file_byte           (pcm_streamer_byte),
    .file_byte_offset    (pcm_streamer_byte_offset),
    .fs_error            (pcm_streamer_fs_error),
    .sector_req          (pcm_streamer_sector_req),
    .sector_lba          (pcm_streamer_sector_lba),
    .sector_accept       (pcm_streamer_sector_accept),
    .sector_valid        (pcm_streamer_sector_valid),
    .sector_byte         (pcm_streamer_sector_byte),
    .sector_byte_index   (pcm_streamer_sector_byte_index),
    .sector_done         (pcm_streamer_sector_done),
    .sector_error        (pcm_streamer_sector_error)
);

fat32_file_sector_arbiter u_file_arbiter (
    .clk               (clk),
    .rst_n             (~rst),
    .enable            (use_streamer),
    .bmp_req           (streamer_sector_req),
    .bmp_lba           (streamer_sector_lba),
    .bmp_accept        (file_arb_bmp_accept),
    .pcm_req           (pcm_streamer_sector_req),
    .pcm_lba           (pcm_streamer_sector_lba),
    .pcm_accept        (file_arb_pcm_accept),
    .sector_req        (file_arb_sector_req),
    .sector_lba        (file_arb_sector_lba),
    .sector_busy       (arb_sector_busy),
    .sector_valid      (arb_sector_valid),
    .sector_byte       (arb_sector_byte),
    .sector_byte_index (arb_sector_byte_index),
    .sector_done       (arb_sector_done),
    .sector_error      (arb_sector_error),
    .bmp_valid         (file_arb_bmp_valid),
    .bmp_byte          (file_arb_bmp_byte),
    .bmp_byte_index    (file_arb_bmp_byte_index),
    .bmp_done          (file_arb_bmp_done),
    .bmp_error         (file_arb_bmp_error),
    .pcm_valid         (file_arb_pcm_valid),
    .pcm_byte          (file_arb_pcm_byte),
    .pcm_byte_index    (file_arb_pcm_byte_index),
    .pcm_done          (file_arb_pcm_done),
    .pcm_error         (file_arb_pcm_error)
);

// SD卡扇区接口适配：仲裁后的单周期请求转成官方驱动要求的保持请求。
// sd_sec_read和sd_sec_read_addr统一由本实例驱动。
sd_sector_adapter #(
    .CLK_FREQ_HZ            (CLK_FREQ_HZ),
    .SD_READ_TIMEOUT_MS     (SD_READ_TIMEOUT_MS)
) u_adapter (
    .clk                    (clk),
    .rst_n                  (~rst),
    .sector_req             (arb_sector_req),
    .sector_lba             (arb_sector_lba),
    .sector_busy            (arb_sector_busy),
    .sector_valid           (arb_sector_valid),
    .sector_byte            (arb_sector_byte),
    .sector_byte_index      (arb_sector_byte_index),
    .sector_done            (arb_sector_done),
    .sector_error           (arb_sector_error),
    .read_timeout           (debug_sd_read_timeout),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_end        (sd_sec_read_end)
);

sd_card_top sd_card_top_m0(
    .clk                    (clk),
    .rst                    (rst),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

endmodule

module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,    // 默认：松开=1，按下=0
    output reg  press_pulse,     // 按下时输出一个单拍脉冲
    output reg button_stable_out
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg button_sync0;
reg button_sync1;
reg button_stable;//消抖后的稳定按键状态
reg [31:0] cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        button_stable_out <= 1'b1;//新增变量复位
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;//记录输入经过缓冲处理过后的状态
        press_pulse  <= 1'b0;//常态下总是输出的0

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else begin
            if (cnt >= DEBOUNCE_CYCLES - 1) begin//这里是消抖条件已经满足的情况
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;//经过消抖过后还满足稳定状态为1（未同步），输入状态为0，才会有有效输出1。
                button_stable <= button_sync1;//消抖后条件满足，稳定按键状态同步按键输入
                button_stable_out <= button_sync1;  // 新增：同步输出
                cnt           <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule
