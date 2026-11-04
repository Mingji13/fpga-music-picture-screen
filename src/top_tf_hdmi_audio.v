
module top(
    input                       clk,
    input                       rst_n,
    input                       key1,           // 手动下一张
    input                       key2,           // 自动播放 开/关
    input                       key4,           // board KEY4: short next / long previous song
    input                       song_order_sw,  // SW2: 0 repeat one, 1 repeat all
    input [1:0]                 audio_mode_sw,  // {SW4, SW3}: 00测试音，01琶音
    input                       waveform_sw,   // SW1: 1显示波形，0隐藏波形

    output [7:0]                seg_sel,
    output [7:0]                seg_data,

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso
);

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 24'd307200;   // 640*480
parameter BUF0_ADDR     = 24'd0;
parameter BUF1_ADDR     = FRAME_PIXELS;
parameter integer VIDEO_FADE_FRAMES = 16; // 手动淡出/淡入各16帧，自动揭示16帧；0=旁路
parameter integer MAX_MEDIA_IMAGES = 16;
parameter integer AUTO_PERIOD_CYCLES = 300000000;
parameter integer SD_RETRY_RESET_CYCLES = 10000000; // 100 ms @ 100 MHz
parameter integer AUDIO_STARTUP_FRAMES = 150; // ~2.5 s of valid image frames before consuming music
parameter integer SONG_LONG_PRESS_CYCLES = 100000000; // 1 s @ 100 MHz, matching board KEY3
localparam integer SONG_HOLD_COUNT_BITS = SONG_LONG_PRESS_CYCLES < 2 ? 1 :
                                          $clog2(SONG_LONG_PRESS_CYCLES);
localparam integer IMAGE_ID_WIDTH = MAX_MEDIA_IMAGES <= 1 ? 1 : $clog2(MAX_MEDIA_IMAGES);
localparam integer IMAGE_COUNT_WIDTH = $clog2(MAX_MEDIA_IMAGES+1);
localparam integer VIDEO_CLOCK_HZ = 74000000;
localparam integer IMAGE_BANK_WORDS = 518400; // each of four slots holds up to 960x540 RGB words

wire Sdr_init_done;
wire Sdr_init_done_sd;
wire Sdr_init_ref_vld;
wire Sdr_busy;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;
wire video_clk;
wire hdmi_5x_clk;

// 每个时钟域的同步复位输出，在模块实例使用前声明。
wire sdcard_rst_n;
wire mem_rst_n;
wire video_rst_n;
wire serial_rst_n;
wire audio_rst_n;

wire hs;
wire vs;
wire de;

// 旧链路和新链路分别输出，再统一选择送往 HDMI 的时序与像素。
wire legacy_hs;
wire legacy_vs;
wire legacy_de;
wire [23:0] legacy_vout_data_raw;
wire bank_video_hs;
wire bank_video_vs;
wire bank_video_de;
wire [23:0] bank_video_rgb;
wire        bank_reveal_active;
wire [1:0]  bank_reveal_effect;
wire [8:0]  bank_reveal_progress;
wire [4:0]  bank_fade_level;
wire        bank_reveal_show_pixel;
wire [23:0] bank_rgb_after_transition;
wire [2047:0] waveform_history;
wire        waveform_visible;
wire        bank_waveform_hs, bank_waveform_vs, bank_waveform_de;
wire [23:0] bank_waveform_rgb;
wire        bank_scaled_hs, bank_scaled_vs, bank_scaled_de;
wire [23:0] bank_scaled_rgb;
wire [11:0] bank_scaled_x;
wire        bank_scaled_image_valid;
wire        bank_scale_source_ready;
wire        bank_scale_underflow;
wire        bank_scale_config_error;
wire [11:0] bank_scaled_image_width, bank_scaled_image_left;
wire [11:0] bank_scaled_image_height, bank_scaled_image_top;
wire        bank_frame_stream_start;
wire        bank_frame_stream_ready;

wire [23:0] vout_data_raw;
wire [23:0] vout_data;
wire        display_valid;
wire        source_display_valid;

wire        debug_sd_init_done;
wire        debug_sd_read_timeout;
wire        sdcard_module_rst;

localparam integer SD_INIT_TIMEOUT_CYCLES = 200_000_000; // 2 s @ 100 MHz
reg [31:0]  sd_init_timeout_cnt = 32'd0;
reg [31:0]  sd_retry_reset_cnt  = 32'd0;
reg         sd_retry_reset      = 1'b0;

wire        video_read_req;
wire        video_read_req_ack;
wire        video_read_en;
wire [31:0] video_read_data;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;
wire        frame_write_finish;
reg         frame_write_toggle_mem;
wire        frame_load_start;
wire [1:0]  frame_load_index;
wire        frame_commit;
wire [7:0]  frame_decode_error;
wire frame_config_valid, frame_top_down;
wire [15:0] frame_width, frame_height;
wire [15:0] source_width, source_height;
wire [31:0] frame_pixels;
reg [159:0] bank_image_geometry;
wire [15:0] bank_active_width, bank_active_height;
wire [21:0] bank_active_word_count;

// Metadata is written before pixels and becomes selectable only when the
// loader publishes image_complete after the destination queue has drained.
always @(posedge sd_card_clk or posedge sdcard_module_rst) begin
    if (sdcard_module_rst) bank_image_geometry <= 160'd0;
    else begin
        if (frame_load_start)
            bank_image_geometry[frame_load_index*40 +: 40] <= 40'd0;
        else if (frame_config_valid)
            bank_image_geometry[frame_load_index*40 +: 40] <=
                {frame_pixels[19:0],frame_height[9:0],frame_width[9:0]};
    end
end

wire [1:0]  write_buf_idx;
wire [1:0]  disp_buf_idx;

wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;

// Build-time routing: select the reader and its matching video output together
// after preload scheduling and startup display gating are connected.
localparam USE_BANK_READER = 1'b1;
wire                    bank_reader_path_select;
wire                    mem_app_rd_en;
wire [ADDR_BITS-1:0]    mem_app_rd_addr;
wire                    bank_reader_sdram_rd_valid;
wire                    legacy_reader_sdram_rd_valid;
wire                    legacy_video_read_req;
wire                    mem_app_wr_en;
wire [ADDR_BITS-1:0]    mem_app_wr_addr;
wire [MEM_DATA_BITS-1:0] mem_app_wr_data;
wire [3:0]              mem_app_wr_mask;
wire                    legacy_sd_write_req;
wire                    legacy_sd_write_en;
wire                    bank_write_src_rst_n;
wire                    bank_write_mem_rst_n;
wire                    bank_player_mem_rst_n;

// Catalogue file IDs and physical SDRAM banks are independent.
wire cache_catalog_valid;
wire [MAX_MEDIA_IMAGES-1:0] cache_catalog_mask, cache_bad_mask;
wire [IMAGE_COUNT_WIDTH-1:0] cache_catalog_count;
wire [7:0] cache_catalog_error, cache_load_error;
wire cache_load_valid, cache_load_ready, cache_load_done, cache_load_success;
wire cache_load_busy, cache_switch_valid, cache_switch_ready, cache_switch_done;
wire cache_switch_busy, cache_switch_initial, cache_switch_auto, cache_switch_previous;
wire cache_active_valid, cache_request_pending, cache_no_images;
wire [IMAGE_ID_WIDTH-1:0] cache_load_id, cache_job_id, cache_switch_id;
wire [IMAGE_ID_WIDTH-1:0] cache_active_id, bank_active_image_id, cache_display_id;
wire [1:0] cache_load_bank, cache_switch_bank, cache_active_bank, cache_display_bank;
wire [3:0] cache_slot_valid, cache_slot_loading, cache_protected_mask;
wire [4*IMAGE_ID_WIDTH-1:0] cache_slot_ids;
wire cache_display_commit;
wire cache_flush_request, cache_flush_done, cache_flush_reset, cache_block_writes;
wire cache_recovery_required;
wire bank_active_epoch, bank_reload_done, bank_reload_epoch;
wire [1:0] bank_reload_bank;

// Four-bank write path, shared by initial fill and background cache misses.
wire        bank_loader_busy;
wire        bank_loader_sector_ready;
wire        bank_loader_done;
wire [3:0]  bank_image_complete;
wire [7:0]  bank_memory_error;
wire        bank_loader_wr_en;
wire [20:0] bank_loader_wr_addr;
wire [3:0]  bank_loader_wr_mask;
wire [31:0] bank_loader_wr_data;
wire        bank_loader_write_commit;
wire        bank_loader_write_complete;
wire        bank_cdc_full;
wire        bank_cdc_overflow;
wire        bank_cdc_wr_en;
wire [20:0] bank_cdc_wr_addr;
wire [3:0]  bank_cdc_wr_mask;
wire [31:0] bank_cdc_wr_data;
wire        bank_cdc_idle;
wire        bank_cdc_src_burst_ready;
wire        bank_cdc_burst_request;
wire        bank_cdc_burst_active;
wire        bank_cdc_burst_grant;
wire [7:0]  bank_writer_error;

// Four-bank read path; selects the physical bank committed in video_clk.
wire                    bank_reader_app_rd_en;
wire [ADDR_BITS-1:0]    bank_reader_app_rd_addr;
wire [23:0]             bank_reader_pixel_rgb;
wire                    bank_reader_pixel_valid;
wire                    bank_reader_underflow;
wire                    bank_reader_busy;
wire                    bank_reader_done;
wire [1:0]              bank_reader_index;
wire [7:0]              bank_reader_error;
wire                    bank_reader_burst_request;
wire                    bank_reader_burst_active;
wire                    bank_reader_burst_grant;

// 播放请求在 sd_card_clk 域产生；active_index 在 video_clk 域提交。
wire bank_next_event;
wire bank_prev_event;
wire bank_toggle_auto_event;
wire bank_auto_enabled;
wire [1:0] bank_pending_index;
wire [1:0] bank_active_index;
wire bank_index_commit;
wire bank_control_request_busy;
wire bank_auto_next_event;
wire bank_auto_timer_event, pcm_cue_valid, pcm_cue_event;
wire music_cue_mode;
assign bank_auto_next_event = music_cue_mode ?
    (pcm_cue_event && bank_auto_enabled && cache_active_valid && !cache_switch_busy &&
     !cache_request_pending && !bank_next_event && !bank_prev_event && !bank_toggle_auto_event) : bank_auto_timer_event;
wire bank_fade_busy;
wire bank_control_video_rst_n;
wire bank_active_ready;
wire bank_display_valid_video;
wire bank_any_complete_mem;
wire video_display_valid;

// Only one reader drives the controller; only that reader accepts returns.
assign bank_reader_path_select = USE_BANK_READER;
assign mem_app_rd_en           = bank_reader_path_select ?
                                 bank_reader_app_rd_en : App_rd_en;
assign mem_app_rd_addr         = bank_reader_path_select ?
                                 bank_reader_app_rd_addr : App_rd_addr;
assign bank_reader_sdram_rd_valid = bank_reader_path_select & Sdr_rd_en;
assign legacy_reader_sdram_rd_valid = ~bank_reader_path_select & Sdr_rd_en;
assign legacy_video_read_req = ~bank_reader_path_select & video_read_req;
// 读写通路和视频输出使用同一个编译期选择，避免新旧像素格式混用。
assign mem_app_wr_en   = bank_reader_path_select ? bank_cdc_wr_en   : App_wr_en;
assign mem_app_wr_addr = bank_reader_path_select ? bank_cdc_wr_addr : App_wr_addr;
assign mem_app_wr_data = bank_reader_path_select ? bank_cdc_wr_data : App_wr_din;
assign mem_app_wr_mask = bank_reader_path_select ? bank_cdc_wr_mask : App_wr_dm;
assign legacy_sd_write_req = ~bank_reader_path_select & sd_card_write_req;
assign legacy_sd_write_en  = ~bank_reader_path_select & sd_card_write_en;
assign bank_writer_error = bank_cdc_overflow ? 8'h47 : bank_memory_error;

// Burst admission is centralized in ext_mem_clk. The reader and background
// loader share the SDRAM interface through mutually exclusive burst grants.
media_sdram_burst_arbiter #(
    .TURNAROUND_CYCLES(12)
) u_bank_sdram_burst_arbiter (
    .clk            (ext_mem_clk),
    .rst_n          (bank_player_mem_rst_n),
    .controller_busy(Sdr_busy),
    .refresh_active (Sdr_init_ref_vld),
    .read_request   (bank_reader_burst_request),
    .read_active    (bank_reader_burst_active),
    .write_request  (bank_cdc_burst_request & ~cache_block_writes),
    .write_active   (bank_cdc_burst_active),
    .read_grant     (bank_reader_burst_grant),
    .write_grant    (bank_cdc_burst_grant)
);

// display_valid 在 sd_card_clk 域产生；视频域只有在首图真正提交后
// 才允许 reader 开始请求 SDRAM，避免上电期间读到未写好的 bank。
sync_2ff u_bank_display_valid_sync (
    .clk      (video_clk),
    .rst_n    (bank_control_video_rst_n),
    .async_in (display_valid & bank_reader_path_select),
    .sync_out (bank_display_valid_video)
);

// 至少一张图由写侧提交后即可开放读侧，其余 bank 继续后台加载。
// 具体显示 bank 仍由播放控制器从已完成的 bank 中选择，并在帧边界提交。
sync_2ff u_bank_any_complete_sync (
    .clk      (ext_mem_clk),
    .rst_n    (bank_player_mem_rst_n),
    .async_in (|bank_image_complete),
    .sync_out (bank_any_complete_mem)
);

assign video_display_valid = bank_reader_path_select ?
                              bank_display_valid_video : display_valid;

wire hs_0;
wire vs_0;
wire de_0;

// 正式视频链路：1280x720 画布，SDRAM 按每张图片的实际尺寸存放原图。
wire bank_hs_shadow;
wire bank_vs_shadow;
wire bank_de_shadow;
wire [11:0] bank_x_shadow;
wire [11:0] bank_y_shadow;
reg  bank_vs_shadow_d;
wire bank_frame_start_shadow;
wire bank_pixel_consume_shadow;

// 74 MHz / (1650*750) = 59.798 Hz，与已上板的 720p 链路一致。
video_timing_ctrl #(
    .H_ACT      (1280),
    .H_FP       (110),
    .H_SYNC     (40),
    .H_BP       (220),
    .V_ACT      (720),
    .V_FP       (5),
    .V_SYNC     (5),
    .V_BP       (20),
    .ACTIVE_LAST(1)
) u_bank_video_timing_shadow (
    .pclk       (video_clk),
    .rst_n      (video_rst_n),
    .hsync      (bank_hs_shadow),
    .vsync      (bank_vs_shadow),
    .de         (bank_de_shadow),
    .x          (bank_x_shadow),
    .y          (bank_y_shadow)
);

always @(posedge video_clk or negedge video_rst_n) begin
    if (!video_rst_n)
        bank_vs_shadow_d <= 1'b1;
    else
        bank_vs_shadow_d <= bank_vs_shadow;
end

assign bank_frame_start_shadow  = bank_vs_shadow_d & ~bank_vs_shadow;
// 缩放器复用行缓存：按输入握手取原图，不再每个输出像素都取 SDRAM。
assign bank_pixel_consume_shadow = bank_scale_source_ready & bank_reader_pixel_valid;

// HDMI 1.4b 音频发射相关
wire        sys_pll_lock;
wire        video_pll_lock;
wire        audio_pll_lock;
// 三个 PLL 全部锁定后，再延时 100 ms 释放系统复位
localparam integer POWER_ON_DELAY_CYCLES = 5_000_000;

wire       pll_all_locked;
reg [31:0] power_on_delay_cnt = 32'd0;
reg        power_on_delay_done = 1'b0;

wire        audio_mclk;
wire        audio_i2s_bclk;
wire        audio_i2s_lrck;
wire        audio_i2s_dout;
wire        audio_i2s_valid;
wire [23:0] audio_i2s_left_data;
wire [23:0] audio_i2s_right_data;
wire        pcm_test_valid;
wire [23:0] pcm_test_left_data;
wire [23:0] pcm_test_right_data;
wire        pcm_test_accepted;
wire [3:0]  pcm_test_rom_index;
wire        pcm_media_valid;
wire signed [15:0] pcm_media_left;
wire signed [15:0] pcm_media_right;
wire        pcm_media_overflow;
wire        pcm_media_playback_active;
wire        pcm_sector_ready;
wire        pcm_byte_valid;
wire [7:0]  pcm_byte;
wire        pcm_file_done;
wire [7:0]  pcm_file_error;
wire        pcm_ring_reset_n;
wire        pcm_ring_wr_rst_n, pcm_ring_rd_rst_n;
wire        pcm_ring_overflow, pcm_ring_format_error;
wire        pcm_ring_empty, pcm_ring_done, pcm_ring_underflow;
wire        pcm_wav_sample_valid;
wire        pcm_wav_sample_request;
wire signed [15:0] pcm_wav_sample;
reg         pcm_wav_valid;
wire [23:0] pcm_wav_data;
wire [1:0]  audio_mode_active;
wire pcm_buffer_hold, pcm_buffer_ready, pcm_buffer_cleared;
wire pcm_ring_rd_ready_sd, pcm_playback_done_sd, pcm_wav_mode_sd;
assign music_cue_mode = pcm_wav_mode_sd && pcm_cue_valid;
wire pcm_repeat_all, pcm_next_pressed;
reg pcm_next_pressed_old;
reg pcm_next_track, pcm_previous_track;
reg pcm_song_press_armed, pcm_song_long_marked;
reg [SONG_HOLD_COUNT_BITS-1:0] pcm_song_hold_count;
reg         audio_playback_enabled;
wire        audio_valid;
wire [23:0] audio_left_data;
wire [23:0] audio_right_data;
wire        acr_valid;
wire [19:0] acr_cts;
wire [19:0] acr_n;

// 音源由 SW4/SW3 运行时选择，仍共用现有 HDMI PCM 和 ACR 入口。

wire        axis_s_user;
wire        axis_s_valid;
wire        axis_s_last;
wire [23:0] axis_s_data;
wire        axis_s_ready;
wire        hdmi_video_locked;

wire        edid_trig;
wire        edid_valid;
wire [7:0]  edid_data;

wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

//// 统一复位：TF 图像链路 + HDMI 音频链路
//wire rst_all;
//assign rst_all = ~rst_n | ~audio_pll_lock;

// 现在先保持原有复位条件，2.2再加入另外两个PLL的lock
wire ext_rst_n;

assign pll_all_locked =
    sys_pll_lock &
    video_pll_lock &
    audio_pll_lock;

// 使用原始50MHz时钟产生真正的上电等待时间
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        power_on_delay_cnt  <= 32'd0;
        power_on_delay_done <= 1'b0;
    end else if (!pll_all_locked) begin
        // 任意PLL尚未锁定或重新失锁，都重新开始计时
        power_on_delay_cnt  <= 32'd0;
        power_on_delay_done <= 1'b0;
    end else if (!power_on_delay_done) begin
        if (power_on_delay_cnt >= POWER_ON_DELAY_CYCLES - 1) begin
            power_on_delay_done <= 1'b1;
        end else begin
            power_on_delay_cnt <= power_on_delay_cnt + 32'd1;
        end
    end
end

// 外部复位释放 + 三个PLL锁定 + 100ms等待完成，才释放系统复位
assign ext_rst_n =
    rst_n &
    pll_all_locked &
    power_on_delay_done;

// 保持你原来的 TF / SDRAM / video 时钟
sys_pll sys_pll_m0(
    .refclk     (clk),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft),
    .reset      (~rst_n),
    .extlock    (sys_pll_lock)
);

video_720p_pll video_pll_m0(
    .refclk     (clk),
    .pixel_clk  (video_clk),
    .serial_clk (hdmi_5x_clk),
    .reset      (~rst_n),
    .extlock    (video_pll_lock)
);

// 复用你 I2S->HDMI 工程里的 PLL，只取 12.288MHz 音频主时钟
PLL_HDMI_AUDIO u_audio_pll(
    .refclk     (clk),
    .reset      (~rst_n),
    .extlock    (audio_pll_lock),
    .clk0_out   (),
    .clk1_out   (),
    .clk2_out   (audio_mclk)
);

reset_sync u_sdcard_reset (
    .clk        (sd_card_clk),
    .ext_rst_n  (ext_rst_n),
    .rst_n_sync (sdcard_rst_n)
);

reset_sync u_mem_reset (
    .clk        (ext_mem_clk),
    .ext_rst_n  (ext_rst_n),
    .rst_n_sync (mem_rst_n)
);

reset_sync u_video_reset (
    .clk        (video_clk),
    .ext_rst_n  (ext_rst_n),
    .rst_n_sync (video_rst_n)
);

reset_sync u_serial_reset (
    .clk        (hdmi_5x_clk),
    .ext_rst_n  (ext_rst_n),
    .rst_n_sync (serial_rst_n)
);

reset_sync u_audio_reset (
    .clk        (audio_mclk),
    .ext_rst_n  (ext_rst_n),
    .rst_n_sync (audio_rst_n)
);

// TF卡恢复看门狗：初始化超时或读扇区无进展时，复用同一次100ms业务复位。
// sd_card_bmp及其扫描器、文件流、适配层、官方SD驱动一起复位，避免旧请求混入重试。
always @(posedge sd_card_clk or negedge sdcard_rst_n) begin
    if (!sdcard_rst_n) begin
        sd_init_timeout_cnt <= 32'd0;
        sd_retry_reset_cnt  <= 32'd0;
        sd_retry_reset      <= 1'b0;
    end else if (sd_retry_reset) begin
        sd_init_timeout_cnt <= 32'd0;

        if (sd_retry_reset_cnt >= SD_RETRY_RESET_CYCLES - 1) begin
            sd_retry_reset_cnt <= 32'd0;
            sd_retry_reset     <= 1'b0;
        end else begin
            sd_retry_reset_cnt <= sd_retry_reset_cnt + 32'd1;
        end
    end else if (debug_sd_read_timeout || cache_recovery_required ||
                 (!debug_sd_init_done &&
                  sd_init_timeout_cnt >= SD_INIT_TIMEOUT_CYCLES - 1)) begin
        sd_init_timeout_cnt <= 32'd0;
        sd_retry_reset_cnt  <= 32'd0;
        sd_retry_reset      <= 1'b1;
    end else if (debug_sd_init_done) begin
        sd_init_timeout_cnt <= 32'd0;
        sd_retry_reset_cnt  <= 32'd0;
        sd_retry_reset      <= 1'b0;
    end else begin
        sd_init_timeout_cnt <= sd_init_timeout_cnt + 32'd1;
    end
end

assign sdcard_module_rst = ~sdcard_rst_n | sd_retry_reset;

sync_2ff u_sdram_init_sync (
    .clk      (sd_card_clk),
    .rst_n    (sdcard_rst_n),
    .async_in (Sdr_init_done),
    .sync_out (Sdr_init_done_sd)
);

// mem_clk 域把 write_finish 单拍转成 toggle，供 sd_card_clk 域可靠同步
always @(posedge ext_mem_clk or negedge mem_rst_n) begin
    if (!mem_rst_n)
        frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
end

// ===================== FAT32 catalogue and dynamic cache load jobs =====================
sd_card_bmp #(
    .CLK_FREQ_HZ       (100_000_000),
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd131071),
    .SCAN_TARGET_COUNT (3'd4),
    .USE_BANK_WRITER   (USE_BANK_READER),
    .VARIABLE_BMP_SIZE (USE_BANK_READER),
    .DYNAMIC_CACHE    (USE_BANK_READER),
    .MULTI_SONG       (1),
    .MUSIC_CUES       (1),
    .MAX_IMAGES       (MAX_MEDIA_IMAGES)
) sd_card_bmp_m0(
    .clk               (sd_card_clk),
    .rst               (sdcard_module_rst),
    .key_next          (key1),
    .key_auto          (key2),
    .Sdr_init_done     (Sdr_init_done_sd), 
    .state_code        (),
    .bmp_width         (16'd640),
    .bmp_height        (16'd480),
    .display_valid     (source_display_valid),
    .debug_sd_init_done(debug_sd_init_done),
    .debug_sd_read_timeout(debug_sd_read_timeout),
    .debug_scan_kicked (),
    .debug_scan_done   (),

    .write_finish_toggle(frame_write_toggle_mem),
    .write_buf_idx     (write_buf_idx),
    .disp_buf_idx      (disp_buf_idx),

    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),
    .frame_load_start  (frame_load_start),
    .frame_load_index  (frame_load_index),
    .frame_commit      (frame_commit),
    .frame_decode_error(frame_decode_error),
    .frame_config_valid(frame_config_valid),
    .frame_width       (frame_width),
    .frame_height      (frame_height),
    .source_width      (source_width),
    .source_height     (source_height),
    .frame_pixels      (frame_pixels),
    .frame_top_down    (frame_top_down),
    .frame_writer_busy (bank_loader_busy),
    .frame_writer_sector_ready(bank_loader_sector_ready),
    .frame_writer_done (bank_loader_done),
    .frame_writer_error(bank_writer_error),
    .frame_writer_image_complete(bank_image_complete),
    .pcm_sector_ready  (pcm_sector_ready),
    .pcm_repeat_all    (pcm_repeat_all), .pcm_next_track(pcm_next_track),
    .pcm_previous_track(pcm_previous_track),
    .pcm_playback_done (pcm_playback_done_sd),
    .pcm_buffer_ready  (pcm_buffer_ready), .pcm_buffer_cleared(pcm_buffer_cleared),
    .pcm_buffer_hold   (pcm_buffer_hold),
    .pcm_byte_valid    (pcm_byte_valid),
    .pcm_byte          (pcm_byte),
    .pcm_file_done     (pcm_file_done),
    .pcm_file_error    (pcm_file_error),
    .pcm_play_clk(video_clk), .pcm_play_rst_n(pcm_ring_rd_rst_n),
    .pcm_sample_take(pcm_wav_sample_request && pcm_ring_rd_rst_n && !pcm_ring_done && !pcm_ring_empty),
    .pcm_cue_valid(pcm_cue_valid), .pcm_cue_event(pcm_cue_event),
    .player_next_event (bank_next_event),
    .player_prev_event (bank_prev_event),
    .player_toggle_auto_event(bank_toggle_auto_event),
    .SD_nCS            (sd_ncs),
    .SD_DCLK           (sd_dclk),
    .SD_MOSI           (sd_mosi),
    .SD_MISO           (sd_miso),
    .cache_catalog_valid(cache_catalog_valid),
    .cache_catalog_mask(cache_catalog_mask),
    .cache_catalog_count(cache_catalog_count),
    .cache_catalog_error(cache_catalog_error),
    .cache_load_valid  (cache_load_valid),
    .cache_load_ready  (cache_load_ready),
    .cache_load_image_id(cache_load_id),
    .cache_load_bank   (cache_load_bank),
    .cache_load_done   (cache_load_done),
    .cache_load_success(cache_load_success),
    .cache_load_error  (cache_load_error),
    .cache_job_image_id(cache_job_id),
    .cache_write_flush_req(cache_flush_request),
    .cache_write_flush_done(cache_flush_done),
    .cache_recovery_required(cache_recovery_required)
);

// ===================== 四bank写通路：读卡域 -> SDRAM域 =====================
// File lookup selects an image ID; image_index below is its allocated bank.
media_frame_loader #(
    .ADDR_WIDTH        (ADDR_BITS),
    .IMAGE_WORDS       (307200),
    .FIFO_DEPTH        (512),
    .BURST_SIZE        (128),
    .SECTOR_RESERVE_WORDS(174),
    .STALL_TIMEOUT     (65535),
    .WRITE_V_FLIP      (1),
    .FRAME_WIDTH       (640),
    .FRAME_HEIGHT      (480),
    .RUNTIME_GEOMETRY  (1),
    .BANK_WORDS        (IMAGE_BANK_WORDS)
) u_bank_loader_shadow (
    .clk               (sd_card_clk),
    .rst_n              (~sdcard_module_rst),
    .load_start        (frame_load_start),
    .image_index       (frame_load_index),
    .frame_config_valid(frame_config_valid),
    .frame_width       (frame_width),
    .frame_height      (frame_height),
    .frame_top_down    (frame_top_down),
    .pixel_valid       (sd_card_write_en),
    .pixel_rgb         (sd_card_write_data[31:8]),
    .frame_commit      (frame_commit),
    .decode_error      (bank_cdc_overflow ? 8'h47 : frame_decode_error),
    .write_complete    (bank_loader_write_complete),
    .load_busy         (bank_loader_busy),
    .pixel_sector_ready(bank_loader_sector_ready),
    .load_done         (bank_loader_done),
    .write_commit      (bank_loader_write_commit),
    .image_complete    (bank_image_complete),
    .memory_error      (bank_memory_error),
    .controller_init_done(Sdr_init_done_sd),
    .write_burst_ready (bank_cdc_src_burst_ready),
    .app_wr_en         (bank_loader_wr_en),
    .app_wr_addr       (bank_loader_wr_addr),
    .app_wr_mask       (bank_loader_wr_mask),
    .app_wr_data       (bank_loader_wr_data)
);

// 每次开始一张图时，清除上次失败可能遗留的不足一个burst的命令。
// 只清CDC命令队列，不清loader保存的其他bank完成标志，也不复位SDRAM。
// SD重试同样复位CDC两端；源域释放跟随本域load_start，目的域同步释放。
// Flush reset originates in memory clock: release it synchronously on BOTH
// command-FIFO sides. A frame-start reset also gets this release guard.
reset_sync u_bank_write_src_reset (
    .clk(sd_card_clk),
    .ext_rst_n(~sdcard_module_rst & ~frame_load_start & ~cache_flush_reset),
    .rst_n_sync(bank_write_src_rst_n)
);
reset_sync u_bank_write_mem_reset (
    .clk        (ext_mem_clk),
    .ext_rst_n  (mem_rst_n & bank_write_src_rst_n),
    .rst_n_sync (bank_write_mem_rst_n)
);

media_sdram_write_cdc #(
    .ADDR_WIDTH        (ADDR_BITS),
    .FIFO_ADDR_WIDTH   (9),
    .BURST_SIZE        (128)
) u_bank_write_cdc_shadow (
    .src_clk           (sd_card_clk),
    .src_rst_n         (bank_write_src_rst_n),
    .src_wr_en         (bank_loader_wr_en),
    .src_wr_addr       (bank_loader_wr_addr),
    .src_wr_mask       (bank_loader_wr_mask),
    .src_wr_data       (bank_loader_wr_data),
    .src_full          (bank_cdc_full),
    .src_burst_ready   (bank_cdc_src_burst_ready),
    .src_overflow      (bank_cdc_overflow),
    .src_frame_commit  (bank_loader_write_commit),
    .src_frame_done    (bank_loader_write_complete),
    .dst_clk           (ext_mem_clk),
    .dst_rst_n         (bank_write_mem_rst_n),
    .dst_controller_busy(Sdr_busy),
    .dst_refresh_active (Sdr_init_ref_vld),
    .dst_burst_grant    (bank_cdc_burst_grant & ~cache_block_writes),
    .dst_burst_request  (bank_cdc_burst_request),
    .dst_burst_active   (bank_cdc_burst_active),
    .dst_app_wr_en     (bank_cdc_wr_en),
    .dst_app_wr_addr   (bank_cdc_wr_addr),
    .dst_app_wr_mask   (bank_cdc_wr_mask),
    .dst_app_wr_data   (bank_cdc_wr_data),
    .dst_idle          (bank_cdc_idle)
);

// SD 重试会复位控制域，也必须同时清除视频域的请求/应答状态。
// 视频域仍通过 reset_sync 在 video_clk 上同步释放复位。
reset_sync u_bank_control_video_reset (
    .clk        (video_clk),
    .ext_rst_n  (video_rst_n & ~sdcard_module_rst),
    .rst_n_sync (bank_control_video_rst_n)
);

// Whole-player recovery clears the read handshake and arbitration together.
// SDRAM itself remains initialized; cached contents are invalid until reloaded.
reset_sync u_bank_player_mem_reset (
    .clk(ext_mem_clk), .ext_rst_n(mem_rst_n & ~sdcard_module_rst),
    .rst_n_sync(bank_player_mem_rst_n)
);

media_cache_write_flush u_cache_write_flush (
    .src_clk(sd_card_clk), .src_rst_n(~sdcard_module_rst),
    .flush_request(cache_flush_request),
    .src_writer_idle(~bank_loader_busy & ~bank_loader_wr_en),
    .flush_done(cache_flush_done),
    .mem_clk(ext_mem_clk), .mem_rst_n(bank_player_mem_rst_n),
    .mem_write_active(bank_cdc_burst_active), .mem_write_en(bank_cdc_wr_en),
    .controller_busy(Sdr_busy), .refresh_active(Sdr_init_ref_vld),
    .mem_block_requests(cache_block_writes), .reset_request(cache_flush_reset)
);

// ===================== Dynamic four-bank cache and display transaction =====================
// Raw direction events go only to the scheduler. It retains at most one
// waiting manual direction while an accepted display target stays immutable.
media_image_cache_scheduler #(.MAX_IMAGES(MAX_MEDIA_IMAGES), .ENABLE_CUE_SKIP(1)) u_image_cache (
    .clk(sd_card_clk), .rst_n(~sdcard_module_rst),
    .catalog_valid(cache_catalog_valid), .catalog_image_mask(cache_catalog_mask),
    .next_event(bank_next_event), .prev_event(bank_prev_event),
    .auto_event(bank_auto_next_event),
    .auto_skip_unready(music_cue_mode),
    .load_valid(cache_load_valid), .load_ready(cache_load_ready),
    .load_image_id(cache_load_id), .load_bank(cache_load_bank),
    .load_done(cache_load_done), .load_success(cache_load_success), .load_busy(cache_load_busy),
    .switch_valid(cache_switch_valid), .switch_ready(cache_switch_ready),
    .switch_image_id(cache_switch_id), .switch_bank(cache_switch_bank),
    .switch_initial(cache_switch_initial), .switch_auto(cache_switch_auto),
    .switch_previous(cache_switch_previous), .switch_done(cache_switch_done),
    .switch_busy(cache_switch_busy), .active_valid(cache_active_valid),
    .active_image_id(cache_active_id), .active_bank(cache_active_bank),
    .request_pending(cache_request_pending), .pending_previous(), .pending_auto(),
    .slot_valid_mask(cache_slot_valid), .slot_loading_mask(cache_slot_loading),
    .protected_bank_mask(cache_protected_mask), .slot_image_ids(cache_slot_ids),
    .bad_image_mask(cache_bad_mask), .no_usable_images(cache_no_images)
);

media_cache_auto_timer #(.PERIOD_CYCLES(AUTO_PERIOD_CYCLES)) u_cache_auto_timer (
    .clk(sd_card_clk), .rst_n(~sdcard_module_rst),
    .toggle_event(bank_toggle_auto_event), .display_valid(cache_active_valid),
    .switch_busy(cache_switch_busy), .request_pending(cache_request_pending),
    .manual_event(bank_next_event | bank_prev_event | music_cue_mode),
    .auto_enabled(bank_auto_enabled), .auto_event(bank_auto_timer_event)
);

media_cache_display_control #(
    .IMAGE_ID_WIDTH(IMAGE_ID_WIDTH), .FADE_FRAMES(VIDEO_FADE_FRAMES)
) u_bank_control (
    .sys_clk(sd_card_clk), .sys_rst_n(~sdcard_module_rst),
    .switch_valid(cache_switch_valid), .switch_ready(cache_switch_ready),
    .switch_image_id(cache_switch_id), .switch_bank(cache_switch_bank),
    .switch_initial(cache_switch_initial), .switch_auto(cache_switch_auto),
    .switch_previous(cache_switch_previous), .image_geometry(bank_image_geometry),
    .switch_done(cache_switch_done), .request_busy(bank_control_request_busy),
    .pending_bank(bank_pending_index), .display_valid(display_valid),
    .display_image_id(cache_display_id), .display_bank(cache_display_bank),
    .display_commit(cache_display_commit),
    .pix_clk(video_clk), .pix_rst_n(bank_control_video_rst_n),
    .frame_start(bank_frame_start_shadow), .active_valid(bank_active_ready),
    .active_image_id(bank_active_image_id), .active_bank(bank_active_index),
    .index_commit(bank_index_commit), .active_epoch(bank_active_epoch),
    .active_width(bank_active_width), .active_height(bank_active_height),
    .active_word_count(bank_active_word_count),
    .reader_reload_done(bank_reload_done), .reader_reload_epoch(bank_reload_epoch),
    .reader_reload_bank(bank_reload_bank),
    .fade_busy(bank_fade_busy), .reveal_active(bank_reveal_active),
    .reveal_effect(bank_reveal_effect), .reveal_progress(bank_reveal_progress),
    .fade_level(bank_fade_level)
);

// ===================== Four-bank frame reader =====================
// The unselected reader stays idle; active_index comes from the video domain.
media_frame_reader #(
    .ADDR_WIDTH          (ADDR_BITS),
    .IMAGE_WORDS         (FRAME_PIXELS),
    .FIFO_DEPTH          (512),
    .FIFO_MARGIN         (8),
    .BURST_SIZE          (128),
    .BURST_GAP_CYCLES    (12),
    .MAX_INFLIGHT_BURSTS (2),
    .FRAME_STREAM_HANDSHAKE(1),
    .DIAG_RETURN_PATTERN (0),
    .DIAG_SYNTH_STREAM   (0),
    .RUNTIME_GEOMETRY    (1),
    .BANK_WORDS          (IMAGE_BANK_WORDS),
    .DISPLAY_EPOCH_TAG   (1)
) u_bank_reader_shadow (
    .mem_clk             (ext_mem_clk),
    .mem_rst_n           (bank_player_mem_rst_n),
    .controller_init_done(Sdr_init_done & bank_reader_path_select &
                          bank_any_complete_mem),
    .controller_busy     (Sdr_busy),
    .refresh_active      (Sdr_init_ref_vld),
    .burst_grant         (bank_reader_burst_grant),
    .burst_request       (bank_reader_burst_request),
    .burst_active        (bank_reader_burst_active),
    .app_rd_en           (bank_reader_app_rd_en),
    .app_rd_addr         (bank_reader_app_rd_addr),
    // Only the selected reader is allowed to accept SDRAM return beats.
    .sdram_rd_valid      (bank_reader_sdram_rd_valid),
    .sdram_rd_data       (Sdr_rd_dout),
    .pix_clk             (video_clk),
    .pix_rst_n           (bank_control_video_rst_n),
    .frame_start         (bank_frame_start_shadow & bank_reader_path_select &
                          bank_display_valid_video & bank_active_ready),
    .active_index        (bank_active_index),
    .active_word_count   (bank_active_word_count),
    .active_epoch        (bank_active_epoch),
    .reload_done         (bank_reload_done),
    .reload_epoch        (bank_reload_epoch),
    .reload_bank         (bank_reload_bank),
    .pixel_consume       (bank_pixel_consume_shadow & bank_reader_path_select &
                          bank_display_valid_video & bank_active_ready),
    .pixel_rgb           (bank_reader_pixel_rgb),
    .pixel_valid         (bank_reader_pixel_valid),
    .frame_stream_start  (bank_frame_stream_start),
    .frame_stream_ready  (bank_frame_stream_ready),
    .underflow           (bank_reader_underflow),
    .reader_busy         (bank_reader_busy),
    .frame_read_done     (bank_reader_done),
    .reader_index        (bank_reader_index),
    .reader_error        (bank_reader_error),
    .diag_reload_pending (),
    .diag_inflight_pending(),
    .diag_request_inflight(),
    .diag_fifo_reset     ()
);

// ===================== 原图流 -> 720p 等比缩放 =====================
// reader 清空旧帧 FIFO 并确认 bank 后，才允许缩放器接收新帧。
// 三行缓存完成双线性等比放大，显示区域之外填黑。
video_scale_bilinear u_bank_scaler (
    .pclk              (video_clk),
    .rst_n             (bank_control_video_rst_n),
    .video_rst_n       (video_rst_n),
    .frame_start       (bank_frame_start_shadow),
    .frame_enable      (bank_display_valid_video & bank_active_ready),
    .source_width      (bank_active_width),
    .source_height     (bank_active_height),
    .source_frame_start(bank_frame_stream_start),
    .source_valid      (bank_reader_pixel_valid),
    .source_rgb        (bank_reader_pixel_rgb),
    .source_ready      (bank_scale_source_ready),
    .in_hs             (bank_hs_shadow),
    .in_vs             (bank_vs_shadow),
    .in_de             (bank_de_shadow),
    .in_x              (bank_x_shadow - 12'd370),
    .in_y              (bank_y_shadow - 12'd30),
    .out_hs            (bank_scaled_hs),
    .out_vs            (bank_scaled_vs),
    .out_de            (bank_scaled_de),
    .out_x             (bank_scaled_x),
    .out_rgb           (bank_scaled_rgb),
    .out_image_valid   (bank_scaled_image_valid),
    .underflow         (bank_scale_underflow),
    .config_error      (bank_scale_config_error),
    .image_width       (bank_scaled_image_width),
    .image_left        (bank_scaled_image_left),
    .image_height      (bank_scaled_image_height),
    .image_top         (bank_scaled_image_top)
);

// 遮罩与缩放器共用原始画布坐标、五拍延迟，只流水一个可见位。
// 不增加整套 RGB/同步信号/坐标流水线，也不增加 SDRAM 读取。
video_reveal_mask u_reveal_mask (
    .pclk(video_clk), .video_rst_n(video_rst_n),
    .reveal_active(bank_reveal_active), .effect(bank_reveal_effect),
    .progress(bank_reveal_progress),
    .image_width(bank_scaled_image_width), .image_height(bank_scaled_image_height),
    .image_left(bank_scaled_image_left), .image_top(bank_scaled_image_top),
    .in_x(bank_x_shadow-12'd370), .in_y(bank_y_shadow-12'd30),
    .show_pixel(bank_reveal_show_pixel)
);
// Fold reveal-to-black into the shared coefficient instead of masking three
// separate RGB results. Products feed the overlay's EXISTING first register.
// 8x5-bit products preserve every channel at full brightness. Division
// by 16 is just wiring; endpoints are exactly black and the unmodified image.
// No extra RGB/coordinate/sync pipeline or SDRAM read is needed.
function [7:0] fade_channel;
    input [7:0] colour;
    input [4:0] level;
    reg [12:0] product;
    begin
        product=colour*level;
        fade_channel=product[11:4];
    end
endfunction
wire [4:0] picture_level = bank_reveal_show_pixel ? bank_fade_level : 5'd0;
assign bank_rgb_after_transition=
    {fade_channel(bank_scaled_rgb[23:16],picture_level),
     fade_channel(bank_scaled_rgb[15:8],picture_level),
     fade_channel(bank_scaled_rgb[7:0],picture_level)};

// 音频轮廓叠加在转场之后，图片被遮黑时仍能随音乐显示。
// 使用缩放器已经对齐的坐标；绘图的三级流水线同时延迟 RGB/HS/VS/DE。
// SW1 只控制叠加显示；音频播放和历史采样持续运行。
// 开关先同步并稳定 20 ms，overlay 再在帧边界接纳该显示状态。
switch_level_debounce #(
    .STABLE_CYCLES((VIDEO_CLOCK_HZ / 1000) * 20)
) u_waveform_switch (
    .clk          (video_clk),
    .rst_n        (video_rst_n),
    .level_async  (waveform_sw),
    .level_stable (waveform_visible)
);

video_waveform_overlay u_waveform_overlay (
    .pclk         (video_clk),
    .rst_n        (video_rst_n),
    .enable       (audio_playback_enabled && waveform_visible),
    .history_peaks(waveform_history),
    .in_hs        (bank_scaled_hs),
    .in_vs        (bank_scaled_vs),
    .in_de        (bank_scaled_de),
    .in_x         (bank_scaled_x),
    .in_rgb       (bank_rgb_after_transition),
    .out_hs       (bank_waveform_hs),
    .out_vs       (bank_waveform_vs),
    .out_de       (bank_waveform_de),
    .out_rgb      (bank_waveform_rgb),
    .overlay_active()
);

// 适配器使用叠加后的成套颜色和同步信号，再一起寄存一拍。
rgb_video_adapter u_bank_video_adapter (
    .pclk       (video_clk),
    .rst_n      (video_rst_n),
    .hs_in      (bank_waveform_hs),
    .vs_in      (bank_waveform_vs),
    .de_in      (bank_waveform_de),
    .rgb_in     (bank_waveform_rgb),
    .video_clk  (),
    .video_hs   (bank_video_hs),
    .video_vs   (bank_video_vs),
    .video_de   (bank_video_de),
    .video_rgb  (bank_video_rgb)
);

// User panel follows the acknowledged pixel commit, not background prefetch.
// All metadata is in sd_card_clk; no new multi-bit clock crossing is needed.
wire panel_rst_n;
reset_sync u_panel_reset (
    .clk(sd_card_clk), .ext_rst_n(~sdcard_module_rst), .rst_n_sync(panel_rst_n)
);
image_info_panel u_image_info_panel (
    .clk(sd_card_clk), .rst_n(panel_rst_n),
    .metadata_valid(frame_config_valid), .metadata_bank(frame_load_index),
    .source_width(source_width[10:0]), .source_height(source_height[10:0]),
    .display_valid(display_valid), .display_commit(cache_display_commit),
    .display_bank(cache_display_bank), .display_id(cache_display_id),
    .seg_sel(seg_sel), .seg_data(seg_data)
);

// ===================== 原图像时序与帧缓存 =====================
video_timing_data video_timing_data_m0(
    .video_clk         (video_clk),
    .rst               (~video_rst_n),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0)
);

video_delay video_delay_m0(
    .video_clk         (video_clk),
    .rst               (~video_rst_n),
    .read_en           (video_read_en),
    .read_data         (video_read_data[31:8]),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0),
    .hs_r              (legacy_hs),
    .vs_r              (legacy_vs),
    .de_r              (legacy_de),
    .vout_data         (legacy_vout_data_raw)
);

// 和 SDRAM 读请求共用编译期选择，防止时序与像素来自不同链路。
assign hs = bank_reader_path_select ? bank_video_hs : legacy_hs;
assign vs = bank_reader_path_select ? bank_video_vs : legacy_vs;
assign de = bank_reader_path_select ? bank_video_de : legacy_de;
assign vout_data_raw = bank_reader_path_select ?
                       bank_video_rgb : legacy_vout_data_raw;

// 首图提交前黑屏；提交后一直显示当前显示缓冲区内容
assign vout_data = video_display_valid ? vout_data_raw : 24'd0;

frame_read_write #(
    .WRITE_V_FLIP     (1),
    .FRAME_WIDTH      (640),
    .FRAME_HEIGHT     (480)
) frame_read_write_m0(
    .mem_clk           (ext_mem_clk),
    .rst               (~mem_rst_n),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),

    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (legacy_reader_sdram_rd_valid),
    .Sdr_rd_dout       (Sdr_rd_dout),

    .read_clk          (video_clk),
    .read_req          (legacy_video_read_req),
    .read_req_ack      (video_read_req_ack),
    .read_finish       (),
    .read_addr_0       (BUF0_ADDR),
    .read_addr_1       (BUF1_ADDR),
    .read_addr_2       (24'd0),
    .read_addr_3       (24'd0),
    .read_addr_index   (disp_buf_idx),
    .read_len          (FRAME_PIXELS),
    .read_en           (video_read_en),
    .read_data         (video_read_data),

    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_din        (App_wr_din),
    .App_wr_dm         (App_wr_dm),

    .write_clk         (sd_card_clk),
    .write_req         (legacy_sd_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_finish      (frame_write_finish),
    .write_addr_0      (BUF0_ADDR),
    .write_addr_1      (BUF1_ADDR),
    .write_addr_2      (24'd0),
    .write_addr_3      (24'd0),
    .write_addr_index  (write_buf_idx),
    .write_len         (FRAME_PIXELS),
    .write_en          (legacy_sd_write_en),
    .write_data        (sd_card_write_data)
);

sdram U3(
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (~mem_rst_n),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (mem_app_wr_en),
    .App_wr_addr       (mem_app_wr_addr),
    .App_wr_dm         (mem_app_wr_mask),
    .App_wr_din        (mem_app_wr_data),
    .App_rd_en         (mem_app_rd_en),
    .App_rd_addr       (mem_app_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

// ===================== 音频：内部 I2S 测试音 =====================
hdmi_audio_tone_i2s_64fs #(
    .PHASE_INC (32'd39370534),
    .AMP       (24'sd2000000)
) u_hdmi_audio_tone_i2s_64fs (
    .I_mclk      (audio_mclk),
    .I_rst       (~audio_rst_n),
    .O_i2s_BCLK  (audio_i2s_bclk),
    .O_i2s_LRCK  (audio_i2s_lrck),
    .O_i2s_DOUT  (audio_i2s_dout)
);

I2S_receiver u_I2S_receiver(
    .I_clk              (video_clk),
    .I_rst              (~video_rst_n),
    .I_i2s_BCLK         (audio_i2s_bclk),
    .I_i2s_LRCK         (audio_i2s_lrck),
    .I_i2s_DOUT         (audio_i2s_dout),
    .O_audio_valid      (audio_i2s_valid),
    .O_audio_left_data  (audio_i2s_left_data),
    .O_audio_right_data (audio_i2s_right_data)
);

// First-picture pixels alone do not mean the monitor has unmuted its speaker.
// Keep silent PCM/ACR and the file read pointer held for a startup allowance
// AFTER image pixels begin. The image path itself is never gated by this wait.
// A local reset branch limits fanout of the main pixel-domain reset. It
// asserts immediately and releases two pixel clocks later for audio logic.
wire audio_pixel_rst_n;
reset_sync u_audio_pixel_reset (
    .clk(video_clk), .ext_rst_n(video_rst_n), .rst_n_sync(audio_pixel_rst_n)
);
// All bank-side inputs below are already in video_clk. This is a conservative
// receiver settling allowance, NOT a display acknowledgement. Start counting
// only after a valid image is ready, so SD/image loading cannot consume it.
// Silent PCM/ACR continues while the mode selector holds the file read pointer.
localparam integer AUDIO_STARTUP_COUNT_BITS = AUDIO_STARTUP_FRAMES < 2 ? 1 :
                                              $clog2(AUDIO_STARTUP_FRAMES);
reg [AUDIO_STARTUP_COUNT_BITS-1:0] audio_startup_frames;
reg audio_frame_has_image;
reg audio_vs_old;
wire audio_image_pixel = bank_reader_path_select ?
                        (bank_scaled_de && bank_scaled_image_valid) : de;
wire audio_frame_start = bank_reader_path_select ?
                        bank_frame_start_shadow : (audio_vs_old && !vs);
wire audio_image_frame_error = bank_reader_path_select &&
                               (bank_scale_underflow || bank_scale_config_error);
always @(posedge video_clk or negedge audio_pixel_rst_n) begin
    if (!audio_pixel_rst_n) begin
        audio_vs_old <= 1'b1;
        audio_playback_enabled <= 1'b0;
        audio_startup_frames <= 0;
        audio_frame_has_image <= 1'b0;
    end else begin
        audio_vs_old <= vs;
        if (!video_display_valid || (bank_reader_path_select && !bank_active_ready)) begin
            audio_playback_enabled <= 1'b0;
            audio_startup_frames <= 0;
            audio_frame_has_image <= 1'b0;
        end else if (!audio_playback_enabled) begin
            // Reacquire the full allowance if TX lock drops before playback.
            // Once playing, ordinary image blanking/transitions do not rearm it.
            if (!hdmi_video_locked) begin
                audio_startup_frames <= 0;
                audio_frame_has_image <= 1'b0;
            end else if (AUDIO_STARTUP_FRAMES == 0) begin
                if (audio_image_pixel) audio_playback_enabled <= 1'b1;
            end else if (audio_frame_start) begin
                audio_frame_has_image <= 1'b0;
                if (audio_frame_has_image && !audio_image_frame_error) begin
                    if (audio_startup_frames == AUDIO_STARTUP_FRAMES - 1)
                        audio_playback_enabled <= 1'b1;
                    else audio_startup_frames <= audio_startup_frames + 1'b1;
                end else audio_startup_frames <= 0;
            end else if (audio_image_pixel) audio_frame_has_image <= 1'b1;
        end
    end
end

// 新正弦测试音已经转换为当前 HDMI 核所需的 24 位 PCM。
// 旧 HDMI 核没有 audio_ready，因此本验证阶段把下游视为始终就绪；
// 适配器仍保留 ready 端口，后续换成音频包链路时可直接接入真实握手。
pcm_test_tone_to_pcm #(
    .SAMPLE_DIV (1542), // 74 MHz / 1542 = 47.9896 kHz, integer divider.
    .NOTE_HOLD_SAMPLES (24000) // C4..C5, approximately 0.5 s each, repeat.
) u_pcm_test_tone_to_pcm (
    .clk          (video_clk),
    .rst_n        (audio_pixel_rst_n),
    .enable       (audio_playback_enabled),
    .pcm_valid    (pcm_test_valid),
    .pcm_ready    (1'b1),
    .pcm_left     (pcm_test_left_data),
    .pcm_right    (pcm_test_right_data),
    .pcm_accepted (pcm_test_accepted),
    .rom_index    (pcm_test_rom_index)
);

// bank_active_image_id 在 video_clk 域的帧边界提交，直接驱动同域音源。
// 使用显示编号而非后台预加载编号，避免加载其他 bank 时提前换音调。
// 琶音档内切图才播放一遍六音符；进入档位本身不触发，播完持续输出零 PCM。
pcm_media_tone #(
    .CLOCK_HZ  (VIDEO_CLOCK_HZ),
    .SAMPLE_HZ (48000),
    .MEDIA_ID_WIDTH(IMAGE_ID_WIDTH)
) u_pcm_media_tone (
    .clk          (video_clk),
    .rst_n        (audio_pixel_rst_n),
    .enable       (audio_playback_enabled && audio_mode_active == 2'b01),
    .media_id     (bank_active_image_id),
    .volume       (8'd128),
    .sample_valid (pcm_media_valid),
    .sample_ready (1'b1),
    .sample_left  (pcm_media_left),
    .sample_right (pcm_media_right),
    .overflow     (pcm_media_overflow),
    .playback_active(pcm_media_playback_active)
);

// PCM bytes arrive in sd_card_clk; the same ring is read in video_clk.
// Recovery resets BOTH pointers, then releases reset in each local domain.
// A song change resets only the audio ring. The accepted SD sector drains
// through the arbiter, and HDMI sync / image jobs continue uninterrupted.
assign pcm_ring_reset_n = !sdcard_module_rst && video_rst_n && !pcm_buffer_hold;
switch_level_debounce #(.STABLE_CYCLES(2000000)) u_song_order_switch (
    .clk(sd_card_clk), .rst_n(!sdcard_module_rst),
    .level_async(song_order_sw), .level_stable(pcm_repeat_all)
);
switch_level_debounce #(.STABLE_CYCLES(2000000)) u_song_next_key (
    .clk(sd_card_clk), .rst_n(!sdcard_module_rst),
    .level_async(!key4), .level_stable(pcm_next_pressed)
);
always @(posedge sd_card_clk or posedge sdcard_module_rst) begin
    if(sdcard_module_rst) begin
        pcm_next_pressed_old<=0;
        pcm_next_track<=0;pcm_previous_track<=0;
        pcm_song_press_armed<=0;pcm_song_long_marked<=0;pcm_song_hold_count<=0;
    end else begin
        pcm_next_pressed_old<=pcm_next_pressed;
        pcm_next_track<=0;pcm_previous_track<=0;
        // Match KEY3: short action on release; long action once at threshold.
        // A gesture begun outside WAV, or interrupted by leaving WAV, stays
        // disarmed until release, even if the user re-enters WAV while held.
        if(!pcm_next_pressed) begin
            pcm_song_hold_count<=0;
            if(pcm_next_pressed_old && pcm_song_press_armed &&
               !pcm_song_long_marked && pcm_wav_mode_sd)
                pcm_next_track<=1;
            pcm_song_press_armed<=0;
            pcm_song_long_marked<=0;
        end else if(!pcm_next_pressed_old) begin
            pcm_song_press_armed<=pcm_wav_mode_sd;
            pcm_song_hold_count<=0;
            pcm_song_long_marked<=0;
        end else if(!pcm_wav_mode_sd) begin
            pcm_song_press_armed<=0;
            pcm_song_hold_count<=0;
        end else if(pcm_song_press_armed && !pcm_song_long_marked) begin
            if(pcm_song_hold_count < SONG_LONG_PRESS_CYCLES-1)
                pcm_song_hold_count<=pcm_song_hold_count+1'b1;
            else begin
                pcm_song_long_marked<=1;
                pcm_previous_track<=1;
            end
        end
    end
end
sync_2ff u_pcm_ring_ready_sync (
    .clk(sd_card_clk), .rst_n(!sdcard_module_rst),
    .async_in(pcm_ring_rd_rst_n), .sync_out(pcm_ring_rd_ready_sd)
);
sync_2ff u_pcm_playback_done_sync (
    .clk(sd_card_clk), .rst_n(!sdcard_module_rst),
    .async_in(pcm_ring_done), .sync_out(pcm_playback_done_sd)
);
sync_2ff u_pcm_wav_mode_sync (
    .clk(sd_card_clk), .rst_n(!sdcard_module_rst),
    .async_in(audio_mode_active==2'b10), .sync_out(pcm_wav_mode_sd)
);
assign pcm_buffer_ready=pcm_ring_wr_rst_n && pcm_ring_rd_ready_sd;
assign pcm_buffer_cleared=!pcm_ring_wr_rst_n && !pcm_ring_rd_ready_sd;
reset_sync u_pcm_write_reset (
    .clk(sd_card_clk), .ext_rst_n(pcm_ring_reset_n),
    .rst_n_sync(pcm_ring_wr_rst_n)
);
reset_sync u_pcm_read_reset (
    .clk(video_clk), .ext_rst_n(pcm_ring_reset_n),
    .rst_n_sync(pcm_ring_rd_rst_n)
);

pcm_async_ring_buffer u_pcm_ring (
    .wr_clk           (sd_card_clk),
    .wr_rst_n         (pcm_ring_wr_rst_n),
    .wr_byte_valid    (pcm_byte_valid),
    .wr_byte          (pcm_byte),
    .wr_file_done     (pcm_file_done),
    .wr_sector_ready  (pcm_sector_ready),
    .wr_overflow      (pcm_ring_overflow),
    .wr_format_error  (pcm_ring_format_error),
    .rd_clk           (video_clk),
    .rd_rst_n         (pcm_ring_rd_rst_n),
    // Reuse the continuous 48 kHz cadence of the media generator.
    // Pausing the file source must not pause HDMI silent PCM/ACR packets.
    .rd_sample_tick   (pcm_wav_sample_request && !pcm_ring_done),
    .rd_sample_valid  (pcm_wav_sample_valid),
    .rd_sample        (pcm_wav_sample),
    .rd_empty         (pcm_ring_empty),
    .rd_playback_done (pcm_ring_done),
    .rd_underflow     (pcm_ring_underflow)
);

// Match the synchronous BRAM read latency, including silence during reset,
// missing files and EOF. One mono sample goes to both HDMI channels.
always @(posedge video_clk or negedge audio_pixel_rst_n) begin
    if (!audio_pixel_rst_n)
        pcm_wav_valid <= 1'b0;
    else
        pcm_wav_valid <= pcm_wav_sample_request;
end
assign pcm_wav_data = (pcm_ring_rd_rst_n && pcm_wav_sample_valid) ?
                      {pcm_wav_sample, 8'h00} : 24'd0;

// 测试音和琶音持续 ready=1；PCM文件只在WAV档推进读取指针。
// 选择器同步/消抖拨码，并把完整的 24 位样本和 valid 一起寄存输出。
// 10 为WAV/PCM播放，11保留连续静音PCM。
pcm_audio_mode_select #(
    .MODE_STABLE_CYCLES(VIDEO_CLOCK_HZ / 50), // 20 ms
    .MIN_SAMPLE_CYCLES (1541)
) u_pcm_audio_mode_select (
    .clk          (video_clk),
    .rst_n        (audio_pixel_rst_n),
    .playback_enable(audio_playback_enabled),
    .mode_async   (audio_mode_sw),
    .test_valid   (pcm_test_valid),
    .test_left    (pcm_test_left_data),
    .test_right   (pcm_test_right_data),
    .media_valid  (pcm_media_valid),
    .media_left   ({pcm_media_left, 8'h00}),
    .media_right  ({pcm_media_right, 8'h00}),
    .wav_valid    (pcm_wav_valid),
    .wav_left     (pcm_wav_data),
    .wav_right    (pcm_wav_data),
    .wav_sample_request(pcm_wav_sample_request),
    .mode_active  (audio_mode_active),
    .pcm_valid    (audio_valid),
    .pcm_left     (audio_left_data),
    .pcm_right    (audio_right_data)
);

// 旁路观察最终送往 HDMI 核的 PCM，不影响音源选择和播放器取样。
// 当前 16 位音频左对齐在 24 位接口中，所以取 [23:8]，不能取低 16 位。
pcm_waveform_history u_waveform_history (
    .clk          (video_clk),
    .rst_n        (audio_pixel_rst_n),
    .pcm_valid    (audio_valid),
    .pcm_sample   (audio_left_data[23:8]),
    .bar_valid    (),
    .bar_peak     (),
    .history_count(),
    .history_peaks(waveform_history)
);

audio_arc_calculate #(
    .ACR_N         (6144)
) u_audio_arc_calculate (
    .I_clk         (video_clk),
    .I_rst         (~audio_pixel_rst_n),
    .I_audio_valid (audio_valid),
    .O_acr_valid   (acr_valid),
    .O_acr_cts     (acr_cts),
    .O_acr_n       (acr_n)
);

// ===================== RGB/DE 转 AXIS 视频 =====================
video_rgb_to_axis_640x480 #(.H_ACTIVE(1280)) u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),
    .I_rst         (~video_rst_n),
    .I_vs          (vs),
    .I_de          (de),
    .I_rgb         (vout_data),
    .O_video_user  (axis_s_user),
    .O_video_valid (axis_s_valid),
    .O_video_last  (axis_s_last),
    .O_video_data  (axis_s_data)
);

// 上电后自动打一拍，触发一次 EDID 读取
startup_pulse #(
    .CNT_MAX(20'd100000)
) u_startup_pulse (
    .I_clk   (video_clk),
    .I_rst   (~video_rst_n),
    .O_pulse (edid_trig)
);

// ===================== 带音频的 HDMI 1.4b 发射 =====================
hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ),
    .HTOTAL                 ( 1650       ),
    .HSA                    ( 40         ),
    .HFP                    ( 110        ),
    .HBP                    ( 220        ),
    .HACTIVE                ( 1280       ),
    .VTOTAL                 ( 750        ),
    .VSA                    ( 5          ),
    .VFP                    ( 5          ),
    .VBP                    ( 20         ),
    .VACTIVE                ( 720        ),
    .VIDEO_VIC              ( 4          ),
    .VIDEO_TPG              ( "Disable"  ),
    .VIDEO_FORMAT           ( "RGB"      ),
    .AUDIO_SAMPLE_RATE      ( "48K"      ),
    .IIC_SCL_DIV            ( 740        )
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),
    .I_rst              (~video_rst_n),
    .I_edid_read_trig   (edid_trig),
    .O_edid_read_valid  (edid_valid),
    .O_edid_read_data   (edid_data),

    .I_axis_s_user      (axis_s_user),
    .I_axis_s_valid     (axis_s_valid),
    .I_axis_s_last      (axis_s_last),
    .I_axis_s_data      (axis_s_data),
    .O_axis_s_ready     (axis_s_ready),

    .I_audio_valid      (audio_valid),
    .I_audio_left_data  (audio_left_data),
    .I_audio_right_data (audio_right_data),
    .I_acr_valid        (acr_valid),
    .I_acr_cts          (acr_cts),
    .I_acr_n            (acr_n),

    .O_video_locked     (hdmi_video_locked),
    .O_ddc_scl          (HDMI_DDC_SCL),
    .IO_ddc_sda         (HDMI_DDC_SDA),

    .O_ch0_tmds_data    (tmds_ch0_data),
    .O_ch1_tmds_data    (tmds_ch1_data),
    .O_ch2_tmds_data    (tmds_ch2_data),
    .O_clk_tmds_data    (tmds_clk_data)
);

hdmi_720p_phy u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_pixel_rst        (~video_rst_n),
    .I_serial_rst       (~serial_rst_n),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

endmodule
