`timescale 1ns/1ps

// 来源：第8课配套 media_frame_loader.v；本工程增加 BMP 垂直翻转地址映射。
// 本模块所有输入/输出都属于 clk 域；跨时钟域的像素必须先经过 CDC。
// load_start 接受后的下一拍才能送像素；frame_commit 仅用于成功结束的像素流。
// 开启 WRITE_V_FLIP 时，IMAGE_WORDS 必须等于 FRAME_WIDTH * FRAME_HEIGHT。
// EFFECTIVE_BURST 必须是4的倍数：控制器每4个字执行一次物理突发。
module media_frame_loader #(
    parameter integer ADDR_WIDTH = 21,
    parameter integer IMAGE_WORDS = 307200,
    parameter integer FIFO_DEPTH = 512,
    parameter integer BURST_SIZE = 128,
    parameter integer SECTOR_RESERVE_WORDS = 174,
    parameter integer STALL_TIMEOUT = 65535,
    parameter integer WRITE_V_FLIP = 1,
    parameter integer FRAME_WIDTH = 640,
    parameter integer FRAME_HEIGHT = 480,
    parameter integer RUNTIME_GEOMETRY = 0,
    parameter integer BANK_WORDS = IMAGE_WORDS
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  load_start,
    input  wire [1:0]            image_index,
    input  wire                  pixel_valid,
    input  wire [23:0]           pixel_rgb,
    input  wire                  frame_commit,
    input  wire [7:0]            decode_error,
    input  wire                  write_complete,
    output wire                  load_busy,
    output wire                  pixel_sector_ready,
    output reg                   load_done,
    output reg                   write_commit,
    output reg  [3:0]            image_complete,
    output reg  [7:0]            memory_error,
    input  wire                  controller_init_done,
    input  wire                  write_burst_ready,
    output reg                   app_wr_en,
    output reg  [ADDR_WIDTH-1:0] app_wr_addr,
    output reg  [3:0]            app_wr_mask,
    output reg  [31:0]           app_wr_data,
    // Runtime mode: reserve the bank with load_start BEFORE reading its header.
    // Send configuration once after header validation and before the first pixel.
    input  wire                 frame_config_valid,
    input  wire [15:0]          frame_width,
    input  wire [15:0]          frame_height,
    input  wire                 frame_top_down
);
    localparam ERR_NONE          = 8'h00;
    localparam ERR_NOT_ACTIVE    = 8'h41;
    localparam ERR_FIFO_OVERFLOW = 8'h42;
    localparam ERR_PIXEL_COUNT   = 8'h43;
    localparam ERR_DECODE        = 8'h44;
    localparam ERR_SHORT_FRAME   = 8'h45;
    localparam ERR_WRITE_STALL   = 8'h46;
    localparam ERR_CONFIG        = 8'h47;

    localparam integer PTR_WIDTH = (FIFO_DEPTH <= 2) ? 1 : $clog2(FIFO_DEPTH);
    localparam [PTR_WIDTH:0] EFFECTIVE_BURST =
        (BURST_SIZE < FIFO_DEPTH) ? BURST_SIZE : FIFO_DEPTH;
    // 地址单位是一个32位字（一个像素），不是字节。
    localparam [ADDR_WIDTH-1:0] BANK0_BASE = 0;
    localparam [ADDR_WIDTH-1:0] BANK1_BASE = BANK_WORDS;
    localparam [ADDR_WIDTH-1:0] BANK2_BASE = BANK_WORDS * 2;
    localparam [ADDR_WIDTH-1:0] BANK3_BASE = BANK_WORDS * 3;
    localparam [ADDR_WIDTH-1:0] VFLIP_FIRST_OFFSET =
        (FRAME_HEIGHT - 1) * FRAME_WIDTH;
    localparam [ADDR_WIDTH-1:0] VFLIP_LINE_JUMP = FRAME_WIDTH * 2 - 1;

    reg active;
    reg [1:0] active_image;
    reg commit_seen;
    reg [31:0] pixels_received;
    reg [31:0] pixels_written;
    reg [ADDR_WIDTH-1:0] write_offset;
    reg [15:0] write_x;
    reg [31:0] stall_count;
    reg [31:0] fifo_mem [0:FIFO_DEPTH-1];
    reg [PTR_WIDTH-1:0] wr_ptr;
    reg [PTR_WIDTH-1:0] rd_ptr;
    reg [PTR_WIDTH:0] fifo_count;
    reg burst_active;
    reg [PTR_WIDTH:0] burst_remaining;
    reg [3:0] command_phase;
    reg row_padding;
    reg geometry_ready;
    reg [31:0] frame_words_q;
    reg [15:0] frame_width_q;
    reg flip_q;
    reg [ADDR_WIDTH-1:0] line_jump_q;
    wire [19:0] config_words = frame_width[9:0] * frame_height[9:0];
    // Every legal geometry fits when the bank holds the maximum 960x540.
    // Resolve this at elaboration so the pixel-count multiplier does not
    // feed the per-cycle FIFO/error-control path for full-size banks.
    // Smaller banks still need the runtime capacity check.
    wire config_fits_bank = (BANK_WORDS >= 960 * 540) ?
                            1'b1 : (config_words <= BANK_WORDS);
    wire config_ok = frame_width != 0 && frame_width <= 960 &&
                     frame_height != 0 && frame_height <= 540 && config_fits_bank;
    wire [31:0] expected_words = RUNTIME_GEOMETRY ? frame_words_q : IMAGE_WORDS;
    wire [15:0] row_width = RUNTIME_GEOMETRY ? frame_width_q : FRAME_WIDTH;
    wire flip_rows = RUNTIME_GEOMETRY ? flip_q : WRITE_V_FLIP;
    wire [ADDR_WIDTH-1:0] line_jump = RUNTIME_GEOMETRY ? line_jump_q : VFLIP_LINE_JUMP;
    wire geometry_available = !RUNTIME_GEOMETRY || geometry_ready;
    // The real controller samples one start address per four data words and
    // pipelines the next bank activation before the current WRITE ends.
    // Rotate through all four physical banks (address bits 3:2), including
    // masked groups, so a reversed row cannot activate the same bank early.
    // Masked lanes do not consume FIFO pixels or change the packed layout.
    wire [ADDR_WIDTH-1:0] pixel_address = bank_base(active_image) + write_offset;
    wire pop_allowed;
    wire command_emit = active && controller_init_done && burst_active &&
                        (fifo_count != 0 ||
                         (commit_seen && pixels_written == expected_words));
    wire push_allowed;
    wire start_burst;
    wire [PTR_WIDTH:0] next_burst_words;

    assign load_busy = active;
    // A 512-byte RGB24 sector can complete at most 171 pixels, including
    // two bytes left from the previous pixel. Reserve three extra entries
    // for the preceding sector's streamer/parser output pipeline.
    // When this output controls real sector admission, the configuration
    // must satisfy FIFO_DEPTH - SECTOR_RESERVE_WORDS >= EFFECTIVE_BURST - 1,
    // so waiting for a full write burst cannot block the next input sector.
    assign pixel_sector_ready = rst_n && active &&
                                (FIFO_DEPTH >= SECTOR_RESERVE_WORDS) &&
                                (fifo_count <= FIFO_DEPTH - SECTOR_RESERVE_WORDS);
    // This module submits commands to the same-clock side of the CDC FIFO.
    // Reserve room for a complete group before starting; ready is NOT a
    // per-word handshake and must not split an active group. SDRAM busy and
    // refresh admission are handled by the destination-domain arbiter.
    assign start_burst = active && geometry_available && controller_init_done && !burst_active &&
                         write_burst_ready &&
                         fifo_count != 0 &&
                         (fifo_count >= EFFECTIVE_BURST || commit_seen);
    assign next_burst_words = EFFECTIVE_BURST;
    assign pop_allowed = command_emit && fifo_count != 0 && !row_padding &&
                         command_phase == pixel_address[3:0];
    assign push_allowed = active && geometry_available && pixel_valid &&
                          (fifo_count < FIFO_DEPTH || pop_allowed);

    function [ADDR_WIDTH-1:0] bank_base;
        input [1:0] index;
        begin
            case (index)
                2'd0: bank_base = BANK0_BASE;
                2'd1: bank_base = BANK1_BASE;
                2'd2: bank_base = BANK2_BASE;
                default: bank_base = BANK3_BASE;
            endcase
        end
    endfunction

    task fail_load;
        input [7:0] code;
        begin
            memory_error <= code;
            active <= 1'b0;
            burst_active <= 1'b0;
            burst_remaining <= 0;
            write_commit <= 1'b0;
            load_done <= 1'b1;
        end
    endtask

    // Read data every cycle. Only app_wr_en marks a valid command, so idle
    // data need not be held. This removes the long admission/error-control
    // path from the FIFO data register's high-fanout clock enable, without
    // adding a cycle to valid command data or changing the read pointer.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            app_wr_data <= 32'd0;
        end else begin
            // Keep one unconditional registered RAM read (preserves BRAM).
            // On masked commands the data is ignored and rd_ptr stays put.
            app_wr_data <= fifo_mem[rd_ptr];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active <= 1'b0;
            active_image <= 2'd0;
            commit_seen <= 1'b0;
            pixels_received <= 32'd0;
            pixels_written <= 32'd0;
            write_offset <= 0;
            write_x <= 16'd0;
            stall_count <= 32'd0;
            wr_ptr <= {PTR_WIDTH{1'b0}};
            rd_ptr <= {PTR_WIDTH{1'b0}};
            fifo_count <= {(PTR_WIDTH+1){1'b0}};
            burst_active <= 1'b0;
            burst_remaining <= 0;
            command_phase <= 0;
            row_padding <= 1'b0;
            geometry_ready <= 0;
            frame_words_q <= 0;
            frame_width_q <= 0;
            flip_q <= 0;
            line_jump_q <= 0;
            load_done <= 1'b0;
            write_commit <= 1'b0;
            image_complete <= 4'd0;
            memory_error <= ERR_NONE;
            app_wr_en <= 1'b0;
            app_wr_addr <= {ADDR_WIDTH{1'b0}};
            app_wr_mask <= 4'b0000;
        end else begin
            load_done <= 1'b0;
            app_wr_en <= 1'b0;

            if (load_start && !active) begin
                active <= 1'b1;
                active_image <= image_index;
                commit_seen <= 1'b0;
                pixels_received <= 32'd0;
                pixels_written <= 32'd0;
                write_offset <= WRITE_V_FLIP ? VFLIP_FIRST_OFFSET : 0;
                write_x <= 16'd0;
                stall_count <= 32'd0;
                wr_ptr <= {PTR_WIDTH{1'b0}};
                rd_ptr <= {PTR_WIDTH{1'b0}};
                fifo_count <= {(PTR_WIDTH+1){1'b0}};
                burst_active <= 1'b0;
                burst_remaining <= 0;
                command_phase <= 0;
                row_padding <= 1'b0;
                geometry_ready <= !RUNTIME_GEOMETRY;
                write_commit <= 1'b0;
                memory_error <= ERR_NONE;
                case (image_index)
                    2'd0: image_complete[0] <= 1'b0;
                    2'd1: image_complete[1] <= 1'b0;
                    2'd2: image_complete[2] <= 1'b0;
                    default: image_complete[3] <= 1'b0;
                endcase
            end

            if (active) begin
                if (decode_error != 0) begin
                    fail_load(ERR_DECODE);
                end else if (RUNTIME_GEOMETRY && frame_config_valid &&
                             (geometry_ready || !config_ok)) begin
                    fail_load(ERR_CONFIG);
                end else if (RUNTIME_GEOMETRY && pixel_valid && !geometry_ready) begin
                    fail_load(ERR_CONFIG);
                end else if (RUNTIME_GEOMETRY && commit_seen &&
                             (!geometry_ready || pixels_received != expected_words)) begin
                    fail_load(ERR_SHORT_FRAME);
                end else if (pixel_valid && pixels_received >= expected_words) begin
                    fail_load(ERR_PIXEL_COUNT);
                end else if (pixel_valid && !push_allowed) begin
                    fail_load(ERR_FIFO_OVERFLOW);
                end else begin
                    if (RUNTIME_GEOMETRY && frame_config_valid) begin
                        geometry_ready <= 1'b1;
                        frame_words_q <= {12'd0,config_words};
                        frame_width_q <= frame_width;
                        flip_q <= !frame_top_down;
                        line_jump_q <= ({1'b0,frame_width} << 1) - 1'b1;
                        write_offset <= frame_top_down ? 0 : config_words-frame_width;
                    end
                    if (start_burst) begin
                        burst_active <= 1'b1;
                        burst_remaining <= next_burst_words;
                    end

                    if (push_allowed) begin
                        fifo_mem[wr_ptr] <= {8'h00, pixel_rgb};
                        wr_ptr <= wr_ptr + 1'b1;
                        pixels_received <= pixels_received + 1'b1;
                    end

                    if (command_emit) begin
                        app_wr_en <= 1'b1;
                        // Latch the group address only at lane zero. A row
                        // jump may change pixel_address during this group;
                        // its remaining lanes must retain the old group.
                        if (command_phase[1:0] == 0)
                            app_wr_addr[ADDR_WIDTH-1:4] <= pixel_address[ADDR_WIDTH-1:4];
                        app_wr_addr[3:0] <= command_phase;
                        app_wr_mask <= pop_allowed ? 4'h0 : 4'hf;
                        command_phase <= command_phase + 1'b1;
                        if (command_phase[1:0] == 3)
                            row_padding <= 1'b0;
                        stall_count <= 32'd0;
                        if (burst_remaining == 1) begin
                            burst_active <= 1'b0;
                            burst_remaining <= 0;
                        end else begin
                            burst_remaining <= burst_remaining - 1'b1;
                        end
                    end else if (fifo_count != 0 &&
                                 (commit_seen || fifo_count >= EFFECTIVE_BURST)) begin
                        if (stall_count == STALL_TIMEOUT)
                            fail_load(ERR_WRITE_STALL);
                        else
                            stall_count <= stall_count + 1'b1;
                    end else begin
                        stall_count <= 32'd0;
                    end

                    if (pop_allowed) begin
                        rd_ptr <= rd_ptr + 1'b1;
                        pixels_written <= pixels_written + 1'b1;
                        // BMP正高度文件先发底行：行内正序，行末转到上一行。
                        // 这段原来位于 frame_fifo_write，替换写路径时需保留。
                        if (pixels_written < expected_words - 1) begin
                            if (flip_rows) begin
                                if (write_x == row_width - 1) begin
                                    write_x <= 16'd0;
                                    write_offset <= write_offset - line_jump;
                                    // Finish the old row's physical group
                                    // before emitting the next row's head.
                                    row_padding <= command_phase[1:0] != 3;
                                end else begin
                                    write_x <= write_x + 1'b1;
                                    write_offset <= write_offset + 1'b1;
                                end
                            end else begin
                                write_offset <= write_offset + 1'b1;
                            end
                        end
                    end

                    case ({push_allowed, pop_allowed})
                        2'b10: fifo_count <= fifo_count + 1'b1;
                        2'b01: fifo_count <= fifo_count - 1'b1;
                        default: fifo_count <= fifo_count;
                    endcase

                    if (frame_commit)
                        commit_seen <= 1'b1;

                    if (commit_seen && fifo_count == 0 && !pop_allowed &&
                        !burst_active) begin
                        if (pixels_received != expected_words || pixels_written != expected_words) begin
                            fail_load(ERR_SHORT_FRAME);
                        end else begin
                            // The local FIFO is empty here, but the commands
                            // may still be queued in the CDC FIFO. Keep the
                            // commit request asserted until the destination
                            // clock domain confirms that its queue drained.
                            write_commit <= 1'b1;
                            if (write_commit && write_complete) begin
                                case (active_image)
                                    2'd0: image_complete[0] <= 1'b1;
                                    2'd1: image_complete[1] <= 1'b1;
                                    2'd2: image_complete[2] <= 1'b1;
                                    default: image_complete[3] <= 1'b1;
                                endcase
                                active <= 1'b0;
                                write_commit <= 1'b0;
                                load_done <= 1'b1;
                            end
                        end
                    end
                end
            end else if (pixel_valid) begin
                memory_error <= ERR_NOT_ACTIVE;
            end
        end
    end
endmodule
