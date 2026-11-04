`timescale 1ns/1ps

module media_frame_reader #(
    parameter integer ADDR_WIDTH = 21,
    parameter integer IMAGE_WORDS = 307200,
    parameter integer FIFO_DEPTH = 512,
    parameter integer FIFO_MARGIN = 8,
    parameter integer BURST_SIZE = 128,
    parameter integer BURST_GAP_CYCLES = 12,
    parameter integer MAX_INFLIGHT_BURSTS = 2,
    parameter integer DIAG_RETURN_PATTERN = 0,
    parameter integer DIAG_SYNTH_STREAM = 0,
    // Enable for a valid/ready stream consumer such as the 720p scaler.
    // Default 0 preserves the existing direct-to-video pixel interface.
    parameter integer FRAME_STREAM_HANDSHAKE = 0,
    parameter integer RUNTIME_GEOMETRY = 0,
    parameter integer BANK_WORDS = IMAGE_WORDS,
    parameter integer DISPLAY_EPOCH_TAG = 0
) (
    input  wire                  mem_clk,
    input  wire                  mem_rst_n,
    input  wire                  controller_init_done,
    input  wire                  controller_busy,
    input  wire                  refresh_active,
    input  wire                  burst_grant,
    output wire                  burst_request,
    output wire                  burst_active,
    output reg                   app_rd_en,
    output reg  [ADDR_WIDTH-1:0] app_rd_addr,
    input  wire                  sdram_rd_valid,
    input  wire [31:0]           sdram_rd_data,

    input  wire                  pix_clk,
    input  wire                  pix_rst_n,
    input  wire                  frame_start,
    input  wire [1:0]            active_index,
    input  wire                  pixel_consume,
    output wire [23:0]           pixel_rgb,
    output wire                  pixel_valid,
    output reg                   underflow,

    output reg                   reader_busy,
    output reg                   frame_read_done,
    output reg  [1:0]            reader_index,
    output reg  [7:0]            reader_error,

    // Use with FRAME_STREAM_HANDSHAKE=1. In the pixel-clock domain, the
    // selected bank and a fresh FIFO epoch are ready, not necessarily pixels
    // yet. Start pulses once per published frame; superseded frames are skipped.
    output reg                   frame_stream_start,
    output wire                  frame_stream_ready,
    // Read-only observation ports in this isolated diagnostic copy.
    output wire                  diag_reload_pending,
    output wire                  diag_inflight_pending,
    output wire                  diag_request_inflight,
    output wire                  diag_fifo_reset,
    // Same pixel-domain metadata commit as active_index; held with its request
    // until acknowledged. Only used with RUNTIME_GEOMETRY=1.
    input wire [ADDR_WIDTH:0]     active_word_count,
    // Optional display-transaction tag, latched WITH the bank request. These
    // pixel-domain outputs acknowledge drained old returns + a fresh FIFO,
    // even when the acknowledged frame was superseded before publication.
    // The tag is held locally while the existing reader token crosses memory;
    // it cannot change until that exact token is acknowledged.
    input wire                   active_epoch,
    output reg                   reload_done,
    output reg                   reload_epoch,
    output reg [1:0]             reload_bank
);
    localparam [7:0] ERR_NONE          = 8'h00;
    localparam [7:0] ERR_FRAME_OVERRUN = 8'h61;
    localparam [7:0] ERR_FIFO_OVERFLOW = 8'h62;
    localparam [7:0] ERR_UNDERFLOW     = 8'h63;
    localparam [7:0] ERR_SHORT_FRAME   = 8'h64;
    localparam [7:0] ERR_FRAME_SIZE    = 8'h65;
    localparam integer FIFO_COUNT_W = (FIFO_DEPTH <= 2) ? 1 : $clog2(FIFO_DEPTH);
    localparam integer BURST_COUNT_W =
        (BURST_SIZE < 2) ? 1 : $clog2(BURST_SIZE + 1);
    localparam integer INFLIGHT_COUNT_W =
        (MAX_INFLIGHT_BURSTS * BURST_SIZE < 2) ? 1 :
        $clog2(MAX_INFLIGHT_BURSTS * BURST_SIZE + 1);
    localparam [ADDR_WIDTH:0] READ_ALIGN_MASK = ~3;

    reg request_pending_pix;
    reg request_toggle_pix;
    reg [1:0] request_index_pix;
    reg [ADDR_WIDTH:0] request_words_pix;
    reg request_epoch_pix;
    reg frame_armed_pix;
    reg [ADDR_WIDTH:0] consumed_count_pix;
    reg request_inflight_pix;
    reg stream_ready_pix;
    reg stream_ack_sync1_pix;
    reg stream_ack_sync2_pix;

    reg request_sync1_mem;
    reg request_sync2_mem;
    reg request_seen_mem;
    reg [1:0] index_sync1_mem;
    reg [1:0] index_sync2_mem;
    reg [ADDR_WIDTH:0] words_sync1_mem, words_sync2_mem;
    reg [ADDR_WIDTH:0] reload_words_mem, frame_words_mem;
    reg [ADDR_WIDTH:0] transfer_words_mem;
    reg image_returns_done_mem;
    reg reload_pending_mem;
    reg [1:0] reload_index_mem;
    reg reload_token_mem;
    reg stream_flush_pending_mem;
    reg stream_flush_token_mem;
    reg stream_ack_mem;
    reg [1:0] fifo_pix_ready_sync_mem;
    reg [2:0] fifo_reset_count_mem;
    reg [ADDR_WIDTH:0] issued_count_mem;
    reg [ADDR_WIDTH:0] returned_count_mem;
    reg [INFLIGHT_COUNT_W-1:0] inflight_count_mem;
    reg inflight_empty_mem /* fehdl keep="true" */;
    reg burst_active_mem;
    reg [BURST_COUNT_W-1:0] burst_issued_mem;
    reg [BURST_COUNT_W-1:0] burst_target_mem;
    reg [7:0] burst_gap_count_mem;
    reg [ADDR_WIDTH:0] diag_written_count_mem;
    reg [9:0] diag_column_mem;
    reg [23:0] diag_rgb_mem;
    reg [1:0] fifo_release_mem;
    reg [1:0] fifo_release_pix;

    wire fifo_reset_request = !mem_rst_n || !pix_rst_n ||
                              (fifo_reset_count_mem != 0);
    wire fifo_reset = fifo_reset_request ||
                      !fifo_release_mem[1] || !fifo_release_pix[1];
    wire fifo_full;
    wire fifo_empty;
    wire fifo_almost_full;
    wire [FIFO_COUNT_W-1:0] fifo_wrusedw;
    wire [31:0] fifo_raw_data;
    wire fifo_raw_valid;
    wire fifo_raw_read_en;
    wire prefetch_valid;
    wire [31:0] prefetch_data;
    wire stream_open_pix = stream_ready_pix && !frame_start &&
                           !request_pending_pix && !fifo_reset;
    wire pixel_available = prefetch_valid &&
                           (!FRAME_STREAM_HANDSHAKE || stream_open_pix);
    wire pixel_pop = pixel_consume && pixel_available;
    assign frame_stream_ready = stream_open_pix;
    assign diag_reload_pending = reload_pending_mem;
    assign diag_inflight_pending = !inflight_empty_mem;
    assign diag_request_inflight = request_inflight_pix;
    assign diag_fifo_reset = fifo_reset;
    wire [ADDR_WIDTH:0] inflight_words =
        {{(ADDR_WIDTH+1-INFLIGHT_COUNT_W){1'b0}}, inflight_count_mem};
    wire [ADDR_WIDTH:0] expected_words_mem = RUNTIME_GEOMETRY ? frame_words_mem : IMAGE_WORDS;
    wire [ADDR_WIDTH:0] expected_words_pix = RUNTIME_GEOMETRY ? request_words_pix : IMAGE_WORDS;
    wire last_pixel_return = returned_count_mem == expected_words_mem - 1;
    wire final_return = returned_count_mem == transfer_words_mem - 1;
    wire [ADDR_WIDTH+1:0] reserved_words =
        {{(ADDR_WIDTH+2-FIFO_COUNT_W){1'b0}}, fifo_wrusedw} +
        {{1{1'b0}}, inflight_words};
    localparam integer FULL_BURSTS_ONLY = !RUNTIME_GEOMETRY && (IMAGE_WORDS % BURST_SIZE) == 0;
    wire [ADDR_WIDTH:0] remaining_words = transfer_words_mem - issued_count_mem;
    wire [ADDR_WIDTH:0] next_burst_words =
        FULL_BURSTS_ONLY ? BURST_SIZE :
        ((remaining_words > BURST_SIZE) ? BURST_SIZE : remaining_words);
    // Reserve a FULL burst even for the last short read. This conservative
    // admission rule breaks the frame-length subtract/min/add/compare chain
    // at 125 MHz. The controller requires complete groups of four words;
    // round only the frame tail up inside its reserved SDRAM bank. Padding
    // returns drain normally but never enter the pixel FIFO.
    wire [ADDR_WIDTH+2:0] reserved_after_burst =
        {{1{1'b0}}, reserved_words} + BURST_SIZE;
    wire inflight_window_ok = (MAX_INFLIGHT_BURSTS == 1) ?
        inflight_empty_mem :
        ((inflight_words + BURST_SIZE) <=
         (MAX_INFLIGHT_BURSTS * BURST_SIZE));
    wire fifo_reservation_ok =
        reserved_after_burst <= (FIFO_DEPTH - FIFO_MARGIN);
    reg inflight_window_ok_q;
    reg fifo_reservation_ok_q;
    reg read_words_pending_q;
    wire burst_ready = mem_rst_n && !DIAG_SYNTH_STREAM && reader_busy && controller_init_done &&
                       !controller_busy && !refresh_active &&
                       fifo_reset_count_mem == 0 && !fifo_reset &&
                       !burst_active_mem && !app_rd_en &&
                       burst_gap_count_mem == 0 &&
                       read_words_pending_q &&
                       inflight_empty_mem &&
                       !reload_pending_mem &&
                       (request_sync2_mem == request_seen_mem) &&
                       inflight_window_ok_q && fifo_reservation_ok_q;
    wire start_burst = burst_ready && burst_grant;
    assign burst_request = burst_ready;
    // Keep ownership through the registered command output and all returns,
    // even if a frame reload/error has cleared reader_busy.
    // The registered empty flag tracks the same issue/return events as the
    // frame counters. Use it in the arbitration path instead of a wide
    // combinational comparison of issued_count_mem and returned_count_mem.
    assign burst_active = burst_active_mem || app_rd_en || !inflight_empty_mem;
    wire issue_allowed = !DIAG_SYNTH_STREAM && reader_busy && burst_active_mem;
    wire diag_stream_write = DIAG_SYNTH_STREAM && reader_busy &&
                             !fifo_reset && !fifo_full;
    wire fifo_write_enable = DIAG_SYNTH_STREAM ? diag_stream_write :
                             (sdram_rd_valid && !image_returns_done_mem &&
                              !fifo_reset && !fifo_full);
    wire [31:0] fifo_write_data = (DIAG_RETURN_PATTERN || DIAG_SYNTH_STREAM) ?
                                  {8'h00, diag_rgb_mem} : sdram_rd_data;

    always @* begin
        if (diag_column_mem < 10'd80)
            diag_rgb_mem = 24'hffffff;
        else if (diag_column_mem < 10'd160)
            diag_rgb_mem = 24'hffff00;
        else if (diag_column_mem < 10'd240)
            diag_rgb_mem = 24'h00ffff;
        else if (diag_column_mem < 10'd320)
            diag_rgb_mem = 24'h00ff00;
        else if (diag_column_mem < 10'd400)
            diag_rgb_mem = 24'hff00ff;
        else if (diag_column_mem < 10'd480)
            diag_rgb_mem = 24'hff0000;
        else if (diag_column_mem < 10'd560)
            diag_rgb_mem = 24'h0000ff;
        else
            diag_rgb_mem = 24'h000000;
    end

    function [ADDR_WIDTH-1:0] bank_base;
        input [1:0] index;
        begin
            case (index)
                2'd0: bank_base = {ADDR_WIDTH{1'b0}};
                2'd1: bank_base = BANK_WORDS;
                2'd2: bank_base = BANK_WORDS * 2;
                default: bank_base = BANK_WORDS * 3;
            endcase
        end
    endfunction

    // Delay the request by one pixel clock so media_player_control can commit
    // active_index on the frame_start edge before this module captures it.
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            request_pending_pix <= 1'b0;
            request_toggle_pix <= 1'b0;
            request_index_pix <= 2'd0;
            request_words_pix <= IMAGE_WORDS;
            frame_armed_pix <= 1'b0;
            consumed_count_pix <= 0;
            underflow <= 1'b0;
            fifo_release_pix <= 2'b00;
            request_inflight_pix <= 1'b0;
            stream_ready_pix <= 1'b0;
            stream_ack_sync1_pix <= 1'b0;
            stream_ack_sync2_pix <= 1'b0;
            frame_stream_start <= 1'b0;
            request_epoch_pix <= 1'b0;
            reload_done <= 1'b0;
            reload_epoch <= 1'b0;
            reload_bank <= 2'd0;
        end else begin
            stream_ack_sync1_pix <= stream_ack_mem;
            stream_ack_sync2_pix <= stream_ack_sync1_pix;
            frame_stream_start <= 1'b0;
            reload_done <= 1'b0;
            if (fifo_reset_request)
                fifo_release_pix <= 2'b00;
            else
                fifo_release_pix <= {fifo_release_pix[0], 1'b1};

            if (frame_start) begin
                if (frame_armed_pix && consumed_count_pix != expected_words_pix) begin
                    underflow <= 1'b1;
                end
                consumed_count_pix <= 0;
                request_pending_pix <= 1'b1;
                stream_ready_pix <= 1'b0;
            end else if (request_pending_pix &&
                         (!(FRAME_STREAM_HANDSHAKE || RUNTIME_GEOMETRY) || !request_inflight_pix)) begin
                request_index_pix <= active_index;
                request_words_pix <= RUNTIME_GEOMETRY ? active_word_count : IMAGE_WORDS;
                request_epoch_pix <= DISPLAY_EPOCH_TAG ? active_epoch : 1'b0;
                request_toggle_pix <= ~request_toggle_pix;
                request_pending_pix <= 1'b0;
                frame_armed_pix <= 1'b1;
                request_inflight_pix <= 1'b1;
                stream_ready_pix <= 1'b0;
            end else if (request_inflight_pix && !fifo_reset &&
                         stream_ack_sync2_pix == request_toggle_pix) begin
                request_inflight_pix <= 1'b0;
                reload_done <= 1'b1;
                reload_epoch <= request_epoch_pix;
                reload_bank <= request_index_pix;
                // A newer frame may have arrived while the old returns were
                // draining. Retire its ack, then send the latest pending bank;
                // never publish the superseded stream or reuse a busy token.
                if (!request_pending_pix) begin
                    stream_ready_pix <= 1'b1;
                    frame_stream_start <= 1'b1;
                end
            end

            if (pixel_consume && (!FRAME_STREAM_HANDSHAKE || stream_open_pix)) begin
                if (pixel_available)
                    consumed_count_pix <= consumed_count_pix + 1'b1;
                else if (frame_armed_pix)
                    underflow <= 1'b1;
            end
        end
    end

    always @(posedge mem_clk or negedge mem_rst_n) begin
        if (!mem_rst_n) begin
            request_sync1_mem <= 1'b0;
            request_sync2_mem <= 1'b0;
            request_seen_mem <= 1'b0;
            index_sync1_mem <= 2'd0;
            index_sync2_mem <= 2'd0;
            words_sync1_mem <= IMAGE_WORDS;
            words_sync2_mem <= IMAGE_WORDS;
            reload_words_mem <= IMAGE_WORDS;
            frame_words_mem <= IMAGE_WORDS;
            transfer_words_mem <= ((IMAGE_WORDS + 3) / 4) * 4;
            image_returns_done_mem <= 1'b0;
            reload_pending_mem <= 1'b0;
            reload_index_mem <= 2'd0;
            reload_token_mem <= 1'b0;
            stream_flush_pending_mem <= 1'b0;
            stream_flush_token_mem <= 1'b0;
            stream_ack_mem <= 1'b0;
            fifo_pix_ready_sync_mem <= 2'b00;
            fifo_reset_count_mem <= 3'd0;
            issued_count_mem <= 0;
            returned_count_mem <= 0;
            inflight_count_mem <= 0;
            inflight_empty_mem <= 1'b1;
            burst_active_mem <= 1'b0;
            burst_issued_mem <= 0;
            burst_target_mem <= 0;
            burst_gap_count_mem <= 0;
            diag_written_count_mem <= 0;
            diag_column_mem <= 0;
            fifo_release_mem <= 2'b00;
            inflight_window_ok_q <= 1'b1;
            fifo_reservation_ok_q <= 1'b1;
            read_words_pending_q <= 1'b0;
            app_rd_en <= 1'b0;
            app_rd_addr <= 0;
            reader_busy <= 1'b0;
            frame_read_done <= 1'b0;
            reader_index <= 2'd0;
            reader_error <= ERR_NONE;
        end else begin
            if (fifo_reset_request)
                fifo_release_mem <= 2'b00;
            else
                fifo_release_mem <= {fifo_release_mem[0], 1'b1};

            if (fifo_reset_request)
                fifo_pix_ready_sync_mem <= 2'b00;
            else
                fifo_pix_ready_sync_mem <=
                    {fifo_pix_ready_sync_mem[0], fifo_release_pix[1]};

            // Acknowledge the actual reload, after its FIFO reset has passed
            // through both domains. The token stays stable until the next
            // reload; the pixel domain synchronizes it before opening data.
            if (stream_flush_pending_mem && !fifo_reset &&
                fifo_pix_ready_sync_mem[1] && !reload_pending_mem &&
                request_sync2_mem == request_seen_mem) begin
                stream_ack_mem <= stream_flush_token_mem;
                stream_flush_pending_mem <= 1'b0;
            end

            request_sync1_mem <= request_toggle_pix;
            request_sync2_mem <= request_sync1_mem;
            index_sync1_mem <= request_index_pix;
            index_sync2_mem <= index_sync1_mem;
            words_sync1_mem <= request_words_pix;
            words_sync2_mem <= words_sync1_mem;
            app_rd_en <= 1'b0;
            frame_read_done <= 1'b0;
            inflight_window_ok_q <= inflight_window_ok;
            fifo_reservation_ok_q <= fifo_reservation_ok;
            // Keep the wide frame-length comparison out of request -> grant
            // -> burst-counter enable. It may lag the last issued word by
            // one cycle; !app_rd_en blocks that cycle even with zero gap and
            // an always-ready grant. Reload's FIFO reset covers a new length.
            read_words_pending_q <= issued_count_mem < transfer_words_mem;

            if (fifo_reset_count_mem != 0)
                fifo_reset_count_mem <= fifo_reset_count_mem - 1'b1;
            if (burst_gap_count_mem != 0)
                burst_gap_count_mem <= burst_gap_count_mem - 1'b1;

            case ({issue_allowed, sdram_rd_valid})
                2'b10: begin
                    inflight_count_mem <= inflight_count_mem + 1'b1;
                    inflight_empty_mem <= 1'b0;
                end
                2'b01: begin
                    if (!inflight_empty_mem) begin
                        inflight_count_mem <= inflight_count_mem - 1'b1;
                        if (inflight_count_mem == 1)
                            inflight_empty_mem <= 1'b1;
                    end
                end
                default: begin
                    inflight_count_mem <= inflight_count_mem;
                    inflight_empty_mem <= inflight_empty_mem;
                end
            endcase

            if (request_sync2_mem != request_seen_mem) begin
                request_seen_mem <= request_sync2_mem;
                reload_pending_mem <= 1'b1;
                reload_index_mem <= index_sync2_mem;
                reload_words_mem <= words_sync2_mem;
                reload_token_mem <= request_sync2_mem;
                stream_flush_pending_mem <= 1'b0;
                if (reader_busy || !inflight_empty_mem) begin
                    reader_busy <= 1'b0;
                    burst_active_mem <= 1'b0;
                    reader_error <= ERR_FRAME_OVERRUN;
                end
            end

            if (reload_pending_mem && inflight_empty_mem &&
                (!(FRAME_STREAM_HANDSHAKE || RUNTIME_GEOMETRY) ||
                 request_sync2_mem == request_seen_mem)) begin
                reload_pending_mem <= 1'b0;
                reader_index <= reload_index_mem;
                frame_words_mem <= reload_words_mem;
                transfer_words_mem <= RUNTIME_GEOMETRY ?
                    ((reload_words_mem + 3) & READ_ALIGN_MASK) : ((IMAGE_WORDS + 3) / 4) * 4;
                image_returns_done_mem <= 1'b0;
                issued_count_mem <= 0;
                returned_count_mem <= 0;
                inflight_count_mem <= 0;
                inflight_empty_mem <= 1'b1;
                burst_active_mem <= 1'b0;
                burst_issued_mem <= 0;
                burst_target_mem <= 0;
                burst_gap_count_mem <= 0;
                diag_written_count_mem <= 0;
                diag_column_mem <= 0;
                fifo_reset_count_mem <= 3'd4;
                stream_flush_token_mem <= reload_token_mem;
                stream_flush_pending_mem <= 1'b1;
                if (RUNTIME_GEOMETRY && (reload_words_mem == 0 || reload_words_mem > BANK_WORDS)) begin
                    reader_busy <= 1'b0;
                    reader_error <= ERR_FRAME_SIZE;
                end else begin
                    reader_busy <= 1'b1;
                    reader_error <= ERR_NONE;
                end
            end

            if (start_burst) begin
                burst_active_mem <= 1'b1;
                burst_issued_mem <= 0;
                burst_target_mem <= next_burst_words[BURST_COUNT_W-1:0];
            end

            if (issue_allowed) begin
                app_rd_en <= 1'b1;
                app_rd_addr <= bank_base(reader_index) + issued_count_mem[ADDR_WIDTH-1:0];
                issued_count_mem <= issued_count_mem + 1'b1;
                burst_issued_mem <= burst_issued_mem + 1'b1;
                if (burst_issued_mem + 1'b1 == burst_target_mem) begin
                    burst_active_mem <= 1'b0;
                    burst_gap_count_mem <= BURST_GAP_CYCLES;
                end
            end

            if (!DIAG_SYNTH_STREAM && sdram_rd_valid) begin
                returned_count_mem <= returned_count_mem + 1'b1;
                if (last_pixel_return)
                    image_returns_done_mem <= 1'b1;
                if (fifo_full && !image_returns_done_mem) begin
                    reader_busy <= 1'b0;
                    burst_active_mem <= 1'b0;
                    reader_error <= ERR_FIFO_OVERFLOW;
                end else if (final_return) begin
                    reader_busy <= 1'b0;
                    burst_active_mem <= 1'b0;
                    frame_read_done <= 1'b1;
                end
            end

            if (DIAG_SYNTH_STREAM && diag_stream_write) begin
                diag_written_count_mem <= diag_written_count_mem + 1'b1;
                diag_column_mem <= (diag_column_mem == 10'd639) ?
                                   10'd0 : diag_column_mem + 1'b1;
                if (diag_written_count_mem == expected_words_mem - 1) begin
                    reader_busy <= 1'b0;
                    frame_read_done <= 1'b1;
                end
            end else if (!DIAG_SYNTH_STREAM && DIAG_RETURN_PATTERN &&
                         sdram_rd_valid) begin
                diag_column_mem <= (diag_column_mem == 10'd639) ?
                                   10'd0 : diag_column_mem + 1'b1;
            end
        end
    end

    rfifo_32_32_512 #(
        .DATA_WIDTH_W(32), .DATA_WIDTH_R(32),
        .ADDR_WIDTH_W(FIFO_COUNT_W), .ADDR_WIDTH_R(FIFO_COUNT_W),
        .AL_FULL_NUM(FIFO_DEPTH-FIFO_MARGIN), .AL_EMPTY_NUM(2),
        .SHOW_AHEAD_EN(1'b0), .OUTREG_EN("NOREG")
    ) u_read_fifo (
        .rst(fifo_reset), .clkw(mem_clk), .clkr(pix_clk),
        .we(fifo_write_enable), .di(fifo_write_data),
        .re(fifo_raw_read_en), .dout(fifo_raw_data), .valid(fifo_raw_valid),
        .full_flag(fifo_full), .empty_flag(fifo_empty),
        .afull(fifo_almost_full), .aempty(),
        .wrusedw(fifo_wrusedw), .rdusedw()
    );

    fifo_prefetch_2 #(.DATA_WIDTH(32)) u_prefetch (
        .clk(pix_clk), .rst(fifo_reset), .raw_empty(fifo_empty),
        .raw_valid(fifo_raw_valid), .raw_data(fifo_raw_data),
        .raw_read_en(fifo_raw_read_en), .consume(pixel_pop),
        .out_valid(prefetch_valid), .out_empty(), .out_data(prefetch_data)
    );

    assign pixel_rgb = prefetch_data[23:0];
    assign pixel_valid = pixel_available;

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            // The memory-domain error remains authoritative for controller faults.
        end else if (underflow) begin
            // underflow is exported separately; the Top may merge it as 8'h63.
        end
    end

endmodule


