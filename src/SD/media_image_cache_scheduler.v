`timescale 1ns/1ps

// Four physical frame slots, independent of the number of files on the card.
// All ports belong to clk (the SD/control domain); this module is NOT a CDC.
//
// Integration contract:
// - Latch a completed catalogue once per coordinated player reset. File ID 0
//   means PIC1, while bank 0..3 is only a storage address. Holes are permitted.
// - load_valid/ready transfers ONE file/bank job. The adapter owns bounded
//   retries, sector timeouts and draining. load_done reports its FINAL result,
//   after all old file bytes and memory writes have drained; it cannot coincide
//   with acceptance. A failed file is excluded until the next player reset.
// - switch_valid/ready transfers a READY, reserved target to the transition /
//   display controller (initial=1 bypasses the effect). switch_done must arrive
//   only AFTER the pixel-domain bank/geometry commit, retirement of all old-bank
//   reads, AND completion of the transition. It cannot coincide with acceptance.
//   Both old display and target remain protected throughout this transaction.
// - A single waiting manual direction is overwritten by the latest press.
//   Accepted switches are immutable. Auto ticks never accumulate while busy.
// - Reset the loader, reader, transition and scheduler together; dropping this
//   module's reset alone does not cancel transactions elsewhere in the system.
// Geometry remains in the existing per-bank table and follows the same locks.
module media_image_cache_scheduler #(
    parameter integer MAX_IMAGES = 16,
    parameter integer ENABLE_CUE_SKIP = 0,
    parameter integer IMAGE_ID_WIDTH = (MAX_IMAGES <= 1) ? 1 : $clog2(MAX_IMAGES)
) (
    input wire clk,
    input wire rst_n,
    input wire catalog_valid,
    input wire [MAX_IMAGES-1:0] catalog_image_mask,
    input wire next_event,
    input wire prev_event,
    input wire auto_event,
    input wire auto_skip_unready,

    output reg load_valid,
    input wire load_ready,
    output reg [IMAGE_ID_WIDTH-1:0] load_image_id,
    output reg [1:0] load_bank,
    input wire load_done,
    input wire load_success,
    output wire load_busy,

    output reg switch_valid,
    input wire switch_ready,
    output reg [IMAGE_ID_WIDTH-1:0] switch_image_id,
    output reg [1:0] switch_bank,
    output reg switch_initial,
    output reg switch_auto,
    output reg switch_previous,
    input wire switch_done,
    output wire switch_busy,

    output reg active_valid,
    output reg [IMAGE_ID_WIDTH-1:0] active_image_id,
    output reg [1:0] active_bank,
    output reg request_pending,
    output reg pending_previous,
    output reg pending_auto,
    output reg [3:0] slot_valid_mask,
    output wire [3:0] slot_loading_mask,
    output wire [3:0] protected_bank_mask,
    output wire [4*IMAGE_ID_WIDTH-1:0] slot_image_ids,
    output reg [MAX_IMAGES-1:0] bad_image_mask,
    output wire no_usable_images
);
    localparam integer WALK_COUNT_WIDTH = $clog2(MAX_IMAGES + 1);
    localparam [2:0] S_IDLE = 0, S_WALK = 1, S_TARGET = 2,
                     S_PLAN = 3, S_SELECT = 4, S_VICTIM = 5, S_PUBLISH = 6,
                     S_VICTIM_INIT = 7;
    localparam [2:0] W_FIRST = 0, W_REQUEST = 1, W_NEXT = 2,
                     W_PREVIOUS = 3, W_NEXT2 = 4;
    reg [2:0] state, walk_kind;
    reg [IMAGE_ID_WIDTH-1:0] walk_id;
    reg [WALK_COUNT_WIDTH-1:0] walk_left;
    reg walk_previous;
    reg catalog_seen;
    reg pending_skip;
    reg [MAX_IMAGES-1:0] image_mask;
    reg load_inflight, switch_inflight;
    reg [IMAGE_ID_WIDTH-1:0] slot_id [0:3];
    // Ordered by eviction/prefetch priority: centre, next, previous, next-next.
    reg [IMAGE_ID_WIDTH-1:0] wanted_id [0:3];
    reg [IMAGE_ID_WIDTH-1:0] plan_center, work_image;
    reg plan_for_target;
    wire [MAX_IMAGES-1:0] usable_mask = image_mask & ~bad_image_mask;
    wire manual_event = active_valid && (next_event || prev_event);
    wire accept_auto = active_valid && auto_event && !manual_event &&
                       !request_pending && !switch_busy;
    wire load_result = load_inflight && load_done;
    wire switch_result = switch_inflight && switch_done;

    assign load_busy = load_valid || load_inflight;
    assign switch_busy = switch_valid || switch_inflight;
    assign slot_loading_mask = load_busy ? (4'b0001 << load_bank) : 4'd0;
    assign protected_bank_mask =
        (active_valid ? (4'b0001 << active_bank) : 4'd0) |
        (switch_busy ? (4'b0001 << switch_bank) : 4'd0);
    assign slot_image_ids = {slot_id[3], slot_id[2], slot_id[1], slot_id[0]};
    assign no_usable_images = catalog_seen && !(|usable_mask);

    function [IMAGE_ID_WIDTH-1:0] advance_id;
        input [IMAGE_ID_WIDTH-1:0] id;
        input previous;
        begin
            if (previous)
                advance_id = (id == 0) ? MAX_IMAGES-1 : id - 1'b1;
            else
                advance_id = (id == MAX_IMAGES-1) ? 0 : id + 1'b1;
        end
    endfunction

    function [2:0] cached_slot;
        input [IMAGE_ID_WIDTH-1:0] id;
        input [3:0] valid_slots;
        input [4*IMAGE_ID_WIDTH-1:0] ids;
        integer n;
        begin
            cached_slot = 3'd0;
            for (n = 0; n < 4; n = n + 1)
                if (!cached_slot[2] && valid_slots[n] && ids[n*IMAGE_ID_WIDTH+:IMAGE_ID_WIDTH] == id)
                    cached_slot = {1'b1, n[1:0]};
        end
    endfunction
    wire [2:0] target_hit = cached_slot(plan_center, slot_valid_mask, slot_image_ids);

    // Inspect one wanted file per control clock, in the SAME priority order.
    // This removes four parallel lookups followed by a long priority/enable
    // chain (wanted_id -> missing_found -> victim initialisation).
    reg [1:0] prefetch_cursor;
    wire [IMAGE_ID_WIDTH-1:0] prefetch_id = wanted_id[prefetch_cursor];
    wire [2:0] prefetch_hit = cached_slot(prefetch_id, slot_valid_mask, slot_image_ids);
    reg victim_found;
    reg [1:0] victim_bank;
    integer j;
    reg [1:0] victim_cursor;
    reg [2:0] candidate_rank, best_rank;
    always @* begin
        // Invalid slot first, then a slot outside the desired neighbourhood.
        // A foreground miss may evict the lowest-priority unprotected neighbour.
        // Prefetch may NOT do that, otherwise a temporarily pinned old picture
        // would cause continuous eviction/reloading of the other neighbours.
        // Inspect one bank per clock. The old four-bank max-rank chain was
        // too long for 100 MHz; ranks need only three bits (0..5).
        candidate_rank = 3'd4;
        for (j = 3; j >= 0; j = j - 1)
            if (slot_id[victim_cursor] == wanted_id[j]) candidate_rank = j[2:0];
        if (!slot_valid_mask[victim_cursor]) candidate_rank = 3'd5;
    end

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            walk_kind <= W_FIRST;
            walk_id <= 0;
            walk_left <= 0;
            walk_previous <= 1'b0;
            catalog_seen <= 1'b0;
            image_mask <= 0;
            bad_image_mask <= 0;
            slot_valid_mask <= 0;
            load_valid <= 1'b0;
            load_inflight <= 1'b0;
            load_image_id <= 0;
            load_bank <= 0;
            switch_valid <= 1'b0;
            switch_inflight <= 1'b0;
            switch_image_id <= 0;
            switch_bank <= 0;
            switch_initial <= 1'b0;
            switch_auto <= 1'b0;
            switch_previous <= 1'b0;
            active_valid <= 1'b0;
            active_image_id <= 0;
            active_bank <= 0;
            request_pending <= 1'b0;
            pending_previous <= 1'b0;
            pending_auto <= 1'b0;
            pending_skip <= 1'b0;
            plan_center <= 0;
            work_image <= 0;
            plan_for_target <= 1'b0;
            prefetch_cursor <= 0;
            victim_cursor <= 0;
            victim_found <= 0;
            victim_bank <= 0;
            best_rank <= 0;
            for (k = 0; k < 4; k = k + 1) begin
                slot_id[k] <= 0;
                wanted_id[k] <= 0;
            end
        end else begin
            if (!catalog_seen && catalog_valid) begin
                catalog_seen <= 1'b1;
                image_mask <= catalog_image_mask;
            end
            if (load_valid && load_ready) begin
                load_valid <= 1'b0;
                load_inflight <= 1'b1;
            end
            if (load_result) begin
                load_inflight <= 1'b0;
                slot_valid_mask[load_bank] <= load_success;
                if (!load_success) bad_image_mask[load_image_id] <= 1'b1;
            end
            if (switch_valid && switch_ready) begin
                switch_valid <= 1'b0;
                switch_inflight <= 1'b1;
            end
            if (switch_result) begin
                switch_inflight <= 1'b0;
                active_valid <= 1'b1;
                active_image_id <= switch_image_id;
                active_bank <= switch_bank;
            end

            // Restart the small search after its assumptions change. Iterating
            // over file IDs takes a few control clocks, not a long priority/
            // modulo combinational chain in the 100 MHz timing path.
            if (manual_event) begin
                pending_skip <= 1'b0;
                request_pending <= 1'b1;
                pending_previous <= !next_event && prev_event;
                pending_auto <= 1'b0;
                state <= S_IDLE;
            end else if (accept_auto) begin
                pending_skip <= ENABLE_CUE_SKIP && auto_skip_unready;
                request_pending <= 1'b1;
                pending_previous <= 1'b0;
                pending_auto <= 1'b1;
                state <= S_IDLE;
            end else if (load_result || switch_result) begin
                state <= S_IDLE;
            end else begin
                case (state)
                S_IDLE: begin
                    if (catalog_seen && (|usable_mask)) begin
                        if (!switch_busy && (!active_valid || request_pending)) begin
                            walk_kind <= active_valid ? W_REQUEST : W_FIRST;
                            walk_previous <= active_valid && pending_previous;
                            walk_id <= active_valid ? advance_id(active_image_id, pending_previous) : 0;
                            walk_left <= MAX_IMAGES;
                            state <= S_WALK;
                        end else if (active_valid || switch_busy) begin
                            plan_center <= switch_busy ? switch_image_id : active_image_id;
                            plan_for_target <= 1'b0;
                            state <= S_PLAN;
                        end
                    end
                end
                S_WALK: begin
                    if (usable_mask[walk_id]) begin
                        case (walk_kind)
                        W_FIRST, W_REQUEST: begin
                            if (active_valid && walk_id == active_image_id) begin
                                // Only the current usable image remains.
                                request_pending <= 1'b0;
                                state <= S_IDLE;
                            end else begin
                                plan_center <= walk_id;
                                state <= S_TARGET;
                            end
                        end
                        W_NEXT: begin
                            wanted_id[1] <= walk_id;
                            walk_id <= advance_id(plan_center, 1'b1);
                            walk_previous <= 1'b1;
                            walk_left <= MAX_IMAGES;
                            walk_kind <= W_PREVIOUS;
                        end
                        W_PREVIOUS: begin
                            wanted_id[2] <= walk_id;
                            walk_id <= advance_id(wanted_id[1], 1'b0);
                            walk_previous <= 1'b0;
                            walk_left <= MAX_IMAGES;
                            walk_kind <= W_NEXT2;
                        end
                        default: begin
                            wanted_id[3] <= walk_id;
                            state <= S_SELECT;
                        end
                        endcase
                    end else if (walk_left == 1) begin
                        state <= S_IDLE;
                        request_pending <= 1'b0;
                    end else begin
                        walk_id <= advance_id(walk_id, walk_previous);
                        walk_left <= walk_left - 1'b1;
                    end
                end
                S_TARGET: begin
                    if (target_hit[2]) begin
                        switch_valid <= 1'b1;
                        switch_image_id <= plan_center;
                        switch_bank <= target_hit[1:0];
                        switch_initial <= !active_valid;
                        switch_auto <= active_valid && pending_auto;
                        switch_previous <= active_valid && pending_previous;
                        request_pending <= 1'b0;
                        state <= S_IDLE;
                    end else if (active_valid && pending_auto && pending_skip) begin
                        // A musical point expires now; never queue a late cut.
                        request_pending <= 1'b0;
                        state <= S_IDLE;
                    end else if (!load_busy) begin
                        plan_for_target <= 1'b1;
                        state <= S_PLAN;
                    end
                end
                S_PLAN: begin
                    prefetch_cursor <= 0;
                    wanted_id[0] <= plan_center;
                    walk_id <= advance_id(plan_center, 1'b0);
                    walk_previous <= 1'b0;
                    walk_left <= MAX_IMAGES;
                    walk_kind <= W_NEXT;
                    state <= S_WALK;
                end
                S_SELECT: begin
                    if (load_busy) state <= S_IDLE;
                    else begin
                        // Advance during this dedicated scan cycle regardless
                        // of the compare result. On a miss the cursor is not
                        // observed again before S_PLAN resets it; on a hit it
                        // selects the next neighbour as before. This removes
                        // the wanted_id -> cache compare -> cursor-enable path.
                        if (prefetch_cursor != 3)
                            prefetch_cursor <= prefetch_cursor + 1'b1;
                        if (plan_for_target || !prefetch_hit[2]) begin
                            work_image <= plan_for_target ? plan_center : prefetch_id;
                            state <= S_VICTIM_INIT;
                        end else if (prefetch_cursor == 3) state <= S_IDLE;
                    end
                end
                S_VICTIM_INIT: begin
                    victim_cursor <= 0;
                    victim_found <= 0;
                    best_rank <= 0;
                    state <= S_VICTIM;
                end
                S_VICTIM: begin
                    if (!protected_bank_mask[victim_cursor] &&
                        (plan_for_target || candidate_rank >= 4) &&
                        (!victim_found || candidate_rank > best_rank)) begin
                        victim_found <= 1;
                        victim_bank <= victim_cursor;
                        best_rank <= candidate_rank;
                    end
                    if (victim_cursor == 3) state <= S_PUBLISH;
                    else victim_cursor <= victim_cursor + 1'b1;
                end
                S_PUBLISH: begin
                    if (!load_busy && victim_found && !protected_bank_mask[victim_bank]) begin
                        // Invalidate BEFORE the adapter may write any pixels.
                        slot_valid_mask[victim_bank] <= 1'b0;
                        slot_id[victim_bank] <= work_image;
                        load_image_id <= work_image;
                        load_bank <= victim_bank;
                        load_valid <= 1'b1;
                    end
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
                endcase
            end
        end
    end
endmodule
