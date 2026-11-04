`timescale 1ns/1ps

// SDRAM user-interface burst admission, entirely in the memory clock domain.
// A client starts on the edge where request && grant is true, and raises
// active on that same edge. Grant is a start pulse, NOT per-word ready.
// Read active includes every outstanding return; write active includes the
// FIFO output pipeline. A running burst is never interrupted by arbitration.
//
// TURNAROUND_CYCLES is a conservative user-interface idle guard (>= 1).
// It is not an acknowledgement from the encrypted SDRAM controller. Keep
// staged preload/display until concurrent traffic and FIFO budgets are tested.
module media_sdram_burst_arbiter #(
    parameter integer TURNAROUND_CYCLES = 12
) (
    input  wire clk,
    input  wire rst_n,
    input  wire controller_busy,
    input  wire refresh_active,
    input  wire read_request,
    input  wire read_active,
    input  wire write_request,
    input  wire write_active,
    output wire read_grant,
    output wire write_grant
);
    localparam [1:0] IDLE = 2'd0, READ = 2'd1,
                     WRITE = 2'd2, DRAIN = 2'd3;
    localparam integer COUNT_WIDTH = (TURNAROUND_CYCLES <= 1) ? 1 :
                                      $clog2(TURNAROUND_CYCLES);
    reg [1:0] state;
    reg [COUNT_WIDTH-1:0] idle_count;
    reg last_was_read;
    wire quiet = !controller_busy && !refresh_active &&
                 !read_active && !write_active;
    wire can_grant = rst_n && (state == IDLE) && quiet;

    // Round robin if both clients are ready, otherwise serve the ready one.
    // State leaves IDLE at acceptance, so a held request cannot restart a
    // second burst while the current owner is finishing its first one.
    assign read_grant = can_grant && read_request &&
                        (!write_request || !last_was_read);
    assign write_grant = can_grant && write_request &&
                         (!read_request || last_was_read);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            idle_count <= 0;
            last_was_read <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    idle_count <= 0;
                    if (read_grant) begin
                        state <= READ;
                        last_was_read <= 1'b1;
                    end else if (write_grant) begin
                        state <= WRITE;
                        last_was_read <= 1'b0;
                    end
                end
                READ: if (!read_active) begin
                    state <= DRAIN;
                    idle_count <= 0;
                end
                WRITE: if (!write_active) begin
                    state <= DRAIN;
                    idle_count <= 0;
                end
                DRAIN: begin
                    if (!quiet) begin
                        idle_count <= 0;
                    end else if (idle_count == TURNAROUND_CYCLES - 1) begin
                        state <= IDLE;
                        idle_count <= 0;
                    end else begin
                        idle_count <= idle_count + 1'b1;
                    end
                end
                default: begin
                    state <= IDLE;
                    idle_count <= 0;
                end
            endcase
        end
    end
endmodule
