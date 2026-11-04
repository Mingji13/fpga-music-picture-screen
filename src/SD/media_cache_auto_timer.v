`timescale 1ns/1ps

// Only generates an event. All image selection and request queuing belongs
// to media_image_cache_scheduler. Background loading does not pause this timer.
module media_cache_auto_timer #(
    parameter integer PERIOD_CYCLES = 300000000
) (
    input wire clk, rst_n,
    input wire toggle_event,
    input wire display_valid,
    input wire switch_busy,
    input wire request_pending,
    input wire manual_event,
    output reg auto_enabled,
    output wire auto_event
);
    localparam integer CW = (PERIOD_CYCLES <= 1) ? 1 : $clog2(PERIOD_CYCLES);
    reg [CW-1:0] counter;
    wire paused = !display_valid || switch_busy || request_pending || manual_event || toggle_event;
    assign auto_event = rst_n && auto_enabled && !paused && counter == PERIOD_CYCLES-1;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin counter<=0;auto_enabled<=0;end
        else begin
            if(toggle_event) auto_enabled<=~auto_enabled;
            if(!auto_enabled || paused || auto_event) counter<=0;
            else counter<=counter+1'b1;
        end
    end
endmodule
