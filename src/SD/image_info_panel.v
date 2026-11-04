`timescale 1ns/1ps

// Eight common-anode digits, active-low segment and PNP digit-select pins.
// seg_sel[7] is the leftmost digit. Pages: P + file number, raw width/height.
// Passive metadata consumer: it never stalls or controls image/audio traffic.
module image_info_panel #(
    parameter integer SLOT_CYCLES = 100000, // 1 ms @ 100 MHz, full scan 125 Hz
    parameter integer BLANK_CYCLES = 1000, // >= 432: digit conversion + PNP release
    parameter integer PAGE_SCANS = 250    // 2 s per page
) (
    input wire clk, rst_n,
    input wire metadata_valid,
    input wire [1:0] metadata_bank,
    input wire [10:0] source_width, source_height,
    input wire display_valid, display_commit,
    input wire [1:0] display_bank,
    input wire [3:0] display_id, // PIC1..PIC16 represented by 0..15
    output reg [7:0] seg_sel, seg_data
);
    localparam integer SLOT_BITS = SLOT_CYCLES < 2 ? 1 : $clog2(SLOT_CYCLES);
    localparam integer PAGE_BITS = PAGE_SCANS < 2 ? 1 : $clog2(PAGE_SCANS);
    // Use embedded RAM to relieve the nearly full slice fabric. A successful
    // load writes metadata before commit. RAM and its read register need no
    // reset: info_ready hides their contents until the next successful commit.
    (* ram_style = "bram" *) reg [21:0] bank_size [0:3];
    always @(posedge clk)
        if (rst_n && metadata_valid)
            bank_size[metadata_bank] <= {source_width,source_height};

    reg [21:0] displayed_size;
    always @(posedge clk)
        if (display_commit)
            displayed_size <= bank_size[display_bank];
    reg [7:0] picture_digits;
    reg info_ready;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            picture_digits<=0; info_ready<=0;
        end else if (!display_valid) info_ready<=0;
        else if (display_commit) begin
            info_ready<=1;
            if (display_id >= 4'd9)
                picture_digits <= {4'd1,(display_id - 4'd9)};
            else picture_digits <= {4'd0,(display_id + 4'd1)};
        end
    end

    reg [SLOT_BITS-1:0] slot_count;
    reg [PAGE_BITS-1:0] page_count;
    reg [2:0] digit_index;
    reg dimension_page;
    wire slot_end = (slot_count==SLOT_CYCLES-1);
    reg slot_start;
    reg [10:0] remainder, divisor;
    reg [3:0] decimal_digit;
    reg reducing, converting, higher_nonzero;
    // During the blank interval, first remove more-significant digits, then
    // count this digit. One subtractor serves all eight positions. At most
    // 214 clocks for an 11-bit value, well before segment latch at clock 500.
    always @* begin
        case (digit_index[1:0])
            0: divisor=reducing ? 11'd10 : 11'd1;
            1: divisor=reducing ? 11'd100 : 11'd10;
            2: divisor=reducing ? 11'd1000 : 11'd100;
            default: divisor=11'd1000;
        endcase
    end
    wire [11:0] difference={1'b0,remainder}-{1'b0,divisor};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            remainder<=0; decimal_digit<=0; reducing<=0;
            converting<=0; higher_nonzero<=0;
        end else if (display_commit || !display_valid) converting<=0;
        else if (slot_start) begin
            remainder<=digit_index[2] ? displayed_size[21:11] : displayed_size[10:0];
            decimal_digit<=0; reducing<=(digit_index[1:0]!=3);
            converting<=1; higher_nonzero<=0;
        end else if (converting) begin
            if (!difference[11]) begin
                remainder<=difference[10:0];
                if (reducing) higher_nonzero<=1;
                else decimal_digit<=decimal_digit+1'b1;
            end else if (reducing) reducing<=0;
            else converting<=0;
        end
    end
    reg [3:0] digit;
    reg digit_blank;
    always @* begin
        digit = decimal_digit;
        digit_blank = !higher_nonzero && (decimal_digit==0) && (digit_index[1:0]!=0);
        if (!dimension_page) begin
            digit_blank = 1'b0;
            case (digit_index)
                7: digit=4'ha; // P
                1: begin digit=picture_digits[7:4]; digit_blank=(digit==0); end
                0: digit=picture_digits[3:0];
                default: digit_blank=1;
            endcase
        end
    end
    function [7:0] segments;
        input [3:0] value;
        begin
            case (value)
                0: segments=8'hc0; 1: segments=8'hf9;
                2: segments=8'ha4; 3: segments=8'hb0;
                4: segments=8'h99; 5: segments=8'h92;
                6: segments=8'h82; 7: segments=8'hf8;
                8: segments=8'h80; 9: segments=8'h90;
                10: segments=8'h8c; // P
                default: segments=8'hff;
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            slot_count<=0; page_count<=0; digit_index<=0; dimension_page<=0;
            slot_start<=1;
            seg_sel<=8'hff; seg_data<=8'hff;
        end else begin
            // Register the next slot's start one clock early. This preserves
            // the exact start cycle while removing a wide timer decode from
            // the decimal counter's arithmetic/control path.
            slot_start<=slot_end;
            if (slot_end) begin
                seg_sel<=8'hff;
                slot_count<=0;
                digit_index<=digit_index+1'b1;
                if (digit_index==7) begin
                    if (page_count==PAGE_SCANS-1) begin
                        page_count<=0; dimension_page<=~dimension_page;
                    end else page_count<=page_count+1'b1;
                end
            end else slot_count<=slot_count+1'b1;
            // Break before make: disable, change segments, then enable.
            if (slot_count==0) seg_sel<=8'hff;
            if (slot_count==BLANK_CYCLES/2 && !converting)
                seg_data<=digit_blank ? 8'hff : segments(digit);
            if (slot_count==BLANK_CYCLES && !converting)
                seg_sel<=~(8'b1 << digit_index);
            if (!display_valid || !info_ready || display_commit) begin
                seg_sel<=8'hff;
                slot_count<=0; digit_index<=0; page_count<=0; dimension_page<=0;
                slot_start<=1;
            end
        end
    end
endmodule
