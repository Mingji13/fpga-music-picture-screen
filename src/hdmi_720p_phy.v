`timescale 1ns / 1ps

// 720p derivative of the official 10:1 serializer.
// The input is stable for one pixel period (five serial cycles). Capture
// after the toggle pipeline and one load register, at serial rising edge 4
// following each pixel edge. The first stage uses the falling edge, with
// 2.027 ns of nominal setup time instead of 0.676 ns at the rising edge.
// The SDC budgets word arrival to edge 4; shift timing and
// the related-clock toggle path remain naturally timed.
module hdmi_720p_lane (
    input wire I_serial_clk, I_serial_rst, I_load_en,
    input wire [9:0] I_data_in,
    output wire O_serial_out
);
    (* preserve = "true" *) reg S_load_en_1d;
    reg [9:0] S_shift_data;
    reg [1:0] S_oddr_data;

    always @(posedge I_serial_clk)
        S_load_en_1d <= I_load_en;

    always @(posedge I_serial_clk or posedge I_serial_rst)
        if (I_serial_rst) S_shift_data <= 10'd0;
        else if (S_load_en_1d) S_shift_data <= I_data_in;
        else S_shift_data <= S_shift_data << 2;

    always @(posedge I_serial_clk)
        S_oddr_data <= S_shift_data[9:8];

    EG_LOGIC_ODDR u_oddr (
        // A new valid symbol overwrites the full pipeline every pixel.
        // ODDR reset is unnecessary; it adds a long high-fanout recovery
        // path at 370 MHz. The monitor locks after valid symbols arrive.
        .clk(I_serial_clk), .rst(1'b0),
        .d0(S_oddr_data[1]), .d1(S_oddr_data[0]), .q(O_serial_out)
    );
endmodule

module hdmi_720p_phy (
    input wire I_pixel_clk, I_serial_clk,
    input wire I_pixel_rst, I_serial_rst,
    input wire [9:0] I_tmds_channel_0, I_tmds_channel_1,
    input wire [9:0] I_tmds_channel_2, I_tmds_channel_clk,
    output wire O_tmds_ch0_p, O_tmds_ch1_p, O_tmds_ch2_p, O_tmds_clk_p
);
    // Share ONE symbol strobe across all four lanes. The clocks are related
    // PLL outputs: video rises at phase 12.837 ns; serial rises next at
    // 13.514 ns and falls at 14.865 ns. Sample on that falling edge, then
    // transfer to the serial rising edge. STA must check both paths;
    // do not mask this path with a one-serial-period max-delay exception.
    reg S_odd_even;
    (* preserve = "true" *) reg S_odd_even_1d, S_odd_even_2d, S_odd_even_3d;
    always @(posedge I_pixel_clk or posedge I_pixel_rst)
        if (I_pixel_rst) S_odd_even <= 1'b0;
        else S_odd_even <= ~S_odd_even;
    always @(negedge I_serial_clk)
        S_odd_even_1d <= S_odd_even;
    always @(posedge I_serial_clk) begin
        S_odd_even_2d <= S_odd_even_1d;
        S_odd_even_3d <= S_odd_even_2d;
    end
    // XOR the two rising-edge samples over a FULL serial period before
    // each lane's load register. This is cycle-equivalent to registering
    // (1d ^ 2d), but leaves only a direct register transfer on the short
    // falling-to-rising path. Pixel-word capture remains on rising edge 4.
    wire S_load_en = S_odd_even_2d ^ S_odd_even_3d;
    wire [39:0] source_words = {I_tmds_channel_clk, I_tmds_channel_2,
                               I_tmds_channel_1, I_tmds_channel_0};
    wire [3:0] serial_out;
    genvar lane, bit_index;
    generate for (lane=0; lane<4; lane=lane+1) begin: lanes
        wire [9:0] reversed;
        for (bit_index=0; bit_index<10; bit_index=bit_index+1) begin: bits
            assign reversed[9-bit_index] = source_words[lane*10+bit_index];
        end
        hdmi_720p_lane u_lane (
            .I_serial_clk(I_serial_clk), .I_serial_rst(I_serial_rst),
            .I_load_en(S_load_en),
            .I_data_in(reversed), .O_serial_out(serial_out[lane])
        );
    end endgenerate
    assign {O_tmds_clk_p, O_tmds_ch2_p, O_tmds_ch1_p, O_tmds_ch0_p} = serial_out;
endmodule
