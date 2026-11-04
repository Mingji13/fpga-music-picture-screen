`timescale 1ns/1ps

module pcm_test_tone #(
    // Integer division: 25 MHz / 521 = approximately 47.985 kHz.
    // Counts sample intervals; the module clock remains at 25 MHz.
    parameter integer SAMPLE_DIV = 521,
    // Match the original lesson-1 I2S test: C4 through C5, 0.5 s per note.
    parameter integer NOTE_HOLD_SAMPLES = 24000
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,
    output reg                sample_valid,
    input  wire               sample_ready,
    output reg signed [15:0]  sample_left,
    output reg signed [15:0]  sample_right,
    output wire [3:0]         rom_index,
    output wire               sample_accepted
);
    localparam integer DIV_WIDTH = (SAMPLE_DIV <= 2) ? 1 : $clog2(SAMPLE_DIV);
    localparam integer NOTE_COUNT_WIDTH = (NOTE_HOLD_SAMPLES <= 2) ? 1 : $clog2(NOTE_HOLD_SAMPLES);
    reg [DIV_WIDTH-1:0] div_count;
    reg [31:0] phase_acc;
    reg [2:0] note_index;
    reg [NOTE_COUNT_WIDTH-1:0] note_sample_count;

    assign sample_accepted = sample_valid && sample_ready;
    assign rom_index = phase_acc[31:28];

    // These are the eight phase increments from hdmi_audio_tone_i2s_64fs.v
    // in the original lesson-1 example. Pitch DDS is independent of the
    // existing integer sample-clock divider; no extra PLL is required.
    function [31:0] note_increment;
        input [2:0] index;
        begin
            case (index)
                3'd0: note_increment = 32'd23409862; // C4, 261.63 Hz
                3'd1: note_increment = 32'd26276681; // D4, 293.66 Hz
                3'd2: note_increment = 32'd29494578; // E4, 329.63 Hz
                3'd3: note_increment = 32'd31248410; // F4, 349.23 Hz
                3'd4: note_increment = 32'd35075155; // G4, 392.00 Hz
                3'd5: note_increment = 32'd39370534; // A4, 440.00 Hz
                3'd6: note_increment = 32'd44191930; // B4, 493.88 Hz
                default: note_increment = 32'd46819716; // C5, 523.25 Hz
            endcase
        end
    endfunction

    // 256 phase bins instead of 16, with the same +/-12000 peak.
    // Store just the inclusive first quadrant (0..pi/2) and mirror/sign it.
    // This changes waveform resolution only: no sample-rate, gain, DDS
    // increment, handshake or pipeline-latency change. rom_index remains
    // the old four-bit diagnostic output; it no longer addresses the LUT.
    function [15:0] sine_quarter;
        input [6:0] index;
        begin
            case (index)
                7'd0: sine_quarter = 16'd0;
                7'd1: sine_quarter = 16'd294;
                7'd2: sine_quarter = 16'd589;
                7'd3: sine_quarter = 16'd883;
                7'd4: sine_quarter = 16'd1176;
                7'd5: sine_quarter = 16'd1469;
                7'd6: sine_quarter = 16'd1761;
                7'd7: sine_quarter = 16'd2052;
                7'd8: sine_quarter = 16'd2341;
                7'd9: sine_quarter = 16'd2629;
                7'd10: sine_quarter = 16'd2916;
                7'd11: sine_quarter = 16'd3201;
                7'd12: sine_quarter = 16'd3483;
                7'd13: sine_quarter = 16'd3764;
                7'd14: sine_quarter = 16'd4043;
                7'd15: sine_quarter = 16'd4319;
                7'd16: sine_quarter = 16'd4592;
                7'd17: sine_quarter = 16'd4863;
                7'd18: sine_quarter = 16'd5131;
                7'd19: sine_quarter = 16'd5395;
                7'd20: sine_quarter = 16'd5657;
                7'd21: sine_quarter = 16'd5915;
                7'd22: sine_quarter = 16'd6169;
                7'd23: sine_quarter = 16'd6420;
                7'd24: sine_quarter = 16'd6667;
                7'd25: sine_quarter = 16'd6910;
                7'd26: sine_quarter = 16'd7148;
                7'd27: sine_quarter = 16'd7383;
                7'd28: sine_quarter = 16'd7613;
                7'd29: sine_quarter = 16'd7838;
                7'd30: sine_quarter = 16'd8059;
                7'd31: sine_quarter = 16'd8274;
                7'd32: sine_quarter = 16'd8485;
                7'd33: sine_quarter = 16'd8691;
                7'd34: sine_quarter = 16'd8891;
                7'd35: sine_quarter = 16'd9087;
                7'd36: sine_quarter = 16'd9276;
                7'd37: sine_quarter = 16'd9460;
                7'd38: sine_quarter = 16'd9638;
                7'd39: sine_quarter = 16'd9811;
                7'd40: sine_quarter = 16'd9978;
                7'd41: sine_quarter = 16'd10138;
                7'd42: sine_quarter = 16'd10293;
                7'd43: sine_quarter = 16'd10441;
                7'd44: sine_quarter = 16'd10583;
                7'd45: sine_quarter = 16'd10719;
                7'd46: sine_quarter = 16'd10848;
                7'd47: sine_quarter = 16'd10971;
                7'd48: sine_quarter = 16'd11087;
                7'd49: sine_quarter = 16'd11196;
                7'd50: sine_quarter = 16'd11299;
                7'd51: sine_quarter = 16'd11394;
                7'd52: sine_quarter = 16'd11483;
                7'd53: sine_quarter = 16'd11565;
                7'd54: sine_quarter = 16'd11640;
                7'd55: sine_quarter = 16'd11708;
                7'd56: sine_quarter = 16'd11769;
                7'd57: sine_quarter = 16'd11823;
                7'd58: sine_quarter = 16'd11870;
                7'd59: sine_quarter = 16'd11910;
                7'd60: sine_quarter = 16'd11942;
                7'd61: sine_quarter = 16'd11967;
                7'd62: sine_quarter = 16'd11986;
                7'd63: sine_quarter = 16'd11996;
                7'd64: sine_quarter = 16'd12000;
                default: sine_quarter = 16'd0;
            endcase
        end
    endfunction

    function signed [15:0] sine_sample;
        input [7:0] index;
        reg [6:0] quarter_index;
        reg signed [15:0] magnitude;
        begin
            quarter_index = index[6] ? (7'd64 - {1'b0, index[5:0]}) :
                                      {1'b0, index[5:0]};
            magnitude = $signed(sine_quarter(quarter_index));
            sine_sample = index[7] ? -magnitude : magnitude;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_valid <= 1'b0;
            sample_left <= 16'sd0;
            sample_right <= 16'sd0;
            phase_acc <= 32'd0;
            note_index <= 3'd0;
            note_sample_count <= {NOTE_COUNT_WIDTH{1'b0}};
            div_count <= {DIV_WIDTH{1'b0}};
        end else if (!enable) begin
            sample_valid <= 1'b0;
            phase_acc <= 32'd0;
            note_index <= 3'd0;
            note_sample_count <= {NOTE_COUNT_WIDTH{1'b0}};
            div_count <= {DIV_WIDTH{1'b0}};
        end else begin
            if (div_count == SAMPLE_DIV-1)
                div_count <= {DIV_WIDTH{1'b0}};
            else
                div_count <= div_count + 1'b1;

            if (sample_accepted) begin
                sample_valid <= 1'b0;
                // Advance musical time only when the sample is consumed.
                // Keep phase continuous when moving to the next note.
                phase_acc <= phase_acc + note_increment(note_index);
                if (note_sample_count == NOTE_HOLD_SAMPLES-1) begin
                    note_sample_count <= {NOTE_COUNT_WIDTH{1'b0}};
                    note_index <= note_index + 3'd1;
                end else begin
                    note_sample_count <= note_sample_count + 1'b1;
                end
            end

            if (!sample_valid && div_count == SAMPLE_DIV-1) begin
                sample_left <= sine_sample(phase_acc[31:24]);
                sample_right <= sine_sample(phase_acc[31:24]);
                sample_valid <= 1'b1;
            end
        end
    end
endmodule
