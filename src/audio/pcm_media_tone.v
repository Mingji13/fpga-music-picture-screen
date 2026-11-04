`timescale 1ns/1ps

// A committed picture change in enabled arpeggio mode plays six notes once.
// Idle, disabled and completed playback still produce zero PCM at 48 kHz.
// Entering the mode alone does not trigger playback; a new picture restarts
// the next of four melodies from its first note, independently of file ID.
// Other modes pause the melody cycle. There is no separate single-tone cue.
// The FIFO/valid/ready contract matches hdmi_pcm_tone_fifo so the audio
// packetizer chain is unchanged.
module pcm_media_tone #(
    parameter integer CLOCK_HZ = 25000000,
    parameter integer SAMPLE_HZ = 48000,
    parameter integer NOTE_SAMPLES = 12000,  // 250 ms per melody note
    // Final note only: 256 samples / 48 kHz = 5.33 ms. 0 disables release.
    // Prefer powers of two so the constant division reduces to a shift.
    parameter integer RELEASE_SAMPLES = 256,
    parameter integer MEDIA_ID_WIDTH = 2
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable,
    input  wire [MEDIA_ID_WIDTH-1:0] media_id,
    input  wire [7:0]         volume,
    output wire               sample_valid,
    input  wire               sample_ready,
    output wire signed [15:0] sample_left,
    output wire signed [15:0] sample_right,
    output reg                overflow,
    output reg                playback_active
);
    localparam integer NOTE_COUNT_BITS = (NOTE_SAMPLES < 2) ? 1 :
                                         $clog2(NOTE_SAMPLES);
    localparam integer RATE_BITS = (CLOCK_HZ < 2) ? 1 : $clog2(CLOCK_HZ);
    reg [RATE_BITS-1:0] rate_accumulator;
    reg [19:0] phase;
    reg [MEDIA_ID_WIDTH-1:0] media_id_q;
    reg [2:0] note_index;
    reg [NOTE_COUNT_BITS-1:0] note_sample_count;
    reg signed [15:0] fifo [0:7];
    reg [2:0] write_pointer;
    reg [2:0] read_pointer;
    reg [3:0] fifo_count;
    wire sample_pop;
    wire sample_tick;
    wire sample_push;

    assign sample_valid = fifo_count != 0;
    assign sample_left = fifo[read_pointer];
    assign sample_right = fifo[read_pointer];
    assign sample_pop = sample_valid && sample_ready;
    assign sample_tick = rate_accumulator >= CLOCK_HZ - SAMPLE_HZ;
    assign sample_push = sample_tick && (fifo_count != 8 || sample_pop);

    // 12-tone equal temperament, A4 = 440 Hz; rounded for 48 kHz DDS.
    // phase_inc = freq * 2**20 / SAMPLE_HZ (freq * 21.8453)
    function [19:0] melody_inc;
        input [1:0] melody;
        input [2:0] note;
        begin
            case ({melody, note})
                // 0: Cmaj7, bright arch: C5 E5 G5 B5 G5 C5
                5'b00_000: melody_inc = 20'd11431;  // C5 523.251 Hz
                5'b00_001: melody_inc = 20'd14402;  // E5 659.255 Hz
                5'b00_010: melody_inc = 20'd17127;  // G5 784 Hz
                5'b00_011: melody_inc = 20'd21578;  // B5 987.767 Hz
                5'b00_100: melody_inc = 20'd17127;  // G5
                5'b00_101: melody_inc = 20'd11431;  // C5
                // 1: Fmaj7, gentle descent: A4 C5 E5 C5 A4 F4
                5'b01_000: melody_inc = 20'd9612;   // A4 440 Hz
                5'b01_001: melody_inc = 20'd11431;  // C5
                5'b01_010: melody_inc = 20'd14402;  // E5
                5'b01_011: melody_inc = 20'd11431;  // C5
                5'b01_100: melody_inc = 20'd9612;   // A4
                5'b01_101: melody_inc = 20'd7629;   // F4 349.228 Hz
                // 2: Am7, reflective wave: E5 C5 A4 G4 C5 A4
                5'b10_000: melody_inc = 20'd14402;  // E5
                5'b10_001: melody_inc = 20'd11431;  // C5
                5'b10_010: melody_inc = 20'd9612;   // A4
                5'b10_011: melody_inc = 20'd8563;   // G4 391.995 Hz
                5'b10_100: melody_inc = 20'd11431;  // C5
                5'b10_101: melody_inc = 20'd9612;   // A4
                // 3: G6, hopeful rise: G4 B4 D5 B4 E5 G5
                5'b11_000: melody_inc = 20'd8563;   // G4
                5'b11_001: melody_inc = 20'd10789;  // B4 493.883 Hz
                5'b11_010: melody_inc = 20'd12830;  // D5 587.330 Hz
                5'b11_011: melody_inc = 20'd10789;  // B4
                5'b11_100: melody_inc = 20'd14402;  // E5
                5'b11_101: melody_inc = 20'd17127;  // G5
                default:   melody_inc = 20'd11431;
            endcase
        end
    endfunction

    // Full file ID detects committed changes, but does not select the melody.
    // Start at 3 so the first enabled change advances to melody 0. Disabled
    // picture changes are still tracked without consuming a melody.
    reg [1:0] melody_id;
    reg [19:0] melody_note_inc;
    wire playing_sample = enable && playback_active && (media_id == media_id_q);
    wire last_playing_sample = sample_push && playing_sample &&
                              (note_index == 3'd5) &&
                              (note_sample_count == NOTE_SAMPLES - 1);
    // Keep raw_sample aligned with phase, including same-edge restart/end.
    // Select reset/hold AFTER the sine LUT: sample/FIFO/note control no longer
    // passes through the phase mux, quarter lookup and sign arithmetic.
    // The generated PCM samples and FIFO timing are cycle-for-cycle unchanged.
    wire phase_restart = !enable || (media_id != media_id_q) || last_playing_sample;
    wire phase_advance = sample_push && playing_sample;
    // Decode the pitch one clock ahead, alongside the note/file state update.
    // This removes melody decoding from the phase-add + sine-LUT path without
    // changing any PCM sample or its timestamp, even at two clocks/sample.
    wire [2:0] next_note_index = phase_restart ? 3'd0 :
        (phase_advance && note_sample_count == NOTE_SAMPLES-1) ? note_index+3'd1 : note_index;
    wire [19:0] advanced_phase = phase + melody_note_inc;
    wire [19:0] phase_next = phase_restart ? 20'd0 :
                            phase_advance ? advanced_phase : phase;

    // Same 256-bin, +/-12000 sine as the test source; keep the existing
    // 20-bit pitch DDS, 48 kHz cadence, volume and one-shot sequencing.
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

    reg signed [15:0] raw_sample;
    wire signed [23:0] scaled = raw_sample * $signed({1'b0, volume});
    wire signed [15:0] scaled_clamped =
        (scaled > 24'sd8388352)  ? 16'sd32767 :
        (scaled < -24'sd8388608) ? -16'sd32768 : scaled >>> 8;

    // Apply the release before enqueueing so stalled FIFO samples stay stable.
    // Shortened simulations/notes clamp the tail length to the whole last note.
    // The first tail sample is just below full gain; the last is exactly zero.
    wire signed [15:0] playback_sample;
    generate
        if (RELEASE_SAMPLES > 0) begin : g_release
            localparam integer RELEASE_LENGTH = (RELEASE_SAMPLES < NOTE_SAMPLES) ?
                                                 RELEASE_SAMPLES : NOTE_SAMPLES;
            wire release_active = (note_index == 3'd5) &&
                                  (note_sample_count >= NOTE_SAMPLES - RELEASE_LENGTH);
            wire [NOTE_COUNT_BITS-1:0] samples_after_current =
                (NOTE_SAMPLES - 1) - note_sample_count;
            wire [NOTE_COUNT_BITS+7:0] gain_numerator = {samples_after_current, 8'd0};
            wire [7:0] release_gain = gain_numerator / RELEASE_LENGTH;
            wire signed [24:0] release_product =
                scaled_clamped * $signed({1'b0, release_gain});
            assign playback_sample = release_active ? (release_product >>> 8) : scaled_clamped;
        end else begin : g_no_release
            assign playback_sample = scaled_clamped;
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rate_accumulator <= 0;
            phase <= 20'd0;
            raw_sample <= 16'sd0;
            media_id_q <= 0;
            melody_id <= 2'd3;
            melody_note_inc <= 20'd11431;
            playback_active <= 1'b0;
            note_index <= 3'd0;
            note_sample_count <= 0;
            write_pointer <= 3'd0;
            read_pointer <= 3'd0;
            fifo_count <= 4'd0;
            overflow <= 1'b0;
        end else begin
            media_id_q <= media_id;
            melody_note_inc <= melody_inc(melody_id, next_note_index);
            phase <= phase_next;
            if (phase_restart)
                raw_sample <= 16'sd0;
            else if (phase_advance)
                raw_sample <= sine_sample(advanced_phase[19:12]);

            if (sample_tick)
                rate_accumulator <= rate_accumulator + SAMPLE_HZ - CLOCK_HZ;
            else
                rate_accumulator <= rate_accumulator + SAMPLE_HZ;

            if (sample_push) begin
                fifo[write_pointer] <= playing_sample ? playback_sample : 16'sd0;
                write_pointer <= write_pointer + 1'b1;
                if (playing_sample) begin
                    if (note_sample_count == NOTE_SAMPLES - 1) begin
                        note_sample_count <= 0;
                        if (note_index == 3'd5) begin
                            playback_active <= 1'b0;
                            note_index <= 3'd0;
                        end else begin
                            note_index <= note_index + 1'b1;
                        end
                    end else begin
                        note_sample_count <= note_sample_count + 1'b1;
                    end
                end
            end else if (sample_tick) begin
                overflow <= 1'b1;
            end

            if (sample_pop)
                read_pointer <= read_pointer + 1'b1;

            case ({sample_push, sample_pop})
                2'b10: fifo_count <= fifo_count + 1'b1;
                2'b01: fifo_count <= fifo_count - 1'b1;
                default: fifo_count <= fifo_count;
            endcase

            // Always track the displayed picture, including in other modes,
            // so returning to arpeggio mode cannot replay a hidden change.
            // Event handling wins over a same-edge sample/note completion.
            if (!enable) begin
                playback_active <= 1'b0;
                note_index <= 3'd0;
                note_sample_count <= 0;
            end else if (media_id != media_id_q) begin
                melody_id <= melody_id + 2'd1;
                // Register the new first pitch on the trigger itself, keeping
                // the existing pitch pipeline correct even at two clocks/sample.
                melody_note_inc <= melody_inc(melody_id + 2'd1, 3'd0);
                playback_active <= 1'b1;
                note_index <= 3'd0;
                note_sample_count <= 0;
            end
        end
    end
endmodule
