`timescale 1ns/1ps

// MCUE v1 validator and one-song dual-clock cue RAM. Metadata/RAM are stable
// before the playlist releases the PCM ring reset; no live table replacement.
module music_cue_table (
    input wire clk, rst_n,
    input wire prepare, load_start,
    input wire [31:0] file_size, pcm_samples,
    input wire byte_valid,
    input wire [7:0] byte_data,
    input wire [31:0] byte_offset,
    input wire file_done,
    input wire [7:0] file_error,
    output reg table_valid,
    output reg [10:0] cue_count,
    input wire play_clk, play_rst_n,
    // True only for a real file sample removed from the PCM FIFO (not silence).
    input wire sample_take,
    output reg event_toggle,
    output wire play_ready
);
    (* ram_style = "bram" *) reg [31:0] positions [0:1023];
    reg [31:0] read_position;
    reg [9:0] read_address;
    reg [31:0] word_data;
    reg [10:0] word_index;
    reg word_valid, receiving, invalid;
    reg [1:0] finish_delay;
    reg [31:0] checksum, previous_position;
    reg [12:0] received_bytes;
    reg range_high_gt, range_high_eq, range_low_ge;
    reg order_high_lt, order_high_eq, order_low_le;
    wire payload_word = word_index >= 16 && word_index < 1040;
    // The loader and playlist hold these descriptors for the entire load.
    // Count/size arithmetic is bounded by MCUE's 1024-point capacity.
    wire [12:0] declared_size=13'd64+{cue_count,2'b00};

    // No reset on either RAM port, preserving BRAM inference.
    always @(posedge clk) begin
        if(receiving && word_valid && payload_word)
            positions[word_index[9:0]-10'd16] <= word_data;
    end
    always @(posedge play_clk) read_position <= positions[read_address];

    // One word arrives every four valid bytes at most. Precheck its two
    // halves as the last byte arrives; the following word_valid cycle only
    // combines flags. This avoids a 32-bit compare plus parser decode in one
    // path. Previous_position is already committed before the next word.
    always @(posedge clk) begin
        if(receiving && byte_valid && byte_offset[1:0]==3) begin
            range_high_gt<={byte_data,word_data[23:16]}>pcm_samples[31:16];
            range_high_eq<={byte_data,word_data[23:16]}==pcm_samples[31:16];
            range_low_ge<=word_data[15:0]>=pcm_samples[15:0];
            order_high_lt<={byte_data,word_data[23:16]}<previous_position[31:16];
            order_high_eq<={byte_data,word_data[23:16]}==previous_position[31:16];
            order_low_le<=word_data[15:0]<=previous_position[15:0];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            table_valid<=0;cue_count<=0;word_data<=0;word_index<=0;
            word_valid<=0;receiving<=0;invalid<=0;finish_delay<=0;
            checksum<=0;previous_position<=0;
            received_bytes<=0;
        end else begin
            word_valid<=0;
            if(prepare) begin table_valid<=0;receiving<=0;finish_delay<=0;end
            if(load_start) begin
                table_valid<=0;cue_count<=0;receiving<=1;invalid<=0;finish_delay<=0;
                checksum<=0;previous_position<=0;received_bytes<=0;
            end else if(receiving) begin
                if(byte_valid) begin
                    received_bytes<=received_bytes+1'b1;
                    if(byte_offset!={19'd0,received_bytes}) invalid<=1;
                    case(byte_offset[1:0])
                        0:word_data[7:0]<=byte_data;
                        1:word_data[15:8]<=byte_data;
                        2:word_data[23:16]<=byte_data;
                        3:begin word_data[31:24]<=byte_data;word_index<=byte_offset[12:2];word_valid<=1;end
                    endcase
                end
                if(word_valid) begin
                    checksum<=checksum+word_data;
                    case(word_index)
                        0:if(word_data!=32'h4555434d) invalid<=1; // bytes MCUE
                        1:if(word_data!=32'h00400001) invalid<=1;
                        2:if(word_data!=48000) invalid<=1;
                        3:if(word_data==0 || word_data!=pcm_samples) invalid<=1;
                        4:begin
                            if(word_data==0 || word_data>1024) invalid<=1;
                            cue_count<=word_data[10:0];
                        end
                        5:if(word_data!=1) invalid<=1;
                        15:if(word_data!=0) invalid<=1;
                        default:if(payload_word) begin
                            if(range_high_gt || (range_high_eq && range_low_ge) ||
                               (word_index!=16 && (order_high_lt || (order_high_eq && order_low_le)))) invalid<=1;
                            previous_position<=word_data;
                        end
                    endcase
                end
                if(file_done) begin
                    if(file_error!=0) invalid<=1;
                    finish_delay<=2;
                end else if(finish_delay!=0) begin
                    finish_delay<=finish_delay-1'b1;
                    if(finish_delay==1) begin
                        receiving<=0;
                        table_valid<=!invalid && checksum==0 &&
                            file_size[31:13]==0 && declared_size==file_size[12:0] &&
                            received_bytes==file_size[12:0];
                    end
                end
            end
        end
    end

    wire valid_play;
    sync_2ff u_valid_sync(.clk(play_clk),.rst_n(play_rst_n),
        .async_in(table_valid),.sync_out(valid_play));
    reg [31:0] sample_index;
    reg [1:0] play_state;
    assign play_ready=play_state==2 || play_state==3;
    // Preserve the toggle across song resets so a reset cannot look like a cue.
    always @(posedge play_clk or negedge rst_n) begin
        if(!rst_n) event_toggle<=0;
        else if(play_rst_n && valid_play && play_state==2 &&
                sample_take && read_position==sample_index) event_toggle<=~event_toggle;
    end
    // Table count is held with the RAM while play_rst_n is released. Only
    // the validity crosses a synchronizer; no independently sampled live bus.
    always @(posedge play_clk or negedge play_rst_n) begin
        if(!play_rst_n) begin
            read_address<=0;sample_index<=0;
            play_state<=0;
        end else begin
            if(sample_take) sample_index<=sample_index+1'b1;
            case(play_state)
                0:if(valid_play) begin read_address<=0;play_state<=1;end
                1:play_state<=2;
                // The RAM read address is held while waiting for its cue;
                // read_position itself is the registered next position.
                2:if(valid_play) begin
                    if(read_position<sample_index || (sample_take && read_position==sample_index)) begin
                        if({1'b0,read_address}+11'd1>=cue_count) play_state<=3;
                        else begin read_address<=read_address+1'b1;play_state<=1;end
                    end
                end else play_state<=0;
                default:begin end // Exhausted: never fall back to the timer mid-song.
            endcase
        end
    end
endmodule
