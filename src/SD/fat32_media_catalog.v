`timescale 1ns/1ps

// Root-directory catalogue for the dynamic image cache. File IDs are NOT bank
// numbers: ID 0 means PIC1.BMP regardless of which SDRAM slot holds that image.
// MAX_IMAGES is 1..99; the normal project configuration reserves 16 records.
// Read query outputs only after scan_done and one rising clock with a stable
// query_image_id (synchronous metadata RAM). A missing name is an empty slot,
// not a requirement to populate every ID. SONG1.PCM is independently optional.
// Supports sector-0 FAT32 or the first MBR partition (type 0B/0C), 512-byte
// sectors, FAT #0 and short names in the root directory (no subdirectory/LFN).
module fat32_media_catalog #(
    parameter integer MAX_IMAGES = 16,
    parameter integer ENABLE_PLAYLIST = 0,
    parameter integer ENABLE_CUES = 0,
    // Consumers must qualify descriptors with query_*_valid. Raw shared RAM
    // data avoids three redundant 64-bit zeroing muxes in the integrated core.
    // Keep the legacy zero-on-invalid interface as the default.
    parameter integer RAW_QUERY_DATA = 0,
    parameter integer MAX_SONGS = 16,
    parameter integer SONG_ID_WIDTH = MAX_SONGS <= 1 ? 1 : $clog2(MAX_SONGS),
    parameter integer MAX_DIR_CLUSTERS = 256,
    parameter integer IMAGE_ID_WIDTH = (MAX_IMAGES <= 1) ? 1 : $clog2(MAX_IMAGES),
    parameter integer IMAGE_COUNT_WIDTH = $clog2(MAX_IMAGES+1)
) (
    input wire clk, rst_n, scan_start,
    output wire scan_busy,
    output reg scan_done,
    output reg [7:0] fs_error,
    output reg [31:0] partition_lba,
    output reg [7:0] sectors_per_cluster,
    output reg [31:0] fat_start_lba, data_start_lba, max_cluster,
    output reg [MAX_IMAGES-1:0] image_valid_mask,
    output reg [IMAGE_COUNT_WIDTH-1:0] image_count,
    input wire [IMAGE_ID_WIDTH-1:0] query_image_id,
    output wire query_valid,
    output wire [31:0] query_cluster, query_size,
    output reg audio_valid,
    output reg [31:0] audio_cluster, audio_size,
    output reg [MAX_SONGS-1:0] song_valid_mask,
    input wire query_song_enable,
    input wire [SONG_ID_WIDTH-1:0] query_song_id,
    output wire query_song_valid,
    output wire [31:0] query_song_cluster, query_song_size,
    output reg [MAX_SONGS-1:0] cue_valid_mask,
    input wire query_cue_enable,
    output wire query_cue_valid,
    output wire [31:0] query_cue_cluster, query_cue_size,
    output reg sector_req,
    output reg [31:0] sector_lba,
    input wire sector_busy, sector_valid,
    input wire [7:0] sector_byte,
    input wire [8:0] sector_byte_index,
    input wire sector_done,
    input wire [7:0] sector_error
);
    localparam [7:0] ERR_NONE=0, ERR_SECTOR=8'h10, ERR_BOOT_SIGNATURE=8'h20,
        ERR_UNSUPPORTED_BPB=8'h21, ERR_NO_IMAGE=8'h22, ERR_FAT_LOOP=8'h23,
        ERR_CLUSTER_RANGE=8'h24, ERR_DIR_LIMIT=8'h26;
    localparam [4:0] IDLE=0, READ0_REQ=1, READ0_WAIT=2, READ0_PARSE=3,
        BOOT_REQ=4, BOOT_WAIT=5, BOOT_PARSE=6, BOOT_LAYOUT=7, BOOT_BOUNDS=8,
        BOOT_ROOT=9, CLUSTER_OFFSET=10, CLUSTER_SCALE=11, CLUSTER_LBA=12,
        ROOT_REQ=13, ROOT_WAIT=14, ROOT_NEXT=15, FAT_REQ=16, FAT_WAIT=17,
        FAT_PARSE=18, FINISH=19, BOOT_CAPACITY=20, BOOT_LIMIT=21, BOOT_VALIDATE=22,
        FAIL=23, BOOT_ACCEPT=24, FAT_ACCEPT=25, BOOT_DATA=26;
    localparam integer CHAIN_W=(MAX_DIR_CLUSTERS<=1)?1:$clog2(MAX_DIR_CLUSTERS+1);
    reg [4:0] state;
    // Consume the mount fields directly. Full-sector random access duplicated
    // the retained geometry and created a large byte-write decoder. Fields
    // that are only compared with constants need match bits, not whole bytes.
    reg boot_jump, partition_type_ok, fat_count_one, fat_count_two;
    reg [1:0] signature_ok, sector_size_ok, root_entries_zero, fat16_zero;
    reg [7:0] boot_spc;
    reg [31:0] boot_partition_lba;
    // Valid bits own lifetime. Do not reset RAM contents: stale records are
    // hidden by image_valid_mask, allowing the metadata to use embedded RAM.
    localparam integer RECORDS = MAX_IMAGES + (ENABLE_PLAYLIST ? MAX_SONGS : 0) + (ENABLE_CUES ? MAX_SONGS : 0);
    localparam integer RECORD_WIDTH = RECORDS <= 1 ? 1 : $clog2(RECORDS);
    (* ram_style = "bram" *) reg [63:0] image_records[0:RECORDS-1];
    reg [63:0] query_record;
    reg [IMAGE_ID_WIDTH-1:0] query_id_q;
    reg [SONG_ID_WIDTH-1:0] query_song_id_q, matched_song_id;
    reg query_song_q, match_song, query_cue_q, match_cue;
    reg [RECORD_WIDTH-1:0] write_record_id;
    reg [31:0] root_cluster, total_sectors, fat_sectors;
    reg [15:0] reserved_sectors;
    reg [32:0] fat_span, layout_sectors;
    reg [31:0] data_sectors, cluster_capacity, cluster_limit;
    reg [2:0] spc_shift;
    reg [31:0] current_cluster, cluster_offset, cluster_sector_offset;
    reg [7:0] sector_in_cluster;
    reg [CHAIN_W-1:0] visited_clusters;
    reg directory_end;
    // Compare each 8.3 byte as it arrives instead of storing the whole name
    // and building three wide parallel comparators at the attribute byte.
    reg image_name_match, song_name_match, pcm_extension_match, cue_extension_match;
    reg match_image, match_audio;
    reg [IMAGE_ID_WIDTH-1:0] matched_id;
    reg [15:0] entry_cluster_high, entry_cluster_low;
    reg [23:0] entry_size_low;
    reg [31:0] next_cluster;
    reg [3:0] fat_bytes_seen;
    reg bpb_invalid, entry_cluster_ok;
    reg [7:0] file_number;

    assign scan_busy=(state!=IDLE);
    // Partially discovered records are not published during a scan.
    wire song_query=ENABLE_PLAYLIST && query_song_enable;
    wire cue_query=ENABLE_CUES && query_cue_enable;
    wire [RECORD_WIDTH-1:0] read_record_id=cue_query ? MAX_IMAGES+MAX_SONGS+query_song_id :
        song_query ? MAX_IMAGES+query_song_id : query_image_id;
    assign query_valid=rst_n && !scan_busy && !song_query && !query_song_q && !cue_query && !query_cue_q && query_image_id==query_id_q &&
        query_image_id<MAX_IMAGES && image_valid_mask[query_image_id];
    assign query_cluster=(RAW_QUERY_DATA || query_valid)?query_record[63:32]:32'd0;
    assign query_size=(RAW_QUERY_DATA || query_valid)?query_record[31:0]:32'd0;
    assign query_song_valid=rst_n && !scan_busy && song_query && query_song_q && !cue_query && !query_cue_q &&
        query_song_id==query_song_id_q && query_song_id<MAX_SONGS && song_valid_mask[query_song_id];
    assign query_song_cluster=(RAW_QUERY_DATA || query_song_valid)?query_record[63:32]:32'd0;
    assign query_song_size=(RAW_QUERY_DATA || query_song_valid)?query_record[31:0]:32'd0;
    assign query_cue_valid=rst_n && !scan_busy && cue_query && query_cue_q &&
        query_song_id==query_song_id_q && query_song_id<MAX_SONGS && cue_valid_mask[query_song_id];
    assign query_cue_cluster=(RAW_QUERY_DATA || query_cue_valid)?query_record[63:32]:32'd0;
    assign query_cue_size=(RAW_QUERY_DATA || query_cue_valid)?query_record[31:0]:32'd0;
    wire first_digit_ok=sector_byte>="1" && sector_byte<="9";
    wire second_digit_ok=sector_byte==" " || (sector_byte>="0" && sector_byte<="9");
    wire [7:0] digit_value={4'd0,sector_byte[3:0]};
    wire image_name_ok=image_name_match && file_number<=MAX_IMAGES;
    // Accept regular files with any combination of R/H/S/A attribute bits.
    wire regular_file=(sector_byte & 8'hd8)==0;
    wire [31:0] entry_cluster={entry_cluster_high,entry_cluster_low};
    wire [31:0] entry_size={sector_byte,entry_size_low};
    wire entry_usable=entry_cluster_ok && entry_size!=0;
    reg [7:0] song_number_q;
    wire numbered_song_name=song_name_match && song_number_q<=MAX_SONGS;
    wire song_name_ok=numbered_song_name && pcm_extension_match;
    wire cue_name_ok=numbered_song_name && cue_extension_match;
    wire song_usable=entry_usable && !entry_size[0];
    wire [8:0] fat_entry_offset={current_cluster[6:0],2'b00};
    wire [32:0] partition_end={1'b0,partition_lba}+{1'b0,total_sectors};

    function valid_spc;
        input [7:0] value;
        begin valid_spc=value!=0 && ((value&(value-1'b1))==0); end
    endfunction
    function [2:0] shift_for_spc;
        input [7:0] value;
        begin
            case(value)
                2:shift_for_spc=1;4:shift_for_spc=2;8:shift_for_spc=3;
                16:shift_for_spc=4;32:shift_for_spc=5;64:shift_for_spc=6;
                128:shift_for_spc=7;default:shift_for_spc=0;
            endcase
        end
    endfunction
    task fail_scan;
        input [7:0] error;
        begin
            // Publish failure in a dedicated state instead of feeding every
            // range check into the high-fanout catalogue clear enables.
            fs_error<=error;state<=FAIL;
        end
    endtask

    always @(posedge clk) begin
        query_record<=image_records[read_record_id];
        query_id_q<=query_image_id;
        query_song_q<=song_query;
        query_cue_q<=cue_query;
        query_song_id_q<=query_song_id;
        if(rst_n && state==ROOT_WAIT && sector_valid && !directory_end &&
           sector_byte_index[4:0]==31 && entry_usable &&
           ((match_image && !image_valid_mask[matched_id]) ||
            (ENABLE_PLAYLIST && match_song && !entry_size[0] && !song_valid_mask[matched_song_id]) ||
            (ENABLE_CUES && match_cue && !cue_valid_mask[matched_song_id])))
            image_records[write_record_id]<={entry_cluster,entry_size};
    end
    task finish_directory;
        begin fs_error<=(|image_valid_mask)?ERR_NONE:ERR_NO_IMAGE;state<=FINISH;end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;scan_done<=0;fs_error<=0;partition_lba<=0;
            sectors_per_cluster<=0;fat_start_lba<=0;data_start_lba<=0;max_cluster<=0;
            image_valid_mask<=0;image_count<=0;audio_valid<=0;audio_cluster<=0;audio_size<=0;
            song_valid_mask<=0;match_song<=0;matched_song_id<=0;song_number_q<=0;
            cue_valid_mask<=0;match_cue<=0;write_record_id<=0;
            sector_req<=0;sector_lba<=0;root_cluster<=0;total_sectors<=0;fat_sectors<=0;
            reserved_sectors<=0;fat_span<=0;layout_sectors<=0;data_sectors<=0;spc_shift<=0;
            cluster_capacity<=0;cluster_limit<=0;
            current_cluster<=0;cluster_offset<=0;cluster_sector_offset<=0;sector_in_cluster<=0;
            visited_clusters<=0;directory_end<=0;match_image<=0;match_audio<=0;
            image_name_match<=0;song_name_match<=0;pcm_extension_match<=0;cue_extension_match<=0;
            matched_id<=0;entry_cluster_high<=0;entry_cluster_low<=0;entry_size_low<=0;
            next_cluster<=0;fat_bytes_seen<=0;
            bpb_invalid<=0;entry_cluster_ok<=0;file_number<=0;
        end else begin
            scan_done<=0;sector_req<=0;
            // All mount fields arrive before sector_done; READ0_PARSE and
            // BOOT_PARSE remain separate cycles so byte 511 is committed.
            if(sector_valid && (state==READ0_WAIT || state==BOOT_WAIT)) begin
                case(sector_byte_index)
                    0:boot_jump<=sector_byte==8'heb || sector_byte==8'he9;
                    11:sector_size_ok[0]<=sector_byte==0;
                    12:sector_size_ok[1]<=sector_byte==2;
                    13:boot_spc<=sector_byte;
                    14:reserved_sectors[7:0]<=sector_byte;
                    15:reserved_sectors[15:8]<=sector_byte;
                    16:begin fat_count_one<=sector_byte==1;fat_count_two<=sector_byte==2;end
                    17:root_entries_zero[0]<=sector_byte==0;
                    18:root_entries_zero[1]<=sector_byte==0;
                    22:fat16_zero[0]<=sector_byte==0;
                    23:fat16_zero[1]<=sector_byte==0;
                    32,33,34,35:total_sectors<={sector_byte,total_sectors[31:8]};
                    36,37,38,39:fat_sectors<={sector_byte,fat_sectors[31:8]};
                    44,45,46,47:root_cluster<={sector_byte,root_cluster[31:8]};
                    450:partition_type_ok<=sector_byte==8'h0b || sector_byte==8'h0c;
                    454,455,456,457:boot_partition_lba<={sector_byte,boot_partition_lba[31:8]};
                    510:signature_ok[0]<=sector_byte==8'h55;
                    511:signature_ok[1]<=sector_byte==8'haa;
                    default:begin end
                endcase
            end

            if(state==ROOT_WAIT && sector_valid && !directory_end) begin
                case(sector_byte_index[4:0])
                    0:begin
                        image_name_match<=sector_byte=="P";
                        song_name_match<=sector_byte=="S";
                        pcm_extension_match<=0;cue_extension_match<=0;
                        match_image<=0;match_audio<=0;match_song<=0;match_cue<=0;
                        if(sector_byte==0) directory_end<=1;
                    end
                    1:begin image_name_match<=image_name_match && sector_byte=="I";
                        song_name_match<=song_name_match && sector_byte=="O";end
                    2:begin image_name_match<=image_name_match && sector_byte=="C";
                        song_name_match<=song_name_match && sector_byte=="N";end
                    3:begin
                        image_name_match<=image_name_match && first_digit_ok;
                        file_number<=digit_value;
                        song_name_match<=song_name_match && sector_byte=="G";
                    end
                    4:begin
                        image_name_match<=image_name_match && second_digit_ok;
                        // Decimal tens use shifts/adds, not a DSP multiplier.
                        if(sector_byte!=" ") file_number<=({4'd0,file_number[3:0]}<<3)+({4'd0,file_number[3:0]}<<1)+digit_value;
                        song_name_match<=song_name_match && first_digit_ok;
                        song_number_q<=digit_value;
                    end
                    5:begin
                        image_name_match<=image_name_match && sector_byte==" ";
                        song_name_match<=song_name_match && second_digit_ok;
                        if(sector_byte!=" ") song_number_q<=({4'd0,song_number_q[3:0]}<<3)+({4'd0,song_number_q[3:0]}<<1)+digit_value;
                    end
                    6,7:begin image_name_match<=image_name_match && sector_byte==" ";
                        song_name_match<=song_name_match && sector_byte==" ";end
                    8:begin
                        image_name_match<=image_name_match && sector_byte=="B";
                        pcm_extension_match<=sector_byte=="P";
                        cue_extension_match<=sector_byte=="C";
                    end
                    9:begin
                        image_name_match<=image_name_match && sector_byte=="M";
                        pcm_extension_match<=pcm_extension_match && sector_byte=="C";
                        cue_extension_match<=cue_extension_match && sector_byte=="U";
                    end
                    10:begin
                        image_name_match<=image_name_match && sector_byte=="P";
                        pcm_extension_match<=pcm_extension_match && sector_byte=="M";
                        cue_extension_match<=cue_extension_match && sector_byte=="E";
                    end
                    11:begin
                        match_image<=regular_file && image_name_ok;
                        matched_id<=file_number-1'b1;
                        match_audio<=regular_file && song_name_match && pcm_extension_match && song_number_q==1;
                        match_song<=regular_file && song_name_ok;
                        match_cue<=regular_file && cue_name_ok;
                        matched_song_id<=song_number_q-1'b1;
                    end
                    // Names/IDs are known at offset 11. Prepare one physical
                    // RAM address at 12, well before the offset-31 commit,
                    // even when directory bytes arrive on consecutive clocks.
                    12:write_record_id<=match_cue ? MAX_IMAGES+MAX_SONGS+matched_song_id :
                        match_song ? MAX_IMAGES+matched_song_id : matched_id;
                    20:entry_cluster_high[7:0]<=sector_byte;
                    21:entry_cluster_high[15:8]<=sector_byte;
                    26:entry_cluster_low[7:0]<=sector_byte;
                    27:entry_cluster_low[15:8]<=sector_byte;
                    // The cluster is complete at offset 27. Check it before
                    // offset 31 commits the entry to the catalogue table.
                    28:begin
                        entry_size_low[7:0]<=sector_byte;
                        entry_cluster_ok<=entry_cluster>=2 && entry_cluster<=max_cluster;
                    end
                    29:entry_size_low[15:8]<=sector_byte;
                    30:entry_size_low[23:16]<=sector_byte;
                    31:begin
                        // First usable duplicate wins; an invalid entry cannot
                        // prevent a later valid entry of the same name.
                        if(match_image && entry_usable && !image_valid_mask[matched_id]) begin
                            image_valid_mask[matched_id]<=1;
                            image_count<=image_count+1'b1;
                        end
                        if(match_audio && entry_usable && !audio_valid) begin
                            audio_cluster<=entry_cluster;audio_size<=entry_size;audio_valid<=1;
                        end
                        if(ENABLE_PLAYLIST && match_song && song_usable && !song_valid_mask[matched_song_id])
                            song_valid_mask[matched_song_id]<=1;
                        if(ENABLE_CUES && match_cue && entry_usable && !cue_valid_mask[matched_song_id])
                            cue_valid_mask[matched_song_id]<=1;
                    end
                    default:begin end
                endcase
            end

            if(state==FAT_WAIT && sector_valid) begin
                if(sector_byte_index==fat_entry_offset) begin next_cluster[7:0]<=sector_byte;fat_bytes_seen[0]<=1;end
                if(sector_byte_index==fat_entry_offset+9'd1) begin next_cluster[15:8]<=sector_byte;fat_bytes_seen[1]<=1;end
                if(sector_byte_index==fat_entry_offset+9'd2) begin next_cluster[23:16]<=sector_byte;fat_bytes_seen[2]<=1;end
                if(sector_byte_index==fat_entry_offset+9'd3) begin next_cluster[31:24]<={4'd0,sector_byte[3:0]};fat_bytes_seen[3]<=1;end
            end

            case(state)
                IDLE:if(scan_start) begin
                    fs_error<=0;partition_lba<=0;image_valid_mask<=0;image_count<=0;
                    audio_valid<=0;audio_cluster<=0;audio_size<=0;directory_end<=0;
                    song_valid_mask<=0;match_song<=0;cue_valid_mask<=0;match_cue<=0;
                    match_image<=0;match_audio<=0;visited_clusters<=0;
                    state<=READ0_REQ;
                end
                READ0_REQ:if(!sector_busy) begin sector_lba<=0;sector_req<=1;state<=READ0_WAIT;end
                READ0_WAIT:if(sector_done) begin
                    if(sector_error!=0) fail_scan(ERR_SECTOR); else state<=READ0_PARSE;
                end
                READ0_PARSE:begin
                    if(!(&signature_ok)) fail_scan(ERR_BOOT_SIGNATURE);
                    else if(boot_jump) state<=BOOT_PARSE;
                    else if(partition_type_ok) begin
                        partition_lba<=boot_partition_lba;sector_lba<=boot_partition_lba;state<=BOOT_REQ;
                    end else fail_scan(ERR_UNSUPPORTED_BPB);
                end
                BOOT_REQ:if(!sector_busy) begin sector_req<=1;state<=BOOT_WAIT;end
                BOOT_WAIT:if(sector_done) begin
                    if(sector_error!=0) fail_scan(ERR_SECTOR); else state<=BOOT_PARSE;
                end
                BOOT_PARSE:begin
                    if(!(&signature_ok)) fail_scan(ERR_BOOT_SIGNATURE);
                    else begin
                        bpb_invalid<=!(&sector_size_ok) || !valid_spc(boot_spc) || reserved_sectors==0 ||
                            !(fat_count_one || fat_count_two) || !(&root_entries_zero) || !(&fat16_zero) ||
                            total_sectors==0 || fat_sectors==0 || root_cluster<2;
                        state<=BOOT_VALIDATE;
                    end
                end
                BOOT_VALIDATE:begin
                    if(bpb_invalid) fail_scan(ERR_UNSUPPORTED_BPB);
                    else begin
                        sectors_per_cluster<=boot_spc;
                        spc_shift<=shift_for_spc(boot_spc);
                        fat_span<=fat_count_two?{fat_sectors,1'b0}:{1'b0,fat_sectors};
                        fat_start_lba<=partition_lba+reserved_sectors;state<=BOOT_LAYOUT;
                    end
                end
                BOOT_LAYOUT:begin layout_sectors<=fat_span+reserved_sectors;state<=BOOT_BOUNDS;end
                BOOT_BOUNDS:begin
                    if(layout_sectors>={1'b0,total_sectors} || partition_end>33'h100000000)
                        fail_scan(ERR_UNSUPPORTED_BPB);
                    else state<=BOOT_DATA;
                end
                BOOT_DATA:begin
                    data_start_lba<=partition_lba+layout_sectors[31:0];
                    data_sectors<=total_sectors-layout_sectors[31:0];state<=BOOT_CAPACITY;
                end
                // Mount-time arithmetic has no byte-stream deadline. Separate
                // the variable shift, increment and range checks at 100 MHz.
                BOOT_CAPACITY:begin cluster_capacity<=data_sectors>>spc_shift;state<=BOOT_LIMIT;end
                BOOT_LIMIT:begin cluster_limit<=cluster_capacity+1'b1;state<=BOOT_ROOT;end
                BOOT_ROOT:begin
                    if(cluster_capacity==0 || cluster_capacity>=32'h0fffffef ||
                        root_cluster>cluster_limit || (cluster_limit>>7)>=fat_sectors)
                        fail_scan(ERR_UNSUPPORTED_BPB);
                    else state<=BOOT_ACCEPT;
                end
                BOOT_ACCEPT:begin
                    max_cluster<=cluster_limit;current_cluster<=root_cluster;
                    visited_clusters<=1;state<=CLUSTER_OFFSET;
                end
                CLUSTER_OFFSET:begin cluster_offset<=current_cluster-2;state<=CLUSTER_SCALE;end
                CLUSTER_SCALE:begin cluster_sector_offset<=cluster_offset<<spc_shift;state<=CLUSTER_LBA;end
                CLUSTER_LBA:begin sector_lba<=data_start_lba+cluster_sector_offset;sector_in_cluster<=0;state<=ROOT_REQ;end
                ROOT_REQ:if(!sector_busy) begin
                    match_image<=0;match_audio<=0;sector_req<=1;state<=ROOT_WAIT;
                end
                ROOT_WAIT:if(sector_done) begin
                    if(sector_error!=0) fail_scan(ERR_SECTOR);else state<=ROOT_NEXT;
                end
                ROOT_NEXT:begin
                    // ROOT_NEXT is a separate cycle: a directory entry in byte
                    // 511 may commit on the very same clock as sector_done.
                    if(directory_end || (!ENABLE_CUES && (&image_valid_mask) &&
                        (ENABLE_PLAYLIST ? (&song_valid_mask) : audio_valid))) finish_directory;
                    else if(sector_in_cluster<sectors_per_cluster-1'b1) begin
                        sector_in_cluster<=sector_in_cluster+1'b1;sector_lba<=sector_lba+1'b1;state<=ROOT_REQ;
                    end else begin
                        sector_lba<=fat_start_lba+(current_cluster>>7);state<=FAT_REQ;
                    end
                end
                FAT_REQ:if(!sector_busy) begin next_cluster<=0;fat_bytes_seen<=0;sector_req<=1;state<=FAT_WAIT;end
                FAT_WAIT:if(sector_done) begin
                    if(sector_error!=0) fail_scan(ERR_SECTOR);else state<=FAT_PARSE;
                end
                FAT_PARSE:begin
                    if(fat_bytes_seen!=4'hf) fail_scan(ERR_SECTOR);
                    else if(next_cluster>=32'h0ffffff8) finish_directory;
                    else if(next_cluster<2 || next_cluster>max_cluster || next_cluster>=32'h0ffffff0)
                        fail_scan(ERR_CLUSTER_RANGE);
                    else if(next_cluster==current_cluster) fail_scan(ERR_FAT_LOOP);
                    else if(visited_clusters>=MAX_DIR_CLUSTERS) fail_scan(ERR_DIR_LIMIT);
                    else state<=FAT_ACCEPT;
                end
                FAT_ACCEPT:begin
                    current_cluster<=next_cluster;visited_clusters<=visited_clusters+1'b1;state<=CLUSTER_OFFSET;
                end
                FAIL:begin
                    image_valid_mask<=0;image_count<=0;
                    audio_valid<=0;audio_cluster<=0;audio_size<=0;state<=FINISH;
                    song_valid_mask<=0;
                    cue_valid_mask<=0;
                end
                FINISH:begin scan_done<=1;state<=IDLE;end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
