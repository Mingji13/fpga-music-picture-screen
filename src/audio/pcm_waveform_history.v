`timescale 1ns/1ps

// 音乐轮廓的数据采集部分：每 120 个有效 PCM 样本生成一根柱的幅度。
// 48 kHz 时，每根柱代表 2.5 ms，128 根柱保留最近约 0.32 s。
// 所有端口使用同一个时钟域；pcm_valid 是取样脉冲，没有反压输出。
module pcm_waveform_history #(
    parameter integer SAMPLES_PER_BAR = 120 // Must be >= 1.
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        pcm_valid,
    input  wire signed [15:0] pcm_sample,

    // bar_valid 拉高一拍时，bar_peak 是刚完成的一组的最大绝对值。
    // 无符号范围为 0..32768，能正确表示 |-32768|。
    output reg         bar_valid,
    output reg  [15:0] bar_peak,
    output reg  [7:0]  history_count,

    // 128 个 16 位无符号幅度，按画面从左到右的顺序排列：
    // [2047:2032] 最旧/最左；[15:0] 最新/最右；未填满的左侧为 0。
    // 这是并行寄存器历史，后续绘图模块需在帧边界锁存这份数据。
    // 采集历史会在每组完成时更新，本模块本身不负责帧同步或绘图。
    output reg [2047:0] history_peaks
);
    localparam integer COUNT_BITS = (SAMPLES_PER_BAR < 2) ? 1 :
                                    $clog2(SAMPLES_PER_BAR);
    localparam [COUNT_BITS-1:0] LAST_SAMPLE = SAMPLES_PER_BAR - 1;

    reg [COUNT_BITS-1:0] sample_count;
    reg [15:0] window_peak;

    // 保留完整的 16 位精度；高度缩放留给绘图模块。
    // 负数取补码得到幅值，结果按 unsigned 解释，所以 16'h8000 是 32768。
    wire [15:0] sample_magnitude = pcm_sample[15] ?
                                   (~pcm_sample + 16'd1) : pcm_sample;
    wire [15:0] peak_with_current = (sample_magnitude > window_peak) ?
                                    sample_magnitude : window_peak;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_count  <= 0;
            window_peak   <= 16'd0;
            bar_valid     <= 1'b0;
            bar_peak      <= 16'd0;
            history_count <= 8'd0;
            history_peaks <= 2048'd0;
        end else begin
            bar_valid <= 1'b0;
            if (pcm_valid) begin
                if (sample_count == LAST_SAMPLE) begin
                    // 第 120 个样本也参与比较，不能只提交之前的 window_peak。
                    bar_peak      <= peak_with_current;
                    bar_valid     <= 1'b1;
                    history_peaks <= {history_peaks[2031:0], peak_with_current};
                    if (history_count < 8'd128)
                        history_count <= history_count + 1'b1;

                    // 下一组重新统计；全零样本同样会生成一根幅度为零的新柱。
                    sample_count <= 0;
                    window_peak  <= 16'd0;
                end else begin
                    sample_count <= sample_count + 1'b1;
                    window_peak  <= peak_with_current;
                end
            end
        end
    end
endmodule
