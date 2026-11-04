`timescale 1ns/1ps

// Configure one frame of aspect-preserving enlargement. Dimensions are runtime
// inputs; no per-pixel divider is used. A new start supersedes any old request.
// Q20 source-coordinate steps, floor rounding; Q8 weights are formed by scaler.
module video_scale_geometry (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [15:0] source_width,
    input  wire [15:0] source_height,
    output wire        busy,
    output reg         done,
    output reg         error,
    output reg  [11:0] draw_width,
    output reg  [11:0] draw_height,
    output reg  [11:0] left,
    output reg  [11:0] top,
    output reg  [31:0] step_x,
    output reg  [31:0] step_y
);
    localparam IDLE=0, DRAW_START=1, DRAW_WAIT=2,
               X_START=3, X_WAIT=4, Y_START=5, Y_WAIT=6;
    reg [2:0] state;
    reg [15:0] width_q, height_q;
    reg fit_width;
    reg div_start;
    reg [31:0] dividend, divisor;
    wire div_done;
    wire [31:0] quotient;
    wire [31:0] width_times_720 = source_width * 32'd720;
    wire [31:0] height_times_1280 = source_height * 32'd1280;
    assign busy = (state != IDLE);

    scale_divider_u32 u_divider (
        .clk(clk), .rst_n(rst_n), .start(div_start),
        .dividend(dividend), .divisor(divisor),
        .done(div_done), .quotient(quotient)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; done <= 0; error <= 0;
            width_q <= 0; height_q <= 0; fit_width <= 0;
            div_start <= 0; dividend <= 0; divisor <= 1;
            draw_width <= 0; draw_height <= 0; left <= 0; top <= 0;
            step_x <= 0; step_y <= 0;
        end else begin
            done <= 0;
            div_start <= 0;
            if (start) begin
                width_q <= source_width;
                height_q <= source_height;
                fit_width <= width_times_720 >= height_times_1280;
                error <= 0;
                if (source_width == 0 || source_height == 0 ||
                    source_width > 960 || source_height > 540) begin
                    state <= IDLE; done <= 1; error <= 1;
                    draw_width <= 0; draw_height <= 0;
                    step_x <= 0; step_y <= 0; left <= 0; top <= 0;
                end else state <= DRAW_START;
            end else case (state)
                DRAW_START: begin
                    dividend <= fit_width ? height_q * 32'd1280 : width_q * 32'd720;
                    divisor <= fit_width ? {16'd0,width_q} : {16'd0,height_q};
                    div_start <= 1;
                    state <= DRAW_WAIT;
                end
                // A superseded division may finish just as its replacement
                // launches. Ignore that old done pulse during div_start.
                DRAW_WAIT: if (div_done && !div_start) begin
                    draw_width <= fit_width ? 12'd1280 : quotient[11:0];
                    draw_height <= fit_width ? quotient[11:0] : 12'd720;
                    left <= fit_width ? 12'd0 : (12'd1280 - quotient[11:0]) >> 1;
                    top <= fit_width ? (12'd720 - quotient[11:0]) >> 1 : 12'd0;
                    state <= X_START;
                end
                X_START: begin
                    dividend <= {2'd0,width_q[9:0],20'd0};
                    divisor <= {20'd0,draw_width};
                    div_start <= 1;
                    state <= X_WAIT;
                end
                X_WAIT: if (div_done && !div_start) begin
                    step_x <= quotient;
                    state <= Y_START;
                end
                Y_START: begin
                    dividend <= {2'd0,height_q[9:0],20'd0};
                    divisor <= {20'd0,draw_height};
                    div_start <= 1;
                    state <= Y_WAIT;
                end
                Y_WAIT: if (div_done && !div_start) begin
                    step_y <= quotient;
                    done <= 1;
                    state <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule

// Unsigned restoring division. Each request completes in 32 clocks, with no
// combinational '/' operator or large combinational divider in the pixel path.
module scale_divider_u32 (
    input wire clk, rst_n, start,
    input wire [31:0] dividend, divisor,
    output reg done,
    output reg [31:0] quotient
);
    reg running;
    reg [5:0] remaining;
    reg [31:0] work, denominator;
    reg [32:0] remainder;
    wire [32:0] trial = {remainder[31:0],work[31]};
    wire subtract = trial >= {1'b0,denominator};
    wire [31:0] next_work = {work[30:0],subtract};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            running <= 0; remaining <= 0; work <= 0; denominator <= 1;
            remainder <= 0; done <= 0; quotient <= 0;
        end else begin
            done <= 0;
            if (start) begin
                running <= 1; remaining <= 6'd31; work <= dividend;
                denominator <= divisor; remainder <= 0;
            end else if (running) begin
                remainder <= subtract ? trial - {1'b0,denominator} : trial;
                work <= next_work;
                if (remaining == 0) begin
                    running <= 0; done <= 1; quotient <= next_work;
                end else remaining <= remaining - 1'b1;
            end
        end
    end
endmodule
