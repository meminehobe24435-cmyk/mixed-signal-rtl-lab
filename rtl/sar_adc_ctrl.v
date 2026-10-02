//======================================================================
// sar_adc_ctrl.v —— 12 位逐次逼近型 ADC 的数字控制器
//
// 数模混合芯片里，ADC 的数字部分负责：采样开关时序、逐次逼近逻辑
// （SAR register）、比较器结果的采样与判决、转换完成握手，以及增益/失调的
// 数字校准。
//
// 本模块把「比较器」与「CDAC」当作外部模拟块：
//   - 数字侧输出 dac_code 给 CDAC（当前试探码）
//   - 模拟侧回送 comp_in（1 表示 DAC 输出 <= 输入，即试探码偏小或正好）
// 这样数字控制逻辑可以完全用数字仿真验证，把模拟非理想项隔离在外。
//
// 时序：IDLE --start--> SAMPLE(CLK_DIV 拍) --> CONVERT(WIDTH*2 拍) --> DONE
// 每个 bit 两拍：第一拍更新试探码给 DAC 建立，第二拍采样比较器结果。
// 转换结果 = 满足 CDAC(code) <= Vin 的最大码（向下取整）。
// 校准：dout = sat((raw * cfg_gain) >>> GAIN_SHIFT + cfg_offset)，带上下饱和。
//======================================================================
`timescale 1ns/1ps
`default_nettype none

module sar_adc_ctrl #(
    parameter WIDTH      = 12,    // 分辨率
    parameter CLK_DIV    = 4,     // 采样窗口长度（时钟数），需 >= 2
    parameter GAIN_SHIFT = 7      // 增益定点小数位：gain=128 表示 1.0
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    start,      // 启动一次转换（单拍脉冲）
    input  wire                    comp_in,    // 比较器输出
    input  wire [WIDTH-1:0]        cfg_gain,   // 数字增益（Q1.GAIN_SHIFT）
    input  wire signed [WIDTH-1:0] cfg_offset, // 数字失调（有符号）
    output reg  [WIDTH-1:0]        dac_code,   // 送给 CDAC 的试探码
    output reg  [WIDTH-1:0]        dout,       // 校准后的转换结果
    output reg  [WIDTH-1:0]        dout_raw,   // 未校准的原始码
    output reg                     dout_valid, // 转换完成脉冲
    output wire                    sample_en,  // 采样开关使能（跟随状态，组合输出）
    output reg  [2:0]              state_dbg   // 0=IDLE 1=SAMPLE 2=CONVERT 3=DONE
);

    localparam S_IDLE    = 3'd0;
    localparam S_SAMPLE  = 3'd1;
    localparam S_CONVERT = 3'd2;
    localparam S_DONE    = 3'd3;

    // 采样开关跟随状态**组合输出**：做成寄存器会比状态晚一拍，
    // 实测采样窗口就少一拍（CLK_DIV=4 只采到 3 拍）—— 这是被 testbench 抓出来的。
    assign sample_en = (state == S_SAMPLE);

    reg [2:0]       state;
    reg [15:0]      div_cnt;    // 采样计数
    reg [7:0]       bit_idx;    // 当前判决的位（WIDTH-1 → 0）
    reg             phase;      // 0=驱动 DAC 建立，1=采样比较器
    reg [WIDTH-1:0] result;     // 已确定的位

    wire [WIDTH-1:0] bit_mask = {{(WIDTH-1){1'b0}}, 1'b1} << bit_idx;
    wire [WIDTH-1:0] trial    = result | bit_mask;
    // 当前位的判决结果：比较器说「试探码仍不大于输入」就保留这一位
    wire [WIDTH-1:0] decided  = comp_in ? trial : result;

    // ---------------- 数字校准（组合）：乘增益 → 加失调 → 上下饱和 --------
    // 中间量取 2*WIDTH+1 位，避免「大增益把小码放大后回绕」这种典型定点事故；
    // 最后再按输出位宽做饱和，而不是直接截断。
    localparam signed [2*WIDTH:0] CAL_MAX = (1 << WIDTH) - 1;
    wire signed [2*WIDTH:0] cal_mul = ($signed({1'b0, decided}) *
                                       $signed({1'b0, cfg_gain})) >>> GAIN_SHIFT;
    wire signed [2*WIDTH:0] cal_sum = cal_mul + $signed(cfg_offset);
    reg  [WIDTH-1:0] cal_sat;
    always @(*) begin
        if (cal_sum < 0)
            cal_sat = {WIDTH{1'b0}};          // 下饱和
        else if (cal_sum > CAL_MAX)
            cal_sat = {WIDTH{1'b1}};          // 上饱和
        else
            cal_sat = cal_sum[WIDTH-1:0];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            div_cnt    <= 16'd0;
            bit_idx    <= 8'd0;
            phase      <= 1'b0;
            result     <= {WIDTH{1'b0}};
            dac_code   <= {WIDTH{1'b0}};
            dout       <= {WIDTH{1'b0}};
            dout_raw   <= {WIDTH{1'b0}};
            dout_valid <= 1'b0;
            state_dbg  <= 3'd0;
        end else begin
            dout_valid <= 1'b0;

            case (state)
                // ---------------- 空闲：等 start ----------------
                S_IDLE: begin
                    state_dbg <= 3'd0;
                    if (start) begin
                        div_cnt <= 16'd0;
                        state   <= S_SAMPLE;
                    end
                end

                // ---------------- 采样：拉高采样开关 CLK_DIV 拍 ----------------
                S_SAMPLE: begin
                    state_dbg <= 3'd1;
                    if (div_cnt == CLK_DIV - 1) begin
                        result    <= {WIDTH{1'b0}};
                        bit_idx   <= WIDTH - 1;
                        phase     <= 1'b0;
                        dac_code  <= {{(WIDTH-1){1'b0}}, 1'b1} << (WIDTH-1);
                        state     <= S_CONVERT;
                    end else begin
                        div_cnt <= div_cnt + 16'd1;
                    end
                end

                // ---------------- 逐次逼近 ----------------
                S_CONVERT: begin
                    state_dbg <= 3'd2;
                    if (phase == 1'b0) begin
                        // 第一拍：试探码送 CDAC，等建立
                        dac_code <= trial;
                        phase    <= 1'b1;
                    end else begin
                        // 第二拍：采样比较器结果
                        phase <= 1'b0;
                        if (bit_idx == 0) begin
                            dout_raw   <= decided;
                            dout       <= cal_sat;     // 校准用的是本拍的 decided
                            dout_valid <= 1'b1;
                            state      <= S_DONE;
                        end else begin
                            result   <= decided;
                            bit_idx  <= bit_idx - 8'd1;
                            dac_code <= decided | ({{(WIDTH-1){1'b0}}, 1'b1} << (bit_idx - 8'd1));
                        end
                    end
                end

                // ---------------- 完成：回 IDLE ----------------
                S_DONE: begin
                    state_dbg <= 3'd3;
                    state     <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
