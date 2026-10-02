//======================================================================
// rdc_tracking.v —— 旋变数字转换器（RDC）的 Type-II 跟踪环路
//
// 旋变（resolver）输出两路正交模拟信号 sin(θ)、cos(θ)，数字侧要解出转子
// 角度 θ，并顺带给出转速。工程上不直接做 atan2，而是用一个**跟踪环路**：
// 让内部估计角 φ 去追 θ。
//
//   误差检测： err = sin(θ)cos(φ) − cos(θ)sin(φ) = sin(θ−φ) ≈ θ−φ
//   环路滤波： PI（比例 + 积分；积分器同时充当速度输出）
//   角度积分： φ ← φ + prop + integ
//
// 三个"不试就不知道"的设计决定（都是被定点参考模型逼出来的，见 README）：
//   1. **象限预置**：误差信号是 sin(θ−φ)，初始偏差正好 180° 时恒为 0，
//      环路永远锁不上 → 必须先按 sin/cos 符号把 φ 预置到 ±45° 以内（acquire）；
//   2. **正弦表线性插值**：纯整数查表时，估计用的 sin/cos 每 64 个角度 LSB 才
//      更新一次，误差被量化成 64 LSB 台阶（静态误差 ≥32 LSB 且单向漂移）；
//      插值后误差恢复成 ≈Δθ，静态误差降到 ≤1 LSB；
//   3. **抗积分饱和**：大偏差期间冻结积分器，否则积分器灌满后由于误差很小
//      而长期退不下来。
//
// 定点约定：sin/cos 输入 Q1.15；角度 16 位（65536 = 360°）；
//          err 为角度单位；积分器为 Q(KI_SHIFT) 定点。
//======================================================================
`timescale 1ns/1ps
`default_nettype none

module rdc_tracking #(
    parameter KP_SHIFT   = 6,      // 比例增益右移：prop = round(err*32 / 2^6)
    parameter KP_GAIN    = 32,     // 比例增益：32/64 = 0.5（实测 12~14 拍收敛）
    parameter KI_SHIFT   = 16,     // 积分器定点小数位
    parameter INT_FREEZE = 1024,   // 抗积分饱和门限（角度 LSB）
    parameter LOCK_TOL   = 4,      // 锁定门限（角度 LSB）
    parameter LOCK_N     = 16      // 连续多少拍在门限内算锁定
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                en,
    input  wire                acquire,    // 单拍脉冲：做一次象限预置并清积分器
    input  wire signed [15:0]  sin_in,     // sin(θ)，Q1.15
    input  wire signed [15:0]  cos_in,     // cos(θ)，Q1.15
    output reg  [15:0]         angle_out,  // 解出的角度（65536 = 360°）
    output reg  signed [31:0]  speed_out,  // 速度输出（积分器，正比于角速度）
    output reg  signed [31:0]  err_out,    // 观测：当前角度误差（角度 LSB）
    output reg                 locked      // 观测：连续 LOCK_N 拍误差在门限内
);

    `include "sincos_rom.vh"

    localparam signed [31:0] ERR_SCALE = 32'sd10430;   // 65536/(2π)：弧度 → 角度 LSB
    localparam signed [31:0] FREEZE    = INT_FREEZE;

    reg signed [31:0] integ_q;     // Q(KI_SHIFT) 积分器
    reg [7:0]         lock_cnt;

    // ---------------- 估计角的正弦/余弦（象限折叠 + 线性插值）----------------
    function signed [15:0] sin_at;    // p6: 0..16384 对应 0..90°，低 6 位插值
        input [14:0] p6;
        reg [8:0]  idx;
        reg [5:0]  frac;
        reg signed [16:0] v0, v1;
        reg signed [31:0] prod, acc;   // 32 位上下文，保证插值乘法不回绕
        begin
            idx  = p6[14:6];
            frac = p6[5:0];
            if (idx >= SIN_LUT_N) begin
                sin_at = SIN_LUT[SIN_LUT_N];
            end else begin
                v0   = SIN_LUT[idx];
                v1   = SIN_LUT[idx + 1];
                prod = (v1 - v0) * frac;      // 最大约 201×63，32 位足够
                acc  = v0 + (prod >>> 6);
                sin_at = acc[15:0];
            end
        end
    endfunction

    reg  [1:0]        quad;
    reg  [13:0]       a_q;
    reg signed [15:0] s_q, c_q, s_est, c_est;
    always @(*) begin
        quad = angle_out[15:14];
        a_q  = angle_out[13:0];
        s_q  = sin_at({1'b0, a_q});
        c_q  = sin_at(15'd16384 - {1'b0, a_q});
        case (quad)
            2'd0: begin s_est =  s_q; c_est =  c_q; end
            2'd1: begin s_est =  c_q; c_est = -s_q; end
            2'd2: begin s_est = -s_q; c_est = -c_q; end
            default: begin s_est = -c_q; c_est = s_q; end
        endcase
    end

    // ---------------- 误差检测与环路滤波 ----------------
    // ⚠️ 位宽陷阱：Verilog 里乘法结果的位宽 = max(操作数位宽)，**不是两者之和**。
    //    `diff * ERR_SCALE` 如果直接写进 32 位上下文会**静默回绕**
    //    （diff 可达 ±2^30，乘 10430 需要 44 位），表现为环路增益几乎为 0、
    //    角度一拍只挪 1 LSB —— 这个 bug 就是被逐位比对的 testbench 抓出来的。
    //    必须显式给足 64 位中间量。
    wire signed [31:0] diff = sin_in * c_est - cos_in * s_est;   // Q2.30，±2^30
    wire signed [63:0] err_wide = diff * ERR_SCALE;              // 64 位上下文
    wire signed [31:0] err  = err_wide[61:30];                   // >>> 30 → 角度 LSB

    // 四舍五入右移：纯截断会留下几 LSB 的静态误差
    wire signed [31:0] prop     = (err * KP_GAIN + (32'sd1 <<< (KP_SHIFT - 1))) >>> KP_SHIFT;
    wire signed [31:0] int_term = (integ_q + (32'sd1 <<< (KI_SHIFT - 1))) >>> KI_SHIFT;

    wire signed [31:0] angle_s    = {16'b0, angle_out};          // 角度恒为非负，零扩展
    wire signed [31:0] angle_next = angle_s + prop + int_term;

    wire do_integ = (err < FREEZE) && (err > -FREEZE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            angle_out <= 16'd0;
            integ_q   <= 32'sd0;
            speed_out <= 32'sd0;
            err_out   <= 32'sd0;
            locked    <= 1'b0;
            lock_cnt  <= 8'd0;
        end else if (en) begin
            if (acquire) begin
                // ---- 象限预置：取 sin/cos 的符号，把初值放到 ±45° 以内 ----
                if (!sin_in[15] && !cos_in[15])      angle_out <= 16'h0000;
                else if (!sin_in[15] && cos_in[15])  angle_out <= 16'h4000;
                else if (sin_in[15]  && cos_in[15])  angle_out <= 16'h8000;
                else                                 angle_out <= 16'hC000;
                integ_q  <= 32'sd0;
                lock_cnt <= 8'd0;
                locked   <= 1'b0;
            end else begin
                angle_out <= angle_next[15:0];
                if (do_integ)
                    integ_q <= integ_q + err;

                // 锁定判定：连续 LOCK_N 拍误差都在 ±LOCK_TOL 之内
                if (err < LOCK_TOL && err > -LOCK_TOL) begin
                    if (lock_cnt < 8'd255)
                        lock_cnt <= lock_cnt + 8'd1;
                    if (lock_cnt >= LOCK_N - 1)
                        locked <= 1'b1;
                end else begin
                    lock_cnt <= 8'd0;
                    locked   <= 1'b0;
                end
            end

            speed_out <= integ_q;
            err_out   <= err;
        end
    end

endmodule

`default_nettype wire
