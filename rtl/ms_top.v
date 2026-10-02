//======================================================================
// ms_top.v —— 数模混合外设数字子系统（SPI 配置 + SAR ADC + PWM + RDC）
//
// 这是把一个"数模混合芯片的数字部分"拼起来的最小系统：
//
//   ┌────────────┐  配置/回读   ┌──────────────┐
//   │ SPI 从机 + │◄────────────►│  寄存器堆     │
//   │ 寄存器堆   │              └──────┬───────┘
//   └────────────┘                     │ cfg
//        ▲                             ▼
//        │ 状态            ┌───────────┴───────────┐
//        └─────────────────┤ SAR ADC 控制 / PWM /  │
//                          │ RDC 跟踪环路          │
//                          └───────────┬───────────┘
//                                      │ 模拟侧接口
//                     dac_code → CDAC，comp_in ← 比较器，
//                     pwm_h/pwm_l → 栅极驱动，sin/cos ← 旋变
//
// 数字侧替模拟侧守住的三件事：
//   1. ADC 时序（采样窗口、逐次逼近节拍）与数字校准；
//   2. 互补 PWM 死区，非法配置直接进安全态；
//   3. 旋变角度解算与锁定指示。
// 故障位（PWM 配置错 / 曾出现上下桥臂重叠）回读到寄存器，主机可查。
//======================================================================
`timescale 1ns/1ps
`default_nettype none

module ms_top #(
    parameter ADC_W    = 12,
    parameter CLK_DIV  = 4,
    parameter PWM_W    = 12,
    parameter GAIN_SHIFT = 7
)(
    input  wire                    clk,
    input  wire                    rst_n,
    // ---- SPI 配置口 ----
    input  wire                    spi_sclk,
    input  wire                    spi_cs_n,
    input  wire                    spi_mosi,
    output wire                    spi_miso,
    // ---- ADC 模拟侧接口 ----
    output wire [ADC_W-1:0]        adc_dac_code,   // 送 CDAC 的试探码
    input  wire                    adc_comp_in,    // 比较器输出
    // ---- PWM 输出 ----
    output wire                    pwm_h,
    output wire                    pwm_l,
    // ---- 旋变接口 ----
    input  wire signed [15:0]      rdc_sin,
    input  wire signed [15:0]      rdc_cos,
    // ---- 观测 ----
    output wire                    rdc_locked,
    output wire [1:0]              faults          // {重叠告警, PWM 配置错}
);

    wire [15:0] pwm_period, pwm_duty, pwm_dt;
    wire [11:0] adc_gain;
    wire signed [11:0] adc_offset;
    wire        adc_start_pulse, pwm_en, rdc_en;
    wire [11:0] adc_raw;
    wire        adc_valid;
    wire [15:0] rdc_angle;
    wire        pwm_cfg_error, pwm_overlap_seen;
    wire        wr_seen;
    wire [7:0]  last_addr, last_wdata;

    assign faults = {pwm_overlap_seen, pwm_cfg_error};

    // ---------------- SPI 配置寄存器堆 ----------------
    spi_regfile u_spi (
        .clk(clk), .rst_n(rst_n),
        .sclk(spi_sclk), .cs_n(spi_cs_n), .mosi(spi_mosi), .miso(spi_miso),
        .pwm_period(pwm_period), .pwm_duty(pwm_duty), .pwm_dt(pwm_dt),
        .adc_gain(adc_gain), .adc_offset(adc_offset),
        .adc_start(adc_start_pulse), .pwm_en(pwm_en), .rdc_en(rdc_en),
        .adc_raw(adc_raw), .adc_valid(adc_valid), .rdc_angle(rdc_angle),
        .faults(faults),
        .wr_seen(wr_seen), .last_addr(last_addr), .last_wdata(last_wdata)
    );

    // ---------------- SAR ADC 数字控制 ----------------
    // start 既可以用 SPI 的单拍脉冲启动，也可以在 pwm_en 打开后自走；
    // 这里采用「主机写 CTRL.start 启动一次」，便于逐次核验时序。
    sar_adc_ctrl #(.WIDTH(ADC_W), .CLK_DIV(CLK_DIV), .GAIN_SHIFT(GAIN_SHIFT)) u_adc (
        .clk(clk), .rst_n(rst_n),
        .start(adc_start_pulse), .comp_in(adc_comp_in),
        .cfg_gain(adc_gain[ADC_W-1:0]), .cfg_offset(adc_offset[ADC_W-1:0]),
        .dac_code(adc_dac_code),
        .dout(), .dout_raw(adc_raw), .dout_valid(adc_valid),
        .sample_en(), .state_dbg()
    );

    // ---------------- 互补 PWM 死区发生器 ----------------
    pwm_deadtime #(.WIDTH(PWM_W)) u_pwm (
        .clk(clk), .rst_n(rst_n), .en(pwm_en),
        .cfg_period(pwm_period[PWM_W-1:0]),
        .cfg_duty(pwm_duty[PWM_W-1:0]),
        .cfg_deadtime(pwm_dt[PWM_W-1:0]),
        .pwm_h(pwm_h), .pwm_l(pwm_l),
        .cfg_error(pwm_cfg_error), .overlap_seen(pwm_overlap_seen),
        .cnt_dbg()
    );

    // ---------------- RDC 跟踪环路 ----------------
    // 主机把 rdc_en 从 0 写到 1 时，产生一次象限预置脉冲
    reg rdc_en_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rdc_en_d <= 1'b0;
        else        rdc_en_d <= rdc_en;
    end
    wire rdc_acquire = rdc_en & ~rdc_en_d;

    rdc_tracking u_rdc (
        .clk(clk), .rst_n(rst_n), .en(rdc_en), .acquire(rdc_acquire),
        .sin_in(rdc_sin), .cos_in(rdc_cos),
        .angle_out(rdc_angle), .speed_out(), .err_out(), .locked(rdc_locked)
    );

endmodule

`default_nettype wire
