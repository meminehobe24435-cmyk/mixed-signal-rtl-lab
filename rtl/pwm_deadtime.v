//======================================================================
// pwm_deadtime.v —— 互补 PWM 发生器（带可编程死区）
//
// 马达驱动 / 电源类数模混合芯片的数字侧一定会有这一块：
// 上下桥臂互补输出，**必须插入死区**，否则直通短路。
//
// 死区插在**两个边沿**上：
//   pwm_h 高电平区间 [0, duty)
//   pwm_l 高电平区间 [duty + dt, period - dt)
// 于是 h 下降 → l 上升 之间隔 dt，l 下降 → h 上升 之间也隔 dt，
// 「上下桥臂同时导通」的时钟数恒为 0。
//
// 非法配置（duty + dt >= period）不会输出重叠波形，而是拉高 cfg_error
// 并保持两个输出都为低（安全态）—— 这是数字侧必须替模拟侧守住的一环。
//======================================================================
`timescale 1ns/1ps
`default_nettype none

module pwm_deadtime #(
    parameter WIDTH = 12
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              en,             // 总使能；为 0 时输出全低
    input  wire [WIDTH-1:0]  cfg_period,     // PWM 周期（时钟数，需 >= 4）
    input  wire [WIDTH-1:0]  cfg_duty,       // 高电平时间
    input  wire [WIDTH-1:0]  cfg_deadtime,   // 死区时间（时钟数）
    output reg               pwm_h,
    output reg               pwm_l,
    output reg               cfg_error,      // 配置非法
    output reg               overlap_seen,   // 曾出现上下同时导通（应恒为 0）
    output reg  [WIDTH-1:0]  cnt_dbg
);

    reg [WIDTH-1:0] cnt;

    // 配置合法性：周期至少 4 拍；h 与 l 的高电平区间不能吃掉整个周期
    wire cfg_bad = (cfg_period < 4) ||
                   (cfg_duty < cfg_deadtime) ||
                   (cfg_duty + cfg_deadtime >= cfg_period);

    wire h_on = (cnt < cfg_duty);
    wire l_on = (cnt >= (cfg_duty + cfg_deadtime)) &&
                (cnt <  (cfg_period - cfg_deadtime));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt          <= {WIDTH{1'b0}};
            pwm_h        <= 1'b0;
            pwm_l        <= 1'b0;
            cfg_error    <= 1'b0;
            overlap_seen <= 1'b0;
            cnt_dbg      <= {WIDTH{1'b0}};
        end else begin
            cfg_error <= cfg_bad;
            if (!en || cfg_bad) begin
                // 安全态：两个桥臂全关
                cnt       <= {WIDTH{1'b0}};
                pwm_h     <= 1'b0;
                pwm_l     <= 1'b0;
                cnt_dbg   <= {WIDTH{1'b0}};
            end else begin
                cnt <= (cnt == cfg_period - 1) ? {WIDTH{1'b0}} : cnt + 1'b1;
                pwm_h <= h_on;
                pwm_l <= l_on;
                cnt_dbg <= cnt;
                // 自检：任何一拍出现同时导通就置位（用于外部读出与回归）
                if (h_on && l_on)
                    overlap_seen <= 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
