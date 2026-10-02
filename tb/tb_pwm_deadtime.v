//======================================================================
// tb_pwm_deadtime.v —— 互补 PWM 死区发生器的自检测试台
//
// 检查项：
//   1. **上下桥臂同时导通的时钟数恒为 0**（这是死区存在的唯一理由）；
//   2. 死区宽度实测等于配置值（两个边沿都测：h↓→l↑ 与 l↓→h↑）；
//   3. 占空比与周期正确（用 DUT 自己的计数器测量边沿位置）；
//   4. 非法配置（duty+dt >= period）时进入安全态：两个输出都为低、cfg_error 拉高；
//   5. 使能关闭时输出全低。
//======================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_pwm_deadtime;

    localparam WIDTH = 12;

    reg  clk = 0;
    reg  rst_n = 0;
    reg  en = 0;
    reg  [WIDTH-1:0] cfg_period = 12'd1000;
    reg  [WIDTH-1:0] cfg_duty   = 12'd300;
    reg  [WIDTH-1:0] cfg_dt     = 12'd10;

    wire pwm_h, pwm_l, cfg_error, overlap_seen;
    wire [WIDTH-1:0] cnt_dbg;

    integer errors = 0;
    integer overlap_cnt = 0;
    integer i;

    // 边沿时刻记录（用 DUT 自己的计数器，避免在 TB 里再数一遍）
    reg h_prev = 0, l_prev = 0;
    integer h_fall_at = -1, l_rise_at = -1, l_fall_at = -1, h_rise_at = -1;
    integer gap_h2l = -1, gap_l2h = -1;
    integer duty_meas = -1;

    pwm_deadtime #(.WIDTH(WIDTH)) dut (
        .clk(clk), .rst_n(rst_n), .en(en),
        .cfg_period(cfg_period), .cfg_duty(cfg_duty), .cfg_deadtime(cfg_dt),
        .pwm_h(pwm_h), .pwm_l(pwm_l), .cfg_error(cfg_error),
        .overlap_seen(overlap_seen), .cnt_dbg(cnt_dbg)
    );

    always #5 clk = ~clk;

    // 重叠检测：每一拍只要 h 与 l 同时为高就计数（必须永远是 0）
    always @(negedge clk) begin
        if (rst_n && en && !cfg_error && pwm_h && pwm_l)
            overlap_cnt = overlap_cnt + 1;
    end

    // 边沿捕获
    always @(negedge clk) begin
        if (rst_n && en && !cfg_error) begin
            if (h_prev && !pwm_h) h_fall_at = cnt_dbg;   // h 下降
            if (!h_prev && pwm_h) h_rise_at = cnt_dbg;   // h 上升
            if (l_prev && !pwm_l) l_fall_at = cnt_dbg;   // l 下降
            if (!l_prev && pwm_l) l_rise_at = cnt_dbg;   // l 上升
        end
        h_prev <= pwm_h;
        l_prev <= pwm_l;
    end

    task run_cycles(input integer n);
        begin
            for (i = 0; i < n; i = i + 1) @(negedge clk);
        end
    endtask

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_pwm_deadtime.vcd");
            $dumpvars(0, tb_pwm_deadtime);
        end

        rst_n = 0; en = 0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);

        // ---------- 1) 正常配置 ----------
        cfg_period = 12'd1000; cfg_duty = 12'd300; cfg_dt = 12'd10;
        en = 1'b1;
        overlap_cnt = 0;
        h_fall_at = -1; l_rise_at = -1; l_fall_at = -1; h_rise_at = -1;
        run_cycles(4000);        // 跑满 4 个周期

        if (overlap_cnt !== 0) begin
            errors = errors + 1;
            $display("  [FAIL] 上下桥臂重叠 %0d 拍（必须为 0）", overlap_cnt);
        end else begin
            $display("  [OK] 4 个 PWM 周期内上下桥臂重叠 0 拍");
        end

        if (h_fall_at < 0 || l_rise_at < 0) begin
            errors = errors + 1;
            $display("  [FAIL] 未捕获到 h↓ / l↑ 边沿");
        end else begin
            gap_h2l = l_rise_at - h_fall_at;
            if (gap_h2l !== cfg_dt) begin
                errors = errors + 1;
                $display("  [FAIL] h↓→l↑ 死区期望 %0d 实际 %0d", cfg_dt, gap_h2l);
            end
        end

        if (l_fall_at < 0 || h_rise_at < 0) begin
            errors = errors + 1;
            $display("  [FAIL] 未捕获到 l↓ / h↑ 边沿");
        end else begin
            gap_l2h = (h_rise_at - l_fall_at + cfg_period) % cfg_period;
            if (gap_l2h !== cfg_dt) begin
                errors = errors + 1;
                $display("  [FAIL] l↓→h↑ 死区期望 %0d 实际 %0d", cfg_dt, gap_l2h);
            end
        end
        duty_meas = h_fall_at;      // h 在高电平结束时下降，其计数即占空比
        if (duty_meas !== cfg_duty) begin
            errors = errors + 1;
            $display("  [FAIL] 占空比期望 %0d 实际 %0d", cfg_duty, duty_meas);
        end
        $display("  [OK] 死区实测 h↓→l↑ = %0d 拍, l↓→h↑ = %0d 拍, 占空比 = %0d 拍",
                 gap_h2l, gap_l2h, duty_meas);

        // ---------- 2) 换一组死区与占空比 ----------
        cfg_period = 12'd500; cfg_duty = 12'd120; cfg_dt = 12'd25;
        overlap_cnt = 0;
        h_fall_at = -1; l_rise_at = -1; l_fall_at = -1; h_rise_at = -1;
        run_cycles(3000);
        if (overlap_cnt !== 0) begin
            errors = errors + 1;
            $display("  [FAIL] 第二组配置出现重叠 %0d 拍", overlap_cnt);
        end
        if ((l_rise_at - h_fall_at) !== cfg_dt) begin
            errors = errors + 1;
            $display("  [FAIL] 第二组 h↓→l↑ 死区期望 %0d 实际 %0d", cfg_dt, l_rise_at - h_fall_at);
        end
        if (h_fall_at !== cfg_duty) begin
            errors = errors + 1;
            $display("  [FAIL] 第二组占空比期望 %0d 实际 %0d", cfg_duty, h_fall_at);
        end
        $display("  [OK] 第二组配置（period=500 duty=120 dt=25）死区与占空比正确");

        // ---------- 3) 非法配置：duty + dt >= period ----------
        cfg_period = 12'd100; cfg_duty = 12'd95; cfg_dt = 12'd10;   // 95+10 > 100
        run_cycles(400);
        if (!cfg_error) begin
            errors = errors + 1;
            $display("  [FAIL] 非法配置未拉高 cfg_error");
        end
        if (pwm_h || pwm_l) begin
            errors = errors + 1;
            $display("  [FAIL] 非法配置下输出未进入安全态（h=%b l=%b）", pwm_h, pwm_l);
        end else begin
            $display("  [OK] 非法配置进入安全态并拉高 cfg_error");
        end

        // ---------- 4) 使能关闭 ----------
        cfg_period = 12'd1000; cfg_duty = 12'd300; cfg_dt = 12'd10;
        run_cycles(100);
        en = 1'b0;
        run_cycles(50);
        if (pwm_h || pwm_l) begin
            errors = errors + 1;
            $display("  [FAIL] 使能关闭后仍有输出（h=%b l=%b）", pwm_h, pwm_l);
        end else begin
            $display("  [OK] 使能关闭后输出全低");
        end

        $display("TEST_RESULT: tb_pwm_deadtime errors=%0d", errors);
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #2000000;
        $display("TEST_RESULT: tb_pwm_deadtime TIMEOUT");
        $finish;
    end

endmodule

`default_nettype wire
