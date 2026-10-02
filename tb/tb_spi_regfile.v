//======================================================================
// tb_spi_regfile.v —— SPI 从机 + 配置寄存器堆的自检测试台
//
// 主机侧按 SPI Mode 0（CPOL=0 / CPHA=0）逐位打时序：
//   CS 拉低 → 8 位地址 → 8 位数据（写）/ 8 位回读（读）→ CS 拉高
//
// 检查项：
//   1. 配置寄存器写进去能原样读回（16 位寄存器按 L/H 两字节访问）；
//   2. 有符号寄存器（ADC 失调）位扩展正确；
//   3. CTRL 的 start 位产生**单拍脉冲**，使能位保持；
//   4. 只读状态寄存器（ADC 原始码 / RDC 角度 / 故障位）能正确回读；
//   5. 未映射地址的写被忽略。
//======================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_spi_regfile;

    localparam HALF = 50;      // sclk 半周期（ns），远大于 3 拍同步所需

    reg  clk = 0;
    reg  rst_n = 0;
    reg  sclk = 0, cs_n = 1, mosi = 0;
    wire miso;

    wire [15:0] pwm_period, pwm_duty, pwm_dt;
    wire [11:0] adc_gain;
    wire signed [11:0] adc_offset;
    wire        adc_start, pwm_en, rdc_en, wr_seen;
    wire [7:0]  last_addr, last_wdata;

    reg  [11:0] adc_raw = 12'hABC;
    reg         adc_valid = 1'b1;
    reg  [15:0] rdc_angle = 16'h1234;
    reg  [1:0]  faults = 2'b10;

    integer errors = 0;
    integer b;
    reg [7:0] rdata;
    reg       start_pulse_seen = 0;
    integer   start_high_cycles = 0;

    // 地址映射（与 RTL 一致）
    localparam A_PWM_PERIOD_L = 8'h10, A_PWM_PERIOD_H = 8'h11;
    localparam A_PWM_DUTY_L   = 8'h12, A_PWM_DUTY_H   = 8'h13;
    localparam A_PWM_DT_L     = 8'h14, A_PWM_DT_H     = 8'h15;
    localparam A_ADC_GAIN     = 8'h20, A_ADC_OFF_L    = 8'h21, A_ADC_OFF_H = 8'h22;
    localparam A_CTRL         = 8'h30;
    localparam A_ADC_RAW_L    = 8'h40, A_ADC_RAW_H    = 8'h41;
    localparam A_RDC_ANG_L    = 8'h42, A_RDC_ANG_H    = 8'h43;
    localparam A_FAULT        = 8'h44;

    spi_regfile dut (
        .clk(clk), .rst_n(rst_n), .sclk(sclk), .cs_n(cs_n), .mosi(mosi), .miso(miso),
        .pwm_period(pwm_period), .pwm_duty(pwm_duty), .pwm_dt(pwm_dt),
        .adc_gain(adc_gain), .adc_offset(adc_offset),
        .adc_start(adc_start), .pwm_en(pwm_en), .rdc_en(rdc_en),
        .adc_raw(adc_raw), .adc_valid(adc_valid), .rdc_angle(rdc_angle), .faults(faults),
        .wr_seen(wr_seen), .last_addr(last_addr), .last_wdata(last_wdata)
    );

    always #5 clk = ~clk;

    // 观测 adc_start 是否为单拍脉冲
    always @(posedge clk) begin
        if (rst_n && adc_start) begin
            start_pulse_seen = 1;
            start_high_cycles = start_high_cycles + 1;
        end
    end

    // 一帧 16 bit：地址字节 = {R/W, A[6:0]}，数据字节在后
    // （is_read=1 时数据字节由 MISO 回读，从机不会把 0x00 写进寄存器）
    task spi_frame(input [7:0] addr, input [7:0] wdata, input is_read);
        reg [7:0] addr_byte;
        begin
            addr_byte = {is_read, addr[6:0]};
            cs_n = 1'b0; sclk = 1'b0; mosi = 1'b0;
            #(HALF);
            for (b = 7; b >= 0; b = b - 1) begin
                mosi = addr_byte[b];
                #(HALF); sclk = 1'b1;         // 上升沿：从机采样
                #(HALF); sclk = 1'b0;         // 下降沿：从机更新 MISO
            end
            for (b = 7; b >= 0; b = b - 1) begin
                mosi = is_read ? 1'b0 : wdata[b];
                #(HALF); sclk = 1'b1;         // 上升沿：主机采样 MISO
                rdata[b] = miso;
                #(HALF); sclk = 1'b0;
            end
            #(HALF); cs_n = 1'b1;
            #(HALF);
        end
    endtask

    // 读：返回寄存器值
    task spi_read(input [7:0] addr, output [7:0] value);
        begin
            spi_frame(addr, 8'h00, 1'b1);
            value = rdata;
        end
    endtask

    task expect8(input [7:0] got, input [7:0] want, input [127:0] what);
        begin
            if (got !== want) begin
                errors = errors + 1;
                $display("  [FAIL] %0s 期望 %02X 实际 %02X", what, want, got);
            end
        end
    endtask

    reg [7:0] rd;

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_spi_regfile.vcd");
            $dumpvars(0, tb_spi_regfile);
        end

        rst_n = 0; cs_n = 1; sclk = 0; mosi = 0;
        repeat (5) @(negedge clk);
        rst_n = 1;
        repeat (5) @(negedge clk);

        // ---------- 1) 写 16 位配置寄存器并读回 ----------
        spi_frame(A_PWM_PERIOD_L, 8'h34, 1'b0);
        spi_frame(A_PWM_PERIOD_H, 8'h12, 1'b0);
        spi_read(A_PWM_PERIOD_L, rd); expect8(rd, 8'h34, "PWM_PERIOD_L");
        spi_read(A_PWM_PERIOD_H, rd); expect8(rd, 8'h12, "PWM_PERIOD_H");
        if (pwm_period !== 16'h1234) begin
            errors = errors + 1;
            $display("  [FAIL] pwm_period 期望 1234 实际 %04X", pwm_period);
        end

        spi_frame(A_PWM_DUTY_L, 8'h78, 1'b0);
        spi_frame(A_PWM_DUTY_H, 8'h56, 1'b0);
        spi_frame(A_PWM_DT_L,   8'h0A, 1'b0);
        spi_frame(A_PWM_DT_H,   8'h00, 1'b0);
        if (pwm_duty !== 16'h5678 || pwm_dt !== 16'h000A) begin
            errors = errors + 1;
            $display("  [FAIL] pwm_duty=%04X pwm_dt=%04X", pwm_duty, pwm_dt);
        end
        $display("  [OK] 16 位配置寄存器写入并读回一致");

        // ---------- 2) 有符号寄存器位扩展 ----------
        spi_frame(A_ADC_GAIN, 8'hA0, 1'b0);              // 160
        spi_frame(A_ADC_OFF_L, 8'hEC, 1'b0);             // -20 的低 8 位
        spi_frame(A_ADC_OFF_H, 8'h0F, 1'b0);             // 高 4 位 = 1111 → -20
        if (adc_gain !== 12'h0A0) begin
            errors = errors + 1;
            $display("  [FAIL] adc_gain 期望 0A0 实际 %03X", adc_gain);
        end
        if (adc_offset !== -12'sd20) begin
            errors = errors + 1;
            $display("  [FAIL] adc_offset 期望 -20 实际 %0d", adc_offset);
        end
        spi_read(A_ADC_OFF_H, rd); expect8(rd, 8'h0F, "ADC_OFF_H 回读");
        $display("  [OK] 有符号失调寄存器位扩展正确（%0d）", adc_offset);

        // ---------- 3) CTRL：start 单拍脉冲 + 使能位 ----------
        spi_frame(A_CTRL, 8'h01, 1'b0);                  // start=1
        repeat (10) @(negedge clk);
        if (!start_pulse_seen) begin
            errors = errors + 1;
            $display("  [FAIL] 写 CTRL.start 未产生脉冲");
        end
        if (start_high_cycles !== 1) begin
            errors = errors + 1;
            $display("  [FAIL] adc_start 应为单拍脉冲，实际高 %0d 拍", start_high_cycles);
        end
        if (adc_start !== 1'b0) begin
            errors = errors + 1;
            $display("  [FAIL] adc_start 未自动清零");
        end
        spi_frame(A_CTRL, 8'h06, 1'b0);                  // pwm_en=1, rdc_en=1
        repeat (10) @(negedge clk);
        if (!pwm_en || !rdc_en) begin
            errors = errors + 1;
            $display("  [FAIL] pwm_en/rdc_en 未置位（%b %b）", pwm_en, rdc_en);
        end
        $display("  [OK] CTRL 的 start 为单拍脉冲，使能位正常");

        // ---------- 4) 只读状态寄存器回读 ----------
        spi_read(A_ADC_RAW_L, rd); expect8(rd, 8'hBC, "ADC_RAW_L");
        spi_read(A_ADC_RAW_H, rd); expect8(rd, 8'hA,  "ADC_RAW_H");
        spi_read(A_RDC_ANG_L, rd); expect8(rd, 8'h34, "RDC_ANGLE_L");
        spi_read(A_RDC_ANG_H, rd); expect8(rd, 8'h12, "RDC_ANGLE_H");
        spi_read(A_FAULT,     rd); expect8(rd, 8'h06, "FAULT(valid=1,faults=10)");
        $display("  [OK] 只读状态寄存器回读正确（含 adc_valid 与故障位）");

        // ---------- 5) 未映射地址的写被忽略 ----------
        spi_frame(8'h7F, 8'hFF, 1'b0);
        if (pwm_period !== 16'h1234) begin
            errors = errors + 1;
            $display("  [FAIL] 未映射地址的写影响了已配置寄存器");
        end
        $display("  [OK] 未映射地址的写被忽略");

        $display("TEST_RESULT: tb_spi_regfile errors=%0d", errors);
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #2000000;
        $display("TEST_RESULT: tb_spi_regfile TIMEOUT");
        $finish;
    end

endmodule

`default_nettype wire
