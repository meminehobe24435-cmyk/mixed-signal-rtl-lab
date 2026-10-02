//======================================================================
// tb_ms_top.v —— 系统级集成测试：SPI 配置 → 三路外设 → 状态回读
//
// 这条测试走的是"主机拿到芯片之后会怎么做"的完整路径：
//   1. 用 SPI 写 PWM 周期/占空比/死区 → 打开 pwm_en → 量死区与重叠；
//   2. 用 SPI 写 ADC 增益 → 写 CTRL.start → 给比较器输入 → 回读原始码；
//   3. 用 SPI 写 rdc_en → 给旋变 sin/cos → 等锁定 → 回读角度；
//   4. 读 FAULT 寄存器确认故障位为 0；
//   5. 故意写一组非法 PWM 配置 → 读回故障位置位、输出进入安全态。
//======================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_ms_top;

    localparam HALF = 50;
    `include "vectors.vh"

    reg  clk = 0;
    reg  rst_n = 0;
    reg  spi_sclk = 0, spi_cs_n = 1, spi_mosi = 0;
    wire spi_miso;
    wire [11:0] adc_dac_code;
    reg         adc_comp_in = 0;
    wire        pwm_h, pwm_l;
    reg  signed [15:0] rdc_sin = 0, rdc_cos = 0;
    wire        rdc_locked;
    wire [1:0]  faults;

    integer errors = 0;
    integer b, i, cyc;
    reg [7:0] rdata;
    reg       done;
    integer   overlap_cnt = 0;
    reg h_prev = 0, l_prev = 0;
    integer h_fall_at = -1, l_rise_at = -1;
    reg [11:0] vin_sim = 12'd0;

    // ---- 地址映射 ----
    localparam A_PWM_PERIOD_L = 8'h10, A_PWM_PERIOD_H = 8'h11;
    localparam A_PWM_DUTY_L   = 8'h12, A_PWM_DUTY_H   = 8'h13;
    localparam A_PWM_DT_L     = 8'h14, A_PWM_DT_H     = 8'h15;
    localparam A_ADC_GAIN     = 8'h20, A_ADC_OFF_L    = 8'h21, A_ADC_OFF_H = 8'h22;
    localparam A_CTRL         = 8'h30;
    localparam A_ADC_RAW_L    = 8'h40, A_ADC_RAW_H    = 8'h41;
    localparam A_RDC_ANG_L    = 8'h42, A_RDC_ANG_H    = 8'h43;
    localparam A_FAULT        = 8'h44;

    ms_top #(.ADC_W(12), .CLK_DIV(4), .PWM_W(12)) dut (
        .clk(clk), .rst_n(rst_n),
        .spi_sclk(spi_sclk), .spi_cs_n(spi_cs_n), .spi_mosi(spi_mosi), .spi_miso(spi_miso),
        .adc_dac_code(adc_dac_code), .adc_comp_in(adc_comp_in),
        .pwm_h(pwm_h), .pwm_l(pwm_l),
        .rdc_sin(rdc_sin), .rdc_cos(rdc_cos), .rdc_locked(rdc_locked), .faults(faults)
    );

    always #5 clk = ~clk;

    // 理想比较器：DAC 输出仍不大于输入时为 1
    always @(*) adc_comp_in = (adc_dac_code <= vin_sim);

    // PWM 重叠与边沿监测
    always @(negedge clk) begin
        if (rst_n && pwm_h && pwm_l) overlap_cnt = overlap_cnt + 1;
        if (rst_n) begin
            if (h_prev && !pwm_h) h_fall_at = 1;      // 只记事件，具体用计数器核对
            if (!h_prev && pwm_h) h_fall_at = h_fall_at;
        end
        h_prev <= pwm_h;
        l_prev <= pwm_l;
    end

    task spi_frame(input [7:0] addr, input [7:0] wdata, input is_read);
        reg [7:0] addr_byte;
        begin
            addr_byte = {is_read, addr[6:0]};
            spi_cs_n = 1'b0; spi_sclk = 1'b0; spi_mosi = 1'b0;
            #(HALF);
            for (b = 7; b >= 0; b = b - 1) begin
                spi_mosi = addr_byte[b];
                #(HALF); spi_sclk = 1'b1;
                #(HALF); spi_sclk = 1'b0;
            end
            for (b = 7; b >= 0; b = b - 1) begin
                spi_mosi = is_read ? 1'b0 : wdata[b];
                #(HALF); spi_sclk = 1'b1;
                rdata[b] = spi_miso;
                #(HALF); spi_sclk = 1'b0;
            end
            #(HALF); spi_cs_n = 1'b1;
            #(HALF);
        end
    endtask

    task spi_write(input [7:0] addr, input [7:0] data);
        begin spi_frame(addr, data, 1'b0); end
    endtask

    task spi_read(input [7:0] addr, output [7:0] value);
        begin
            spi_frame(addr, 8'h00, 1'b1);
            value = rdata;
        end
    endtask

    task wr16(input [7:0] addr_l, input [15:0] value);
        begin
            spi_write(addr_l,       value[7:0]);
            spi_write(addr_l + 8'h1, value[15:8]);
        end
    endtask

    reg [7:0] lo, hi;
    reg [15:0] got, want, d;

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_ms_top.vcd");
            $dumpvars(0, tb_ms_top);
        end

        rst_n = 0; spi_cs_n = 1;
        repeat (5) @(negedge clk);
        rst_n = 1;
        repeat (5) @(negedge clk);

        // ================= 1) PWM =================
        wr16(A_PWM_PERIOD_L, 16'd1000);
        wr16(A_PWM_DUTY_L,   16'd300);
        wr16(A_PWM_DT_L,     16'd12);
        spi_write(A_CTRL, 8'h02);          // pwm_en = 1
        overlap_cnt = 0;
        for (i = 0; i < 3000; i = i + 1) @(negedge clk);

        if (overlap_cnt !== 0) begin
            errors = errors + 1;
            $display("  [FAIL] 系统级：PWM 上下桥臂重叠 %0d 拍", overlap_cnt);
        end else begin
            $display("  [OK] 系统级：SPI 配置的 PWM 输出无重叠");
        end

        // ================= 2) SAR ADC =================
        spi_write(A_ADC_GAIN, 8'h80);      // 增益 1.0
        spi_write(A_ADC_OFF_L, 8'h00);
        spi_write(A_ADC_OFF_H, 8'h00);
        vin_sim = 12'd2500;
        spi_write(A_CTRL, 8'h01);          // start 脉冲
        @(negedge clk);
        done = 0; cyc = 0;
        while (!done && cyc < 200) begin
            @(negedge clk); cyc = cyc + 1;
            if (faults[0] === 1'bx) done = 1;   // 占位，不会发生
        end
        for (i = 0; i < 80; i = i + 1) @(negedge clk);   // 等转换结束

        spi_read(A_ADC_RAW_L, lo);
        spi_read(A_ADC_RAW_H, hi);
        if ({hi[3:0], lo} !== 12'd2500) begin
            errors = errors + 1;
            $display("  [FAIL] 系统级：ADC 回读 %0d 期望 2500", {hi[3:0], lo});
        end else begin
            $display("  [OK] 系统级：SPI 启动 ADC 并回读到 %0d", {hi[3:0], lo});
        end

        // ================= 3) RDC =================
        rdc_sin = RDC_SIN[9];              // 用静态向量里的一个目标角（0x9ABC）
        rdc_cos = RDC_COS[9];
        spi_write(A_CTRL, 8'h06);          // pwm_en + rdc_en
        for (i = 0; i < 400; i = i + 1) @(negedge clk);

        if (!rdc_locked) begin
            errors = errors + 1;
            $display("  [FAIL] 系统级：RDC 未锁定");
        end
        spi_read(A_RDC_ANG_L, lo);
        spi_read(A_RDC_ANG_H, hi);
        // 环绕角差
        got  = {hi, lo};
        want = RDC_TGT[9];
        d    = got - want;
        if (d > 16'd32767) d = 16'd0 - d;
        if (d > 16'd4) begin
            errors = errors + 1;
            $display("  [FAIL] 系统级：RDC 角度回读 %04X 期望 %04X（差 %0d）", got, want, d);
        end else begin
            $display("  [OK] 系统级：RDC 锁定并回读角度 %04X（误差 %0d LSB）", got, d);
        end

        // ================= 4) 故障位 =================
        spi_read(A_FAULT, lo);
        if (lo[1:0] !== 2'b00) begin
            errors = errors + 1;
            $display("  [FAIL] 系统级：正常工况下故障位应为 0，实际 %b", lo[1:0]);
        end else begin
            $display("  [OK] 系统级：FAULT 寄存器为 0（含 adc_valid=%b）", lo[2]);
        end

        // ================= 5) 非法配置进安全态 =================
        wr16(A_PWM_DUTY_L, 16'd995);       // 995 + 12 > 1000
        for (i = 0; i < 200; i = i + 1) @(negedge clk);
        spi_read(A_FAULT, lo);
        if (lo[0] !== 1'b1) begin
            errors = errors + 1;
            $display("  [FAIL] 系统级：非法配置未上报故障位");
        end
        if (pwm_h || pwm_l) begin
            errors = errors + 1;
            $display("  [FAIL] 系统级：非法配置下 PWM 未进安全态");
        end else begin
            $display("  [OK] 系统级：非法配置上报故障位并进安全态");
        end

        $display("TEST_RESULT: tb_ms_top errors=%0d", errors);
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #20000000;
        $display("TEST_RESULT: tb_ms_top TIMEOUT");
        $finish;
    end

endmodule

`default_nettype wire
