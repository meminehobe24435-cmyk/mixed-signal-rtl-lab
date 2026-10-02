//======================================================================
// tb_sar_adc_ctrl.v —— SAR ADC 数字控制器的自检测试台
//
// 检查项：
//   1. 每一组输入码的原始转换结果 == Python 参考模型的期望码；
//   2. 校准通路（增益 / 失调 / 上下饱和）与参考模型一致；
//   3. 采样窗口长度、转换周期数、valid 脉冲宽度符合时序约定；
//   4. 转换期间不会错过启动请求（连续背靠背转换）。
//======================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_sar_adc_ctrl;

    localparam WIDTH   = 12;
    localparam CLK_DIV = 4;

    reg                  clk = 0;
    reg                  rst_n = 0;
    reg                  start = 0;
    reg  [WIDTH-1:0]     vin = 0;
    reg  [WIDTH-1:0]     cfg_gain = 128;
    reg signed [WIDTH-1:0] cfg_offset = 0;

    wire [WIDTH-1:0] dac_code;
    wire [WIDTH-1:0] dout, dout_raw;
    wire             dout_valid, sample_en;
    wire [2:0]       state_dbg;

    // 理想比较器：CDAC 输出正比于 dac_code，输出 1 表示仍不大于输入
    wire comp_in = (dac_code <= vin);

    integer errors = 0;
    integer i;
    integer sample_cycles, convert_cycles;
    reg     got;

    `include "vectors.vh"

    sar_adc_ctrl #(.WIDTH(WIDTH), .CLK_DIV(CLK_DIV), .GAIN_SHIFT(7)) dut (
        .clk(clk), .rst_n(rst_n), .start(start), .comp_in(comp_in),
        .cfg_gain(cfg_gain), .cfg_offset(cfg_offset),
        .dac_code(dac_code), .dout(dout), .dout_raw(dout_raw),
        .dout_valid(dout_valid), .sample_en(sample_en), .state_dbg(state_dbg)
    );

    always #5 clk = ~clk;      // 100 MHz

    // 周期统计放在独立 always 里、并且**在 negedge 采样**：
    //   ① 写进 task 的 while 里会「把 dout_valid 脉冲等过去」（脉冲只有 1 拍）→ task 卡死；
    //   ② 在 posedge 采样会撞上非阻塞赋值的调度顺序，数出来的拍数会少 1 拍。
    always @(negedge clk) begin
        if (rst_n) begin
            if (sample_en)         sample_cycles  = sample_cycles + 1;
            if (state_dbg == 3'd2) convert_cycles = convert_cycles + 1;
        end
    end

    // 单次转换：轮询到 valid 脉冲为止，同时抓走结果
    task do_convert(input [WIDTH-1:0] code, output [WIDTH-1:0] raw_o, output [WIDTH-1:0] cal_o);
        begin
            vin = code;
            sample_cycles = 0;
            convert_cycles = 0;
            @(negedge clk);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;
            got = 1'b0;
            while (!got) begin
                @(negedge clk);
                if (dout_valid) begin
                    raw_o = dout_raw;
                    cal_o = dout;
                    got   = 1'b1;
                end
            end
        end
    endtask

    reg [WIDTH-1:0] raw, cal;

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_sar_adc_ctrl.vcd");
            $dumpvars(0, tb_sar_adc_ctrl);
        end

        rst_n = 0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);

        // ---------- 1) 原始码 + 默认校准（增益 1.0 / 失调 0）----------
        cfg_gain = 128; cfg_offset = 0;
        for (i = 0; i < SAR_N; i = i + 1) begin
            do_convert(SAR_VIN[i], raw, cal);
            if (raw !== SAR_EXP[i]) begin
                errors = errors + 1;
                $display("  [FAIL] vin=%0d 期望 raw=%0d 实际 raw=%0d", SAR_VIN[i], SAR_EXP[i], raw);
            end
            if (cal !== SAR_CAL[i]) begin
                errors = errors + 1;
                $display("  [FAIL] vin=%0d 期望 cal=%0d 实际 cal=%0d", SAR_VIN[i], SAR_CAL[i], cal);
            end
        end
        $display("  [OK] %0d 组原始/校准码与参考模型一致", SAR_N);

        // ---------- 2) 增益 1.25 + 负失调（验证乘加与饱和）----------
        cfg_gain = SAR2_GAIN; cfg_offset = SAR2_OFFSET;
        for (i = 0; i < SAR_N; i = i + 1) begin
            do_convert(SAR_VIN[i], raw, cal);
            if (cal !== SAR2_CAL[i]) begin
                errors = errors + 1;
                $display("  [FAIL] gain=%0d off=%0d vin=%0d 期望 cal=%0d 实际 cal=%0d",
                         SAR2_GAIN, SAR2_OFFSET, SAR_VIN[i], SAR2_CAL[i], cal);
            end
        end
        $display("  [OK] 增益/失调校准 %0d 组与参考模型一致", SAR_N);

        // ---------- 3) 时序：采样窗口 + 转换周期数 ----------
        cfg_gain = 128; cfg_offset = 0;
        do_convert(12'd2048, raw, cal);
        if (sample_cycles !== CLK_DIV) begin
            errors = errors + 1;
            $display("  [FAIL] 采样窗口期望 %0d 拍 实际 %0d 拍", CLK_DIV, sample_cycles);
        end
        if (convert_cycles !== WIDTH * 2) begin
            errors = errors + 1;
            $display("  [FAIL] 转换周期期望 %0d 拍 实际 %0d 拍", WIDTH * 2, convert_cycles);
        end
        $display("  [OK] 采样 %0d 拍 / 转换 %0d 拍", sample_cycles, convert_cycles);

        // ---------- 4) 背靠背转换 ----------
        do_convert(12'd3000, raw, cal);
        if (raw !== 12'd3000) begin
            errors = errors + 1;
            $display("  [FAIL] 背靠背转换结果错误: %0d", raw);
        end
        do_convert(12'd1234, raw, cal);
        if (raw !== 12'd1234) begin
            errors = errors + 1;
            $display("  [FAIL] 连续第二次转换结果错误: %0d", raw);
        end
        $display("  [OK] 背靠背转换正常");

        $display("TEST_RESULT: tb_sar_adc_ctrl errors=%0d", errors);
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #2000000;
        $display("TEST_RESULT: tb_sar_adc_ctrl TIMEOUT");
        $finish;
    end

endmodule

`default_nettype wire
