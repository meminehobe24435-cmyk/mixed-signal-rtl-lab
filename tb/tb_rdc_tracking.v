//======================================================================
// tb_rdc_tracking.v —— 旋变数字转换（RDC）跟踪环路的自检测试台
//
// 检查项：
//   1. 12 个目标角度（含四个象限边界、±1 LSB 极值）都能在限定拍数内收敛，
//      且静态角误差 <= RDC_TOL；
//   2. locked 标志在收敛后拉高；
//   3. **斜坡跟踪逐位比对**：目标角以 64 LSB/拍匀速旋转 400 拍，
//      把 RTL 每拍的 angle_out 与 Python 定点参考模型的期望值**逐位比较** ——
//      任何一处移位方向、饱和或位宽写错都会在这里暴露。
//======================================================================
`timescale 1ns/1ps
`default_nettype none

module tb_rdc_tracking;

    reg                clk = 0;
    reg                rst_n = 0;
    reg                en = 0;
    reg                acquire = 0;
    reg  signed [15:0] sin_in = 0;
    reg  signed [15:0] cos_in = 0;

    wire [15:0]        angle_out;
    wire signed [31:0] speed_out, err_out;
    wire               locked;

    integer errors = 0;
    integer i, cyc;
    reg     done;

    `include "vectors.vh"

    rdc_tracking dut (
        .clk(clk), .rst_n(rst_n), .en(en), .acquire(acquire),
        .sin_in(sin_in), .cos_in(cos_in),
        .angle_out(angle_out), .speed_out(speed_out), .err_out(err_out), .locked(locked)
    );

    always #5 clk = ~clk;

    // 最小环绕角差（|a-b|）
    function [15:0] ang_delta;
        input [15:0] a, b;
        reg [15:0] d;
        begin
            d = a - b;
            ang_delta = (d > 16'd32767) ? (16'd0 - d) : d;
        end
    endfunction

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_rdc_tracking.vcd");
            $dumpvars(0, tb_rdc_tracking);
        end

        rst_n = 0; en = 0; acquire = 0;
        repeat (4) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);
        en = 1;

        // ---------- 1) 静态角度收敛 ----------
        for (i = 0; i < RDC_N; i = i + 1) begin
            sin_in  = RDC_SIN[i];
            cos_in  = RDC_COS[i];
            acquire = 1'b1;
            @(negedge clk);
            acquire = 1'b0;

            cyc  = 0;
            done = 1'b0;
            while (!done && cyc < RDC_CYC_LIMIT) begin
                @(negedge clk);
                cyc = cyc + 1;
                if (ang_delta(angle_out, RDC_TGT[i]) <= RDC_TOL)
                    done = 1'b1;
            end

            if (!done) begin
                errors = errors + 1;
                $display("  [FAIL] 目标 %04X 在 %0d 拍内未收敛（当前 %04X，误差 %0d）",
                         RDC_TGT[i], RDC_CYC_LIMIT, angle_out, ang_delta(angle_out, RDC_TGT[i]));
            end else if (ang_delta(angle_out, RDC_TGT[i]) > RDC_TOL) begin
                errors = errors + 1;
                $display("  [FAIL] 目标 %04X 收敛后误差 %0d > %0d",
                         RDC_TGT[i], ang_delta(angle_out, RDC_TGT[i]), RDC_TOL);
            end
        end
        $display("  [OK] %0d 个静态角度均在 %0d 拍内收敛到 ±%0d LSB",
                 RDC_N, RDC_CYC_LIMIT, RDC_TOL);

        // 收敛判定只保证 |err| <= RDC_TOL，locked 还要求连续若干拍都在门限内，
        // 所以要多等一段再查 locked（这里按 RDC_CYC_LIMIT 再等一轮）。
        for (cyc = 0; cyc < RDC_CYC_LIMIT; cyc = cyc + 1)
            @(negedge clk);

        if (!locked) begin
            errors = errors + 1;
            $display("  [FAIL] 收敛后 locked 未拉高");
        end else begin
            $display("  [OK] locked 标志正常");
        end

        // ---------- 2) 斜坡跟踪：逐位比对 ----------
        rst_n = 0;
        repeat (2) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);

        for (i = 0; i < RAMP_N; i = i + 1) begin
            sin_in  = RAMP_SIN[i];
            cos_in  = RAMP_COS[i];
            acquire = (i == 0);
            @(negedge clk);          // 期间经过一个 posedge，RTL 处理第 i 拍
            if (angle_out !== RAMP_EXP_ANG[i]) begin
                errors = errors + 1;
                if (errors < 12)     // 只打前几条，避免刷屏
                    $display("  [FAIL] 斜坡第 %0d 拍 期望 %04X 实际 %04X",
                             i, RAMP_EXP_ANG[i], angle_out);
            end
        end
        $display("  [OK] 斜坡跟踪 %0d 拍逐位比对完成（rate=%0d LSB/拍）", RAMP_N, RAMP_RATE);

        $display("TEST_RESULT: tb_rdc_tracking errors=%0d", errors);
        if (errors == 0) $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #5000000;
        $display("TEST_RESULT: tb_rdc_tracking TIMEOUT");
        $finish;
    end

endmodule

`default_nettype wire
