//======================================================================
// spi_regfile.v —— SPI 从机 + 配置寄存器堆（芯片的"数字配置口"）
//
// 数模混合芯片几乎都有一个串行配置口：主机写配置寄存器（增益、占空比、
// 死区、使能），读回状态寄存器（转换结果、角度、故障位）。
//
// 协议：SPI Mode 0（CPOL=0 / CPHA=0），CS 低有效，一帧 16 bit：
//   第 1 字节 = {R/W, A[6:0]}：**最高位是读写方向位**（1=读，0=写），低 7 位是地址
//   第 2 字节 = 写数据；读操作时该字节由 MISO 移出寄存器内容（MSB 先出，
//               在 sclk 下降沿更新，主机在上升沿采样）
//
// ⚠️ 方向位是必须的：一开始没设计它，**读操作的数据字节是 0x00**，
//    结果每次读都把目标寄存器写成 0 —— 读回来的值是对的、寄存器却已经被清掉了。
//    这个 bug 是被「写完 → 读回 → 再查输出端口」的 testbench 抓出来的。
//
// sclk 是异步输入，这里用三级同步 + 边沿检测把它搬进 clk 域
// （真实芯片还会再加输入去毛刺与 Schmitt 输入，此处从略并在 README 说明）。
//======================================================================
`timescale 1ns/1ps
`default_nettype none

module spi_regfile (
    input  wire        clk,
    input  wire        rst_n,
    // SPI 物理接口
    input  wire        sclk,
    input  wire        cs_n,
    input  wire        mosi,
    output reg         miso,
    // 配置输出
    output reg  [15:0] pwm_period,
    output reg  [15:0] pwm_duty,
    output reg  [15:0] pwm_dt,
    output reg  [11:0] adc_gain,
    output reg signed [11:0] adc_offset,
    output reg         adc_start,     // 单拍脉冲
    output reg         pwm_en,
    output reg         rdc_en,
    // 状态回读
    input  wire [11:0] adc_raw,
    input  wire        adc_valid,
    input  wire [15:0] rdc_angle,
    input  wire [1:0]  faults,
    output reg         wr_seen,       // 自检：本帧发生过写操作
    output reg  [7:0]  last_addr,     // 自检：最近一次访问的地址
    output reg  [7:0]  last_wdata     // 自检：最近一次写入的数据
);

    // ---------------- 地址映射 ----------------
    localparam A_PWM_PERIOD_L = 8'h10, A_PWM_PERIOD_H = 8'h11;
    localparam A_PWM_DUTY_L   = 8'h12, A_PWM_DUTY_H   = 8'h13;
    localparam A_PWM_DT_L     = 8'h14, A_PWM_DT_H     = 8'h15;
    localparam A_ADC_GAIN     = 8'h20, A_ADC_OFF_L    = 8'h21, A_ADC_OFF_H = 8'h22;
    localparam A_CTRL         = 8'h30;
    localparam A_ADC_RAW_L    = 8'h40, A_ADC_RAW_H    = 8'h41;
    localparam A_RDC_ANG_L    = 8'h42, A_RDC_ANG_H    = 8'h43;
    localparam A_FAULT        = 8'h44;

    // ---------------- 读数据译码（函数，便于在地址字节结束那拍直接取新地址）--
    function [7:0] rd_value;
        input [7:0] a;
        begin
            case (a)
                A_PWM_PERIOD_L: rd_value = pwm_period[7:0];
                A_PWM_PERIOD_H: rd_value = pwm_period[15:8];
                A_PWM_DUTY_L:   rd_value = pwm_duty[7:0];
                A_PWM_DUTY_H:   rd_value = pwm_duty[15:8];
                A_PWM_DT_L:     rd_value = pwm_dt[7:0];
                A_PWM_DT_H:     rd_value = pwm_dt[15:8];
                A_ADC_GAIN:     rd_value = adc_gain[7:0];
                A_ADC_OFF_L:    rd_value = adc_offset[7:0];
                A_ADC_OFF_H:    rd_value = {4'b0, adc_offset[11:8]};
                A_CTRL:         rd_value = {5'b0, rdc_en, pwm_en, adc_start};
                A_ADC_RAW_L:    rd_value = adc_raw[7:0];
                A_ADC_RAW_H:    rd_value = {4'b0, adc_raw[11:8]};
                A_RDC_ANG_L:    rd_value = rdc_angle[7:0];
                A_RDC_ANG_H:    rd_value = rdc_angle[15:8];
                A_FAULT:        rd_value = {5'b0, adc_valid, faults};
                default:        rd_value = 8'h00;
            endcase
        end
    endfunction

    // ---------------- sclk 同步与边沿检测 ----------------
    reg [2:0] sclk_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sclk_sync <= 3'b000;
        else        sclk_sync <= {sclk_sync[1:0], sclk};
    end
    wire sclk_rise = (sclk_sync[2:1] == 2'b01);
    wire sclk_fall = (sclk_sync[2:1] == 2'b10);

    reg [1:0] cs_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) cs_sync <= 2'b11;
        else        cs_sync <= {cs_sync[0], cs_n};
    end
    wire cs_active = ~cs_sync[1];

    // ---------------- 帧状态 ----------------
    localparam P_ADDR = 1'b0, P_DATA = 1'b1;
    reg        phase;
    reg        frame_on;
    reg        frame_read;    // 本帧方向（1=读，0=写），取地址字节最高位
    reg [3:0]  bit_cnt;
    reg [3:0]  data_idx;      // 数据字节内已完成的下降沿个数（决定何时移位）
    reg [7:0]  shreg;
    reg [7:0]  addr_reg;
    reg [7:0]  rd_shreg;

    wire [7:0] rx_byte   = {shreg[6:0], mosi};
    // 函数返回值不能直接位选，先落到 wire（Verilog-2001 限制，踩过一次）
    wire [7:0] rd_next   = rd_value({1'b0, rx_byte[6:0]});    wire       byte_done = sclk_rise && (bit_cnt == 4'd7);
    wire       is_write  = byte_done && (phase == P_DATA) && !frame_read;
    wire       is_addr   = byte_done && (phase == P_ADDR);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase      <= P_ADDR;
            frame_on   <= 1'b0;
            bit_cnt    <= 4'd0;
            data_idx   <= 4'd0;
            shreg      <= 8'd0;
            addr_reg   <= 8'd0;
            rd_shreg   <= 8'd0;
            miso       <= 1'b0;
            wr_seen    <= 1'b0;
            last_addr  <= 8'd0;
            last_wdata <= 8'd0;
            adc_start  <= 1'b0;
            pwm_en     <= 1'b0;
            rdc_en     <= 1'b0;
            pwm_period <= 16'd1000;
            pwm_duty   <= 16'd300;
            pwm_dt     <= 16'd10;
            adc_gain   <= 12'd128;
            adc_offset <= 12'sd0;
        end else begin
            adc_start <= 1'b0;   // 默认清零 → 写 CTRL 时形成单拍脉冲
            wr_seen   <= 1'b0;

            if (!cs_active) begin
                // 片选释放：复位帧
                frame_on <= 1'b0;
                phase    <= P_ADDR;
                bit_cnt  <= 4'd0;
                data_idx <= 4'd0;
                shreg    <= 8'd0;
                miso     <= 1'b0;
            end else begin
                frame_on <= 1'b1;

                if (sclk_rise) begin
                    shreg   <= rx_byte;
                    bit_cnt <= (bit_cnt == 4'd7) ? 4'd0 : bit_cnt + 4'd1;

                    if (is_addr) begin
                        // 地址收完 → 记录方向与地址，并预装载读数据：
                        // MSB 先挂到 MISO 上，主机在数据字节的第一个上升沿就会采到它
                        addr_reg   <= {1'b0, rx_byte[6:0]};
                        last_addr  <= rx_byte;
                        frame_read <= rx_byte[7];
                        phase      <= P_DATA;
                        data_idx   <= 4'd0;
                        rd_shreg   <= {rd_next[6:0], 1'b0};
                        miso       <= rd_next[7];
                    end

                    if (phase == P_DATA)
                        data_idx <= data_idx + 4'd1;

                    if (is_write) begin
                        last_wdata <= rx_byte;
                        wr_seen    <= 1'b1;
                        case (addr_reg)
                            A_PWM_PERIOD_L: pwm_period[7:0]   <= rx_byte;
                            A_PWM_PERIOD_H: pwm_period[15:8]  <= rx_byte;
                            A_PWM_DUTY_L:   pwm_duty[7:0]     <= rx_byte;
                            A_PWM_DUTY_H:   pwm_duty[15:8]    <= rx_byte;
                            A_PWM_DT_L:     pwm_dt[7:0]       <= rx_byte;
                            A_PWM_DT_H:     pwm_dt[15:8]      <= rx_byte;
                            A_ADC_GAIN:     adc_gain          <= {4'b0, rx_byte};
                            A_ADC_OFF_L:    adc_offset[7:0]   <= rx_byte;
                            A_ADC_OFF_H:    adc_offset[11:8]  <= rx_byte[3:0];
                            A_CTRL: begin
                                adc_start <= rx_byte[0];
                                pwm_en    <= rx_byte[1];
                                rdc_en    <= rx_byte[2];
                            end
                            default: ; // 只读或未映射地址：忽略写
                        endcase
                    end
                end

                if (sclk_fall && phase == P_DATA && data_idx != 4'd0) begin
                    // Mode 0：下降沿更新 MISO。⚠️ 必须等**第一个数据上升沿之后**才移位 ——
                    // 地址字节后面紧跟的那个下降沿如果就移位，会把 MSB 顶掉，
                    // 主机读回来的整字节会左移一位（这个 bug 被逐字节比对的 testbench 抓到）。
                    miso     <= rd_shreg[7];
                    rd_shreg <= {rd_shreg[6:0], 1'b0};
                end
            end
        end
    end

endmodule

`default_nettype wire
