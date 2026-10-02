# -*- coding: utf-8 -*-
"""定点参考模型与查找表生成（零第三方依赖）。

三件事：
1. 生成 RTL 用的正弦查找表 `rtl/sincos_rom.vh`（Q1.15，1/4 波 + 象限折叠）；
2. **用与 RTL 完全相同的整数运算**模拟两处环路，产出「黄金参考」：
   - SAR ADC 的逐次逼近过程（哪个输入码应该得到哪个输出码）
   - RDC 跟踪环路的每拍角度序列（用于与 RTL 逐位比对）
3. 把测试向量写成 `tb/*.vh`，让 testbench 直接比对。

之所以要在 Python 里写一遍同样的整数运算，是因为**定点位宽和移位方向错了，
仿真照样跑得动、波形看着也像对的** —— 只有拿独立算出来的期望值逐位对，
才能把「移位少了 1 位」「饱和写成了截断」这类问题抓出来。
"""
from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RTL = os.path.join(ROOT, 'rtl')
TB = os.path.join(ROOT, 'tb')
ART = os.path.join(ROOT, 'artifacts')

# ------------------------------------------------------------------ 参数
WIDTH = 12              # ADC / PWM 位宽
GAIN_SHIFT = 7          # ADC 增益小数位
ANGLE_W = 16            # 角度位宽：65536 = 360°
LUT_BITS = 8            # 每象限表深 2^8 = 256
LUT_N = 1 << LUT_BITS   # 256

KP = 32                 # 比例增益（右移 KP_SHIFT）——32/64 = 0.5，实测 12~14 拍收敛
KP_SHIFT = 6
KI = 1                  # 积分增益（右移 KI_SHIFT）——积分器用 Q(KI_SHIFT) 定点累加
KI_SHIFT = 16
INT_FREEZE = 1024       # 抗积分饱和：|误差| 超过它就冻结积分器（1024 LSB ≈ 5.6°）
ERR_SCALE = 10430       # 弧度 → 角度单位：65536/(2π) ≈ 10430
ERR_SHIFT = 30          # diff(Q2.30) * ERR_SCALE 后右移
RAMP_RATE = 64          # 斜坡跟踪测试的角速度（角度 LSB/拍）
RAMP_CYCLES = 400       # 斜坡跟踪测试的拍数


def rnd_shift(value, shift):
    """带四舍五入的算术右移（RTL 里同样做）——纯截断会留下几 LSB 的静态误差。"""
    return (value + (1 << (shift - 1))) >> shift if shift > 0 else value


def coarse_angle(sin_in, cos_in):
    """象限预置：只看 sin/cos 的符号，把初始角度放到 ±45° 以内。

    ⚠️ 这是被参考模型逼出来的设计决定：Type-II 跟踪环路的误差信号是
    ``sin(θ−φ)``，当初始偏差正好 180° 时误差恒为 0，环路**永远锁不上**。
    所以真实 RDC 都必须先做一次粗略象限捕获再精细跟踪 —— 本模块照做。
    """
    s_neg = (sin_in < 0)
    c_neg = (cos_in < 0)
    if not s_neg and not c_neg:
        return 0x0000
    if not s_neg and c_neg:
        return 0x4000
    if s_neg and c_neg:
        return 0x8000
    return 0xC000


# ------------------------------------------------------------------ 查找表
def sin_lut():
    """1/4 波正弦表，Q1.15，长度 LUT_N+1（末点正好是 32767）。"""
    return [int(round(math.sin(math.pi / 2 * i / LUT_N) * 32767)) for i in range(LUT_N + 1)]


def sin_at(p6, lut):
    """正弦表**线性插值**取值。

    入参 ``p6`` 用 1/64 表步表示象限内位置（0..16384 对应 0..90°）：
    ``idx = p6>>6`` 选表项，低 6 位做线性插值。

    ⚠️ 为什么必须插值（本项目的关键教训）：只用 ``idx = angle>>6`` 直接查表时，
    估计用的 sin/cos **每 64 个角度 LSB 才更新一次**，误差信号 ``sin(θ−φ)``
    因此被量化成 64 LSB 的台阶 —— 环路无论怎么调参都只能收敛到 ±32 LSB，
    而且台阶内误差恒为 0、积分器只会单向漂移。
    加了插值之后误差信号恢复成 ≈Δθ，静态误差直接降到 ≤2 LSB。
    """
    idx = p6 >> 6
    frac = p6 & 63
    if idx >= LUT_N:
        return lut[LUT_N]
    v0 = lut[idx]
    v1 = lut[idx + 1]
    return v0 + (((v1 - v0) * frac) >> 6)


def sin_cos_of_angle(angle, lut):
    """按与 RTL 相同的象限折叠 + 线性插值，由 16 位角度取 (sin, cos)，Q1.15。"""
    a = angle & 0xFFFF
    quad = (a >> 14) & 0x3
    a_q = a & 0x3FFF                  # 象限内角度：0..16383 对应 0..90°
    s_q = sin_at(a_q, lut)
    c_q = sin_at(16384 - a_q, lut)    # cos(x) = sin(90°−x)
    if quad == 0:
        return s_q, c_q
    if quad == 1:
        return c_q, -s_q
    if quad == 2:
        return -s_q, -c_q
    return -c_q, s_q


# ------------------------------------------------------------------ ADC 黄金模型
def sar_convert(vin_code, width=WIDTH):
    """理想的逐次逼近：返回满足 CDAC(code) <= Vin 的最大码。

    RTL 里比较器约定 comp_in = (dac_code <= vin_code)，
    等价于「试探码仍不大于输入就保留这一位」。
    """
    result = 0
    for i in range(width - 1, -1, -1):
        trial = result | (1 << i)
        if trial <= vin_code:
            result = trial
    return result


def adc_calibrate(raw, gain, offset, width=WIDTH, shift=GAIN_SHIFT):
    """与 RTL 相同的校准：乘增益 → 右移 → 加失调 → 上下饱和。"""
    val = (raw * gain) >> shift
    val += offset
    if val < 0:
        return 0
    if val > (1 << width) - 1:
        return (1 << width) - 1
    return val


def sar_trace(vins, gain, offset):
    """（输入码 → 原始码 → 校准码）三元组序列。"""
    out = []
    for vin in vins:
        raw = sar_convert(vin)
        out.append((vin, raw, adc_calibrate(raw, gain, offset)))
    return out


# ------------------------------------------------------------------ RDC 黄金模型
def rdc_step(sin_in, cos_in, angle, integ_q, lut, acquire=False,
             kp=None, kps=None, kis=None, freeze=None):
    """一拍 RDC 环路（整数运算，与 RTL 完全一致）。

    err = sin(θ)cos(φ) − cos(θ)sin(φ) ≈ sin(θ−φ) ≈ θ−φ（小偏差时）
    acquire=1 时先按象限把角预置到 ±45° 以内，并把积分器清零。
    freeze 是抗积分饱和门限：偏差大于它时冻结积分器，避免大偏差期间把
    积分器灌满、之后又因为误差信号很小而退不下来。
    """
    kp = KP if kp is None else kp
    kps = KP_SHIFT if kps is None else kps
    kis = KI_SHIFT if kis is None else kis
    freeze = INT_FREEZE if freeze is None else freeze
    if acquire:
        # 与 RTL 对齐：捕获那一拍**只做象限预置**，不跑环路更新
        return coarse_angle(sin_in, cos_in), 0, 0
    s_est, c_est = sin_cos_of_angle(angle, lut)
    diff = sin_in * c_est - cos_in * s_est              # Q2.30，±2^30
    err = (diff * ERR_SCALE) >> ERR_SHIFT               # 角度单位，±16384
    prop = rnd_shift(err * kp, kps)
    # ⚠️ 顺序必须与 RTL 一致：RTL 的角度更新用的是**当前**（尚未累加）的积分器值，
    #    先累加再用会让参考模型与 RTL 差 1 LSB —— 逐位比对就是这么抓出来的。
    int_term = rnd_shift(integ_q, kis)
    if abs(err) < freeze:
        integ_q = integ_q + err
    angle = (angle + prop + int_term) & 0xFFFF
    return angle, integ_q, err


def rdc_trace(target_angle, cycles, lut, start_angle=0):
    """目标角度固定，跑 cycles 拍（首拍做象限捕获），返回每拍 (angle, integ, err)。"""
    sin_in, cos_in = sin_cos_of_angle(target_angle, lut)
    angle, integ = start_angle, 0
    trace = []
    for n in range(cycles):
        angle, integ, err = rdc_step(sin_in, cos_in, angle, integ, lut, acquire=(n == 0))
        trace.append((angle, integ, err))
    return trace


def rdc_ramp(lut, rate=64, cycles=400, start_tgt=0):
    """匀速旋转的跟踪序列：返回每拍 (sin, cos, 期望角度, 积分器, 误差)。

    目标角每拍 +rate（角度 LSB），输入是该目标角的理想 sin/cos，
    期望输出由本模型按**与 RTL 完全相同的整数运算**推出 ——
    testbench 逐拍比对，能抓出任何一位移位或饱和写错。
    """
    a, integ = 0, 0
    tgt = start_tgt
    trace = []
    for n in range(cycles):
        if n > 0:
            tgt = (tgt + rate) & 0xFFFF
        sin_in, cos_in = sin_cos_of_angle(tgt, lut)
        a, integ, err = rdc_step(sin_in, cos_in, a, integ, lut, acquire=(n == 0))
        trace.append((sin_in, cos_in, a, integ, err))
    return trace


def ang_diff(a, b):
    """两个 16 位角度的最小环绕差（有符号）。"""
    d = (a - b) & 0xFFFF
    return d - 65536 if d > 32767 else d


def rdc_converge(target, lut, tol=2, max_cycles=2000, start_angle=0):
    """跑到 |误差| <= tol 所需的拍数；返回 (拍数, 最终角度, 稳定拍数)。

    「稳定」= 之后连续 16 拍都保持在 tol 内（避免把一次穿过的瞬间当成收敛）。
    """
    sin_in, cos_in = sin_cos_of_angle(target, lut)
    angle, integ = start_angle, 0
    inside = 0
    first = -1
    for n in range(1, max_cycles + 1):
        angle, integ, _ = rdc_step(sin_in, cos_in, angle, integ, lut, acquire=(n == 1))
        if abs(ang_diff(angle, target)) <= tol:
            if inside == 0:
                first = n
            inside += 1
            if inside >= 16:
                return first, angle, inside
        else:
            inside = 0
    return -1, angle, inside


# ------------------------------------------------------------------ 输出
def emit_lut(path):
    lut = sin_lut()
    with open(path, 'w', encoding='utf-8') as f:
        f.write('// 自动生成：sin 1/4 波查找表（Q1.15，%d+1 点）—— 由 tools/golden.py 产出，勿手改\n' % LUT_N)
        f.write('// 用法：angle[15:0] 高 2 位选象限，[13:6] 选索引\n')
        f.write('localparam integer SIN_LUT_N = %d;\n' % LUT_N)
        for i, v in enumerate(lut):
            f.write('// SIN_LUT[%3d] = %6d\n' % (i, v))
        f.write('reg signed [15:0] SIN_LUT [0:%d];\n' % LUT_N)
        f.write('initial begin\n')
        for i, v in enumerate(lut):
            f.write('    SIN_LUT[%d] = 16\'sd%d;\n' % (i, v))
        f.write('end\n')
    return lut


def sv16(v):
    """Verilog 有符号字面量：负号必须写在位宽**前面**（``-16'sd5``），
    ``16'sd-5`` 是语法错误 —— 这个坑让 vectors.vh 编译直接挂掉过。"""
    return ("-16'sd%d" % (-v)) if v < 0 else ("16'sd%d" % v)


def emit_vectors(path, lut):
    """SAR + RDC 的测试向量（含期望值），供 testbench `include。"""
    vins = [0, 1, 17, 255, 256, 1000, 2047, 2048, 3000, 4094, 4095]
    gain, offset = 128, 0
    sar = sar_trace(vins, gain, offset)

    gain2, offset2 = 160, -20          # 1.25x 增益 + 负失调，验证校准通路
    sar2 = sar_trace(vins, gain2, offset2)

    # 目标为 3000 拍内收敛的若干角度（含四象限与边界）
    targets = [0x0000, 0x2000, 0x4000, 0x6000, 0x8000, 0xA000, 0xC000, 0xE000,
               0x1234, 0x9ABC, 0xFFFF, 0x0001]
    conf = [(t,) + rdc_converge(t, lut, tol=2) for t in targets]

    with open(path, 'w', encoding='utf-8') as f:
        f.write('// 自动生成：SAR / RDC 测试向量与期望值 —— 由 tools/golden.py 产出，勿手改\n')
        f.write('localparam integer SAR_N = %d;\n' % len(sar))
        f.write('localparam integer RDC_N = %d;\n' % len(conf))
        f.write('localparam integer RDC_TOL = 2;\n')          # 允许的静态角误差（LSB）
        f.write('localparam integer RDC_CYC_LIMIT = %d;\n' % (max(max(c[1] for c in conf) + 8, 32)))
        for i, (vin, raw, cal) in enumerate(sar):
            f.write('// SAR[%2d] vin=%4d raw=%4d cal=%4d\n' % (i, vin, raw, cal))
        f.write('reg [11:0] SAR_VIN  [0:%d];\n' % (len(sar) - 1))
        f.write('reg [11:0] SAR_EXP  [0:%d];\n' % (len(sar) - 1))
        f.write('reg [11:0] SAR_CAL  [0:%d];\n' % (len(sar) - 1))
        f.write('initial begin\n')
        for i, (vin, raw, cal) in enumerate(sar):
            f.write('    SAR_VIN[%d] = 12\'d%d; SAR_EXP[%d] = 12\'d%d; SAR_CAL[%d] = 12\'d%d;\n'
                    % (i, vin, i, raw, i, cal))
        f.write('end\n')
        # 第二组校准参数
        f.write('localparam integer SAR2_GAIN = %d;\n' % gain2)
        f.write('localparam integer SAR2_OFFSET = %d;\n' % offset2)
        f.write('reg [11:0] SAR2_CAL  [0:%d];\n' % (len(sar2) - 1))
        f.write('initial begin\n')
        for i, (_, _, cal) in enumerate(sar2):
            f.write('    SAR2_CAL[%d] = 12\'d%d;\n' % (i, cal))
        f.write('end\n')
        # RDC 目标角度与期望收敛拍数
        for i, (t, n, a, _) in enumerate(conf):
            f.write('// RDC[%2d] target=16\'h%04X cycles=%s final=%d\n' % (i, t, n, a))
        f.write('reg [15:0] RDC_TGT [0:%d];\n' % (len(conf) - 1))
        f.write('reg [15:0] RDC_CYC [0:%d];\n' % (len(conf) - 1))
        f.write('reg signed [15:0] RDC_SIN [0:%d];\n' % (len(conf) - 1))
        f.write('reg signed [15:0] RDC_COS [0:%d];\n' % (len(conf) - 1))
        f.write('initial begin\n')
        for i, (t, n, _, _) in enumerate(conf):
            s, c = sin_cos_of_angle(t, lut)
            f.write('    RDC_TGT[%d] = 16\'h%04X; RDC_CYC[%d] = 16\'d%d; '
                    'RDC_SIN[%d] = %s; RDC_COS[%d] = %s;\n'
                    % (i, t, i, max(n, 1), i, sv16(s), i, sv16(c)))
        f.write('end\n')

        # ---- 斜坡跟踪的逐拍期望序列：用于与 RTL **逐位比对** ----
        # 目标角每拍 +RAMP_RATE，输入 sin/cos 与期望角度都来自本模型，
        # testbench 只要把两路输入喂进去、把 angle_out 与期望值逐拍比即可。
        ramp = rdc_ramp(lut, rate=RAMP_RATE, cycles=RAMP_CYCLES, start_tgt=0)
        f.write('localparam integer RAMP_N = %d;\n' % len(ramp))
        f.write('localparam integer RAMP_RATE = %d;\n' % RAMP_RATE)
        f.write('reg signed [15:0] RAMP_SIN [0:%d];\n' % (len(ramp) - 1))
        f.write('reg signed [15:0] RAMP_COS [0:%d];\n' % (len(ramp) - 1))
        f.write('reg [15:0] RAMP_EXP_ANG [0:%d];\n' % (len(ramp) - 1))
        f.write('initial begin\n')
        for i, (s, c, ang, _integ, _err) in enumerate(ramp):
            f.write('    RAMP_SIN[%d] = %s; RAMP_COS[%d] = %s; RAMP_EXP_ANG[%d] = 16\'h%04X;\n'
                    % (i, sv16(s), i, sv16(c), i, ang))
        f.write('end\n')
    return sar, conf, ramp


def main():
    os.makedirs(RTL, exist_ok=True)
    os.makedirs(TB, exist_ok=True)
    os.makedirs(ART, exist_ok=True)
    lut = emit_lut(os.path.join(RTL, 'sincos_rom.vh'))
    sar, conf, ramp = emit_vectors(os.path.join(TB, 'vectors.vh'), lut)
    print('LUT %d 点；SAR 向量 %d 组；RDC 静态向量 %d 组；斜坡 %d 拍' %
          (len(lut), len(sar), len(conf), len(ramp)))
    print('RDC 收敛拍数：', [c[1] for c in conf])
    print('RDC 最终误差：', [ang_diff(c[2], c[0]) for c in conf])
    lags = [ang_diff((RAMP_RATE * i) & 0xFFFF, ramp[i][2]) for i in range(100, len(ramp))]
    print('斜坡 rate=%d：稳态滞后 均值 %.1f / 最大 %d LSB' %
          (RAMP_RATE, sum(lags) / len(lags), max(abs(v) for v in lags)))


if __name__ == '__main__':
    sys.exit(main())
