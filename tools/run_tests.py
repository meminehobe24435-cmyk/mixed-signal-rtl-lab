# -*- coding: utf-8 -*-
"""一键跑全部 RTL 测试台（并可选跑综合）。

用法：
    python tools/run_tests.py             # 只跑仿真
    python tools/run_tests.py --synth     # 仿真 + yosys 综合
    python tools/run_tests.py --vcd       # 额外导出波形

退出码非 0 表示有测试台失败或综合失败 —— CI 直接用它当门禁。
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RTL = os.path.join(ROOT, 'rtl')
TB = os.path.join(ROOT, 'tb')
BUILD = os.path.join(ROOT, 'build')
SYNTH = os.path.join(ROOT, 'synth')

RTL_SOURCES = [
    'sar_adc_ctrl.v', 'pwm_deadtime.v', 'spi_regfile.v', 'rdc_tracking.v', 'ms_top.v',
]

TESTS = [
    # (名称, testbench 文件, 需要的 RTL 源文件)
    ('tb_sar_adc_ctrl', 'tb_sar_adc_ctrl.v', ['sar_adc_ctrl.v']),
    ('tb_pwm_deadtime', 'tb_pwm_deadtime.v', ['pwm_deadtime.v']),
    ('tb_spi_regfile',  'tb_spi_regfile.v',  ['spi_regfile.v']),
    ('tb_rdc_tracking', 'tb_rdc_tracking.v', ['rdc_tracking.v']),
    ('tb_ms_top',       'tb_ms_top.v',       RTL_SOURCES),
]


def find_tool(name, extra_dirs=()):
    """先查 PATH，再查几个常见安装位置（Windows 上的 oss-cad-suite / iverilog）。"""
    found = shutil.which(name)
    if found:
        return found
    for d in extra_dirs:
        for suffix in ('.exe', ''):
            cand = os.path.join(d, name + suffix)
            if os.path.isfile(cand):
                return cand
    return None


def run(cmd, cwd=None, env=None):
    proc = subprocess.run(cmd, cwd=cwd, env=env, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT)
    return proc.returncode, proc.stdout.decode('utf-8', 'replace')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--synth', action='store_true', help='额外跑 yosys 综合')
    ap.add_argument('--vcd', action='store_true', help='导出波形')
    args = ap.parse_args()

    os.makedirs(BUILD, exist_ok=True)

    iverilog = find_tool('iverilog', [r'C:\iverilog\bin', '/usr/bin', '/usr/local/bin'])
    vvp = find_tool('vvp', [r'C:\iverilog\bin', '/usr/bin', '/usr/local/bin'])
    if not iverilog or not vvp:
        print('!! 找不到 iverilog / vvp：请先安装 Icarus Verilog（apt install iverilog）')
        return 2

    # 1) 重新生成查找表与测试向量，保证 RTL 与参考模型对的是同一份数据
    print('=' * 72)
    print('生成查找表与测试向量（tools/golden.py）')
    print('=' * 72)
    rc, out = run([sys.executable, os.path.join(HERE, 'golden.py')])
    print(out.strip())
    if rc != 0:
        print('!! 生成失败')
        return rc

    plusargs = ['-g2005', '-Wall']
    if args.vcd:
        plusargs = ['-g2005', '-Wall']

    results = []
    for name, tb_file, sources in TESTS:
        print('=' * 72)
        print('运行 %s' % name)
        print('=' * 72)
        vvp_out = os.path.join(BUILD, name + '.vvp')
        cmd = [iverilog] + plusargs + ['-o', vvp_out, '-I', RTL, '-I', TB]
        cmd += [os.path.join(RTL, s) for s in sources]
        cmd += [os.path.join(TB, tb_file)]
        rc, out = run(cmd)
        if rc != 0:
            print(out)
            results.append((name, 'COMPILE_FAIL', -1, 0.0))
            continue

        sim_cmd = [vvp, vvp_out]
        if args.vcd:
            sim_cmd.append('+vcd')
        t0 = time.time()
        rc, out = run(sim_cmd)
        dt = time.time() - t0
        print(out.strip())

        m = re.search(r'errors=(\d+)', out)
        n_err = int(m.group(1)) if m else -1
        status = 'PASS' if ('ALL TESTS PASSED' in out and n_err == 0) else 'FAIL'
        results.append((name, status, n_err, dt))

    # 2) 可选：yosys 综合
    synth_note = ''
    if args.synth:
        yosys = find_tool('yosys', [r'D:\oss-cad-suite\bin', '/usr/bin', '/usr/local/bin'])
        if not yosys:
            synth_note = '未找到 yosys，跳过综合'
            print('!! ' + synth_note)
        else:
            print('=' * 72)
            print('yosys 综合（synth/synth.ys）')
            print('=' * 72)
            env = dict(os.environ)
            # ⚠️ oss-cad-suite 的工具必须**同时**把 bin 与 lib 加进 PATH，
            #    只加 bin 会以 0xC0000139（DLL 找不到）静默退出 —— 这个坑踩过。
            ydir = os.path.dirname(yosys)
            extra = [ydir, os.path.join(os.path.dirname(ydir), 'lib')]
            env['PATH'] = os.pathsep.join([d for d in extra if os.path.isdir(d)]) \
                + os.pathsep + env.get('PATH', '')
            rc, out = run([yosys, '-q', '-s', os.path.join(SYNTH, 'synth.ys')],
                          cwd=ROOT, env=env)
            print(out.strip()[-4000:])
            with open(os.path.join(SYNTH, 'synth_report.txt'), 'w', encoding='utf-8') as f:
                f.write(out)
            synth_note = 'yosys 综合通过' if rc == 0 else 'yosys 综合失败（rc=%d）' % rc
            if rc != 0:
                results.append(('yosys synth', 'FAIL', -1, 0.0))

    # 3) 汇总
    print()
    print('=' * 72)
    print('%-22s %-8s %8s %10s' % ('测试台', '结果', '错误数', '耗时(s)'))
    print('-' * 72)
    all_ok = True
    for name, status, n_err, dt in results:
        print('%-22s %-8s %8s %10.1f' % (name, status, n_err, dt))
        if status != 'PASS':
            all_ok = False
    print('-' * 72)
    n_pass = sum(1 for r in results if r[1] == 'PASS')
    print('通过 %d / %d' % (n_pass, len(results)))
    if synth_note:
        print(synth_note)
    print('总结论：' + ('全部通过' if all_ok else '存在失败'))
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
