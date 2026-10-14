#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_recommended.py —— 一次跑完「推荐版」全套工况测试（≈55 min + 出图）
================================================================================
三段时间：

  P1  seq 60              13 档工况表（≈20 min）
  P2  day 30 3 G 255      压缩日 第 1 轮（≈17 min）
  P3  day 30 3 G 255      压缩日 第 2 轮（复核一致性 / 看 4G 空闲电平有没有漂）

每段结束立刻出该段的标注曲线 HTML，所以就算后面某段失败，前面的数据也是完整的。
全部日志落到 data/recommended_<时间戳>.log。

用法
    python run_recommended.py            # 全套
    python run_recommended.py --only 1   # 只跑 P1
    python run_recommended.py --skip-plot
"""

import argparse
import datetime
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable

PHASES = [
    ("seq60", "seq 60",          1700.0, "P1 · 13 档工况表"),
    ("day1",  "day 30 3 G 255",  1500.0, "P2 · 压缩日 第1轮"),
    ("day2",  "day 30 3 G 255",  1500.0, "P3 · 压缩日 第2轮（复核）"),
]


def run(cmd, logf, echo=True):
    logf.write("\n$ %s\n" % " ".join(cmd))
    logf.flush()
    t0 = time.time()
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, bufsize=1)
    for line in p.stdout:
        logf.write(line)
        logf.flush()
        if echo:
            sys.stdout.write(line)
            sys.stdout.flush()
    p.wait()
    logf.write("(exit=%d, %.1fs)\n" % (p.returncode, time.time() - t0))
    logf.flush()
    return p.returncode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", type=int, default=0, help="只跑第 N 段（1/2/3）")
    ap.add_argument("--skip-plot", action="store_true")
    ap.add_argument("--poll", type=float, default=15.0)
    ap.add_argument("--stamp", default=None,
                    help="批次时间戳，多段分开跑时用同一个即可拼出一份报告")
    a = ap.parse_args()

    stamp = a.stamp or datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    data = os.path.join(HERE, "data")
    if not os.path.isdir(data):
        os.makedirs(data)
    logpath = os.path.join(data, "recommended_%s.log" % stamp)
    logf = open(logpath, "w", encoding="utf-8")

    print("=== 推荐版全套 ===  日志 %s" % logpath, flush=True)
    t_start = time.time()
    results = []

    for idx, (tag, cmd, mx, title) in enumerate(PHASES, start=1):
        if a.only and idx != a.only:
            continue
        out = os.path.join(data, "%s_%s" % (stamp, tag))
        print("\n########## %s   !%s  (上限 %.0fs) ##########" % (title, cmd, mx), flush=True)
        rc = run([PY, os.path.join(HERE, "bench_swd.py"),
                  "--cmd", cmd, "--out", out, "--max", str(mx),
                  "--poll", str(a.poll)], logf)
        f11 = out + "_f11.csv"
        mk = out + "_markers.csv"
        ok = rc == 0 and os.path.exists(f11) and os.path.exists(mk)
        results.append((title, rc, f11, mk))
        if ok and not a.skip_plot:
            # 保险：板子在窗口轮的 LP 收尾会跑 nettime，把 RTC 计数从自由计数改写成 epoch，
            # 老版单一 linear scale 会把时间轴压扁。重算一遍（纯读文件，不碰探针，幂等）。
            print("  -> 重算时间轴", flush=True)
            run([PY, os.path.join(HERE, "bench_swd.py"), "--remark", mk], logf, echo=False)
            print("  -> 出图", flush=True)
            run([PY, os.path.join(HERE, "bench_plot.py"),
                 "--f11", f11, "--markers", mk,
                 "--out", out + "_annotated.html",
                 "--title", "%s · 整板电流（F-11 真值）" % title], logf, echo=True)
        time.sleep(2)

    print("\n=== 全部结束，用时 %.1f min ===" % ((time.time() - t_start) / 60.0), flush=True)
    for title, rc, f11, mk in results:
        print("  %-26s rc=%d  %s" % (title, rc, os.path.basename(f11)), flush=True)
    logf.write("\n=== done ===\n")
    logf.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
