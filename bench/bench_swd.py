#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bench_swd.py —— 没有 PC 串口时，用 SWD 跑「工况序列 / 压缩日」功耗采集
================================================================================
本板的 PC 口（USART3 → CHUAN 排针）需要外接 USB-TTL 才能读日志。手边没有时，
用调试探针的 SWD 通道替代，全部经 OpenOCD（已实测单次调用 ~0.3 s 且
**完全不冻结目标**：Δtick 10.362 s vs Δwall 10.328 s）：

  触发：把命令字符串写进固件的 g_test_cmd[]，再置 g_test_go=1
        → 主循环调 SEQ_HandleCmd()，与 PC 串口通道共用同一套逻辑
  回读：固件每进入一档就把 {步号, 步名, RTC 秒计数} 记进 g_slog
        → 脚本轮询回读，转成 wall-clock，产出与 bench_run.py 同格式的 markers.csv

时间基准用 **RTC 秒计数**：Stop 期间 SysTick 被挂起会冻结，而 RTC（LSE / VBAT 域）
一直在走，是唯一能跨 Stop 的时标。脚本在注入前后各读一次 RTC 做线性标定。

用法
    python bench_swd.py --cmd "seq 60"
    python bench_swd.py --cmd "day 30 3 G 255" --poll 15
    python bench_swd.py --cmd "stop"
    python bench_swd.py --selftest                 # 只测 SWD 通道通不通
    python bench_swd.py --read-log                 # 回读上次残留的 g_slog
    python bench_swd.py --remark data/x_markers.csv  # 用新算法重算既有 markers 的时间轴

注意（重要）
    板子在**窗口轮的 LP 收尾里会跑 nettime 授时**，会把 RTC 计数从「上电自由计数」
    直接改写成 **Unix epoch**（≈1.79e9）。所以「一个线性 scale 贯穿全程」是错的：
    老版实测算出 scale≈1e-6，把前 10 档全挤到 6.9 s、后 3 档全挤到 1204 s。
    现在改为**逐档判 RTC 增量是否可信，不可信就退回标称时长**（见 reconstruct_times）。

参数
    --cmd         命令（不含前导 '!'）
    --elf         固件 elf（用来查符号地址），默认 ../build/Debug/stm32f103_CubemxCLI_demo0.elf
    --f11-port    F-11 串口，缺省自动探测
    --lead        注入前空采秒数（默认 4）
    --poll        轮询 g_slog 的间隔秒（默认 15）
    --max         硬超时秒（默认 2400）
    --out         输出前缀
    --no-f11      只做 SWD（不采电流）

输出
    <out>_f11.csv / <out>_markers.csv / <out>_barrier.txt
    <out>_swd.log     每次 SWD 调用的记录
"""

import argparse
import csv
import datetime
import os
import re
import struct
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

OCD = r"E:/Program Files/stm32tools/openocd/0.12.0-7.1/.content/bin/openocd.exe"
OCD_SCRIPTS = r"E:/Program Files/stm32tools/openocd/0.12.0-7.1/.content/openocd/scripts"
NM = r"E:/Program Files/stm32tools/arm-none-eabi-gcc/14.2.1-1.1.1/.content/bin/arm-none-eabi-nm.exe"
DEFAULT_ELF = os.path.normpath(os.path.join(HERE, "..", "build", "Debug",
                                            "stm32f103_CubemxCLI_demo0.elf"))

LOG_N = 24
SLOG_WORDS = 127          # g_slog 共 508 字节
RTC_CNT_ADDR = 0x40002818  # RTC_CNTH / RTC_CNTL
# ⚠ STM32F1 的 RTC 寄存器**各占一个 32 位字**（只有低 16 位有效），
#   不是两个 16 位拼在一个字里：读 2 个字 → w[0]=CNTH, w[1]=CNTL。


def rtc_of(w):
    return (w[0] & 0xFFFF) * 65536 + (w[1] & 0xFFFF)


def log(m):
    print(m, flush=True)


# ------------------------------------------------------------------ SWD 通道 ---
class Swd(object):
    def __init__(self, swd_log=None, speed=4000):
        self.speed = speed
        self.logf = open(swd_log, "a", encoding="utf-8") if swd_log else None

    def run(self, cmds, timeout=40):
        args = [OCD, "-s", OCD_SCRIPTS,
                "-c", "adapter driver cmsis-dap; transport select swd; adapter speed %d" % self.speed,
                "-c", "gdb_port disabled; tcl_port disabled; telnet_port disabled",
                "-f", "target/stm32f1x.cfg", "-c", "init"]
        for c in cmds:
            args += ["-c", c]
        args += ["-c", "exit"]
        t0 = time.time()
        p = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        t1 = time.time()
        out = (p.stdout or "") + "\n" + (p.stderr or "")
        if self.logf:
            self.logf.write("--- %.3f  %.3fs  %s\n%s\n" % (t0, t1 - t0, "; ".join(cmds), out))
            self.logf.flush()
        return out, t0, t1

    @staticmethod
    def parse_mdw(text):
        """把 mdw 输出解析成按出现顺序的 [(addr, [words...]), ...]
           （mdw 每 8 个字换一行并重新打地址，故需按地址连续性拼接）"""
        blocks = []
        for line in text.splitlines():
            m = re.match(r"^\s*0x([0-9a-fA-F]{8}):\s*((?:[0-9a-fA-F]{8}\s*)+)$", line)
            if m:
                blocks.append((int(m.group(1), 16),
                               [int(x, 16) for x in m.group(2).split()]))
        return blocks

    def read_words(self, addr, n):
        out, t0, t1 = self.run(["mdw 0x%08x %d" % (addr, n)])
        words = []
        cur = None
        for a, w in self.parse_mdw(out):
            if cur is None:
                if a == addr:
                    words.extend(w)
                    cur = a + 4 * len(w)
            elif a == cur:
                words.extend(w)
                cur = a + 4 * len(w)
            elif a > cur:
                break
        if len(words) < n:
            raise RuntimeError("mdw 0x%08x %d 读取失败（得 %d 字）：\n%s"
                               % (addr, n, len(words), out[-400:]))
        return words[:n], (t0 + t1) / 2.0

    def rtc(self):
        w, mid = self.read_words(RTC_CNT_ADDR, 2)
        return rtc_of(w), mid

    def inject(self, addr_cmd, addr_go, text):
        s = text.encode("ascii", "replace")[:44]
        s = s + b"\0" * (48 - len(s))
        words = struct.unpack("<12I", s)
        cmds = []
        for i, w in enumerate(words):
            # OpenOCD 的 mww 一条只能写一个值，故逐字写
            cmds.append("mww 0x%08x 0x%08x" % (addr_cmd + 4 * i, w))
        cmds.append("mdw 0x%08x 12" % addr_cmd)
        out, t0, t1 = self.run(cmds)
        back = []
        cur = None
        for ad, w in self.parse_mdw(out):
            if ad == addr_cmd and cur is None:
                back.extend(w); cur = ad + 4 * len(w)
            elif cur is not None and ad == cur:
                back.extend(w); cur = ad + 4 * len(w)
            elif cur is not None and ad > cur:
                break
        if back[:12] != list(words):
            raise RuntimeError("g_test_cmd 回读校验失败\n 期望 %s\n 实得 %s\n%s"
                               % (["%08x" % w for w in words],
                                  ["%08x" % w for w in back[:12]], out[-500:]))
        # 读 RTC 锚点 + 置 go（放在同一次调用里，锚点时刻误差 <0.3s）
        out, t0, t1 = self.run(["mdw 0x%08x 2" % RTC_CNT_ADDR, "mww 0x%08x 0x1" % addr_go])
        k = None
        for ad, w in self.parse_mdw(out):
            if ad == RTC_CNT_ADDR:
                k = w
                break
        if not k:
            raise RuntimeError("读 RTC 锚点失败：\n%s" % out[-300:])
        return rtc_of(k), (t0 + t1) / 2.0

    def read_log(self):
        w, mid = self.read_words(SLOG_WORDS_ADDR, SLOG_WORDS)
        b = b"".join(struct.pack("<I", x) for x in w)
        n = b[0]
        done = b[1]
        ix = list(b[2:26])
        rtc = list(struct.unpack_from("<%dI" % LOG_N, b, 28))
        names = []
        for i in range(LOG_N):
            raw = b[124 + 16 * i: 124 + 16 * (i + 1)]
            names.append(raw.split(b"\0")[0].decode("ascii", "replace"))
        return {"n": n, "done": done, "ix": ix, "rtc": rtc, "names": names}, mid


def resolve_symbols(elf):
    p = subprocess.run([NM, "--numeric-sort", "--defined-only", elf],
                       capture_output=True, text=True, timeout=60)
    if p.returncode != 0:
        raise RuntimeError("nm 失败：%s" % p.stderr[:300])
    want = {"g_test_cmd": None, "g_test_go": None, "g_slog": None}
    for line in p.stdout.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] in want:
            want[parts[2]] = int(parts[0], 16)
    missing = [k for k, v in want.items() if v is None]
    if missing:
        raise RuntimeError("elf 里找不到符号：%s（固件是否含测试引擎？）" % missing)
    return want


SLOG_WORDS_ADDR = None    # 运行时填

# 步名 → 标称时长（秒）。与固件 Core/Src/main.c 的 SEQ_TABLE / SEQ_BuildDay 对应，
# 只用于报告里的「标称 s」列，不参与测量。
NOMINAL = {
    "v4goff_dark": 60, "v4goff_G255": 60, "v4goff_G128": 60,
    "v4goff_R255": 60, "v4goff_Y255": 60, "v4goff_R64": 60,
    "v4gon_reg": 120, "online_hb": 120, "online_idle": 60,
    "round_dark": 155, "round_G255": 155, "round_G128": 155,
    "v4goff_end": 60,
    "day_am1": 155, "day_am2": 155, "day_am3": 155, "day_am4": 155,
    "day_pm1": 155, "day_pm2": 155, "day_pm3": 155, "day_pm4": 155,
}


def nominal_of(name, cmd):
    if name in NOMINAL:
        return NOMINAL[name]
    if name.startswith("day_night") or name == "day_midday":
        m = re.match(r"\s*day\s+(\d+)", cmd or "")
        return int(m.group(1)) if m else 30
    return 0


def reconstruct_times(names, raws, dwells, wall_anchor):
    """从「锚点墙钟 + 每档的 RTC 计数」重建每档的起始墙钟。

    ⚠ 为什么不能用一个线性 scale：板子在**窗口轮的 LP 收尾里会跑 nettime 授时**，
    那会把 RTC 计数从「上电自由计数」(≈1000) 直接改写成 **Unix epoch**(≈1.79e9)。
    此时 (wall_b-wall_a)/(rtc_b-rtc_a) 会算出 ~1e-6 的假 scale，
    把整条时间轴压扁（P1 实测：前 10 档全挤在 6.9 s、后 3 档全挤在 1204 s）。

    做法：逐档只看「相邻 RTC 增量」，落在标称时长的可信区间内就用它，
    否则（授时跳变/异常）退回标称时长。scale 只用可信相邻对的中位比来估。
    """
    n = len(names)
    if n == 0:
        return [], 1.0, False

    ratios = []
    for i in range(1, n):
        nom = dwells[i - 1]
        d = raws[i] - raws[i - 1]
        if nom and d > 0 and 0.5 * nom <= d <= 2.0 * nom:
            ratios.append(float(d) / nom)
    ratios.sort()
    scale = ratios[len(ratios) // 2] if ratios else 1.0

    rel = [0.0]
    prev = raws[0]
    jumped = False
    for i in range(1, n):
        nom = dwells[i - 1]
        d = raws[i] - prev
        if nom and 0.5 * nom * scale <= d <= 2.0 * nom * scale:
            rel.append(rel[-1] + d * scale)
        else:
            jumped = jumped or (nom > 0)
            rel.append(rel[-1] + (nom if nom else 0))
        prev = raws[i]
    return [wall_anchor + x for x in rel], scale, jumped


# -------------------------------------------------------------------- 主流程 ---
def main():
    global SLOG_WORDS_ADDR
    ap = argparse.ArgumentParser()
    ap.add_argument("--cmd", default=None)
    ap.add_argument("--elf", default=DEFAULT_ELF)
    ap.add_argument("--f11-port", default=None)
    ap.add_argument("--lead", type=float, default=4.0)
    ap.add_argument("--poll", type=float, default=15.0)
    ap.add_argument("--max", type=float, default=2400.0, dest="max_s")
    ap.add_argument("--out", default=None)
    ap.add_argument("--no-f11", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--read-log", action="store_true")
    ap.add_argument("--remark", default=None, metavar="MARKERS_CSV",
                    help="用新算法重算既有 markers.csv 的时间轴（救回被 nettime 跳变压扁的数据）")
    a = ap.parse_args()

    # ---- 只重算既有 markers（不碰探针）----
    if a.remark:
        with open(a.remark, "r", encoding="utf-8", newline="") as f:
            rows = [d for d in csv.DictReader(f) if (d.get("name") or "").strip()]
        if not rows:
            log("!! %s 里没有有效档位行" % a.remark)
            return 1
        names = [r["name"] for r in rows]
        dwells = [int(float(r["dwell_s"])) for r in rows]
        raws = [int(float(r["board_tick_ms"])) for r in rows]
        wall_a = float(rows[0]["wall_epoch"])
        t0 = wall_a - float(rows[0]["t_rel_s"])
        walls, scale, jumped = reconstruct_times(names, raws, dwells, wall_a)
        rels = [w - t0 for w in walls]
        done = rels[-1] + dwells[-1]
        with open(a.remark, "w", encoding="utf-8", newline="") as f:
            f.write("index,name,dwell_s,wall_epoch,t_rel_s,board_tick_ms\n")
            for i in range(len(rows)):
                f.write("%d,%s,%d,%.3f,%.3f,%d\n"
                        % (i, names[i], dwells[i], walls[i], rels[i], raws[i]))
            f.write("DONE,,0,%.3f,%.3f,0\n" % (t0 + done, done))
        log("重算 %s：%d 档  中位 scale=%.6f%s"
            % (a.remark, len(rows), scale, "  (RTC 跳变后按标称回推)" if jumped else ""))
        for i in range(len(rows)):
            log("   %2d  %-14s t_rel=%8.1f  (标称 %d s)" % (i, names[i], rels[i], dwells[i]))
        return 0

    sym = resolve_symbols(a.elf)
    SLOG_WORDS_ADDR = sym["g_slog"]
    log("符号: g_test_cmd=0x%08x  g_test_go=0x%08x  g_slog=0x%08x"
        % (sym["g_test_cmd"], sym["g_test_go"], sym["g_slog"]))

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = a.out or os.path.join(HERE, "data", "bench_%s" % stamp)
    d = os.path.dirname(os.path.abspath(out))
    if d and not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)

    swd = Swd(swd_log=out + "_swd.log")

    # ---- 自检 ----
    if a.selftest or a.read_log:
        r0, m0 = swd.rtc()
        time.sleep(3.0)
        r1, m1 = swd.rtc()
        log("RTC 计数 %d -> %d  (Δt=%d s, Δwall=%.2fs)  %s"
            % (r0, r1, r1 - r0, m1 - m0, "OK" if 2 <= r1 - r0 <= 4 else "!! 异常"))
        st, _ = swd.read_log()
        log("g_slog: n=%d done=%d" % (st["n"], st["done"]))
        for i in range(st["n"]):
            log("   %2d  %-14s rtc=%d" % (i, st["names"][i], st["rtc"][i]))
        if a.read_log:
            return 0
        if not a.cmd:
            return 0

    if not a.cmd:
        log("!! 需要 --cmd（或 --selftest / --read-log）")
        return 1

    # ---- F-11 ----
    meter = None
    f11_csv = out + "_f11.csv"
    t0 = time.time()
    if not a.no_f11:
        from bench_run import Meter, autodetect_f11
        port = a.f11_port or autodetect_f11()
        if not port:
            log("!! 未找到 F-11（可用 --no-f11 只跑 SWD）")
            return 2
        meter = Meter(port, f11_csv, t0)
        meter.start()
        time.sleep(0.6)
        if meter.err:
            log("!! F-11 打开失败：%s" % meter.err)
            return 2
        log("  F-11 = %s" % port)

    log("  前置基线 %.1fs" % a.lead)
    time.sleep(max(0.0, a.lead))

    # ---- 注入 ----
    log(">>> SWD 注入 !%s" % a.cmd)
    rtc_a, wall_a = swd.inject(sym["g_test_cmd"], sym["g_test_go"], a.cmd)
    log("  锚点 rtc=%d wall=%.3f  (t_rel=%.2f)" % (rtc_a, wall_a, wall_a - t0))

    # stop / help 这类不是序列，注入完就结束（不会置 done，等下去只会白等）
    if not (a.cmd.strip().startswith("seq") or a.cmd.strip().startswith("day")):
        if meter:
            time.sleep(1.5)
            meter.stop_evt.set()
            time.sleep(0.4)
        log("  单发命令（非序列），注入后即退出。")
        return 0

    # ---- 轮询 ----
    markers = []
    deadline = time.time() + a.max_s
    last_n = 0
    st = None
    while time.time() < deadline:
        time.sleep(a.poll)
        try:
            st, mid = swd.read_log()
        except Exception as e:
            log("  (轮询失败，继续: %s)" % str(e)[:80])
            continue
        for i in range(last_n, st["n"]):
            log("  [%6.1fs] %2d  %s   rtc=%d"
                % (mid - t0, i, st["names"][i], st["rtc"][i]))
        last_n = st["n"]
        if st["done"]:
            log("<<< g_slog.done = 1，序列结束")
            break
    else:
        log("<<< 到达 --max %.0fs" % a.max_s)

    time.sleep(2.0)
    rtc_b, wall_b = swd.rtc()
    if meter:
        meter.stop_evt.set()
        time.sleep(0.5)
    t_end = time.time()

    # ---- 标定 & 出 markers ----
    if st is None:
        st, _ = swd.read_log()

    names = [st["names"][i] for i in range(st["n"])]
    dwells = [nominal_of(nm, a.cmd) for nm in names]
    walls, scale, jumped = reconstruct_times(names, st["rtc"][:st["n"]], dwells, wall_a)
    rels = [w - t0 for w in walls]
    log("  终点 rtc=%d wall=%.3f  中位 RTC 标定 scale=%.6f%s"
        % (rtc_b, wall_b, scale,
           "  !! 检测到 RTC 授时跳变，跳变后的档位用标称时长回推" if jumped else ""))

    done_rel = rels[-1] + (dwells[-1] if dwells else 0)
    if not st["done"]:
        done_rel = min(done_rel, t_end - t0)

    with open(out + "_markers.csv", "w", encoding="utf-8", newline="") as f:
        f.write("index,name,dwell_s,wall_epoch,t_rel_s,board_tick_ms\n")
        for i in range(st["n"]):
            f.write("%d,%s,%d,%.3f,%.3f,%d\n"
                    % (i, names[i], dwells[i], walls[i], rels[i], st["rtc"][i]))
        f.write("DONE,,0,%.3f,%.3f,0\n" % (t0 + done_rel, done_rel))

    with open(out + "_barrier.txt", "w", encoding="utf-8") as f:
        f.write("mode       = swd\ncmd        = !%s\n" % a.cmd)
        f.write("t0_epoch   = %.3f\n" % t0)
        f.write("rtc_anchor = %d @ wall %.3f (t_rel %.3f)\n" % (rtc_a, wall_a, wall_a - t0))
        f.write("rtc_end    = %d @ wall %.3f (t_rel %.3f)\n" % (rtc_b, wall_b, wall_b - t0))
        f.write("scale_med  = %.6f%s\n" % (scale, "  (RTC 授时跳变后按标称时长回推)" if jumped else ""))
        f.write("steps      = %d (done=%d)\n" % (st["n"], st["done"]))
        f.write("t_end      = %.3f (t_rel %.3f)\n" % (t_end, t_end - t0))

    log("")
    log("=== 汇总 ===")
    log("  工步 %d 个%s" % (st["n"], "" if st["done"] else "  !! 未收到 done"))
    if meter:
        log("  F-11 采样 %d 行 -> %s" % (meter.n, f11_csv))
    log("  标记 -> %s" % (out + "_markers.csv"))
    log("  SWD 日志 -> %s" % (out + "_swd.log"))
    log("")
    log("下一步：python bench_plot.py --f11 %s --markers %s"
        % (f11_csv, out + "_markers.csv"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
