#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bench_run.py —— 车牌主板「工况序列 / 压缩日」功耗采集器
================================================================================
一条命令同时做三件事：

  1. 用 F-11 电流表（≈10 Hz）连续记录整机电流/电压          → <out>_f11.csv
  2. 读主板 PC 串口（USART3, 115200）日志，抓 SEQ 标记行     → <out>_board.log
                                                              <out>_markers.csv
  3. 居中把 '!<cmd>' 发给主板，启动固件里的工况测试引擎

两个串口跑在同一个进程、共用同一台 PC 的 wall-clock，
所以电流曲线与 SEQ 标记天然对齐，bench_plot.py 直接分段上色。

用法
    python bench_run.py --list                      # 列串口（含 F-11 自动识别）
    python bench_run.py --cmd "seq 60"              # 13 档工况表（约 20 min）
    python bench_run.py --cmd "day 30 3 G 255"      # 压缩日（约 17 min）
    python bench_run.py --cmd "stop"                # 中止测试，回常在线

    python bench_run.py --cmd "seq 60" --max 1800 --lead 5 --verbose

参数
    --cmd        发给板子的 '!' 命令行内容（不含 '!'），如 "seq 60"
    --board-port 主板 PC 串口，缺省自动探测
    --board-baud 缺省 115200
    --f11-port   F-11 电流表串口，缺省自动探测
    --lead       发命令前先空采几秒做基线（默认 3）
    --tail       收到 DONE 后再多采几秒（默认 3）
    --max        硬超时（秒，默认 2400）
    --out        输出前缀，缺省 bench_<YYYYmmdd_HHMMSS>
    --verbose    把板子所有日志行都打到屏幕（默认只打 SEQ 行）

输出
    <out>_f11.csv      t_s,wall_iso,current_uA,current_mA,voltage_mV,voltage_V
    <out>_board.log    wall_epoch <TAB> 板子日志原文
    <out>_markers.csv  index,name,dwell_s,wall_epoch,t_rel_s,board_tick_ms
    <out>_barrier.txt  一条人读的屏障说明（测试开始/结束时刻）

依赖 pyserial。本机推荐解释器：E:\\ProgramData\\Anaconda3\\python.exe
"""

import argparse
import datetime
import os
import re
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.stderr.write("需要 pyserial：pip install pyserial\n")
    sys.exit(1)

F11_LINE = re.compile(rb"^(\d{7}),(\d{5})\s*$")
F11_ONE  = re.compile(rb"^(\d{7})\s*$")
SEQ_MARK = re.compile(r"^SEQ,(\d+),([A-Za-z0-9_]+),(\d+),(\d+)")
SEQ_DONE = re.compile(r"^SEQ,DONE,(\d+),(\d+)")


def log(msg):
    print(msg, flush=True)


# ---------------------------------------------------------------- 端口探测 ---
def list_all_ports():
    out = []
    for p in list_ports.comports():
        out.append({"port": p.device,
                    "desc": (p.description or "").strip(),
                    "hwid": (p.hwid or "").strip()})
    return out


def probe_f11(port, baud=38400, secs=1.5):
    """按 38400 试读，命中 '^\\d{7},\\d{5}$' 即认定为 F-11。"""
    try:
        s = serial.Serial(port, baud, timeout=0.15, bytesize=8,
                          parity="N", stopbits=1)
    except Exception:
        return None
    try:
        s.reset_input_buffer()
        t0 = time.time()
        buf = bytearray()
        while time.time() - t0 < secs:
            buf += s.read(256)
            while b"\n" in buf:
                line, _, rest = buf.partition(b"\n")
                buf = bytearray(rest)
                line = line.strip(b"\r ")
                if F11_LINE.match(line):
                    return line.decode("ascii", "replace")
    except Exception:
        return None
    finally:
        s.close()
    return None


def autodetect_f11():
    for c in list_all_ports():
        hit = probe_f11(c["port"])
        if hit:
            log("  F-11 命中: %s  (%s)  首帧 %s" % (c["port"], c["desc"], hit))
            return c["port"]
    return None


def probe_board(port, baud=115200, secs=2.5):
    """只读探测：主板每秒打一行 'SC7: ...'（或开机横幅）。绝不写端口。"""
    try:
        s = serial.Serial(port, baud, timeout=0.15, bytesize=8,
                          parity="N", stopbits=1)
    except Exception:
        return False
    try:
        t0 = time.time()
        buf = ""
        while time.time() - t0 < secs:
            c = s.read(256)
            if not c:
                continue
            buf += c.decode("utf-8", "replace")
            if "SC7:" in buf or "STM32 4G LED Controller" in buf or "V4G(PA3)" in buf:
                return True
    except Exception:
        return False
    finally:
        s.close()
    return False


def autodetect_board(exclude):
    for c in list_all_ports():
        if c["port"] == exclude:
            continue
        if probe_board(c["port"]):
            log("  主板 PC 串口命中: %s  (%s)" % (c["port"], c["desc"]))
            return c["port"]
    return None


# ------------------------------------------------------------------- 线程 ---
class Meter(threading.Thread):
    """F-11 采样线程：每个数据帧一行 CSV（带 wall-clock）。"""

    daemon = True

    def __init__(self, port, csv_path, t0, baud=38400):
        super(Meter, self).__init__()
        self.port = port
        self.baud = baud
        self.csv_path = csv_path
        self.t0 = t0
        self.stop_evt = threading.Event()
        self.n = 0
        self.err = None

    def run(self):
        try:
            s = serial.Serial(self.port, self.baud, timeout=0.15,
                              bytesize=8, parity="N", stopbits=1)
        except Exception as e:
            self.err = str(e)
            return
        try:
            s.reset_input_buffer()
            buf = bytearray()
            with open(self.csv_path, "w", encoding="utf-8", newline="") as f:
                f.write("t_s,wall_iso,current_uA,current_mA,voltage_mV,voltage_V\n")
                while not self.stop_evt.is_set():
                    try:
                        c = s.read(512)
                    except Exception as e:
                        self.err = str(e)
                        break
                    if not c:
                        continue
                    buf += c
                    while b"\n" in buf:
                        line, _, rest = buf.partition(b"\n")
                        buf = bytearray(rest)
                        line = line.strip(b"\r ")
                        if not line:
                            continue
                        w = time.time()
                        ua = mv = None
                        m = F11_LINE.match(line)
                        if m:
                            ua, mv = int(m.group(1)), int(m.group(2))
                        else:
                            m1 = F11_ONE.match(line)
                            if m1:
                                ua = int(m1.group(1))
                        if ua is None:
                            continue
                        self.n += 1
                        f.write("%.4f,%s,%d,%.4f,%s,%s\n" % (
                            w - self.t0,
                            datetime.datetime.fromtimestamp(w).isoformat(timespec="milliseconds"),
                            ua, ua / 1000.0,
                            "" if mv is None else mv,
                            "" if mv is None else "%.4f" % (mv / 1000.0)))
                        if self.n % 25 == 0:      # 每 ~2.5s 落盘一次，中途挂掉也不丢数据
                            f.flush()
                f.flush()
        finally:
            try:
                s.close()
            except Exception:
                pass


class Board(threading.Thread):
    """主板 PC 串口读取线程：全部落日志，SEQ 行额外上屏。"""

    daemon = True

    def __init__(self, port, baud, log_path, t0, verbose=False):
        super(Board, self).__init__()
        self.port = port
        self.baud = baud
        self.log_path = log_path
        self.t0 = t0
        self.verbose = verbose
        self.stop_evt = threading.Event()
        self.lines = []          # (wall, text)
        self.lock = threading.Lock()
        self.done_evt = threading.Event()
        self.err = None
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.15,
                                     bytesize=8, parity="N", stopbits=1)
        except Exception as e:
            self.err = str(e)
            return
        try:
            buf = bytearray()
            with open(self.log_path, "w", encoding="utf-8", newline="") as f:
                while not self.stop_evt.is_set():
                    try:
                        c = self.ser.read(512)
                    except Exception as e:
                        self.err = str(e)
                        break
                    if not c:
                        continue
                    buf += c
                    while b"\n" in buf:
                        line, _, rest = buf.partition(b"\n")
                        buf = bytearray(rest)
                        s = line.decode("utf-8", "replace").strip()
                        if not s:
                            continue
                        w = time.time()
                        with self.lock:
                            self.lines.append((w, s))
                        f.write("%.3f\t%s\n" % (w, s))
                        f.flush()
                        if s.startswith("SEQ") or self.verbose:
                            log("BOARD| %s" % s)
                        if SEQ_DONE.match(s):
                            self.done_evt.set()
                f.flush()
        finally:
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass

    def send(self, text):
        if not self.ser:
            return False
        self.ser.write(text.encode("ascii", "replace"))
        self.ser.flush()
        return True


# -------------------------------------------------------------------- 主流程 ---
def main():
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument("--list", action="store_true", help="列串口后退出")
    ap.add_argument("--cmd", default=None, help="发给板子的 '!' 命令，如 'seq 60'")
    ap.add_argument("--board-port", default=None)
    ap.add_argument("--board-baud", type=int, default=115200)
    ap.add_argument("--f11-port", default=None)
    ap.add_argument("--lead", type=float, default=3.0)
    ap.add_argument("--tail", type=float, default=3.0)
    ap.add_argument("--max", type=float, default=2400.0, dest="max_s")
    ap.add_argument("--out", default=None)
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()

    if a.list or not a.cmd:
        log("=== 串口列表 ===")
        for c in list_all_ports():
            log("  %-8s  %s" % (c["port"], c["desc"]))
        log("--- F-11 自动识别（逐口试 38400）---")
        f11 = autodetect_f11()
        log("  F-11 = %s" % (f11 or "未找到"))
        log("--- 主板 PC 串口自动识别（逐口只读试 115200）---")
        b = autodetect_board(exclude=f11)
        log("  主板 = %s" % (b or "未找到"))
        if not a.cmd:
            log("\n未给 --cmd，仅列出串口。示例： --cmd \"seq 60\"")
            return 0

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "data", "bench_%s" % stamp)
    d = os.path.dirname(os.path.abspath(out))
    if d and not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)

    f11_csv = out + "_f11.csv"
    board_log = out + "_board.log"
    markers_csv = out + "_markers.csv"
    barrier = out + "_barrier.txt"

    log("=== bench_run ===")
    log("  cmd   : !%s" % a.cmd)
    log("  输出  : %s_*" % out)

    f11_port = a.f11_port or autodetect_f11()
    if not f11_port:
        log("!! 未找到 F-11，请用 --f11-port COMx 指定")
        return 2
    board_port = a.board_port or autodetect_board(exclude=f11_port)
    if not board_port:
        log("!! 未找到主板 PC 串口，请用 --board-port COMx 指定")
        return 2
    if board_port == f11_port:
        log("!! 主板口与 F-11 口相同，请显式指定")
        return 2

    t0 = time.time()
    meter = Meter(f11_port, f11_csv, t0)
    board = Board(board_port, a.board_baud, board_log, t0, a.verbose)
    meter.start()
    board.start()

    time.sleep(0.5)
    if meter.err:
        log("!! F-11 打开失败：%s" % meter.err)
        return 2
    if board.err:
        log("!! 主板串口打开失败：%s" % board.err)
        return 2

    log("  前置基线 %.1fs …" % a.lead)
    time.sleep(max(0.0, a.lead))

    t_send = time.time()
    log(">>> 发送命令  !%s" % a.cmd)
    if not board.send("!" + a.cmd + "\r"):
        log("!! 命令发送失败")
    with open(barrier, "w", encoding="utf-8") as f:
        f.write("cmd        = !%s\n" % a.cmd)
        f.write("t0_epoch   = %.3f\n" % t0)
        f.write("send_epoch = %.3f\n" % t_send)
        f.write("send_t_rel = %.3f\n" % (t_send - t0))
        f.write("f11_port   = %s\n" % f11_port)
        f.write("board_port = %s\n" % board_port)

    # 等待 DONE 或硬超时
    deadline = time.time() + a.max_s
    while time.time() < deadline:
        if board.done_evt.wait(0.5):
            log("<<< 收到 SEQ,DONE")
            break
        if board.err or meter.err:
            log("!! 串口中断：board=%s meter=%s" % (board.err, meter.err))
            break
    else:
        log("<<< 到达 --max %.0fs，主动停止" % a.max_s)

    time.sleep(max(0.0, a.tail))
    meter.stop_evt.set()
    board.stop_evt.set()
    time.sleep(0.6)

    t_end = time.time()
    with open(barrier, "a", encoding="utf-8") as f:
        f.write("end_epoch  = %.3f\n" % t_end)
        f.write("end_t_rel  = %.3f\n" % (t_end - t0))
        f.write("meter_rows = %d\n" % meter.n)
        f.write("board_rows = %d\n" % len(board.lines))

    # 抽 SEQ 标记
    n_mark = 0
    with open(markers_csv, "w", encoding="utf-8", newline="") as f:
        f.write("index,name,dwell_s,wall_epoch,t_rel_s,board_tick_ms\n")
        for w, s in board.lines:
            m = SEQ_MARK.match(s)
            if m:
                f.write("%s,%s,%s,%.3f,%.3f,%s\n" %
                        (m.group(1), m.group(2), m.group(3), w, w - t0, m.group(4)))
                n_mark += 1
        f.write("DONE,,0,%.3f,%.3f,0\n" % (t_end, t_end - t0))

    log("")
    log("=== 汇总 ===")
    log("  F-11 采样 : %d 行 -> %s" % (meter.n, f11_csv))
    log("  板子日志  : %d 行 -> %s" % (len(board.lines), board_log))
    log("  SEQ 标记  : %d 个 -> %s" % (n_mark, markers_csv))
    log("  屏障说明  : %s" % barrier)
    if meter.err:
        log("  !! F-11 异常: %s" % meter.err)
    if board.err:
        log("  !! 板子串口异常: %s" % board.err)
    log("")
    log("下一步出图：python bench_plot.py --f11 %s --markers %s" % (f11_csv, markers_csv))
    return 0


if __name__ == "__main__":
    sys.exit(main())
