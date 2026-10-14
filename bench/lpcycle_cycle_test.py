#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lpcycle_cycle_test.py —— lp_cycle「睡 → 自主醒 → 跑 → 再睡」循环验证
================================================================================
目的：下发 {"cmd":"lp_cycle","period_min":N} 后，验证板子能不能自己循环：

  1) 会话跑完 → 收尾 → 进 Stop（电流 ≈0.2 mA；此时 SWD 也读不到，属正常）
  2) RTC 闹钟到点 → **无任何外部干预**自主唤醒（SLEEPDEEP 睡眠 + EXTI17）
  3) 醒来重开 4G 会话并上报 MQTT（"wake":"rtc"）
  4) 如此往复

观测三路：
  F-11 电流   —— 分「睡段(<1mA) / 醒段(>3mA)」，看振荡周期是否 = period + 会话
  SWD 轮询    —— s_lp_state(0idle/1session/2closing/3sleeping)；睡态读失败也算证据
  MQTT 旁听   —— 每次唤醒应出现一条 {"cmd":"report","wake":"rtc",...}

用法
    python lpcycle_cycle_test.py --period 2 --cycles 3
    python lpcycle_cycle_test.py --period 2 --cycles 3 --skip-plot
输出
    data/lpcyc_<stamp>_f11.csv / _mqtt.log / _swd.log / _markers.csv
    data/lpcyc_<stamp>_summary.txt
    data/lpcyc_<stamp>_annotated.html（bench_plot.py 出图）
"""
import argparse
import csv
import datetime
import os
import statistics as st
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from bench_swd import Swd, NM, DEFAULT_ELF, log  # noqa: E402

PY_A = r"E:/ProgramData/Anaconda3/python.exe"
F11 = r"C:/Users/Administrator/.workbuddy/skills/afu-f11-meter/scripts/f11.py"
MQTT = os.path.join(HERE, "mqtt_watch.py")
ICCID = "898608961026C0241530"
DATA = os.path.join(HERE, "data")
os.makedirs(DATA, exist_ok=True)

WANT = ["s_lp_state", "g_lp_cycle", "g_lp_period_s",
        "g_rtc_alarm_hit", "g_wdg_kick", "g_wdg_miss", "g_rst_wdg", "g_dtu_valid_rx_ms"]
# 变量宽度（字节）。uint8_t 的符号地址常常不 4 字节对齐，mdw 只能按对齐字读，
# 必须按偏移从对齐字里抠出对应字节，否则读到的是相邻变量打包值（曾把 state 读成
# 0x30380001 这种垃圾）。
WIDTH = {"s_lp_state": 1, "g_lp_cycle": 1, "g_lp_period_s": 4, "g_rtc_alarm_hit": 1,
         "g_wdg_kick": 4, "g_wdg_miss": 4, "g_rst_wdg": 1, "g_dtu_valid_rx_ms": 4}
MASK = {1: 0xFF, 2: 0xFFFF, 4: 0xFFFFFFFF}
STATE_TXT = {0: "idle", 1: "session", 2: "closing", 3: "SLEEP(Stop)"}
SLEEP_MA = 1.0     # < 此值 判为 Stop 睡段
AWAKE_MA = 3.0     # > 此值 判为清醒段


def resolve(elf):
    p = subprocess.run([NM, "--numeric-sort", "--defined-only", elf],
                       capture_output=True, text=True, timeout=60)
    sym = {}
    for line in p.stdout.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] in WANT:
            sym[parts[2]] = int(parts[0], 16)
    miss = [k for k in WANT if k not in sym]
    if miss:
        raise RuntimeError("elf 缺符号: %s" % miss)
    return sym


def mqtt_send(payload, tries=3):
    for _ in range(tries):
        r = subprocess.run([PY_A, MQTT, "--send-only", "--allow-multi",
                            "--iccid", ICCID, "--send-delay", "1", "--cmd", payload],
                           capture_output=True, text=True, timeout=90)
        if r.returncode == 0 or '"status": "ok"' in (r.stdout + r.stderr):
            return True
        time.sleep(5)
    return False


def read_many(swd, sym, names):
    """一次 OpenOCD 会话读多个符号（按 4 字节对齐块去重后读，再按字节偏移抠值）。
       睡态（Stop 里 APB 断电）会读不到 → 返回空 dict。"""
    al = sorted({sym[n] & ~3 for n in names})
    cmds = ["mdw 0x%08x 1" % a for a in al]
    out, t0, t1 = swd.run(cmds, timeout=25)
    words = {}
    for a, w in Swd.parse_mdw(out):
        if a in words or a in al:
            words[a] = w[0]
    vals = {}
    for n in names:
        a, off = sym[n] & ~3, sym[n] & 3
        if a in words:
            vals[n] = (words[a] >> (8 * off)) & MASK[WIDTH[n]]
    return vals, (t0 + t1) / 2.0


# ---------------------------------------------------------------- F-11 分段 ---
def load_f11(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for d in csv.DictReader(f):
            try:
                rows.append((float(d["t_s"]), float(d["current_mA"])))
            except (KeyError, TypeError, ValueError):
                pass
    rows.sort(key=lambda r: r[0])
    return rows


def split_segments(rows, min_dur=3.0):
    """按电流阈值切「睡/醒」段。返回 [{kind,t0,t1,dur,med,min,max,n}]。"""
    if not rows:
        return []
    def kind(c):
        return "SLEEP" if c < SLEEP_MA else ("AWAKE" if c > AWAKE_MA else "MID")
    segs, cur = [], None
    for t, c in rows:
        k = kind(c)
        if k == "MID" and cur is not None:
            k = cur["kind"]          # 过渡样本跟随当前段
        if cur is None or k != cur["kind"]:
            if cur is not None:
                segs.append(cur)
            cur = {"kind": k, "t0": t, "vals": []}
        cur["vals"].append((t, c))
    if cur is not None:
        segs.append(cur)
    out = []
    for s in segs:
        ts = [v[0] for v in s["vals"]]
        cs = [v[1] for v in s["vals"]]
        dur = ts[-1] - ts[0]
        if dur < min_dur:
            continue
        out.append({"kind": s["kind"], "t0": ts[0], "t1": ts[-1], "dur": dur,
                    "med": st.median(cs), "mn": min(cs), "mx": max(cs), "n": len(cs)})
    return out


def write_markers(path, segs):
    """转成 bench_plot.py 认的 markers.csv（index,name,dwell_s,t_rel_s）。"""
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["index", "name", "dwell_s", "t_rel_s"])
        for i, s in enumerate(segs):
            w.writerow([i, s["kind"], int(round(s["dur"])), "%.2f" % s["t0"]])
        w.writerow(["", "", "", "%.2f" % (segs[-1]["t1"] if segs else 0.0)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--period", type=int, default=2, help="睡眠分钟数 (period_min)")
    ap.add_argument("--cycles", type=int, default=3, help="观察几个完整循环")
    ap.add_argument("--poll", type=float, default=15.0, help="SWD 轮询间隔秒")
    ap.add_argument("--elf", default=DEFAULT_ELF)
    ap.add_argument("--port", default=None, help="F-11 串口，缺省自动探测")
    ap.add_argument("--skip-plot", action="store_true")
    a = ap.parse_args()

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = os.path.join(DATA, "lpcyc_" + stamp)
    f11_csv = prefix + "_f11.csv"
    mqtt_log = prefix + "_mqtt.log"
    swd_log = prefix + "_swd.log"
    mk_csv = prefix + "_markers.csv"

    sym = resolve(a.elf)
    swd = Swd(swd_log)

    # 时长预算：先导 + cycles×(睡眠 + 会话90s + 收尾) + 尾巴
    per_cycle = a.period * 60 + 120
    total = 20 + a.cycles * per_cycle + 30
    log("=== lp_cycle 循环验证：period_min=%d（睡 %ds） cycles=%d  预计 %.1f min ==="
        % (a.period, a.period * 60, a.cycles, total / 60.0))
    log("符号: " + ", ".join("%s=0x%x" % (k, sym[k]) for k in WANT))

    # 落回普通在线（避免上一轮残留）
    mqtt_send('{"cmd":"lp_cycle_off"}')
    time.sleep(4)

    mf = open(mqtt_log, "w", encoding="utf-8")
    p_mqtt = subprocess.Popen([PY_A, MQTT, "--allow-multi", "--quiet-json",
                               "--idle-exit", str(total + 30)],
                              stdout=mf, stderr=subprocess.STDOUT)
    p_f11 = subprocess.Popen([PY_A, F11, "stream", "--csv", f11_csv, "--secs", str(total)],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(8)

    log("\n== 起始状态 ==")
    v0, _ = read_many(swd, sym, WANT)
    log("  " + (" ".join("%s=%s" % (k, STATE_TXT.get(v, v) if k == "s_lp_state" else v)
                         for k, v in v0.items()) if v0 else "SWD 读失败"))

    log("\n== 下发 lp_cycle period_min=%d ==" % a.period)
    ok = mqtt_send('{"cmd":"lp_cycle","period_min":%d}' % a.period)
    log("  %s" % ("OK" if ok else "FAIL"))
    t_start = time.time()

    # ---- 轮询 ----
    trace = []
    last = None
    while time.time() - t_start < total - 30:
        time.sleep(a.poll)
        el = time.time() - t_start
        try:
            vals, _ = read_many(swd, sym, WANT)
            lost = not vals
        except Exception as e:                       # noqa: BLE001
            vals, lost = {}, True
        if lost:
            row = {"t": el, "lost": True, "state": None, "kick": None}
            line = "  t=%6.1fs  SWD 失联（Stop 中 / 或正在收尾）" % el
        else:
            stt = vals.get("s_lp_state")
            kick = vals.get("g_wdg_kick")
            row = {"t": el, "lost": False, "state": stt, "kick": kick,
                   "cycle": vals.get("g_lp_cycle"), "alarm": vals.get("g_rtc_alarm_hit"),
                   "rst": vals.get("g_rst_wdg")}
            line = ("  t=%6.1fs  state=%-12s lpc=%s kick=%s alarm=%s rst=%s"
                    % (el, STATE_TXT.get(stt, stt), row.get("cycle"), kick,
                       row.get("alarm"), row.get("rst")))
        trace.append(row)
        if row != last:
            log(line)
            last = row

    log("\n== 停止采集 ==")
    p_f11.wait(timeout=total + 90)
    try:
        p_mqtt.wait(timeout=60)
    except subprocess.TimeoutExpired:
        p_mqtt.kill()
    mf.close()

    # ---- 分析 ----
    rows = load_f11(f11_csv)
    segs = split_segments(rows)
    if segs:
        write_markers(mk_csv, segs)

    log("\n===== F-11 电流分段（阈值 <%.1f mA=睡 / >%.1f mA=醒）=====" % (SLEEP_MA, AWAKE_MA))
    log("  %-6s %8s %8s %8s %10s %10s" % ("kind", "t0(s)", "dur(s)", "n", "med(mA)", "min/max"))
    sleeps, awakes = [], []
    for s in segs:
        log("  %-6s %8.1f %8.1f %8d %10.3f %5.2f/%-6.2f"
            % (s["kind"], s["t0"], s["dur"], s["n"], s["med"], s["mn"], s["mx"]))
        (sleeps if s["kind"] == "SLEEP" else awakes).append(s)

    # MQTT 唤醒上报
    mtxt = open(mqtt_log, encoding="utf-8", errors="replace").read()
    rep_rtc = mtxt.count('"wake":"rtc"') + mtxt.count('"wake": "rtc"')
    ready = mtxt.count('"cmd":"ready"') + mtxt.count('"cmd": "ready"')

    log("\n===== 结论 =====")
    res = {}
    log("  睡段数 = %d（观察 %d 个循环，预期 ≥%d）" % (len(sleeps), a.cycles, min(a.cycles, len(sleeps))))
    for i, s in enumerate(sleeps, 1):
        dev = s["dur"] - a.period * 60
        log("    #%d 睡 %5.1fs（目标 %ds，偏差 %+.1fs）  中位 %.3f mA"
            % (i, s["dur"], a.period * 60, dev, s["med"]))
    res["sleep_count"] = len(sleeps)
    res["sleep_median_s"] = round(st.median([s["dur"] for s in sleeps]), 1) if sleeps else None
    res["sleep_med_mA"] = round(st.median([s["med"] for s in sleeps]), 3) if sleeps else None
    res["sleep_timed_ok"] = bool(sleeps and all(
        abs(s["dur"] - a.period * 60) <= max(20, a.period * 60 * 0.25) for s in sleeps))

    wake_after_sleep = 0
    for s in sleeps:
        if any(x["t0"] >= s["t1"] - 1.0 for x in awakes):
            wake_after_sleep += 1
    log("  睡段之后自主醒来次数 = %d / %d" % (wake_after_sleep, len(sleeps)))
    res["auto_wake_after_sleep"] = wake_after_sleep

    log("  MQTT 中 \"wake\":\"rtc\" 上报 = %d 次；ready = %d 次" % (rep_rtc, ready))
    res["mqtt_rtc_reports"] = rep_rtc
    res["mqtt_ready"] = ready

    trace_lost = sum(1 for r in trace if r["lost"])
    log("  SWD 轮询：%d 次中 %d 次失联（睡态读不到属正常）" % (len(trace), trace_lost))
    res["swd_polls"] = len(trace)
    res["swd_lost"] = trace_lost

    rst_seen = any(r.get("rst") for r in trace)
    log("  全程 g_rst_wdg = %s（0 = 没有被看门狗咬过）"
        % ("1 ← 有复位!" if rst_seen else "0 ✓"))
    res["no_wdg_reset"] = (not rst_seen)

    # 最后一个睡段是采集结束时还没醒（正常），故只要求 min(cycles, 睡段数) 次醒来
    need_wake = min(a.cycles, len(sleeps))
    ok_all = (len(sleeps) >= 1 and wake_after_sleep >= need_wake
              and rep_rtc >= 1 and not rst_seen)
    log("  自主唤醒 %d 次（需 ≥%d）；MQTT 上报 %d 次（需 ≥1）"
        % (wake_after_sleep, need_wake, rep_rtc))
    log("  总判定: %s" % ("✅ 自主循环成立" if ok_all else "⚠ 见上方明细"))
    res["ALL_OK"] = ok_all

    with open(prefix + "_summary.txt", "w", encoding="utf-8") as f:
        for k, v in res.items():
            f.write("%s = %s\n" % (k, v))
    log("  数据: %s" % os.path.basename(f11_csv))

    if segs and not a.skip_plot:
        log("\n== 出图 ==")
        r = subprocess.run([PY_A, os.path.join(HERE, "bench_plot.py"),
                            "--f11", f11_csv, "--markers", mk_csv,
                            "--out", prefix + "_annotated.html",
                            "--title", "lp_cycle 自主唤醒循环 · period_min=%d（睡 %ds）"
                                       % (a.period, a.period * 60)],
                           capture_output=True, text=True, timeout=180)
        log("  " + (r.stdout or r.stderr or "")[-200:])
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
