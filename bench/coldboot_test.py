# -*- coding: utf-8 -*-
"""连续冷启动可靠性测试
流程（循环 N 次）：继电器断电 10 s → 上电 → F-11 连续采 150 s
判据：采样窗内出现 >40 mA 的 4G 在线特征电流（峰值法），且 MQTT 日志出现新 ready
结果：bench/data/coldboot_<stamp>.csv + 控制台汇总
"""
import subprocess, sys, time, csv, os, datetime, re

PY_A = r"E:/ProgramData/Anaconda3/python.exe"
RELAY = r"C:/Users/Administrator/.workbuddy/skills/usb-relay/scripts/relay.py"
F11 = r"C:/Users/Administrator/.workbuddy/skills/afu-f11-meter/scripts/f11.py"
MQTT_LOG = r"bench/data/mqtt_live.log"
OUTDIR = r"bench/data"
CYCLES = 8
OFF_S = 10
WATCH_S = 150

def sh(cmd, timeout=60):
    return subprocess.run(cmd, capture_output=True, text=True,
                          encoding="utf-8", errors="replace", timeout=timeout)

def relay(action):
    r = sh([PY_A, RELAY, action, "--port", "COM12", "--addr", "1"], timeout=30)
    return "ok" in (r.stdout + r.stderr).lower()

def mqtt_ready_count():
    try:
        with open(MQTT_LOG, encoding="utf-8", errors="replace") as f:
            return sum(1 for ln in f if '"ready"' in ln or "ready" in ln.split(",")[-1])
    except OSError:
        return 0

def last_ready_line():
    try:
        with open(MQTT_LOG, encoding="utf-8", errors="replace") as f:
            lines = [ln for ln in f if "ready" in ln]
            return lines[-1].strip() if lines else ""
    except OSError:
        return ""

def main():
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_csv = os.path.join(OUTDIR, "coldboot_%s.csv" % stamp)
    logf = open(out_csv, "w", newline="", encoding="utf-8")
    w = csv.writer(logf)
    w.writerow(["cycle", "t_s", "current_mA"])

    base_ready = mqtt_ready_count()
    print("基准 ready 计数=%d，最后一条: %s" % (base_ready, last_ready_line()[:120]), flush=True)
    results = []
    t0 = time.time()

    for c in range(1, CYCLES + 1):
        print("\n===== 第 %d/%d 次冷启 =====" % (c, CYCLES), flush=True)
        if not relay("off"):
            print("  [warn] relay off 反馈异常，继续", flush=True)
        time.sleep(OFF_S)
        if not relay("on"):
            print("  [warn] relay on 反馈异常，继续", flush=True)
        t_on = time.time()
        pre_ready = mqtt_ready_count()

        f11csv = os.path.join(OUTDIR, "coldboot_%s_c%02d_f11.csv" % (stamp, c))
        r = sh([PY_A, F11, "stream", "--csv", f11csv, "--secs", str(WATCH_S)],
               timeout=WATCH_S + 90)
        rows = []
        try:
            with open(f11csv, newline="", encoding="utf-8") as f:
                for x in csv.DictReader(f):
                    try:
                        rows.append(float(x["current_mA"]))
                    except (KeyError, ValueError):
                        pass
        except OSError:
            pass

        peak = max(rows) if rows else 0.0
        # 4G 上线特征：窗内出现 >40mA（搜网/注网/在线），或至少 >15 mA 维持（在线空闲）
        n_hi = sum(1 for v in rows if v > 40.0)
        online_sig = peak > 40.0
        new_ready = mqtt_ready_count() > pre_ready
        verdict = "PASS" if (online_sig and new_ready) else ("PASS(电流)" if online_sig else "FAIL")
        results.append((c, peak, n_hi, online_sig, new_ready, verdict))
        print("  峰值 %.1f mA | >40mA 样本 %d | 电流特征 %s | 新 ready %s => %s"
              % (peak, n_hi, "有" if online_sig else "无", "有" if new_ready else "无", verdict), flush=True)
        for v in rows:
            w.writerow([c, "%.1f" % (time.time() - t_on), "%.3f" % v])
        logf.flush()

    logf.close()
    print("\n========== 汇总 ==========", flush=True)
    fails = 0
    for c, peak, n_hi, sig, nr, v in results:
        if v.startswith("FAIL"):
            fails += 1
        print("  冷启#%d: %s  峰值 %.1f mA" % (c, v, peak))
    print("共 %d 次，FAIL %d 次，总耗时 %.1f min"
          % (CYCLES, fails, (time.time() - t0) / 60.0), flush=True)

if __name__ == "__main__":
    main()
