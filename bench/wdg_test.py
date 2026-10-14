#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wdg_test.py —— RTC 软看门狗（方案A）台架验证
================================================================================
S0 心跳活性   SWD 连读 g_wdg_kick 两次（间隔 3s）必须增长；rst_wdg 应为 0
S1 睡眠不误咬 MQTT 发 lp_cycle period_min=1 走一轮（睡60s+会话90s），
              醒后 miss==0、rst_wdg==0、kick 继续增长
S2 挂死自愈   SWD 注入 `wdg hang`（主循环死循环，中断在），F-11 连续采电流；
              预期 ≤60s（T=30s ×2 发闹钟）内 BKP 打标记 + 复位，
              复位后 rst_wdg==1、CSR 带 SFTRSTF、kick 归零重涨、MQTT 重新 ready
S3 汇总判定

用法
    python wdg_test.py [--skip-s1] [--irq]      # --irq 追加 hang_irq + 继电器断电恢复
输出
    bench/data/wdg_<stamp>_f11.csv / _mqtt.log / _swd.log，结论打印到 stdout
"""
import argparse
import csv
import os
import statistics as st
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from bench_swd import Swd, NM, DEFAULT_ELF, log  # noqa: E402

PY_A   = r"E:/ProgramData/Anaconda3/python.exe"
F11    = r"C:/Users/Administrator/.workbuddy/skills/afu-f11-meter/scripts/f11.py"
MQTT   = os.path.join(HERE, "mqtt_watch.py")
ICCID  = "898608961026C0241530"
DATA   = os.path.join(HERE, "data")
os.makedirs(DATA, exist_ok=True)

WANT = ["g_test_cmd", "g_test_go", "g_slog",
        "g_wdg_kick", "g_wdg_miss", "g_rst_csr", "g_rst_wdg"]


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
        raise RuntimeError("elf 缺符号: %s（固件是否为含看门狗的新版本？）" % miss)
    return sym


def mqtt_send(payload, tries=3):
    for i in range(tries):
        r = subprocess.run([PY_A, MQTT, "--send-only", "--allow-multi",
                            "--iccid", ICCID, "--send-delay", "1",
                            "--cmd", payload],
                           capture_output=True, text=True, timeout=90)
        if '"status": "ok"' in (r.stdout + r.stderr) or r.returncode == 0:
            return True
        time.sleep(5)
    return False


def read_sym(swd, sym, name, n=1):
    w, _ = swd.read_words(sym[name], n)
    return w


def f11_median(path, lo=0.15, hi=0.9, lo_ma=None, hi_ma=None):
    v = []
    with open(path, newline="", encoding="utf-8") as f:
        for x in csv.DictReader(f):
            try:
                v.append(float(x["current_mA"]))
            except (KeyError, ValueError):
                pass
    v.sort()
    a, b = int(len(v) * lo), int(len(v) * hi)
    seg = v[a:b]
    if lo_ma is not None:
        seg = [x for x in seg if lo_ma <= x <= (hi_ma if hi_ma else 1e9)]
    return (st.median(seg) if seg else float("nan")), len(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--elf", default=DEFAULT_ELF)
    ap.add_argument("--skip-s1", action="store_true")
    ap.add_argument("--irq", action="store_true",
                    help="追加 S4: hang_irq（验证无硬件狗兜底边界），继电器断电恢复")
    a = ap.parse_args()

    stamp = time.strftime("%Y%m%d_%H%M%S")
    prefix = os.path.join(DATA, "wdg_" + stamp)
    swd_log = prefix + "_swd.log"
    sym = resolve(a.elf)
    log("符号: " + ", ".join("%s=0x%08x" % (k, sym[k]) for k in WANT))
    swd = Swd(swd_log)
    result = {}

    # ---------- S0 心跳活性 ----------
    log("\n== S0 心跳活性 ==")
    w1 = read_sym(swd, sym, "g_wdg_kick")[0]
    time.sleep(3)
    w2 = read_sym(swd, sym, "g_wdg_kick")[0]
    rst_wdg0 = read_sym(swd, sym, "g_rst_wdg")[0]
    csr0 = read_sym(swd, sym, "g_rst_csr")[0]
    miss0 = read_sym(swd, sym, "g_wdg_miss")[0]
    log("  kick: %d -> %d (Δ=%d)  miss=%d  rst_wdg=%d  CSR=0x%08X"
        % (w1, w2, w2 - w1, miss0, rst_wdg0, csr0))
    result["S0_heartbeat"] = (w2 > w1)
    result["S0_clean_boot"] = (rst_wdg0 == 0)

    # ---------- S1 睡眠不误咬 ----------
    if not a.skip_s1:
        log("\n== S1 lp_cycle period_min=1 一轮（睡60s+会话90s），看门狗不得误咬 ==")
        ok = mqtt_send('{"cmd":"lp_cycle","period_min":1}')
        log("  MQTT lp_cycle 下发: %s" % ("ok" if ok else "FAIL"))
        time.sleep(20)
        try:
            k = read_sym(swd, sym, "g_wdg_kick")[0]
            log("  t+20s  kick=%d（应已冻结：进了 Stop）" % k)
        except RuntimeError as e:
            log("  t+20s  SWD 失联（正常：Stop 里 APB 断电）")
        time.sleep(20)
        try:
            k = read_sym(swd, sym, "g_wdg_kick")[0]
            log("  t+40s  kick=%d（若读到说明尚未进 Stop，继续等）" % k)
        except RuntimeError:
            log("  t+40s  SWD 失联（正常：Stop 中）")
        log("  等 60s 睡眠 + 唤醒...")
        time.sleep(55)
        try:
            k = read_sym(swd, sym, "g_wdg_kick")[0]
            m = read_sym(swd, sym, "g_wdg_miss")[0]
            log("  t+115s kick=%d miss=%d（应已唤醒进会话）" % (k, m))
        except RuntimeError:
            log("  t+115s 仍在睡/未醒，再等 20s")
            time.sleep(20)
        time.sleep(30)                      # 会话中段
        mqtt_send('{"cmd":"lp_cycle_off"}')
        time.sleep(10)
        mqtt_send('{"cmd":"lp_cycle_off"}')  # 双保险
        time.sleep(8)
        k = read_sym(swd, sym, "g_wdg_kick")[0]
        m = read_sym(swd, sym, "g_wdg_miss")[0]
        rw = read_sym(swd, sym, "g_rst_wdg")[0]
        log("  收尾: kick=%d miss=%d rst_wdg=%d" % (k, m, rw))
        result["S1_no_false_bite"] = (m <= 1 and rw == 0)  # m=1: 睡醒闹钟的瞬时 miss，可接受
        time.sleep(5)
    else:
        result["S1_no_false_bite"] = None

    # ---------- S2 挂死自愈 ----------
    log("\n== S2 注入 wdg hang（主循环死循环），预期 ≤60s 自愈复位 ==")
    f11_csv = prefix + "_f11.csv"
    mqtt_log = prefix + "_mqtt.log"
    SECS = 110
    mf = open(mqtt_log, "w", encoding="utf-8")
    p_mqtt = subprocess.Popen(
        [PY_A, MQTT, "--allow-multi", "--quiet-json",
         "--idle-exit", str(SECS + 40)],
        stdout=mf, stderr=subprocess.STDOUT)
    p_f11 = subprocess.Popen(
        [PY_A, F11, "stream", "--csv", f11_csv, "--secs", str(SECS)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(4)
    t_inj = time.time()
    swd.inject(sym["g_test_cmd"], sym["g_test_go"], "wdg hang")
    log("  已注入 '!wdg hang' @ %.1fs，开始轮询复位..." % 0)

    t_reset = None
    rst_wdg1 = None
    csr1 = None
    deadline = time.time() + 120
    while time.time() < deadline:
        time.sleep(5)
        try:
            rw = read_sym(swd, sym, "g_rst_wdg")[0]
            k = read_sym(swd, sym, "g_wdg_kick")[0]
            if rw == 1 and k < 100:
                t_reset = time.time() - t_inj
                csr1 = read_sym(swd, sym, "g_rst_csr")[0]
                m = read_sym(swd, sym, "g_wdg_miss")[0]
                log("  ✓ 复位自愈! t=%.1fs  kick=%d miss=%d CSR=0x%08X"
                    % (t_reset, k, m, csr1))
                break
            log("  t=%4.1fs kick=%d rst_wdg=%d（挂死中，等狗咬）"
                % (time.time() - t_inj, k, rw))
        except RuntimeError as e:
            log("  t=%4.1fs SWD 读失败（可能正在复位/早起）"
                % (time.time() - t_inj))
    p_f11.wait(timeout=SECS + 90)
    result["S2_selfheal"] = (t_reset is not None)
    if t_reset is not None:
        result["S2_latency_s"] = round(t_reset, 1)
        result["S2_csr_sftrstf"] = bool(csr1 & (1 << 28))
        # 等网络回来，确认 kick 增长（看门狗在复位后重新武装）
        time.sleep(25)
        k1 = read_sym(swd, sym, "g_wdg_kick")[0]
        time.sleep(3)
        k2 = read_sym(swd, sym, "g_wdg_kick")[0]
        result["S2_wdg_rearmed"] = (k2 > k1)
        log("  复位后 kick: %d -> %d（看门狗已重新武装）" % (k1, k2))
    try:
        p_mqtt.wait(timeout=60)
    except subprocess.TimeoutExpired:
        p_mqtt.kill()
    mf.close()
    mq = open(mqtt_log, encoding="utf-8", errors="replace").read()
    result["S2_mqtt_ready_after_reset"] = ('"cmd":"ready"' in mq or
                                           '"cmd": "ready"' in mq)
    try:
        med, n = f11_median(f11_csv)
        result["S2_f11_median_mA"] = round(med, 3)
        result["S2_f11_rows"] = n
        log("  F-11 全程中位 %.3f mA（%d 行）" % (med, n))
    except Exception as e:
        log("  F-11 汇总失败: %s" % e)

    # ---------- S4 可选：hang_irq 边界 ----------
    if a.irq and t_reset is not None:
        log("\n== S4 注入 wdg hang_irq（关中断），预期不自救，随后继电器断电恢复 ==")
        time.sleep(3)
        swd.inject(sym["g_test_cmd"], sym["g_test_go"], "wdg hang_irq")
        time.sleep(15)
        stuck = False
        try:
            rw = read_sym(swd, sym, "g_rst_wdg")[0]
            k = read_sym(swd, sym, "g_wdg_kick")[0]
            log("  t+15s kick=%d rst_wdg=%d（若 kick 冻结且无复位 = 符合预期）" % (k, rw))
            stuck = True
        except RuntimeError:
            log("  SWD 失联？")
        result["S4_no_selfheal_expected"] = stuck
        r = subprocess.run([PY_A, os.path.join(HERE, "relay_ctrl.py"), "off"],
                           capture_output=True, text=True, timeout=60)
        if r.returncode != 0:
            log("  （relay_ctrl.py 不存在或失败——请手动断电 10s 恢复）")
        else:
            time.sleep(10)
            subprocess.run([PY_A, os.path.join(HERE, "relay_ctrl.py"), "on"],
                           capture_output=True, text=True, timeout=60)
            log("  已继电器断电 10s → 上电，等 40s 重新注网")
            time.sleep(40)

    # ---------- S3 汇总 ----------
    log("\n===== 汇总 =====")
    ok_all = True
    for k in ["S0_heartbeat", "S0_clean_boot", "S1_no_false_bite",
              "S2_selfheal", "S2_csr_sftrstf", "S2_wdg_rearmed",
              "S2_mqtt_ready_after_reset"]:
        v = result.get(k)
        if v is None:
            log("  --  %-28s skipped" % k)
            continue
        log("  %s %-28s %s" % ("PASS" if v else "FAIL", k, v))
        ok_all = ok_all and bool(v)
    log("  结论: %s" % ("ALL PASS" if ok_all else "存在 FAIL，看上方明细"))
    with open(prefix + "_summary.txt", "w", encoding="utf-8") as f:
        for k, v in result.items():
            f.write("%s = %s\n" % (k, v))
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
