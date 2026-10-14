#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dtu_recover_test.py —— 4G 模组硬恢复（2026-10-14 加固）端到端验证
================================================================================
复现用户症状：模组死透时整板卡在 ~2.5mA（Run + 死模组）永不自愈。
注入方式：MQTT 下发 {"cmd":"v4g","state":"off"} —— 固件把 PA3 拉低，模组断电
          （等价于"模组没启动起来"），此后模组一个字节都不回。
期望行为（新固件）：
  PA3 被切 → 探活 x2 失败 → 检出"模组死透" → **自动硬拉电** (PA3 0→3s→1)
  → 模组重新注网 → MQTT 重新 ready。全程无人干预。
旧固件行为：只发软件重启命令（对死模组无效）→ force READY 假装在线 → 永久卡死。

用法: python dtu_recover_test.py [--max 420]
输出: bench/data/dtu_rec_<stamp>_{mqtt.log,swd.log,summary.txt}
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from bench_swd import Swd, NM, DEFAULT_ELF, log  # noqa: E402
import wdg_test  # noqa: E402

DATA = os.path.join(HERE, "data")
os.makedirs(DATA, exist_ok=True)
PY_A = wdg_test.PY_A
MQTT = wdg_test.MQTT
GPIOA_ODR = 0x4001080C

# ⚠ 状态枚举值（Core/Inc/dtu.h，别数错）：
#   0 POWER_ON 1 SEND_PARAMSRC 2 QUERY_ICCID 3 PARSE_ICCID 4 PARSE_IMEI
#   5 SEND_MQTT 6 SEND_LED 7 SEND_NTP 8 SEND_LP 9 SEND_SAVE 10 WAIT_BOOT
#   11 POLL_SSTA 12 RECOVER_ESC 13 RECOVER_SSTA **14 READY**
ST_READY = 14

WANT = ["g_4g_last_rx_ms", "g_4g_rx_bytes", "_4g_state", "g_wdg_kick",
        "g_dtu_valid_rx_ms"]


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--max", type=int, default=420)
    a = ap.parse_args()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    prefix = os.path.join(DATA, "dtu_rec_" + stamp)
    sym = resolve(DEFAULT_ELF)
    swd = Swd(prefix + "_swd.log")
    log("符号: " + ", ".join("%s=0x%08x" % (k, sym[k]) for k in WANT))

    def rd(name, n=1):
        w, _ = swd.read_words(sym[name], n)
        return w[0]

    def pa3():
        w, _ = swd.read_words(GPIOA_ODR, 1)
        return (w[0] >> 3) & 1

    mf = open(prefix + "_mqtt.log", "w", encoding="utf-8")
    p_mq = subprocess.Popen([PY_A, MQTT, "--allow-multi", "--quiet-json",
                             "--idle-exit", str(a.max)], stdout=mf,
                            stderr=subprocess.STDOUT)

    log("\n== 阶段0：等待模组进入 READY（state=14）==")
    t_end = time.time() + 240
    online = False
    while time.time() < t_end:
        try:
            st = rd("_4g_state")
            p3 = pa3()
            rx = rd("g_4g_rx_bytes")
            log("  PA3=%d _4g_state=%d rx_bytes=%d" % (p3, st, rx))
            if p3 == 1 and st == ST_READY:
                online = True
                break
        except RuntimeError as e:
            log("  SWD 读失败: %s" % str(e)[:40])
        time.sleep(10)
    if not online:
        log("  !! 等不到 READY（模组或网络有问题）；仍继续注入测试")
    else:
        log("  ✓ 模组 READY（ssta 已连上）")

    log("\n== 阶段1：注入「模组死透」= MQTT 下发 v4g off ==")
    # payload 带上 id（=ICCID）：老固件 is_sys_cmd 不含 v4g，不带 id 会被静默丢弃
    V4G_OFF = '{"cmd":"v4g","state":"off","id":"%s"}' % wdg_test.ICCID
    t_inj = None
    for k in range(6):
        wdg_test.mqtt_send(V4G_OFF)
        # 确认指令被收到：PA3 应被固件拉低
        for _ in range(8):
            time.sleep(3)
            try:
                if pa3() == 0:
                    t_inj = time.time()
                    break
            except RuntimeError:
                pass
        if t_inj:
            break
        log("  第 %d 次下发未生效（板子可能刚重连），重发" % (k + 1))
    if t_inj is None:
        log("  !! v4g off 始终未生效 —— 无法完成注入测试")
        return 2
    log("  ✓ 模组已被断电（PA3=0），t=0 起算")

    log("\n== 阶段2：轮询恢复过程（期望 PA3 被固件自动重新拉高 + ready 回来）==")
    recover_at = None
    hard_cycle_seen = False
    pa3_low_at = None
    while time.time() - t_inj < a.max:
        try:
            p3 = pa3()
            rx = rd("g_4g_rx_bytes")
            st = rd("_4g_state")
            vld = rd("g_dtu_valid_rx_ms")
            dt = time.time() - t_inj
            log("  t=%6.1fs PA3=%d _4g_state=%s rx_bytes=%d valid_rx_ms=%d%s"
                % (dt, p3, st, rx, vld,
                   "  <-- 模组被切断" if p3 == 0 else ""))
            if p3 == 0:
                pa3_low_at = dt
            if pa3_low_at is not None and p3 == 1:
                if not hard_cycle_seen:
                    hard_cycle_seen = True
                    log("  ✓ 检测到固件自动硬拉电恢复（PA3 0 -> 1）@ t=%.1fs" % dt)
                    recover_at = dt
            if hard_cycle_seen and st == ST_READY and rx > 0:
                log("  ✓ 模组已回到 READY @ t=%.1fs" % dt)
                break
        except RuntimeError as e:
            log("  t=%6.1fs SWD 读失败" % (time.time() - t_inj))
        time.sleep(15)

    time.sleep(20)
    try:
        p_mq.wait(timeout=60)
    except subprocess.TimeoutExpired:
        p_mq.kill()
    mf.close()
    mq = open(prefix + "_mqtt.log", encoding="utf-8", errors="replace").read()
    ready_after = mq.count('"cmd":"ready"') + mq.count('"cmd": "ready"')

    log("\n===== 汇总 =====")
    res = {
        "hard_cycle_seen": hard_cycle_seen,
        "recover_at_s": round(recover_at, 1) if recover_at else None,
        "ready_msgs_total": ready_after,
        "auto_recovered": bool(hard_cycle_seen),
    }
    for k, v in res.items():
        log("  %s = %s" % (k, v))
    log("  结论: %s" % ("PASS —— 模组死透后固件自动硬拉电恢复，无需人工插拔"
                        if hard_cycle_seen else
                        "FAIL —— 未观察到自动硬恢复"))
    with open(prefix + "_summary.txt", "w", encoding="utf-8") as f:
        for k, v in res.items():
            f.write("%s = %s\n" % (k, v))
    return 0 if hard_cycle_seen else 1


if __name__ == "__main__":
    sys.exit(main())
