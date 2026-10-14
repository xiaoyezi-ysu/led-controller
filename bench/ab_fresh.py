# -*- coding: utf-8 -*-
"""
冷启动 A/B：整板 power-cycle 后立刻用 MQTT 驱动 lp_cycle，全程不碰探针，
F-11 记录。用来判定 "Stop 1.03 mA" 是否与板子的运行历史有关。
"""
import subprocess, time, os, sys, io, re

PY    = r"E:\ProgramData\Anaconda3\python.exe"
F11   = r"C:\Users\Administrator\.workbuddy\skills\afu-f11-meter\scripts\f11.py"
RELAY = r"C:\Users\Administrator\.workbuddy\skills\usb-relay\scripts\relay.py"
MQTT  = r"E:\C++proj\stm32f103\stm32f103_CubemxCLI_demo0\stm32f103_CubemxCLI_demo0\bench\mqtt_watch.py"
LOG   = r"E:\C++proj\stm32f103\stm32f103_CubemxCLI_demo0\stm32f103_CubemxCLI_demo0\bench\data\mqtt_live.log"
OUT   = r"E:\C++proj\stm32f103\stm32f103_CubemxCLI_demo0\stm32f103_CubemxCLI_demo0\bench\data\ab_fresh_f11.csv"
ICCID = "898608961026C0241530"

def run(cmd, **kw):
    print("  $ " + " ".join(str(c) for c in cmd), flush=True)
    r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace", **kw)
    if r.stdout:
        print("    " + r.stdout.strip()[:400], flush=True)
    if r.returncode != 0 and r.stderr:
        print("    [err] " + r.stderr.strip()[:300], flush=True)
    return r

def wait_online(t_after, timeout=120):
    """等到 mqtt_live.log 里出现 t_after 之后的 ready/report 行"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            with open(LOG, "r", encoding="utf-8", errors="replace") as f:
                tail = f.readlines()[-40:]
        except Exception:
            tail = []
        for ln in tail:
            m = re.match(r"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})", ln)
            if m:
                ts = time.mktime(time.strptime(m.group(1), "%Y-%m-%dT%H:%M:%S"))
                if ts > t_after and ("ready" in ln or "report" in ln):
                    return True
        time.sleep(3)
    return False

print("== [1] 整板 power-cycle ==", flush=True)
run([PY, RELAY, "pulse", "--port", "COM12", "--addr", "1"])
t_reboot = time.time()
print("  已断电重启，等待板子启动 + 4G 上线 ...", flush=True)
time.sleep(20)

print("== [2] 等 MQTT 上线 ==", flush=True)
ok = wait_online(t_reboot, timeout=120)
print("  上线判定：%s" % ("OK" if ok else "超时（仍继续）"), flush=True)
time.sleep(3)

print("== [3] 启动 F-11 记录 260 s ==", flush=True)
fs = subprocess.Popen([PY, F11, "stream", "--csv", OUT, "--secs", "260"],
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(4)

print("== [4] MQTT 下发 lp_cycle period_min=1 ==", flush=True)
run([PY, MQTT, "--send-only", "--allow-multi", "--iccid", ICCID,
     "--send-delay", "1", "--cmd", '{"cmd":"lp_cycle","period_min":1}'])

print("== [5] 等 F-11 采完 ==", flush=True)
fs.wait()
print("完成。输出 %s" % OUT, flush=True)
