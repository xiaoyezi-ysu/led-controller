# -*- coding: utf-8 -*-
"""ab_probe2：板子已是冷启好状态。开 lp_cycle 测基线 -> 接一次 OpenOCD -> 再测。
判定「OpenOCD 接入」是否把 Stop 睡态从 0.21 抬到 1.03。
"""
import subprocess, time, re, os

PY    = r"E:\ProgramData\Anaconda3\python.exe"
F11   = r"C:\Users\Administrator\.workbuddy\skills\afu-f11-meter\scripts\f11.py"
MQTT  = r"E:\C++proj\stm32f103\stm32f103_CubemxCLI_demo0\stm32f103_CubemxCLI_demo0\bench\mqtt_watch.py"
LOG   = r"E:\C++proj\stm32f103\stm32f103_CubemxCLI_demo0\stm32f103_CubemxCLI_demo0\bench\data\mqtt_live.log"
OUT   = r"E:\C++proj\stm32f103\stm32f103_CubemxCLI_demo0\stm32f103_CubemxCLI_demo0\bench\data\ab_probe2_f11.csv"
TMARK = r"E:\C++proj\stm32f103\stm32f103_CubemxCLI_demo0\stm32f103_CubemxCLI_demo0\bench\data\ab_probe2_ocd_t.txt"
ICCID = "898608961026C0241530"
OCD   = r"E:/Program Files/stm32tools/openocd/0.12.0-7.1/.content/bin/openocd.exe"
OCDS  = r"E:/Program Files/stm32tools/openocd/0.12.0-7.1/.content/openocd/scripts"

def send(payload):
    return subprocess.run([PY, MQTT, "--send-only", "--allow-multi", "--iccid", ICCID,
                           "--send-delay", "1", "--cmd", payload],
                          capture_output=True, text=True, timeout=60)

def fresh_ready(t_after):
    """最近日志里是否有 t_after 之后的 ready/report（=板子醒着在线）"""
    try:
        with open(LOG, "r", encoding="utf-8", errors="replace") as f:
            tail = f.readlines()[-25:]
    except Exception:
        return False
    for ln in tail:
        m = re.match(r"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})", ln)
        if m:
            ts = time.mktime(time.strptime(m.group(1), "%Y-%m-%dT%H:%M:%S"))
            if ts > t_after and ("ready" in ln or "report" in ln):
                return True
    return False

STREAM_S = 470.0

print("== [1] F-11 记录 %.0fs ==" % STREAM_S, flush=True)
t0 = time.time()
fs = subprocess.Popen([PY, F11, "stream", "--csv", OUT, "--secs", str(int(STREAM_S))],
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(5)

print("== [2] 开 lp_cycle (period_min=1) ==", flush=True)
for k in range(3):
    send('{"cmd":"lp_cycle","period_min":1}')
    time.sleep(10)

print("== [3] 等基线睡窗（~120s）==", flush=True)
while time.time() - t0 < 130:
    time.sleep(5)

print("== [4] 等板子醒来(在线心跳)后再接 OpenOCD ==", flush=True)
got = False
while time.time() - t0 < 320:
    if fresh_ready(time.time() - 8):
        got = True
        break
    time.sleep(3)
t_ocd = time.time() - t0
print("  板子在线=%s  接入时刻 t_rel=%.1fs" % (got, t_ocd), flush=True)

r = subprocess.run([OCD, "-s", OCDS,
                    "-c", "adapter driver cmsis-dap; transport select swd; adapter speed 4000",
                    "-c", "gdb_port disabled; tcl_port disabled; telnet_port disabled",
                    "-f", "target/stm32f1x.cfg", "-c", "init",
                    "-c", "mdw 0x4001080c 1", "-c", "exit"],
                   capture_output=True, text=True, timeout=60)
print("  OpenOCD rc=%d" % r.returncode, flush=True)
with open(TMARK, "w") as f:
    f.write("%.2f\n" % t_ocd)

fs.wait()
print("== [5] 完成。OpenOCD 时刻 t_rel=%.1fs ==" % t_ocd, flush=True)
