# -*- coding: utf-8 -*-
"""
受控 A/B：探查 "接入 OpenOCD(SWD)" 是否把 Stop 睡态从 0.21 抬到 1.03。
流程：relay 冷启动 -> 等上线 -> F-11 连续记录 -> MQTT 开 lp_cycle
      -> 中途做一次 OpenOCD connect+mdw -> 继续记录
分析：以 OpenOCD 接入时刻为界，比较前后睡窗的中位电流。
"""
import subprocess, time, re, os, sys

PY    = r"E:\ProgramData\Anaconda3\python.exe"
F11   = r"C:\Users\Administrator\.workbuddy\skills\afu-f11-meter\scripts\f11.py"
RELAY = r"C:\Users\Administrator\.workbuddy\skills\usb-relay\scripts\relay.py"
MQTT  = r"E:\C++proj\stm32f103\stm32f103_CubemxCLI_demo0\stm32f103_CubemxCLI_demo0\bench\mqtt_watch.py"
LOG   = r"E:\C++proj\stm32f103\stm32f103_CubemxCLI_demo0\stm32f103_CubemxCLI_demo0\bench\data\mqtt_live.log"
OUT   = r"E:\C++proj\stm32f103\stm32f103_CubemxCLI_demo0\stm32f103_CubemxCLI_demo0\bench\data\ab_probe_f11.csv"
ICCID = "898608961026C0241530"
OCD   = r"E:/Program Files/stm32tools/openocd/0.12.0-7.1/.content/bin/openocd.exe"
OCDS  = r"E:/Program Files/stm32tools/openocd/0.12.0-7.1/.content/openocd/scripts"

def run(cmd, quiet=False, **kw):
    print("  $ " + " ".join(str(c) for c in cmd), flush=True)
    if quiet:
        kw.setdefault("stdout", subprocess.DEVNULL)
        kw.setdefault("stderr", subprocess.DEVNULL)
    else:
        kw.setdefault("capture_output", True)
        kw.setdefault("text", True)
        kw.setdefault("encoding", "utf-8")
        kw.setdefault("errors", "replace")
    return subprocess.run(cmd, **kw)

def wait_online(t_after, timeout=120):
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

STREAM_S   = 430.0
OCD_AT     = 205.0   # 相对 F-11 起点，接入 OpenOCD 的时刻

print("== [1] 冷启动 ==", flush=True)
run([PY, RELAY, "pulse", "--port", "COM12", "--addr", "1"])
t_reboot = time.time()
time.sleep(20)
print("== [2] 等上线 ==", flush=True)
print("  %s" % ("OK" if wait_online(t_reboot, 120) else "超时"), flush=True)
time.sleep(3)

print("== [3] F-11 记录 %.0fs ==" % STREAM_S, flush=True)
t_stream = time.time()
fs = subprocess.Popen([PY, F11, "stream", "--csv", OUT, "--secs", str(int(STREAM_S))],
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(4)

print("== [4] MQTT 开 lp_cycle (period_min=1) ==", flush=True)
for k in range(3):
    run([PY, MQTT, "--send-only", "--allow-multi", "--iccid", ICCID,
         "--send-delay", "1", "--cmd", '{"cmd":"lp_cycle","period_min":1}'], quiet=True)
    time.sleep(12)

el = time.time() - t_stream
if el < OCD_AT:
    time.sleep(OCD_AT - el)

print("== [5] 接入 OpenOCD 一次 (t_rel=%.0fs) ==" % (time.time() - t_stream), flush=True)
t_ocd = time.time() - t_stream
r = run([OCD, "-s", OCDS,
         "-c", "adapter driver cmsis-dap; transport select swd; adapter speed 4000",
         "-c", "gdb_port disabled; tcl_port disabled; telnet_port disabled",
         "-f", "target/stm32f1x.cfg", "-c", "init", "-c", "mdw 0x4001080C 1", "-c", "exit"])
print("  OpenOCD rc=%d  (%.0fs)" % (r.returncode, time.time() - t_stream - t_ocd), flush=True)

fs.wait()
print("== [6] 完成。OpenOCD 接入时刻 t_rel=%.1fs ==" % t_ocd, flush=True)
with open(os.path.join(os.path.dirname(OUT), "ab_probe_ocd_t.txt"), "w") as f:
    f.write("%.2f\n" % t_ocd)
