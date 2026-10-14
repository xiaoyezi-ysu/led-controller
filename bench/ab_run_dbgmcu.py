# -*- coding: utf-8 -*-
"""DBGMCU_CR 对 "4G 断电 + 灯灭 + WFI 待机"（Run 档）电流的影响。
流程：lp_cycle_off -> v4g off -> F-11 窗A -> OpenOCD 清 DBGMCU_CR -> F-11 窗B
"""
import subprocess, time, re

PY   = r"E:\ProgramData\Anaconda3\python.exe"
F11  = r"C:\Users\Administrator\.workbuddy\skills\afu-f11-meter\scripts\f11.py"
MQTT = r"E:\C++proj\stm32f103\stm32f103_CubemxCLI_demo0\stm32f103_CubemxCLI_demo0\bench\mqtt_watch.py"
LOG  = r"E:\C++proj\stm32f103\stm32f103_CubemxCLI_demo0\stm32f103_CubemxCLI_demo0\bench\data\mqtt_live.log"
D    = r"E:\C++proj\stm32f103\stm32f103_CubemxCLI_demo0\stm32f103_CubemxCLI_demo0\bench\data"
ICCID= "898608961026C0241530"
OCD  = r"E:/Program Files/stm32tools/openocd/0.12.0-7.1/.content/bin/openocd.exe"
OCDS = r"E:/Program Files/stm32tools/openocd/0.12.0-7.1/.content/openocd/scripts"
DBGMCU = 0xE0042004

def send(p):
    return subprocess.run([PY, MQTT, "--send-only", "--allow-multi", "--iccid", ICCID,
                           "--send-delay", "1", "--cmd", p],
                          capture_output=True, text=True, timeout=60)

def awake():
    try:
        with open(LOG, "r", encoding="utf-8", errors="replace") as f:
            tail = f.readlines()[-25:]
    except Exception:
        return False
    for ln in tail:
        m = re.match(r"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})", ln)
        if m:
            ts = time.mktime(time.strptime(m.group(1), "%Y-%m-%dT%H:%M:%S"))
            if time.time() - ts < 7 and ("ready" in ln or "report" in ln):
                return True
    return False

def ocd(cmds):
    args = [OCD, "-s", OCDS,
            "-c", "adapter driver cmsis-dap; transport select swd; adapter speed 4000",
            "-c", "gdb_port disabled; tcl_port disabled; telnet_port disabled",
            "-f", "target/stm32f1x.cfg", "-c", "init"]
    for c in cmds: args += ["-c", c]
    args += ["-c", "exit"]
    r = subprocess.run(args, capture_output=True, text=True, timeout=60)
    return (r.stdout or "") + "\n" + (r.stderr or "")

def rec(path, secs):
    subprocess.run([PY, F11, "stream", "--csv", path, "--secs", str(secs)],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=secs+90)

print("== [1] 停 lp_cycle（连续重发直到收到）==", flush=True)
for k in range(12):
    send('{"cmd":"lp_cycle_off"}')
    time.sleep(10)
    if awake():
        print("   板子在线", flush=True); break

print("== [2] 关 4G（v4g off）→ 进入 4G断+灯灭 待机 ==", flush=True)
send('{"cmd":"v4g","state":"off"}')
time.sleep(8)

print("== [3] 窗A：DBGMCU_CR 当前值（应为 0x307）==", flush=True)
rec(D + r"\ab_run_A_f11.csv", 70)
out = ocd(["mdw 0x%08x 1" % DBGMCU])
for ln in out.splitlines():
    if ln.strip().startswith("0x"): print("   " + ln.strip(), flush=True)

print("== [4] 清 DBGMCU_CR = 0 ==", flush=True)
out = ocd(["mww 0x%08x 0" % DBGMCU, "mdw 0x%08x 1" % DBGMCU])
for ln in out.splitlines():
    if ln.strip().startswith("0x"): print("   " + ln.strip(), flush=True)

print("== [5] 窗B：清零后 ==", flush=True)
rec(D + r"\ab_run_B_f11.csv", 70)
print("完成", flush=True)
