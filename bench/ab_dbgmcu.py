# -*- coding: utf-8 -*-
"""验证 DBGMCU_CR(0xE0042004) 的 DBG_STOP/DBG_SLEEP 位是否就是 1.03 mA 的元凶。
步骤：等板子醒着 -> OpenOCD 读 DBGMCU_CR -> 写 0 -> 回读确认 -> 退出 -> F-11 测睡态。
"""
import subprocess, time, re, os, sys

PY  = r"E:\ProgramData\Anaconda3\python.exe"
F11 = r"C:\Users\Administrator\.workbuddy\skills\afu-f11-meter\scripts\f11.py"
LOG = r"E:\C++proj\stm32f103\stm32f103_CubemxCLI_demo0\stm32f103_CubemxCLI_demo0\bench\data\mqtt_live.log"
OUT = r"E:\C++proj\stm32f103\stm32f103_CubemxCLI_demo0\stm32f103_CubemxCLI_demo0\bench\data\ab_dbgmcu_f11.csv"
OCD  = r"E:/Program Files/stm32tools/openocd/0.12.0-7.1/.content/bin/openocd.exe"
OCDS = r"E:/Program Files/stm32tools/openocd/0.12.0-7.1/.content/openocd/scripts"
DBGMCU = 0xE0042004

def awake(max_wait=90):
    t0 = time.time()
    while time.time() - t0 < max_wait:
        try:
            with open(LOG, "r", encoding="utf-8", errors="replace") as f:
                tail = f.readlines()[-25:]
        except Exception:
            tail = []
        for ln in tail:
            m = re.match(r"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})", ln)
            if m:
                ts = time.mktime(time.strptime(m.group(1), "%Y-%m-%dT%H:%M:%S"))
                if time.time() - ts < 7 and ("ready" in ln or "report" in ln):
                    return True
        time.sleep(2)
    return False

def ocd(cmds):
    args = [OCD, "-s", OCDS,
            "-c", "adapter driver cmsis-dap; transport select swd; adapter speed 4000",
            "-c", "gdb_port disabled; tcl_port disabled; telnet_port disabled",
            "-f", "target/stm32f1x.cfg", "-c", "init"]
    for c in cmds:
        args += ["-c", c]
    args += ["-c", "exit"]
    r = subprocess.run(args, capture_output=True, text=True, timeout=60)
    return (r.stdout or "") + "\n" + (r.stderr or "")

print("== 等板子醒着 ==", flush=True)
print("  awake=%s" % awake(), flush=True)

print("== [1] 读 DBGMCU_CR (OpenOCD 连上后) ==", flush=True)
out = ocd(["mdw 0x%08x 1" % DBGMCU])
for ln in out.splitlines():
    if ln.strip().startswith("0x"):
        print("   " + ln.strip(), flush=True)

print("== [2] 写 0 + 回读（同一次会话内，之后不再连 OpenOCD）==", flush=True)
out = ocd(["mww 0x%08x 0" % DBGMCU, "mdw 0x%08x 1" % DBGMCU])
for ln in out.splitlines():
    if ln.strip().startswith("0x"):
        print("   " + ln.strip(), flush=True)

print("== [3] F-11 记录 300s（板子仍在 lp_cycle）==", flush=True)
subprocess.run([PY, F11, "stream", "--csv", OUT, "--secs", "300"],
               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=400)
print("完成 -> %s" % OUT, flush=True)
