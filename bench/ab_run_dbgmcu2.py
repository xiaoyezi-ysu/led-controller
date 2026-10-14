# -*- coding: utf-8 -*-
"""DBGMCU_CR 对 Run 档（4G断+灯灭+WFI）电流的影响 —— 修正版。
v4g 非 sys 指令，payload 必须含当前 my_iccid（已变为 uid_066CFF51），所以两个 id 都带上。
"""
import subprocess, time

PY   = r"E:\ProgramData\Anaconda3\python.exe"
F11  = r"C:\Users\Administrator\.workbuddy\skills\afu-f11-meter\scripts\f11.py"
MQTT = r"E:\C++proj\stm32f103\stm32f103_CubemxCLI_demo0\stm32f103_CubemxCLI_demo0\bench\mqtt_watch.py"
D    = r"E:\C++proj\stm32f103\stm32f103_CubemxCLI_demo0\stm32f103_CubemxCLI_demo0\bench\data"
ICCID= "898608961026C0241530"
OCD  = r"E:/Program Files/stm32tools/openocd/0.12.0-7.1/.content/bin/openocd.exe"
OCDS = r"E:/Program Files/stm32tools/openocd/0.12.0-7.1/.content/openocd/scripts"
DBGMCU = 0xE0042004

def send(p):
    return subprocess.run([PY, MQTT, "--send-only", "--allow-multi", "--iccid", ICCID,
                           "--send-delay", "1", "--cmd", p],
                          capture_output=True, text=True, timeout=60)

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

print("== [1] 关 4G（payload 带两种 id）==", flush=True)
r = send('{"cmd":"v4g","state":"off","id":"uid_066CFF51","iccid":"898608961026C0241530"}')
print("   rc=%d" % r.returncode, flush=True)
time.sleep(12)

print("== [2] 窗A（DBGMCU_CR 应为 0x307，上一轮已置）——先确认 ==", flush=True)
out = ocd(["mdw 0x%08x 1" % DBGMCU])
for ln in out.splitlines():
    if ln.strip().startswith("0x"): print("   " + ln.strip(), flush=True)
rec(D + r"\ab_run_A_f11.csv", 70)

print("== [3] 清 DBGMCU_CR = 0 ==", flush=True)
out = ocd(["mww 0x%08x 0" % DBGMCU, "mdw 0x%08x 1" % DBGMCU])
for ln in out.splitlines():
    if ln.strip().startswith("0x"): print("   " + ln.strip(), flush=True)

print("== [4] 窗B（清零后）==", flush=True)
rec(D + r"\ab_run_B_f11.csv", 70)
print("完成", flush=True)
