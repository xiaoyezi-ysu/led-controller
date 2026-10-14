# -*- coding: utf-8 -*-
"""72 MHz vs 8 MHz Run 电流对比实验
1) MQTT 发 lp_cycle_off（板子退出 lp_cycle 回常在线 Run）
2) SWD: 断 4G（PA3=0）→ F-11 基线（8 MHz, 含 DBG_SLEEP 偏置，两次口径一致）
3) SWD: FLASH 加 2WS + 开 PLL x9 + 切 SYSCLK=72 MHz（APB1=/2=36）
4) F-11 量 72 MHz 档 → 差值 = 纯时钟贡献
5) 软复位恢复出厂时钟
"""
import subprocess, time, sys, os

PY_A = r"E:/ProgramData/Anaconda3/python.exe"
MQTT = r"bench/mqtt_watch.py"
F11 = r"C:/Users/Administrator/.workbuddy/skills/afu-f11-meter/scripts/f11.py"
OCD = r"E:/Program Files/stm32tools/openocd/0.12.0-7.1/.content/bin/openocd.exe"
SCRIPTS = r"E:/Program Files/stm32tools/openocd/0.12.0-7.1/.content/openocd/scripts"

BASE_ARGS = ["-s", SCRIPTS,
             "-c", "adapter driver cmsis-dap; transport select swd; adapter speed 4000",
             "-c", "gdb_port disabled; tcl_port disabled; telnet_port disabled",
             "-f", "target/stm32f1x.cfg"]

def ocd(cmds, timeout=60):
    argv = [OCD] + BASE_ARGS
    for c in cmds:
        argv += ["-c", c]
    argv += ["-c", "exit"]
    r = subprocess.run(argv, capture_output=True, text=True,
                       encoding="utf-8", errors="replace", timeout=timeout)
    return r.stdout + r.stderr

def rd(out, addr):
    m = None
    for ln in out.splitlines():
        if ln.strip().startswith("0x%08x" % addr):
            m = ln
    if m is None:
        raise RuntimeError("no read 0x%08x in:\n%s" % (addr, out))
    return int(m.split(":")[1].strip(), 16)

def f11(path, secs):
    subprocess.run([PY_A, F11, "stream", "--csv", path, "--secs", str(segs(secs))],
                   capture_output=True, timeout=secs + 90)

def segs(n):
    return n

def mqtt_off():
    subprocess.run([PY_A, MQTT, "--send-only", "--allow-multi",
                    "--iccid", "898608961026C0241530", "--send-delay", "1",
                    "--cmd", '{"cmd":"lp_cycle_off"}'],
                   capture_output=True, timeout=60)

def median(path, lo=0.15, hi=0.9):
    import csv, statistics as st
    v = []
    with open(path, newline="", encoding="utf-8") as f:
        for x in csv.DictReader(f):
            try:
                v.append(float(x["current_mA"]))
            except (KeyError, ValueError):
                pass
    v.sort()
    a, b = int(len(v) * lo), int(len(v) * hi)
    return st.median(v[a:b]), len(v)

def main():
    print("== [1] MQTT 退 lp_cycle（连发 6 次）==", flush=True)
    for i in range(6):
        mqtt_off()
        time.sleep(10)
    time.sleep(5)

    print("== [2] SWD 会话1：断 4G + 读时钟寄存器 ==", flush=True)
    out = ocd(["init",
               "mdw 0x40021000 1",   # RCC_CR
               "mdw 0x40021004 1",   # RCC_CFGR
               "mdw 0x40022000 1",   # FLASH_ACR
               "mww 0x40010810 0x00080000",  # BSRR: BR3 => PA3=0
               "mdw 0x4001080c 1",   # ODR 验证
               "mww 0xE0042004 0"])  # 清调试位? 不行——清了 Stop 联不上，但这里非 Stop；
                                       # 保留默认(init 已置0x307)，两次口径一致，先不清
    print(out[-600:], flush=True)
    rcc_cr = rd(out, 0x40021000)
    rcc_cfgr = rd(out, 0x40021004)
    flash_acr = rd(out, 0x40022000)
    odr = rd(out, 0x4001080C)
    print("RCC_CR=0x%08x CFGR=0x%08x ACR=0x%08x ODR=0x%08x (PA3=%d)"
          % (rcc_cr, rcc_cfgr, flash_acr, odr, (odr >> 3) & 1), flush=True)
    assert (odr >> 3) & 1 == 0, "PA3 没断掉"

    print("== [3] F-11 基线 45 s（8 MHz）==", flush=True)
    f11("bench/data/clk72_base_8m.csv", 45)
    m8, n8 = median("bench/data/clk72_base_8m.csv")
    print("8 MHz Run 中位 = %.4f mA (n=%d)" % (m8, n8), flush=True)

    print("== [4] SWD 会话2：72 MHz 切换 ==", flush=True)
    cr_new = (rcc_cr | 0x01000000) & ~0x02000000          # PLLON=1
    cfgr_pre = (rcc_cfgr & 0xF) | 0x1D0400                # PLLSRC=HSE, PLLMUL=x9, PPRE1=/2, SW=HSE 暂不变
    cfgr_sw = (cfgr_pre & ~0x3) | 0x2                     # SW=PLL（先清 SW 再置 10）
    out = ocd(["init",
               "mww 0x40022000 0x12",                     # FLASH: 2WS + PRFTBE（切频前）
               "mww 0x40021004 0x%08x" % cfgr_pre,        # PLL 源/倍频（PLL 仍关时写）
               "mww 0x40021000 0x%08x" % cr_new,          # PLLON
               "sleep 30",                                 # 等 PLLRDY (ms)
               "mww 0x40021004 0x%08x" % cfgr_sw,         # SW=PLL
               "sleep 30",
               "mdw 0x40021004 1",                         # 验 SWS
               "mdw 0x40021000 1",
               "mdw 0x40022000 1"])
    print(out[-600:], flush=True)
    cfgr = rd(out, 0x40021004)
    sws = (cfgr >> 2) & 0x3
    print("SWS=%d (2=PLL %s)" % (sws, "OK" if sws == 2 else "未切上!"), flush=True)
    if sws != 2:
        print("切换失败，复位退出", flush=True)
        ocd(["init", "reset run", "shutdown"], timeout=40)
        sys.exit(2)

    print("== [5] F-11 量 72 MHz 60 s ==", flush=True)
    f11("bench/data/clk72_pll_72m.csv", 60)
    m72, n72 = median("bench/data/clk72_pll_72m.csv")
    print("72 MHz Run 中位 = %.4f mA (n=%d)" % (m72, n72), flush=True)
    print("\n===== 结论：Δ(72M-8M) = %+.3f mA（同一 WFI 口径、同一调试位状态）=====" % (m72 - m8), flush=True)

    print("== [6] 软复位恢复 ==", flush=True)
    out = ocd(["init", "reset run", "shutdown"], timeout=40)
    print("复位完成", flush=True)

if __name__ == "__main__":
    main()
