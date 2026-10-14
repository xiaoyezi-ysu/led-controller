# -*- coding: utf-8 -*-
"""
分析 Stop 睡段波形：判断 1.03 mA 是稳态直流漏电还是高频唤醒尖峰。
对比 ab_stop_f11 (今晚停 Stop) 与 ab1_lpcycle_f11 (MQTT驱动 lp_cycle)。
"""
import csv, sys, statistics as st
from collections import Counter

def load(path):
    rows = []
    with open(path, newline='', encoding='utf-8') as f:
        r = csv.DictReader(f)
        for x in r:
            try:
                rows.append((float(x['t_s']), float(x['current_mA'])))
            except Exception:
                pass
    rows.sort(key=lambda z: z[0])
    return rows

def report(name, rows, lo=0.0, hi=3.0):
    vals = [v for _, v in rows if lo <= v <= hi]
    if not vals:
        print(f"  [{name}] 无落在 [{lo},{hi}] 的样本")
        return
    vals_sorted = sorted(vals)
    n = len(vals_sorted)
    print(f"  [{name}] N={n}  min={min(vals):.4f}  p1={vals_sorted[int(n*0.01)]:.4f}  "
          f"median={st.median(vals_sorted):.4f}  p99={vals_sorted[int(n*0.99)]:.4f}  max={max(vals):.4f}  mean={st.mean(vals):.4f}")
    # 直方图：0.1 mA 分桶
    c = Counter()
    for v in vals:
        c[round(v, 1)] += 1
    print("    直方图(0.1mA桶): " + "  ".join(f"{k}mA:{v}" for k, v in sorted(c.items())))
    # 电流 uA 分辨率下的离散度
    neg = [v for v in vals if v < 1.0]
    if neg:
        print(f"    <1.0mA 样本 {len(neg)} 个，占比 {100*len(neg)/n:.1f}%；其 min={min(neg):.4f} max={max(neg):.4f}")

def segments(rows, thresh=3.0):
    """按电流阈值切段：低于 thresh 视为睡段"""
    segs = []
    cur = None
    for t, v in rows:
        if v <= thresh:
            if cur is None:
                cur = [t, t, v, v, 0]
            cur[1] = t; cur[2] = min(cur[2], v); cur[3] = max(cur[3], v); cur[4] += 1
        else:
            if cur is not None and cur[4] > 5:
                segs.append(cur)
            cur = None
    if cur is not None and cur[4] > 5:
        segs.append(cur)
    return segs

def main():
    base = "bench/data/"
    print("="*80)
    print("ab_stop_f11  —— 今晚 Stop 实验全览")
    rows = load(base+"ab_stop_f11.csv")
    print(f"  总样本 {len(rows)}，时长 {rows[-1][0]-rows[0][0]:.1f}s")
    # 采样率估算
    dts = [rows[i+1][0]-rows[i][0] for i in range(min(len(rows)-1, 500))]
    dts = [d for d in dts if d > 0]
    if dts:
        print(f"  采样率 ≈ {1/st.median(dts):.2f} Hz")
    print("\n  --- 睡段波形统计 (<=3.0 mA) ---")
    report("stop_sleep", rows)
    print("\n  --- 睡段自动切分 ---")
    for i, s in enumerate(segments(rows)):
        print(f"    段{i}: t={s[0]:.1f}→{s[1]:.1f}s ({s[1]-s[0]:.1f}s) min={s[2]:.4f} max={s[3]:.4f} n={s[4]}")

    print("\n" + "="*80)
    print("ab1_lpcycle_f11  —— MQTT 驱动 lp_cycle")
    rows1 = load(base+"ab1_lpcycle_f11.csv")
    print(f"  总样本 {len(rows1)}，时长 {rows1[-1][0]-rows1[0][0]:.1f}s")
    print("\n  --- 睡段波形统计 (<=3.0 mA) ---")
    report("lp1_sleep", rows1)
    print("\n  --- 睡段自动切分 ---")
    for i, s in enumerate(segments(rows1)):
        print(f"    段{i}: t={s[0]:.1f}→{s[1]:.1f}s ({s[1]-s[0]:.1f}s) min={s[2]:.4f} max={s[3]:.4f} n={s[4]}")

    print("\n" + "="*80)
    print("历史 lpcycle_rerun.csv（上午定案 0.226 mA）对照")
    import os
    hist = "D:/ProgramData/WorkBuddy/车牌光伏workbudy/.workbuddy/tmp/f11/lpcycle_rerun.csv"
    if os.path.exists(hist):
        rh = load(hist)
        print(f"  总样本 {len(rh)}")
        report("hist_rerun", rh)
    else:
        print(f"  未找到 {hist}")

if __name__ == "__main__":
    main()
