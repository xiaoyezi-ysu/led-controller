# -*- coding: utf-8 -*-
import csv, glob, re, statistics as st, datetime

paths = glob.glob("bench/data/coldboot_*_c01_f11.csv")
stamp = sorted(re.search(r"coldboot_(\d{8}_\d{6})_c01_f11", p).group(1) for p in paths)[-1]
lat = []
for c in range(1, 9):
    rows = []
    with open("bench/data/coldboot_%s_c%02d_f11.csv" % (stamp, c), newline="", encoding="utf-8") as f:
        for x in csv.DictReader(f):
            try:
                rows.append((float(x["t_s"]), float(x["current_mA"])))
            except (KeyError, ValueError):
                pass
    rows.sort()
    t40 = next((t for t, v in rows if v > 40.0), None)
    lat.append(t40)
    print("冷启#%d: 首次 >40mA @ t=%s" % (c, "%.1f s" % t40 if t40 else "未出现"))
print("上电→4G活跃: 中位 %.1f s / 最快 %.1f / 最慢 %.1f" % (st.median(lat), min(lat), max(lat)))

lines = [l for l in open("bench/data/mqtt_live.log", encoding="utf-8", errors="replace") if "ready" in l]
ts = [datetime.datetime.fromisoformat(l.split()[0]) for l in lines[-16:]]
gaps = [(ts[i + 1] - ts[i]).total_seconds() for i in range(0, len(ts) - 1, 2)]
print("ready 间隔:", ["%.0f s" % g for g in gaps[-8:]])
