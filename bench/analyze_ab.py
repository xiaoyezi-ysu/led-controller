# -*- coding: utf-8 -*-
"""按 markers 把 F-11 CSV 切成逐档统计（中位/最小/最大/样本数）。
用法: analyze_ab.py <f11.csv> [markers.csv]
缺 markers 时只打印低电流段。"""
import csv, sys, statistics as st, os

def load(p):
    rows = []
    with open(p, newline='', encoding='utf-8') as f:
        for x in csv.DictReader(f):
            try: rows.append((float(x['t_s']), float(x['current_mA'])))
            except: pass
    rows.sort(key=lambda z: z[0])
    return rows

def load_markers(p):
    if not p or not os.path.exists(p): return []
    ms = []
    with open(p, newline='', encoding='utf-8') as f:
        for x in csv.DictReader(f):
            try: ms.append((float(x.get('wall_rel_s') or x.get('t_s') or x.get('t_rel_s')), x.get('name','?')))
            except: pass
    ms.sort()
    return ms

def main():
    f11 = sys.argv[1]
    mk  = sys.argv[2] if len(sys.argv) > 2 else None
    rows = load(f11)
    print("F-11: %s  样本 %d  时长 %.1f s" % (os.path.basename(f11), len(rows), rows[-1][0]-rows[0][0]))
    ms = load_markers(mk)
    if ms:
        print("markers: %d 档" % len(ms))
        bounds = [(ms[i][0], ms[i][1], ms[i+1][0] if i+1 < len(ms) else rows[-1][0])
                  for i in range(len(ms))]
        print("%-16s %8s %8s %8s %8s %8s" % ("档名","t0","t1","中位","最小","最大"))
        for t0, name, t1 in bounds:
            v = [c for t, c in rows if t0 <= t < t1]
            if not v: 
                print("%-16s %8.1f %8.1f %8s" % (name, t0, t1, "(无样本)")); continue
            print("%-16s %8.1f %8.1f %8.4f %8.4f %8.4f  n=%d"
                  % (name, t0, t1, st.median(v), min(v), max(v), len(v)))
    else:
        print("--- 低电流(<=3mA)段 ---")
        segs=[]; cur=None
        for t,v in rows:
            if v<=3.0:
                if cur is None: cur=[t,t,v,v,0]
                cur[1]=t; cur[2]=min(cur[2],v); cur[3]=max(cur[3],v); cur[4]+=1
            else:
                if cur and cur[4]>5: segs.append(cur)
                cur=None
        if cur and cur[4]>5: segs.append(cur)
        for i,s in enumerate(segs):
            print("  段%d t=%.1f->%.1f (%.1fs) 中位=%.4f min=%.4f max=%.4f n=%d"
                  % (i, s[0], s[1], s[1]-s[0], st.median([c for t,c in rows if s[0]<=t<=s[1]]), s[2], s[3], s[4]))

if __name__ == "__main__":
    main()
