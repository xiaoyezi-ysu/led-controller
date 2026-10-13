#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bench_plot.py —— 把 bench_run.py 采到的 F-11 CSV + SEQ 标记画成标注曲线 HTML
================================================================================
纯 CSS/SVG、零 CDN（本机 CDN 加载不出来，Chart.js 会出空白图区）。

输入
    --f11      bench_run.py 输出的 <out>_f11.csv
    --markers  bench_run.py 输出的 <out>_markers.csv
    --out      输出 HTML 路径（缺省与 f11 同名前缀 + .html）
    --title    报告标题
    --note     标题下的补充说明

输出内容
    0  结论卡片：整机平均 / 峰值 / 总电荷 / 各档中位
    1  全程电流曲线（SVG：分档色带 + 最小–最大包络 + 均值折线）
    2  各档中位电流横向柱状图
    3  逐档统计表（中位/均值/最小/最大/σ/电荷 mAh/占比）
    4  电压曲线（若接了 V 通道）
    5  口径与复现说明

用法
    python bench_plot.py --f11 data/bench_x_f11.csv --markers data/bench_x_markers.csv
"""

import argparse
import csv
import datetime
import html
import math
import os
import statistics
import sys

# 与 docs/ 下既有报告同一套配色（浅色主题）
PALETTE = ["#2563eb", "#16a34a", "#d97706", "#7c3aed", "#dc2626",
           "#0891b2", "#c2410c", "#4d7c0f", "#be185d", "#0e7490",
           "#a16207", "#4338ca", "#15803d", "#b91c1c", "#6d28d9", "#0369a1"]
BAND_ALPHA = [0.10, 0.17]


def esc(s):
    return html.escape(str(s), quote=True)


def load_f11(path):
    rows = []
    with open(path, "r", encoding="utf-8", newline="") as f:
        r = csv.DictReader(f)
        for d in r:
            try:
                t = float(d["t_s"])
                ma = float(d["current_mA"])
            except (KeyError, TypeError, ValueError):
                continue
            v = d.get("voltage_V") or ""
            try:
                v = float(v)
            except ValueError:
                v = None
            rows.append((t, ma, v, (d.get("wall_iso") or "").strip()))
    return rows


def iso_to_epoch(s):
    """把 bench_run/bench_swd 写的本地墙钟串转成 epoch 秒。"""
    s = (s or "").strip().replace("Z", "")
    for fmt in ("%Y-%m-%dT%H:%M:%S.%f", "%Y-%m-%dT%H:%M:%S",
                "%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S"):
        try:
            return datetime.datetime.strptime(s, fmt).timestamp()
        except ValueError:
            continue
    raise ValueError("无法解析时间串: %r" % s)


def classify_mqtt(payload):
    """把一条上行报文归类，用于曲线上的标记配色。"""
    s = (payload or "").strip()
    if s.startswith("{"):
        if '"report"' in s:
            return "report"
        if '"ready"' in s:
            return "ready"
        return "json"
    if s.startswith("+++") or s.startswith("AT+"):
        return "at"
    return "other"


def load_mqtt(path, epoch_base):
    """读 mqtt_watch.py 的 mqtt_live.csv，取板子上行（dir=='->'），换算成 t_rel_s。"""
    ev = []
    if not path or not os.path.exists(path) or epoch_base is None:
        return ev
    with open(path, "r", encoding="utf-8", newline="") as f:
        r = csv.DictReader(f)
        for d in r:
            try:
                ep = float(d["ts_epoch"])
            except (KeyError, TypeError, ValueError):
                continue
            if (d.get("dir") or "").strip() != "->":
                continue
            pl = d.get("payload") or ""
            ev.append({"t": ep - epoch_base, "kind": classify_mqtt(pl),
                       "topic": (d.get("topic") or "").strip(),
                       "payload": pl.strip(), "iso": (d.get("ts_iso") or "").strip()})
    ev.sort(key=lambda e: e["t"])
    return ev


def load_markers(path):
    segs = []
    if not path or not os.path.exists(path):
        return segs            # 运行中还没有 markers：只画曲线，不画档带
    with open(path, "r", encoding="utf-8", newline="") as f:
        r = csv.DictReader(f)
        for d in r:
            nm = (d.get("name") or "").strip()
            try:
                t = float(d["t_rel_s"])
            except (KeyError, TypeError, ValueError):
                continue
            if nm:
                segs.append({"index": int(d["index"]), "name": nm,
                             "dwell": int(float(d["dwell_s"])), "t": t})
            else:                                  # DONE 行：只取结束时刻
                segs.append({"index": -1, "name": "_END_", "dwell": 0, "t": t})
    if segs and segs[-1]["name"] != "_END_":
        segs.append({"index": -1, "name": "_END_", "dwell": 0, "t": None})
    return segs


def seg_stats(rows, t0, t1):
    sub = [r for r in rows if t0 <= r[0] < t1]
    if not sub:
        return None
    cs = [r[1] for r in sub]
    chg = 0.0
    for i in range(1, len(sub)):
        dt = sub[i][0] - sub[i - 1][0]
        chg += (sub[i][1] + sub[i - 1][1]) / 2.0 * dt / 3600.0     # mAh
    vs = [r[2] for r in sub if r[2] is not None]
    return {
        "n": len(sub), "t0": sub[0][0], "t1": sub[-1][0],
        "dur": sub[-1][0] - sub[0][0] + (sub[1][0] - sub[0][0] if len(sub) > 1 else 0),
        "med": statistics.median(cs), "mean": sum(cs) / len(cs),
        "min": min(cs), "max": max(cs),
        "sd": (statistics.pstdev(cs) if len(cs) > 1 else 0.0),
        "mAh": chg,
        "vmin": (min(vs) if vs else None), "vmax": (max(vs) if vs else None),
    }


def seg_at(segs, t, t_last):
    """返回时刻 t 落在哪个工况档（按 SEQ 标记切分）。"""
    for k, s in enumerate(segs):
        if s["index"] < 0 or s["t"] is None:
            continue
        t0 = s["t"]
        t1 = segs[k + 1]["t"] if k + 1 < len(segs) and segs[k + 1]["t"] else t_last
        if t0 <= t < t1:
            return s["name"]
    return None


def bucket(rows, target=1400):
    n = len(rows)
    if n == 0:
        return []
    step = max(1, int(math.ceil(n / float(target))))
    out = []
    for i in range(0, n, step):
        blk = rows[i:i + step]
        ts = [b[0] for b in blk]
        cs = [b[1] for b in blk]
        out.append((sum(ts) / len(ts), sum(cs) / len(cs), min(cs), max(cs)))
    return out


def mmss(x):
    x = int(round(x))
    return "%d:%02d" % (x // 60, x % 60)


# ------------------------------------------------------------------ SVG 绘图 ---
EV_STYLE = {"report": ("#16a34a", 3.6, 0.95),
            "ready":  ("#94a3b8", 2.1, 0.55),
            "at":     ("#7c3aed", 3.0, 0.85),
            "json":   ("#2563eb", 3.0, 0.85),
            "other":  ("#d97706", 2.6, 0.75)}


def svg_curve(rows, segs, title, unit="mA", height=400, ylab="电流 (mA)", mqtt=None):
    W, H = 1240, height
    L, R, T, B = 76, 22, 34, 46
    if not rows:
        return '<p class="mut">（无数据）</p>'
    t_first, t_last = rows[0][0], rows[-1][0]
    span = max(1e-6, t_last - t_first)
    ymax = max(r[1] for r in rows)
    ytop = max(1.0, ymax * 1.10)

    def X(t):
        return L + (t - t_first) / span * (W - L - R)

    def Y(v):
        return H - B - (v / ytop) * (H - T - B)

    p = []
    p.append('<svg viewBox="0 0 %d %d" role="img" aria-label="%s">' % (W, H, esc(title)))
    p.append('<rect x="0" y="0" width="%d" height="%d" fill="#ffffff"/>' % (W, H))

    # 分档色带
    n_seg = len([s for s in segs if s["index"] >= 0])
    for k, s in enumerate(segs):
        if s["index"] < 0:
            continue
        t0 = s["t"]
        t1 = segs[k + 1]["t"] if k + 1 < len(segs) and segs[k + 1]["t"] else t_last
        t1 = min(t1, t_last + 0.001)
        x0, x1 = X(max(t0, t_first)), X(min(t1, t_last))
        if x1 - x0 < 0.5:
            continue
        fill = PALETTE[s["index"] % len(PALETTE)]
        p.append('<rect x="%.2f" y="%d" width="%.2f" height="%d" fill="%s" opacity="%.2f">'
                 '<title>%d · %s (%.1f–%.1f s)</title></rect>'
                 % (x0, T, x1 - x0, H - T - B, fill, BAND_ALPHA[s["index"] % 2],
                    s["index"] + 1, esc(s["name"]), t0, t1))
        if x1 - x0 > 26:
            p.append('<text x="%.2f" y="%d" font-size="11" fill="#475569" text-anchor="middle">%d</text>'
                     % ((x0 + x1) / 2, T - 8, s["index"] + 1))

    # y 网格
    for i in range(6):
        v = ytop * i / 5.0
        y = Y(v)
        p.append('<line x1="%d" y1="%.2f" x2="%d" y2="%.2f" stroke="#e5e8ef" stroke-width="1"/>'
                 % (L, y, W - R, y))
        p.append('<text x="%d" y="%.2f" font-size="11" fill="#6b7280" text-anchor="end">%s</text>'
                 % (L - 8, y + 4, ("%.1f" % v) if ytop < 100 else ("%.0f" % v)))
    p.append('<text x="%d" y="%d" font-size="11.5" fill="#6b7280" text-anchor="start">%s</text>'
             % (L - 62, T - 12, esc(ylab)))

    # 包络 + 均值线
    bk = bucket(rows)
    if bk:
        up = [(X(b[0]), Y(b[3])) for b in bk]
        dn = [(X(b[0]), Y(b[2])) for b in bk]
        pts = " ".join("%.1f,%.1f" % q for q in up + list(reversed(dn)))
        p.append('<polygon points="%s" fill="#93c5fd" opacity="0.35"/>' % pts)
        mid = " ".join("%.1f,%.1f" % (X(b[0]), Y(b[1])) for b in bk)
        p.append('<polyline points="%s" fill="none" stroke="#1d4ed8" stroke-width="1.3"/>' % mid)

    # MQTT 上行事件：竖虚线 + 顶部圆点（颜色按报文类型）
    if mqtt:
        rail = T + 6
        for e in mqtt:
            if e["t"] < t_first - 0.5 or e["t"] > t_last + 0.5:
                continue
            x = X(e["t"])
            col, r_, op = EV_STYLE.get(e["kind"], EV_STYLE["other"])
            p.append('<line x1="%.2f" y1="%d" x2="%.2f" y2="%d" stroke="%s" '
                     'stroke-width="0.8" stroke-dasharray="2 3" opacity="0.45"/>'
                     % (x, rail, x, H - B, col))
            p.append('<circle cx="%.2f" cy="%d" r="%.1f" fill="%s" opacity="%.2f" '
                     'stroke="#ffffff" stroke-width="0.6"><title>%s | %s</title></circle>'
                     % (x, rail, r_, col, op,
                        esc("%s t=%s" % (e["kind"], mmss(e["t"]))), esc(e["payload"][:90])))

    # x 轴刻度
    for i in range(9):
        t = t_first + span * i / 8.0
        x = X(t)
        p.append('<line x1="%.2f" y1="%d" x2="%.2f" y2="%d" stroke="#cbd5e1" stroke-width="1"/>'
                 % (x, H - B, x, H - B + 5))
        p.append('<text x="%.2f" y="%d" font-size="11" fill="#6b7280" text-anchor="middle">%s</text>'
                 % (x, H - B + 20, mmss(t - t_first)))
    p.append('<text x="%d" y="%d" font-size="11" fill="#6b7280" text-anchor="middle">t (mm:ss)</text>'
             % ((L + W - R) / 2, H - 8))
    p.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#94a3b8" stroke-width="1.2"/>'
             % (L, H - B, W - R, H - B))
    p.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#94a3b8" stroke-width="1.2"/>'
             % (L, T, L, H - B))
    p.append('</svg>')
    return "\n".join(p)


def css_bars(items, unit="mA"):
    """items: [(label, value, extra)] 横向柱状图（纯 CSS）"""
    if not items:
        return ""
    vmax = max(v for _, v, _ in items) or 1.0
    h = []
    h.append('<div class="hbars">')
    for i, (lab, v, extra) in enumerate(items):
        w = max(0.6, v / vmax * 100.0)
        c = PALETTE[i % len(PALETTE)]
        h.append(
            '<div class="hrow"><div class="hlab">%s</div>'
            '<div class="htrack"><div class="hfill" style="width:%.2f%%;background:%s"></div></div>'
            '<div class="hval">%.3f %s%s</div></div>'
            % (esc(lab), w, c, v, unit, ("  <span class='mut'>%s</span>" % esc(extra)) if extra else ""))
    h.append('</div>')
    return "\n".join(h)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--f11", required=True)
    ap.add_argument("--markers", required=True)
    ap.add_argument("--out", default=None)
    ap.add_argument("--title", default="工况序列实测 · 整板电流曲线（F-11 真值）")
    ap.add_argument("--note", default="")
    ap.add_argument("--mqtt", default=None,
                    help="mqtt_watch.py 的 mqtt_live.csv；缺省自动找 F-11 同目录的 mqtt_live.csv")
    a = ap.parse_args()

    rows = load_f11(a.f11)
    segs = load_markers(a.markers)
    if not rows:
        sys.stderr.write("CSV 无有效数据：%s\n" % a.f11)
        return 1

    # 与 MQTT epoch 对齐的时基：epoch_base 满足 t_rel = ts_epoch - epoch_base
    epoch_base = None
    for r in rows:
        if r[3]:
            try:
                epoch_base = iso_to_epoch(r[3]) - r[0]
                break
            except ValueError:
                pass
    mqtt_path = a.mqtt
    if mqtt_path is None:
        cand = os.path.join(os.path.dirname(os.path.abspath(a.f11)), "mqtt_live.csv")
        if os.path.exists(cand):
            mqtt_path = cand
    mqtt = load_mqtt(mqtt_path, epoch_base)
    mqtt_kind_n = {}
    for e in mqtt:
        mqtt_kind_n[e["kind"]] = mqtt_kind_n.get(e["kind"], 0) + 1

    t_first, t_last = rows[0][0], rows[-1][0]
    if segs and segs[-1]["t"] is None:
        segs[-1]["t"] = t_last + 0.01

    # 逐档统计
    stats = []
    for k, s in enumerate(segs):
        if s["index"] < 0:
            continue
        t0 = s["t"]
        t1 = segs[k + 1]["t"] if k + 1 < len(segs) and segs[k + 1]["t"] else t_last
        st = seg_stats(rows, t0, t1)
        if st:
            st["name"] = s["name"]
            st["index"] = s["index"]
            st["nominal"] = s["dwell"]
            stats.append(st)

    all_cs = [r[1] for r in rows]
    total_s = t_last - t_first
    tot_mAh = 0.0
    for i in range(1, len(rows)):
        dt = rows[i][0] - rows[i - 1][0]
        tot_mAh += (rows[i][1] + rows[i - 1][1]) / 2.0 * dt / 3600.0
    # 电荷(mAh) → 平均电流(mA)：mAh × 3600 / 秒
    avg_mA = (tot_mAh * 3600.0 / total_s) if total_s > 0 else 0.0
    has_v = any(r[2] is not None and r[2] > 0.01 for r in rows)

    out = a.out or (os.path.splitext(a.f11)[0] + "_annotated.html")
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # ---------------- 组装页面 ----------------
    H = []
    H.append('<!DOCTYPE html>')
    H.append('<html lang="zh-CN"><head><meta charset="UTF-8">')
    H.append('<meta name="viewport" content="width=device-width,initial-scale=1">')
    H.append('<title>%s</title>' % esc(a.title))
    H.append('''<style>
  :root{
    --bg:#f6f7fb; --card:#ffffff; --ink:#1e2430; --mut:#6b7280; --line:#e5e8ef;
    --green:#16a34a; --red:#dc2626; --blue:#2563eb; --purple:#7c3aed; --amber:#d97706;
  }
  *{box-sizing:border-box}
  body{margin:0;padding:32px 4vw;font-family:-apple-system,"Segoe UI","Microsoft YaHei",system-ui,sans-serif;
       background:var(--bg);color:var(--ink);line-height:1.65}
  h1{font-size:24px;margin:0 0 6px;letter-spacing:.2px}
  h2{font-size:18px;margin:34px 0 12px;border-left:4px solid var(--blue);padding-left:11px}
  h3{font-size:15px;margin:18px 0 8px}
  .sub{color:var(--mut);font-size:13px;margin-bottom:22px}
  .card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:18px 20px;margin-bottom:16px}
  .kpis{display:grid;grid-template-columns:repeat(auto-fit,minmax(158px,1fr));gap:12px;margin:14px 0 4px}
  .kpi{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px 16px}
  .kpi .lab{font-size:12px;color:var(--mut)}
  .kpi .val{font-size:22px;font-weight:700;margin-top:3px}
  .kpi .val small{font-size:12.5px;font-weight:400;color:var(--mut)}
  .good{border-left:4px solid var(--green);background:#f0fdf4;border-radius:8px;padding:12px 15px;margin:11px 0}
  .bad{border-left:4px solid var(--red);background:#fef2f2;border-radius:8px;padding:12px 15px;margin:11px 0}
  .hl{border-left:4px solid var(--amber);background:#fffbeb;border-radius:8px;padding:12px 15px;margin:11px 0}
  .info{border-left:4px solid var(--blue);background:#eff6ff;border-radius:8px;padding:12px 15px;margin:11px 0}
  table{width:100%;border-collapse:collapse;font-size:13.3px;margin-top:8px}
  th,td{border:1px solid var(--line);padding:6px 9px;text-align:left;vertical-align:top}
  th{background:#f1f5f9}
  td.num{font-variant-numeric:tabular-nums;text-align:right;white-space:nowrap}
  .mut{color:var(--mut);font-size:12.5px}
  code{background:#eef2ff;border-radius:4px;padding:1px 5px;font-size:12.6px;font-family:"Cascadia Code",Consolas,monospace}
  pre{background:#0f172a;color:#e2e8f0;border-radius:10px;padding:14px 16px;overflow:auto;font-size:12.6px;line-height:1.55}
  svg{max-width:100%;height:auto;display:block}
  .hbars{margin:6px 0 2px}
  .hrow{display:flex;align-items:center;gap:10px;margin:5px 0}
  .hlab{width:150px;flex:none;font-size:12.6px;text-align:right;color:#334155;
        white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
  .htrack{flex:1;height:17px;background:#f1f5f9;border-radius:4px;overflow:hidden;border:1px solid var(--line)}
  .hfill{height:100%;border-radius:3px}
  .hval{width:168px;flex:none;font-size:12.4px;font-variant-numeric:tabular-nums;color:#1e2430}
  .swatch{display:inline-block;width:11px;height:11px;border-radius:3px;margin-right:6px;vertical-align:-1px}
  .foot{color:var(--mut);font-size:12px;margin-top:34px;border-top:1px solid var(--line);padding-top:12px}
</style></head><body>''')

    H.append('<h1>%s</h1>' % esc(a.title))
    H.append('<div class="sub">数据源：安服优 F-11 真值电流表（串联在回路内，10 Hz）＋ 固件 SEQ 标记（SWD/串口）%s'
             ' &nbsp;·&nbsp; 生成 %s &nbsp;·&nbsp; 记录 %s（%d 样本）</div>'
             % ('＋ MQTT 旁听端上行事件' if mqtt else '', esc(stamp), mmss(total_s), len(rows)))
    if a.note:
        H.append('<div class="info">%s</div>' % esc(a.note))

    # --- 0 结论 ---
    H.append('<h2>0 · 结论先说</h2>')
    peak = max(all_cs)
    trough = min(all_cs)
    H.append('<div class="kpis">')
    H.append('<div class="kpi"><div class="lab">全程平均电流</div><div class="val">%.3f <small>mA</small></div>'
             '<div class="mut">按电荷/时长加权</div></div>' % avg_mA)
    H.append('<div class="kpi"><div class="lab">峰值 / 谷值</div><div class="val">%.2f <small>/ %.3f mA</small></div>'
             '<div class="mut">单样本极值</div></div>' % (peak, trough))
    H.append('<div class="kpi"><div class="lab">全程电荷</div><div class="val">%.2f <small>mAh</small></div>'
             '<div class="mut">∫I dt</div></div>' % tot_mAh)
    H.append('<div class="kpi"><div class="lab">工况档数</div><div class="val">%d <small>档</small></div>'
             '<div class="mut">按 SEQ 标记切分</div></div>' % len(stats))
    if mqtt:
        H.append('<div class="kpi"><div class="lab">MQTT 上行事件</div><div class="val">%d <small>条</small></div>'
                 '<div class="mut">旁听端捕获 · 上报 %d / 心跳 %d</div></div>'
                 % (len(mqtt), mqtt_kind_n.get("report", 0), mqtt_kind_n.get("ready", 0)))
    H.append('</div>')

    if stats:
        lo = min(stats, key=lambda s: s["med"])
        hi = max(stats, key=lambda s: s["med"])
        H.append('<div class="good"><b>最省档</b>：<code>%s</code> 中位 <b>%.3f mA</b>；'
                 '<b>最费档</b>：<code>%s</code> 中位 <b>%.3f mA</b>。'
                 '两者相差 <b>%.1f×</b>。</div>'
                 % (esc(lo["name"]), lo["med"], esc(hi["name"]), hi["med"],
                    (hi["med"] / lo["med"]) if lo["med"] > 0 else float("nan")))

    # --- 1 曲线 ---
    H.append('<h2>1 · 全程电流曲线</h2>')
    H.append('<div class="card">')
    H.append(svg_curve(rows, segs, a.title, mqtt=mqtt))
    H.append('<div class="mut" style="margin-top:10px">浅蓝带 = 每个采样桶的最小–最大包络（保留 4G 瞬态尖峰）；'
             '深蓝折线 = 桶内均值；底色色带 = 固件 SEQ 标记切出的工况档，编号与下表一致。'
             + ('　竖虚线+顶部圆点 = 4G 模组的上行 MQTT 事件（<span style="color:#16a34a">绿=report</span>、'
                '<span style="color:#94a3b8">灰=ready 心跳</span>、'
                '<span style="color:#7c3aed">紫=+++/AT 指令</span>），悬停看原文。'
                if mqtt else '') + '</div>')
    H.append('</div>')

    # --- 2 中位柱状 ---
    H.append('<h2>2 · 各档中位电流</h2>')
    H.append('<div class="card">')
    H.append(css_bars([("%d %s" % (s["index"] + 1, s["name"]), s["med"],
                        "均值 %.2f" % s["mean"]) for s in stats]))
    H.append('<div class="mut" style="margin-top:6px">中位数比均值更抗 4G 的脉冲式跳变，'
             '所以口径统一用中位；均值一并列出供对照。</div>')
    H.append('</div>')

    # --- 3 逐档统计表 ---
    H.append('<h2>3 · 逐档统计</h2>')
    H.append('<div class="card"><table><thead><tr>'
             '<th>#</th><th>工况</th><th>标称 s</th><th>实测 s</th>'
             '<th>中位 mA</th><th>均值 mA</th><th>最小</th><th>最大</th><th>σ</th>'
             '<th>电荷 mAh</th><th>占全程</th></tr></thead><tbody>')
    for s in stats:
        share = (s["mAh"] / tot_mAh * 100.0) if tot_mAh else 0.0
        sw = '<span class="swatch" style="background:%s"></span>' % PALETTE[s["index"] % len(PALETTE)]
        H.append('<tr><td class="num">%d</td><td>%s%s</td>'
                 '<td class="num">%d</td><td class="num">%.1f</td>'
                 '<td class="num"><b>%.3f</b></td><td class="num">%.3f</td>'
                 '<td class="num">%.3f</td><td class="num">%.3f</td><td class="num">%.3f</td>'
                 '<td class="num">%.3f</td><td class="num">%.1f%%</td></tr>'
                 % (s["index"] + 1, sw, esc(s["name"]), s["nominal"], s["dur"],
                    s["med"], s["mean"], s["min"], s["max"], s["sd"], s["mAh"], share))
    H.append('<tr><td></td><td><b>合计</b></td><td class="num">%d</td><td class="num">%.1f</td>'
             '<td class="num">—</td><td class="num"><b>%.3f</b></td><td class="num">—</td>'
             '<td class="num">—</td><td class="num">—</td><td class="num"><b>%.3f</b></td>'
             '<td class="num">100%%</td></tr>'
             % (sum(s["nominal"] for s in stats), sum(s["dur"] for s in stats),
                avg_mA, tot_mAh))
    H.append('</tbody></table>')
    H.append('<div class="mut" style="margin-top:8px">σ = 样本总体标准差。'
             '「占全程」按电荷（∫I dt）计，不是按时长 —— 功耗账本要按电荷算。</div>')
    H.append('</div>')

    # --- 3b MQTT 上行事件 ---
    if mqtt:
        H.append('<h2>4 · MQTT 上行事件（旁听端捕获）</h2>')
        H.append('<div class="card">')
        H.append('<div class="mut">数据源 <code>%s</code>；t=0 与 F-11 曲线同一时基。'
                 '用于交叉验证：板子在<b>哪一档、什么电流水平下</b>实际发出了哪条上行报文。</div>'
                 % esc(os.path.basename(mqtt_path or "mqtt_live.csv")))
        per = {}
        for e in mqtt:
            nm = seg_at(segs, e["t"], t_last) or "（档外）"
            d = per.setdefault(nm, {})
            d[e["kind"]] = d.get(e["kind"], 0) + 1
        if per:
            H.append('<h3>按工况档汇总</h3><table><thead><tr><th>工况档</th>'
                     '<th>report</th><th>ready 心跳</th><th>AT/+++</th><th>其他</th><th>合计</th>'
                     '</tr></thead><tbody>')
            for nm, d in per.items():
                tot = sum(d.values())
                H.append('<tr><td>%s</td><td class="num">%d</td><td class="num">%d</td>'
                         '<td class="num">%d</td><td class="num">%d</td><td class="num"><b>%d</b></td></tr>'
                         % (esc(nm), d.get("report", 0), d.get("ready", 0),
                            d.get("at", 0), d.get("json", 0) + d.get("other", 0), tot))
            H.append('</tbody></table>')
        H.append('<h3>逐条事件</h3><div style="max-height:340px;overflow:auto">'
                 '<table><thead><tr><th>t (mm:ss)</th><th>类型</th><th>所在档</th><th>报文</th>'
                 '</tr></thead><tbody>')
        for e in mqtt:
            col = EV_STYLE.get(e["kind"], EV_STYLE["other"])[0]
            H.append('<tr><td class="num">%s</td>'
                     '<td><span class="swatch" style="background:%s"></span>%s</td>'
                     '<td>%s</td><td><code>%s</code></td></tr>'
                     % (mmss(e["t"]), col, esc(e["kind"]),
                        esc(seg_at(segs, e["t"], t_last) or "—"), esc(e["payload"][:120])))
        H.append('</tbody></table></div>')
        H.append('</div>')

    # --- 5 电压 ---
    if has_v:
        H.append('<h2>5 · 电池电压曲线</h2>')
        H.append('<div class="card">')
        H.append(svg_curve([(r[0], (r[2] or 0.0) * 1000.0, None) for r in rows], segs,
                           "电压", height=300, ylab="电压 (mV)"))
        vs = [r[2] for r in rows if r[2] is not None]
        H.append('<div class="kpis" style="margin-top:12px">'
                 '<div class="kpi"><div class="lab">最低电压</div><div class="val">%.3f <small>V</small></div></div>'
                 '<div class="kpi"><div class="lab">最高电压</div><div class="val">%.3f <small>V</small></div></div>'
                 '<div class="kpi"><div class="lab">最大压降</div><div class="val">%.0f <small>mV</small></div>'
                 '<div class="mut">用于判断 4G 上电浪涌是否把电池拉到低压复位</div></div></div>'
                 % (min(vs), max(vs), (max(vs) - min(vs)) * 1000.0))
        H.append('</div>')

    # --- 6 复现 ---
    H.append('<h2>6 · 口径与复现</h2>')
    H.append('<div class="card">')
    H.append('<h3>测量口径</h3><ul>'
             '<li>电流为 <b>F-11 真值</b>：表串在被测回路内（<code>电源GND → IN−</code>、'
             '<code>OUT− → 板GND</code>），不是探针 <code>power-read</code>。'
             '后者有 +6.7 mA 中位直流偏置且对起伏负载给窗口均值，本场景会明显失真。</li>'
             '<li>工况置态由<b>固件内的测试引擎</b>完成（PC 串口 <code>!</code> 命令），'
             '不经过 MQTT —— MQTT 侧任何被识别的指令都会顺手 <code>DTU_SetLowPower(0)</code>，'
             '把被测的低功耗态清掉。</li>'
             '<li>档间切换与采样串行进行，不与 SWD 探针并行（并行会抢探针导致整轮数据作废）。</li>'
             '</ul>')
    H.append('<h3>复现命令</h3><pre>%s</pre>'
             % esc('python bench/mqtt_watch.py --quiet-json          # 常驻旁听端（后台）\n'
                   'python bench/bench_swd.py --cmd "seq 60" --out data/xxx\n'
                   'python bench/bench_plot.py --f11 data/xxx_f11.csv --markers data/xxx_markers.csv'))
    H.append('<h3>原始文件</h3><ul><li>%s</li><li>%s</li>%s</ul>'
             % (esc(os.path.basename(a.f11)), esc(os.path.basename(a.markers)),
                ('<li>%s（MQTT 上行事件）</li>' % esc(os.path.basename(mqtt_path)))
                if mqtt else ''))
    H.append('</div>')

    H.append('<div class="foot">由 <code>bench/bench_plot.py</code> 生成 · 纯 CSS/SVG、零外部依赖 · '
             '固件：<code>Core/Src/main.c</code> 的 SEQK_* 测试引擎</div>')
    H.append('</body></html>')

    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(H))
    print("写出: %s  (%d 字节)" % (out, os.path.getsize(out)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
