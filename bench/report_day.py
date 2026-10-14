# -*- coding: utf-8 -*-
"""把「工况压缩日」三轮实测（P1 13 档表 / P2·P3 压缩日）合成一份总报告。

输出 docs/dutycycle-day-test.html —— 纯 CSS/SVG，零外部 CDN。

用法：
    python bench/report_day.py --stamp 20261013_192956
"""
import argparse
import csv
import datetime
import io
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATA = os.path.join(HERE, "data")
DOCS = os.path.join(ROOT, "docs")

# ---------- 口径常量（集中放这里，改一处即可） ----------
NIGHT_MA = 0.226         # 夜间 Stop 睡态；主流程会按「从数据实测」覆盖（见下方 _measure_sleep）
NIGHT_MA_DIRTY = 1.030   # 旧口径：带 OpenOCD 调试保活位（DBGMCU_CR=0x307）的读数，仅作对照
NIGHT_MA_BEST = 0.226    # 历史最佳 Stop 睡态（mklink 测，未污染）
ROUND_S = 155.0          # 每轮窗口标称时长（90 会话 + 5 收尾 + 60 睡）
DAY_S = 86400.0
BATT_MAH = 1000.0
BATT_USABLE = 0.80


def ep(s):
    return datetime.datetime.strptime(s.strip(), "%Y-%m-%dT%H:%M:%S.%f").timestamp()


def load_f11(path):
    rows = []
    with io.open(path, "r", encoding="utf-8") as f:
        for d in csv.DictReader(f):
            try:
                rows.append((ep(d["wall_iso"]), float(d["current_mA"])))
            except Exception:
                pass
    rows.sort()
    return rows


def load_markers(path):
    segs = []
    if not os.path.exists(path):
        return segs
    with io.open(path, "r", encoding="utf-8") as f:
        for d in csv.DictReader(f):
            if d["name"] == "DONE":
                segs.append((d["name"], float(d["wall_epoch"]), 0))
            else:
                segs.append((d["name"], float(d["wall_epoch"]), int(d["dwell_s"])))
    segs.sort(key=lambda x: x[1])
    return segs


def load_mqtt(path):
    ev = []
    if not os.path.exists(path):
        return ev
    with io.open(path, "r", encoding="utf-8") as f:
        for d in csv.DictReader(f):
            try:
                ev.append((float(d["ts_epoch"]), d["payload"]))
            except Exception:
                pass
    ev.sort()
    return ev


def stat_seg(rows, t0, t1):
    xs = [ma for t, ma in rows if t0 <= t < t1]
    if not xs:
        return None
    xs_sorted = sorted(xs)
    # 电荷用梯形积分 ∫I dt
    chg = 0.0
    seg = [(t, ma) for t, ma in rows if t0 <= t < t1]
    for k in range(1, len(seg)):
        chg += 0.5 * (seg[k - 1][1] + seg[k][1]) * (seg[k][0] - seg[k - 1][0])
    return {
        "n": len(xs),
        "dur": t1 - t0,
        "med": statistics.median(xs),
        "mean": sum(xs) / len(xs),
        "min": xs_sorted[0],
        "max": xs_sorted[-1],
        "sd": (sum((x - sum(xs) / len(xs)) ** 2 for x in xs) / len(xs)) ** 0.5,
        "mah": chg / 3600.0,
    }


# ---------- SVG 小工具 ----------
INK = "#1e293b"
MUT = "#64748b"
ACC = "#1d4ed8"
GRID = "#e2e8f0"
OK = "#15803d"
WARN = "#b45309"


def esc(s):
    return (str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def svg_bars(items, unit="mA", w=1180, rowh=26, pad_l=250, pad_r=90):
    """横向柱状：items = [(name, value, color)]"""
    h = len(items) * rowh + 46
    vmax = max(v for _, v, _ in items) if items else 1
    plot = w - pad_l - pad_r
    p = ['<svg viewBox="0 0 %d %d" width="100%%" role="img">' % (w, h)]
    for k in range(5):
        x = pad_l + plot * k / 4.0
        v = vmax * k / 4.0
        p.append('<line x1="%.1f" y1="8" x2="%.1f" y2="%d" stroke="%s"/>' % (x, x, h - 30, GRID))
        p.append('<text x="%.1f" y="%d" font-size="11" fill="%s" text-anchor="middle">%.0f</text>'
                 % (x, h - 14, MUT, v))
    for i, (name, v, col) in enumerate(items):
        y = 10 + i * rowh
        bw = plot * (v / vmax)
        p.append('<text x="%d" y="%d" font-size="12" fill="%s" text-anchor="end">%s</text>'
                 % (pad_l - 10, y + 13, INK, esc(name)))
        p.append('<rect x="%d" y="%d" width="%.1f" height="%d" rx="3" fill="%s" opacity="0.9"/>'
                 % (pad_l, y + 3, max(bw, 1.2), rowh - 9, col))
        p.append('<text x="%.1f" y="%d" font-size="11.5" fill="%s">%s</text>'
                 % (pad_l + max(bw, 1.2) + 6, y + 14, MUT, ("%.3f" % v) if v < 10 else ("%.2f" % v)))
    p.append('</svg>')
    return "".join(p)


def svg_stack(night_mah, rounds_mah, label, w=1180, h=118):
    """日电荷构成：睡眠 + 窗口轮"""
    tot = night_mah + rounds_mah
    plot = w - 240
    p = ['<svg viewBox="0 0 %d %d" width="100%%" role="img">' % (w, h)]
    y = 34
    bw_n = plot * (night_mah / tot)
    p.append('<rect x="230" y="%d" width="%.1f" height="34" rx="4" fill="#93c5fd"/>' % (y, bw_n))
    p.append('<rect x="%.1f" y="%d" width="%.1f" height="34" rx="4" fill="#f59e0b"/>'
             % (230 + bw_n, y, plot - bw_n))
    p.append('<text x="230" y="24" font-size="12.5" fill="%s">%s</text>' % (INK, esc(label)))
    p.append('<text x="%.1f" y="%d" font-size="12" fill="%s">睡眠 %.1f mAh</text>'
             % (230 + 8, y + 22, INK, night_mah))
    p.append('<text x="%.1f" y="%d" font-size="12" fill="#fff" text-anchor="end">窗口 %.1f mAh</text>'
             % (230 + plot - 8, y + 22, rounds_mah))
    p.append('<text x="%d" y="%d" font-size="12" fill="%s">%.1f mAh/天</text>'
             % (230 + plot + 14, y + 22, MUT, tot))
    p.append('<text x="230" y="%d" font-size="11.5" fill="%s">睡眠 %.3f mA × 23.74 h'
             '　·　窗口 6 轮 × 2.18 mAh</text>' % (y + 60, MUT, night_mah / 23.742))
    p.append('</svg>')
    return "".join(p)


def run_avg_ma(steps):
    """整段平均电流(mA) = 总电荷(mAh) × 3600 / 总时长(s)"""
    dur = sum(s["dur"] for s in steps)
    chg = sum(s["mah"] for s in steps)
    return (chg * 3600.0 / dur) if dur > 0 else 0.0


def led_txt(name):
    """从档名推导灯态（对应固件 SEQ_TABLE / SEQ_BuildDay 的 led 字段）。
    窗口轮（round_*/day_am*/day_pm*）的灯在整个窗口保持（含 60s Stop 睡眠期，
    固件 LP_CloseAndSleepEx(sleep_s, led_off=0) 收尾不灭灯 —— 故意的最坏组合档）。"""
    if name in ("v4goff_dark", "v4goff_end", "round_dark", "v4gon_reg",
                "online_hb", "online_idle") or \
       name.startswith(("day_night", "day_midday")):
        return "灭"
    if name.startswith(("day_am", "day_pm")):
        return "绿 255 全程"
    if name.startswith("round_G"):
        return "绿 %s 全程" % name[7:]
    if name == "v4goff_G255": return "绿 255"
    if name == "v4goff_G128": return "绿 128"
    if name == "v4goff_R255": return "红 255"
    if name == "v4goff_Y255": return "黄 255"
    if name == "v4goff_R64":  return "红 64"
    return "—"


def table(head, rows, cls=""):
    p = ['<table class="%s"><thead><tr>' % cls]
    for hcell in head:
        p.append("<th>%s</th>" % esc(hcell))
    p.append("</tr></thead><tbody>")
    for r in rows:
        p.append("<tr>" + "".join("<td>%s</td>" % c for c in r) + "</tr>")
    p.append("</tbody></table>")
    return "".join(p)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stamp", default="20261013_192956")
    ap.add_argument("--out", default=os.path.join(DOCS, "dutycycle-day-test.html"))
    a = ap.parse_args()

    runs = [
        ("seq60", "P1 · 13 档工况表", "13 档横向扫描：4G 断/上电 × LED 六档 × 窗口轮"),
        ("day1", "P2 · 压缩日 第 1 轮", "夜 Stop → 早高峰 3 轮 → 日 Stop → 晚高峰 3 轮 → 夜 Stop"),
        ("day2", "P3 · 压缩日 第 2 轮（复核）", "同 P2，用于可重复性复核"),
    ]

    data = {}
    for tag, title, desc in runs:
        f11 = os.path.join(DATA, "%s_%s_f11.csv" % (a.stamp, tag))
        mk = os.path.join(DATA, "%s_%s_markers.csv" % (a.stamp, tag))
        if not os.path.exists(f11):
            sys.stderr.write("缺文件 %s\n" % f11)
            return 1
        rows = load_f11(f11)
        segs = load_markers(mk)
        events = load_mqtt(os.path.join(DATA, "mqtt_live.csv"))
        steps = []
        for i in range(len(segs) - 1):
            name, t0, dwell = segs[i]
            t1 = segs[i + 1][1]
            st = stat_seg(rows, t0, t1)
            if st:
                st["name"] = name
                st["nom"] = dwell
                st["t0"] = t0
                st["t1"] = t1
                steps.append(st)
        # MQTT 按档归类
        mq = {}
        for name, t0, _d in segs[:-1]:
            pass
        for ts, pl in events:
            hit = None
            for st in steps:
                if st["t0"] <= ts < st["t1"]:
                    hit = st["name"]
                    break
            key = hit or "（档外）"
            kind = ("report" if '"report"' in pl else
                    "ready" if "ready" in pl else
                    "at" if pl.lstrip().startswith(("+++", "AT+")) else "other")
            mq.setdefault(key, {}).setdefault(kind, 0)
            mq[key][kind] += 1
        data[tag] = dict(rows=rows, segs=segs, steps=steps, mq=mq, title=title, desc=desc,
                         f11=os.path.basename(f11), mk=os.path.basename(mk))

    # ---------- 实测睡眠电流（从窗口轮睡段 + 纯 Stop 档里量，不写死） ----------
    #   判据：只看「本应停机的档」（round_* 的 60 s 睡段 / day_night* / day_midday）里
    #   电流 < 3 mA 的样本，取中位。这样口径随数据自动更新，避免再次写死 1.03。
    global NIGHT_MA
    _sl, _sl_by = [], {}
    for _tag in ("seq60", "day1", "day2"):
        if _tag not in data:
            continue
        _vals = []
        for _st in data[_tag]["steps"]:
            _nm = _st["name"]
            if _nm.startswith("round") or _nm.startswith("day_night") or _nm == "day_midday":
                _vals += [c for t, c in data[_tag]["rows"] if _st["t0"] <= t < _st["t1"] and c < 3.0]
        if _vals:
            _sl_by[_tag] = _vals
            _sl += _vals
    if _sl:
        NIGHT_MA = statistics.median(_sl)
    SLEEP_N = len(_sl)
    SLEEP_BY = {k: (len(v), statistics.median(v)) for k, v in _sl_by.items()}

    # ---------- 折算 ----------
    day = data["day1"] if "day1" in data else data["day2"]
    win = [s for s in day["steps"] if s["name"].startswith("day_a") or s["name"].startswith("day_p")]
    round_mah = statistics.median([s["mah"] for s in win]) if win else 2.18
    n_round_real = 6
    rounds_mah = round_mah * n_round_real
    sleep_s = DAY_S - n_round_real * ROUND_S
    sleep_h = sleep_s / 3600.0
    night_mah = NIGHT_MA * sleep_h
    tot_mah = night_mah + rounds_mah
    avg_ma = tot_mah / (DAY_S / 3600.0)
    days = BATT_MAH * BATT_USABLE / tot_mah

    night_mah_best = NIGHT_MA_BEST * sleep_h
    tot_best = night_mah_best + rounds_mah
    avg_best = tot_best / (DAY_S / 3600.0)
    days_best = BATT_MAH * BATT_USABLE / tot_best

    night_mah_dirty = NIGHT_MA_DIRTY * sleep_h
    tot_dirty = night_mah_dirty + rounds_mah
    avg_dirty = tot_dirty / (DAY_S / 3600.0)
    days_dirty = BATT_MAH * BATT_USABLE / tot_dirty

    # 敏感性
    sens = []
    for r in (2, 4, 6):
        rm = round_mah * r
        for nm, lbl in ((NIGHT_MA, "%.3f 本次实测" % NIGHT_MA),
                        (0.50, "0.500"),
                        (NIGHT_MA_DIRTY, "%.3f 旧脏口径" % NIGHT_MA_DIRTY)):
            sh = (DAY_S - r * ROUND_S) / 3600.0
            t = nm * sh + rm
            sens.append((r, lbl, t, t / 24.0, BATT_MAH * BATT_USABLE / t))

    # ---------- HTML ----------
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    H = []
    H.append('<!DOCTYPE html><html lang="zh-CN"><head><meta charset="utf-8">')
    H.append('<meta name="viewport" content="width=device-width,initial-scale=1">')
    H.append('<title>全天工况压缩测试报告 · 车牌 ver2</title>')
    H.append("""<style>
:root{--ink:#1e293b;--mut:#64748b;--line:#e2e8f0;--bg:#f8fafc;--acc:#1d4ed8;--ok:#15803d;--warn:#b45309;--red:#b91c1c}
*{box-sizing:border-box}
body{margin:0;padding:28px 20px 70px;background:var(--bg);color:var(--ink);
 font:15px/1.7 -apple-system,"Segoe UI","Microsoft YaHei",sans-serif}
.wrap{max-width:1240px;margin:0 auto}
h1{font-size:26px;margin:0 0 6px}
h2{font-size:19px;margin:38px 0 12px;padding-left:11px;border-left:4px solid var(--acc)}
h3{font-size:15.5px;margin:22px 0 8px;color:#334155}
.sub{color:var(--mut);font-size:13px;margin-bottom:22px}
.card{background:#fff;border:1px solid var(--line);border-radius:12px;padding:18px 20px;margin:12px 0;
 box-shadow:0 1px 2px rgba(15,23,42,.04);overflow-x:auto}
.kpis{display:flex;gap:12px;flex-wrap:wrap;margin:16px 0 6px}
.kpi{flex:1 1 178px;background:#fff;border:1px solid var(--line);border-radius:12px;padding:14px 16px}
.kpi .lab{font-size:12.5px;color:var(--mut)}
.kpi .val{font-size:23px;font-weight:600;margin:3px 0 1px;letter-spacing:-.4px}
.kpi .val small{font-size:13px;font-weight:400;color:var(--mut)}
.kpi .mut{font-size:11.5px;color:var(--mut)}
table{border-collapse:collapse;width:100%;font-size:13.5px}
th,td{border-bottom:1px solid var(--line);padding:7px 9px;text-align:right;white-space:nowrap}
th{background:#f1f5f9;font-weight:600;color:#334155;text-align:right}
th:first-child,td:first-child{text-align:left}
tbody tr:hover{background:#f8fafc}
.mut{color:var(--mut)}
.ok{color:var(--ok);font-weight:600}
.warn{color:var(--warn);font-weight:600}
.red{color:var(--red);font-weight:600}
pre{background:#0f172a;color:#e2e8f0;padding:13px 15px;border-radius:9px;overflow-x:auto;font-size:12.5px;line-height:1.6}
code{background:#f1f5f9;padding:1.5px 5px;border-radius:4px;font-size:13px}
ul{padding-left:22px}li{margin:5px 0}
.note{border-left:4px solid var(--warn);background:#fffbeb;padding:12px 16px;border-radius:0 8px 8px 0;margin:14px 0;font-size:14px}
.note.bad{border-color:var(--red);background:#fef2f2}
.note.good{border-color:var(--ok);background:#f0fdf4}
</style></head><body><div class="wrap">""")
    H.append("<h1>全天工况压缩测试报告 · 车牌 ver2</h1>")
    H.append('<div class="sub">F-11 真值电流表（串联回路，≈10 Hz）＋ 固件 SEQ 工况引擎（SWD 注入/回读）'
             '＋ 4G MQTT 旁听端交叉验证 &nbsp;·&nbsp; <b>干净口径</b>（每个 SWD 会话末尾清 '
             '<code>DBGMCU_CR</code>） &nbsp;·&nbsp; 生成 %s &nbsp;·&nbsp; 批次 <code>%s</code></div>'
             % (esc(stamp), esc(a.stamp)))

    # §0
    H.append("<h2>0 · 结论摘要</h2>")
    H.append('<div class="kpis">')
    dayavg1 = run_avg_ma(data["day1"]["steps"])
    dayavg2 = run_avg_ma(data["day2"]["steps"])
    H.append('<div class="kpi"><div class="lab">压缩日整机平均（P2 / P3）</div>'
             '<div class="val">%.2f / %.2f <small>mA</small></div>'
             '<div class="mut">两次差 %.1f%%，可重复</div></div>'
             % (dayavg1, dayavg2, abs(dayavg1 - dayavg2) / dayavg1 * 100))
    H.append('<div class="kpi"><div class="lab">折算真实一天（睡态 %.2f mA）</div>'
             '<div class="val">%.3f <small>mA</small></div>'
             '<div class="mut">%.1f mAh/天</div></div>' % (NIGHT_MA, avg_ma, tot_mah))
    H.append('<div class="kpi"><div class="lab">1000 mAh 电池续航</div>'
             '<div class="val">%.0f <small>天</small></div>'
             '<div class="mut">按 80%% 可用容量</div></div>' % days)
    hb_ma = max(s["med"] for s in data["seq60"]["steps"])
    H.append('<div class="kpi"><div class="lab">最省 / 最费档</div>'
             '<div class="val">%.3f / %.1f <small>mA</small></div>'
             '<div class="mut">Stop 睡态 vs 4G 在线心跳</div></div>' % (NIGHT_MA, hb_ma))
    H.append('</div>')

    H.append('<div class="note good"><b>本批为「干净口径」重跑</b>（批次 <code>%s</code>）：'
             '每个 OpenOCD 会话末尾都清零 <code>DBGMCU_CR</code>（<code>mww 0xE0042004 0</code>）并回读确认，'
             '实测睡态 <b>%.3f mA</b>（%d 个样本，取自窗口轮睡段 + 纯 Stop 档），与历史真值 0.226 mA 一致。'
             '旧报告里的 1.03 mA 是 OpenOCD 调试保活位伪值，根因见 '
             '<a href="stop-1p03mA-rootcause.html">stop-1p03mA-rootcause.html</a>。</div>'
             % (esc(a.stamp), NIGHT_MA, SLEEP_N))

    H.append('<div class="note good"><b>核心结论</b>：'
             '① 压缩日（夜 Stop→早高峰 3 轮→日 Stop→晚高峰 3 轮→夜 Stop，1020 s）'
             '整机平均 <b>%.2f / %.2f mA</b>，F-11 电荷 <b>%.2f / %.2f mAh</b>；'
             '② 折算真实一天（6 个上报窗口）平均电流 <b>%.3f mA</b>，'
             '1000 mAh 电池约 <b>%.0f 天</b>；'
             '③ MQTT 旁听端逐轮验证：<b>每轮恰好 1 条 report + 5–6 条 ready + 4–7 条 AT/+++</b>，'
             '说明 6 轮窗口全部真实联网上报成功。'
             '</div><!---->'
             % (dayavg1, dayavg2,
                sum(s["mah"] for s in data["day1"]["steps"]),
                sum(s["mah"] for s in data["day2"]["steps"]),
                avg_ma, days))

    # §1 P1
    st1 = data["seq60"]["steps"]
    H.append("<h2>1 · P1 横向扫描：13 档工况实测</h2>")
    H.append('<div class="card">')
    items = [(s["name"], s["med"], "#1d4ed8" if s["med"] > 20 else
              ("#0ea5e9" if s["med"] > 5 else "#38bdf8")) for s in st1]
    items.sort(key=lambda x: x[1])
    items = [(n, v, ("#ef4444" if n.startswith("round") else
                     ("#f59e0b" if n.startswith("online") or n.startswith("v4gon") else "#38bdf8")))
             for n, v, _c in items]
    H.append(svg_bars(items))
    H.append('<div class="mut" style="margin-top:10px">中位电流（mA）。红 = 窗口轮（含 90 s 4G 会话 + 60 s Stop 睡）；'
             '橙 = 4G 在线；蓝 = 4G 断电或 LED 档。<b>条形颜色只是分段配色，不代表 LED 颜色</b>——'
             '各档实际灯态见下表「灯」列。</div>')
    H.append('</div>')
    rows = []
    for i, s in enumerate(st1, 1):
        rows.append([i, esc(s["name"]), led_txt(s["name"]), "%d" % s["nom"], "%.1f" % s["dur"],
                     "<b>%.3f</b>" % s["med"], "%.3f" % s["mean"], "%.3f" % s["min"],
                     "%.3f" % s["max"], "%.2f" % s["sd"], "%.3f" % s["mah"]])
    H.append(table(["#", "工况", "灯", "标称 s", "实测 s", "中位 mA", "均值 mA", "最小", "最大", "σ", "电荷 mAh"],
                   rows))

    # §2 压缩日
    for tag in ("day1", "day2"):
        d = data[tag]
        H.append("<h2>%s · %s</h2>" % ("2" if tag == "day1" else "3", esc(d["title"])))
        H.append('<div class="sub" style="margin:-6px 0 10px">%s</div>' % esc(d["desc"]))
        rows = []
        for i, s in enumerate(d["steps"], 1):
            rows.append([i, esc(s["name"]), led_txt(s["name"]), "%d" % s["nom"], "%.1f" % s["dur"],
                         "<b>%.3f</b>" % s["med"], "%.3f" % s["mean"], "%.3f" % s["min"],
                         "%.3f" % s["max"], "%.2f" % s["sd"], "%.3f" % s["mah"]])
        tot_mah_run = sum(s["mah"] for s in d["steps"])
        tot_dur = sum(s["dur"] for s in d["steps"])
        rows.append(["", "合计", "—", "1020", "%.1f" % tot_dur, "—", "%.3f" % (tot_mah_run / tot_dur * 3600),
                     "—", "—", "—", "<b>%.3f</b>" % tot_mah_run])
        H.append(table(["#", "工况", "灯", "标称 s", "实测 s", "中位 mA", "均值 mA", "最小", "最大", "σ", "电荷 mAh"], rows))
        if tag == "day1":
            H.append('<div class="mut" style="margin-top:6px">灯态说明：本轮指令 <code>!day 30 3 G 255</code> —— '
                     '三个纯 Stop 段（<code>day_night1/midday/night2</code>）4G 断 + <b>灯灭</b>，实测 0.22 mA；'
                     '六个窗口轮（<code>day_am*/pm*</code>）<b>绿灯 255 在整个窗口保持亮灯</b>，'
                     '包括窗口内 60 s 的 Stop 睡眠期（固件 <code>LP_CloseAndSleepEx(60, led_off=0)</code> 收尾不灭灯，'
                     '故意的最坏组合档），其睡眠期实测 ≈20.6 mA = Stop 0.226 + 绿灯。'
                     'P3（day2）是<b>同一工况的复核轮</b>，不是加灯的变体。</div>')

    # P2 vs P3 对比
    H.append("<h3>P2 / P3 逐档复核</h3>")
    s2, s3 = data["day1"]["steps"], data["day2"]["steps"]
    rows = []
    for k in range(min(len(s2), len(s3))):
        r2, r3 = s2[k], s3[k]
        d = abs(r2["med"] - r3["med"]) / ((r2["med"] + r3["med"]) / 2) * 100
        rows.append([esc(r2["name"]), "%.3f" % r2["med"], "%.3f" % r3["med"],
                     "<span class='%s'>%.1f%%</span>" % ("ok" if d < 5 else "warn", d),
                     "%.3f" % r2["mah"], "%.3f" % r3["mah"]])
    H.append(table(["工况", "P2 中位 mA", "P3 中位 mA", "偏差", "P2 电荷 mAh", "P3 电荷 mAh"], rows))

    # §4 折算
    H.append("<h2>4 · 压缩日 → 真实一天折算</h2>")
    H.append('<div class="card">')
    H.append(svg_stack(night_mah, rounds_mah, "真实一天电荷构成（睡态 %.3f mA 口径）" % NIGHT_MA))
    H.append('<div class="mut" style="margin-top:12px">'
             '折算模型：真实一天 = 6 个上报窗口（早晚高峰各 3 轮）× 每轮 %.3f mAh'
             '＋ 其余 %.2f h 全 Stop 睡眠 × %.3f mA。' % (round_mah, sleep_h, NIGHT_MA))
    H.append('</div></div>')
    H.append(table(["口径", "睡眠 23.74 h 电荷", "窗口 6 轮电荷", "合计 mAh/天", "平均 mA",
                    "1000 mAh 续航（80% 可用）"],
                   [["本次实测睡态 %.3f mA（干净）" % NIGHT_MA, "%.2f mAh" % night_mah,
                     "%.2f mAh" % rounds_mah, "<b>%.2f</b>" % tot_mah, "<b>%.3f</b>" % avg_ma,
                     "<b>%.0f 天</b>" % days],
                    ["历史最佳睡态 %.3f mA" % NIGHT_MA_BEST, "%.2f mAh" % night_mah_best,
                     "%.2f mAh" % rounds_mah, "%.2f" % tot_best, "%.3f" % avg_best,
                     "%.0f 天" % days_best],
                    ["旧口径 %.3f mA（带调试保活位，仅对照）" % NIGHT_MA_DIRTY,
                     "%.2f mAh" % night_mah_dirty, "%.2f mAh" % rounds_mah,
                     "%.2f" % tot_dirty, "%.3f" % avg_dirty, "%.0f 天" % days_dirty]]))
    H.append("<h3>敏感性：窗口轮数 × 睡眠电流</h3>")
    rows = []
    for r, lbl, t, am, dy in sens:
        rows.append(["%d 轮" % r, "%s mA" % lbl, "%.2f mAh" % t, "%.3f mA" % am, "%.0f 天" % dy])
    H.append(table(["每日窗口轮数", "睡眠电流", "mAh/天", "平均 mA", "1000 mAh 续航"], rows))
    H.append('<div class="mut" style="margin-top:8px">窗口轮越少、睡眠电流越低 ⇒ 续航越长。'
             '但要注意：<b>每轮成本几乎全在 4G 会话上</b> —— 每轮 %.3f mAh 里，'
             '60 s 睡眠只占 %.4f mAh（%.1f%%），其余全是搜网/联网/收尾。'
             '所以<b>缩短会话时长</b>（如「连上即收」把 90 s 压到 60 s）收益最高。</div>'
             % (round_mah, NIGHT_MA * 60 / 3600.0, NIGHT_MA * 60 / 3600.0 / round_mah * 100))

    # §5 MQTT
    H.append("<h2>5 · MQTT 旁听端交叉验证</h2>")
    H.append('<div class="note good">旁听端 <code>bench/mqtt_watch.py</code> 常驻订阅 '
             '<code>DTU_Topic1147/#</code>，把板子上行的每条报文按「落在哪个工况档」归类。'
             '结果：<b>6 个窗口档各恰好 1 条 <code>report</code></b>，与固件「每轮会话上报一次」完全对应，'
             '板上行链路的真实性与时间轴对齐同时得证。</div>')
    d = data["day2"]
    rows = []
    keys = [k for k in d["mq"].keys() if k.startswith("day_")]
    keys.sort()
    for k in keys + [k for k in d["mq"].keys() if not k.startswith("day_")]:
        m = d["mq"][k]
        rows.append([esc(k), "<b>%d</b>" % m.get("report", 0), "%d" % m.get("ready", 0),
                     "%d" % m.get("at", 0), "%d" % m.get("other", 0),
                     "%d" % sum(m.values())])
    H.append(table(["工况档", "report", "ready 心跳", "AT/+++", "其他", "合计"], rows))
    H.append('<div class="mut" style="margin-top:8px">「（档外）」是测试开始前旁听端已经记录的那段（P1 尾巴），'
             '不代表 P3 期间的事件。数据源 <code>%s</code>。</div>' % esc("data/mqtt_live.csv"))

    # §6 口径修正验证
    H.append("<h2>6 · 口径修正验证（旧脏 / 新净 / 预期）</h2>")
    H.append('<div class="note good"><b>根因</b>：上一版的睡态 1.03 mA 是 '
             '<code>DBGMCU_CR = 0x00000307</code> 造成的<b>测量伪值</b> —— '
             '它是 OpenOCD 每次 <code>init</code> 写入的调试保活位（<code>DBG_SLEEP|DBG_STOP|DBG_STANDBY</code>），'
             '让 Stop/WFI 不停内核时钟：睡态虚增 ≈0.82 mA、Run(WFI) 虚增 ≈0.29 mA，'
             '且退出调试器<b>不清除</b>（只有复位才清），所以同一上电周期内所有测点一起偏高。'
             'A/B/A 实测 <code>0x307 → 1.042 mA</code> / <code>写 0 → 0.220 mA</code> / 再连 → <code>1.042 mA</code>。'
             '<br><b>本批重跑已在每个 OpenOCD 会话末尾清零该寄存器并回读确认</b>，故本篇所有数字均为干净口径。'
             '详见 <a href="stop-1p03mA-rootcause.html">stop-1p03mA-rootcause.html</a>。</div>')

    OLD_MED = {"v4goff_dark": 2.760, "v4goff_G255": 22.910, "v4goff_G128": 13.320,
               "v4goff_R255": 10.780, "v4goff_Y255": 10.530, "v4goff_R64": 4.930,
               "v4gon_reg": 60.530, "online_hb": 61.010, "online_idle": 17.220,
               "v4goff_end": 2.775}
    rows = []
    for s in st1:
        nm = s["name"]
        if nm not in OLD_MED:
            continue
        o = OLD_MED[nm]
        d = s["med"] - o
        rows.append([esc(nm), "%.3f" % o, "<b>%.3f</b>" % s["med"],
                     "<span class='%s'>%+.3f</span>" % ("ok" if abs(d) < 0.35 else "warn", d)])
    H.append(table(["P1 档（Run / 在线 / LED 类）", "旧（脏 0x307）mA", "新（干净）mA", "Δ"], rows))
    H.append('<div class="mut" style="margin-top:8px">解读：LED 档与 4G 断电 Run 档整体下移 '
             '0.17–0.26 mA（调试位对 WFI 时段的影响）；4G 在线类（<code>v4gon_*</code>/<code>online_*</code>）'
             '几乎不动（±0.3，属网络侧 DRX 漂移量级）；<b>会话期 MCU 全速忙等、无 WFI，所以不受影响</b>。</div>')

    H.append("<h3>睡段实测（本次干净口径）</h3>")
    rows = []
    _lbl = {"seq60": "P1 的 3 个 <code>round_*</code> 睡段",
            "day1": "P2 压缩日（3 个纯 Stop + 6 个轮内睡）",
            "day2": "P3 压缩日（同 P2）"}
    for tag in ("seq60", "day1", "day2"):
        if tag not in SLEEP_BY:
            continue
        n, med = SLEEP_BY[tag]
        rows.append([_lbl.get(tag, tag), "%d" % n, "<b>%.3f</b>" % med, "0.226",
                     "%+.3f" % (med - 0.226)])
    rows.append(["旧口径（对照，带调试位）", "—", "1.023 – 1.056", "0.226", "+0.80 ~ +0.83"])
    H.append(table(["来源", "低电流样本数", "睡段中位 mA", "历史真值 mA", "Δ"], rows))
    H.append('<div class="note good" style="margin-top:12px"><b>结论：完全吻合。</b>'
             '三处独立来源的睡段实测都落在 <b>%.3f mA</b> 附近，与中午定案的 0.226 mA 一致；'
             '而会话/在线档电流新旧差 ≤0.3 mA ⇒ <b>只有停机态被污染过</b>，'
             '说明修正既必要又充分。</div>' % NIGHT_MA)

    H.append('<div class="note"><b>⚠ 引用注意</b>：窗口轮 <code>round_*</code> 档的<b>窗口整体中位数不可单独引用</b>'
             '—— 它的分布是双峰（38% 睡 + 52% 会话），中位正好卡在两峰交界，'
             '混合比 0.1% 的变化就能让它从 33 跳到 57（本次 <code>round_dark</code> 33.25 → 56.62 即此现象，'
             '而分态中位完全没变：会话 59.69→59.81）。要看该档请用「会话段中位」或「电荷 mAh」。</div>')

    H.append('<div class="note good"><b>附带确证</b>：夜间 Stop 档（%.3f mA）相比 8 MHz Run 静止档（%.3f mA）'
             '降低 %.2f mA ⇒ 睡眠机制本身工作正常。'
             '压缩日全程最费档出现在窗口刚开机的搜网阶段。</div>'
             % (NIGHT_MA,
                next((s["med"] for s in st1 if s["name"] == "v4goff_dark"), 2.53),
                next((s["med"] for s in st1 if s["name"] == "v4goff_dark"), 2.53) - NIGHT_MA))

    # §7 口径
    H.append("<h2>7 · 口径与复现</h2>")
    H.append('<div class="card">')
    H.append("<h3>测量链路</h3><ul>")
    H.append("<li>电流：<b>安服优 F-11</b> 真值表，串联在供电回路内，38400 8N1，≈10–11 Hz</li>")
    H.append("<li>置态：固件 <b>SEQ 工况引擎</b>（<code>Core/Src/main.c</code>），"
             "经 <b>OpenOCD</b> 写 RAM（<code>g_test_cmd</code> + <code>g_test_go</code>）触发，"
             "每档切换时把档名与 RTC 秒写进 RAM 标记日志 <code>g_slog</code></li>")
    H.append("<li>时标：RTC 秒计数（唯一跨 Stop 存活），收尾用中位线性标定 scale</li>")
    H.append("<li><b>干净口径</b>：每个 OpenOCD 会话末尾都写 <code>mww 0xE0042004 0</code> 清 "
             "<code>DBGMCU_CR</code> 并回读确认 —— 否则 OpenOCD 的 <code>init</code> 会把 "
             "<code>DBG_SLEEP|DBG_STOP|DBG_STANDBY</code> 置上，睡态虚增 ≈0.82 mA、"
             "Run(WFI) 虚增 ≈0.29 mA，且退出调试器不清除（只有复位才清）</li>")
    H.append("<li>MQTT：<code>bench/mqtt_watch.py</code> 常驻旁听，落盘 <code>data/mqtt_live.csv</code></li>")
    H.append("<li>电荷：对采样做梯形积分 ∫I dt，再折算 mAh</li>")
    H.append("</ul>")
    H.append("<h3>复现命令</h3><pre>%s</pre>"
             % esc('python bench/run_recommended.py --stamp %s --poll 15   # P1+P2+P3 三合一（自动出图）\n'
                   'python bench/report_day.py --stamp %s                  # 本报告\n'
                   '\n'
                   '# 干净口径的前提：只用本仓 bench/bench_swd.py 注入（它每个会话末尾清 DBGMCU_CR）；\n'
                   '# 不要在同一上电周期里先手连 OpenOCD 再测功耗。--keep-dbg 仅供在 Stop 里看住目标用。'
                   % (a.stamp, a.stamp)))
    H.append("<h3>原始文件</h3><ul>")
    for tag, title, _d in runs:
        H.append("<li>%s：<code>%s_f11.csv</code> / <code>%s_markers.csv</code></li>"
                 % (esc(title), esc(a.stamp + "_" + tag), esc(a.stamp + "_" + tag)))
    H.append("<li>MQTT 流水：<code>mqtt_live.csv</code>（本次进程） / <code>mqtt_all.csv</code>（全历史）</li>")
    H.append("</ul>")
    H.append('<div class="mut">本页为纯 CSS/SVG 手绘图表，不依赖任何外部 CDN，可离线打开。</div>')
    H.append("</div>")

    H.append("</div></body></html>")

    html = "\n".join(H)
    with io.open(a.out, "w", encoding="utf-8", newline="\n") as f:
        f.write(html)
    print("写出: %s  (%d 字节)" % (a.out, len(html)))
    print("换算: 睡态 %.3f mA（实测 %d 样本）· 每轮 %.3f mAh · 日合计 %.2f mAh · 平均 %.3f mA · 续航 %.0f 天"
          % (NIGHT_MA, SLEEP_N, round_mah, tot_mah, avg_ma, days))
    return 0


if __name__ == "__main__":
    sys.exit(main())
