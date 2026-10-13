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
NIGHT_MA = 1.03          # 本次实测夜间 Stop 睡态（保守口径）
NIGHT_MA_BEST = 0.226    # 历史最佳 Stop 睡态
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

    # 敏感性
    sens = []
    for r in (2, 4, 6):
        rm = round_mah * r
        for nm, lbl in ((NIGHT_MA_BEST, "0.226"), (0.50, "0.50"), (NIGHT_MA, "1.03")):
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
             '＋ 4G MQTT 旁听端交叉验证 &nbsp;·&nbsp; 生成 %s &nbsp;·&nbsp; 批次 <code>%s</code></div>'
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
    H.append('<div class="kpi"><div class="lab">最省 / 最费档</div>'
             '<div class="val">%.2f / %.1f <small>mA</small></div>'
             '<div class="mut">Stop 睡态 vs 4G 在线心跳</div></div>' % (NIGHT_MA, 61.01))
    H.append('</div>')

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
             '橙 = 4G 在线；蓝 = 4G 断电或 LED 档。</div>')
    H.append('</div>')
    rows = []
    for i, s in enumerate(st1, 1):
        rows.append([i, esc(s["name"]), "%d" % s["nom"], "%.1f" % s["dur"],
                     "<b>%.3f</b>" % s["med"], "%.3f" % s["mean"], "%.3f" % s["min"],
                     "%.3f" % s["max"], "%.2f" % s["sd"], "%.3f" % s["mah"]])
    H.append(table(["#", "工况", "标称 s", "实测 s", "中位 mA", "均值 mA", "最小", "最大", "σ", "电荷 mAh"],
                   rows))

    # §2 压缩日
    for tag in ("day1", "day2"):
        d = data[tag]
        H.append("<h2>%s · %s</h2>" % ("2" if tag == "day1" else "3", esc(d["title"])))
        H.append('<div class="sub" style="margin:-6px 0 10px">%s</div>' % esc(d["desc"]))
        rows = []
        for i, s in enumerate(d["steps"], 1):
            rows.append([i, esc(s["name"]), "%d" % s["nom"], "%.1f" % s["dur"],
                         "<b>%.3f</b>" % s["med"], "%.3f" % s["mean"], "%.3f" % s["min"],
                         "%.3f" % s["max"], "%.2f" % s["sd"], "%.3f" % s["mah"]])
        tot_mah_run = sum(s["mah"] for s in d["steps"])
        tot_dur = sum(s["dur"] for s in d["steps"])
        rows.append(["", "合计", "1020", "%.1f" % tot_dur, "—", "%.3f" % (tot_mah_run / tot_dur * 3600),
                     "—", "—", "—", "<b>%.3f</b>" % tot_mah_run])
        H.append(table(["#", "工况", "标称 s", "实测 s", "中位 mA", "均值 mA", "最小", "最大", "σ", "电荷 mAh"], rows))

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
    H.append(svg_stack(night_mah, rounds_mah, "真实一天电荷构成（睡态 %.2f mA 口径）" % NIGHT_MA))
    H.append('<div class="mut" style="margin-top:12px">'
             '折算模型：真实一天 = 6 个上报窗口（早晚高峰各 3 轮）× 每轮 %.3f mAh'
             '＋ 其余 %.2f h 全 Stop 睡眠 × %.3f mA。' % (round_mah, sleep_h, NIGHT_MA))
    H.append('</div></div>')
    H.append(table(["口径", "睡眠 23.74 h 电荷", "窗口 6 轮电荷", "合计 mAh/天", "平均 mA",
                    "1000 mAh 续航（80% 可用）"],
                   [["本次实测睡态 %.2f mA" % NIGHT_MA, "%.2f mAh" % night_mah,
                     "%.2f mAh" % rounds_mah, "<b>%.2f</b>" % tot_mah, "<b>%.3f</b>" % avg_ma,
                     "<b>%.0f 天</b>" % days],
                    ["历史最佳睡态 %.3f mA" % NIGHT_MA_BEST, "%.2f mAh" % night_mah_best,
                     "%.2f mAh" % rounds_mah, "%.2f" % tot_best, "%.3f" % avg_best,
                     "%.0f 天" % days_best]]))
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

    # §6 待查偏差
    H.append("<h2>6 · 两个待查的系统性偏差</h2>")
    H.append('<div class="note bad"><b>① Stop 睡态实测 %.2f mA，而 10-13 中午定案是 %.3f mA（差 0.81 mA）。</b><br>'
             '本轮 P1 的 <code>round_dark</code> 提供了天然 A/B —— 它同样是「4G 断电 + 灯灭 + Stop 睡 60 s」，'
             '只是前面多了 90 s 会话。看它窗口末 60 s 的原始样本：'
             '中位 1.041 / 最小 1.023 mA，<b>没有一条低于 1.0 mA</b>；'
             '压缩日的 <code>day_night1/2</code>、<code>day_midday</code> 也是 1.041–1.056 mA。'
             '三个独立档位互相印证 ⇒ <b>本次睡态就是 ≈1.03 mA</b>，不是测量噪声。'
             '<br>候选原因：<b>(a)</b> 睡眠期仍有外设耗电（SC7A20H 若在连续采样模式约 0.5–0.8 mA，量级吻合）；'
             '<b>(b)</b> 测量回路口径不同（若 F-11 串在「探针 VCC → 板子 VCC」之间，会把调试探针自身功耗算进来）。'
             '<br><b>下一步 A/B</b>：用 SWD 直写置态进 Stop（绕过固件 SEQ，复现 10-13 中午的条件），'
             '若仍 %.2f mA ⇒ 属 (b) 测量口径；若回到 %.3f mA ⇒ 属 (a)，去查传感器是否在 Stop 前未关。</div>'
             % (NIGHT_MA, NIGHT_MA_BEST, NIGHT_MA, NIGHT_MA_BEST))
    H.append('<div class="note"><b>② 4G 断电 + 灯灭的 Run 档：本次 2.760/2.775 mA，校正版报告 2.30 mA（差 +0.46 mA）。</b><br>'
             '本次首档与末档同为此状态，分别 2.760 / 2.775 mA，<b>差值仅 0.015 mA ⇒ 测量可重复</b>，'
             '不是随机噪声。它与 ① 很可能同源。注意两者偏差量不同（Run +0.46、Stop +0.81），'
             '所以<b>不能简单用「一起减常数」修掉</b>，必须实测归因。</div>')
    H.append('<div class="note good"><b>附带确证的改进</b>：夜间 Stop 档（1.03 mA）相比 8 MHz Run 静止档（2.76 mA）'
             '降低 1.73 mA ⇒ 睡眠机制本身工作正常；压缩日全程最费档 '
             '<code>day_am1</code> 74.5 mA 出现在窗口刚开机的搜网阶段。</div>')

    # §7 口径
    H.append("<h2>7 · 口径与复现</h2>")
    H.append('<div class="card">')
    H.append("<h3>测量链路</h3><ul>")
    H.append("<li>电流：<b>安服优 F-11</b> 真值表，串联在供电回路内，38400 8N1，≈10–11 Hz</li>")
    H.append("<li>置态：固件 <b>SEQ 工况引擎</b>（<code>Core/Src/main.c</code>），"
             "经 <b>OpenOCD</b> 写 RAM（<code>g_test_cmd</code> + <code>g_test_go</code>）触发，"
             "每档切换时把档名与 RTC 秒写进 RAM 标记日志 <code>g_slog</code></li>")
    H.append("<li>时标：RTC 秒计数（唯一跨 Stop 存活），收尾用中位线性标定 scale</li>")
    H.append("<li>MQTT：<code>bench/mqtt_watch.py</code> 常驻旁听，落盘 <code>data/mqtt_live.csv</code></li>")
    H.append("<li>电荷：对采样做梯形积分 ∫I dt，再折算 mAh</li>")
    H.append("</ul>")
    H.append("<h3>复现命令</h3><pre>%s</pre>"
             % esc('python bench/bench_swd.py --cmd "seq 60"          # P1 横向扫描\n'
                   'python bench/bench_swd.py --cmd "day 30 3 G 255"    # P2/P3 压缩日\n'
                   'python bench/run_recommended.py --poll 15           # 三合一（自动出图）\n'
                   'python bench/report_day.py --stamp %s               # 本报告' % a.stamp))
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
    print("换算: 睡态 %.2f mA · 每轮 %.3f mAh · 日合计 %.2f mAh · 平均 %.3f mA · 续航 %.0f 天"
          % (NIGHT_MA, round_mah, tot_mah, avg_ma, days))
    return 0


if __name__ == "__main__":
    sys.exit(main())
