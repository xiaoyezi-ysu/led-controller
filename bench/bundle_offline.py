# -*- coding: utf-8 -*-
"""把 docs/ 下互相链接的多份报告打包成**单个 HTML**（手机本地可点）。

原理：每份原报告原样存进 <template>（惰性、不执行），点导航时把它塞进
iframe.srcdoc —— CSS/JS 完全隔离，不会互相污染；iframe 同源（about:srcdoc），
所以可以在 load 后挂一个点击拦截，把页面内的 *.html 链接改成"切到对应报告"。

用法：
  python bench/bundle_offline.py                     # 默认从 final-summary + pv-overview 起步爬
  python bench/bundle_offline.py --roots a.html b.html --out docs/xxx.html
"""
import argparse
import html
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.join(os.path.dirname(HERE), 'docs')

SKIP = {'index.html', 'led_app.html'}          # 交互控制页，不打包
HREF = re.compile(r'href="([^"#?]+\.html)(?:[^"]*)?"', re.I)
TITLE = re.compile(r'<title>(.*?)</title>', re.I | re.S)
H1 = re.compile(r'<h1[^>]*>(.*?)</h1>', re.I | re.S)
TAG = re.compile(r'<[^>]+>')
IMG = re.compile(r'data:image/jpeg;base64,([A-Za-z0-9+/=]+)')
LINK = re.compile(r'href=(["\'])([^"\'#?:]+\.html)([^"\']*)\1', re.I)

# 嵌进每个内嵌页：点本地报告链接 → 通知外层切换（不依赖跨文档监听时机）
BRIDGE = """<script>(function(){document.addEventListener('click',function(e){
var a=e.target&&e.target.closest?e.target.closest('[data-rpt]'):null;if(!a)return;
e.preventDefault();try{parent.postMessage('RPT|'+a.getAttribute('data-rpt'),'*');}catch(x){}
},true);})();</script>"""


def rewrite_links(src):
    """把 href="x.html" 改成 href="#" data-rpt="x.html"，并由 BRIDGE 回传外层。"""
    s = LINK.sub(lambda m: f'href="#" data-rpt="{m.group(2)}{m.group(3)}"', src)
    return s + BRIDGE


def shrink_images(src, max_px=1200, quality=78, floor_kb=150):
    """把页面内嵌的 base64 JPEG 压到 max_px / quality（只影响合集，不改原文件）。"""
    try:
        from PIL import Image
        import base64
        import io
    except Exception:
        return src, 0
    saved = 0

    def repl(m):
        nonlocal saved
        b64 = m.group(1)
        if len(b64) * 3 // 4 < floor_kb * 1024:
            return m.group(0)
        try:
            im = Image.open(io.BytesIO(base64.b64decode(b64)))
        except Exception:
            return m.group(0)
        if im.mode not in ('RGB', 'L'):
            im = im.convert('RGB')
        im.thumbnail((max_px, max_px))
        buf = io.BytesIO()
        im.save(buf, 'JPEG', quality=quality, optimize=True, progressive=True)
        new = base64.b64encode(buf.getvalue()).decode()
        if len(new) >= len(b64):
            return m.group(0)
        saved += (len(b64) - len(new)) * 3 // 4
        return 'data:image/jpeg;base64,' + new

    return IMG.sub(repl, src), saved


def clean(s):
    return html.unescape(TAG.sub('', s)).strip()


def crawl(roots):
    order, seen = [], set()
    queue = list(roots)
    while queue:
        name = queue.pop(0)
        if name in seen or name in SKIP:
            continue
        path = os.path.join(DOCS, name)
        if not os.path.isfile(path):
            continue
        seen.add(name)
        order.append(name)
        src = open(path, encoding='utf-8', errors='ignore').read()
        for h in HREF.findall(src):
            if h not in seen and h not in SKIP:
                queue.append(h)
    return order


def meta(src, fallback):
    m = TITLE.search(src)
    t = clean(m.group(1)) if m else ''
    if not t:
        m = H1.search(src)
        t = clean(m.group(1)) if m else ''
    return t or fallback


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--roots', nargs='*', default=['final-summary.html', 'pv-overview.html'])
    ap.add_argument('--entry', default='', help='打开时默认显示哪一页（默认=第一个 root）')
    ap.add_argument('--out', default=os.path.join(DOCS, '车牌光伏报告合集.html'))
    ap.add_argument('--title', default='车牌光伏 · 报告合集（离线单文件）')
    ap.add_argument('--img-px', type=int, default=1200, help='内嵌照片最长边（0=不压）')
    a = ap.parse_args()

    order = crawl(a.roots)
    if not order:
        sys.exit('没爬到任何页面')

    pages, raws, saved = [], [], 0
    for name in order:
        raw = open(os.path.join(DOCS, name), encoding='utf-8', errors='ignore').read()
        raws.append((name, raw))
        src = raw
        if a.img_px:
            src, s = shrink_images(src, max_px=a.img_px)
            saved += s
        pages.append((name, meta(raw, name), rewrite_links(src)))

    total = sum(len(s) for _, _, s in pages)

    entry = a.entry or a.roots[0]
    entry_idx = order.index(entry) if entry in order else 0

    tpls, opts = [], []
    for i, (name, title, src) in enumerate(pages):
        tpls.append(f'<template id="t{i}" data-name="{html.escape(name)}">{src}</template>')
        opts.append(f'<option value="{i}">{html.escape(title)}</option>')

    shell = f"""<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{html.escape(a.title)}</title>
<style>
:root{{color-scheme:dark;--bg:#0d1117;--bar:#161b22;--bd:#30363d;--tx:#e6edf3;--dim:#8b949e;--acc:#58a6ff}}
*{{box-sizing:border-box}}
html,body{{margin:0;height:100%}}
body{{background:var(--bg);color:var(--tx);font:14px/1.5 "Segoe UI","Microsoft YaHei",sans-serif;display:flex;flex-direction:column}}
header{{background:var(--bar);border-bottom:1px solid var(--bd);padding:8px 10px;display:flex;gap:8px;align-items:center;flex-wrap:wrap}}
header .ttl{{font-weight:700;font-size:13px;color:var(--acc);white-space:nowrap}}
select{{flex:1 1 220px;min-width:0;background:#0d1117;color:var(--tx);border:1px solid var(--bd);border-radius:7px;padding:7px 9px;font-size:13px}}
button{{background:#0d1117;color:var(--tx);border:1px solid var(--bd);border-radius:7px;padding:7px 11px;font-size:13px}}
button:active{{border-color:var(--acc);color:var(--acc)}}
button:disabled{{opacity:.38}}
iframe{{flex:1;width:100%;border:0;background:var(--bg)}}
#tip{{padding:5px 10px;font-size:11.5px;color:var(--dim);border-top:1px solid var(--bd);background:var(--bar);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}}
</style></head><body>
<header>
  <span class="ttl">📄 报告合集</span>
  <button id="j-sum">🏠 总结</button>
  <button id="j-pv">☀ 光伏总览</button>
  <select id="sel">{''.join(opts)}</select>
  <button id="prev">‹ 上一个</button>
  <button id="next">下一个 ›</button>
</header>
<iframe id="fr" title="report"></iframe>
<div id="tip">共 {len(pages)} 份 · 点页面内的报告链接也会在这里直接切换</div>
{''.join(tpls)}
<script>
(function(){{
  var fr=document.getElementById('fr'), sel=document.getElementById('sel');
  var prev=document.getElementById('prev'), next=document.getElementById('next');
  var tpls=[].slice.call(document.querySelectorAll('template[id^=t]'));
  var ENTRY={entry_idx};
  var cur=-1;
  function open(i){{
    if(i<0||i>=tpls.length) return;
    cur=i; sel.value=String(i);
    prev.disabled=(i===0); next.disabled=(i===tpls.length-1);
    fr.srcdoc=tpls[i].innerHTML;
    document.title=tpls[i].getAttribute('data-name');
  }}
  function byName(m){{
    for(var k=0;k<tpls.length;k++) if(tpls[k].getAttribute('data-name')===m) return k;
    return -1;
  }}
  // 深链：合集.html#pv-overview.html 直接开到那一页
  function fromHash(){{
    var h=(location.hash||'').replace(/^#/,'');
    if(!h) return -1;
    return byName(decodeURIComponent(h).split('/').pop());
  }}
  // 内嵌页里的本地报告链接已改写成 data-rpt，点击后 postMessage 回来切换
  window.addEventListener('message',function(ev){{
    var s=ev.data;
    if(typeof s!=='string'||s.indexOf('RPT|')!==0) return;
    var m=s.slice(4).split('#')[0].split('?')[0].split('/').pop();
    var k=byName(m);
    if(k>=0){{ open(k); fr.scrollTop=0; return; }}
    // 没打包的页面（如 index.html / led_app.html）给个提示，别让它 404
    var tip=document.getElementById('tip');
    tip.textContent='⚠ '+m+' 未打包进本合集（控制页/外链），手机上请用 Pages 网址访问';
    setTimeout(function(){{tip.textContent='共 {len(pages)} 份 · 点页面内的报告链接也会在这里直接切换';}},4000);
  }});
  sel.addEventListener('change',function(){{open(parseInt(this.value,10));}});
  function jump(n){{ var k=byName(n); if(k>=0) open(k); }}
  document.getElementById('j-sum').addEventListener('click',function(){{jump('final-summary.html');}});
  document.getElementById('j-pv').addEventListener('click',function(){{jump('pv-overview.html');}});
  prev.addEventListener('click',function(){{open(cur-1);}});
  next.addEventListener('click',function(){{open(cur+1);}});
  window.addEventListener('popstate',function(e){{ if(e.state&&typeof e.state.i==='number') open(e.state.i); }});
  document.addEventListener('keydown',function(e){{
    if(e.key==='ArrowLeft') open(cur-1);
    else if(e.key==='ArrowRight') open(cur+1);
  }});
  var h0=fromHash();
  open(h0>=0?h0:ENTRY);
  window.addEventListener('hashchange',function(){{ var k=fromHash(); if(k>=0) open(k); }});
}})();
</script>
</body></html>
"""

    with open(a.out, 'w', encoding='utf-8') as f:
        f.write(shell)
    print(f'打包 {len(pages)} 份，源 {total/1048576:.2f} MB -> 单文件 {os.path.getsize(a.out)/1048576:.2f} MB'
          + (f'（内嵌照片压掉 {saved/1048576:.2f} MB）' if saved else ''))
    print('输出:', a.out)
    for i, (n, t, s) in enumerate(pages):
        print(f'  [{i:2d}] {len(s)/1024:8.0f} KB  {n:42s} {t[:34]}')

    # 自检：页面里的本地 *.html 链接是否都能在本文件内打开
    have = {n for n, _, _ in pages}
    miss, skip_hit = {}, {}
    for name, raw in raws:
        for h in HREF.findall(raw):
            b = h.split('#')[0].split('?')[0].split('/').pop()
            if b in have or b == name:
                continue
            if b in SKIP:
                skip_hit.setdefault(b, set()).add(name)
            else:
                miss.setdefault(b, set()).add(name)
    if miss:
        print('⚠ 未打包但被引用的页面（点了会提示）：')
        for b, srcs in sorted(miss.items()):
            print(f'   - {b}  ← {", ".join(sorted(srcs))[:90]}')
    else:
        print('✅ 所有页面内的 *.html 链接都能在本文件里打开')
    if skip_hit:
        print('（控制页有意不打包：' + ', '.join(sorted(skip_hit)) + '）')


if __name__ == '__main__':
    main()
