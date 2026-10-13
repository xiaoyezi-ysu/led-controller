#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mqtt_watch.py —— 车牌板 4G 模组的 MQTT 旁听端（后台常驻）
================================================================================
模组的 MQTT 配置（见 Core/Src/dtu.c 的 _4G_SEND_MQTT 分支）：

    broker       broker.emqx.io:1883        （公网 EMQX，user/pwd 均空）
    clientid     = ICCID
    订阅(下行)    xiaoyeziUESTC1147/<ICCID>   ← 服务器/Web App 往这里发指令
    发布(上行)    DTU_Topic1147/<ICCID>       ← 板子的 report / ack / ready 都发到这里
    proto 3.1.1 · clean=0（持久会话）· retain=0

本机读不到板子的 ICCID，所以默认用**通配符**订阅两个前缀：
    DTU_Topic1147/#          抓板子上行（report / ack / ready / 探针回执）
    xiaoyeziUESTC1147/#      顺带看下行指令（别人发的也能看到）
通配订阅既不用知道 ICCID，也会自动把 ICCID 从上行主题里识别出来。

能力
    常驻监听 + 自动重连（断网/掉线自愈，指数退避）
    每条消息落两份：<out>.csv（结构化）+ <out>.log（人读），并追加到 mqtt_all.csv
    --cmd / --send  往板子发一条指令（需要先识别到 ICCID，或 --iccid 指定）
    --discover      只打印听到的 ICCID / 主题
    --idle-exit N   N 秒没有新消息就退出（给自动化用；默认一直跑）

用法
    # 常驻监听（后台）
    python mqtt_watch.py

    # 监听 20 分钟，没消息就退出
    python mqtt_watch.py --idle-exit 1200

    # 顺便给板子发一条指令（板子会顺手清掉低功耗态，注意测量场景别用）
    python mqtt_watch.py --cmd '{"cmd":"lp_cycle","period_min":1}'

    # 只发一条就退出
    python mqtt_watch.py --send-only --cmd '{"cmd":"v4g","state":"on"}'

    # 看看现在有哪些板子在说话
    python mqtt_watch.py --discover --idle-exit 30
"""

import argparse
import csv
import datetime
import json
import os
import signal
import socket
import sys
import time

# 单实例守护：占用一个本地端口当互斥量。第二个实例直接退出（退出码 3），
# 这样「开机自启 + 每 5 分钟自愈」的计划任务可以无脑重复触发而不会重复监听。
LOCK_PORT = 47931
_lock_sock = None


def acquire_single_instance():
    global _lock_sock
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("127.0.0.1", LOCK_PORT))
        s.listen(1)
        _lock_sock = s
        return True
    except OSError:
        s.close()
        return False

try:
    import paho.mqtt.client as mqtt
except ImportError:
    sys.stderr.write("需要 paho-mqtt：pip install paho-mqtt\n")
    sys.exit(1)

HERE = os.path.dirname(os.path.abspath(__file__))

BROKER = "broker.emqx.io"
PORT = 1883
SUB_UP = "DTU_Topic1147/#"          # 板子 → 服务器
SUB_DOWN = "xiaoyeziUESTC1147/#"    # 服务器 → 板子
PUB_FMT = "xiaoyeziUESTC1147/%s"    # 指令下行主题
UP_PREFIX = "DTU_Topic1147/"

STATE = {"iccid": None, "n": 0, "last": 0.0, "topics": {}}
_ARGS = None
_CSV = None
_CSVW = None
_LOG = None
_ALL = None
_ALLW = None


def now_iso():
    return datetime.datetime.now().isoformat(timespec="milliseconds")


def open_retry(path, mode, tries=8, delay=1.0):
    """带重试地打开文件。本机 data/ 目录被百度云同步盯着，
    云同步上传瞬间会给文件上锁（PermissionError），计划任务自启动撞上就会直接挂掉。"""
    last = None
    for i in range(tries):
        try:
            return open(path, mode, encoding="utf-8", newline="")
        except PermissionError as e:
            last = e
            time.sleep(delay)
    raise last


def log(msg):
    line = "%s  %s" % (now_iso(), msg)
    print(line, flush=True)
    if _LOG:
        _LOG.write(line + "\n")
        _LOG.flush()


def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        log("已连接 %s:%d" % (BROKER, PORT))
        for t in (SUB_UP, SUB_DOWN):
            client.subscribe(t, qos=0)
            log("  订阅 %s" % t)
    else:
        log("!! 连接失败 rc=%s" % rc)


def on_disconnect(client, userdata, rc, properties=None):
    if rc != 0:
        log("!! 断开 rc=%s，paho 会自动重连" % rc)


def on_message(client, userdata, msg):
    payload = msg.payload.decode("utf-8", "replace")
    ts = time.time()
    STATE["n"] += 1
    STATE["last"] = ts
    STATE["topics"][msg.topic] = STATE["topics"].get(msg.topic, 0) + 1

    # 从上行主题里识别 ICCID
    if msg.topic.startswith(UP_PREFIX):
        ic = msg.topic[len(UP_PREFIX):]
        if ic and STATE["iccid"] != ic:
            STATE["iccid"] = ic
            log("  ★ 识别到 ICCID = %s" % ic)

    kind = "<-" if msg.topic.startswith("xiaoyeziUESTC1147/") else "->"
    body = payload.replace("\r", "").replace("\n", "\\n").strip()
    log("%s %s  %s" % (kind, msg.topic, body if len(body) <= 400 else body[:400] + "…"))

    if _CSVW:
        _CSVW.writerow(["%.3f" % ts, now_iso(), kind, msg.topic, len(msg.payload), body])
        _CSV.flush()
    if _ALLW:
        # 用 csv.writer 走标准引号转义：payload 里的逗号/引号原样保留，
        # 跟 mqtt_live.csv 完全同构（此前手写 % 格式化 + replace(",", ";") 会毁掉 JSON）。
        _ALLW.writerow(["%.3f" % ts, now_iso(), kind, msg.topic, len(msg.payload), body])
        _ALL.flush()

    if _ARGS and _ARGS.quiet_json:
        try:
            o = json.loads(payload)
            if isinstance(o, dict) and "cmd" in o:
                log("    json: cmd=%s %s" % (o.get("cmd"),
                                             {k: v for k, v in o.items() if k != "cmd"}))
        except Exception:
            pass


def publish_cmd(client, cmd, iccid):
    if not iccid:
        log("!! 还没有识别到 ICCID，无法发指令（可用 --iccid 指定，或先让它收一会儿）")
        return False
    topic = PUB_FMT % iccid
    client.publish(topic, cmd, qos=0)
    log("=> 发布 %s  %s" % (topic, cmd))
    return True


def main():
    global _ARGS, _CSV, _CSVW, _LOG, _ALL, _ALLW
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", default=BROKER)
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--iccid", default=None, help="板子 ICCID（发指令用；缺省自动识别）")
    ap.add_argument("--cmd", default=None, help="连上后发一条指令（JSON 字符串）")
    ap.add_argument("--send", default=None, nargs=2, metavar=("TOPIC", "PAYLOAD"),
                    help="发一条原始消息")
    ap.add_argument("--send-only", action="store_true", help="发完就退出，不常驻")
    ap.add_argument("--send-delay", type=float, default=3.0, help="连上后等几秒再发")
    ap.add_argument("--out", default=None, help="输出前缀（默认 bench/data/mqtt_<时间戳>）")
    ap.add_argument("--idle-exit", type=float, default=0.0, help="空闲 N 秒后退出（0=常驻）")
    ap.add_argument("--discover", action="store_true", help="只打印识别到的 ICCID / 主题")
    ap.add_argument("--quiet-json", action="store_true", help="把 JSON 指令再摘要打一行")
    ap.add_argument("--allow-multi", action="store_true", help="允许同机多实例并存")
    a = ap.parse_args()
    _ARGS = a

    if not a.allow_multi and not acquire_single_instance():
        sys.stderr.write("已有 mqtt_watch 实例在跑（本地端口 %d 被占）——本次退出\n" % LOCK_PORT)
        return 3

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = a.out or os.path.join(HERE, "data", "mqtt_live")
    d = os.path.dirname(os.path.abspath(out))
    if d and not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)

    # 追加模式：常驻监听被重启后历史仍累积在同一个文件里
    new_csv = not (os.path.exists(out + ".csv") and os.path.getsize(out + ".csv") > 0)
    _CSV = open_retry(out + ".csv", "a")
    _CSVW = csv.writer(_CSV)
    if new_csv:
        _CSVW.writerow(["ts_epoch", "ts_iso", "dir", "topic", "bytes", "payload"])
    _LOG = open_retry(out + ".log", "a")
    allpath = os.path.join(d, "mqtt_all.csv")
    new_all = not (os.path.exists(allpath) and os.path.getsize(allpath) > 0)
    _ALL = open_retry(allpath, "a")
    _ALLW = csv.writer(_ALL)
    if new_all:
        _ALLW.writerow(["ts_epoch", "ts_iso", "dir", "topic", "bytes", "payload"])

    log("=== MQTT 旁听 ===")
    log("  broker   %s:%d" % (a.broker, a.port))
    log("  捕获     %s   %s" % (SUB_UP, SUB_DOWN))
    log("  下行主题 %s" % (PUB_FMT % (a.iccid or "<ICCID>")))
    log("  输出     %s.{csv,log}（追加）" % out)
    log("  汇总     %s（追加，跨会话累积）" % os.path.basename(allpath))

    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1,
                             client_id="wb-watch-%d" % (os.getpid() % 100000),
                             clean_session=True)
    except (AttributeError, TypeError):
        client = mqtt.Client(client_id="wb-watch-%d" % (os.getpid() % 100000))

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.reconnect_delay_set(min_delay=1, max_delay=30)

    if a.iccid:
        STATE["iccid"] = a.iccid

    try:
        client.connect(a.broker, a.port, keepalive=60)
    except Exception as e:
        log("!! 连接 %s:%d 失败：%s" % (a.broker, a.port, e))
        return 2
    client.loop_start()

    sent = False
    stop = {"v": False}

    def _sig(_s, _f):
        stop["v"] = True
    try:
        signal.signal(signal.SIGINT, _sig)
        signal.signal(signal.SIGTERM, _sig)
    except Exception:
        pass

    t_start = time.time()
    while not stop["v"]:
        time.sleep(0.5)
        if not sent and (a.cmd or a.send) and (time.time() - t_start) >= a.send_delay:
            sent = True
            if a.send:
                client.publish(a.send[0], a.send[1], qos=0)
                log("=> 发布 %s  %s" % (a.send[0], a.send[1]))
            else:
                publish_cmd(client, a.cmd, STATE["iccid"])
            if a.send_only:
                time.sleep(1.5)
                break
        if a.idle_exit and (time.time() - t_start) > 5 and \
           (time.time() - max(STATE["last"], t_start)) > a.idle_exit:
            log("  空闲 %.0fs，退出" % a.idle_exit)
            break

    client.loop_stop()
    client.disconnect()
    log("")
    log("=== 汇总 ===")
    log("  收到 %d 条消息" % STATE["n"])
    for t, c in sorted(STATE["topics"].items(), key=lambda x: -x[1]):
        log("    %-40s %d 条" % (t, c))
    log("  识别到的 ICCID: %s" % (STATE["iccid"] or "(无)"))
    if _CSV:
        _CSV.close()
    if _LOG:
        _LOG.flush()
        _LOG.close()
    if _ALL:
        _ALL.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
