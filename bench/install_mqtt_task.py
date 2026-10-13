#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
install_mqtt_task.py —— 把 mqtt_watch.py 注册成常驻的 Windows 计划任务
================================================================================
任务设计
    名称   MQTT_Watch_LicensePlate
    动作   E:\\ProgramData\\Anaconda3\\python.exe <bench>\\mqtt_watch.py --quiet-json
    触发   ① 登录时   ② 每天 + 每 5 分钟重复（自愈：进程挂了最多 5 分钟内被拉起）
    身份   Administrator / S4U（不存密码、不弹 UAC）/ 最高权限
    设置   忽略新实例（MultipleInstances=IgnoreNew）、无执行时长上限、失败重试

为什么不担心「重复启动」：mqtt_watch.py 自己带单实例守护（占用本地 127.0.0.1:47931），
后起的实例发现端口被占会直接 exit(3)，不会产生第二个监听连接。

用法
    python install_mqtt_task.py                # 注册 / 覆盖
    python install_mqtt_task.py --status       # 查看状态
    python install_mqtt_task.py --run          # 立刻手动触发一次
    python install_mqtt_task.py --uninstall    # 删除任务
"""

import argparse
import os
import sys

TASK_NAME = "MQTT_Watch_LicensePlate"
HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "mqtt_watch.py")
PYTHON = r"E:\ProgramData\Anaconda3\python.exe"

TASK_CREATE_OR_UPDATE = 6
TASK_LOGON_INTERACTIVE_TOKEN_OR_PASSWORD = 3   # S4U 风格登录类型见 LogonType=3
TASK_RUNLEVEL_HIGHEST = 1
TASK_TRIGGER_LOGON = 9
TASK_TRIGGER_DAILY = 2
TASK_INSTANCES_IGNORE_NEW = 2


def svc():
    import win32com.client
    s = win32com.client.Dispatch("Schedule.Service")
    s.Connect()
    return s


def get_task(s):
    try:
        return s.GetFolder("\\").GetTask(TASK_NAME)
    except Exception:
        return None


def show(s):
    t = get_task(s)
    if t is None:
        print("任务不存在：%s" % TASK_NAME)
        return 1
    print("任务 %s  State=%d  Enabled=%s" % (t.Name, t.State, t.Enabled))
    for a in t.Definition.Actions:
        print("  动作: %s %s" % (a.Path, a.Arguments))
    for tr in t.Definition.Triggers:
        print("  触发: type=%d enabled=%s" % (tr.Type, tr.Enabled))
    return 0


def install(s):
    root = s.GetFolder("\\")
    td = s.NewTask(0)
    td.RegistrationInfo.Description = "车牌板 4G 模组 MQTT 旁听端（常驻，单实例）"
    td.RegistrationInfo.Author = "Administrator"

    st = td.Settings
    st.Enabled = True
    st.StartWhenAvailable = True
    st.MultipleInstances = TASK_INSTANCES_IGNORE_NEW
    st.ExecutionTimeLimit = "PT0S"          # 不限执行时长
    st.DisallowStartIfOnBatteries = False
    st.StopIfGoingOnBatteries = False
    st.RestartInterval = "PT1M"
    st.RestartCount = 3

    # ① 登录即启动
    tr1 = td.Triggers.Create(TASK_TRIGGER_LOGON)
    tr1.UserId = "Administrator"
    tr1.Enabled = True

    # ② 每天 + 每 5 分钟重复（自愈）
    tr2 = td.Triggers.Create(TASK_TRIGGER_DAILY)
    tr2.StartBoundary = "2026-01-01T00:05:00"
    tr2.DaysInterval = 1
    tr2.Enabled = True
    try:
        tr2.Repetition.Interval = "PT5M"
        tr2.Repetition.Duration = "P1D"
        tr2.Repetition.StopAtDurationEnd = False
    except Exception as e:
        print("  (重复设置失败，退化为每天一次: %s)" % e)

    act = td.Actions.Create(0)              # TASK_ACTION_EXEC
    act.Path = PYTHON
    act.Arguments = '"%s" --quiet-json' % SCRIPT
    act.WorkingDirectory = HERE

    pr = td.Principal
    pr.UserId = "Administrator"
    pr.LogonType = TASK_LOGON_INTERACTIVE_TOKEN_OR_PASSWORD
    pr.RunLevel = TASK_RUNLEVEL_HIGHEST

    root.RegisterTaskDefinition(TASK_NAME, td, TASK_CREATE_OR_UPDATE,
                                "Administrator", None,
                                TASK_LOGON_INTERACTIVE_TOKEN_OR_PASSWORD)
    print("已注册/更新任务：%s" % TASK_NAME)
    print("  %s \"%s\" --quiet-json" % (PYTHON, SCRIPT))
    return 0


def run_now(s):
    t = get_task(s)
    if t is None:
        print("任务不存在，先 install")
        return 1
    t.Run("")
    print("已手动触发一次（若本机已有实例在跑，新实例会因单实例守护立即退出，属正常）")
    return 0


def uninstall(s):
    try:
        s.GetFolder("\\").DeleteTask(TASK_NAME, 0)
        print("已删除任务：%s" % TASK_NAME)
        return 0
    except Exception as e:
        print("删除失败：%s" % e)
        return 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--status", action="store_true")
    ap.add_argument("--run", action="store_true")
    ap.add_argument("--uninstall", action="store_true")
    a = ap.parse_args()

    if not os.path.exists(SCRIPT):
        print("找不到 %s" % SCRIPT)
        return 1

    s = svc()
    if a.status:
        return show(s)
    if a.run:
        return run_now(s)
    if a.uninstall:
        return uninstall(s)
    return install(s)


if __name__ == "__main__":
    sys.exit(main())
