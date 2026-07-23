# MQTT 主题架构方案

## 现状（方案A）

```
订阅：xiaoyeziUESTC1147
发布：DTU_Topic1147
过滤：ICCID 字段匹配（固件内做）
```

所有模块共享同一组订阅/发布主题，通过 JSON 中的 ICCID 字段区分目标设备。

**优点：**
- Web App 只需向一个主题发命令
- 简单，当前已实现

**缺点：**
- 所有模块收到所有命令，由固件过滤（CPU 开销）
- 日志会看到无关模块的命令
- Web App 无法明确知道命令发给了哪个模块

## 方案B：按 ICCID 分主题

```
订阅：xiaoyeziUESTC1147/<ICCID>
发布：DTU_Topic1147/<ICCID>
```

每个模块使用独立的子主题，Web App 按 ICCID 选择发往哪个子主题。

### 改动点

#### 固件
```c
// Core/Inc/board_config.h
// 无需改动，直接复用 MQTT_TOPIC_PREFIX

// Core/Src/dtu_ctrl.c
// MQTTSUB1 改为：AT+MQTTSUB1=xiaoyeziUESTC1147/<ICCID>,0
// MQTTPUB1 改为：AT+MQTTPUB1=DTU_Topic1147/<ICCID>,0,0
```

ICCID 在固件运行时已知，可拼入主题字符串。

#### Web App
- 增加模块选择界面（ICCID 列表）
- 发布时发往 `xiaoyeziUESTC1147/<选中的ICCID>`
- 订阅时接收所有模块的回复：`DTU_Topic1147/+`
- 或为每个模块单独维护一个订阅

#### 固件 ICCID 过滤
可移除 `main.c` 中的 ICCID `strstr` 检查，因为主题本身已唯一。

## 方案C：Topic 前缀 + ICCID 后缀（折中）

同样按 ICCID 分子主题，但结构更清晰：

```
发布（模块→Web）：DTU_Topic1147/<ICCID>
订阅（Web→模块）：xiaoyeziUESTC1147/<ICCID>
```

与方案B本质相同。

## 总结

| | 方案A（当前） | 方案B（分主题） |
|--|--|--|
| 主题数 | 2（共享） | 2N（N=模块数） |
| Web App | 简单，一个入口 | 需模块选择界面 |
| 固件 ICCID 过滤 | 需要 | 可去掉 |
| 扩展性 | 模块越多，无效消息越多 | 无无效消息 |
| 改动量 | 0 | 固件小改 + Web大改 |
