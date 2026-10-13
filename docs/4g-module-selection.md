# 4G 模组驱动选择（银尔达 / 飞思创，预编译二选一）

本工程原先只有一套 4G 驱动（银尔达透传），历史提交 `d27bad3`
「refactor: rename dtu_ctrl->dtu」把原来的飞思创 AT 方案**整体删除并替换**掉了。
现在改为**两套驱动并存 + 预编译宏切换**：同一份 `main.c`、同一套 LED/MQTT
逻辑，只换 4G 驱动即可编出面向不同模组的固件。

## 怎么切换

只改一处 —— `Core/Inc/board_config.h` 顶部的模组宏（取消注释对应行）：

```c
#define MODULE_YINERDA      /* 银尔达 YED_DTU4 透传固件（默认） */
// #define MODULE_FSC_AT    /* 飞思创 AT 指令固件 */
```

两个宏**互斥**，同时定义会触发 `#error`。若两个都没定义，默认走银尔达。

`CMakeLists.txt` 会自动读取该宏：
- 定义了 `MODULE_FSC_AT` → 编译 `Core/Src/dtu_ctrl.c`
- 否则 → 编译 `Core/Src/dtu.c`

并且 `board_config.h` 已加入 `CMAKE_CONFIGURE_DEPENDS`，**改完宏直接 build 即可**
（CMake 会自动重新 configure，无需手动删构建缓存）。

## 两套驱动对照

| 项目 | 银尔达（默认） | 飞思创 |
|---|---|---|
| 选中宏 | `MODULE_YINERDA` | `MODULE_FSC_AT` |
| 驱动文件 | `Core/Src/dtu.c` + `Core/Inc/dtu.h` | `Core/Src/dtu_ctrl.c` + `Core/Inc/dtu_ctrl.h` |
| 命令集 | `config,set/get,...`（透传固件） | `AT+WKMOD1` / `AT+MQTTSV1` / `AT+MQTTCONN1` / `AT+MQTTPUB1` / `AT+MQTTSUB1` / `AT+ICCID?` / `AT+S` / `AT+Z` |
| 进入透传 | 配置完 save 后模组自动透传 | 配置完 `AT+Z` 重启后自动透传 |
| 下行 JSON | 模组按行打包，从串口吐出 → `json_buf` | 同左（连接建立后同样自动透传） |
| 状态机差异 | `PARAMSRC→QUERY_ICCID→SEND_MQTT(24字段)→SEND_LED→SEND_LP→SEND_SAVE→WAIT_BOOT→POLL_SSTA→RECOVER_ESC→READY` | `SEND_PLUS→SEND_WKMOD→MQTT_SVR→MQTT_CONN→MQTT_PUB→MQTT_SUB→SEND_AT→SEND_ICCID→SAVE→REBOOT→FLUSH→READY` |

## 统一接口（为什么 main.c 不用分叉）

两套驱动对外提供**完全相同**的 API，`main.c` 只用 `#ifdef` 换包含的头文件：

```c
#ifdef MODULE_FSC_AT
#include "dtu_ctrl.h"      /* 飞思创 AT */
#else
#include "dtu.h"           /* 银尔达（默认）*/
#endif
```

| 函数/符号 | 银尔达 | 飞思创 AT |
|---|---|---|
| `_4g_state` / `my_iccid` / `iccid_loaded` | ✅ | ✅ |
| `DTU_Init()` / `DTU_Process(now)` | ✅ | ✅ |
| `DTU_RxByte(b)` | 捕获配置期应答行 | no-op（回包已由 main.c 入 `json_buf`） |
| `DTU_MarkCmdReceived()` | 记录时刻，避免探活打断在途指令 | no-op（AT 版无透传探活） |
| `DTU_DebugProbe()` | 退出透传查 `netstatus` | 退出透传查 `AT+CSQ` / `AT+MQTTSTAT?` / `AT+IP?` |
| `DTU_SetLowPower()` / `DTU_IsLowPower()` | 控制模组 `lp` 私有指令 | 仅本地记录（无该私有指令） |

> 注意：飞思创 AT 版补齐的 `DTU_RxByte` / `DTU_MarkCmdReceived` /
> `DTU_SetLowPower` 等是**保持接口一致的适配实现**，不是原历史版本的原始逻辑。

## 编译验证（2026-10-12）

| 配置 | FLASH | RAM | 结果 |
|---|---|---|---|
| `MODULE_YINERDA`（默认，当前工作区状态） | 29044 B / 64 KB (44.32%) | 3288 B / 20 KB (16.05%) | ✅ 链接通过 |
| `MODULE_FSC_AT` | 27588 B / 64 KB (42.10%) | 3040 B / 20 KB (14.84%) | ✅ 链接通过 |

两种配置均无 `undefined reference` / `multiple definition`。工作区已恢复为
**银尔达默认配置**，`build/Debug/*.hex|.bin` 由银尔达版 ELF 生成（29544→29044 B）。

## 排障

- **切宏后没生效**：确认改的是 `board_config.h` 里那一行（不是注释掉的那行）；
  CMake 会打印 `4G module driver: ...` 供确认。
- **重建时偶发 `CMake Error: Unable to open check cache file for write` /
  `ninja: error: remove(.ninja_lock)`**：Windows 文件句柄延迟释放或杀软实时扫描
  占用构建目录所致，**稍等 1~2 秒重跑一次**即可，非代码问题。
- **换成飞思创模组后 AT 指令不匹配**：`dtu_ctrl.c` 里的 `AT+MQTTxxx` 指令集按
  飞思创手册编写；若实际模组是合宙/移远等其它 AT 集，需对应调整该文件的
  状态机命令与 `DTU_DebugProbe()` 里的查询指令。
