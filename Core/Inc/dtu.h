#ifndef DTU_H
#define DTU_H

#include "main.h"
#include "board_config.h"
#include <stdint.h>

/* ===========================================================================
 * DTU 驱动（银尔达 YED_DTU4 透传固件）
 * ---------------------------------------------------------------------------
 * 取代原飞思创 AT 指令方案，改用银尔达的 config,set/get,mqtt 参数命令集。
 * 功能与原 dtu_ctrl 完全一致：上电后自动配置 DTU 连接 broker.emqx.io，
 * 按设备号区分订阅/发布主题，进入透传后 STM32 即可经串口收发 JSON。
 *
 * 协议要点（已在 PC 端 SSCOM 验证）：
 *   查询 : config,get,<key>        -> config,<key>,ok,<value...>
 *   设置 : config,set,<key>[,...]  -> config,<key>,ok  (失败 config,<key>,error)
 *   MQTT : config,set,mqtt,<24字段>
 *   保存 : config,set,save         -> 模块重启并掉电保存
 * ========================================================================= */

typedef enum {
  _4G_POWER_ON = 0,
  _4G_SEND_PARAMSRC,   /* 参数源设为仅串口，避免 web 覆盖 */
  _4G_QUERY_ICCID,     /* 查询 ICCID */
  _4G_PARSE_ICCID,     /* 解析 ICCID */
  _4G_PARSE_IMEI,      /* ICCID 失败 -> 回退 IMEI */
  _4G_SEND_MQTT,       /* 下发 MQTT 配置（24 字段） */
  _4G_SEND_LED,        /* 下发模组指示灯关闭（config,set,led,2） */
  _4G_SEND_LP,         /* 下发模组低功耗保持连接（config,set,lp,1） */
  _4G_SEND_SAVE,       /* config,set,save（模块重启） */
  _4G_WAIT_BOOT,       /* 等待重启完成 */
  _4G_POLL_SSTA,       /* 轮询连接状态 ssta==4 */
  _4G_RECOVER_ESC,    /* 透传中探活：发 +++ 退出透传进入指令模式 */
  _4G_RECOVER_SSTA,   /* 查 ssta 判断链路是否存活 */
  _4G_READY            /* 透传就绪 */
} _4G_State;

extern _4G_State _4g_state;
extern char my_iccid[32];       /* 复用为设备ID缓冲（ICCID / IMEI / MCU UID） */
extern uint8_t iccid_loaded;

void DTU_Init(void);
void DTU_Process(uint32_t now);
void DTU_RxByte(uint8_t b);     /* 由 UART RX 回调调用，捕获配置期应答 */
void DTU_MarkCmdReceived(void); /* main.c 收到指令时调用，避免探活打断在途指令 */
void DTU_DebugProbe(void);      /* COM21 发 '~' 触发，上报模块真实 MQTT 状态 */

/* 睡眠/低功耗：开启后停掉 STM32 自身的心跳与链路探活，信任 4G 模组内部的
   MQTT keepalive 维持服务器连接；STM32 仅靠 4G 模组 UART 来字节唤醒。 */
void DTU_SetLowPower(uint8_t on);
uint8_t DTU_IsLowPower(void);

#endif /* DTU_H */
