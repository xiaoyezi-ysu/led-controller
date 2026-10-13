#ifndef DTU_CTRL_H
#define DTU_CTRL_H

/* ===========================================================================
 * DTU 驱动（飞思创 AT 指令固件）—— 与 dtu.h（银尔达透传）二选一
 * ---------------------------------------------------------------------------
 * 由 board_config.h 的 MODULE_FSC_AT 选中。对外 API 与 dtu.h 完全一致，
 * 使 main.c 无需为两种模组分叉（预编译宏切换，同一份 main.c / 同一 main loop）。
 *
 * 与银尔达透传方案的区别：
 *   飞思创 = 纯 AT 指令流程：
 *     AT+WKMOD1=MQTT → AT+MQTTSV1(服务器) → AT+MQTTCONN1(客户端ID) →
 *     AT+MQTTPUB1(上行主题) → AT+MQTTSUB1(下行主题) → AT+ICCID?(卡号) →
 *     AT+S(保存) → AT+Z(重启) → 进入透传
 *   连接建立后模组自动透传：服务器下行 JSON 直接从串口吐出，与银尔达一致，
 *   故 main.c 的 json_buf 解析 / ACK 逻辑可原样复用。
 * ========================================================================= */

#include "main.h"
#include "board_config.h"
#include <string.h>
#include <stdio.h>

typedef enum {
  _4G_POWER_ON = 0,
  _4G_SEND_PLUS,
  _4G_SEND_WKMOD,
  _4G_SEND_MQTT_SVR,
  _4G_SEND_MQTT_CONN,
  _4G_SEND_MQTT_PUB,
  _4G_SEND_MQTT_SUB,
  _4G_SEND_AT,
  _4G_SEND_ICCID,
  _4G_PARSE_ICCID,
  _4G_SAVE,
  _4G_REBOOT,
  _4G_FLUSH,
  _4G_READY
} _4G_State;

extern _4G_State _4g_state;
extern char my_iccid[32];
extern uint8_t iccid_loaded;

/* —— 与 dtu.h 对齐的统一接口（main.c 无条件调用，两种模组都必须提供）—— */
void DTU_Init(void);
void DTU_Process(uint32_t now);
void DTU_RxByte(uint8_t b);        /* AT 版：回包已由 main.c 入 json_buf，本函数 no-op */
void DTU_MarkCmdReceived(void);    /* AT 版：无透传探活，本函数 no-op */
void DTU_DebugProbe(void);         /* AT 版：退出透传并查询模组网络/MQTT 状态 */
void DTU_SetLowPower(uint8_t on);  /* AT 版：无模组 lp 私有指令，仅本地记录 */
uint8_t DTU_IsLowPower(void);

#endif /* DTU_CTRL_H */
