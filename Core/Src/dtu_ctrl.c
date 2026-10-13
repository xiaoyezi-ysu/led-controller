#include "dtu_ctrl.h"
#include "iccid_storage.h"

/* ===========================================================================
 * 飞思创 AT 指令固件驱动（与 dtu.c 银尔达透传二选一，由 board_config.h 的
 * MODULE_FSC_AT 选中）。对外 API 与 dtu.h 完全一致，main.c 无需分叉。
 * 源码恢复自 git 提交 d27bad3^（"refactor: rename dtu_ctrl->dtu" 之前），
 * 并补齐 dtu.h 后续新增的可选接口以保持 API 对齐。
 * ========================================================================= */

extern UART_HandleTypeDef huart1;

_4G_State _4g_state = _4G_POWER_ON;
char my_iccid[32] = {0};
uint8_t iccid_loaded = 0;

/* main.c 的 JSON 接收缓冲：AT 固件在 MQTT 连接建立后自动透传，服务器下行
   JSON 直接进 json_buf。配置期（AT+ICCID?）的回包也走 json_buf，故这里借用。 */
extern uint8_t json_buf[];
extern uint16_t json_len;

/* 低功耗标志：飞思创 AT 版没有银尔达的 config,set,lp 私有指令，仅做本地记录
   以保持与 dtu.h 相同的对外行为。真正的 MCU 睡眠由 main.c 的 g_low_power
   控制，与本标志无关。 */
static uint8_t dtu_low_power = 0;

static void send_at(const char *cmd)
{
  HAL_UART_Transmit(&UART_4G, (uint8_t*)cmd, strlen(cmd), 100);
}

void DTU_Init(void)
{
  ICCID_Load(my_iccid);
  iccid_loaded = (my_iccid[0] != '\0');
  if (iccid_loaded)
    printf("ICCID loaded from flash: %s\r\n", my_iccid);
  else
    printf("No ICCID in flash, will query\r\n");
  printf("DTU: Fibocom AT driver init\r\n");
  _4g_state = _4G_POWER_ON;
}

/* 配置期回包已由 main.c 的 HAL_UART_RxCpltCallback 统一写入 json_buf，
   这里无需再捕获；保留空实现以与 dtu.h 的接口对齐（main.c 无条件调用）。 */
void DTU_RxByte(uint8_t b) { (void)b; }

/* 飞思创 AT 版不做透传链路探活，无在途指令保护需求 */
void DTU_MarkCmdReceived(void) { }

void DTU_SetLowPower(uint8_t on) { dtu_low_power = on ? 1 : 0; }
uint8_t DTU_IsLowPower(void) { return dtu_low_power; }

/* 调试探针（PC 口 '~' 触发）：先 "+++" 退出透传回到 AT 指令模式，再查询模组
   网络/MQTT 状态，用于排障"假在线"（显示已连但收发不通）。
   指令名以飞思创手册为准；若型号不同（如合宙/移远 AT 集），在此处调整即可。 */
void DTU_DebugProbe(void)
{
  printf("DBG(FSC AT): exit transparent (+++) + query\r\n");
  send_at("+++");                 /* 退出透传须不带 CRLF */
  HAL_Delay(1200);
  send_at("AT+CSQ\r\n");          /* 信号质量 */
  HAL_Delay(300);
  send_at("AT+MQTTSTAT?\r\n");    /* MQTT 连接状态（部分型号可能回 ERROR）*/
  HAL_Delay(300);
  send_at("AT+IP?\r\n");          /* 本机 IP */
}

void DTU_Process(uint32_t now)
{
  static uint32_t tick = 0;
  if (_4g_state == _4G_READY) return;

  uint32_t wait = 2500;
  if      (_4g_state == _4G_POWER_ON) wait = 10000;
  else if (_4g_state == _4G_FLUSH)    wait = 20000;
  if (now - tick < wait) return;
  tick = now;

  switch (_4g_state)
  {
    case _4G_POWER_ON:
      _4g_state = _4G_SEND_PLUS;
      break;
    case _4G_SEND_PLUS:
      printf("4G: +++\r\n");
      send_at("+++");
      _4g_state = _4G_SEND_WKMOD;
      break;
    case _4G_SEND_WKMOD:
      printf("4G: WKMOD\r\n");
      send_at("AT+WKMOD1=MQTT\r\n");
      _4g_state = _4G_SEND_MQTT_SVR;
      break;
    case _4G_SEND_MQTT_SVR:
      printf("4G: MQTTSV1\r\n");
      send_at("AT+MQTTSV1=broker.emqx.io,1883\r\n");
      _4g_state = _4G_SEND_MQTT_CONN;
      break;
    case _4G_SEND_MQTT_CONN:
      printf("4G: MQTTCONN1\r\n");
    {
      char buf[96];
      if (iccid_loaded && my_iccid[0])
        snprintf(buf, sizeof(buf), "AT+MQTTCONN1=card_%s,,,60,1\r\n", my_iccid);
      else
      {
        uint32_t uid = *(uint32_t*)0x1FFFF7E8;
        snprintf(buf, sizeof(buf), "AT+MQTTCONN1=stm32_%08X,,,60,1\r\n", (unsigned int)uid);
      }
      send_at(buf);
    }
      _4g_state = _4G_SEND_MQTT_PUB;
      break;
    case _4G_SEND_MQTT_PUB:
      printf("4G: MQTTPUB1\r\n");
    {
      char cmd[96];
      if (iccid_loaded && my_iccid[0])
        snprintf(cmd, sizeof(cmd), "AT+MQTTPUB1=DTU_Topic1147/%s,0,0\r\n", my_iccid);
      else
        snprintf(cmd, sizeof(cmd), "AT+MQTTPUB1=DTU_Topic1147,0,0\r\n");
      send_at(cmd);
    }
      _4g_state = _4G_SEND_MQTT_SUB;
      break;
    case _4G_SEND_MQTT_SUB:
      printf("4G: MQTTSUB1\r\n");
    {
      char cmd[96];
      if (iccid_loaded && my_iccid[0])
        snprintf(cmd, sizeof(cmd), "AT+MQTTSUB1=" MQTT_TOPIC_PREFIX "/%s,0\r\n", my_iccid);
      else
        snprintf(cmd, sizeof(cmd), "AT+MQTTSUB1=" MQTT_TOPIC_PREFIX ",0\r\n");
      send_at(cmd);
    }
      _4g_state = _4G_SEND_AT;
      break;
    case _4G_SEND_AT:
      printf("4G: AT\r\n");
      send_at("AT\r\n");
      _4g_state = _4G_SEND_ICCID;
      break;
    case _4G_SEND_ICCID:
      if (iccid_loaded)
      {
        printf("ICCID already loaded, skip query\r\n");
        _4g_state = _4G_SAVE;
        break;
      }
      json_len = 0;
      printf("4G: AT+ICCID?\r\n");
      send_at("AT+ICCID?\r\n");
      _4g_state = _4G_PARSE_ICCID;
      break;
    case _4G_PARSE_ICCID:
    {
      char *p = strstr((char*)json_buf, "+ICCID:");
      if (p)
      {
        char *num = p + 7;
        while (*num >= '0' && *num <= '9') num++;
        *num = '\0';
        strcpy(my_iccid, p + 7);
        printf("ICCID: %s\r\n", my_iccid);
        ICCID_Save(my_iccid);
        iccid_loaded = 1;
        printf("ICCID saved to flash\r\n");
      }
      _4g_state = _4G_SAVE;
      break;
    }
    case _4G_SAVE:
      printf("4G: AT+S\r\n");
      send_at("AT+S\r\n");
      _4g_state = _4G_REBOOT;
      break;
    case _4G_REBOOT:
      printf("4G: AT+Z\r\n");
      send_at("AT+Z\r\n");
      json_len = 0;
      _4g_state = _4G_FLUSH;
      break;
    case _4G_FLUSH:
    {
      char msg[96];
      int n = snprintf(msg, sizeof(msg), "{\"cmd\":\"ready\",\"id\":\"%s\"}", my_iccid[0] ? my_iccid : "unknown");
      if (n > 0) HAL_UART_Transmit(&UART_4G, (uint8_t*)msg, n, 200);
      printf("4G: READY (%s)\r\n", msg);
      json_len = 0;
      _4g_state = _4G_READY;
      break;
    }
    default:
      _4g_state = _4G_READY;
      break;
  }
}
