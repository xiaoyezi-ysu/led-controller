#include "dtu_ctrl.h"
#include "iccid_storage.h"

extern UART_HandleTypeDef huart1;

_4G_State _4g_state = _4G_POWER_ON;
char my_iccid[32] = {0};
uint8_t iccid_loaded = 0;

extern uint8_t json_buf[];
extern uint16_t json_len;

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
  _4g_state = _4G_POWER_ON;
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
