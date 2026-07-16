#include "mqtt_cmd.h"
#include "led_ctrl.h"
#include <stdio.h>

extern uint8_t json_buf[JSON_BUF_SIZE];
extern uint16_t json_len;
extern volatile uint8_t json_ready;
extern volatile uint32_t json_last_rx;
extern uint8_t iccid_loaded;
extern char my_iccid[];

void MQTT_Feed(uint8_t byte)
{
  if (json_len < JSON_BUF_SIZE)
    json_buf[json_len++] = byte;
  if (byte == '}')
    json_ready = 1;
  json_last_rx = HAL_GetTick();
}

uint8_t MQTT_ParseBrightness(void)
{
  uint8_t bright = 255;
  char *bp = strstr((char*)json_buf, "\"bright\":");
  if (!bp) bp = strstr((char*)json_buf, "\"bright\": ");
  if (bp)
  {
    bp += 9; while (*bp == ' ') bp++;
    int v = 0;
    while (*bp >= '0' && *bp <= '9') { v = v * 10 + (*bp - '0'); bp++; }
    if (v >= 0 && v <= 255) bright = (uint8_t)v;
  }
  return bright;
}

void MQTT_CmdDispatch(void)
{
  if (!json_ready) return;
  json_ready = 0;

  uint8_t match = 1;
  if (iccid_loaded)
    match = (strstr((char*)json_buf, my_iccid) != NULL);

  if (match)
  {
    if (strstr((char*)json_buf, "\"cmd\":\"pc13\"") || strstr((char*)json_buf, "\"cmd\":\"green\""))
    {
      PC13_On(); printf("PC13: ON (green)\r\n");
    }
    else if (strstr((char*)json_buf, "\"cmd\":\"pc13off\"") || strstr((char*)json_buf, "\"cmd\":\"off\""))
    {
      PC13_Off(); printf("PC13: OFF\r\n");
    }
  }

  json_len = 0;
  memset(json_buf, 0, JSON_BUF_SIZE);
}

void MQTT_TimeoutCheck(uint32_t now)
{
  if (json_len > 0 && now - json_last_rx > 3000)
    json_len = 0;
}
