#ifndef MQTT_CMD_H
#define MQTT_CMD_H

#include "main.h"
#include <string.h>

#define JSON_BUF_SIZE  256

extern uint8_t json_buf[JSON_BUF_SIZE];
extern uint16_t json_len;
extern volatile uint8_t json_ready;
extern volatile uint32_t json_last_rx;

void MQTT_Feed(uint8_t byte);
uint8_t MQTT_ParseBrightness(void);
void MQTT_CmdDispatch(void);
void MQTT_TimeoutCheck(uint32_t now);

#endif
