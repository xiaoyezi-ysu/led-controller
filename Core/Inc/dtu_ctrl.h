#ifndef DTU_CTRL_H
#define DTU_CTRL_H

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

void DTU_Init(void);
void DTU_Process(uint32_t now);

#endif
