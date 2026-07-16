#ifndef UART_BRIDGE_H
#define UART_BRIDGE_H

#include "main.h"
#include <string.h>

#define RBUF_SIZE  512

void Bridge_Init(void);
void Bridge_Drain(void);
void Bridge_Store4GByte(uint8_t byte);

extern volatile uint8_t rbuf[RBUF_SIZE];
extern volatile uint16_t rbuf_wr;
extern uint16_t rbuf_rd;

#endif
