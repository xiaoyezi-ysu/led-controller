#include <string.h>
#include "uart_bridge.h"

extern UART_HandleTypeDef huart3;
extern volatile uint8_t rbuf[RBUF_SIZE];
extern volatile uint16_t rbuf_wr;
extern uint16_t rbuf_rd;

extern uint8_t uart1_rx_byte;

void Bridge_Init(void)
{
  rbuf_wr = 0;
  rbuf_rd = 0;
  memset((void*)rbuf, 0, RBUF_SIZE);
}

void Bridge_Store4GByte(uint8_t byte)
{
  uint16_t next = (rbuf_wr + 1) % RBUF_SIZE;
  if (next != rbuf_rd)
  {
    rbuf[rbuf_wr] = byte;
    rbuf_wr = next;
  }
}

void Bridge_Drain(void)
{
  while (rbuf_rd != rbuf_wr)
  {
    if (HAL_UART_Transmit(&huart3, (uint8_t*)&rbuf[rbuf_rd], 1, 1) != HAL_OK)
      break;
    rbuf_rd = (rbuf_rd + 1) % RBUF_SIZE;
  }
}
