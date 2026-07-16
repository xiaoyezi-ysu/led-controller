#ifndef _SC7A20H_H_
#define _SC7A20H_H_

#include "main.h"

#define SC7A20H_I2C_ADDR (0x18 << 1)

typedef enum {
    SC7A20H_INTERFACE_I2C = 0,
    SC7A20H_INTERFACE_SPI
} SC7A20H_Interface_t;

void    SC7A20H_SoftI2C_Init(void);
signed char SC7A20H_Driver_Init(SC7A20H_Interface_t interface);
unsigned char SC7A20H_Read_FIFO_Buf(SC7A20H_Interface_t interface, signed short *x_buf, signed short *y_buf, signed short *z_buf);

#endif
