#ifndef LED_CTRL_H
#define LED_CTRL_H

#include "main.h"

void LED_Init(void);
void LED_Green(uint8_t bright);
void LED_Off(void);
void PC13_On(void);
void PC13_Off(void);

#endif
