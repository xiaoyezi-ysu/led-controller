#ifndef BOARD_IO_H
#define BOARD_IO_H

#include "main.h"

/* 颜色索引（与 LED_SetColor 配合使用） */
#define LED_COLOR_RED     0
#define LED_COLOR_YELLOW  1
#define LED_COLOR_GREEN   2

/* ===================== 板级 IO 抽象 ===================== */

/* ---------- LED ----------
   车牌板：TIM3 CH1/CH2/CH3 三路 PWM，亮度 0..LED_BRIGHT_MAX(255)
   最小板：PC13 单色 GPIO，bright>0 即点亮 */
void  LED_Init(void);
void  LED_SetColor(uint8_t color, uint8_t bright);   /* 点亮某色，其余熄灭 */
void  LED_SetRGB(uint8_t r, uint8_t y, uint8_t g);   /* 三路独立，可混色 */
void  LED_AllOff(void);
uint8_t LED_GetBright(uint8_t color);                /* 查询当前亮度 */

/* ---------- 4G 模组电源（V4G，车牌板 PA3）---------- */
void  V4G_Init(void);        /* 配成推挽输出并立即上电，须在 DTU_Init() 之前调用 */
void  V4G_Set(uint8_t on);
uint8_t V4G_Get(void);

#endif /* BOARD_IO_H */
