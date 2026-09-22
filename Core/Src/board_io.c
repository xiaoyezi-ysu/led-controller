/**
  ******************************************************************************
  * @file    board_io.c
  * @brief   板级 IO 抽象：LED 调光（车牌板走 TIM3 PWM）+ 4G 模组电源控制
  *
  * 为什么不用 HAL 的 TIM 驱动：
  *   本工程的 CMake 源文件清单由 CubeMX 生成，其中**没有** stm32f1xx_hal_tim.c，
  *   且 CubeMX 每次重新生成都会覆盖该清单。这里直接操作寄存器，自包含、
  *   不依赖 HAL TIM 模块，也不怕重新生成把配置冲掉。
  *
  * 硬件对应（车牌板，2026-09-23 改版）：
  *   PA6 = TIM3_CH1 → 红灯    PA7 = TIM3_CH2 → 黄灯    PB0 = TIM3_CH3 → 绿灯
  *   PA3 → Q5(AO3400A) → PMOS-Q2(AO3401A) → V4G（4G 模组电源），高电平上电
  ******************************************************************************
  */

#include "board_io.h"
#include <stdio.h>

/* ==========================================================================
 * 车牌板：TIM3 三路 PWM 调光
 * ========================================================================== */
#ifdef LICENSE_PLATE_BOARD

static uint8_t bright_cur[3] = {0, 0, 0};   /* 红/黄/绿 当前亮度 */

static void led_pwm_gpio_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 复用推挽输出。PA6/PA7/PB0 都是 TIM3 的**默认**复用功能，无需 AFIO 重映射 */
  gpio.Mode  = GPIO_MODE_AF_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  gpio.Pin = LED_RED_PIN;   /* PA6 = TIM3_CH1 */
  HAL_GPIO_Init(LED_RED_PORT, &gpio);
  gpio.Pin = LED_YEL_PIN;   /* PA7 = TIM3_CH2 */
  HAL_GPIO_Init(LED_YEL_PORT, &gpio);
  gpio.Pin = LED_GRN_PIN;   /* PB0 = TIM3_CH3 */
  HAL_GPIO_Init(LED_GRN_PORT, &gpio);
}

static void led_pwm_tim_init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();

  /* 时基：72MHz / (255+1) / (255+1) ≈ 1098Hz，CCR 与 8 位亮度 1:1 */
  LED_PWM_TIM->PSC = LED_PWM_PRESCALER;
  LED_PWM_TIM->ARR = LED_PWM_PERIOD;
  LED_PWM_TIM->CNT = 0;

  /* PWM 模式 1（OCxM=110b）：CNT < CCRx 时通道输出有效电平
     CCMR1 布局：OC2M=[14:12] OC2PE=[11] | OC1M=[6:4] OC1PE=[3]
     置 OCxPE 让 CCR 写入在更新事件时生效，避免调光时输出毛刺 */
  LED_PWM_TIM->CCMR1 = (uint16_t)((6u << 4) | (1u << 3)      /* CH1 */
                                | (6u << 12) | (1u << 11));  /* CH2 */
  LED_PWM_TIM->CCMR2 = (uint16_t)((6u << 4) | (1u << 3));    /* CH3 */

  /* 先清零比较值 → 上电瞬间 LED 全灭，避免闪一下 */
  LED_PWM_TIM->CCR1 = 0;
  LED_PWM_TIM->CCR2 = 0;
  LED_PWM_TIM->CCR3 = 0;

  /* 使能三路输出 */
  LED_PWM_TIM->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;

  /* 产生一次更新事件，把 PSC/ARR/CCR 预装载值载入影子寄存器 */
  LED_PWM_TIM->EGR = TIM_EGR_UG;

  /* ARPE=1（ARR 预装载）+ CEN=1（启动计数） */
  LED_PWM_TIM->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void led_set_ccr(uint8_t color, uint8_t bright)
{
  switch (color)
  {
    case LED_COLOR_RED:    LED_PWM_TIM->CCR1 = bright; break;
    case LED_COLOR_YELLOW: LED_PWM_TIM->CCR2 = bright; break;
    case LED_COLOR_GREEN:  LED_PWM_TIM->CCR3 = bright; break;
    default: return;
  }
  bright_cur[color] = bright;
}

void LED_Init(void)
{
  led_pwm_gpio_init();
  led_pwm_tim_init();
  LED_AllOff();
  printf("LED: TIM3 PWM ready (PA6=RED PA7=YEL PB0=GRN, ~%luHz)\r\n",
         (unsigned long)(72000000UL / ((LED_PWM_PRESCALER + 1UL) * (LED_PWM_PERIOD + 1UL))));
}

void LED_SetColor(uint8_t color, uint8_t bright)
{
  /* 单色语义：点亮指定颜色，其余熄灭（与原 GPIO 版行为一致） */
  led_set_ccr(LED_COLOR_RED, 0);
  led_set_ccr(LED_COLOR_YELLOW, 0);
  led_set_ccr(LED_COLOR_GREEN, 0);
  if (color <= LED_COLOR_GREEN) led_set_ccr(color, bright);
}

void LED_SetRGB(uint8_t r, uint8_t y, uint8_t g)
{
  led_set_ccr(LED_COLOR_RED, r);
  led_set_ccr(LED_COLOR_YELLOW, y);
  led_set_ccr(LED_COLOR_GREEN, g);
}

void LED_AllOff(void)
{
  LED_SetRGB(0, 0, 0);
}

uint8_t LED_GetBright(uint8_t color)
{
  return (color <= LED_COLOR_GREEN) ? bright_cur[color] : 0;
}

/* ==========================================================================
 * 最小板：PC13 单色，只能开关（bright 仅作 亮/灭 判定）
 * ========================================================================== */
#else

void LED_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  gpio.Pin   = LED_GRN_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &gpio);
  HAL_GPIO_WritePin(LED_PORT, LED_GRN_PIN, LED_GPIO_OFF);
  printf("LED: PC13 GPIO ready (no PWM)\r\n");
}

static void led_gpio_write(uint8_t bright)
{
  HAL_GPIO_WritePin(LED_PORT, LED_GRN_PIN, bright ? LED_GPIO_ON : LED_GPIO_OFF);
}

void LED_SetColor(uint8_t color, uint8_t bright)
{
  (void)color;          /* 最小板只有绿灯，颜色忽略 */
  led_gpio_write(bright);
}

void LED_SetRGB(uint8_t r, uint8_t y, uint8_t g)
{
  led_gpio_write((uint8_t)((r || y || g) ? 1 : 0));
}

void LED_AllOff(void)
{
  led_gpio_write(0);
}

uint8_t LED_GetBright(uint8_t color)
{
  (void)color;
  return (uint8_t)(HAL_GPIO_ReadPin(LED_PORT, LED_GRN_PIN) == LED_GPIO_ON ? LED_BRIGHT_MAX : 0);
}

#endif /* LICENSE_PLATE_BOARD */

/* ==========================================================================
 * 4G 模组电源开关 V4G
 * ========================================================================== */
#ifdef LICENSE_PLATE_BOARD

void V4G_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin   = V4G_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;          /* 外部已有 100kΩ 下拉，无需内部上下拉 */
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(V4G_PORT, &gpio);

  V4G_Set(1);                        /* 立即上电，否则 4G 模组不启动 */
}

void V4G_Set(uint8_t on)
{
  HAL_GPIO_WritePin(V4G_PORT, V4G_PIN, on ? V4G_ON_LEVEL : V4G_OFF_LEVEL);
}

uint8_t V4G_Get(void)
{
  return (uint8_t)(HAL_GPIO_ReadPin(V4G_PORT, V4G_PIN) == V4G_ON_LEVEL ? 1 : 0);
}

#else  /* 最小板无 4G 电源开关 */

void V4G_Init(void)             { }
void V4G_Set(uint8_t on)        { (void)on; }
uint8_t V4G_Get(void)           { return 1; }

#endif
