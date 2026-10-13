#ifndef RTC_APP_H
#define RTC_APP_H

#include "main.h"

/* ===========================================================================
 * RTC_APP —— F103 内部 RTC（LSE 32.768kHz）作 Stop 唤醒定时器 + 授时维持
 *
 * 纯寄存器实现（不依赖 HAL_RTC 模块），CubeMX 重新生成代码不会冲突：
 *   - LSE 开启 + BDCR 选 LSE 为 RTCCLK + RTCEN
 *   - 20 位分频 32767 -> 1 Hz 秒计数
 *   - 闹钟 = 计数器比较，事件接 EXTI Line17（上升沿），可唤醒 Stop
 *   - RTC 在 VBAT 域：Stop / SW4 复位 / 主电掉电期间照跑，时间不丢
 *
 * 授时来源（可选，不影响定时唤醒本身）：
 *   - RTC_APP_ParseCCLK() 解析 DTU 的 "+CCLK:YYYY/MM/DD,HH:MM:SS"
 *   - 或服务器下发 {"cmd":"time","epoch":<unix秒>} 由 main.c 调 RTC_APP_SetEpoch()
 * ========================================================================= */

/* RTC 闹钟中断置位（RTC_Alarm_IRQHandler 里置 1，主循环消费后清 0） */
extern volatile uint8_t g_rtc_alarm_hit;

/* 初始化：LSE + RTC 使能 + EXTI17 + NVIC。幂等，可重复调用。 */
void     RTC_APP_Init(void);

/* RTC 就绪（LSE 起振成功且 RTC 已使能） */
uint8_t  RTC_APP_Ready(void);

/* 挂闹钟：当前计数 + delay_s 秒后触发。0=成功，1=RTC 未就绪。 */
uint8_t  RTC_APP_ArmIn(uint32_t delay_s);

/* 对表：把计数器写成绝对 epoch 秒。0=成功。 */
uint8_t  RTC_APP_SetEpoch(uint32_t epoch);

/* 读当前计数（若从未对表，则是"距首次初始化的秒数"，仅作相对计时用） */
uint32_t RTC_APP_GetEpoch(void);

/* 在文本里找 "+CCLK:YYYY/MM/DD,HH:MM:SS" 并换算 epoch。
 * 找到返回 1 并写 *epoch（同时把 RTC 对表）。 */
uint8_t  RTC_APP_ParseCCLK(const char *s, uint32_t *epoch);

/* 在文本里找银尔达 DTU 的 "config,nettime,ok,YYYY,M,D,H,M,S,W"（北京时间）
 * 并换算 UTC epoch。找到返回 1 并写 *epoch（同时把 RTC 对表）。 */
uint8_t  RTC_APP_ParseNettime(const char *s, uint32_t *epoch);

/* 由 stm32f1xx_it.c 的 RTC_Alarm_IRQHandler 调用 */
void     RTC_APP_AlarmIRQ(void);

#endif /* RTC_APP_H */
