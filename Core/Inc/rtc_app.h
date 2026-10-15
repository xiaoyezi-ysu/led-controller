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

/* 挂闹钟：当前计数 + delay_s 秒后触发。0=成功，1=RTC 未就绪，2=回读校验失败。
 * ⚠ 内部会回读 ALR 校验；只有返回 0 才代表"唤醒源确实武装好了"。 */
uint8_t  RTC_APP_ArmIn(uint32_t delay_s);

/* 唤醒源是否已武装（ArmIn 成功后为 1，闹钟触发后清零）。
 * ⚠ 进 Stop 前必须为 1，否则睡下无人能唤醒 = 变砖。
 * ⚠ 2026-10-15 起本函数还会检查 IRQ 门禁（CRH.ALRIE + NVIC.ISER1 bit9）：
 *   门禁缺失时返回 0，主循环据此拒绝进 Stop。 */
uint8_t  RTC_APP_WakeArmed(void);

/* ★ 2026-10-15 新增：从 Stop 唤醒后重开闹钟中断门禁。
 * 唤醒瞬间 RSF=0，CRH/NVIC 的写入会被丢弃（实测导致低压保护"睡下醒不来"）；
 * 本函数等 RSF=1 后重写 CRH.ALRIE + EXTI17 + NVIC。0=就位，1=失败。 */
uint8_t  RTC_APP_RearmIRQ(void);

/* IRQ 门禁自检结果（RTC_APP_Init 里算好，SWD/日志可查） */
uint8_t  RTC_APP_IRQGateOK(void);

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

/* ---- RTC 软看门狗（方案A："闹钟心跳 + token 判活"）----
 * F1 只有一个 RTC 闹钟、IWDG 在 Stop 照跑且关不掉 ⇒ 用闹钟本身做狗：
 * 任何 ArmIn()（睡眠/阶段/心跳）都算一次"推进"（token++），清醒期主循环
 * 每 ≥1s 重装 +T 心跳；ISR 发现连续 2 发闹钟之间 token 没变 → BKP_DR1
 * 打标记 + NVIC 复位（T=30s 时挂死后 ≤60s 自愈）。睡眠期主循环冻结是
 * 安全的：睡前 ArmIn(sleep_s) 已 bump token。已知边界：关中断的死循环
 * 救不了（无硬件狗兜底）。 */
extern volatile uint32_t g_wdg_kick;   /* token：主流程推进计数（SWD 可读） */
extern volatile uint32_t g_wdg_miss;   /* 连续未认领闹钟数（SWD 可读） */

/* 使能看门狗并挂第一发心跳（RTC 未就绪则静默不启用） */
void     RTC_APP_WDG_Start(uint32_t period_s);

/* 运行中改配置：on=0 关闭；period_s=0 保持原周期 */
void     RTC_APP_WDG_Set(uint8_t on, uint32_t period_s);

/* 主循环清醒期喂狗（内部 ≥1s 节流重装 +T 心跳；睡眠期主循环冻结、不用调） */
void     RTC_APP_WDG_Poll(uint32_t now_ms);

uint8_t  RTC_APP_WDG_On(void);
uint32_t RTC_APP_WDG_Period(void);

/* 开机查询"上次复位是否看门狗所咬"（BKP_DR1 标记，读后自动清）。
 * 须在 RTC_APP_Init() 之后调用（依赖 DBP 与 BKP 时钟）。 */
uint8_t  RTC_APP_WasWdgReset(void);

#endif /* RTC_APP_H */
