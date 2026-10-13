#include "rtc_app.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ===========================================================================
 * 纯寄存器 RTC 驱动（F1 的 RTC 是"秒计数器 + 闹钟比较器"，在 VBAT 域）
 *   关键寄存器：
 *     RCC_BDCR  : LSEON/LSERDY/RTCSEL[1:0]/RTCEN（备份域，写前要 DBP=1）
 *     RTC_PRLH/PRLL : 分频（LSE 32768Hz ÷ (PRL+1) = 1Hz）
 *     RTC_CNTL/CNTH : 32 位秒计数
 *     RTC_ALRH/ALRL : 闹钟比较值（相等触发 ALRAF）
 *     RTC_CRH     : ALRAIE（中断使能）/ ALRAE（闹钟使能）
 *     RTC_CRL     : RSF(寄存器同步) RTOFF(写操作完成) CNF(配置模式) ALRAF
 *   写 PRL/CNT/ALR 必须先进配置模式（CNF=1），写完等 RTOFF。
 *   读 CNT 必须先读 CNTH 再读 CNTL（读 CNTH 会锁存 CNTL）。
 * ========================================================================= */

volatile uint8_t g_rtc_alarm_hit = 0;
static uint8_t s_rtc_ok = 0;

#define RTC_EXTI_LINE   (1u << 17)

static void rtc_wait_rtoff(void)
{
  uint32_t guard = 0;
  while (!(RTC->CRL & RTC_CRL_RTOFF))
    if (++guard > 2000000u) break;   /* ≈250ms @8MHz，防呆 */
}

static void rtc_enter_cfg(void)
{
  rtc_wait_rtoff();
  RTC->CRL |= RTC_CRL_CNF;
}

static void rtc_exit_cfg(void)
{
  RTC->CRL &= (uint16_t)~RTC_CRL_CNF;
  rtc_wait_rtoff();
}

void RTC_APP_Init(void)
{
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_BKP_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();          /* PWR_CR DBP=1，解锁备份域写 */

  /* 1) LSE 起振（已就绪则跳过；5s 超时放弃——RTC 不可用但不影响其它功能） */
  if (!(RCC->BDCR & RCC_BDCR_LSERDY))
  {
    uint32_t t0 = HAL_GetTick();
    RCC->BDCR |= RCC_BDCR_LSEON;
    while (!(RCC->BDCR & RCC_BDCR_LSERDY))
    {
      if (HAL_GetTick() - t0 > 5000u)
      {
        printf("RTC: LSE start TIMEOUT -> RTC disabled (no wakeup timer)\r\n");
        return;                        /* s_rtc_ok 保持 0 */
      }
    }
  }

  /* 2) 选 LSE 为 RTCCLK 并使能 RTC。只在未配置过时执行，
        避免重复写入导致备份域复位、丢掉已维持的时间。 */
  if ((RCC->BDCR & (RCC_BDCR_RTCSEL_Msk | RCC_BDCR_RTCEN)) !=
      (RCC_BDCR_RTCSEL_0 | RCC_BDCR_RTCEN))
  {
    RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL_Msk)
              | RCC_BDCR_RTCSEL_0      /* 01: LSE */
              | RCC_BDCR_RTCEN;

    /* 3) 等 APB 与 RTC 内核同步（RSF），首次配置分频/清计数 */
    RTC->CRL &= (uint16_t)~RTC_CRL_RSF;
    {
      uint32_t guard = 0;
      while (!(RTC->CRL & RTC_CRL_RSF))
        if (++guard > 2000000u) break;
    }
    rtc_enter_cfg();
    RTC->PRLL = 0x7FFFu; RTC->PRLH = 0;      /* 32768Hz -> 1Hz */
    RTC->CNTL = 0;       RTC->CNTH = 0;      /* 计数从 0 起 */
    RTC->ALRL = 0xFFFFu; RTC->ALRH = 0xFFFFu;/* 闹钟先放最远 */
    rtc_exit_cfg();
    printf("RTC: first init (LSE/32768 -> 1Hz), counter reset to 0\r\n");
  }
  s_rtc_ok = 1;

  /* 4) EXTI Line17：上升沿 + 不屏蔽（Stop 唤醒通道），清挂起 */
  EXTI->IMR  |= RTC_EXTI_LINE;
  EXTI->RTSR |= RTC_EXTI_LINE;
  EXTI->FTSR &= ~RTC_EXTI_LINE;
  EXTI->PR    = RTC_EXTI_LINE;

  /* 5) NVIC */
  NVIC_SetPriority(RTC_Alarm_IRQn, 2);
  NVIC_EnableIRQ(RTC_Alarm_IRQn);
}

uint8_t RTC_APP_Ready(void) { return s_rtc_ok; }

uint32_t RTC_APP_GetEpoch(void)
{
  uint32_t hi = RTC->CNTH;      /* 先读高位（锁存低位） */
  uint32_t lo = RTC->CNTL;
  return (hi << 16) | lo;
}

uint8_t RTC_APP_SetEpoch(uint32_t epoch)
{
  if (!s_rtc_ok) return 1;
  rtc_enter_cfg();
  RTC->CNTH = epoch >> 16;
  RTC->CNTL = epoch & 0xFFFFu;
  rtc_exit_cfg();
  return 0;
}

uint8_t RTC_APP_ArmIn(uint32_t delay_s)
{
  if (!s_rtc_ok) return 1;
  if (delay_s < 2u) delay_s = 2u;      /* 下限保护：太短来不及休眠 */

  RTC->CRL &= (uint16_t)~RTC_CRL_ALRF;      /* 清旧标志（rc_w0，CNF 外可写） */
  g_rtc_alarm_hit = 0;
  EXTI->PR = RTC_EXTI_LINE;

  uint32_t al = RTC_APP_GetEpoch() + delay_s;
  rtc_enter_cfg();
  RTC->ALRH = al >> 16;
  RTC->ALRL = al & 0xFFFFu;
  RTC->CRH |= RTC_CRH_ALRIE;   /* F1 无 ALRE 位：闹钟比较器恒开，使能中断即可 */
  rtc_exit_cfg();
  return 0;
}

void RTC_APP_AlarmIRQ(void)
{
  if (RTC->CRL & RTC_CRL_ALRF)
  {
    RTC->CRL &= (uint16_t)~RTC_CRL_ALRF;    /* 清标志，否则 IRQ 重入 */
    g_rtc_alarm_hit = 1;
  }
  EXTI->PR = RTC_EXTI_LINE;                  /* 清 EXTI 挂起 */
}

/* ---- "+CCLK:YYYY/MM/DD,HH:MM:SS" 解析 + 对表 ---- */

/* Howard Hinnant 的 days_from_civil：公历日期 -> 自 1970-01-01 的天数 */
static uint32_t days_from_civil(int y, int m, int d)
{
  y -= (m <= 2) ? 1 : 0;
  long era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);                     /* [0,399] */
  unsigned doy = (unsigned)((153 * (m + ((m > 2) ? -3 : 9)) + 2) / 5) + (unsigned)d - 1;
  unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
  return (uint32_t)era * 146097u + doe - 719468u;
}

uint8_t RTC_APP_ParseCCLK(const char *s, uint32_t *epoch)
{
  if (s == NULL) return 0;
  const char *p = strstr(s, "+CCLK:");
  if (p == NULL) return 0;

  int y = 0, mo = 0, d = 0, h = 0, mi = 0, se = 0;
  if (sscanf(p + 6, "%4d/%2d/%2d,%2d:%2d:%2d",
             &y, &mo, &d, &h, &mi, &se) != 6)
    return 0;
  if (y < 2020 || y > 2099 || mo < 1 || mo > 12 || d < 1 || d > 31)
    return 0;

  uint32_t ep = days_from_civil(y, mo, d) * 86400u
              + (uint32_t)h * 3600u + (uint32_t)mi * 60u + (uint32_t)se;
  if (epoch) *epoch = ep;
  RTC_APP_SetEpoch(ep);
  return 1;
}

/* ---- 银尔达 DTU "config,nettime,ok,YYYY,M,D,H,M,S,Weekday" 解析 + 对表 ----
   返回的是北京时间(UTC+8)——DTU 用基站/NTP 时间，实测与本地钟一致。
   epoch = 民用时间 - 8h 得 UTC。2026-10-13 实测样例：
   "config,nettime,ok,2026,10,13,2,17,54,2"。 */
uint8_t RTC_APP_ParseNettime(const char *s, uint32_t *epoch)
{
  if (s == NULL) return 0;
  const char *p = strstr(s, "nettime,ok,");
  if (p == NULL) return 0;
  p += strlen("nettime,ok,");

  int y = 0, mo = 0, d = 0, h = 0, mi = 0, se = 0;
  if (sscanf(p, "%d,%d,%d,%d,%d,%d", &y, &mo, &d, &h, &mi, &se) != 6)
    return 0;
  if (y < 2020 || y > 2099 || mo < 1 || mo > 12 || d < 1 || d > 31)
    return 0;

  uint32_t ep = days_from_civil(y, mo, d) * 86400u
              + (uint32_t)h * 3600u + (uint32_t)mi * 60u + (uint32_t)se
              - 8u * 3600u;                    /* 北京时间 -> UTC */
  if (epoch) *epoch = ep;
  RTC_APP_SetEpoch(ep);
  return 1;
}
