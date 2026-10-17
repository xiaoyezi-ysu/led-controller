/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "board_config.h"
#include "board_io.h"
/* 4G 模组驱动二选一（见 board_config.h 的 MODULE_* 宏）：
   两套驱动对外 API 完全一致，故 main.c 其余部分无需分叉。 */
#ifdef MODULE_FSC_AT
#include "dtu_ctrl.h"      /* 飞思创 AT 指令固件 */
#else
#include "dtu.h"           /* 银尔达 YED_DTU4 透传固件（默认） */
#endif
#include "sc7a20h.h"
#include "rtc_app.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;/* SC7A20H I2C address (7-bit 0x18, write=0x30, read=0x31) */
#define SC7A20H_ADDR  0x31
#define SC7A20H_CTRL1 0x20
#define SC7A20H_CTRL4 0x23
#define SC7A20H_OUT_X 0x28

/* USART1 (4G) single-byte RX */
uint8_t uart1_rx_byte;

/* USART3 (PC) single-byte RX */
uint8_t uart3_rx_byte;

/* Ring buffer: fast ISR→main loop forwarding */
#define RBUF_SIZE  256
static volatile uint8_t rbuf[RBUF_SIZE];
static volatile uint16_t rbuf_wr = 0;
static uint16_t rbuf_rd = 0;
volatile uint8_t dbg_trigger = 0;   /* COM21 收到 '~' 时置位，主循环执行调试探针 */

/* SWD 调试注入通道 -----------------------------------------------------------
   脚本用 SWD 写 g_dbg_tx[] 再写 g_dbg_tx_len，主循环把这段字节**一次性**发给 4G 模组。
   为什么需要它：没有 PC 调试串口时，只能用 SWD 逐字节写 USART1->DR 来对模组说话，
   但那样每字节之间会插进 200~400us 的 SWD 事务空隙，足以把银尔达的
   "config,get,xxx\r\n" 整条命令切碎成多帧 —— 模组按帧识别，于是「发了但没反应」。
   交给 HAL_UART_Transmit 连续发送才能形成完整命令帧。
   模组的应答仍走正常 UART 中断 -> rbuf / json_buf，脚本照旧读得到。 */
volatile uint8_t g_dbg_tx[128];
volatile uint8_t g_dbg_tx_len = 0;
uint8_t g_low_power = 0;            /* 1=睡眠模式：STM32 深度空闲，仅被 4G 模组 UART 唤醒；
                                         模组维持 MQTT 连接，LED 已熄灭 */

/* 4G auto-test state machine */
extern _4G_State _4g_state;

/* ICCID from flash or AT+ICCID? query */
extern char my_iccid[32];
extern uint8_t iccid_loaded;

/* SC7A20H 姿态传感器（软 I2C: PB6=SCL, PB7=SDA）
   全部放 volatile 全局 —— 没有 PC 调试串口时，用 SWD 直接读这些地址
   （同 g_dbg_tx 的思路；符号地址用 arm-none-eabi-nm 查）。
   g_sc7_id 是总开关：非 0x11 时主循环跳过整个读数分支，
   所以想临时关掉传感器，只需在 init 处不调用 Driver_Init 即可。 */
volatile uint8_t  g_sc7_id  = 0;    /* Driver_Init 返回：0x11 = 芯片就绪/应答正常 */
volatile uint8_t  g_sc7_ok  = 0;    /* 1 = 最近一次读数有效 */
volatile uint8_t  g_sc7_err = 0;    /* 累计 I2C 读失败次数 */
volatile int16_t  g_sc7_x = 0, g_sc7_y = 0, g_sc7_z = 0;   /* 原始加速度 LSB */
volatile int16_t  g_sc7_roll  = 0;  /* 横滚角 roll，单位 0.1°（绕 X 轴） */
volatile int16_t  g_sc7_pitch = 0;  /* 俯仰角 pitch，单位 0.1°（绕 Y 轴） */
volatile uint16_t g_sc7_cnt   = 0;  /* 成功读取累计次数 */

/* MQTT JSON receive buffer */
#define JSON_BUF_SIZE  256
uint8_t json_buf[JSON_BUF_SIZE];
uint16_t json_len = 0;
volatile uint8_t json_ready = 0;
volatile uint32_t json_last_rx = 0;

/* 延时 ACK --------------------------------------------------------------
   银尔达 DTU 透传固件在向 MCU 下发服务器数据后的短时间内会丢弃 MCU 立刻回发
   的数据：实测「紧接下行发出的 ACK 永远到不了 MQTT」，而内容完全相同的字节
   晚一点再发就能正常上报（伪造 json_ready 触发 ACK 可立即上报即为反证）。
   所以 ACK 不再当场发送，而是排进队列，等 ACK_DELAY_MS 之后再发。
   等待期间若又来新指令，直接覆盖旧 ACK —— 拖动亮度滑块连发时只保留最后一条。 */
#define ACK_DELAY_MS   800u
static char     g_ack_buf[96];
static uint32_t g_ack_at = 0;
static uint8_t  g_ack_pending = 0;

/* ===== lp_cycle：RTC 定时唤醒循环 =========================================
   收到 {"cmd":"lp_cycle","period_min":N,"cycles":M} 后进入循环模式
   （period_min = 睡眠时长；cycles = 循环次数，0 = 无限，睡满 M 轮自动退出）：
     会话(4G 开, ~90s)：上报 report + 服务器可下发 {"cmd":"time","epoch":..} 对表
     收尾：+++ 进指令模式 → AT+CCLK? 授时（探测式，失败也无妨）→ 挂 RTC 闹钟
           → 关 4G → Stop（电流 ≈1.16 mA，见 docs/r24-removed-1mA.html）
     唤醒：RTC 闹钟 EXTI17 自动唤醒 → 重开会话
   ★ 2026-10-15 变更：**上电后默认自动进低功耗循环，周期 2 h**（生产形态）。
     旧版「SW4 复位/重新上电后回到普通在线模式」改为：上电先在线 AUTO_LP_BOOT_MS
     (15s) 让 4G/MQTT 起来，随后若无干预则自动进 2h 循环。
     防变砖 / 逃生：
       - RTC 未就绪 ⇒ 拒绝自动进入，留常在线（绝不下睡，杜绝睡死）。
       - 上电 15s 窗口内从 PC 串口发 "!online" ⇒ 取消本次自动低功耗（只影响本次）。
       - BKP_DR3 打 0x4F46('OF') ⇒ **永久禁用**自动低功耗（VBAT 域，掉电不丢）；
         用 "!lpon" 命令清除该标志、恢复自动低功耗。
       - 服务器 lp_cycle_off 仍可随时退出当前循环（回常在线）。 */
#define LP_SESSION_MS    90000u   /* 每个会话最长在线时长 */
#define LP_REPORT_AT_MS  45000u   /* 会话内上报时刻（MQTT 通常已连上） */
/* ★ 2026-10-15：上电默认周期 = 2 h（生产形态）。指令 period_min 可覆盖。 */
#define LP_DEFAULT_PERIOD_S  600u    /* 调试期 10 min（量产改回 7200u=2h） */
/* ★ 上电自动进低功耗循环的状态机参数（见 AUTO_LP_* 说明） */
#define AUTO_LP_BOOT_MS      15000u  /* 上电后先在线 15s：等 4G/MQTT 起来、留 PC 逃生窗口 */
#define AUTO_BKP_DR          BKP->DR3  /* VBAT 域持久标志：0x4F46=自动低功耗被禁用 */
#define AUTO_BKP_OFF_MAGIC   0x4F46u   /* 'O''F'（Off）：用户主动禁用自动低功耗 */

/* ===== 电池低压保护（Low-Battery Stop）=====================================
   ★ 2026-10-15：VBAT 掉到阈值以下 → 自动进 Stop 省电 → 定时醒来复测 →
     电压回升到恢复阈值以上就回常在线（带迟滞，避免在阈值附近反复抖动）。
   语义：
     - 监测量 = g_bat_mv（VBAT，ADC 指令同款口径，含 VREFINT 反算）。
     - 进睡阈值 3.00 V（单节锂电过放保护线），恢复阈值 3.30 V（迟滞 0.3 V）。
     - 睡下后挂 RTC 闹钟 BAT_LP_POLL_S(60s) 定时醒来复测；仍低就继续睡。
     - 防变砖：RTC 未就绪绝不下睡（复用既有 Stop 门禁）。
     - 与 lp_cycle 不冲突：本功能在 lp_cycle 未使能时才动作（lp_cycle 优先）。
   ESC：'!batlp off' 关闭本功能（BKP_DR4=0x4F46 持久），'!batlp on' 恢复。 */
#define BAT_LP_ENTER_MV      3000u   /* VBAT < 3.00 V → 进 Stop */
#define BAT_LP_RESUME_MV     3300u   /* VBAT > 3.30 V → 回在线（迟滞 0.3 V） */
#define BAT_LP_POLL_S        60u     /* 睡下后每 60s 醒一次复测 */
#define BAT_LP_CHECK_MS      10000u  /* 在线期每 10s 测一次电池 */
#define BAT_BKP_DR           BKP->DR4  /* VBAT 域：0x4F4F=低压保护显式开启 */
#define BAT_BKP_OFF_MAGIC    0x4F46u
#define BAT_BKP_ON_MAGIC     0x4F4Fu
static uint8_t  s_batlp_en     = 0;   /* 0 = 调试期默认关闭（!batlp on 显式开启，BKP 持久） */
static uint8_t  s_bat_sleep    = 0;   /* 1 = 正因低压处于 Stop 睡眠态 */
static uint32_t s_bat_last_ms  = 0;   /* 上次电池检测时刻 */
static uint16_t s_bat_mv_last  = 0;   /* 最近一次 VBAT（SWD 可读） */

static uint8_t  g_lp_cycle   = 0;     /* 1 = 循环模式使能 */
static uint32_t g_lp_period_s = LP_DEFAULT_PERIOD_S; /* 上报周期（秒），指令 period_min 设定 */
static uint16_t g_lp_cycles  = 0;     /* 循环次数上限：0 = 无限；N>0 = 睡满 N 轮后自动退出 */
static uint8_t  s_auto_lp       = 0;  /* 1 = 本次上电要做自动低功耗（尚未启动） */
static uint8_t  s_auto_lp_done  = 0;  /* 1 = 本轮已启动或已取消 */
static uint16_t s_lp_done    = 0;     /* 已完成的睡眠轮数（每次 RTC 唤醒 +1） */
static uint8_t  s_lp_state    = 0;    /* 0 idle 1 session 2 closing 3 sleeping */
static uint32_t s_lp_t0       = 0;    /* 会话开始时刻 */
static uint8_t  s_lp_reported = 0;

/* ===== 电池电压 ADC 采样（车牌板 PA1 = ADC1_IN1，ADC_BAT 分压）==============
   ver2 原理图：VBAT → PMOS-Q1(栅由 PA2/BAT_EN 经 NMOS-Q5 驱动) → R7 100k
                → ADC_BAT → R8 220k → GND
   ⇒ VBAT = Vadc × (R7+R8)/R8 = Vadc × 320k/220k = Vadc × 1.4545
   ★ 极性（2026-10-15 实测定案）：PA2=1 → Q5 导通 → Q1 栅拉低 → 支路**接通**；
     PA2=0 → 支路**断开**（静态 0 耗流）。旧代码写反过，务必以本行为准。
   ★ 基准（同日实测定案）：**VDDA 必须实测**，不能假设 3.3 V。
     本板电源为光伏/锂电 → CN3142 充电 → VBAT → LDO ME6206A33 → VCC33，
     但调试期曾把 VBAT 直短到 3V3 网络 ⇒ VDDA 变成 4.0~4.2 V，用 3300 假设
     去算会系统性偏低 ~20%。用 VREFINT 反算后两种接法都能自动适配：
       VDDA(实测) = 1.20V × 4095 / vrefint_raw
     注意 F1 上 VREFINT 会随 VDDA 一起变 ⇒ **纯分压比的 raw 恒 ≈ 0.6875×4095**，
     只有 VREFINT 反算才测得出绝对电压。VREFINT 标称 1.20V ±5% ⇒ 绝对精度 ±5%。
   ★ 采样时间：源阻抗 = R7∥R8 ≈ 68.6 kΩ、C14 100 nF ⇒ RC ≈ 6.9 ms，
     故必须等 ~50 ms 再采，且 ADC 采样时间用 239.5 周期（旧 1.5 周期严重不足）。 */
#define BAT_ADC_DIV_NUM   320u   /* R7 + R8 (kΩ) */
#define BAT_ADC_DIV_DEN   220u   /* R8 (kΩ) */
#define BAT_ADC_VREF_MV   3300u  /* 退路：VREFINT 反算失败时才用（正常不该走到） */
volatile uint16_t g_bat_adc_raw = 0;      /* 最近一次 ADC 原始值（0..4095） */
volatile uint16_t g_bat_mv      = 0;      /* 电池电压 mV（已乘分压比） */
volatile uint16_t g_bat_adc_mv  = 0;      /* ADC 引脚电压 mV（分压点） */
volatile uint16_t g_vrefint_raw = 0;      /* VREFINT(1.2V) 原始值，0=未测 */
volatile uint16_t g_vdda_mv     = 0;      /* 实测 VDDA mV（VREFINT 反算，±5%） */

/* 读内部基准 VREFINT（ADC1_IN17，典型 1.20 V）反算 VDDA。
   用于把"假设 3.3 V"变成实测基准 —— 2026-10-15 排查电压偏低时加入。 */
static uint16_t adc_read_vdda(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  uint16_t raw;

  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;   /* VREFINT 要求 ≥5 µs */
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) return 0xFFFFu;
  if (HAL_ADC_Start(&hadc1) != HAL_OK) return 0xFFFFu;
  if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK) { HAL_ADC_Stop(&hadc1); return 0xFFFFu; }
  raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  /* 恢复 PA1 通道，别影响后续 bat_adc_sample */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) return 0xFFFFu;

  if (raw < 100) return 0xFFFFu;      /* 异常保护 */
  g_vrefint_raw = raw;
  g_vdda_mv = (uint16_t)((1200u * 4095u) / raw);
  return g_vdda_mv;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void SC7A20H_Init(void);
static void SC7A20H_ReadAccel(int16_t *x, int16_t *y, int16_t *z);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 从 JSON 文本里取整数键值：{"bright":180} → 180。
   工程里没有 JSON 库（DTU 侧同样是字符串匹配风格），这里做个最小实现：
   定位 "\"key\"" 后跳过空白/冒号/引号，再 atoi；取不到或越界时返回 def。 */
static int json_int(const char *json, const char *key, int def, int lo, int hi)
{
  const char *p = strstr(json, key);
  if (p == NULL) return def;
  p += strlen(key);
  while (*p == ' ' || *p == ':' || *p == '\t') p++;
  if (*p < '0' || *p > '9') return def;
  int v = atoi(p);
  if (v < lo) v = lo;
  if (v > hi) v = hi;
  return v;
}

/* ===== 电池电压采样（bat_adc_sample）=====================================
   时序（约 15 ms，阻塞，不影响 4G 会话）：
     ① PA2=0 拉低 → NMOS-Q5 导通 → PMOS-Q1 栅被拉低 → Q1 导通，分压支路接上 VBAT
     ② 等 ≥2 ms 让 R7(100k)+R8(220k) 与 C14(100nF) 的 RC 稳定
        （τ = 100k ∥ 220k × 100n ≈ 6.9 µs，2 ms 余量足够）
     ③ 单次软件触发采样（MX_ADC1_Init 已配 ADC_CHANNEL_1 / 1.5 周期采样）
     ④ 立即 PA2=1 断开支路（恢复 0 耗流；分压支路静态 320kΩ 跨 VBAT ≈ 11 µA）
   返回 ADC 原始值（0..4095）；HAL 失败返回 0xFFFF。
   注：采样期间模组/LED 的脉冲负载会拉低 VBAT（电池内阻），
       所以读数是"此刻"电压，低压告警请多次取最小值。 */
static uint16_t bat_adc_sample(void)
{
  uint16_t raw = 0, lo = 0xFFFFu, hi = 0;

  /* 防御：MX_ADC1_Init() 未跑过时 hadc1.Instance == NULL，直接喂给 HAL 会
     在 ADC_Enable() 里写空指针 => HardFault。这里先挡掉（2026-10-15 踩过）。 */
  if (hadc1.Instance == NULL) { g_bat_adc_raw = 0xFFFFu; return 0xFFFFu; }

  /* ⚠ 极性对照原理图（2026-10-15 实测定案）：BAT_EN(PA2)=高 → R29 → NMOS-Q5
     栅极高 → Q5 导通 → PMOS-Q1 栅被拉低 → Q1 导通 ⇒ 接通分压支路 = PA2=1！
     （旧版写反成 PA2=0 接通：采样时支路其实是断的，读到的是 C14 浮空残荷，
     数值系统性偏低 ~30% 且无意义；平时 PA2=1 反而把支路常通，白漏 12 µA。） */
  GPIOA->BSRR = GPIO_PIN_2;                 /* PA2 = 1，Q5/Q1 导通接通分压支路 */
  /* ⚠ 等待时间定案（2026-10-15 实测）：τ = (R7∥R8)×C14 = 68.6kΩ × 100nF ≈ 6.9 ms
     （旧注释写 6.9 µs 是 µs/ms 笔误！旧等待 3 ms = 0.43τ，节点还在爬坡，
     实测只读出 2.2 V / 真值 2.72 V）。50 ms ≈ 7.2τ，误差 <0.1%。 */
  HAL_Delay(50);

  /* 连续采 8 次取中值（去脉冲噪声） */
  uint8_t i;
  for (i = 0; i < 8; i++)
  {
    if (HAL_ADC_Start(&hadc1) != HAL_OK) { GPIOA->BRR = GPIO_PIN_2; return 0xFFFFu; }
    if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK) { HAL_ADC_Stop(&hadc1); GPIOA->BRR = GPIO_PIN_2; return 0xFFFFu; }
    raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    if (raw < lo) lo = raw;
    if (raw > hi) hi = raw;
  }
  GPIOA->BRR = GPIO_PIN_2;                  /* PA2 = 0，Q5/Q1 关断，支路断开省电 */

  raw = (uint16_t)((lo + hi) / 2u);         /* 去极值中值，抗单点毛刺 */
  g_bat_adc_raw = raw;

  /* 用 VREFINT 实测 VDDA 做基准（测不出则退回 3300 假设值）。
     ⚠ 2026-10-15 实锤：本板 VCC33 = 电池直通（万用表实测 4.0~4.2V），
     根本不是 3.3V！旧 3300 假设导致读数系统性偏低 ~20%。
     且 VDDA=VBAT ⇒ 纯分压的 raw 恒 ≈ 0.6875×4095，只有 VREFINT 反算才测得出
     绝对电压。VREFINT 标称 1.20V ±5%，绝对精度 ±5% 量级。 */
  uint16_t vdda = adc_read_vdda();
  uint16_t vref = (vdda != 0xFFFFu && vdda > 2500 && vdda < 4600) ? vdda : BAT_ADC_VREF_MV;

  g_bat_adc_mv  = (uint16_t)(((uint32_t)raw * vref) / 4095u);
  g_bat_mv      = (uint16_t)(((uint32_t)g_bat_adc_mv * BAT_ADC_DIV_NUM) / BAT_ADC_DIV_DEN);
  return raw;
}

/* ===== lp_cycle 辅助函数 ================================================== */

/* 会话收尾（阻塞 ≈6s，时序不敏感）：
   等 ACK 发完 → +++ 进指令模式 → config,get,nettime 授时（~1.5s，无应答跳过）
   → AT+ENTM 回透传 → 挂 RTC 闹钟 → 关 4G → 进睡眠标志（主循环下一个 WFI 即 Stop）
   sleep_s: RTC 闹钟秒数（常规 lp_cycle = g_lp_period_s）
   led_off: 1 = 收尾灭灯（常规）；0 = 保持点灯（工况测试的「窗口轮」最坏组合档）
   返回：1 = 已进入睡眠态（闹钟可靠武装）；0 = 中止（RTC 未就绪/回读校验失败），
         调用方**必须**放弃进 Stop，否则睡下无人唤醒 = 变砖。 */
static uint8_t LP_CloseAndSleepEx(uint32_t sleep_s, uint8_t led_off)
{
  /* 等 ACK 队列发完（最多 1.5s）。注意此时主循环被本函数阻塞，
     ACK 必须就地发送，不能指望主循环——否则 ACK 永远发不出去。 */
  uint32_t t0 = HAL_GetTick();
  while (g_ack_pending && HAL_GetTick() - t0 < 1500u)
  {
    if ((int32_t)(HAL_GetTick() - g_ack_at) >= 0)
    {
      g_ack_pending = 0;
      HAL_UART_Transmit(&UART_4G, (uint8_t*)g_ack_buf, (uint16_t)strlen(g_ack_buf), 200);
      printf("LP: ack flushed\r\n");
    }
  }

  /* +++ 前后各留 ≥1s 静默（模组退出透传的时序要求，见 dtu.c ESC_GUARD_MS） */
  HAL_Delay(1100);
  HAL_UART_Transmit(&UART_4G, (uint8_t *)"+++", 3, 200);
  HAL_Delay(1200);

  /* 试探授时：+++ 已进指令模式，阻塞问 DTU 要网络时间（~1.5s，PC 直连实测
     nettime 秒回 0.15s；ntptime=1 已持久化，时间走 NTP 校准）。
     无应答只跳过授时，不影响后续流程；定时唤醒用相对闹钟，不依赖绝对时间。 */
  rbuf_rd = rbuf_wr;                       /* 丢掉 +++ 前的旧数据 */
  {
    uint32_t ep = 0;
    if (DTU_QueryNettime(&ep))
      printf("LP: RTC synced from DTU nettime, epoch=%u\r\n", ep);
    else
      printf("LP: no nettime reply -> keep old RTC count\r\n");
  }
  HAL_UART_Transmit(&UART_4G, (uint8_t *)"AT+ENTM\r\n", 9, 200);
  HAL_Delay(300);

  /* 挂闹钟（对表之后，闹钟基于最新计数）。返回 0 才代表唤醒源可靠武装：
     ArmIn 内部会回读 ALR 校验（写失败/被覆盖 = 睡死）。 */
  if (RTC_APP_ArmIn(sleep_s) != 0)
  {
    printf("LP: RTC alarm NOT armed -> abort cycle, stay online\r\n");
    g_lp_cycle = 0;
    s_lp_state = 0;
    g_low_power = 0;
    HAL_ResumeTick();
    V4G_Set(1);
    return 0;
  }
  printf("LP: alarm armed +%us, entering Stop (4G cut)\r\n", (unsigned)sleep_s);

  /* 关 4G、清接收态、置睡眠标志；主循环顶部 WFI 即进 Stop */
  V4G_Set(0);
  json_len = 0; json_ready = 0;
  rbuf_rd = rbuf_wr;
  if (led_off) LED_AllOff();
  g_low_power = 1;
  DTU_SetLowPower(1);
  s_lp_state = 3;
  return 1;
}

/* lp_cycle 常规收尾：睡 g_lp_period_s 秒、灭灯。返回同 LP_CloseAndSleepEx */
static uint8_t LP_CloseAndSleep(void)
{
  return LP_CloseAndSleepEx(g_lp_period_s, 1);
}

/* ===== 工况测试引擎（bench test）=========================================
   目的：把「一整天的不同工况」压进一次 F-11 连续采样里跑完。
   入口：PC 串口（USART3）发一行 '!' 起头的命令
       !seq 60               内置 13 档工况表（常规档 60s，4G 重档 ×2）
       !day 30 3 G 255       压缩日：窗口外压到 30s、窗口内 3 轮真联网、绿灯 255
       !stop                 中止测试，回常在线
       !help                 帮助

   为什么走 PC 串口而不是 MQTT：MQTT 侧任何被识别的指令都会命中主循环里的
   g_low_power = 0; DTU_SetLowPower(0)，顺手把被测状态清掉（v2 报告里 LED 档
   读数偏高的根因）。PC 通道完全不碰 DTU，置态干净。

   每档进入时打印一行 ASCII 标记（供出图脚本自动分段标注）：
       SEQ,<索引>,<名称>,<标称秒>,<tick_ms>
   与本机 F-11 的 CSV 按时间戳对齐。 ======================================== */

#define SEQ_MAX_STEPS   16
#define SEQ_WIN_ROUND_S 155u   /* 窗口整轮标称 = 会话 90 + 收尾 ≈5 + 睡 60 */

/* 步类型 */
enum {
  SEQK_V4G_OFF = 0,   /* 4G 断电 + 灯灭（Run 对照基线） */
  SEQK_V4G_ON,        /* 4G 上电注网 */
  SEQK_LED,           /* 只动灯，4G 态保持 */
  SEQK_HB_ON,         /* 在线 + 心跳/探活开 */
  SEQK_HB_OFF,        /* 在线 + 心跳/探活关（4G 仍供电，DTU_SetLowPower(1)） */
  SEQK_LP_ROUND,      /* 真 lp_cycle 一轮：会话 90s → 收尾 → Stop sleep_s */
  SEQK_SLEEP          /* 纯 Stop 睡 sleep_s（4G 断、灯灭、无会话） */
};

typedef struct {
  const char *name;     /* ASCII 短名（打印/出图分段） */
  uint16_t    dwell_s;  /* 标称驻留（SLEEP / LP_ROUND 步为标称总时长） */
  uint16_t    sleep_s;  /* SLEEP / LP_ROUND：RTC 睡眠秒数 */
  uint8_t     kind;
  uint8_t     led;      /* LED_COLOR_*；0xFF = 灭 */
  uint8_t     bright;
} seq_step_t;

/* 内置 13 档工况表：合计约 1185s ≈ 20min（见 docs/rtc-commute-window-schedule.html） */
static const seq_step_t SEQ_TABLE[] = {
  { "v4goff_dark",  60,                  0, SEQK_V4G_OFF, 0xFF,                0 },
  { "v4goff_G255",  60,                  0, SEQK_LED,     LED_COLOR_GREEN,   255 },
  { "v4goff_G128",  60,                  0, SEQK_LED,     LED_COLOR_GREEN,   128 },
  { "v4goff_R255",  60,                  0, SEQK_LED,     LED_COLOR_RED,     255 },
  { "v4goff_Y255",  60,                  0, SEQK_LED,     LED_COLOR_YELLOW,  255 },
  { "v4goff_R64",   60,                  0, SEQK_LED,     LED_COLOR_RED,      64 },
  { "v4gon_reg",   120,                  0, SEQK_V4G_ON,  0xFF,                0 },
  { "online_hb",   120,                  0, SEQK_HB_ON,   0xFF,                0 },
  { "online_idle",  60,                  0, SEQK_HB_OFF,  0xFF,                0 },
  { "round_dark",  SEQ_WIN_ROUND_S,     60, SEQK_LP_ROUND, 0xFF,               0 },
  { "round_G255",  SEQ_WIN_ROUND_S,     60, SEQK_LP_ROUND, LED_COLOR_GREEN,   255 },
  { "round_G128",  SEQ_WIN_ROUND_S,     60, SEQK_LP_ROUND, LED_COLOR_GREEN,   128 },
  { "v4goff_end",   60,                  0, SEQK_V4G_OFF, 0xFF,                0 },
};
#define SEQ_TABLE_N (sizeof(SEQ_TABLE) / sizeof(SEQ_TABLE[0]))

/* 运行时副本（!seq 允许改常规档驻留，表本身是 const） */
static seq_step_t SEQ_RUN[SEQ_MAX_STEPS];
static seq_step_t SEQ_DAY[SEQ_MAX_STEPS];
static char       seq_name_pool[SEQ_MAX_STEPS][16];

static seq_step_t *s_seq_tab = SEQ_RUN;
static uint8_t  s_seq_n    = 0;    /* 步数 */
static uint8_t  s_seq_i    = 0;    /* 当前步 */
static uint8_t  s_seq_run  = 0;    /* 1 = 测试进行中 */
static uint8_t  s_seq_stop = 0;    /* 1 = 已挂闹钟，主循环顶部进 Stop */
static uint8_t  s_seq_wait = 0;    /* 1 = 正在 Stop，等 RTC 唤醒后推进 */
static uint32_t s_seq_t0   = 0;    /* 本步进入时刻 */
static uint16_t s_seq_dwell = 0;   /* 本步标称驻留（秒） */
static uint16_t s_seq_sleep = 0;   /* 本步 RTC 睡眠秒数 */
static uint8_t  s_seq_round = 0;   /* LP_ROUND 子状态：0 无 / 1 会话中 / 2 待收尾 */
static uint32_t s_seq_rt0   = 0;   /* LP_ROUND 会话起点 */
static uint8_t  s_seq_reported = 0;

/* PC '!' 命令行缓冲（ISR 填、主循环消费） */
static char     pc_cmd[48];
static uint8_t  pc_cmd_len  = 0;
static uint8_t  pc_cmd_mode = 0;
volatile uint8_t pc_cmd_ready = 0;

/* ---- SWD 注入通道 + RAM 标记日志（没有 PC 串口时用脚本直接写/读 RAM）----
   bench/bench_swd.py 用 OpenOCD 的 mww/mdw 做两件事：
     ① 写 g_test_cmd[]（不含前导 '!'）+ g_test_go=1 → 主循环调 SEQ_HandleCmd()
     ② 回读 g_slog（每步的名称 / 步号 / 进入时刻）→ 与 F-11 曲线对齐
   时标选 RTC 秒计数：Stop 期间 SysTick 被挂起会冻结，而 RTC（LSE / VBAT 域）
   一直在走，所以它是本测试里唯一能跨 Stop 的时间基准。
   与 PC 串口通道互不干扰：两条路径都只是喂给同一个 SEQ_HandleCmd()。 */
volatile char    g_test_cmd[48] __attribute__((aligned(4)));
volatile uint8_t g_test_go      __attribute__((aligned(4)));

/* 复位原因遥测：开机读 RCC_CSR（IWDGRSTF/WWDGRSTF/SFTRSTF/PINRSTF/LPWRRSTF）
   + RTC 软看门狗的 BKP_DR1 标记。SWD 可读，也随 MQTT report 上报。 */
volatile uint32_t g_rst_csr __attribute__((aligned(4))) = 0;
volatile uint8_t  g_rst_wdg __attribute__((aligned(4))) = 0;

/* 4G 模组健康时间戳：RX ISR 每收到一个字节就刷新。
   dtu.c 用它判"模组死透"（MODULE_SILENT_MS 内零字节）→ 硬拉电恢复。
   旧版只看 ssta 轮询超时，模组真死时无人应答也会被当成"没连上"而假 READY。 */
volatile uint32_t g_4g_last_rx_ms __attribute__((aligned(4))) = 0;
volatile uint32_t g_4g_rx_bytes   __attribute__((aligned(4))) = 0;

#define SEQ_LOG_N 24
typedef struct {
  uint8_t  n;                   /* 已记录步数 */
  uint8_t  done;                /* 1 = 序列跑完 */
  uint8_t  ix[SEQ_LOG_N];       /* 步号 */
  uint8_t  pad[2];
  uint32_t rtc[SEQ_LOG_N];      /* 进入该步时的 RTC 秒计数 */
  char     name[SEQ_LOG_N][16]; /* 步名（ASCII），供脚本直接回读 */
} seq_log_t;
volatile seq_log_t g_slog __attribute__((aligned(4)));

/* 进入第 i 步：置态 + 记录标记 + 打印 */
static void SEQ_Enter(uint8_t i)
{
  const seq_step_t *st = &s_seq_tab[i];

  s_seq_i     = i;
  s_seq_t0    = HAL_GetTick();
  s_seq_stop  = 0;
  s_seq_wait  = 0;
  s_seq_round = 0;
  s_seq_dwell = st->dwell_s;
  s_seq_sleep = st->sleep_s;

  /* 标记日志：先记录再置态，时间戳贴近真实切换时刻 */
  if (g_slog.n < SEQ_LOG_N)
  {
    uint8_t k = g_slog.n;
    g_slog.ix[k]  = i;
    g_slog.rtc[k] = RTC_APP_GetEpoch();
    strncpy((char *)g_slog.name[k], st->name, 15);
    ((char *)g_slog.name[k])[15] = '\0';
    g_slog.n = (uint8_t)(k + 1u);
  }

  printf("SEQ,%u,%s,%u,%lu\r\n", (unsigned)i, st->name, (unsigned)st->dwell_s,
         (unsigned long)HAL_GetTick());

  switch (st->kind)
  {
  case SEQK_V4G_OFF:
    V4G_Set(0);
    LED_AllOff();
    g_low_power = 0; DTU_SetLowPower(0);
    break;

  case SEQK_V4G_ON:
    V4G_Set(1);
    break;

  case SEQK_LED:
    if (st->led == 0xFF) LED_AllOff();
    else                 LED_SetColor(st->led, st->bright);
    break;

  case SEQK_HB_ON:
    g_low_power = 0; DTU_SetLowPower(0);
    break;

  case SEQK_HB_OFF:
    DTU_SetLowPower(1);      /* 4G 仍供电，只是停掉心跳/探活 */
    break;

  case SEQK_SLEEP:
    V4G_Set(0);
    LED_AllOff();
    g_low_power = 0; DTU_SetLowPower(0);
    if (RTC_APP_ArmIn(st->sleep_s ? st->sleep_s : 1u) != 0)
    {
      printf("SEQ: RTC not ready -> abort sequence\r\n");
      s_seq_run = 0;
      return;
    }
    printf("SEQ: sleep %us -> Stop\r\n", (unsigned)st->sleep_s);
    g_low_power = 1;
    s_seq_stop  = 1;
    s_seq_wait  = 1;
    break;

  case SEQK_LP_ROUND:
    /* 真一轮：4G 上电走正常注网/MQTT，会话 90s 后收尾进 Stop(sleep_s)。
       灯在整个窗口保持（最坏组合档），收尾不灭灯。 */
    V4G_Set(1);
    g_low_power = 0; DTU_SetLowPower(0);
    if (st->led == 0xFF) LED_AllOff();
    else                 LED_SetColor(st->led, st->bright);
    s_seq_rt0      = HAL_GetTick();
    s_seq_reported = 0;
    s_seq_round    = 1;
    break;

  default:
    break;
  }
}

/* 推进到下一步；序列跑完则回常在线 */
static void SEQ_Advance(void)
{
  uint8_t next = (uint8_t)(s_seq_i + 1u);

  if (next >= s_seq_n)
  {
    printf("SEQ,DONE,%u,%lu\r\n", (unsigned)s_seq_n, (unsigned long)HAL_GetTick());
    g_slog.done = 1;                  /* 供 SWD 回读判定收尾 */
    s_seq_run  = 0;
    s_seq_stop = 0;
    s_seq_wait = 0;
    V4G_Set(1);
    g_low_power = 0; DTU_SetLowPower(0);
    LED_AllOff();
    return;
  }
  SEQ_Enter(next);
}

/* 组装工况表：!seq [dwell_s] */
static void SEQ_BuildSeq(uint16_t dwell)
{
  uint8_t k;

  if (dwell < 5) dwell = 5;

  for (k = 0; k < SEQ_TABLE_N && k < SEQ_MAX_STEPS; k++)
  {
    SEQ_RUN[k] = SEQ_TABLE[k];
    if (SEQ_RUN[k].kind == SEQK_LED || SEQ_RUN[k].kind == SEQK_V4G_OFF)
      SEQ_RUN[k].dwell_s = dwell;
    else if (SEQ_RUN[k].kind == SEQK_V4G_ON || SEQ_RUN[k].kind == SEQK_HB_ON)
      SEQ_RUN[k].dwell_s = (uint16_t)(dwell * 2u);  /* 4G 重档驻留加倍：空闲电平漂移大 */
    /* LP_ROUND 保持 155 = 90 会话 + 5 收尾 + 60 睡眠 */
  }
  s_seq_tab = SEQ_RUN;
  s_seq_n   = (uint8_t)SEQ_TABLE_N;
}

/* 组装压缩日：!day [idle_s] [rounds] [R|Y|G] [bright]
   结构 = 夜 Stop → 早高峰 rounds 轮 → 日间 Stop → 晚高峰 rounds 轮 → 夜 Stop */
static void SEQ_BuildDay(uint16_t idle_s, uint8_t rounds, uint8_t led, uint8_t bright)
{
  uint8_t n = 0, r;

  if (idle_s < 5)  idle_s = 5;
  if (rounds < 1)  rounds = 1;
  if (rounds > 4)  rounds = 4;

  /* 睡前夜里 */
  snprintf(seq_name_pool[n], sizeof(seq_name_pool[n]), "day_night1");
  SEQ_DAY[n].name = seq_name_pool[n];
  SEQ_DAY[n].dwell_s = idle_s; SEQ_DAY[n].sleep_s = idle_s;
  SEQ_DAY[n].kind = SEQK_SLEEP; SEQ_DAY[n].led = 0xFF; SEQ_DAY[n].bright = 0;
  n++;

  /* 早高峰窗口 */
  for (r = 0; r < rounds; r++)
  {
    snprintf(seq_name_pool[n], sizeof(seq_name_pool[n]), "day_am%u", (unsigned)(r + 1u));
    SEQ_DAY[n].name = seq_name_pool[n];
    SEQ_DAY[n].dwell_s = (uint16_t)SEQ_WIN_ROUND_S; SEQ_DAY[n].sleep_s = 60;
    SEQ_DAY[n].kind = SEQK_LP_ROUND; SEQ_DAY[n].led = led; SEQ_DAY[n].bright = bright;
    n++;
  }

  /* 日间 */
  snprintf(seq_name_pool[n], sizeof(seq_name_pool[n]), "day_midday");
  SEQ_DAY[n].name = seq_name_pool[n];
  SEQ_DAY[n].dwell_s = idle_s; SEQ_DAY[n].sleep_s = idle_s;
  SEQ_DAY[n].kind = SEQK_SLEEP; SEQ_DAY[n].led = 0xFF; SEQ_DAY[n].bright = 0;
  n++;

  /* 晚高峰窗口 */
  for (r = 0; r < rounds; r++)
  {
    snprintf(seq_name_pool[n], sizeof(seq_name_pool[n]), "day_pm%u", (unsigned)(r + 1u));
    SEQ_DAY[n].name = seq_name_pool[n];
    SEQ_DAY[n].dwell_s = (uint16_t)SEQ_WIN_ROUND_S; SEQ_DAY[n].sleep_s = 60;
    SEQ_DAY[n].kind = SEQK_LP_ROUND; SEQ_DAY[n].led = led; SEQ_DAY[n].bright = bright;
    n++;
  }

  /* 收尾夜 */
  snprintf(seq_name_pool[n], sizeof(seq_name_pool[n]), "day_night2");
  SEQ_DAY[n].name = seq_name_pool[n];
  SEQ_DAY[n].dwell_s = idle_s; SEQ_DAY[n].sleep_s = idle_s;
  SEQ_DAY[n].kind = SEQK_SLEEP; SEQ_DAY[n].led = 0xFF; SEQ_DAY[n].bright = 0;
  n++;

  s_seq_tab = SEQ_DAY;
  s_seq_n   = n;
}

/* 估算序列总时长（秒）：SLEEP/LP_ROUND 用标称，其余用 dwell */
static uint32_t SEQ_Estimate(void)
{
  uint32_t t = 0;
  uint8_t  k;
  for (k = 0; k < s_seq_n; k++) t += s_seq_tab[k].dwell_s;
  return t;
}

/* 处理 PC '!' 命令行（不含前导 '!' 与行尾 CR/LF） */
static void SEQ_HandleCmd(const char *line)
{
  printf("PC-CMD> !%s\r\n", line);

  if (strncmp(line, "help", 4) == 0)
  {
    printf("SEQ help:\r\n");
    printf("  !seq [dwell_s]                        13 档工况表，常规档默认 60s\r\n");
    printf("  !day [idle_s] [rounds] [R|Y|G] [bright]  压缩日\r\n");
    printf("  !stop                                 中止测试，回常在线\r\n");
    printf("  !wdg [status|hang|hang_irq|on|off|T]  软看门狗状态/故障注入/配置\r\n");
    printf("  !adc [n]                              读电池电压（PA1/R7R8 分压），n 次取最小\r\n");
    printf("  !online                               取消自动低功耗并常在线（置 BKP_DR3 禁用标志）\r\n");
    printf("  !lpon                                 清除 BKP_DR3，恢复上电自动低功耗(2h)\r\n");
    printf("  !batlp [on|off|status]                电池低压保护：<3.0V 进 Stop，>3.3V 回在线\r\n");
    printf("  !help                                 本帮助\r\n");
    printf("  例: !seq 60   !day 30 3 G 255   !adc\r\n");
    return;
  }

  /* 取消自动低功耗 → 常在线（同时持久禁用，掉电不丢；用 !lpon 恢复） */
  if (strncmp(line, "online", 6) == 0)
  {
    s_auto_lp = 0; s_auto_lp_done = 1;
    g_lp_cycle = 0; s_lp_state = 0; s_lp_done = 0;
    g_low_power = 0; HAL_ResumeTick(); DTU_SetLowPower(0); V4G_Set(1);
    AUTO_BKP_DR = AUTO_BKP_OFF_MAGIC;   /* BKP_DR3 打禁用标志 */
    printf("AUTO-LP: cancelled by '!online' -> stay online, auto-lp DISABLED "
           "(BKP_DR3=0x%04X; use !lpon to re-enable)\r\n", (unsigned)AUTO_BKP_DR);
    return;
  }

  /* 恢复上电自动低功耗：清 BKP_DR3 禁用标志 */
  if (strncmp(line, "lpon", 4) == 0)
  {
    AUTO_BKP_DR = 0;
    printf("AUTO-LP: re-enabled (BKP_DR3 cleared) -> next power-up auto-enters %us cycle\r\n",
           (unsigned)LP_DEFAULT_PERIOD_S);
    return;
  }

  /* 电池低压保护：!batlp [on|off|status]（off 持久禁用，写 BKP_DR4） */
  if (strncmp(line, "batlp", 5) == 0)
  {
    const char *p = line + 5;
    while (*p == ' ') p++;
    if (strncmp(p, "on", 2) == 0)
    {
      s_batlp_en = 1; BAT_BKP_DR = BAT_BKP_ON_MAGIC;
      printf("BAT-LP: ON (BKP_DR4 cleared) enter<%u mV resume>%u mV\r\n",
             (unsigned)BAT_LP_ENTER_MV, (unsigned)BAT_LP_RESUME_MV);
    }
    else if (strncmp(p, "off", 3) == 0)
    {
      s_batlp_en = 0; BAT_BKP_DR = BAT_BKP_OFF_MAGIC;
      s_bat_sleep = 0; g_low_power = 0; HAL_ResumeTick();
      DTU_SetLowPower(0); V4G_Set(1);
      printf("BAT-LP: OFF (BKP_DR4=0x%04X) -> never sleep on low battery\r\n",
             (unsigned)BAT_BKP_DR);
    }
    else
    {
      printf("BAT-LP: %s  VBAT_last=%u mV  sleeping=%u  enter<%u resume>%u poll=%us\r\n",
             s_batlp_en ? "ON" : "OFF", (unsigned)s_bat_mv_last,
             (unsigned)s_bat_sleep, (unsigned)BAT_LP_ENTER_MV,
             (unsigned)BAT_LP_RESUME_MV, (unsigned)BAT_LP_POLL_S);
    }
    return;
  }

  if (strncmp(line, "adc", 3) == 0)
  {
    /* 读电池电压。带参数 N = 连续采 N 次打印每次数值（看 4G 脉冲造成的下坠）。
       不加参数 = 单次。模拟 MQTT 的 {"cmd":"adc"} 走的是另一条分支。 */
    const char *p = line + 3;
    while (*p == ' ') p++;
    int n = (*p >= '0' && *p <= '9') ? atoi(p) : 1;
    if (n < 1) n = 1;
    if (n > 100) n = 100;
    int k;
    for (k = 0; k < n; k++)
    {
      uint16_t raw = bat_adc_sample();
      if (raw == 0xFFFFu) { printf("ADC: sample FAILED\r\n"); break; }
      printf("ADC[%d/%d]: raw=%u  PA1=%u mV  VBAT=%u mV (%u.%03u V)\r\n",
             k + 1, n, (unsigned)raw, (unsigned)g_bat_adc_mv, (unsigned)g_bat_mv,
             (unsigned)(g_bat_mv / 1000u), (unsigned)(g_bat_mv % 1000u));
      if (k + 1 < n) HAL_Delay(200);
    }
    return;
  }

  if (strncmp(line, "stop", 4) == 0)
  {
    s_seq_run = 0; s_seq_stop = 0; s_seq_wait = 0; s_seq_round = 0;
    g_low_power = 0; DTU_SetLowPower(0);
    V4G_Set(1);
    LED_AllOff();
    printf("SEQ: aborted -> stay online\r\n");
    return;
  }

  if (strncmp(line, "wdg", 3) == 0)
  {
    const char *p = line + 3;
    while (*p == ' ') p++;

    if (*p == '\0' || strncmp(p, "status", 6) == 0)
    {
      printf("WDG: %s T=%us kick=%u miss=%u | rst CSR=0x%08X wdg=%u\r\n",
             RTC_APP_WDG_On() ? "ON" : "OFF",
             (unsigned)RTC_APP_WDG_Period(),
             g_wdg_kick, g_wdg_miss, g_rst_csr, g_rst_wdg);
    }
    else if (strncmp(p, "hang", 4) == 0)
    {
      /* 故障注入①：主循环挂死（中断仍在）。心跳闹钟连续无人认领，
         预期 ≤2T=60s 内 BKP 打标记 + 复位，F-11 应看到电流回到上电指纹。 */
      printf("WDG: INJECT hang (main loop) -> expect reset in <=%us\r\n",
             (unsigned)(RTC_APP_WDG_Period() * 2u));
      HAL_Delay(50);
      for (;;) { }
    }
    else if (strncmp(p, "hang_irq", 8) == 0)
    {
      /* 故障注入②：关中断挂死。软看门狗无法自救（无硬件狗兜底）——
         预期永远不复位（电流恒定），验证已知边界。需断电恢复。 */
      printf("WDG: INJECT hang with irq OFF -> NO self-heal (known limit)\r\n");
      HAL_Delay(50);
      __disable_irq();
      for (;;) { }
    }
    else if (strncmp(p, "off", 3) == 0)
    {
      RTC_APP_WDG_Set(0, 0);
      printf("WDG: OFF (until next reset)\r\n");
    }
    else if (strncmp(p, "on", 2) == 0)
    {
      RTC_APP_WDG_Set(1, 0);
      printf("WDG: ON T=%us\r\n", (unsigned)RTC_APP_WDG_Period());
    }
    else if (*p >= '0' && *p <= '9')
    {
      uint32_t t = (uint32_t)atoi(p);
      if (t < 5u) t = 5u;
      if (t > 3600u) t = 3600u;
      RTC_APP_WDG_Set(1, t);
      printf("WDG: ON T=%us maxmiss=1\r\n", (unsigned)RTC_APP_WDG_Period());
    }
    else
    {
      printf("WDG usage: !wdg [status|hang|hang_irq|on|off|<T_seconds>]\r\n");
    }
    return;
  }

  if (strncmp(line, "seq", 3) == 0)
  {
    const char *p = line + 3;
    int dwell = 60;
    while (*p == ' ') p++;
    if (*p >= '0' && *p <= '9') dwell = atoi(p);
    if (dwell < 5) dwell = 5;
    if (dwell > 600) dwell = 600;

    SEQ_BuildSeq((uint16_t)dwell);
  }
  else if (strncmp(line, "day", 3) == 0)
  {
    const char *p = line + 3;
    int idle = 30, rounds = 3, bright = 255;
    uint8_t led = LED_COLOR_GREEN;
    int got;

    for (got = 0; got < 2; got++)
    {
      while (*p == ' ') p++;
      if (*p < '0' || *p > '9') break;
      if (got == 0) idle   = atoi(p);
      else          rounds = atoi(p);
      while (*p && *p != ' ') p++;
    }
    while (*p == ' ') p++;
    if (*p == 'R' || *p == 'r')      led = LED_COLOR_RED;
    else if (*p == 'Y' || *p == 'y') led = LED_COLOR_YELLOW;
    else if (*p == 'G' || *p == 'g') led = LED_COLOR_GREEN;
    if (*p) p++;
    while (*p == ' ') p++;
    if (*p >= '0' && *p <= '9') bright = atoi(p);
    if (bright < 0)   bright = 0;
    if (bright > 255) bright = 255;

    SEQ_BuildDay((uint16_t)idle, (uint8_t)rounds, led, (uint8_t)bright);
  }
  else
  {
    printf("SEQ: unknown cmd '%s' (try !help)\r\n", line);
    return;
  }

  /* 启动：先退出 lp_cycle，避免两套状态机打架 */
  g_lp_cycle = 0;
  s_lp_state = 0;
  g_slog.n     = 0;                     /* 清空标记日志 */
  g_slog.done  = 0;
  s_seq_run  = 1;
  printf("SEQ: start, %u steps, est %lu s\r\n", (unsigned)s_seq_n,
         (unsigned long)SEQ_Estimate());
  SEQ_Enter(0);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* ⚠ 2026-10-15 修复：MX_ADC1_Init() 之前一直被漏掉（CubeMX 生成后没接进 main）。
     后果：hadc1.Instance 恒为 NULL，首次 bat_adc_sample() 调 HAL_ADC_Start ->
     ADC_Enable() 去写 NULL->CR2（地址 0x24）=> IMPRECISERR 总线错误 => HardFault，
     整板死机（症状：g_test_go 永不被消费、_4g_state 停在残值 0x0E）。
     ADC1 时钟由 HAL_ADC_MspInit() 使能，所以这里调用即可。 */
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* ---- 最先起来：RTC + 软看门狗 -----------------------------------------
     顺序很重要：LSE/RTC 初始化（最多 5s 起振等待）后立刻武装看门狗，
     这样后面**所有**初始化代码（SC7 软 I2C、LED、DTU、4G 配置）卡死
     都能被自愈复位。旧版把 WDG_Start 放在 DTU_Init 之后，启动阶段卡死
     （实测症状：断电重上电后整板卡在 ~2.5mA 不上来）无人兜底。 */
  RTC_APP_Init();

  /* 复位原因：CSR 各 *RSTF 位保持到下次 RMVF 清除，开机读出即上次复位源 */
  g_rst_csr = RCC->CSR;
  g_rst_wdg = RTC_APP_WasWdgReset();
  printf("RTC: %s (cnt=%u)\r\n", RTC_APP_Ready() ? "ready" : "UNAVAILABLE",
         RTC_APP_GetEpoch());
  printf("RST: CSR=0x%08X%s%s%s%s%s\r\n", g_rst_csr,
         (g_rst_csr & RCC_CSR_IWDGRSTF) ? " IWDG" : "",
         (g_rst_csr & RCC_CSR_WWDGRSTF) ? " WWDG" : "",
         (g_rst_csr & RCC_CSR_SFTRSTF)  ? " SW"   : "",
         (g_rst_csr & RCC_CSR_LPWRRSTF) ? " LPWR" : "",
         g_rst_wdg ? "  <-- RTC-SOFT-WDG RESET (self-healed)" : "");

  RTC_APP_WDG_Start(30u);
  printf("WDG: %s T=%us maxmiss=1 (hang -> reset in <=%us)\r\n",
         RTC_APP_WDG_On() ? "ON" : "OFF (RTC unavailable)",
         (unsigned)RTC_APP_WDG_Period(),
         (unsigned)(RTC_APP_WDG_Period() * 2u));

  /* SC7A20H 姿态传感器（软 I2C: PB6=SCL, PB7=SDA）。
     硬件 I2C1 版(hi2c1) 与 MX_I2C1_Init 保留未用 —— 实测软 I2C 更省心：
     不占外设、引脚任意、失败也能逐拍观察。 */
  g_sc7_id = (uint8_t)SC7A20H_Driver_Init(SC7A20H_INTERFACE_I2C);
  printf("SC7A20H: id=0x%02X (%s)\r\n", g_sc7_id,
         g_sc7_id == 0x11 ? "OK" : "NOT FOUND -> check PB6/PB7 & pull-ups");

  /* 4G 模组电源必须最先拉起来：外部 100kΩ 下拉使复位后 V4G 默认断电，
     模组上电到注网要几秒，早通电才能早连上 MQTT。 */
  V4G_Init();

  /* Print banner */
  printf("\r\n=== STM32 4G LED Controller ===\r\n");
  printf("V4G(PA3) = %s  (4G module power)\r\n", V4G_Get() ? "ON" : "OFF");

  HAL_UART_Receive_IT(&UART_4G, &RX_BYTE_4G, 1);
  HAL_UART_Receive_IT(&UART_PC, &RX_BYTE_PC, 1);

  /* LED：车牌板 = TIM3 三路 PWM 调光；最小板 = PC13 GPIO 开关 */
  LED_Init();

  DTU_Init();

  /* ---- 上电自动低功耗（默认 2 h 周期）-----------------------------------
     生产形态：上电后自动进入 lp_cycle 循环（睡 2h → 醒 → 上报 → 再睡）。
     安全闸门（任一不满足 ⇒ 放弃自动进入，留常在线）：
       ① RTC 必须就绪（否则没有唤醒源，进 Stop = 变砖）；
       ② BKP_DR3 未打 0x4F46 禁用标志（用户/调试可用 "!lpon"/"!online" 管理）；
       ③ 硬件上不区分复位源（POR/SWD/WDG 一视同仁 —— WDG 自愈复位后也应回到
          低功耗，否则看门狗一咬就把设备留在常在线，失能）。
     本次先置 s_auto_lp=1，主循环在线 AUTO_LP_BOOT_MS(15s) 后自动切到收尾入睡；
     期间任何 PC 串口 "!online" 或 MQTT lp_cycle_off 都能取消。
     注意：BKP 时钟在 RTC_APP_Init 里已开（RTC 未就绪时 DR3 读不准，但那种
     情况本就拒绝自动进入，无碍）。 */
  {
    uint8_t rtc_ok  = RTC_APP_Ready();
    uint8_t bkp_off = (AUTO_BKP_DR == AUTO_BKP_OFF_MAGIC) ? 1u : 0u;
    if (!rtc_ok)
    {
      printf("AUTO-LP: skip (RTC unavailable -> cannot arm wake)\r\n");
    }
    else if (bkp_off)
    {
      printf("AUTO-LP: disabled by BKP_DR3 ('!online' set) -> stay online\r\n");
    }
    else
    {
      s_auto_lp = 1;
      s_lp_done = 0;
      s_lp_t0   = HAL_GetTick();   /* 在线窗口起点（也是会话计时基准） */
      printf("AUTO-LP: armed, will enter lp_cycle period=%us after %ums online "
             "(send '!online' to cancel)\r\n",
             (unsigned)g_lp_period_s, (unsigned)AUTO_LP_BOOT_MS);
    }
  }

  /* ---- 电池低压保护开关：默认关闭（调试期），BKP_DR4==0x4F4F 才开启（持久）---- */
  s_batlp_en = (BAT_BKP_DR == BAT_BKP_ON_MAGIC) ? 1u : 0u;
  s_bat_last_ms = HAL_GetTick();   /* 首次检测延后 BAT_LP_CHECK_MS，让 4G 先起来 */
  printf("BAT-LP: %s (enter<%u mV, resume>%u mV, poll %us)\r\n",
         s_batlp_en ? "ON" : "OFF (BKP_DR4)",
         (unsigned)BAT_LP_ENTER_MV, (unsigned)BAT_LP_RESUME_MV, (unsigned)BAT_LP_POLL_S);

  /* USER CODE END 2 */

  /* Infinite loop — SysTick wakes every 1ms, WFI sleeps otherwise */
  /* USER CODE BEGIN WHILE */
  uint32_t last_sc7 = HAL_GetTick();   /* 上次读姿态的时刻 */
  while (1)
  {
    /* 睡眠模式：挂起 SysTick，让 MCU 真正深度空闲，仅被 UART/RTC 等中断唤醒；
       非睡眠模式：恢复 SysTick 保证 1ms 时基与 HAL_Delay 正常。 */
    /* lp_cycle 睡眠态 = 真 Stop（SLEEPDEEP=1，仅 RTC 闹钟 EXTI17 能唤醒）；
       其余（含旧 sleep 模式）= Sleep（内核停、外设时钟在，UART RX 可唤醒，
       模组维持 MQTT —— 这个语义依赖 UART 唤醒，绝不能进 Stop）。 */
    uint8_t want_stop = (g_lp_cycle && s_lp_state == 3) || s_seq_stop || s_bat_sleep;
    /* ⚠ Stop 门禁（2026-10-14 加固）：没有已武装的唤醒源就绝不下睡 ——
       F1 进 Stop 后唯一唤醒通道是 RTC 闹钟（EXTI17），闹钟没装上就睡下去
       = 永久变砖。这里把"想睡"和"能睡"分开判，异常时退回常在线自愈。 */
    uint8_t lp_stop = want_stop && (RTC_APP_WakeArmed() != 0);
    if (want_stop && !lp_stop)
    {
      printf("STOP: REFUSED - no wake source armed -> abort to online\r\n");
      g_lp_cycle = 0; s_lp_state = 0;
      s_seq_stop = 0; s_seq_wait = 0; s_seq_run = 0;
      g_low_power = 0; HAL_ResumeTick(); V4G_Set(1);
    }
    if (g_low_power) HAL_SuspendTick(); else HAL_ResumeTick();
    if (lp_stop) SCB->SCR |=  SCB_SCR_SLEEPDEEP_Msk;
    else         SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    __WFI();
    uint32_t now = HAL_GetTick();

    /* RTC 闹钟唤醒（lp_cycle 睡眠态）：重开 4G 会话。
       注意：这一圈 HAL_SuspendTick 已把 SysTick 挂起，now 读数停在睡眠前，
       会话内不再依赖 now 做绝对计时，收尾前的 HAL_Delay 都不依赖 tick。 */
    if (g_rtc_alarm_hit)
    {
      g_rtc_alarm_hit = 0;
      /* ★ 2026-10-15：Stop 唤醒瞬间 APB1 与 RTC 未同步（RSF=0），此刻写 CRH/NVIC
         会被丢弃 ⇒ 下一发闹钟没中断 ⇒ 醒来又立刻睡回去 = 锁死。
         第一件事就是把闹钟中断门禁重开（内部等 RSF=1）。 */
      RTC_APP_RearmIRQ();
      if (s_seq_run && s_seq_wait)
      {
        /* 工况测试的 Stop 醒来：F1 退出 Stop 后 SYSCLK = HSI，
           必须重配时钟，否则 UART 波特率全错。 */
        g_low_power = 0;
        HAL_ResumeTick();
        SystemClock_Config();
        s_seq_stop = 0;
        s_seq_wait = 0;
        printf("SEQ: wake (rtc=%u) -> next step\r\n", RTC_APP_GetEpoch());
        SEQ_Advance();
      }
      else if (g_lp_cycle && s_lp_state == 3)
      {
        g_low_power = 0;          /* 恢复 SysTick（下一圈顶部会 Resume） */
        DTU_SetLowPower(0);
        HAL_ResumeTick();
        /* ⚠ F1 从 Stop 退出后 SYSCLK = HSI，必须重配时钟回 HSE 8M，
           否则 USART1/3 波特率全错、模组通信成乱码。 */
        SystemClock_Config();
        V4G_Set(1);               /* 4G 上电，走正常注网/MQTT 流程 */
        s_lp_t0 = HAL_GetTick();
        s_lp_state = 1;
        s_lp_reported = 0;
        if (s_lp_done < 0xFFFFu) s_lp_done++;      /* 本轮睡眠已完成（供循环次数判定） */
        printf("LP: RTC alarm -> session start (rtc=%u, %u/%u)\r\n",
               RTC_APP_GetEpoch(), (unsigned)s_lp_done, (unsigned)g_lp_cycles);
      }
      /* ⚠ 2026-10-13 修复（会话被瞬间掐断的根因）：
         s_lp_t0 是在上面 HAL_ResumeTick()+SystemClock_Config()+V4G_Set(1)
         之后才赋值的，已比本圈开头「__WFI() 之后立刻取」的 now 大几毫秒。
         SysTick 在 Stop 期间被挂起，那一圈的 now 还停在睡眠前的冻结值，
         于是 now - s_lp_t0 为负 → uint32 下溢成 0xFFFFFFxx → 会话首圈即被判
         t_in >= LP_SESSION_MS，直接跳收尾。实测会话只剩 ~13.7 s（本该 90 s），
         并连带导致 4G 来不及注网（发不出 report）、nettime 授时必然失败。
         这里刷新 now，供本圈剩余逻辑（会话计时 / DTU / ACK / SC7）使用。 */
      else if (s_bat_sleep)
      {
        /* 电池低压 Stop 醒来：F1 退出 Stop 后 SYSCLK = HSI，先重配时钟。
           4G 全程断电（省电），醒来只复测电池：> 恢复阈值就回常在线，
           否则继续睡（重挂闹钟）。 */
        HAL_ResumeTick();
        SystemClock_Config();
        uint16_t mv = bat_adc_sample();
        s_bat_mv_last = (mv == 0xFFFFu) ? 0 : mv;
        if (mv != 0xFFFFu && mv >= BAT_LP_RESUME_MV)
        {
          s_bat_sleep = 0;
          DTU_SetLowPower(0);
          g_low_power = 0;
          V4G_Set(1);              /* 4G 上电回常在线 */
          printf("BAT-LP: wake VBAT=%u mV >= %u -> RESUME online\r\n",
                 (unsigned)mv, (unsigned)BAT_LP_RESUME_MV);
        }
        else
        {
          /* 仍偏低：重挂闹钟继续睡（ArmIn 失败则放弃睡眠回在线，绝不变砖）。 */
          if (RTC_APP_ArmIn(BAT_LP_POLL_S) == 0)
          {
            printf("BAT-LP: still low VBAT=%u mV -> sleep again (+%us)\r\n",
                   (unsigned)mv, (unsigned)BAT_LP_POLL_S);
          }
          else
          {
            s_bat_sleep = 0;
            DTU_SetLowPower(0);
            g_low_power = 0;
            V4G_Set(1);
            printf("BAT-LP: re-arm failed -> abort sleep, stay online\r\n");
          }
        }
      }
      now = HAL_GetTick();
    }

    /* SWD 注入：脚本写入的命令原样转给 4G 模组（见 g_dbg_tx 注释） */
    if (g_dbg_tx_len)
    {
      uint8_t n = g_dbg_tx_len;
      g_dbg_tx_len = 0;
      HAL_UART_Transmit(&UART_4G, (uint8_t *)g_dbg_tx, n, 500);
    }

    /* SC7A20H 姿态：每 500ms 读一次并算倾角。
       g_sc7_id != 0x11 时整段跳过 -> 传感器不在板子上也不会拖慢主循环。
       lp_cycle 睡眠态（4G 已断）也跳过 —— 省电且醒来时刻读一次即可。 */
    uint8_t lp_sleeping = (g_lp_cycle && s_lp_state == 3);
    if (!lp_sleeping && g_sc7_id == 0x11 && now - last_sc7 >= 500)
    {
      int16_t ax, ay, az;
      last_sc7 = now;
      if (!SC7A20H_ReadXYZ(&ax, &ay, &az))
      {
        g_sc7_x = ax; g_sc7_y = ay; g_sc7_z = az;
        /* 重力矢量定姿态 -> 只能得 roll / pitch（yaw 要磁力计）。
           atan2 只用两轴比值，与量程/灵敏度无关，换量程不必改系数。
           0.1° 单位：180/pi*10 = 572.9578 */
        g_sc7_roll  = (int16_t)(atan2f((float)ay, (float)az) * 572.9578f);
        g_sc7_pitch = (int16_t)(atan2f(-(float)ax,
                             sqrtf((float)ay * (float)ay + (float)az * (float)az)) * 572.9578f);
        g_sc7_ok = 1;
        g_sc7_cnt++;
        if ((g_sc7_cnt & 1) == 0)   /* 每 2 次(≈1s) 打一行，别刷爆串口 */
          printf("SC7: X=%6d Y=%6d Z=%6d roll=%4d pitch=%4d (0.1deg)\r\n",
                 ax, ay, az, g_sc7_roll, g_sc7_pitch);
      }
      else
      {
        g_sc7_err++;
      }
    }

    if (!lp_sleeping)
    {
      DTU_Process(now);
      RTC_APP_WDG_Poll(now);   /* 软看门狗喂狗：清醒期 ≥1s 节流重装心跳闹钟 */

      /* ---- 电池低压保护：在线期每 BAT_LP_CHECK_MS 测一次 VBAT --------------
         低于进睡阈值 → 关 4G → 挂闹钟 → 置 s_bat_sleep，主循环顶部 WFI 即 Stop。
         lp_cycle 使能时本功能让位（lp_cycle 优先，避免两套睡眠状态机打架）。
         RTC 未就绪则拒绝下睡（绝不能没唤醒源就睡）。 */
      if (s_batlp_en && !g_lp_cycle && !s_bat_sleep)
      {
        uint32_t since = now - s_bat_last_ms;
        if (s_bat_last_ms == 0 || since >= BAT_LP_CHECK_MS)
        {
          s_bat_last_ms = now;
          uint16_t mv = bat_adc_sample();
          s_bat_mv_last = (mv == 0xFFFFu) ? 0 : mv;
          if (mv != 0xFFFFu && mv < BAT_LP_ENTER_MV)
          {
            if (!RTC_APP_Ready())
            {
              printf("BAT-LP: VBAT=%u mV low but RTC unavailable -> stay online\r\n",
                     (unsigned)mv);
            }
            else if (RTC_APP_ArmIn(BAT_LP_POLL_S) != 0)
            {
              printf("BAT-LP: alarm arm FAILED -> stay online\r\n");
            }
            else
            {
              printf("BAT-LP: VBAT=%u mV < %u -> enter Stop, poll every %us\r\n",
                     (unsigned)mv, (unsigned)BAT_LP_ENTER_MV, (unsigned)BAT_LP_POLL_S);
              V4G_Set(0);          /* 关 4G 省电 */
              json_len = 0; json_ready = 0;
              rbuf_rd = rbuf_wr;
              LED_AllOff();
              g_low_power = 1;
              DTU_SetLowPower(1);
              s_bat_sleep = 1;     /* 主循环顶部 WFI 即 Stop */
            }
          }
        }
      }
    }

    /* 延时 ACK：到点后把排队的 ACK 发给 4G 模组（尾带 \r\n，模组按行打包上报） */
    if (g_ack_pending && (int32_t)(now - g_ack_at) >= 0)
    {
      g_ack_pending = 0;
      HAL_UART_Transmit(&UART_4G, (uint8_t*)g_ack_buf, (uint16_t)strlen(g_ack_buf), 200);
    }

    /* ---- 工况测试引擎推进（bench test）---- */
    if (s_seq_run)
    {
      if (s_seq_round == 1)          /* LP_ROUND：4G 会话进行中 */
      {
        uint32_t t_in = HAL_GetTick() - s_seq_rt0;
        if (!s_seq_reported && t_in >= LP_REPORT_AT_MS)
        {
          char rep[128];
          int n = snprintf(rep, sizeof(rep),
                   "{\"cmd\":\"report\",\"wake\":\"seq\",\"roll\":%d,\"pitch\":%d,"
                   "\"rtc\":%lu}\r\n", g_sc7_roll, g_sc7_pitch,
                   (unsigned long)RTC_APP_GetEpoch());
          HAL_UART_Transmit(&UART_4G, (uint8_t*)rep, (uint16_t)n, 200);
          s_seq_reported = 1;
          printf("SEQ: report published\r\n");
        }
        if (t_in >= LP_SESSION_MS) s_seq_round = 2;
      }
      else if (s_seq_round == 2)     /* LP_ROUND：收尾 → Stop */
      {
        s_seq_round = 0;
        /* ⚠ 收尾失败（闹钟没装上）时必须放弃本轮 Stop，绝不能置 s_seq_stop */
        if (LP_CloseAndSleepEx(s_seq_sleep ? s_seq_sleep : 60u, 0))
        {
          s_seq_stop = 1;
          s_seq_wait = 1;
        }
        else
        {
          printf("SEQ: LP_ROUND close aborted -> stop sequence\r\n");
          s_seq_run = 0; s_seq_stop = 0; s_seq_wait = 0;
        }
      }
      else if (!s_seq_wait &&
               (HAL_GetTick() - s_seq_t0) >= (uint32_t)s_seq_dwell * 1000u)
      {
        SEQ_Advance();
      }
    }

    /* ---- PC '!' 测试命令行 ---- */
    if (pc_cmd_ready)
    {
      pc_cmd_ready = 0;
      SEQ_HandleCmd(pc_cmd);
    }

    /* ---- SWD 注入的命令行（无 PC 串口时的入口，脚本写 RAM 触发）---- */
    if (g_test_go)
    {
      g_test_go = 0;
      ((char *)g_test_cmd)[sizeof(g_test_cmd) - 1u] = '\0';
      SEQ_HandleCmd((const char *)g_test_cmd);
    }

    /* ---- 上电自动低功耗：在线窗口结束后启动 ---- */
    if (s_auto_lp && !s_auto_lp_done)
    {
      /* 在线满 AUTO_LP_BOOT_MS 就切到收尾入睡（state=2 → LP_CloseAndSleep）。
         不在这里赶 ACK/报告，收尾函数自己会 flush ACK 队列。 */
      if ((uint32_t)(HAL_GetTick() - s_lp_t0) >= AUTO_LP_BOOT_MS)
      {
        s_auto_lp_done = 1;
        g_lp_cycle   = 1;         /* 使能循环模式 */
        g_lp_cycles  = 0;         /* 无限循环（生产形态：一直 2h 一轮） */
        g_lp_period_s = LP_DEFAULT_PERIOD_S;
        s_lp_done    = 0;
        s_lp_state   = 2;         /* 主循环下一圈走收尾 → Stop */
        printf("AUTO-LP: enter lp_cycle now (period=%us, infinite)\r\n",
               (unsigned)g_lp_period_s);
      }
    }

    /* ---- lp_cycle 会话推进 ---- */
    if (g_lp_cycle && s_lp_state == 1)
    {
      /* 现读 HAL_GetTick()（会话期 g_low_power=0，SysTick 在跑）：
         不能用本圈开头的 now —— 唤醒那一圈 now 是睡眠前的冻结值，
         与刚赋值的 s_lp_t0 相减会下溢，详见上面 g_rtc_alarm_hit 处的注释。 */
      uint32_t t_in = HAL_GetTick() - s_lp_t0;
      /* 到点上报一次：姿态 + RTC 时间。模组 MQTT 透传模式自动发布到上行主题。 */
      if (!s_lp_reported && t_in >= LP_REPORT_AT_MS)
      {
        char rep[128];
        int n = snprintf(rep, sizeof(rep),
                 "{\"cmd\":\"report\",\"wake\":\"rtc\",\"roll\":%d,\"pitch\":%d,"
                 "\"sc7_ok\":%u,\"sc7_err\":%u,\"rtc\":%u,"
                 "\"csr\":%u,\"rst_wdg\":%u}\r\n",
                 g_sc7_roll, g_sc7_pitch, g_sc7_ok, g_sc7_err, RTC_APP_GetEpoch(),
                 g_rst_csr, g_rst_wdg);
        HAL_UART_Transmit(&UART_4G, (uint8_t*)rep, (uint16_t)n, 200);
        s_lp_reported = 1;
        printf("LP: report published (%d bytes)\r\n", n);
      }
      /* 会话到时（或网络没起来也算超时）→ 收尾 */
      if (t_in >= LP_SESSION_MS)
        s_lp_state = 2;
    }
    if (g_lp_cycle && s_lp_state == 2)
    {
      /* 循环次数用尽（已睡满 g_lp_cycles 轮）→ 不再入睡，退出循环留在常在线。
         这里 4G 还没关（关 4G 是收尾函数干的），透传模式仍通，通知能直接发出去。
         带 "status" 字段 = 走既有的 ACK 回显抑制路径，不会自触发新一轮解析。 */
      if (g_lp_cycles != 0 && s_lp_done >= g_lp_cycles)
      {
        char rep[112];
        int n = snprintf(rep, sizeof(rep),
                 "{\"cmd\":\"lp_cycle_done\",\"status\":\"ok\","
                 "\"cycles\":%u,\"rtc\":%u}\r\n",
                 (unsigned)s_lp_done, RTC_APP_GetEpoch());
        HAL_UART_Transmit(&UART_4G, (uint8_t*)rep, (uint16_t)n, 200);
        printf("LP: cycles done (%u/%u) -> lp_cycle OFF, stay online\r\n",
               (unsigned)s_lp_done, (unsigned)g_lp_cycles);
        g_lp_cycle = 0;
        s_lp_state = 0;
        g_low_power = 0;
        DTU_SetLowPower(0);
        HAL_ResumeTick();
      }
      else
        LP_CloseAndSleep();    /* 阻塞 ≈3.5s：授时 → 挂闹钟 → 关 4G → 置睡 */
    }

    if (dbg_trigger)
    {
      dbg_trigger = 0;
      HAL_ResumeTick();            /* 调试探针内部用 HAL_Delay，需恢复 SysTick */
      g_low_power = 0; DTU_SetLowPower(0);
      DTU_DebugProbe();
    }

    /* 4G UART → PC UART passthrough */
    while (rbuf_rd != rbuf_wr)
    {
      if (HAL_UART_Transmit(&UART_PC, (uint8_t*)&rbuf[rbuf_rd], 1, 1) != HAL_OK)
        break;
      rbuf_rd = (rbuf_rd + 1) % RBUF_SIZE;
    }

    /* Parse MQTT JSON command */
    if (json_ready)
    {
      json_ready = 0;
      printf("MQTT> %s\r\n", (char*)json_buf);
      DTU_MarkCmdReceived();
      /* 模组把 STM32 自己发出的 ACK 回显回来，被当成新指令再处理会导致
         echo 死循环(sleep 反复自睡自 ack、led 反复自 ack 刷屏)。
         含 "status" 的是我们自己的 ACK 回显，直接忽略。 */
      if (strstr((char*)json_buf, "\"status\"") == NULL)
      {
      /* Web App 格式 {"cmd":"led","color":"red|blue|green","state":"on|off"}
         该格式通过 MQTT 订阅主题(已含 ICCID)路由到本机，无需再校验 ICCID */
      int is_web_led = (strstr((char*)json_buf, "\"cmd\":\"led\"") != NULL) ||
                       (strstr((char*)json_buf, "\"cmd\": \"led\"") != NULL);
      /* 系统指令（sleep 等）与 Web 格式一样，靠订阅主题(已含 ICCID)路由到本机，
         不再要求 payload 内出现 ICCID —— 否则 {"cmd":"sleep"} 会被 match 校验直接丢弃 */
      int is_sys_cmd = (strstr((char*)json_buf, "\"cmd\":\"sleep\"") != NULL) ||
                       (strstr((char*)json_buf, "\"cmd\": \"sleep\"") != NULL) ||
                       (strstr((char*)json_buf, "\"cmd\":\"lp_cycle\"") != NULL) ||
                       (strstr((char*)json_buf, "\"cmd\": \"lp_cycle\"") != NULL) ||
                       (strstr((char*)json_buf, "\"cmd\":\"lp_cycle_off\"") != NULL) ||
                       (strstr((char*)json_buf, "\"cmd\":\"v4g\"") != NULL) ||
                       (strstr((char*)json_buf, "\"cmd\": \"v4g\"") != NULL) ||
                       (strstr((char*)json_buf, "\"cmd\":\"time\"") != NULL) ||
                       (strstr((char*)json_buf, "\"cmd\": \"time\"") != NULL) ||
                       (strstr((char*)json_buf, "\"cmd\":\"adc\"") != NULL) ||
                       (strstr((char*)json_buf, "\"cmd\": \"adc\"") != NULL) ||
                       (strstr((char*)json_buf, "\"cmd\":\"bat\"") != NULL) ||
                       (strstr((char*)json_buf, "\"cmd\": \"bat\"") != NULL);
      uint8_t match = 1;
      if (!is_web_led && !is_sys_cmd && my_iccid[0])
      {
        match = (strstr((char*)json_buf, my_iccid) != NULL);
      }
      if (match)
      {
        /* 任意被识别的指令都唤醒到活动模式（恢复心跳/探活、SysTick 已恢复） */
        g_low_power = 0;
        DTU_SetLowPower(0);
        /* 默认 NULL：仅当识别到具体指令时才回 ACK。
           防止模组自回显(如心跳 ready、探针 +++)被当成指令解析后，
           因默认值 "off" 而自动刷 {"cmd":"off","status":"ok"}。 */
        const char *ack_cmd = NULL;
        uint8_t ack_bright = 0;

        /* 亮度字段（Web 端 bright，0..255）。缺省给最大亮度，保持旧版"非暗即亮"行为 */
        int bright = json_int((const char*)json_buf, "\"bright\"", LED_BRIGHT_MAX, 0, LED_BRIGHT_MAX);

        /* 睡眠命令：关灯 + 进入 STM32 低功耗；4G 模组照常维持 MQTT 连接 */
        if (strstr((char*)json_buf, "\"cmd\":\"sleep\"") || strstr((char*)json_buf, "\"cmd\": \"sleep\""))
        {
          LED_AllOff();
          g_low_power = 1;
          DTU_SetLowPower(1);
          printf("LP: SLEEP (LED off, module keeps MQTT)\r\n");
          ack_cmd = "sleep";
        }
        /* 4G 模组电源开关：{"cmd":"v4g","state":"on|off"}；不带 state 则翻转（便于手工调试） */
        else if (strstr((char*)json_buf, "\"cmd\":\"v4g\"") || strstr((char*)json_buf, "\"cmd\": \"v4g\""))
        {
          int on;
          if (strstr((char*)json_buf, "\"state\":\"off\"") || strstr((char*)json_buf, "\"state\": \"off\""))
            on = 0;
          else if (strstr((char*)json_buf, "\"state\":\"on\"") || strstr((char*)json_buf, "\"state\": \"on\""))
            on = 1;
          else
            on = !V4G_Get();
          V4G_Set((uint8_t)on);
          printf("V4G(PA3): %s\r\n", on ? "ON  -> 4G module powered" : "OFF -> 4G module cut");
          ack_cmd = "v4g";
        }
        /* 定时唤醒循环：{"cmd":"lp_cycle","period_min":N,"cycles":M}
           （period N=1..1440，缺省 30；cycles M=0..65535，**0=无限循环**，缺省 0）。
           立即回 ACK 后收尾当前会话：授时→挂闹钟→关 4G→Stop；
           之后每 N 分钟 RTC 唤醒一轮（上报+授时+再睡）；睡满 M 轮后自动退出回常在线。 */
        else if (strstr((char*)json_buf, "\"cmd\":\"lp_cycle\"") || strstr((char*)json_buf, "\"cmd\": \"lp_cycle\""))
        {
          int per = json_int((const char*)json_buf, "\"period_min\"",
                             (int)(g_lp_period_s / 60u), 1, 1440);
          int cyc = json_int((const char*)json_buf, "\"cycles\"", 0, 0, 65535);
          g_lp_period_s = (uint32_t)per * 60u;
          g_lp_cycles = (uint16_t)cyc;
          s_lp_done = 0;     /* 重新计数：本次下发重新开始算轮数 */
          g_lp_cycle = 1;
          s_lp_state = 2;    /* 主循环下一圈走收尾（先让 ACK 发出去） */
          printf("LP: lp_cycle ON, period=%u min, cycles=%u (%s)\r\n",
                 per, (unsigned)g_lp_cycles,
                 g_lp_cycles ? "finite" : "infinite");
          ack_cmd = "lp_cycle";
        }
        /* 退出循环模式，回到常在线：{"cmd":"lp_cycle_off"} */
        else if (strstr((char*)json_buf, "\"cmd\":\"lp_cycle_off\""))
        {
          g_lp_cycle = 0;
          s_lp_state = 0;
          s_lp_done = 0;
          g_low_power = 0;
          DTU_SetLowPower(0);
          printf("LP: lp_cycle OFF -> stay online\r\n");
          ack_cmd = "lp_cycle_off";
        }
        /* 服务器授时（RTC 对表后备通道）：{"cmd":"time","epoch":<unix秒>} */
        else if (strstr((char*)json_buf, "\"cmd\":\"time\"") || strstr((char*)json_buf, "\"cmd\": \"time\""))
        {
          int ep = json_int((const char*)json_buf, "\"epoch\"", 0, 0, 2147483647);
          if (ep > 1600000000 && RTC_APP_SetEpoch((uint32_t)ep) == 0)
            printf("LP: RTC set from server, epoch=%d\r\n", ep);
          else
            printf("LP: RTC set FAILED (bad epoch or RTC unavailable)\r\n");
          ack_cmd = "time";
        }
        /* 查询电池电压：{"cmd":"adc"} 或 {"cmd":"bat"}
           返回 {"cmd":"adc","status":"ok","mv":<电池mV>,"adc_mv":<PA1 mV>,"raw":<0..4095>}
           车牌板：PA1(ADC1_IN1) 经 R7 100k / R8 220k 分压测 VBAT，PA2(BAT_EN) 控制通断 */
        else if (strstr((char*)json_buf, "\"cmd\":\"adc\"") || strstr((char*)json_buf, "\"cmd\": \"adc\"") ||
                 strstr((char*)json_buf, "\"cmd\":\"bat\"") || strstr((char*)json_buf, "\"cmd\": \"bat\""))
        {
          uint16_t raw = bat_adc_sample();
          if (raw == 0xFFFFu)
          {
            printf("ADC: sample FAILED (HAL error)\r\n");
            ack_cmd = "adc_err";        /* 走下面通用分支，回 {"cmd":"adc_err",...} */
          }
          else
          {
            printf("ADC: raw=%u, PA1=%u mV, VBAT=%u mV, VDDA=%u mV (vrefint_raw=%u)\r\n",
                   (unsigned)raw, (unsigned)g_bat_adc_mv, (unsigned)g_bat_mv,
                   (unsigned)g_vdda_mv, (unsigned)g_vrefint_raw);
            /* 本指令字段多，通用分支的固定格式装不下 ⇒ 就地组包入队，
               并把 ack_cmd 保持 NULL，免得下面通用分支再用旧格式覆盖一次 */
            snprintf(g_ack_buf, sizeof(g_ack_buf),
                     "{\"cmd\":\"adc\",\"status\":\"ok\",\"mv\":%u,\"adc_mv\":%u,\"raw\":%u,\"vdda\":%u}\r\n",
                     (unsigned)g_bat_mv, (unsigned)g_bat_adc_mv, (unsigned)raw,
                     (unsigned)g_vdda_mv);
            g_ack_at = now + ACK_DELAY_MS;
            g_ack_pending = 1;
          }
        }
      #ifdef LICENSE_PLATE_BOARD
        else if (is_web_led)
        {
          /* Web App 格式：{"cmd":"led","color":"red|yellow|green","state":"on|off","bright":0-255}
             （"blue" 作为黄色的历史别名继续兼容） */
          int led_on = (strstr((char*)json_buf, "\"state\":\"on\"") != NULL) ||
                       (strstr((char*)json_buf, "\"state\": \"on\"") != NULL);
          if (!led_on)
          {
            LED_AllOff();
            printf("LED: ALL OFF\r\n");
            ack_cmd = "off";
          }
          else
          {
            uint8_t color = LED_COLOR_RED;    /* 未指定颜色 → 默认红灯（保持旧行为） */
            if (strstr((char*)json_buf, "\"color\":\"yellow\"") || strstr((char*)json_buf, "\"color\": \"yellow\"") ||
                strstr((char*)json_buf, "\"color\":\"blue\"")   || strstr((char*)json_buf, "\"color\": \"blue\""))
              color = LED_COLOR_YELLOW;
            else if (strstr((char*)json_buf, "\"color\":\"green\"") || strstr((char*)json_buf, "\"color\": \"green\""))
              color = LED_COLOR_GREEN;

            LED_SetColor(color, (uint8_t)bright);
            ack_bright = (uint8_t)bright;
            printf("LED: %s ON @ bright=%d\r\n",
                   color == LED_COLOR_YELLOW ? "YELLOW" : (color == LED_COLOR_GREEN ? "GREEN" : "RED"), bright);
            ack_cmd = (color == LED_COLOR_YELLOW) ? "yellow" : (color == LED_COLOR_GREEN ? "green" : "red");
          }
        }
        else if (strstr((char*)json_buf, "\"cmd\":\"red\"") || strstr((char*)json_buf, "\"cmd\": \"red\""))
        {
          LED_SetColor(LED_COLOR_RED, (uint8_t)bright);
          ack_bright = (uint8_t)bright;
          printf("LED: RED ON @ bright=%d\r\n", bright);
          ack_cmd = "red";
        }
        else if (strstr((char*)json_buf, "\"cmd\":\"yellow\"") || strstr((char*)json_buf, "\"cmd\": \"yellow\"") ||
                 strstr((char*)json_buf, "\"cmd\":\"blue\"")   || strstr((char*)json_buf, "\"cmd\": \"blue\""))
        {
          LED_SetColor(LED_COLOR_YELLOW, (uint8_t)bright);
          ack_bright = (uint8_t)bright;
          printf("LED: YELLOW ON @ bright=%d\r\n", bright);
          ack_cmd = "yellow";
        }
        else if (strstr((char*)json_buf, "\"cmd\":\"green\"") || strstr((char*)json_buf, "\"cmd\": \"green\""))
        {
          LED_SetColor(LED_COLOR_GREEN, (uint8_t)bright);
          ack_bright = (uint8_t)bright;
          printf("LED: GREEN ON @ bright=%d\r\n", bright);
          ack_cmd = "green";
        }
        else if (strstr((char*)json_buf, "\"cmd\":\"off\"") || strstr((char*)json_buf, "\"cmd\": \"off\""))
        {
          LED_AllOff();
          printf("LED: ALL OFF\r\n");
          ack_cmd = "off";
        }
      #else
        if (strstr((char*)json_buf, "\"cmd\":\"pc13\"") || strstr((char*)json_buf, "\"cmd\": \"pc13\""))
        {
          LED_SetColor(LED_COLOR_GREEN, (uint8_t)bright);
          printf("PC13: ON (green)\r\n");
          ack_cmd = "on";
        }
        else if (strstr((char*)json_buf, "\"cmd\":\"pc13off\"") || strstr((char*)json_buf, "\"cmd\": \"pc13off\""))
        {
          LED_AllOff();
          printf("PC13: OFF\r\n");
          ack_cmd = "off";
        }
        else if (strstr((char*)json_buf, "\"cmd\":\"green\"") || strstr((char*)json_buf, "\"cmd\": \"green\""))
        {
          LED_SetColor(LED_COLOR_GREEN, (uint8_t)bright);
          printf("LED: GREEN (PC13 ON)\r\n");
          ack_cmd = "on";
        }
        else if (strstr((char*)json_buf, "\"cmd\":\"off\"") || strstr((char*)json_buf, "\"cmd\": \"off\""))
        {
          LED_AllOff();
          printf("LED: OFF\r\n");
          ack_cmd = "off";
        }
      #endif
        /* ACK: 仅当 ack_cmd 被具体指令赋值时才回（unknown 消息静默，避免自动刷 off）
           bright 一并回报，方便 Web 端确认亮度真的下发生效。
           注意：不能立刻发（会落进模组下发后的丢包窗口被丢掉），改为排队延时发送，
           并补上行尾 \r\n，让模组按完整一行打包上报。 */
        if (ack_cmd != NULL)
        {
          snprintf(g_ack_buf, sizeof(g_ack_buf),
                   "{\"cmd\":\"%s\",\"status\":\"ok\",\"bright\":%d}\r\n",
                   ack_cmd, ack_bright);
          g_ack_at = now + ACK_DELAY_MS;
          g_ack_pending = 1;
        }
      }
      } /* end: skip echoed ACK (contains "status") */
      json_len = 0;
      memset(json_buf, 0, JSON_BUF_SIZE);
    }
    if (json_len > 0 && now - json_last_rx > 3000)
    {
      json_len = 0;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
#ifdef CPU_CLOCK_8MHZ
  /* L1 降频档：HSE 8MHz 直出，不用 PLL → PLL 自身功耗全部省掉 */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
#else
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
#endif
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
#ifdef CPU_CLOCK_8MHZ
  /* SYSCLK = HSE = 8MHz，全部不分频：
        HCLK = 8MHz  → Flash 0 等待周期（0WS 可撑到 24MHz）
        PCLK1 = 8MHz → TIM3CLK = 8MHz（APB1 不分频时无 ×2 倍频），
                       与 board_config.h 的 LED_PWM_PRESCALER 计算一致；
                       USART3 也在 PCLK1 上，8MHz / 115200 误差 +0.64%
        PCLK2 = 8MHz → USART1（4G）用 */
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /* SYSCLK 已切到 HSE，HSI 不再被任何外设引用 → 关掉省电。
     （HAL_RCC_OscConfig 只配置 OscillatorType 里声明的振荡器，
       上面只声明了 HSE，所以 HSI 仍是复位默认的 ON，需显式关。） */
  __HAL_RCC_HSI_DISABLE();
#else
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
#endif
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
#ifdef CPU_CLOCK_8MHZ
  /* PCLK2 = 8MHz，DIV2 → ADC 时钟 4MHz（合法区间 1~14MHz，留有余量） */
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
#else
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;   /* 72/6 = 12MHz */
#endif
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  /* ⚠ 采样时间必须够长（2026-10-15 实测定案）：ADC_BAT 节点源阻抗 =
     R7∥R8 = 100k∥220k ≈ 68.6 kΩ，采样电容充电 τ ≈ 0.55 µs。
     1.5 周期 @ 4 MHz ADCCLK 只有 0.375 µs ⇒ 电容充不满 ⇒ 读数系统性偏低
     ~30%（实测万用表 VBAT=3.96 V / 节点 2.72 V，固件只读出 1.86 V）。
     239.5 周期 = 63 µs ≈ 114τ，充分充电；单次仍为阻塞 ~65 µs，无影响。 */
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* PA2: Q1 gate control (AO3401A, low=battery ADC on, default off) */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

  /* PA9/PA10: USART1 for 4G module (redundant with MSP, kept for clarity) */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Unused pins → analog input for minimum power (dig. input disabled) */

  /* GPIOA: 已交给外设/输出的脚**不配 ANALOG**：
       PA3     = V4G 4G 模组电源（board_io.c: V4G_Init）
       PA6/PA7 = TIM3_CH1/CH2 LED 红/黄（board_io.c: LED_Init） */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4
                      | GPIO_PIN_5 | GPIO_PIN_8
                      | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* GPIOB: unused pins → analog input。
     PB0 = LED 绿灯(TIM3_CH3)，必须排除；
     旧版 LED 脚 PB12/13/14 已弃用，回收为 ANALOG 省电 */
  {
    uint16_t pb_all = GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4
                    | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9
                    | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  #ifdef LICENSE_PLATE_BOARD
    pb_all &= (uint16_t)~(LED_GRN_PIN);     /* PB0 = TIM3_CH3 */
  #endif
    if (pb_all)
    {
      GPIO_InitStruct.Pin = pb_all;
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
  }

  /* GPIOC: PC13 */
  {
    uint16_t pc13 = GPIO_PIN_13;
  #ifdef MINIMUM_BOARD
    /* PC13 is LED output for minimum board — skip ANALOG */
    pc13 = 0;
  #endif
    if (pc13) {
      GPIO_InitStruct.Pin = pc13;
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
  }

  /* GPIOD: PD0, PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* printf → PC UART */
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&UART_PC, (uint8_t*)ptr, len, 100);
  return len;
}

/* UART RX callback — direction per board_config.h */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == _4G_INST)
  {
    /* 4G→PC: buffer in ring buffer, drain to PC UART */
    uint16_t next = (rbuf_wr + 1) % RBUF_SIZE;
    if (next != rbuf_rd)
    {
      rbuf[rbuf_wr] = RX_BYTE_4G;
      rbuf_wr = next;
    }

    /* Also buffer for JSON parsing */
    if (json_len < JSON_BUF_SIZE)
      json_buf[json_len++] = RX_BYTE_4G;
    if (RX_BYTE_4G == '}')
      json_ready = 1;
    json_last_rx = HAL_GetTick();
    g_4g_last_rx_ms = HAL_GetTick();   /* 模组健康时间戳（dtu.c 硬恢复判据） */
    g_4g_rx_bytes++;

    DTU_RxByte(RX_BYTE_4G);
    HAL_UART_Receive_IT(&UART_4G, &RX_BYTE_4G, 1);
  }
  else if (huart->Instance == _PC_INST)
  {
    /* PC→4G 透传；以下按键被本地拦下做调试，不转发给 4G 模组：
       '~' = 触发 DTU 调试探针
       'v' = 翻转 4G 模组电源 (PA3/V4G)
             —— 4G 一旦断电 MQTT 也断了，所以必须留一个**独立于 4G 链路**的验证入口
       'p' = 打印 PA3 电平与三路 LED 当前亮度
       '!' = 进入本地命令行（行尾 CR/LF 提交），内容不转发给 4G 模组。
             命令见 SEQ_HandleCmd：!seq / !day / !stop / !help */
    if (pc_cmd_mode)
    {
      if (RX_BYTE_PC == '\r' || RX_BYTE_PC == '\n')
      {
        if (pc_cmd_len) { pc_cmd[pc_cmd_len] = '\0'; pc_cmd_ready = 1; }
        pc_cmd_mode = 0;
        pc_cmd_len  = 0;
      }
      else if (pc_cmd_len < (uint8_t)(sizeof(pc_cmd) - 1u))
      {
        pc_cmd[pc_cmd_len++] = (char)RX_BYTE_PC;
      }
    }
    else if (RX_BYTE_PC == '!')
    {
      pc_cmd_mode = 1;
      pc_cmd_len  = 0;
    }
    else if (RX_BYTE_PC == '~')
    {
      dbg_trigger = 1;
    }
    else if (RX_BYTE_PC == 'v' || RX_BYTE_PC == 'V')
    {
      uint8_t on = (uint8_t)(!V4G_Get());
      V4G_Set(on);
      printf("V4G(PA3) -> %s\r\n", on ? "ON  (4G powered)" : "OFF (4G cut)");
    }
    else if (RX_BYTE_PC == 'p' || RX_BYTE_PC == 'P')
    {
      printf("STAT: V4G(PA3)=%u  LED R=%u Y=%u G=%u\r\n", V4G_Get(),
             LED_GetBright(LED_COLOR_RED), LED_GetBright(LED_COLOR_YELLOW),
             LED_GetBright(LED_COLOR_GREEN));
    }
    else
    {
      HAL_UART_Transmit(&UART_4G, &RX_BYTE_PC, 1, 100);
    }
    HAL_UART_Receive_IT(&UART_PC, &RX_BYTE_PC, 1);
  }
}

/* SC7A20H init: enable 100Hz, all axes */
static void SC7A20H_Init(void)
{
  uint8_t id = 0;
  HAL_I2C_Mem_Read(&hi2c1, SC7A20H_ADDR, 0x0F, 1, &id, 1, 100);
  if (id == 0x11)
  {
    printf("SC7A20H detected (WHO_AM_I=0x%02X)\r\n", id);
  }
  else
  {
    printf("SC7A20H not found (id=0x%02X)\r\n", id);
    return;
  }
  uint8_t v;
  v = 0x47; HAL_I2C_Mem_Write(&hi2c1, SC7A20H_ADDR, SC7A20H_CTRL1, 1, &v, 1, 100);
  v = 0x80; HAL_I2C_Mem_Write(&hi2c1, SC7A20H_ADDR, 0x24, 1, &v, 1, 100);
  v = 0x00; HAL_I2C_Mem_Write(&hi2c1, SC7A20H_ADDR, 0x21, 1, &v, 1, 100);
  v = 0x00; HAL_I2C_Mem_Write(&hi2c1, SC7A20H_ADDR, SC7A20H_CTRL4, 1, &v, 1, 100);
  printf("SC7A20H configured\r\n");
}

/* SC7A20H read acceleration (16-bit signed) */
static void SC7A20H_ReadAccel(int16_t *x, int16_t *y, int16_t *z)
{
  uint8_t buf[6];
  HAL_I2C_Mem_Read(&hi2c1, SC7A20H_ADDR, SC7A20H_OUT_X | 0x80, 1, buf, 6, 100);
  *x = (int16_t)(buf[1] << 8 | buf[0]);
  *y = (int16_t)(buf[3] << 8 | buf[2]);
  *z = (int16_t)(buf[5] << 8 | buf[4]);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
