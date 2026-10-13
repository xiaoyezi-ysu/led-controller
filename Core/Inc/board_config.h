#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* 选择主板：取消注释对应行 */
#define LICENSE_PLATE_BOARD
// #define MINIMUM_BOARD

/* ---------- 系统主频档位（L1 降频省电）----------
   定义 CPU_CLOCK_8MHZ : HSE 8MHz 直出、关闭 PLL → SYSCLK = HCLK = 8MHz。
                         MCU 的 Sleep 电流从 ~22mA 降到 ~4mA（见
                         docs/low-power-1mA-plan.md §L1）。
   注释掉（默认）      : HSE 8MHz × PLL9 = 72MHz —— 原行为。
   回滚：只改这一行即可，SystemClock_Config 与 LED 分频会自动跟随。 */
#define CPU_CLOCK_8MHZ

/* ---------- 4G 模组驱动选择（二选一：取消注释对应行）----------
   MODULE_YINERDA : 银尔达 YED_DTU4 透传固件。命令集 config,set/get,mqtt；
                    ICCID 用 config,get,iccid 查询；连接建立后模组自动透传。
                    驱动文件 Core/Src/dtu.c + Core/Inc/dtu.h
   MODULE_FSC_AT  : 飞思创 AT 指令固件。命令集 AT+WKMOD1 / AT+MQTTSV1 /
                    AT+MQTTCONN1 / AT+MQTTPUB1 / AT+MQTTSUB1 / AT+ICCID? / AT+S / AT+Z；
                    连接建立后模组同样自动透传。
                    驱动文件 Core/Src/dtu_ctrl.c + Core/Inc/dtu_ctrl.h

   两个驱动对外 API 完全一致（DTU_Init/Process/RxByte/MarkCmdReceived/
   DebugProbe/SetLowPower/IsLowPower），main.c 用 #ifdef 条件包含即可共用。
   CMakeLists.txt 会自动读取本宏决定编译哪个 .c，无需手工改构建脚本。 */
#define MODULE_YINERDA
// #define MODULE_FSC_AT

#if !defined(MODULE_YINERDA) && !defined(MODULE_FSC_AT)
  #define MODULE_YINERDA          /* 未指定时默认银尔达 */
#endif
#if defined(MODULE_YINERDA) && defined(MODULE_FSC_AT)
  #error "MODULE_YINERDA 与 MODULE_FSC_AT 只能选择一个"
#endif

/* MQTT 主题前缀 */
#define MQTT_TOPIC_PREFIX  "xiaoyeziUESTC1147"

#ifdef LICENSE_PLATE_BOARD
  /* 车牌板：USART1→4G, USART3→PC（与最小板统一接线） */
  #define UART_4G      huart1
  #define UART_PC      huart3
  #define RX_BYTE_4G   uart1_rx_byte
  #define RX_BYTE_PC   uart3_rx_byte

  /* ---------- LED（2026-09-23 硬件改版：改用 TIM3 通道，支持 PWM 调光）----------
     红 = PA6 = TIM3_CH1，黄 = PA7 = TIM3_CH2，绿 = PB0 = TIM3_CH3
     三路同属 TIM3 → 一个定时器出三路独立占空比，可调光也可混色。
     驱动级为 AO3400A 低边 NMOS（栅极经 220Ω，100kΩ 下拉），LED 阳极接 VCC33
     → 高电平点亮，占空比越大越亮。

     旧版引脚 PB12/PB13/PB14 已弃用：它们在 F103 上是 TIM1_BKIN / TIM1_CH1N /
     TIM1_CH2N（刹车输入 + 互补输出），结构上无法输出普通 PWM。 */
  #define LED_RED_PORT    GPIOA
  #define LED_RED_PIN     GPIO_PIN_6      /* TIM3_CH1 */
  #define LED_YEL_PORT    GPIOA
  #define LED_YEL_PIN     GPIO_PIN_7      /* TIM3_CH2 */
  #define LED_GRN_PORT    GPIOB
  #define LED_GRN_PIN     GPIO_PIN_0      /* TIM3_CH3 */

  /* TIM3 时基：ARR=255 使 CCR 与 8 位亮度值 1:1 对应（0..255）。
     PSC 随主频选，保证 PWM 频率远高于人眼频闪阈值：
       72MHz（APB1=DIV2 → TIM3CLK=2×36M=72M）: PSC=255 → 72M/256/256 ≈ 1098 Hz
        8MHz（APB1=DIV1 → TIM3CLK=  8M     ）: PSC=  0 →  8M/256/256 = 31.25 kHz
     TIM3CLK 由 APB1 分频决定（见 SystemClock_Config）；DIV1 时无 ×2 倍频。 */
#ifdef CPU_CLOCK_8MHZ
  #define LED_PWM_TIMCLK_HZ    8000000UL
  #define LED_PWM_PRESCALER    0
#else
  #define LED_PWM_TIMCLK_HZ    72000000UL
  #define LED_PWM_PRESCALER    255
#endif
  #define LED_PWM_TIM          TIM3
  #define LED_PWM_PERIOD       255
  #define LED_BRIGHT_MAX       255

  #define HAS_PWM         1
  #define LED_ON          1               /* 高电平点亮 */
  #define BOARD_NAME      "车牌板"

  /* ---------- 4G 模组电源开关（2026-09-23 新增）----------
     PA3 → 1kΩ → Q5(AO3400A) 栅极（100kΩ 下拉到 GND）
         → 拉低 PMOS-Q2(AO3401A) 栅极 → V4G 得电
     高电平 = 上电。注意外部 R57 下拉，复位/高阻时 V4G 默认断电，
     因此固件必须尽早主动拉高，否则 4G 模组永远不上电。 */
  #define V4G_PORT        GPIOA
  #define V4G_PIN         GPIO_PIN_3
  #define V4G_ON_LEVEL    GPIO_PIN_SET
  #define V4G_OFF_LEVEL   GPIO_PIN_RESET
#endif

#ifdef MINIMUM_BOARD
  /* 最小板：USART1→4G, USART3→PC */
  #define UART_4G      huart1
  #define UART_PC      huart3
  #define RX_BYTE_4G   uart1_rx_byte
  #define RX_BYTE_PC   uart3_rx_byte
  /* LED: PC13 绿灯（低电平有效），无 PWM */
  #define LED_PORT     GPIOC
  #define LED_GRN_PIN  GPIO_PIN_13
  #define LED_ON       0
  #define LED_BRIGHT_MAX  255
  #define BOARD_NAME   "最小板"
#endif

/* UART Instance 宏，用于回调中区分 UART */
#define _4G_INST  (UART_4G.Instance)
#define _PC_INST  (UART_PC.Instance)

/* LED 开关电平值（仅 GPIO 开关式驱动需要，如最小板 PC13） */
#ifdef LED_PORT
  #define LED_GPIO_ON   (LED_ON ? GPIO_PIN_SET : GPIO_PIN_RESET)
  #define LED_GPIO_OFF  (LED_ON ? GPIO_PIN_RESET : GPIO_PIN_SET)
#endif

#endif /* BOARD_CONFIG_H */
