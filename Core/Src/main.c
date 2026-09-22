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
#include "dtu.h"
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

/* MQTT JSON receive buffer */
#define JSON_BUF_SIZE  256
uint8_t json_buf[JSON_BUF_SIZE];
uint16_t json_len = 0;
volatile uint8_t json_ready = 0;
volatile uint32_t json_last_rx = 0;

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
  /* USER CODE BEGIN 2 */

  /* SC7A20H disabled for debug */

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

  /* USER CODE END 2 */

  /* Infinite loop — SysTick wakes every 1ms, WFI sleeps otherwise */
  /* USER CODE BEGIN WHILE */
  int16_t ax, ay, az;
  uint32_t last_tick = HAL_GetTick();
  while (1)
  {
    /* 睡眠模式：挂起 SysTick，让 MCU 真正深度空闲，仅被 UART/RTC 等中断唤醒；
       非睡眠模式：恢复 SysTick 保证 1ms 时基与 HAL_Delay 正常。 */
    if (g_low_power) HAL_SuspendTick(); else HAL_ResumeTick();
    __WFI();
    uint32_t now = HAL_GetTick();

    /* SWD 注入：脚本写入的命令原样转给 4G 模组（见 g_dbg_tx 注释） */
    if (g_dbg_tx_len)
    {
      uint8_t n = g_dbg_tx_len;
      g_dbg_tx_len = 0;
      HAL_UART_Transmit(&UART_4G, (uint8_t *)g_dbg_tx, n, 500);
    }

    /* SC7A20H every 500ms (disabled for 4G debug) */
    if (0 && now - last_tick >= 500)
    {
      SC7A20H_ReadAccel(&ax, &ay, &az);
      printf("X=%6d  Y=%6d  Z=%6d\r\n", ax, ay, az);
      last_tick = now;
    }

    DTU_Process(now);

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
                       (strstr((char*)json_buf, "\"cmd\": \"sleep\"") != NULL);
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
           bright 一并回报，方便 Web 端确认亮度真的下发生效 */
        if (ack_cmd != NULL)
        {
          char ack[96];
          int n = snprintf(ack, sizeof(ack), "{\"cmd\":\"%s\",\"status\":\"ok\",\"bright\":%d}",
                           ack_cmd, ack_bright);
          if (n > 0) HAL_UART_Transmit(&UART_4G, (uint8_t*)ack, n, 200);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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

    DTU_RxByte(RX_BYTE_4G);
    HAL_UART_Receive_IT(&UART_4G, &RX_BYTE_4G, 1);
  }
  else if (huart->Instance == _PC_INST)
  {
    /* PC→4G 透传；以下按键被本地拦下做调试，不转发给 4G 模组：
       '~' = 触发 DTU 调试探针
       'v' = 翻转 4G 模组电源 (PA3/V4G)
             —— 4G 一旦断电 MQTT 也断了，所以必须留一个**独立于 4G 链路**的验证入口
       'p' = 打印 PA3 电平与三路 LED 当前亮度 */
    if (RX_BYTE_PC == '~')
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
