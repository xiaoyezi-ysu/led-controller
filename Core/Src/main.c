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
#include "dtu_ctrl.h"
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

  HAL_UART_Receive_IT(&UART_4G, &RX_BYTE_4G, 1);
  HAL_UART_Receive_IT(&UART_PC, &RX_BYTE_PC, 1);

  /* Init LED per board config */
  {
    GPIO_InitTypeDef led = {0};
    led.Pin = LED_ALL;
    led.Mode = GPIO_MODE_OUTPUT_PP;
    led.Pull = GPIO_NOPULL;
    led.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &led);
    HAL_GPIO_WritePin(LED_PORT, LED_ALL, LED_GPIO_OFF);
  }

  /* Print banner */
  printf("\r\n=== STM32 4G LED Controller ===\r\n");

  DTU_Init();

  /* USER CODE END 2 */

  /* Infinite loop — SysTick wakes every 1ms, WFI sleeps otherwise */
  /* USER CODE BEGIN WHILE */
  int16_t ax, ay, az;
  uint32_t last_tick = HAL_GetTick();
  while (1)
  {
    __WFI();
    uint32_t now = HAL_GetTick();

    /* SC7A20H every 500ms (disabled for 4G debug) */
    if (0 && now - last_tick >= 500)
    {
      SC7A20H_ReadAccel(&ax, &ay, &az);
      printf("X=%6d  Y=%6d  Z=%6d\r\n", ax, ay, az);
      last_tick = now;
    }

    DTU_Process(now);

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
      uint8_t match = 1;
      if (my_iccid[0])
      {
        match = (strstr((char*)json_buf, my_iccid) != NULL);
      }
      if (match)
      {
        const char *ack_cmd = "off";
      #ifdef LICENSE_PLATE_BOARD
        if (strstr((char*)json_buf, "\"cmd\":\"red\"") || strstr((char*)json_buf, "\"cmd\": \"red\""))
        {
          HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, LED_GPIO_ON);
          printf("LED: RED ON\r\n");
          ack_cmd = "red";
        }
        else if (strstr((char*)json_buf, "\"cmd\":\"blue\"") || strstr((char*)json_buf, "\"cmd\": \"blue\""))
        {
          HAL_GPIO_WritePin(LED_PORT, LED_YEL_PIN, LED_GPIO_ON);
          printf("LED: YELLOW ON\r\n");
          ack_cmd = "yellow";
        }
        else if (strstr((char*)json_buf, "\"cmd\":\"green\"") || strstr((char*)json_buf, "\"cmd\": \"green\""))
        {
          HAL_GPIO_WritePin(LED_PORT, LED_GRN_PIN, LED_GPIO_ON);
          printf("LED: GREEN ON\r\n");
          ack_cmd = "green";
        }
        else if (strstr((char*)json_buf, "\"cmd\":\"off\"") || strstr((char*)json_buf, "\"cmd\": \"off\""))
        {
          HAL_GPIO_WritePin(LED_PORT, LED_ALL, LED_GPIO_OFF);
          printf("LED: ALL OFF\r\n");
          ack_cmd = "off";
        }
      #else
        if (strstr((char*)json_buf, "\"cmd\":\"pc13\"") || strstr((char*)json_buf, "\"cmd\": \"pc13\""))
        {
          HAL_GPIO_WritePin(LED_PORT, LED_GRN_PIN, LED_GPIO_ON);
          printf("PC13: ON (green)\r\n");
          ack_cmd = "on";
        }
        else if (strstr((char*)json_buf, "\"cmd\":\"pc13off\"") || strstr((char*)json_buf, "\"cmd\": \"pc13off\""))
        {
          HAL_GPIO_WritePin(LED_PORT, LED_GRN_PIN, LED_GPIO_OFF);
          printf("PC13: OFF\r\n");
          ack_cmd = "off";
        }
        else if (strstr((char*)json_buf, "\"cmd\":\"green\"") || strstr((char*)json_buf, "\"cmd\": \"green\""))
        {
          HAL_GPIO_WritePin(LED_PORT, LED_GRN_PIN, LED_GPIO_ON);
          printf("LED: GREEN (PC13 ON)\r\n");
          ack_cmd = "on";
        }
        else if (strstr((char*)json_buf, "\"cmd\":\"off\"") || strstr((char*)json_buf, "\"cmd\": \"off\""))
        {
          HAL_GPIO_WritePin(LED_PORT, LED_GRN_PIN, LED_GPIO_OFF);
          printf("LED: OFF\r\n");
          ack_cmd = "off";
        }
      #endif
        /* ACK: transparent mode auto-publishes to broker */
        {
          char ack[64];
          int n = snprintf(ack, sizeof(ack), "{\"cmd\":\"%s\",\"status\":\"ok\"}", ack_cmd);
          if (n > 0) HAL_UART_Transmit(&UART_4G, (uint8_t*)ack, n, 200);
        }
      }
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

  /* GPIOA: PA0, PA1, PA3-PA8, PA11, PA12, PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4
                      | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8
                      | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* GPIOB: unused pins → analog input */
  {
    uint16_t pb_all = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4
                    | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9
                    | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  #ifdef LICENSE_PLATE_BOARD
    /* Exclude LED/PWM pins (configured in LED init or TIM1 MSP) */
    pb_all &= ~(LED_ALL);
  #endif
    GPIO_InitStruct.Pin = pb_all;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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

    HAL_UART_Receive_IT(&UART_4G, &RX_BYTE_4G, 1);
  }
  else if (huart->Instance == _PC_INST)
  {
    /* PC→4G: forward byte to 4G UART */
    HAL_UART_Transmit(&UART_4G, &RX_BYTE_PC, 1, 100);
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
