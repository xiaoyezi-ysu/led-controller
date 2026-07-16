#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* 选择主板：取消注释对应行 */
// #define LICENSE_PLATE_BOARD
#define MINIMUM_BOARD

/* MQTT 主题前缀 */
#define MQTT_TOPIC_PREFIX  "xiaoyeziUESTC1147"

#ifdef LICENSE_PLATE_BOARD
  /* 车牌板：USART1→4G, USART3→PC（与最小板统一接线） */
  #define UART_4G      huart1
  #define UART_PC      huart3
  #define RX_BYTE_4G   uart1_rx_byte
  #define RX_BYTE_PC   uart3_rx_byte
  /* LED: PB12=红, PB13=黄, PB14=绿 */
  #define LED_PORT     GPIOB
  #define LED_RED_PIN  GPIO_PIN_12
  #define LED_YEL_PIN  GPIO_PIN_13
  #define LED_GRN_PIN  GPIO_PIN_14
  #define LED_ALL      (LED_RED_PIN | LED_YEL_PIN | LED_GRN_PIN)
  #define LED_ON       1
  #define HAS_PWM      1
  #define BOARD_NAME   "车牌板"
#endif

#ifdef MINIMUM_BOARD
  /* 最小板：USART1→4G, USART3→PC */
  #define UART_4G      huart1
  #define UART_PC      huart3
  #define RX_BYTE_4G   uart1_rx_byte
  #define RX_BYTE_PC   uart3_rx_byte
  /* LED: PC13 绿灯（低电平有效） */
  #define LED_PORT     GPIOC
  #define LED_GRN_PIN  GPIO_PIN_13
  #define LED_ALL      GPIO_PIN_13
  #define LED_ON       0
  #define BOARD_NAME   "最小板"
#endif

/* UART Instance 宏，用于回调中区分 UART */
#define _4G_INST  (UART_4G.Instance)
#define _PC_INST  (UART_PC.Instance)

/* LED 开关电平值 */
#define LED_GPIO_ON   (LED_ON ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define LED_GPIO_OFF  (LED_ON ? GPIO_PIN_RESET : GPIO_PIN_SET)

#endif /* BOARD_CONFIG_H */
