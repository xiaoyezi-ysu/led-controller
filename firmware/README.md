# STM32F103 4G LED 控制器固件

## 文件说明

| 版本 | 文件 | 说明 |
|------|------|------|
| v1 | `license_plate_v1.hex` | 基础 MQTT 控制，无 ICCID 存储 |
| v2.1 | `license_plate_v21.hex` | ICCID 存储 + 唯一 Client ID + ENTM |
| v2.2 | `license_plate_v22.hex` | 修复UART映射反了、LED缺PB13/PB14 |
| v3-min | `minimum_board.hex` | 当前最新，PC13绿(低电平) |
| v3-plate | `license_plate.hex` | 当前最新，PB12/13/14 |

两板统一接线：USART1(PA9/PA10)→4G模块，USART3(PB10/PB11)→PC调试串口。

## 硬件连接

### 两板统一
| STM32 引脚 | 连接对象 |
|-----------|---------|
| PA9 (USART1_TX) | 4G 模块 RX |
| PA10 (USART1_RX) | 4G 模块 TX |
| PB10 (USART3_TX) | PC 串口 RX |
| PB11 (USART3_RX) | PC 串口 TX |
| PB6 (I2C1_SCL) | SC7A20H 加速度计 |
| PB7 (I2C1_SDA) | SC7A20H 加速度计 |

### 板型切换
`Core/Inc/board_config.h` 中切换：

```c
#define LICENSE_PLATE_BOARD   // PB12/13/14 高电平亮
// #define MINIMUM_BOARD       // PC13 低电平亮
```

## 功能

- MQTT 远程控制 LED（红/黄/绿/灭）
- 4G模块自动配置：+++ → MQTTSV1 → MQTTCONN1 → MQTTSUB1 → AT+S → 透传
- ICCID 闪存存储，首次查询后跳过
- 唯一 Client ID：`card_[ICCID]`
- cardID 匹配验证（只响应本机 ICCID 的命令）
- 环形缓冲区 256 字节防数据截断

## MQTT 配置

| 参数 | 值 |
|------|-----|
| Broker | `broker.emqx.io:1883` |
| 订阅主题 | `xiaoyeziUESTC1147` |
| Web App | https://xiaoyezi-ysu.github.io/led-controller/led_app.html |

### 命令格式

```json
{"cmd":"led","id":"89860812192380777549","led":"green","state":1}
```

- `id`: 目标 ICCID（仅匹配本机时处理）
- `led`: `red`, `yellow`, `green`
- `state`: 1(开) / 0(关)

### 回复

```json
{"cmd":"ready","id":"89860812192380777549"}
{"cmd":"led","led":"green","state":1,"status":"ok"}
```

## 烧录方式

### 最小板（MKLink/MicroLink）
```bash
arm-none-eabi-objcopy -O ihex build/stm32f103_CubemxCLI_demo0.elf f.hex
# 复制到 H:\f.hex（探针虚拟U盘，自动烧录）
```

### 车牌板（OpenOCD + ST-Link）
```bash
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "program build/stm32f103_CubemxCLI_demo0.elf verify reset exit"
```
