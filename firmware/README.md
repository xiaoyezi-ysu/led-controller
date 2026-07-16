# STM32F103 4G LED 控制器固件

## 文件说明

| 版本 | Git 提交 | 分支 | 说明 |
|------|---------|------|------|
| v1 | `1a554a3` | — | 基础 MQTT 控制，无 ICCID 存储 |
| v2.1 | `961c56d` | `license-plate-v2` | ICCID 存储 + 唯一 Client ID + ENTM 透传（推荐） |

> hex/elf 是编译产物，已被 `.gitignore` 忽略。需要烧录时从对应 commit 编译：
> ```bash
> git checkout <commit>
> cmake --build --preset Debug
> # 烧录 build/Debug/stm32f103_CubemxCLI_demo0.hex

## 硬件连接

### 车牌板（USART1→4G，USART3→PC）
| STM32 引脚 | 连接对象 |
|-----------|---------|
| PA9 (USART1_TX) | 4G 模块 RX |
| PA10 (USART1_RX) | 4G 模块 TX |
| PB10 (USART3_TX) | CH340 RX（PC 串口） |
| PB11 (USART3_RX) | CH340 TX（PC 串口） |
| PB12 | 红灯 |
| PB13 | 黄灯 |
| PB14 | 绿灯 |
| PC13 | 板载 LED（主动低） |
| PA9/PA10 | SWD（MKLink/ST-Link） |

### 最小板（USART1→PC，USART3→4G）
| STM32 引脚 | 连接对象 |
|-----------|---------|
| PA9 (USART1_TX) | CH340 RX（PC 串口） |
| PA10 (USART1_RX) | CH340 TX（PC 串口） |
| PB10 (USART3_TX) | 4G 模块 RX |
| PB11 (USART3_RX) | 4G 模块 TX |
| PC13 | 板载 LED |
| SWD | ST-Link |

## 功能

- MQTT 远程控制 LED（红/黄/绿/灭）
- 自动配置 4G 模块 MQTT 连接
- ICCID 自动识别并存入 Flash
- cardID 匹配验证（只响应本机 ICCID 的命令）
- 环形缓冲区防数据截断

## MQTT 配置

| 参数 | 值 |
|------|-----|
| Broker | `broker.emqx.io:1883` |
| 订阅/发布主题 | `MQTTX_Topic1147` |
| 命令格式 | `{"cardID":"<ICCID>","cmd":"red|green|blue|off|pc13|pc13off","bright":0-255}` |

## 烧录方式

### MKLink
```python
load.flm("/FLM/STM32F10x_1024.FLM",0x08000000,0x20000000)
erase_chip_flash(0x08000000)
load.hex("license_plate.hex")
```

### ST-Link (OpenOCD)
```bash
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "program license_plate.elf verify reset exit"
```

## PC 端脚本

位置：`scripts/LEDControl/`

| 文件 | 用途 |
|------|------|
| `mqtt_led.py` | 命令行控制（车牌板） |
| `mqtt_led_pc13.py` | 命令行控制 PC13（最小板） |
| `web_ui.py` | 浏览器控制网页 |
| `web_ui.html` | Web UI 页面模板 |

## 调试记录

详见 `docs/debug-summary.md`
