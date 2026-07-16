# 调试记录

## 2026-07-13 4G 模块 + SC7A20H 调试总结

### 硬件连接（最终版）
| 外设 | STM32 引脚 | USART |
|------|-----------|-------|
| 4G 模组 | PA9(TX), PA10(RX) | USART1 |
| PC 调试串口(CH340) | PB10(TX), PB11(RX) | USART3 |

### 遇到的问题及解决

#### 1. USART 角色混淆
- **现象**: 4G 模组不回复 AT 命令
- **原因**: 最小系统板与车牌板的 USART 接线相反（车牌板 USART1→4G, USART3→PC；最小板反过来）
- **解决**: 确认实际接线后，修改代码使 USART1 用于 4G，USART3 用于 PC

#### 2. `+++` 退出数据模式
- **现象**: 4G 模组不响应 `+++`
- **原因**: 发送了 `+++\r\n`（带换行），飞思创 DTU 要求 `+++` 前后不能有任何字符，需 200ms 静默窗口
- **解决**: `HAL_UART_Transmit(&huart, "+++", 3, 100)` 只发 3 字节，不加 `\r\n`

#### 3. AT+ICCID 命令格式
- **现象**: AT+ICCID 返回空
- **原因**: 错用 `AT+ICCID`（不带问号），实际命令为 `AT+ICCID?`
- **解决**: 使用 `AT+ICCID?\r\n`

#### 4. MKLink 烧录后 MCU 不复位
- **现象**: 烧录后 MCU 停在 halted 状态，无输出
- **原因**: MKLink 的 `load.hex()` 不自动复位；`cmd.resume()` 不存在；AIRCR 写 0x05FA0004 需要逐字节写入（小端序 `0x04, 0x00, 0xFA, 0x05`）
- **解决**: 物理复位键最可靠

#### 5. STM32F103 寄存器差异
- **现象**: 读寄存器时 MKLink 超时
- **原因**: F103 使用 CRL/CRH（0x40010800/0x40010804）配置 GPIO，而非 F4 的 MODER/AFRH。读不存在的地址导致总线 fault
- **解决**: 使用正确的 F103 寄存器地址

#### 6. DMA 与 IDLE 中断
- **现象**: USART3 没有 DMA 信道配置
- **原因**: 原设计 USART1 使用 DMA1_Channel5 + IDLE 中断实现帧接收。USART3 未配置 DMA
- **解决**: 改用双向字节中断转发（`HAL_UART_Receive_IT` + `HAL_UART_RxCpltCallback`），省略 DMA/IDLE，代码更简洁

#### 7. `cmd.write_ram` 参数格式
- **现象**: 写 AIRCR 寄存器不生效
- **原因**: `cmd.write_ram(addr, value, width)` 格式不正确。正确格式为 `cmd.write_ram(addr, byte0, byte1, byte2, byte3)` 逐字节写入（小端序）
- **示例**: `cmd.write_ram(0xE000ED0C, 0x04, 0x00, 0xFA, 0x05)` 写 0x05FA0004

#### 8. USB 分线器供电不足
- **现象**: 4G 模组反复 disconnect/connect，`FS@MQTT CONNECTED:1` 时断时续
- **原因**: USB 分线器同时带两个 4G 模组时供电不足，模块掉电重启
- **解决**: 去掉一个 4G 模组（每个单独插一个 USB 口），供电正常后模块稳定连接

#### 9. 两个 4G 模块 MQTT 互相踢下线
- **现象**: 两个模块各自供电，但同时上电时反复 `FS@MQTT DISCONNECT:1` / `CONNECTED:1`
- **原因**: MQTT Client ID 都设为 `stm32_led`（硬编码），第二个连接会把第一个踢下线
- **解决**: 使用 STM32 唯一 ID（`0x1FFFF7E8`，96bit）生成不同 Client ID，如 `stm32_XXXXXX`
- **临时测试**: 一次只给一个 4G 模组通电即可

#### 10. swap 脚本导致回调变量不同步
- **现象**: USART 交换后 4G 模块有回复但 STM32 不处理，`\0` 或丢数据
- **原因**: swap 脚本只替换了 USART 实例名（`USART3`↔`USART1`），没替换回调函数内部的变量（`uart3_rx_byte`→`uart1_rx_byte`）和 UART 句柄（`huart3`→`huart1`），数据存到了错误的变量里
- **教训**: 不同板子之间不要写 swap 脚本，直接改 main.c 烧录对应板的固件，避免遗漏

### 关键命令速查
| 操作 | 命令 |
|------|------|
| +++ 退出数据模式 | `+++`（3字节，无`\r\n`） |
| AT 测试 | `AT\r\n` → `OK` |
| 查询 ICCID | `AT+ICCID?\r\n` → `+ICCID:...` |
| 烧录 | `load.flm(...)` → `erase_chip_flash` → `load.hex(...)` |
| 系统复位 | `cmd.write_ram(0xE000ED0C, 0x04, 0x00, 0xFA, 0x05)` |
