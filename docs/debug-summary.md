# Debug 记录

## AT+MQTTMTPC1=ON 导致 UART→MQTT 无应答

### 现象
加上 `AT+MQTTMTPC1=ON` 后，模块能连上 MQTT（`FS@MQTT CONNECTED:1`），
MQTT→UART 方向（LED 控制命令）正常接收，
但 UART→MQTT 方向（ready 自报、ACK 应答）Web App 收不到。

### 结论
`AT+MQTTMTPC1=ON` **不能设**。默认 OFF 即可正常工作。
原因待查，可能与模块固件版本有关。

### 当前配置（不需要设）
- `AT+WKMOD1=MQTT` — 设置工作模式为 MQTT（必需）
- `AT+MQTTMTPC1=ON` — ❌ 不设，保持默认 OFF

## 其他已修复问题

| 问题 | 原因 | 修复 |
|------|------|------|
| MQTTSV1/CONN1 报 ERR:4 | 未设 WKMOD1=MQTT | 增加 AT+WKMOD1=MQTT |
| ICCID 解析为空 | strstr 匹配到回显 AT+ICCID? 中的 +ICCID | 改为搜索 "+ICCID:" |
| json_buf 溢出截断 ICCID | 256字节被前面AT响应填满 | ICCID 查询前清空 json_buf |
| 模块不重启 | AT+S 仅保存不重启 | 加 AT+Z 强制重启 |
| ready 发太早 | FLUSH 等待12s不够 | 增至 20s |

## 只有绿灯有 ACK、其他指令无 ACK（2026-10-12 已修复）

### 现象
Web App 下发绿灯能收到 ACK，红灯/黄灯/关灯没有 ACK；设备上行只剩 `ready` 心跳。

### 排查结论
- 链路与 LED 都是好的：SWD 读 TIM3 CCR1/2/3、`bright_cur` 确认四种颜色都真的驱动了 PWM；`json_buf` 里能看到完整下行 JSON。
- 用 SWD 伪造 `json_buf` + `json_len` + `json_ready=1`（不经模组下行）触发 ACK → **能正常上报 MQTT**。
  → 固件 ACK 代码本身正确，唯一差异是「是否紧跟在模组下行之后发送」。
- **根因：银尔达 DTU 在向 MCU 下发服务器数据后的约 0.2~0.5s 内会丢弃 MCU 立刻回发的数据。**
  紧随下行发出的 ACK 永远到不了 MQTT；同样内容的字节晚一点再发就能正常上报。

### 修复（Core/Src/main.c）
ACK 不再当场发送，改为排队到 `ACK_DELAY_MS = 800ms` 后由主循环统一发出，
并补行尾 `\r\n`，让模组按完整一行打包上报。等待期内若又来新指令，直接覆盖旧 ACK
（拖动亮度滑块连发时只保留最后一条，避免刷屏）。

### 验证
MQTT 端到端 5/5 全部收到 ACK，延迟约 1s：

| 下发 | 上行 ACK |
|------|----------|
| `{"cmd":"led","color":"green","state":"on","bright":200}` | `{"cmd":"green","status":"ok","bright":200}` |
| `{"cmd":"led","color":"red","state":"on","bright":64}` | `{"cmd":"red","status":"ok","bright":64}` |
| `{"cmd":"led","color":"yellow","state":"on","bright":128}` | `{"cmd":"yellow","status":"ok","bright":128}` |
| `{"cmd":"led","color":"green","state":"off","bright":0}` | `{"cmd":"off","status":"ok","bright":0}` |
| `{"cmd":"led","color":"red","state":"off","bright":0}` | `{"cmd":"off","status":"ok","bright":0}` |

注意：关灯 ACK 的 `cmd` 是 `"off"`，不是颜色名（固件设计如此，Web 端解析时不要按颜色匹配）。

## 烧录通道备选：pyOCD 走 CMSIS-DAP（探针命令口卡死时可用）

MicroKeenV4 探针的 CDC 命令口（COM11 = MI_04）会卡死：三个 CDC 全静默，
文本/`\x02`/mux 二进制帧都无响应，但 USB 枚举正常、H: 盘（MICROKEEN）可读。
此时 **MKLink CLI / MCP / 脱机下载都不可用**，但可以用 pyOCD 直接走探针的
CMSIS-DAP 接口（MI_00）烧录目标，完全不需要命令口：

```bash
# 探针：CherryUSB MicroKeenV4 CMSIS-DAP，UID E0CB34CC85A4CDCD
pyocd flash -e chip -u E0CB34CC85A4CDCD -t stm32f103rc -f 1m <hex>
```

要点：
- pyOCD 只有内置 `stm32f103rc`，与 C8 的 RAM/页大小不同，但该 Flash 算法的 RAM
  占用（stack 0x20002800、buffer 0x20001000）都在 C8 的 20KB 内，可直接用。
- **必须加 `-e chip`**。默认 `-e auto` 的页擦除会静默失败，随后编程报 PGERR
  （`FLASH_SR=0x24`、`flash program page failure @0x08000000`）；
  `pyocd erase --mass` 则是正常的。
- CMake+ninja 只产出 ELF，需要自己转 HEX：
  `arm-none-eabi-objcopy -O ihex xxx.elf xxx.hex`。
- 软件「禁用/启用」USB 父设备**不能**复位探针，需物理插拔 USB。
  （2026-10-12 补充：后来发现用 COM12 上的 USB 串口继电器给探针**断电重启**即可恢复，
  `relay.cmd pulse --off-ms 1500` 等价于物理拔插，已封装为 WorkBuddy skill `usb-relay`。）

## 附 A：AT 测试是怎么做的（SWD 通道，非 RTT，未改动固件）

工程**没有集成 SEGGER RTT**（源码树里无任何 RTT/SEGGER 组件），当时测 AT 也**不是**通过 RTT 读的，
全程走 **SWD 内存/寄存器读写**（探针的 CMSIS-DAP 通道）：

| 操作 | 手段 |
|------|------|
| 看模组回包 | SWD 读 `rbuf` 环形缓冲（0x200001b4, 256B）+ 写指针 `rbuf_wr`/读指针 `rbuf_rd`（0x200002b4/0x200002b6） |
| 看固件状态 | SWD 读 `_4g_state`(0x2000045c)、`json_buf`(0x20000350) 等 |
| 向模组注入字节（含 AT） | SWD 写 `g_dbg_tx`(0x200002bc, 128B) + `g_dbg_tx_len`(0x2000033c)，主循环检测到非 0 就把内容发给 USART1 → 模组 |
| 让回包留在缓冲里不被排空 | SWD 临时清 USART3 CR1 的 TE 位（临时关掉 USART3 发送），测完恢复 |
| 独占模组测 AT | SWD 置 `low_power=1` 临时停掉 60s 探活（防止探活把模组切回指令模式），测完恢复 0 |

PC 串口（COM10 = 探针的 USB-UART 口 MI_02）当时**没有回显**，未用于读取 —— 所以才改走 SWD 通道。

**没有修改 main.c、没有烧录固件。** 当时全部是 RAM/寄存器级的易失修改，复位/断电即恢复；
当时会话记录明确写着「测试后固件已恢复 READY，链路在线（ssta=4）；未改动目标固件」。
你记忆中"之前可以正常收发"是对的 —— AT 测试从头到尾没动过 Flash 里的固件。
真正改 main.c 并烧录，是上文的 ACK 延时修复（唯一一次）。

**INJECTCHK-1/2/3 是调试会话里临时构造的注入测试标记**（全工程源码 grep 不到，不在固件里）：
通过 `g_dbg_tx` 注入发给 4G 模组，用来验证「MCU → 4G 模组 → MQTT 上行」通路是否畅通，
并借此测出了模组的**丢包窗口** —— 紧随模组下行之后约 0.2~0.5s 内注入的数据会被模组丢弃，
这也是后面 ACK 修复的关键线索。

## 附 B：响应延迟说明（为什么之前"迟钝"、现在灵敏）

- **LED 动作（亮灭/亮度）从来都是命令到达即执行**：下行 JSON 进 `json_buf` 后立刻写 TIM3 CCR，
  没有人为延时（SWD 读 CCR 验证过四种颜色都即时生效）。
- 之前感觉"迟钝/没反应"的真相：**ACK 被 DTU 丢包窗口吞掉** → Web 端收不到回执 →
  看起来像"没执行、没反应"。修复见上文「只有绿灯有 ACK」一节。
- 修复后的回执延迟是**设计内**的，构成如下：
  - `ACK_DELAY_MS = 800ms`（故意等过丢包窗口）
  - 模组串口分包超时 80ms（`config,uart,...,80,0`）
  - MQTT 上行 + broker 往返（broker.emqx.io 公共服务器，偶发抖动）
  - 合计约 1~1.5s 才能看到 ACK —— 正常。
- **调参**：`Core/Src/main.c` 的 `#define ACK_DELAY_MS 800u`。
  - 不建议低于 500ms：实测丢包窗口约 0.2~0.5s，太短会重新踩进窗口导致 ACK 丢失；
  - 觉得回执慢可微调到 600~700ms，但别低于 500ms。

