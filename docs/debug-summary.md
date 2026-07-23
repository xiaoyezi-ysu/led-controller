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
