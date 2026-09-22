#include "dtu.h"
#include <string.h>
#include <stdio.h>

/* UART_4G 由 board_config.h 提供（= huart1） */
extern UART_HandleTypeDef huart1;

_4G_State _4g_state = _4G_POWER_ON;
char my_iccid[32] = {0};
uint8_t iccid_loaded = 0;

/* main.c 中的 JSON 接收缓冲（进入透传前清零，避免配置期数据污染） */
extern uint8_t json_buf[256];          /* 须与 main.c 中 JSON_BUF_SIZE 一致 */
extern uint16_t json_len;
extern volatile uint8_t json_ready;

/* ---- 配置期应答行捕获 ---- */
#define DTU_LINE_MAX 128
static char dtu_line[DTU_LINE_MAX];
static uint8_t dtu_line_len = 0;
static volatile uint8_t dtu_line_ready = 0;

/* ---- 状态机计时与重试 ---- */
static uint32_t tick = 0;
static uint8_t id_tries = 0;
static uint8_t ssta_tries = 0;
static uint8_t ssta_retry = 0;   /* 探活重试计数：仅真离线才累加，避免误重启 */

/* ---- 心跳保活：周期性发布 ready，避免 broker 因空闲清掉会话/订阅 ---- */
#ifndef HB_PERIOD_MS
#define HB_PERIOD_MS  15000u   /* 每 15s 发一次心跳 */
#endif
static uint32_t hb_tick = 0;

/* ---- 透传链路探活/自愈：周期性退出透传查 ssta，失活则重启模组 ---- */
#ifndef PROBE_PERIOD_MS
#define PROBE_PERIOD_MS         60000u   /* 每 60s 探活一次 */
#endif
#ifndef RECOVER_IDLE_GUARD_MS
#define RECOVER_IDLE_GUARD_MS   3000u    /* 距上次收到指令不足 3s 则跳过本次探活 */
#endif
#ifndef ESC_GUARD_MS
#define ESC_GUARD_MS            1000u    /* 发 +++ 后等待 1s 再查询 */
#endif
#ifndef SSTA_TIMEOUT_MS
#define SSTA_TIMEOUT_MS         5000u    /* 查 ssta 无应答超时 */
#endif
static uint32_t probe_tick = 0;   /* 上次探活时刻 */
static uint32_t cmd_rx_tick = 0;  /* 上次收到指令时刻 */

/* 模组自身指示灯关闭模式（config,set,led,<mode>）：
 * 0=正常, 1=仅关信号指示灯(NET), 2=关全部指示灯(NET+RDY)。
 * 在 config,set,save 之前下发，随 MQTT 配置一并持久化，掉电/重启后保持熄灭。 */
#ifndef DTU_LED_OFF_MODE
#define DTU_LED_OFF_MODE  2
#endif

/* 模组 lp 下发值：1=开低功耗(保持连接)，0=关(满频在线)。
 * 初始化阶段始终下发本值并随 config,set,save 持久化进 Flash，
 * 因此改此宏即切换模组 lp 状态，重新编译即可做 lp,0 / lp,1 对照实验。 */
#ifndef DTU_LP_VALUE
#define DTU_LP_VALUE  0
#endif

/* 睡眠/低功耗标志：开启后停 STM32 心跳与探活，信任模组 keepalive 维持连接 */
static uint8_t low_power = 0;
void DTU_SetLowPower(uint8_t on)
{
  low_power = on ? 1 : 0;
  if (low_power) { hb_tick = 0; probe_tick = 0; }  /* 进入时清零计时，退出后重新开始 */
}
uint8_t DTU_IsLowPower(void) { return low_power; }

/* 模组 lp 指令实测回执：初始化时发送 config,set,lp,1 后回读 config,get,lp，
 * 结果（config,lp,ok,<val> 或 config,lp,error）经 MQTT 上报，便于在 PC 侧确认
 * 本模组(Air780EPM/YED_DTU4_V2.0.x)是否支持该私有低功耗指令。 */
static char lp_result[DTU_LINE_MAX] = "(unset)";  /* 与 dtu_line 等宽，避免回执被截断 */
static uint8_t lp_reported = 0;

/* 每次上电是否强制重新配置 DTU。
 * 银尔达配置掉电保存，生产固件可改为 0 并配合 flash 标志位只配置一次。 */
#define DTU_FORCE_RECONFIG  1

static void dtu_send(const char *s)
{
  HAL_UART_Transmit(&UART_4G, (uint8_t*)s, strlen(s), 200);
  HAL_UART_Transmit(&UART_4G, (uint8_t*)"\r\n", 2, 50);
}

/* 透传模式退出序列 "+++" 必须不带 CRLF，故单独提供裸发送 */
static void dtu_send_raw(const char *s)
{
  HAL_UART_Transmit(&UART_4G, (uint8_t*)s, strlen(s), 200);
}

static void dtu_clear_line(void)
{
  dtu_line_len = 0;
  dtu_line_ready = 0;
}

/* 由 UART RX 回调调用：仅在配置阶段捕获模块应答 */
void DTU_RxByte(uint8_t b)
{
  if (_4g_state >= _4G_READY) return;
  if (dtu_line_len < DTU_LINE_MAX - 1)
    dtu_line[dtu_line_len++] = (char)b;
  if (b == '\n' || b == '\r')
  {
    dtu_line[dtu_line_len] = '\0';
    dtu_line_ready = 1;
  }
}

/* 从 "config,<key>,ok,<value...>" 取出 value（截到行尾/回车） */
static uint8_t dtu_get_value(const char *key, char *out, int outsz)
{
  char prefix[24];
  int n = snprintf(prefix, sizeof(prefix), "config,%s,ok,", key);
  const char *p = strstr(dtu_line, prefix);
  if (!p) return 0;
  const char *v = p + n;
  int i = 0;
  while (*v && *v != '\r' && *v != '\n' && i < outsz - 1)
    out[i++] = *v++;
  out[i] = '\0';
  return (i > 0) ? 1 : 0;
}

void DTU_Init(void)
{
  dtu_clear_line();
  _4g_state = _4G_POWER_ON;
  my_iccid[0] = '\0';
  iccid_loaded = 0;
  printf("DTU: Yinerda YED_DTU4 driver init\r\n");
}

/* main.c 收到一条 MQTT 指令时调用，记录时刻以避免探活打断在途交互 */
void DTU_MarkCmdReceived(void)
{
  cmd_rx_tick = HAL_GetTick();
}

/* 调试探针：经 COM21 触发（'~'）。退出透传并上报模块真实的网络/MQTT 状态，
 * 用于排障“假在线”（ssta=4 但无 MQTT 收发）。 */
void DTU_DebugProbe(void)
{
  _4G_State saved = _4g_state;
  _4g_state = _4G_RECOVER_SSTA;   /* < READY => DTU_RxByte 会捕获模块应答 */
  printf("DBG: exit transparent (+++)\r\n");
  dtu_send_raw("+++");
  HAL_Delay(1200);

  printf("DBG: query netstatus,1\r\n");
  dtu_clear_line();
  dtu_send("config,get,netstatus,1");
  HAL_Delay(1500);
  printf("DBG netstatus: %s\r\n", dtu_line_ready ? dtu_line : "(no response)");
  dtu_line_ready = 0;

  printf("DBG: query ssta\r\n");
  dtu_clear_line();
  dtu_send("config,get,ssta");
  HAL_Delay(1500);
  printf("DBG ssta: %s\r\n", dtu_line_ready ? dtu_line : "(no response)");
  dtu_line_ready = 0;

  printf("DBG: query mqtt cfg,1\r\n");
  dtu_clear_line();
  dtu_send("config,get,mqtt,1");
  HAL_Delay(1500);
  printf("DBG mqtt: %s\r\n", dtu_line_ready ? dtu_line : "(no response)");
  dtu_line_ready = 0;

  printf("DBG: query netchaninfo,1\r\n");
  dtu_clear_line();
  dtu_send("config,get,netchaninfo,1");
  HAL_Delay(1500);
  printf("DBG netchaninfo: %s\r\n", dtu_line_ready ? dtu_line : "(no response)");
  dtu_line_ready = 0;

  /* 重新进入透传：DTU 透传固件用 AT+ENTM 回到数据透传模式（需网络仍在线）。
     不重新进入的话，模组会一直停在指令模式，UART 数据不再被发布到 MQTT。 */
  printf("DBG: re-enter transparent (AT+ENTM)\r\n");
  dtu_clear_line();
  dtu_send("AT+ENTM");
  HAL_Delay(1200);
  char rbuf[96];
  int n = snprintf(rbuf, sizeof(rbuf), "{\"cmd\":\"ready\",\"id\":\"%s\"}\r\n", my_iccid);
  if (n > 0) HAL_UART_Transmit(&UART_4G, (uint8_t*)rbuf, n, 200);
  _4g_state = saved;
  printf("DBG done\r\n");
}

void DTU_Process(uint32_t now)
{
  (void)DTU_FORCE_RECONFIG; /* 当前每次上电均重配（见文件头说明） */

  /* 进入 READY（透传）后：周期性心跳保活 + 链路探活自愈。
     睡眠模式下两者均跳过——4G 模组自身的 MQTT keepalive 已维持服务器连接，
     STM32 无需再发言，可深度空闲。 */
  if (_4g_state == _4G_READY)
  {
    /* 上线后一次性上报 lp 指令实测回执（不依赖 COM21 即可在 PC 侧确认模组是否支持） */
    if (!lp_reported)
    {
      char m[DTU_LINE_MAX + 48];
      int n = snprintf(m, sizeof(m), "{\"cmd\":\"lp\",\"status\":\"%s\"}\r\n", lp_result);
      if (n > 0) HAL_UART_Transmit(&UART_4G, (uint8_t*)m, (uint16_t)n, 200);
      lp_reported = 1;
    }
    if (!low_power)
    {
      if ((uint32_t)(now - hb_tick) >= HB_PERIOD_MS)
      {
        hb_tick = now;
        char hb[96];
        int n = snprintf(hb, sizeof(hb),
                         "{\"cmd\":\"ready\",\"id\":\"%s\"}\r\n", my_iccid);
        if (n > 0) HAL_UART_Transmit(&UART_4G, (uint8_t*)hb, (uint16_t)n, 200);
      }
      /* 探活：周期性退出透传查 ssta，确认链路是否仍在线（空闲时才探，避免打断指令） */
      if ((uint32_t)(now - probe_tick) >= PROBE_PERIOD_MS &&
          (uint32_t)(now - cmd_rx_tick) >= RECOVER_IDLE_GUARD_MS)
      {
        probe_tick = now;
        ssta_retry = 0;
        printf("DTU: probe -> escape transparent\r\n");
        dtu_clear_line();
        dtu_send_raw("+++");          /* 退出透传，进入指令模式（不加 CRLF） */
        tick = now;
        _4g_state = _4G_RECOVER_ESC;
      }
    }
    return;
  }

  /* 探活子状态：独立计时，不受下方通用 wait 影响 */
  if (_4g_state == _4G_RECOVER_ESC)
  {
    if ((uint32_t)(now - tick) < ESC_GUARD_MS) return;
    dtu_clear_line();
    dtu_send("config,get,ssta");     /* 此刻处于指令模式，可被模块解析 */
    tick = now;
    _4g_state = _4G_RECOVER_SSTA;
    return;
  }
  if (_4g_state == _4G_RECOVER_SSTA)
  {
    char val[8];
    if (dtu_line_ready)
    {
      dtu_line_ready = 0;
      if (dtu_get_value("ssta", val, sizeof(val)))
      {
        if (val[0] == '4')
        {
          /* 链路仍在线：重新进入透传（AT+ENTM）后再发 ready 保活。 */
          printf("DTU: ssta=4, re-enter transparent (AT+ENTM)\r\n");
          dtu_clear_line();
          dtu_send("AT+ENTM");
          HAL_Delay(1200);
          char rbuf[96];
          int n = snprintf(rbuf, sizeof(rbuf),
                           "{\"cmd\":\"ready\",\"id\":\"%s\"}\r\n", my_iccid);
          if (n > 0) HAL_UART_Transmit(&UART_4G, (uint8_t*)rbuf, n, 200);
          hb_tick = now;
          probe_tick = now;
          ssta_retry = 0;
          _4g_state = _4G_READY;
          return;
        }
        /* 收到 ssta 但非 4（如 2=有卡无网）：可能是瞬态，重试查询，不直接重启 */
        ssta_retry++;
        if (ssta_retry > 5)
        {
          printf("DTU: ssta=%s, reboot module\r\n", val);
          ssta_retry = 0;
          dtu_clear_line();
          dtu_send("config,set,save");
          _4g_state = _4G_WAIT_BOOT;
          return;
        }
        dtu_clear_line();
        dtu_send("config,get,ssta");
        tick = now;
        return;
      }
      /* 收到的不是 ssta 应答（如 +++ 退出透传的回显）：清空继续等，禁止误重启 */
      dtu_clear_line();
      return;
    }
    if ((uint32_t)(now - tick) >= SSTA_TIMEOUT_MS)
    {
      printf("DTU: ssta no reply, reboot module\r\n");
      ssta_retry = 0;
      dtu_clear_line();
      dtu_send("config,set,save");
      _4g_state = _4G_WAIT_BOOT;
    }
    return;
  }

  uint32_t wait = 2500;
  if      (_4g_state == _4G_POWER_ON)  wait = 3000;
  else if (_4g_state == _4G_WAIT_BOOT) wait = 3000;
  else if (_4g_state == _4G_POLL_SSTA) wait = 3000;
  if (now - tick < wait) return;
  tick = now;

  char buf[96];

  switch (_4g_state)
  {
    case _4G_POWER_ON:
      id_tries = 0;
      ssta_tries = 0;
      _4g_state = _4G_SEND_PARAMSRC;
      break;

    case _4G_SEND_PARAMSRC:
      printf("DTU> config,set,paramsrc,1\r\n");
      dtu_clear_line();
      dtu_send("config,set,paramsrc,1");
      _4g_state = _4G_QUERY_ICCID;
      break;

    case _4G_QUERY_ICCID:
      printf("DTU> config,get,iccid\r\n");
      dtu_clear_line();
      dtu_send("config,get,iccid");
      _4g_state = _4G_PARSE_ICCID;
      break;

    case _4G_PARSE_ICCID:
      if (dtu_line_ready)
      {
        dtu_line_ready = 0;
        if (dtu_get_value("iccid", my_iccid, sizeof(my_iccid)) && my_iccid[0])
        {
          iccid_loaded = 1;
          printf("DTU: ICCID=%s\r\n", my_iccid);
          _4g_state = _4G_SEND_MQTT;
          break;
        }
      }
      /* 未收到或解析失败 -> 查 IMEI（最多再等 2 个周期 ~5s） */
      id_tries++;
      if (id_tries <= 2)
      {
        printf("DTU> config,get,imei\r\n");
        dtu_clear_line();
        dtu_send("config,get,imei");
        _4g_state = _4G_PARSE_IMEI;
      }
      else
      {
        uint32_t uid = *(uint32_t*)0x1FFFF7E8;
        snprintf(my_iccid, sizeof(my_iccid), "uid_%08X", (unsigned int)uid);
        printf("DTU: use MCU UID=%s\r\n", my_iccid);
        _4g_state = _4G_SEND_MQTT;
      }
      break;

    case _4G_PARSE_IMEI:
      if (dtu_line_ready)
      {
        dtu_line_ready = 0;
        if (dtu_get_value("imei", my_iccid, sizeof(my_iccid)) && my_iccid[0])
        {
          iccid_loaded = 1;
          printf("DTU: IMEI=%s\r\n", my_iccid);
          _4g_state = _4G_SEND_MQTT;
          break;
        }
      }
      id_tries++;
      if (id_tries > 5)
      {
        uint32_t uid = *(uint32_t*)0x1FFFF7E8;
        snprintf(my_iccid, sizeof(my_iccid), "uid_%08X", (unsigned int)uid);
        printf("DTU: use MCU UID=%s\r\n", my_iccid);
        _4g_state = _4G_SEND_MQTT;
      }
      /* 否则保持 PARSE_IMEI，下个周期继续等待应答 */
      break;

    case _4G_SEND_MQTT:
    {
      char sub[64], pub[64], cmd[220];
      snprintf(sub, sizeof(sub), "%s/%s", MQTT_TOPIC_PREFIX, my_iccid);
      snprintf(pub, sizeof(pub), "DTU_Topic1147/%s", my_iccid);
      int off = snprintf(cmd, sizeof(cmd),
                         "config,set,mqtt,1,uart,120,broker.emqx.io,1883,%s", my_iccid);
      /* user、pwd 留空；proto=1(3.1.1)、clean=0(持久会话)、retain=0、subq=0、pubq=0
         字段对齐（已用 netchaninfo 回读实证）：
         clientid 之后必须是 user,pwd,proto,clean,retain,subq,pubq 共 7 个字段。
         因为 user、pwd 都为空，需用 ",," 占位，紧接 proto 的 "1" 前面还要再留一个逗号，
         即 ",1" 必须写成 ",,,1"（clientid + 三个逗号 + 1）：
           clientid , , , 1 , 0 , 0 , 0 , 0
                    user pwd proto clean retain subq pubq
         写成 ",,1,0,0,0,0"（只有两个逗号）会让 "1" 落到 pwd，导致主题整体错位、
         模组把订阅主题当成发布主题，下发指令永远到不了 MCU。 */
      off += snprintf(cmd + off, sizeof(cmd) - off, ",,,1,0,0,0,0");
      off += snprintf(cmd + off, sizeof(cmd) - off, ",%s,%s", sub, pub);
      /* will×5 + reg×2 + ipv6 + ssl = 9 个 0 */
      off += snprintf(cmd + off, sizeof(cmd) - off, ",0,0,0,0,0,0,0,0,0");
      printf("DTU> %s\r\n", cmd);
      dtu_clear_line();
      dtu_send(cmd);
      _4g_state = _4G_SEND_LED;
      break;
    }

    case _4G_SEND_LED:
      printf("DTU> config,set,led,%d (turn off module LED)\r\n", DTU_LED_OFF_MODE);
      dtu_clear_line();
      snprintf(buf, sizeof(buf), "config,set,led,%d", DTU_LED_OFF_MODE);
      dtu_send(buf);
      _4g_state = _4G_SEND_LP;
      break;

    case _4G_SEND_LP:
      printf("DTU> config,set,lp,%d (module low-power %s)\r\n", DTU_LP_VALUE,
             DTU_LP_VALUE ? "keep-alive" : "full-power");
      dtu_clear_line();
      snprintf(buf, sizeof(buf), "config,set,lp,%d", DTU_LP_VALUE);
      dtu_send(buf);
      HAL_Delay(600);
      /* 立即回读 lp 状态，确认该私有指令在本模组是否被支持（ok/error） */
      dtu_clear_line();
      dtu_send("config,get,lp");
      HAL_Delay(800);
      snprintf(lp_result, sizeof(lp_result), "%s",
               dtu_line_ready ? dtu_line : "(no-response)");
      /* 去掉回执里附带的 \r\n，避免 MQTT 上报的 JSON 被截断 */
      {
        char *e = lp_result + strlen(lp_result);
        while (e > lp_result && (e[-1] == '\r' || e[-1] == '\n')) *--e = '\0';
      }
      printf("DTU: lp query -> %s\r\n", lp_result);
      _4g_state = _4G_SEND_SAVE;
      break;

    case _4G_SEND_SAVE:
      printf("DTU> config,set,save (device will reboot)\r\n");
      dtu_send("config,set,save");
      _4g_state = _4G_WAIT_BOOT;
      break;

    case _4G_WAIT_BOOT:
      _4g_state = _4G_POLL_SSTA;
      break;

    case _4G_POLL_SSTA:
    {
      char val[8];
      if (dtu_line_ready)
      {
        dtu_line_ready = 0;
        if (dtu_get_value("ssta", val, sizeof(val)) && val[0] == '4')
        {
          /* 已连接服务器 -> 进入透传，清零 JSON 缓冲后上报 ready */
        json_len = 0;
        json_ready = 0;
        memset(json_buf, 0, sizeof(json_buf));
        /* 进入透传数据模式（DTU 透传固件用 AT+ENTM 回到透传；若已透传则忽略）。
           之后发送的业务数据必须带 \r\n 行尾，模组按行把串口数据作为一条 MQTT 消息发布。 */
        dtu_clear_line();
        dtu_send("AT+ENTM");
        HAL_Delay(800);
        int n = snprintf(buf, sizeof(buf), "{\"cmd\":\"ready\",\"id\":\"%s\"}\r\n", my_iccid);
        if (n > 0) HAL_UART_Transmit(&UART_4G, (uint8_t*)buf, n, 200);
        printf("DTU: READY (%s) +CRLF\r\n", buf);
        hb_tick = now;
        _4g_state = _4G_READY;
        break;
        }
      }
      ssta_tries++;
      if (ssta_tries > 20)   /* ~60s 仍未连上，强制进入透传，避免卡死 */
      {
        json_len = 0;
        json_ready = 0;
        memset(json_buf, 0, sizeof(json_buf));
        printf("DTU: force READY (ssta timeout)\r\n");
        hb_tick = now;
        _4g_state = _4G_READY;
        break;
      }
      /* 继续轮询连接状态 */
      dtu_clear_line();
      dtu_send("config,get,ssta");
      break;
    }

    default:
      _4g_state = _4G_READY;
      break;
  }
}
