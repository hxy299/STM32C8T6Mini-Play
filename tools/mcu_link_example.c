/**
  ******************************************************************************
  * @file    tools/mcu_link_example.c
  * @brief   A7Z ↔ STM32 下位机通信 —— 可运行的最小 C 参考实现
  *
  * 这份代码的目的不是"功能齐全"，而是**把协议骨架跑通**：
  *   · 组帧 / CRC16 / 逐字节拆帧状态机
  *   · 串口打开 + raw 模式配置
  *   · 心跳（300 ms 一次 PING），与命令收发交织在同一个循环里
  *   · 发命令 + 等应答（带超时与重试）
  *   · 主动事件的接收与分发
  *
  * 设计上刻意做了三个选择，是为了方便移植到真实项目：
  *
  *   1) **单线程 + select**，不用 pthread。
  *      心跳、命令、事件全在一个循环里推进，状态是线性的，
  *      出问题容易定位；也避免"两个线程同时写串口把帧交错"这类坑。
  *
  *   2) **不用全局可变状态**，所有会话状态放在 link_t 里。
  *      真实项目里要同时管多个串口时，直接多开几个实例即可。
  *
  *   3) **只依赖 POSIX**（termios / select / clock_gettime）。
  *      A7Z 上直接可用；在 Linux PC 上接个 USB-TTL 也能跑，方便先联调。
  *      Windows 原生不支持这套 API —— 那边请用 tools/pc_link_test.py。
  *
  * ---------------------------------------------------------------------------
  * 编译：
  *
  *     gcc -std=c99 -Wall -Wextra -O2 -o mcu_link tools/mcu_link_example.c
  *
  * 运行（A7Z 侧，UART4）：
  *
  *     ./mcu_link /dev/ttyS4 --probe      # 只探活：PING + 读配置
  *     ./mcu_link /dev/ttyS4 --demo       # 完整演示：探活 → 前进 → 停
  *     ./mcu_link /dev/ttyS4 --monitor    # 只心跳 + 打印事件，不下发动作
  *
  * 在 PC 上用 USB-TTL 试：
  *
  *     ./mcu_link /dev/ttyUSB0 --demo
  *
  * ⚠️ 对接前必须确认两件事，否则一定会失败：
  *    a) 波特率 115200（A7Z 的 UART4 默认是 9600，是给 TTS 用的）
  *    b) TX/RX 交叉，且与 MCU 共地
  ******************************************************************************
  */
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <sys/select.h>

/* ============================ 协议常量 ============================ */

#define LINK_SOF              0xA5u
#define LINK_TYPE_CMD         0x01u
#define LINK_TYPE_ACK         0x02u
#define LINK_TYPE_EVT         0x03u

#define LINK_MAX_PAYLOAD      200u
#define LINK_MAX_FRAME        (6u + LINK_MAX_PAYLOAD + 2u)

/* 命令 */
#define OP_MOVE               0x01u
#define OP_SERVO              0x02u
#define OP_TTS                0x03u
#define OP_TOF_CFG            0x04u
#define OP_CLEAR_FAULT        0x05u
#define OP_PING               0x06u
#define OP_SET_CONFIG         0x07u
#define OP_GET_CONFIG         0x08u
#define OP_STEPPER_STAT       0x09u
#define OP_LINK_STAT          0x0Au
#define OP_STEPPER_TEST       0x0Bu
#define OP_MOTOR_STOP         0x0Cu

/* 应答 */
#define OP_ACK                0x81u
#define OP_NAK                0x82u
#define OP_PONG               0x86u
#define OP_CONFIG             0x87u

/* 事件 */
#define OP_EVT_EDGE_DETECTED  0x91u
#define OP_EVT_TOF_DATA       0x92u
#define OP_EVT_FAULT          0x93u
#define OP_EVT_MOTION_DONE    0x94u
#define OP_EVT_OBSTACLE       0x95u
#define OP_EVT_CONFIG_CHANGED 0x96u

/* MOVE 方向 */
#define DIR_FORWARD           0x00u
#define DIR_BACKWARD          0x01u
#define DIR_TURN_LEFT         0x02u
#define DIR_TURN_RIGHT        0x03u
#define DIR_STOP              0x04u

/* ============================ 本示例的配置 ============================ */

/* ★ 心跳周期。MCU 侧 A7Z 的判据是 1000 ms，这里留三倍余量。
 *   改大可以省带宽，但别超过 500 ms —— 超过就只剩一倍余量了。 */
#define HB_PERIOD_MS          300

/* 等应答的超时。115200 下一帧往返不到 1 ms，
 * 500 ms 已经非常宽松，剩下全靠 MCU 的处理时间。 */
#define REPLY_TIMEOUT_MS      500

/* 命令重试次数（只在超时时重试，收到 NAK 不重试） */
#define REPLY_RETRIES         2

/* 两字节之间超过这个时间就认为半帧废了，丢弃重新同步 */
#define INTERBYTE_TIMEOUT_MS  100u

/* ============================ 小工具 ============================ */

static uint32_t now_ms(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    {
        return 0u;
    }
    return (uint32_t)((uint32_t)ts.tv_sec * 1000u +
                      (uint32_t)(ts.tv_nsec / 1000000L));
}

/**
  * @brief  毫秒级睡眠
  * @note   用 nanosleep 而不是 usleep：usleep 在 POSIX.1-2008 里已废弃，
  *         开了 _POSIX_C_SOURCE 200809L 之后有些平台干脆不声明它。
  */
static void sleep_ms(unsigned ms)
{
    struct timespec ts;

    ts.tv_sec  = (time_t)(ms / 1000u);
    ts.tv_nsec = (long)(ms % 1000u) * 1000000L;
    (void)nanosleep(&ts, NULL);
}

static void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint16_t get_le16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_le32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

/* ============================ CRC16 ============================ */

/**
  * @brief  CRC16-CCITT-FALSE
  * @note   多项式 0x1021，初值 0xFFFF，不反转。
  *         ★ 校验范围是【从 SOF 开始】到 payload 末字节。
  *         从 TYPE 开始算是常见错误，会导致所有帧都被判坏帧。
  *         校验向量："123456789" -> 0x29B1
  */
static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    size_t   i;
    int      b;

    for (i = 0u; i < len; i++)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (b = 0; b < 8; b++)
        {
            if ((crc & 0x8000u) != 0u)
            {
                crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021u);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ============================ 会话结构 ============================ */

typedef struct
{
    /* ---- 串口 ---- */
    int      fd;
    char     path[128];

    /* ---- 序号 ---- */
    uint16_t next_seq;

    /* ---- 最近一次收到的帧（供 wait_reply 用）---- */
    uint16_t rx_seq;
    uint8_t  rx_type;      /* 0x02=ACK/NAK  0x03=EVT */
    uint8_t  rx_op;
    uint8_t  rx_payload[LINK_MAX_PAYLOAD];
    uint16_t rx_len;
    int      rx_ready;

    /* ---- 拆帧状态机 ---- */
    uint8_t  st;
    uint8_t  type;
    uint16_t seq;
    uint8_t  op;
    uint16_t len;
    uint16_t idx;
    uint8_t  payload[LINK_MAX_PAYLOAD];
    uint16_t crc_rx;
    uint32_t last_byte_ms;
    int      have_last_byte;

    /* ---- 还没喂完的字节 ----
     * 一次 read() 很可能读到「一帧 ACK + 一帧 EVT」。
     * 如果返回第一帧时把剩下的丢掉，就会**静默丢帧** ——
     * 实测表现正是"电机走完了但没打印 MOTION_DONE"。
     * 这里把余下的字节存起来，下次 link_pump 先喂它们。 */
    uint8_t  pend[256];
    uint16_t pend_len;

    /* ---- 统计 ---- */
    uint32_t good_frames;
    uint32_t bad_frames;
    uint32_t rx_bytes;

    /* ---- 心跳 ---- */
    uint32_t last_hb_ms;

    /* ---- 故障缓存（来自 PONG / EVT_FAULT）---- */
    uint8_t  fault;
} link_t;

/* 拆帧状态 */
enum
{
    ST_SOF = 0,
    ST_TYPE,
    ST_SEQ_L,
    ST_SEQ_H,
    ST_OP,
    ST_LEN,
    ST_PAYLOAD,
    ST_CRC_L,
    ST_CRC_H
};

/* ============================ 串口 ============================ */

/**
  * @brief  打开串口并配成 raw 模式
  * @param  l     会话
  * @param  path  设备路径，如 /dev/ttyS4
  * @param  baud  波特率（本项目固定 115200）
  * @return 0 成功，-1 失败
  */
static int link_open(link_t *l, const char *path, speed_t baud)
{
    struct termios tio;

    memset(l, 0, sizeof(*l));
    l->fd = -1;
    l->next_seq = 1u;
    l->last_hb_ms = now_ms();
    (void)snprintf(l->path, sizeof(l->path), "%s", path);

    l->fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (l->fd < 0)
    {
        fprintf(stderr, "打开 %s 失败: %s\n", path, strerror(errno));
        return -1;
    }

    if (tcgetattr(l->fd, &tio) != 0)
    {
        fprintf(stderr, "tcgetattr 失败: %s\n", strerror(errno));
        close(l->fd);
        l->fd = -1;
        return -1;
    }

    /* raw 模式：不做任何行处理、不回显、不转换换行。
     *
     * 这里手写而不用 cfmakeraw()：后者在 glibc 里要 _DEFAULT_SOURCE，
     * 在部分嵌入式 libc 上也没有。手写更可控，也顺便说明了到底关了什么。
     *
     * ⚠️ 不做这一步的后果：
     *    ICRNL/OPOST 会把 0x0D/0x0A 改掉，IXON 会把 0x11/0x13 当流控吞掉，
     *    ICANON 会攒到换行才给你 —— 表现为"偶发坏帧"，而且极难查。 */
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                     INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    tio.c_oflag &= ~OPOST;
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
    tio.c_cflag |= CS8 | CLOCAL | CREAD;
    tio.c_cc[VMIN]  = 0;    /* 非阻塞读 */
    tio.c_cc[VTIME] = 0;

    if (cfsetispeed(&tio, baud) != 0 || cfsetospeed(&tio, baud) != 0)
    {
        fprintf(stderr, "设置波特率失败\n");
        close(l->fd);
        l->fd = -1;
        return -1;
    }

    /* TCSANOW：立刻生效。改成 TCSAFLUSH 会丢掉已收到的数据。 */
    if (tcsetattr(l->fd, TCSANOW, &tio) != 0)
    {
        fprintf(stderr, "tcsetattr 失败: %s\n", strerror(errno));
        close(l->fd);
        l->fd = -1;
        return -1;
    }

    /* 清掉可能残留的旧数据，从干净状态开始 */
    tcflush(l->fd, TCIOFLUSH);

    return 0;
}

static void link_close(link_t *l)
{
    if (l->fd >= 0)
    {
        close(l->fd);
        l->fd = -1;
    }
}

static int link_write_all(link_t *l, const uint8_t *buf, size_t len)
{
    size_t off = 0u;

    while (off < len)
    {
        ssize_t n = write(l->fd, buf + off, len - off);

        if (n < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            if (errno == EAGAIN)
            {
                /* 非阻塞写满了：让出一下再继续。
                 * 正常不会发生 —— 一帧最多 208 字节，串口缓冲够放。 */
                sleep_ms(1u);
                continue;
            }
            return -1;
        }
        off += (size_t)n;
    }
    return 0;
}

/* ============================ 组帧 ============================ */

/**
  * @brief  组一帧
  * @return 总字节数，失败返回 0
  */
static size_t link_build(uint8_t *out, size_t out_size, uint8_t type,
                         uint16_t seq, uint8_t op,
                         const uint8_t *payload, uint16_t len)
{
    uint16_t total;
    uint16_t crc;

    if (len > LINK_MAX_PAYLOAD)
    {
        return 0u;
    }

    total = (uint16_t)(6u + len + 2u);
    if (out_size < total)
    {
        return 0u;
    }

    out[0] = LINK_SOF;
    out[1] = type;
    put_le16(&out[2], seq);
    out[4] = op;
    out[5] = (uint8_t)len;

    if ((len > 0u) && (payload != NULL))
    {
        memcpy(&out[6], payload, len);
    }

    /* ★ 从 out[0]（SOF）开始算，长度 6+len */
    crc = crc16_ccitt_false(out, (size_t)(6u + len));
    put_le16(&out[6u + len], crc);

    return (size_t)total;
}

/* ============================ 拆帧 ============================ */

static void parser_reset(link_t *l)
{
    l->st  = ST_SOF;
    l->idx = 0u;
    l->len = 0u;
}

/**
  * @brief  喂一个字节
  * @return 1 = 凑齐一帧（结果放在 l 的 rx_* 字段），0 = 还没凑齐，-1 = 坏帧
  * @note   坏帧返回 -1 但**不要中止**，后续字节继续喂，
  *         靠 SOF 自然重新同步。
  */
static int parser_feed(link_t *l, uint8_t b, uint32_t t_ms)
{
    /* 半帧超时：噪声里的假 SOF 若不丢弃，会把后面紧跟的真帧吞掉 */
    if (l->have_last_byte &&
        (uint32_t)(t_ms - l->last_byte_ms) > INTERBYTE_TIMEOUT_MS)
    {
        parser_reset(l);
    }
    l->have_last_byte = 1;
    l->last_byte_ms   = t_ms;

    switch (l->st)
    {
    case ST_SOF:
        if (b == LINK_SOF)
        {
            l->st = ST_TYPE;
        }
        return 0;

    case ST_TYPE:
        if ((b != LINK_TYPE_ACK) && (b != LINK_TYPE_EVT))
        {
            /* 不是发给我们的类型：可能是噪声。
             * 若 b 恰好是 SOF，就停在 TYPE 状态等下一个字节。 */
            l->st = (b == LINK_SOF) ? ST_TYPE : ST_SOF;
            return 0;
        }
        l->type = b;
        l->st   = ST_SEQ_L;
        return 0;

    case ST_SEQ_L:
        l->seq = b;
        l->st  = ST_SEQ_H;
        return 0;

    case ST_SEQ_H:
        l->seq = (uint16_t)(l->seq | ((uint16_t)b << 8));
        l->st  = ST_OP;
        return 0;

    case ST_OP:
        l->op = b;
        l->st = ST_LEN;
        return 0;

    case ST_LEN:
        if (b > LINK_MAX_PAYLOAD)
        {
            /* 长度非法：把这一字节当新的 SOF 起点 */
            l->st = (b == LINK_SOF) ? ST_TYPE : ST_SOF;
            return -1;
        }
        l->len = b;
        l->idx = 0u;
        l->st  = (b == 0u) ? ST_CRC_L : ST_PAYLOAD;
        return 0;

    case ST_PAYLOAD:
        l->payload[l->idx++] = b;
        if (l->idx >= l->len)
        {
            l->st = ST_CRC_L;
        }
        return 0;

    case ST_CRC_L:
        l->crc_rx = b;
        l->st     = ST_CRC_H;
        return 0;

    case ST_CRC_H:
    {
        uint8_t  raw[LINK_MAX_FRAME];
        uint16_t crc_calc;

        l->crc_rx = (uint16_t)(l->crc_rx | ((uint16_t)b << 8));

        raw[0] = LINK_SOF;
        raw[1] = l->type;
        put_le16(&raw[2], l->seq);
        raw[4] = l->op;
        raw[5] = (uint8_t)l->len;
        if (l->len > 0u)
        {
            memcpy(&raw[6], l->payload, l->len);
        }

        crc_calc = crc16_ccitt_false(raw, (size_t)(6u + l->len));

        if (crc_calc != l->crc_rx)
        {
            l->bad_frames++;
            parser_reset(l);
            return -1;
        }

        /* 收下了 */
        l->good_frames++;
        l->rx_seq  = l->seq;
        l->rx_type = l->type;
        l->rx_op   = l->op;
        l->rx_len  = l->len;
        memcpy(l->rx_payload, l->payload, l->len);
        l->rx_ready = 1;

        parser_reset(l);
        return 1;
    }

    default:
        parser_reset(l);
        return 0;
    }
}

/* ============================ 事件打印 ============================ */

static const char *fault_name(uint8_t f)
{
    switch (f)
    {
    case 0x00: return "NONE";
    case 0x01: return "HEARTBEAT_LOST";
    case 0x02: return "EDGE(桌沿)";
    case 0x03: return "OBSTACLE(障碍)";
    case 0x04: return "THERMAL(过热)";
    case 0x05: return "TOF_INIT";
    case 0x0A: return "NO_EDGE_PROTECT(盲走)";
    default:   return "?";
    }
}

static const char *status_name(uint8_t s)
{
    switch (s)
    {
    case 0x00: return "OK";
    case 0x01: return "CLAMPED";
    case 0x02: return "BUSY";
    case 0x03: return "FAULT_LATCHED";
    case 0x04: return "CRC_ERROR";
    case 0x05: return "UNKNOWN_OP";
    case 0x06: return "FEATURE_DISABLED";
    case 0x07: return "FLASH_FAIL";
    case 0x09: return "ARBITRATION";
    default:   return "?";
    }
}

/**
  * @brief  处理主动事件（TYPE=0x03，SEQ=0）
  * @note   事件是"顺便看到就处理"，不影响命令流程。
  */
static void handle_event(link_t *l)
{
    const uint8_t *p = l->rx_payload;

    switch (l->rx_op)
    {
    case OP_EVT_TOF_DATA:
        if (l->rx_len >= 9u)
        {
            /* mask(1) + d0..d3(2 each)；0xFFFF = 无效 */
            printf("  [EVT] TOF mask=0x%02X  d0=%d d1=%d d2=%d d3=%d\n",
                   (unsigned)p[0],
                   (get_le16(&p[1]) == 0xFFFFu) ? -1 : (int)get_le16(&p[1]),
                   (get_le16(&p[3]) == 0xFFFFu) ? -1 : (int)get_le16(&p[3]),
                   (get_le16(&p[5]) == 0xFFFFu) ? -1 : (int)get_le16(&p[5]),
                   (get_le16(&p[7]) == 0xFFFFu) ? -1 : (int)get_le16(&p[7]));
        }
        break;

    case OP_EVT_EDGE_DETECTED:
        if (l->rx_len >= 3u)
        {
            printf("  [EVT] 桌沿! sensor=%u dist=%d mm\n",
                   (unsigned)p[0], (int)get_le16(&p[1]));
        }
        break;

    case OP_EVT_OBSTACLE:
        if (l->rx_len >= 2u)
        {
            printf("  [EVT] 障碍! dist=%d mm\n", (int)get_le16(&p[0]));
        }
        break;

    case OP_EVT_MOTION_DONE:
        if (l->rx_len >= 5u)
        {
            printf("  [EVT] 电机 %u 走完, position=%ld\n",
                   (unsigned)p[0], (long)(int32_t)get_le32(&p[1]));
        }
        break;

    case OP_EVT_FAULT:
        if (l->rx_len >= 1u)
        {
            l->fault = p[0];
            printf("  [EVT] 故障 0x%02X %s\n",
                   (unsigned)p[0], fault_name(p[0]));
        }
        break;

    case OP_EVT_CONFIG_CHANGED:
        if (l->rx_len >= 4u)
        {
            printf("  [EVT] 配置变更为 0x%08lX\n",
                   (unsigned long)get_le32(&p[0]));
        }
        break;

    default:
        printf("  [EVT] 未知事件 op=0x%02X len=%u\n",
               (unsigned)l->rx_op, (unsigned)l->rx_len);
        break;
    }
}

/* ============================ 收发主循环 ============================ */

/**
  * @brief  喂一段字节，最多喂到"凑齐一帧"就停
  * @param  consumed  实际喂掉了多少字节（含完成那一帧的最后一个字节）
  * @return 1 = 凑齐一帧，0 = 这段喂完了也没凑齐
  * @note   返回值只表示"有没有帧"，没喂完的字节由调用方保存。
  */
static int pump_bytes(link_t *l, const uint8_t *p, size_t n, size_t *consumed)
{
    size_t i;

    for (i = 0u; i < n; i++)
    {
        if (parser_feed(l, p[i], now_ms()) == 1)
        {
            *consumed = i + 1u;
            return 1;
        }
    }
    *consumed = n;
    return 0;
}

/**
  * @brief  最多等 timeout_ms，期间持续收字节、按需发心跳
  * @return 收到一帧返回 1（含事件，结果在 l->rx_*），超时返回 0，出错返回 -1
  * @note   ★ 这个函数是整个示例的核心。
  *         所有"等待"都必须走它，绝对不能直接 sleep 或阻塞 read ——
  *         否则等应答期间心跳就断了，MCU 会判心跳丢失并停车。
  *
  *         ★ 事件【不在这里处理】，而是原样返回给调用方。
  *           早期版本在这里直接吞掉事件，导致上层永远等不到 MOTION_DONE，
  *           表现为"电机明明走完了，程序还在傻等"。
  */
static int link_pump(link_t *l, uint32_t timeout_ms)
{
    uint32_t start = now_ms();
    uint8_t  buf[256];

    for (;;)
    {
        uint32_t t     = now_ms();
        uint32_t spent = (uint32_t)(t - start);
        uint32_t left;
        fd_set   rfds;
        struct timeval tv;
        int      rc;

        /* ---- 心跳到点就发 ---- */
        if ((uint32_t)(t - l->last_hb_ms) >= HB_PERIOD_MS)
        {
            uint8_t fr[LINK_MAX_FRAME];
            size_t  n;

            l->last_hb_ms = t;
            n = link_build(fr, sizeof(fr), LINK_TYPE_CMD,
                           l->next_seq++, OP_PING, NULL, 0u);
            if (n > 0u)
            {
                if (link_write_all(l, fr, n) != 0)
                {
                    return -1;
                }
            }
        }

        /* ---- 到时间了 ---- */
        if (spent >= timeout_ms)
        {
            return 0;
        }
        left = timeout_ms - spent;

        /* ---- select：既等数据也不睡死，最长 left 毫秒 ---- */
        FD_ZERO(&rfds);
        FD_SET(l->fd, &rfds);
        tv.tv_sec  = (time_t)(left / 1000u);
        tv.tv_usec = (suseconds_t)((left % 1000u) * 1000u);

        rc = select(l->fd + 1, &rfds, NULL, NULL, &tv);
        if (rc < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            return -1;
        }
        if (rc == 0)
        {
            return 0;           /* 超时 */
        }

        /* ---- 先喂上次没喂完的字节 ----
         * 必须在 select 之前做：这些字节已经在本地了，
         * 等 select 会白等一个超时。 */
        if (l->pend_len > 0u)
        {
            size_t used = 0u;
            int    r    = pump_bytes(l, l->pend, l->pend_len, &used);

            if (r == 1)
            {
                l->pend_len = (uint16_t)(l->pend_len - used);
                if (l->pend_len > 0u)
                {
                    memmove(l->pend, l->pend + used, l->pend_len);
                }
                return 1;
            }
            l->pend_len = 0u;   /* 全喂完了也没凑齐一帧 */
        }

        /* ---- 读干净 ---- */
        for (;;)
        {
            ssize_t n = read(l->fd, buf, sizeof(buf));

            if (n <= 0)
            {
                if ((n < 0) && (errno == EINTR))
                {
                    continue;
                }
                break;          /* EAGAIN 或 0：本轮到头 */
            }

            l->rx_bytes += (uint32_t)n;
            {
                size_t used = 0u;
                int    r    = pump_bytes(l, buf, (size_t)n, &used);

                if (r == 1)
                {
                    /* ★ 这一批里可能还有后续帧，不能丢。
                     *   实测就是这个原因导致 MOTION_DONE 被吞掉。 */
                    l->pend_len = (uint16_t)((size_t)n - used);
                    if (l->pend_len > 0u)
                    {
                        memcpy(l->pend, buf + used, l->pend_len);
                    }
                    return 1;
                }
            }
        }
    }
}

/**
  * @brief  取下一个序号
  * @note   uint16 自然回绕，跳过 0（0 留给主动事件）。
  */
static uint16_t next_seq(link_t *l)
{
    uint16_t s = l->next_seq++;

    if (l->next_seq == 0u)
    {
        l->next_seq = 1u;
    }
    return s;
}

/**
  * @brief  发一条命令并等它的应答
  * @param  op/payload/len  请求内容
  * @return 1 = 收到应答（l->rx_* 有效），0 = 超时/失败
  * @note   超时会重试 REPLY_RETRIES 次。
  *         收到 NAK **不重试** —— 参数错重试多少次都是错的。
  */
static int link_ask(link_t *l, uint8_t op, const uint8_t *payload,
                    uint16_t len)
{
    int attempt;

    for (attempt = 0; attempt <= REPLY_RETRIES; attempt++)
    {
        uint8_t fr[LINK_MAX_FRAME];
        size_t  n;
        uint16_t seq = next_seq(l);
        uint32_t deadline;

        n = link_build(fr, sizeof(fr), LINK_TYPE_CMD, seq, op, payload, len);
        if (n == 0u)
        {
            fprintf(stderr, "组帧失败\n");
            return 0;
        }

        /* 清掉上一条的应答，避免张冠李戴 */
        l->rx_ready = 0;

        if (link_write_all(l, fr, n) != 0)
        {
            fprintf(stderr, "写串口失败: %s\n", strerror(errno));
            return 0;
        }

        deadline = now_ms() + REPLY_TIMEOUT_MS;
        while ((int32_t)(now_ms() - deadline) < 0)
        {
            int r = link_pump(l, 50u);

            if (r < 0)
            {
                return 0;
            }
            if (r == 0)
            {
                continue;       /* 只是这一段没数据，继续等到 deadline */
            }

            /* 等应答期间收到的事件：顺手处理，然后继续等 */
            if (l->rx_type == LINK_TYPE_EVT)
            {
                handle_event(l);
                continue;
            }

            /* ★ 靠 SEQ 配对，不是靠 OP。
             *   查询类命令的应答会回显请求的 OP（如 PONG=0x86），
             *   只判断 OP == 0x81 会漏掉一半的应答。 */
            if (l->rx_seq != seq)
            {
                continue;       /* 是别人（旧命令或心跳）的应答 */
            }

            l->rx_ready = 1;
            return 1;
        }

        printf("  ! 超时，重试 %d/%d\n", attempt + 1, REPLY_RETRIES);
    }

    return 0;
}

/* ============================ 业务层 ============================ */

/**
  * @brief  PING → 解析 PONG
  * @return 1 成功
  */
static int cmd_ping(link_t *l)
{
    const uint8_t *p;

    if (!link_ask(l, OP_PING, NULL, 0u))
    {
        printf("  ✗ PING 无应答\n");
        return 0;
    }
    if (l->rx_op != OP_PONG || l->rx_len < 11u)
    {
        printf("  ✗ PING 应答异常 op=0x%02X len=%u\n",
               (unsigned)l->rx_op, (unsigned)l->rx_len);
        return 0;
    }

    p = l->rx_payload;
    l->fault = p[4];

    printf("  ✓ PONG uptime=%lu ms  fault=0x%02X(%s)\n",
           (unsigned long)get_le32(&p[0]),
           (unsigned)p[4], fault_name(p[4]));
    printf("         config=0x%08lX  flags=0x%02X  impl=0x%02X\n",
           (unsigned long)get_le32(&p[5]),
           (unsigned)p[9], (unsigned)p[10]);

    /* impl 位图能一眼看出板子里是哪一版固件 */
    if (p[10] != 0x3Fu)
    {
        printf("  ! impl=0x%02X 不是 0x3F，板子里可能是旧固件\n",
               (unsigned)p[10]);
    }
    if ((p[9] & 0x02u) != 0u)
    {
        printf("  ! 处在盲走模式（底部 TOF 全关），桌沿无保护\n");
    }

    return 1;
}

static int cmd_get_config(link_t *l)
{
    if (!link_ask(l, OP_GET_CONFIG, NULL, 0u))
    {
        printf("  ✗ GET_CONFIG 无应答\n");
        return 0;
    }
    if (l->rx_op != OP_CONFIG || l->rx_len < 4u)
    {
        printf("  ✗ GET_CONFIG 应答异常\n");
        return 0;
    }

    printf("  ✓ config = 0x%08lX\n",
           (unsigned long)get_le32(l->rx_payload));
    return 1;
}

/**
  * @brief  MOVE
  * @param  dir    方向
  * @param  speed  档位 0..3
  * @param  steps  步数
  * @return 1 已被接受
  * @note   ★ MOVE 的 ACK 有两个字节：status + 预测距离(mm/10)。
  *         第二个字节不是状态码，超过 254 会饱和成 0xFF。
  */
static int cmd_move(link_t *l, uint8_t dir, uint8_t speed, uint16_t steps)
{
    uint8_t p[4];

    p[0] = dir;
    p[1] = speed;
    put_le16(&p[2], steps);

    if (!link_ask(l, OP_MOVE, p, 4u))
    {
        printf("  ✗ MOVE 无应答\n");
        return 0;
    }

    if (l->rx_op == OP_ACK)
    {
        uint8_t st = l->rx_payload[0];

        if (st == 0x00u)
        {
            if (l->rx_len >= 2u)
            {
                uint8_t mm10 = l->rx_payload[1];

                if (mm10 == 0xFFu)
                {
                    printf("  ✓ ACK 预计距离 > 25.4 mm（已饱和）\n");
                }
                else
                {
                    printf("  ✓ ACK 预计距离 %.1f mm\n", mm10 / 10.0);
                }
            }
            else
            {
                printf("  ✓ ACK\n");
            }
            return 1;
        }
        printf("  ! ACK 但状态是 0x%02X(%s)\n", st, status_name(st));
        return 0;
    }

    if (l->rx_op == OP_NAK)
    {
        uint8_t st  = l->rx_payload[0];
        uint8_t det = (l->rx_len >= 2u) ? l->rx_payload[1] : 0u;

        printf("  ✗ NAK 0x%02X(%s) detail=0x%02X\n",
               st, status_name(st), det);

        /* 解释常见的拒绝原因 */
        if (st == 0x03u)
        {
            if (det == 0x02u)
            {
                printf("      -> 桌沿故障，先发 CLEAR_FAULT(0x01)\n");
            }
            else if (det == 0x03u)
            {
                printf("      -> 障碍故障，把障碍移开后自动恢复\n");
            }
            else if (det == 0x0Au)
            {
                printf("      -> 盲走模式，需 CLEAR_FAULT(0x00, 0x5A) 确认\n");
            }
            else
            {
                printf("      -> 步进层拒绝（忙或过热），"
                       "查 STEPPER_STAT 的 cooling 位\n");
            }
        }
        else if (st == 0x09u)
        {
            printf("      -> 控制权不在 A7Z（蓝牙在控制），"
                   "保持持续发帧即可夺回\n");
        }
        return 0;
    }

    printf("  ✗ MOVE 应答异常 op=0x%02X\n", (unsigned)l->rx_op);
    return 0;
}

static int cmd_motor_stop(link_t *l)
{
    if (!link_ask(l, OP_MOTOR_STOP, NULL, 0u))
    {
        printf("  ✗ MOTOR_STOP 无应答\n");
        return 0;
    }
    printf("  ✓ 已停止\n");
    return 1;
}

/* ============================ 运行模式 ============================ */

static int run_probe(link_t *l)
{
    printf("\n[1] PING\n");
    if (!cmd_ping(l))
    {
        return 1;
    }
    printf("\n[2] GET_CONFIG\n");
    (void)cmd_get_config(l);

    printf("\n统计: good=%u bad=%u rx=%u 字节\n",
           (unsigned)l->good_frames, (unsigned)l->bad_frames,
           (unsigned)l->rx_bytes);
    return 0;
}

static int run_demo(link_t *l)
{
    uint32_t t0;

    printf("\n[1] PING\n");
    if (!cmd_ping(l))
    {
        return 1;
    }

    printf("\n[2] GET_CONFIG\n");
    (void)cmd_get_config(l);

    printf("\n[3] MOVE 前进 2000 步，档位 2（300 Hz）\n");
    if (!cmd_move(l, DIR_FORWARD, 2u, 2000u))
    {
        printf("  动作被拒绝，退出\n");
        return 1;
    }

    /* ---- 等它走完 ----
     * 2000 步 @300Hz ≈ 6.7 秒。这里等 10 秒。
     * ★ 等待期间必须持续跑 link_pump 发心跳，否则 1 秒后就被停车。 */
    printf("\n[4] 等待 MOTION_DONE（期间持续发心跳）\n");
    t0 = now_ms();
    while ((uint32_t)(now_ms() - t0) < 10000u)
    {
        int r = link_pump(l, 100u);

        if (r < 0)
        {
            printf("  串口出错\n");
            return 1;
        }
        if (r == 0)
        {
            continue;
        }
        if (l->rx_type != LINK_TYPE_EVT)
        {
            continue;           /* 心跳的 PONG，忽略 */
        }

        handle_event(l);
        if (l->rx_op == OP_EVT_MOTION_DONE)
        {
            printf("  用时 %lu ms\n", (unsigned long)(now_ms() - t0));
            break;
        }
    }

    printf("\n[5] MOTOR_STOP\n");
    (void)cmd_motor_stop(l);

    printf("\n统计: good=%u bad=%u rx=%u 字节\n",
           (unsigned)l->good_frames, (unsigned)l->bad_frames,
           (unsigned)l->rx_bytes);

    if (l->bad_frames > 0u)
    {
        printf("⚠️  有 %u 个坏帧 —— 查波特率与 CRC 算法\n",
               (unsigned)l->bad_frames);
    }
    return 0;
}

/**
  * @brief  监控模式：只发心跳，打印所有事件
  * @note   用来观察 TOF 距离、桌沿触发、故障上报。
  *         按 Ctrl+C 退出。
  */
static int run_monitor(link_t *l)
{
    int did_ping = 0;

    printf("监控中（Ctrl+C 退出）。心跳 %d ms，只打印事件。\n",
           HB_PERIOD_MS);

    while (1)
    {
        int r = link_pump(l, 200u);

        if (r < 0)
        {
            printf("串口出错\n");
            return 1;
        }
        if (r == 0)
        {
            continue;
        }
        if (l->rx_type == LINK_TYPE_EVT)
        {
            handle_event(l);
        }
        else if (!did_ping && (l->rx_op == OP_PONG))
        {
            did_ping = 1;
            printf("链路已建立（PONG 收到）\n");
        }
    }
    return 0;
}

/* ============================ main ============================ */

static void usage(const char *prog)
{
    printf(
        "用法: %s <串口设备> [--probe|--demo|--monitor]\n"
        "\n"
        "  --probe    只探活（PING + 读配置），适合确认接线\n"
        "  --demo     PING → 前进 2000 步 → 停止（默认）\n"
        "  --monitor  只发心跳并打印事件，用来观察 TOF 与故障\n"
        "\n"
        "例:\n"
        "  %s /dev/ttyS4 --probe\n"
        "  %s /dev/ttyUSB0 --demo\n",
        prog, prog, prog);
}

int main(int argc, char **argv)
{
    link_t l;
    const char *path;
    const char *mode = "--demo";
    speed_t     baud = B115200;
    int         rc;

    if (argc < 2)
    {
        usage(argv[0]);
        return 2;
    }

    path = argv[1];
    if (argc >= 3)
    {
        mode = argv[2];
    }

    printf("==============================\n");
    printf(" STM32 下位机通信最小示例\n");
    printf(" 设备: %s\n", path);
    printf(" 参数: 115200 8N1\n");
    printf(" 模式: %s\n", mode);
    printf("==============================\n");

    if (link_open(&l, path, baud) != 0)
    {
        return 1;
    }

    if (strcmp(mode, "--probe") == 0)
    {
        rc = run_probe(&l);
    }
    else if (strcmp(mode, "--monitor") == 0)
    {
        rc = run_monitor(&l);
    }
    else
    {
        rc = run_demo(&l);
    }

    /* ★ 退出前一定先停车。否则 MCU 会继续跑，
     *   直到 1 秒后心跳超时才停 —— 这段时间机器人是失控的。 */
    (void)cmd_motor_stop(&l);
    link_close(&l);

    return rc;
}
