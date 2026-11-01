/**
  ******************************************************************************
  * @file    Protocol/link_proto.c
  * @brief   帧组包 / 拆包 / CRC16-CCITT-FALSE
  *
  * M6/M7 起拆帧状态机改为【实例化】：
  *   USART1（A7Z）和 USART3（蓝牙）各持有一个 link_parser_t，
  *   互不干扰。老的单例 API 保留为一层薄封装，指向默认实例。
  ******************************************************************************
  */
#include <string.h>
#include "app_config.h"
#include "link_proto.h"

/* ============================ CRC16-CCITT-FALSE ============================ */
/* 多项式 0x1021，初值 0xFFFF，输入/输出均不反转，无最终异或。
 * 校验范围不含 CRC 自身两个字节。
 */
uint16_t Link_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    uint8_t  b;

    if (data == 0)
    {
        return 0xFFFFu;
    }

    for (i = 0u; i < len; i++)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (b = 0u; b < 8u; b++)
        {
            if (crc & 0x8000u)
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

/* ============================ 组帧 ============================ */
int Link_BuildFrame(uint8_t *out, uint16_t out_size,
                    uint8_t type, uint16_t seq, uint8_t op,
                    const uint8_t *payload, uint8_t len)
{
    uint16_t total;
    uint16_t crc;

    if (out == 0)
    {
        return LINK_ERR_NULL;
    }
    if (len > LINK_MAX_PAYLOAD)
    {
        return LINK_ERR_TOOLONG;
    }

    total = (uint16_t)(6u + len + 2u);
    if (out_size < total)
    {
        return LINK_ERR_TOOLONG;
    }

    out[0] = LINK_SOF;
    out[1] = type;
    out[2] = (uint8_t)(seq & 0x00FFu);          /* 小端 */
    out[3] = (uint8_t)((seq >> 8) & 0x00FFu);
    out[4] = op;
    out[5] = len;

    if ((len > 0u) && (payload != 0))
    {
        memcpy(&out[6], payload, (size_t)len);
    }

    crc = Link_Crc16(out, (uint16_t)(6u + len));
    out[6u + len]      = (uint8_t)(crc & 0x00FFu);
    out[6u + len + 1u] = (uint8_t)((crc >> 8) & 0x00FFu);

    return (int)total;
}

/* ============================ 拆帧状态机 ============================ */
typedef enum
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
} parse_state_t;

void Link_ParserInit(link_parser_t *p)
{
    if (p == 0)
    {
        return;
    }
    memset(p, 0, sizeof(*p));
    p->state = (uint8_t)ST_SOF;
}

/* 把已接受的字节追加进原始镜像（不含 CRC 两字节）*/
static void raw_push(link_parser_t *p, uint8_t b)
{
    if (p->raw_len < (uint16_t)sizeof(p->raw))
    {
        p->raw[p->raw_len++] = b;
    }
}

static void parser_reset(link_parser_t *p)
{
    p->state     = (uint8_t)ST_SOF;
    p->idx       = 0u;
    p->raw_len   = 0u;
    p->crc_rx    = 0u;
    memset(&p->acc, 0, sizeof(p->acc));
}

int Link_FeedByteTo(link_parser_t *p, uint8_t byte,
                    uint32_t now_ms, link_frame_t *out)
{
    if (p == 0)
    {
        return -1;
    }

    /* ---- 帧间超时保护 ----
     * 若正在拼帧却已超时，说明之前遇到的是假 SOF（噪声），
     * 立刻丢弃半帧，并把当前字节当作新的起点处理。
     * 这一步能避免"假 SOF + 大 LEN 吞掉紧随其后的真帧"。
     *
     * ★ 每个端口有自己的 last_byte_ms，所以在蓝牙端口刷屏时
     *   不会把 A7Z 侧正在拼的半帧误判超时。
     */
    if ((p->state != (uint8_t)ST_SOF) && (p->have_last != 0u) &&
        ((uint32_t)(now_ms - p->last_byte_ms) > LINK_INTERBYTE_TIMEOUT_MS))
    {
        p->timeout_drops++;
        p->bad++;
        parser_reset(p);
    }

    p->last_byte_ms = now_ms;
    p->have_last = 1u;

    switch ((parse_state_t)p->state)
    {
    case ST_SOF:
        if (byte == LINK_SOF)
        {
            parser_reset(p);
            raw_push(p, byte);
            p->state = (uint8_t)ST_TYPE;
        }
        /* 非 SOF：正常的重同步过程，不计入统计 */
        return 0;

    case ST_TYPE:
        p->acc.type = byte;
        raw_push(p, byte);
        p->state = (uint8_t)ST_SEQ_L;
        return 0;

    case ST_SEQ_L:
        p->acc.seq = byte;
        raw_push(p, byte);
        p->state = (uint8_t)ST_SEQ_H;
        return 0;

    case ST_SEQ_H:
        p->acc.seq = (uint16_t)(p->acc.seq | ((uint16_t)byte << 8));
        raw_push(p, byte);
        p->state = (uint8_t)ST_OP;
        return 0;

    case ST_OP:
        p->acc.op = byte;
        raw_push(p, byte);
        p->state = (uint8_t)ST_LEN;
        return 0;

    case ST_LEN:
        if (byte > LINK_MAX_PAYLOAD)
        {
            p->bad++;
            p->drop++;
            parser_reset(p);
            return -1;
        }
        p->acc.len = byte;
        p->idx = 0u;
        raw_push(p, byte);
        p->state = (byte == 0u) ? (uint8_t)ST_CRC_L : (uint8_t)ST_PAYLOAD;
        return 0;

    case ST_PAYLOAD:
        if (p->idx < LINK_MAX_PAYLOAD)
        {
            p->acc.payload[p->idx] = byte;
        }
        raw_push(p, byte);
        p->idx++;
        if (p->idx >= p->acc.len)
        {
            p->state = (uint8_t)ST_CRC_L;
        }
        return 0;

    case ST_CRC_L:
        p->crc_rx = byte;
        p->state = (uint8_t)ST_CRC_H;
        return 0;

    case ST_CRC_H:
    {
        uint16_t crc_calc;

        p->crc_rx = (uint16_t)(p->crc_rx | ((uint16_t)byte << 8));
        crc_calc = Link_Crc16(p->raw, p->raw_len);

        if (crc_calc != p->crc_rx)
        {
            p->bad++;
            parser_reset(p);
            return -1;
        }

        if (out != 0)
        {
            *out = p->acc;
        }
        p->good++;
        parser_reset(p);
        return 1;
    }

    default:
        parser_reset(p);
        return -1;
    }
}

/* ============================ 兼容用：默认实例 ============================ */
static link_parser_t s_default;

void Link_ParserReset(void)
{
    parser_reset(&s_default);
}

int Link_FeedByte(uint8_t byte, uint32_t now_ms, link_frame_t *out)
{
    return Link_FeedByteTo(&s_default, byte, now_ms, out);
}

uint32_t Link_GetTimeoutDrops(void) { return s_default.timeout_drops; }
uint32_t Link_GetGoodFrames(void)   { return s_default.good; }
uint32_t Link_GetBadFrames(void)    { return s_default.bad; }
uint32_t Link_GetByteDrops(void)    { return s_default.drop; }
