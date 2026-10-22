/**
  ******************************************************************************
  * @file    Protocol/link_proto.c
  * @brief   帧组包 / 拆包 / CRC16-CCITT-FALSE
  ******************************************************************************
  */
#include <string.h>
#include "app_config.h"
#include "link_proto.h"

/* ============================ 统计 ============================ */
static uint32_t s_good = 0u;
static uint32_t s_bad = 0u;
static uint32_t s_drop = 0u;

uint32_t Link_GetGoodFrames(void) { return s_good; }
uint32_t Link_GetBadFrames(void)  { return s_bad;  }
uint32_t Link_GetByteDrops(void)  { return s_drop; }

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

static parse_state_t s_state = ST_SOF;
static link_frame_t  s_acc;                       /* 累积中的帧 */
static uint8_t       s_raw[LINK_MAX_FRAME];       /* 原始字节镜像（用于 CRC）*/
static uint16_t      s_raw_len = 0u;
static uint8_t       s_idx = 0u;                  /* payload 写入下标 */
static uint16_t      s_crc_rx = 0u;
static uint32_t      s_last_byte_ms = 0u;
static uint8_t       s_have_last = 0u;
static uint32_t      s_timeout_drops = 0u;

/* 半帧超时保护：见 app_config.h 的 LINK_INTERBYTE_TIMEOUT_MS 说明 */
uint32_t Link_GetTimeoutDrops(void)
{
    return s_timeout_drops;
}

void Link_ParserReset(void)
{
    s_state = ST_SOF;
    s_idx = 0u;
    s_raw_len = 0u;
    s_crc_rx = 0u;
    memset(&s_acc, 0, sizeof(s_acc));
}

/* 把已接受的字节追加进原始镜像（不含 CRC 两字节）*/
static void raw_push(uint8_t b)
{
    if (s_raw_len < (uint16_t)sizeof(s_raw))
    {
        s_raw[s_raw_len++] = b;
    }
}

int Link_FeedByte(uint8_t byte, uint32_t now_ms, link_frame_t *out)
{
    /* ---- 帧间超时保护 ----
     * 若正在拼帧却已超时，说明之前遇到的是假 SOF（噪声），
     * 立刻丢弃半帧，并把当前字节当作新的起点处理。
     * 这一步能避免"假 SOF + 大 LEN 吞掉紧随其后的真帧"。
     */
    if (s_state != ST_SOF && s_have_last &&
        (uint32_t)(now_ms - s_last_byte_ms) > LINK_INTERBYTE_TIMEOUT_MS)
    {
        s_timeout_drops++;
        s_bad++;
        Link_ParserReset();
    }

    s_last_byte_ms = now_ms;
    s_have_last = 1u;

    switch (s_state)
    {
    case ST_SOF:
        if (byte == LINK_SOF)
        {
            Link_ParserReset();
            raw_push(byte);
            s_state = ST_TYPE;
        }
        /* 非 SOF：正常的重同步过程，不计入统计 */
        return 0;

    case ST_TYPE:
        s_acc.type = byte;
        raw_push(byte);
        s_state = ST_SEQ_L;
        return 0;

    case ST_SEQ_L:
        s_acc.seq = byte;
        raw_push(byte);
        s_state = ST_SEQ_H;
        return 0;

    case ST_SEQ_H:
        s_acc.seq = (uint16_t)(s_acc.seq | ((uint16_t)byte << 8));
        raw_push(byte);
        s_state = ST_OP;
        return 0;

    case ST_OP:
        s_acc.op = byte;
        raw_push(byte);
        s_state = ST_LEN;
        return 0;

    case ST_LEN:
        if (byte > LINK_MAX_PAYLOAD)
        {
            s_bad++;
            s_drop++;
            Link_ParserReset();
            return -1;
        }
        s_acc.len = byte;
        s_idx = 0u;
        raw_push(byte);
        s_state = (byte == 0u) ? ST_CRC_L : ST_PAYLOAD;
        return 0;

    case ST_PAYLOAD:
        if (s_idx < LINK_MAX_PAYLOAD)
        {
            s_acc.payload[s_idx] = byte;
        }
        raw_push(byte);
        s_idx++;
        if (s_idx >= s_acc.len)
        {
            s_state = ST_CRC_L;
        }
        return 0;

    case ST_CRC_L:
        s_crc_rx = byte;
        s_state = ST_CRC_H;
        return 0;

    case ST_CRC_H:
    {
        uint16_t crc_calc;

        s_crc_rx = (uint16_t)(s_crc_rx | ((uint16_t)byte << 8));
        crc_calc = Link_Crc16(s_raw, s_raw_len);

        if (crc_calc != s_crc_rx)
        {
            s_bad++;
            Link_ParserReset();
            return -1;
        }

        if (out != 0)
        {
            *out = s_acc;
        }
        s_good++;
        Link_ParserReset();
        return 1;
    }

    default:
        Link_ParserReset();
        return -1;
    }
}
