/**
  ******************************************************************************
  * @file    Protocol/link_proto.h
  * @brief   A7Z <-> MCU 二进制帧协议：CRC16 + 组帧/拆帧状态机
  *
  * 帧格式（与上游 a733home 风格一致，见说明书 §6.2）：
  *   +0  SOF      1B  0xA5
  *   +1  TYPE     1B  0x01 命令 / 0x02 应答 / 0x03 事件
  *   +2  SEQ      2B  小端
  *   +4  OP       1B  操作码
  *   +5  LEN      1B  payload 长度 0..200
  *   +6  PAYLOAD  LEN B
  *   +6+LEN CRC16 2B  小端，CCITT-FALSE(0x1021, init 0xFFFF)，覆盖 SOF..PAYLOAD
  *
  * 设计要点：
  *   - 拆帧用逐字节状态机，靠 SOF + CRC 重新同步，坏帧不阻塞后续
  *   - 组帧函数把完整帧写进调用者提供的缓冲，返回帧长
  ******************************************************************************
  */
#ifndef __LINK_PROTO_H
#define __LINK_PROTO_H

#include <stdint.h>
/* 需要 LINK_SOF / LINK_MAX_PAYLOAD / LINK_MAX_FRAME / LINK_INTERBYTE_TIMEOUT_MS。
 * 原先只在 .c 里用到所以没引，改为实例化后 raw[] 的长度出现在头文件里，必须引。 */
#include "app_config.h"

#define LINK_OK           0
#define LINK_ERR_NULL    -1
#define LINK_ERR_TOOLONG -2
#define LINK_ERR_PARAM   -3

/* 解析出来的帧 */
typedef struct
{
    uint8_t  type;
    uint16_t seq;
    uint8_t  op;
    uint8_t  len;
    uint8_t  payload[LINK_MAX_PAYLOAD];
} link_frame_t;

/* ============================ 解析器实例 ============================ */
/**
  * @brief  拆帧状态机实例
  *
  * ★ 为什么必须是实例而不是全局静态：
  *   M6/M7 之后 MCU 要同时维护 USART1（A7Z）和 USART3（蓝牙）两条链路。
  *   如果两边共用一个状态机，A7Z 的半帧会被蓝牙的字节续上，
  *   结果两边都收不到完整帧 —— 表现为"接了蓝牙以后 A7Z 也失灵"。
  *
  *   每条链路一个实例，互不干扰。
  */
typedef struct
{
    uint8_t      state;                        /* parse_state_t */
    link_frame_t acc;                          /* 累积中的帧 */
    uint8_t      raw[LINK_MAX_FRAME];          /* 原始字节镜像（用于 CRC）*/
    uint16_t     raw_len;
    uint8_t      idx;                          /* payload 写入下标 */
    uint16_t     crc_rx;
    uint32_t     last_byte_ms;
    uint8_t      have_last;

    /* 统计 */
    uint32_t     good;
    uint32_t     bad;
    uint32_t     drop;                         /* 因超长被丢弃 */
    uint32_t     timeout_drops;                /* 因帧间超时丢弃半帧 */
} link_parser_t;

/* CRC16-CCITT-FALSE：多项式 0x1021，初值 0xFFFF，不反转 */
uint16_t Link_Crc16(const uint8_t *data, uint16_t len);

/**
  * @brief  组一帧
  * @param  out       输出缓冲，建议 >= 208 字节
  * @param  out_size  输出缓冲大小
  * @param  type/seq/op/payload/len  帧内容
  * @retval >0 帧长；<0 错误码
  */
int Link_BuildFrame(uint8_t *out, uint16_t out_size,
                    uint8_t type, uint16_t seq, uint8_t op,
                    const uint8_t *payload, uint8_t len);

/* ============================ 实例化解析 ============================ */

/** @brief 初始化一个解析器实例（必须在喂字节之前调用一次）*/
void Link_ParserInit(link_parser_t *p);

/**
  * @brief  向指定实例喂一个字节
  * @param  p       解析器实例
  * @param  byte    输入字节
  * @param  now_ms  当前毫秒时间（用于帧间超时判定）
  * @param  out     解析成功时写入完整帧
  * @retval 1 = 得到一帧；0 = 还需要更多字节；-1 = 丢弃了一个坏帧（可继续）
  *
  * @note   进入本函数时会先做帧间超时检查：若距上一次字节超过
  *         LINK_INTERBYTE_TIMEOUT_MS，则丢弃半帧再处理本字节。
  *         这是防止"假 SOF + 大 LEN 吞掉后续真帧"的关键。
  */
int Link_FeedByteTo(link_parser_t *p, uint8_t byte,
                    uint32_t now_ms, link_frame_t *out);

/* ============================ 兼容用：默认实例 ============================ */
/* 老代码与主机单测仍可用这一套（内部就是 SRC_A7Z 那个实例）。 */

void Link_ParserReset(void);

/** @brief 喂一个字节给默认实例（等价于 Link_FeedByteTo(&默认实例, ...)）*/
int Link_FeedByte(uint8_t byte, uint32_t now_ms, link_frame_t *out);

/* 半帧超时检查（可在主循环周期调用，用于统计"因超时丢弃半帧"的次数）*/
uint32_t Link_GetTimeoutDrops(void);

/* ---------- 统计（用于诊断，默认实例）---------- */
uint32_t Link_GetGoodFrames(void);
uint32_t Link_GetBadFrames(void);      /* CRC 错 / 长度非法 */
uint32_t Link_GetByteDrops(void);      /* 因超长被丢弃 */

#endif /* __LINK_PROTO_H */
