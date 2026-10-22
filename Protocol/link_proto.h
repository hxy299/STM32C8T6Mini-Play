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
    uint8_t  payload[200];
} link_frame_t;

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

/* ---------- 逐字节解析状态机 ---------- */
void Link_ParserReset(void);

/**
  * @brief  喂一个字节
  * @param  byte    输入字节
  * @param  now_ms  当前毫秒时间（用于帧间超时判定）
  * @param  out     解析成功时写入完整帧
  * @retval 1 = 得到一帧；0 = 还需要更多字节；-1 = 丢弃了一个坏帧（可继续）
  *
  * @note   进入本函数时会先做帧间超时检查：若距上一次字节超过
  *         LINK_INTERBYTE_TIMEOUT_MS，则丢弃半帧再处理本字节。
  *         这是防止"假 SOF + 大 LEN 吞掉后续真帧"的关键。
  */
int Link_FeedByte(uint8_t byte, uint32_t now_ms, link_frame_t *out);

/* 半帧超时检查（可在主循环周期调用，用于统计"因超时丢弃半帧"的次数）*/
uint32_t Link_GetTimeoutDrops(void);

/* ---------- 统计（用于诊断）---------- */
uint32_t Link_GetGoodFrames(void);
uint32_t Link_GetBadFrames(void);      /* CRC 错 / 长度非法 */
uint32_t Link_GetByteDrops(void);      /* 因超长被丢弃 */

#endif /* __LINK_PROTO_H */
