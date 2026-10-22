/**
  ******************************************************************************
  * @file    Hardware/serial.h
  * @brief   USART1 与 A7Z 的通信链路（PA9=TX / PA10=RX，115200 8N1）
  *
  * 设计：
  *   - 接收：USART1 RXNE 中断 -> 环形缓冲（只做搬运，不做解析）
  *   - 解析：主循环从环形缓冲取字节，喂给 Protocol/link_proto.c 的状态机
  *   - 发送：阻塞发送（ACK/PONG 等小帧足够）；后续 TTS 大帧再换环形缓冲
  *
  * 参考：旧小车工程 Hardware/Serial.c（改为 115200 + 二进制帧）
  ******************************************************************************
  */
#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>

/* ---------- 初始化 ---------- */
void    Serial_Init(void);

/* ---------- 发送（阻塞）---------- */
void    Serial_SendByte(uint8_t b);
void    Serial_SendBuffer(const uint8_t *buf, uint16_t len);
int     Serial_Printf(const char *fmt, ...);   /* 调试用，会阻塞 */

/* ---------- 接收（从环形缓冲取）---------- */
/* 返回 1 表示取到一个字节，0 表示暂时没有数据 */
uint8_t Serial_ReadByte(uint8_t *b);

/* 返回环形缓冲里当前可读字节数 */
uint16_t Serial_Available(void);

/* ---------- 统计 ---------- */
uint32_t Serial_GetRxOverflowCount(void);   /* 环形缓冲溢出次数 */
uint32_t Serial_GetRxTotalCount(void);      /* 累计收到字节数 */

#endif /* __SERIAL_H */
