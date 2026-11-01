/**
  ******************************************************************************
  * @file    Hardware/tts.h
  * @brief   M9：TW-TTS 语音模块转发（USART2，9600 8N1，中断发送）
  *
  * 职责边界：
  *   本模块只负责"把字节可靠地吐到 USART2"。
  *   协议语义（TW-TTS 帧格式）由 A7Z 负责；MCU 是转发器，
  *   额外提供一个"直接给 UTF-8 正文"的便利入口用于台架自测。
  ******************************************************************************
  */
#ifndef __TTS_H
#define __TTS_H

#include <stdint.h>

/* ---------- 初始化 / 状态 ---------- */
void     Tts_Init(void);
uint8_t  Tts_IsReady(void);          /* 1 = USART2 已初始化 */

/* ---------- 发送 ---------- */
/**
  * @brief  原样转发一段 TW-TTS 字节流（op = 0x03 的负载）
  * @return TTS_OK / TTS_ERR_NOTREADY / TTS_ERR_TOOLONG / TTS_ERR_BUSY
  * @note   仅入队，不等待发送完成。整帧要么全部入队、要么全部拒绝。
  */
int      Tts_Relay(const uint8_t *frame, uint16_t len);

/**
  * @brief  送一段 UTF-8 正文（由 MCU 按 TW-TTS 文本帧组帧）
  * @note   等价于先组 "FD len_hi len_lo 01 04 text"，再 Tts_Relay。
  */
int      Tts_SpeakUtf8(const uint8_t *text, uint16_t len);

/* ---------- 主循环 ---------- */
/**
  * @brief  兜底重启发送
  * @note   正常情况下 TXE 中断会自己把环抽干。
  *         这里只处理"环里有数据但中断停了"的异常态（例如中断被意外关闭）。
  *         每轮调用一次，代价是两次比较。
  */
void     Tts_Poll(void);

/* ---------- 诊断 ---------- */
uint32_t Tts_GetSentFrames(void);      /* 成功入队的帧数 */
uint32_t Tts_GetRejectedFrames(void);  /* 因环满被拒的帧数 */
uint32_t Tts_GetPendingBytes(void);    /* 环内还没发出去的字节数 */
uint8_t  Tts_IsBusy(void);             /* 1 = 还在发 */

#endif /* __TTS_H */
