/**
  ******************************************************************************
  * @file    Hardware/tts_math.h
  * @brief   TTS 环形缓冲的纯逻辑（无寄存器、无 HAL，可在主机上单测）
  *
  * 单独抽出来的理由：
  *   环形缓冲的回绕与"够不够放"判断是最容易写错、又最难在上板时看出来的
  *   地方 —— 出错表现为偶发丢字符或整句吞掉，靠串口日志几乎无法定位。
  *   把这几个函数做成纯函数，就能在 PC 上遍历所有 head/tail 组合做穷举。
  ******************************************************************************
  */
#ifndef __TTS_MATH_H
#define __TTS_MATH_H

#include <stdint.h>

#ifndef TTS_MATH_INLINE
#define TTS_MATH_INLINE static
#endif

/**
  * @brief  环内已占用字节数
  * @note   假设 head/tail 都在 [0, size) 内，size 为 2 的幂。
  *         空环（head == tail）返回 0。
  */
TTS_MATH_INLINE int tts_ring_used(uint16_t head, uint16_t tail, uint16_t size)
{
    if (size == 0u)
    {
        return 0;
    }
    return (head >= tail) ? (int)(head - tail)
                          : (int)((uint32_t)size - tail + head);
}

/**
  * @brief  环内剩余空间
  * @note   固定留 1 个字节空位来区分"满"和"空"，所以最大可用 = size-1。
  */
TTS_MATH_INLINE int tts_ring_free(uint16_t head, uint16_t tail, uint16_t size)
{
    return (int)size - 1 - tts_ring_used(head, tail, size);
}

/**
  * @brief  能否一次放进 n 个字节
  * @note   ★ 必须是"整帧要么全进、要么全不进"。
  *         允许部分入队会把 TW-TTS 数据流截断成半帧，
  *         模块侧表现为念到一半停住，且无法自愈。
  */
TTS_MATH_INLINE int tts_ring_fits(uint16_t head, uint16_t tail, uint16_t size,
                                  uint16_t n)
{
    return (tts_ring_free(head, tail, size) >= (int)n) ? 1 : 0;
}

/**
  * @brief  下标前进一步并回绕
  * @note   size == 0 属于非法配置，一律返回 0（不让调用方拿到越界下标）
  */
TTS_MATH_INLINE uint16_t tts_ring_advance(uint16_t idx, uint16_t size)
{
    uint16_t n;

    if (size == 0u)
    {
        return 0u;
    }

    n = (uint16_t)(idx + 1u);
    if (n >= size)
    {
        n = 0u;
    }
    return n;
}

/**
  * @brief  TW-TTS 文本帧的总长度（含 5 字节帧头）
  * @param  text_len  正文的 UTF-8 字节数
  * @return 总长度；参数非法返回 0
  *
  * 帧格式（与 A7Z 侧 uart_tts.cxx 的 tts_text_frame 完全一致）：
  *   0xFD | len_hi | len_lo | 0x01 | 0x04(UTF-8) | 正文...
  *   其中 len = 正文长度 + 2
  */
TTS_MATH_INLINE uint16_t tts_text_frame_len(uint16_t text_len, uint16_t max_text)
{
    if ((text_len == 0u) || (text_len > max_text))
    {
        return 0u;
    }
    return (uint16_t)(text_len + 5u);
}

/**
  * @brief  判断一帧是不是 TW-TTS 原始帧
  * @note   TW-TTS 所有帧都以 0xFD 开头；
  *         不以 0xFD 开头的负载按"纯 UTF-8 正文"处理（由 MCU 自己组帧）。
  */
TTS_MATH_INLINE int tts_is_raw_frame(uint8_t first_byte)
{
    return (first_byte == 0xFDu) ? 1 : 0;
}

#endif /* __TTS_MATH_H */
