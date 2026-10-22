/**
  ******************************************************************************
  * @file    Hardware/servo_math.h
  * @brief   舵机角度 <-> 脉宽 的纯整数换算（无硬件依赖，可在主机上测试）
  *
  * 抽出来的目的：这是最容易算错、也最该被测试覆盖的一段。
  * 一旦角度映射错了，舵机会撞限位、堵转、烧齿轮。
  *
  * 依赖 app_config.h 里的：
  *   SERVO_CENTER_US      中位脉宽（默认 1500）
  *   SERVO_HALF_RANGE_US  ±MAX_DEG 对应的脉宽偏移（默认 500）
  *   SERVO_MAX_DEG / SERVO_MIN_DEG
  ******************************************************************************
  */
#ifndef __SERVO_MATH_H
#define __SERVO_MATH_H

#include <stdint.h>
#include "app_config.h"

/* 限幅（返回钳位后的角度，clamped 置 1 表示发生过钳位） */
static inline int8_t servo_math_clamp(int16_t deg, uint8_t *clamped)
{
    if (deg > SERVO_MAX_DEG)
    {
        if (clamped) { *clamped = 1u; }
        return (int8_t)SERVO_MAX_DEG;
    }
    if (deg < SERVO_MIN_DEG)
    {
        if (clamped) { *clamped = 1u; }
        return (int8_t)SERVO_MIN_DEG;
    }
    return (int8_t)deg;
}

/**
  * @brief  角度 -> 脉宽（微秒）
  * @note   公式：CENTER + deg * HALF_RANGE / MAX_DEG
  *           deg = +45 -> 1500 + 500  = 2000 us
  *           deg =   0 -> 1500 us
  *           deg = -45 -> 1500 - 500  = 1000 us
  *         输入先钳位，保证输出永远落在 [CENTER-HALF, CENTER+HALF]。
  */
static inline uint16_t servo_math_deg_to_us(int16_t deg)
{
    int8_t  d = servo_math_clamp(deg, 0);
    int32_t delta = ((int32_t)d * (int32_t)SERVO_HALF_RANGE_US)
                    / (int32_t)SERVO_MAX_DEG;

    return (uint16_t)((int32_t)SERVO_CENTER_US + delta);
}

#endif /* __SERVO_MATH_H */
