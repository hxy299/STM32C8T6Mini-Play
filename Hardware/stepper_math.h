/**
  ******************************************************************************
  * @file    Hardware/stepper_math.h
  * @brief   步进电机的纯整数算法（无硬件依赖，可在主机上测试）
  *
  * 抽出来的原因：这几段算错会直接导致丢步、方向错、距离差。
  *   - 8 拍相序的相位推进（含负方向 mod 8）
  *   - 步频 -> 定时器 ARR
  *   - 减速所需步数（梯形速度曲线的关键）
  *   - 半步 -> 距离
  ******************************************************************************
  */
#ifndef __STEPPER_MATH_H
#define __STEPPER_MATH_H

#include <stdint.h>
#include "app_config.h"

/**
  * @brief  8 拍半步相序表
  * @note   bit0=IN1(相A) bit1=IN2(相B) bit2=IN3(相C) bit3=IN4(相D)
  *         0x01 A | 0x03 AB | 0x02 B | 0x06 BC | 0x04 C | 0x0C CD | 0x08 D | 0x09 DA
  *         每一位对应 ULN2003 一个输入；写 1 = 该相通电。
  */
static const uint8_t STEPPER_HALF_SEQ[8] =
{
    0x01u, 0x03u, 0x02u, 0x06u, 0x04u, 0x0Cu, 0x08u, 0x09u
};

/* 4 拍全步：两相始终通电（AB / BC / CD / DA）
 * 2038 步/圈，同样步频下速度是半步的 2 倍，扭矩也更大。
 * 代价：分辨率减半、噪音略大。
 */
static const uint8_t STEPPER_FULL_SEQ[4] =
{
    0x03u, 0x06u, 0x0Cu, 0x09u
};

/* 当前模式下一圈有多少步（全步 2038 / 半步 4076） */
static inline uint32_t stepper_math_steps_per_rev(void)
{
#if STEPPER_FULLSTEP_EN
    return STEPPER_FULLSTEPS_PER_REV;
#else
    return STEPPER_HALFSTEPS_PER_REV;
#endif
}

/* 当前模式的相位掩码：全步 3（4 拍），半步 7（8 拍） */
static inline uint8_t stepper_math_phase_mask(void)
{
#if STEPPER_FULLSTEP_EN
    return 3u;
#else
    return 7u;
#endif
}

/* 当前模式下按拍号取相位掩码 */
static inline uint8_t stepper_math_phase_bits(uint8_t beat)
{
#if STEPPER_FULLSTEP_EN
    return STEPPER_FULL_SEQ[beat & 3u];
#else
    return STEPPER_HALF_SEQ[beat & 7u];
#endif
}

/**
  * @brief  相位推进
  * @param  phase 当前拍号 0..7
  * @param  dir   >0 正转，<0 反转
  * @return 新拍号 0..7
  * @note   负方向用 +7 再 &7 实现 -1 mod 8（避免 C 的负数取模为负）
  */
static inline uint8_t stepper_math_next_phase(uint8_t phase, int dir)
{
    if (dir >= 0)
    {
        return (uint8_t)((phase + 1u) & 7u);
    }
    return (uint8_t)((phase + 7u) & 7u);
}

/**
  * @brief  步频 -> 定时器 ARR（PSC 已使定时器计数频率 = 1 MHz）
  * @return ARR 值（写入寄存器时应为 arr - 1，本函数直接返回该值）
  */
static inline uint16_t stepper_math_hz_to_arr(uint16_t hz)
{
    uint32_t arr;

    if (hz < STEPPER_HZ_MIN) { hz = STEPPER_HZ_MIN; }
    if (hz > STEPPER_HZ_MAX) { hz = STEPPER_HZ_MAX; }

    arr = STEPPER_TIM_TICK_HZ / (uint32_t)hz;
    if (arr < 2u)     { arr = 2u; }
    if (arr > 65535u) { arr = 65535u; }

    return (uint16_t)(arr - 1u);
}

/**
  * @brief  从当前步频减速到停止大约需要多少步
  * @note   v [step/s]，a [Hz/ms] = v 每秒变化 a*1000 [step/s²]
  *         停止时间 t = v / (a*1000) 秒
  *         平均速度 v/2，故 步数 = (v/2) * t = v² / (2000a)
  */
static inline uint32_t stepper_math_decel_steps(uint16_t hz)
{
    return ((uint32_t)hz * (uint32_t)hz)
           / (2000u * (uint32_t)STEPPER_ACCEL_HZ_PER_MS);
}

/**
  * @brief  半步数 -> 距离，单位 0.1 mm
  * @note   mm10 = |steps| * D * π / HALFSTEPS_PER_REV * 10
  *         π 取 31416/10000 避免浮点
  */
static inline uint32_t stepper_math_steps_to_mm10(int32_t halfsteps)
{
    uint32_t s = (halfsteps >= 0) ? (uint32_t)halfsteps
                                  : (uint32_t)(-halfsteps);
    uint32_t num = s * (uint32_t)TRACK_WHEEL_DIAMETER_MM * 31416u;

    return num / (stepper_math_steps_per_rev() * 1000u);
}

/**
  * @brief  速度档 -> 步频（Hz）
  */
static inline uint16_t stepper_math_speed_to_hz(uint8_t speed)
{
    switch (speed)
    {
    case STEPPER_SPEED_0: return STEPPER_START_HZ;
    case STEPPER_SPEED_1: return STEPPER_HZ_SLOW;
    case STEPPER_SPEED_2: return STEPPER_HZ_NORMAL;
    case STEPPER_SPEED_3: return STEPPER_HZ_FAST;
    default:              return STEPPER_HZ_NORMAL;
    }
}

#endif /* __STEPPER_MATH_H */
