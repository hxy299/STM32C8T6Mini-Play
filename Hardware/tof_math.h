/**
  ******************************************************************************
  * @file    Hardware/tof_math.h
  * @brief   TOF 保护判定的纯函数（不碰寄存器，可在 PC 上用 gcc 单测）
  *
  * ★ 这里是整个 TOF 阶段最容易写错的地方：
  *   **底部桌沿传感器和前脸障碍传感器的"危险方向"是相反的。**
  *
  *   底部（朝下测桌面）：
  *       桌面上   距离 ≈ 底盘离桌高度（15~40 mm）
  *       到桌沿   正下方没有桌面 -> 距离突然【变远】或者直接无回波
  *       ⇒ 危险 = 距离变大 / 读数无效
  *
  *   前脸（朝前测障碍）：
  *       前方空旷 距离大（>300 mm）或超量程（无回波）
  *       遇到障碍 距离【变近】
  *       ⇒ 危险 = 距离变小；而"无回波"是【安全】的
  *
  * 如果把两者的无效读数处理写成同一套，会出现两种严重后果：
  *   - 底部按"无效=安全"处理  -> 冲到桌沿时读数无效，不报警，直接掉下去
  *   - 前脸按"无效=危险"处理  -> 前方空旷永远无回波，机器人永远动不了
  ******************************************************************************
  */
#ifndef __TOF_MATH_H
#define __TOF_MATH_H

#include <stdint.h>

/* 判定器状态。字段全部显式，便于单测逐个检查 */
typedef struct
{
    int32_t trig_mm;        /* 触发阈值 */
    int32_t clear_mm;       /* 解除阈值（回滞，避免在阈值附近抖动）*/
    uint8_t confirm_n;      /* 需要连续确认的次数 */
    uint8_t confirm_cnt;    /* 当前连续危险计数 */
    uint8_t clear_cnt;      /* 当前连续安全计数 */
    uint8_t active;         /* 1 = 已判定为危险 */
} tof_guard_t;

/**
  * @brief  初始化判定器
  * @param  confirm_n  连续确认次数（0 会被当成 1）
  */
static inline void tof_math_guard_init(tof_guard_t *g, int32_t trig_mm,
                                       int32_t clear_mm, uint8_t confirm_n)
{
    g->trig_mm     = trig_mm;
    g->clear_mm    = clear_mm;
    g->confirm_n   = (confirm_n == 0u) ? 1u : confirm_n;
    g->confirm_cnt = 0u;
    g->clear_cnt   = 0u;
    g->active      = 0u;
}

/**
  * @brief  恢复到"未触发"初态（阈值配置保留）
  * @note   解除已锁存的故障时调用，避免计数残留导致立刻又触发一次
  */
static inline void tof_math_guard_reset(tof_guard_t *g)
{
    g->confirm_cnt = 0u;
    g->clear_cnt   = 0u;
    g->active      = 0u;
}

/**
  * @brief  底部桌沿判定：距离【变远】或无回波 = 危险
  * @param  valid  1 = 本次读数有效
  * @return 1 = 当前处于危险状态
  */
static inline uint8_t tof_math_guard_edge(tof_guard_t *g,
                                          uint8_t valid, int32_t mm)
{
    uint8_t danger = (uint8_t)((valid == 0u) || (mm > g->trig_mm));

    if (danger)
    {
        g->clear_cnt = 0u;
        if (g->confirm_cnt < 255u)
        {
            g->confirm_cnt++;
        }
        if (g->confirm_cnt >= g->confirm_n)
        {
            g->active = 1u;
        }
    }
    else
    {
        g->confirm_cnt = 0u;

        if (g->active != 0u)
        {
            /* 必须回到 clear_mm 以下才解除（回滞） */
            if (valid != 0u && mm <= g->clear_mm)
            {
                if (g->clear_cnt < 255u)
                {
                    g->clear_cnt++;
                }
                if (g->clear_cnt >= g->confirm_n)
                {
                    g->active    = 0u;
                    g->clear_cnt = 0u;
                }
            }
            else
            {
                g->clear_cnt = 0u;
            }
        }
    }
    return g->active;
}

/**
  * @brief  前脸障碍判定：距离【变近】= 危险；无回波 = 安全
  * @return 1 = 当前处于危险状态
  */
static inline uint8_t tof_math_guard_obstacle(tof_guard_t *g,
                                              uint8_t valid, int32_t mm)
{
    /* ⚠️ 与桌沿相反：无效读数（无回波）意味着前方空旷，是安全的 */
    uint8_t danger = (uint8_t)((valid != 0u) && (mm < g->trig_mm));

    if (danger)
    {
        g->clear_cnt = 0u;
        if (g->confirm_cnt < 255u)
        {
            g->confirm_cnt++;
        }
        if (g->confirm_cnt >= g->confirm_n)
        {
            g->active = 1u;
        }
    }
    else
    {
        g->confirm_cnt = 0u;

        if (g->active != 0u)
        {
            /* 远离到 clear_mm 以上（或干脆没回波）才解除 */
            if ((valid == 0u) || (mm >= g->clear_mm))
            {
                if (g->clear_cnt < 255u)
                {
                    g->clear_cnt++;
                }
                if (g->clear_cnt >= g->confirm_n)
                {
                    g->active    = 0u;
                    g->clear_cnt = 0u;
                }
            }
            else
            {
                g->clear_cnt = 0u;
            }
        }
    }
    return g->active;
}

/**
  * @brief  读数是否可用
  * @note   0 和过大的值（VL53L0X 无回波时常给 8190）都算无效
  */
static inline uint8_t tof_math_range_ok(int32_t mm, int32_t max_mm)
{
    if ((mm <= 0) || (mm > max_mm) || (mm >= 8190))
    {
        return 0u;
    }
    return 1u;
}

/**
  * @brief  三个距离取中值，压掉单次跳变
  */
static inline int32_t tof_math_median3(int32_t a, int32_t b, int32_t c)
{
    if (a > b) { int32_t t = a; a = b; b = t; }
    if (b > c) { b = c; }
    if (a > b) { b = a; }
    return b;
}

#endif /* __TOF_MATH_H */
