/**
  ******************************************************************************
  * @file    Service/tof_service.c
  * @brief   TOF 保护服务实现
  ******************************************************************************
  */
#include "stm32f10x.h"
#include "app_config.h"
#include "tof.h"
#include "tof_math.h"
#include "tof_service.h"
#include "config.h"
#include "link_service.h"
#include "stepper.h"
#include "bsp.h"

/* ============================ 内部状态 ============================ */
static tof_guard_t s_edge[TOF_EDGE_NUM];
static tof_guard_t s_obstacle;

static uint16_t s_last_seq[TOF_COUNT];      /* 上层已消费到的样本序号 */
static uint8_t  s_edge_latch_mask = 0u;     /* 已上报过的桌沿传感器位图 */
static uint8_t  s_obstacle_latched = 0u;
static uint8_t  s_backoff_active  = 0u;
static uint32_t s_last_data_ms    = 0u;
static uint8_t  s_ready           = 0u;

/* ============================ 初始化 ============================ */

void TofSvc_Init(void)
{
    uint8_t mask = 0u;
    uint8_t i;

    s_edge_latch_mask  = 0u;
    s_obstacle_latched = 0u;
    s_backoff_active   = 0u;
    s_last_data_ms     = 0u;
    s_ready            = 0u;

    for (i = 0u; i < TOF_COUNT; i++)
    {
        s_last_seq[i] = 0u;
        if ((Config_Get() & TOF_CFG_BIT(i)) != 0u)
        {
            mask = (uint8_t)(mask | (1u << i));
        }
    }

    /* 判定器一律先建好（即使对应传感器没启用，留着也无害）*/
    for (i = 0u; i < TOF_EDGE_NUM; i++)
    {
        tof_math_guard_init(&s_edge[i], TOF_EDGE_MM,
                            TOF_EDGE_RECOVER_MM, TOF_EDGE_DEBOUNCE);
    }
    tof_math_guard_init(&s_obstacle, TOF_OBSTACLE_MM,
                        TOF_OBSTACLE_RECOVER_MM, TOF_OBSTACLE_DEBOUNCE);

    if (mask == 0u)
    {
        /* 一颗都没启用：盲走模式由 link_service 处理，这里什么都不做 */
        return;
    }

    Tof_Init(mask);

    if (Tof_GetOnlineMask() == 0u)
    {
        /* 启用了一颗都没上线 -> 初始化失败，锁存故障要人工确认 */
        LinkSvc_SetFault(FAULT_TOF_INIT);
        return;
    }

    s_ready = 1u;
}

/* ============================ 周期上报 ============================ */

static void tof_send_data_event(void)
{
    uint8_t p[9];
    uint8_t i;
    uint8_t mask = 0u;

    for (i = 0u; i < TOF_COUNT; i++)
    {
        const tof_dev_t *d = Tof_GetDev(i);
        int32_t mm = 0;

        if ((d != 0) && (d->online != 0u))
        {
            mask = (uint8_t)(mask | (1u << i));
            if (d->valid != 0u)
            {
                mm = d->mm;
                if (mm > 8191)
                {
                    mm = 8191;
                }
            }
            else
            {
                mm = -1;    /* 无效读数用 0xFFFF 表示 */
            }
        }

        p[1u + (i * 2u)] = (uint8_t)(((uint16_t)mm >> 8) & 0xFFu);
        p[2u + (i * 2u)] = (uint8_t)((uint16_t)mm & 0xFFu);
    }
    p[0] = mask;

    LinkSvc_SendEvent(OP_EVT_TOF_DATA, p, 9u);
}

/* ============================ 主循环 ============================ */

void TofSvc_Poll(uint32_t now_ms)
{
    uint8_t i;
    uint8_t edge_hit  = 0u;
    uint8_t edge_new  = 0u;
    uint8_t new_sensor = 0u;
    int32_t new_mm    = 0;

    if (s_ready == 0u)
    {
        return;
    }

    Tof_Poll(now_ms);

    /* 自动回退跑完就放开标记 */
    if ((s_backoff_active != 0u) && (Stepper_IsAnyBusy() == 0u))
    {
        s_backoff_active = 0u;
    }

    /* ---------- 底部三颗：桌沿 ---------- */
    for (i = 0u; i < TOF_EDGE_NUM; i++)
    {
        const tof_dev_t *d = Tof_GetDev(i);

        if ((d == 0) || (d->enabled == 0u) || (d->online == 0u))
        {
            continue;
        }

        /* ★ 只在有新样本时喂判定器，否则去抖计数会在主循环里瞬间跑满 */
        if (d->sample_seq != s_last_seq[i])
        {
            s_last_seq[i] = d->sample_seq;
            (void)tof_math_guard_edge(&s_edge[i], d->valid, d->mm);
        }

        if (s_edge[i].active != 0u)
        {
            edge_hit = 1u;

            if ((s_edge_latch_mask & (uint8_t)(1u << i)) == 0u)
            {
                s_edge_latch_mask = (uint8_t)(s_edge_latch_mask | (1u << i));
                edge_new  = 1u;
                new_sensor = i;
                new_mm     = d->mm;
            }
        }
        else
        {
            s_edge_latch_mask = (uint8_t)(s_edge_latch_mask & (uint8_t)~(1u << i));
        }
    }

    if (edge_new != 0u)
    {
        uint8_t p[3];

        p[0] = new_sensor;
        p[1] = (uint8_t)(((uint32_t)new_mm >> 8) & 0xFFu);
        p[2] = (uint8_t)((uint32_t)new_mm & 0xFFu);
        LinkSvc_SendEvent(OP_EVT_EDGE_DETECTED, p, 3u);

        /* 立即反向回退一小段（说明书 §5.3.1）*/
        if (s_backoff_active == 0u)
        {
            s_backoff_active = 1u;
            (void)LinkSvc_RequestMove(MOVE_DIR_BACKWARD,
                                      TOF_EDGE_BACKOFF_HZ,
                                      (uint16_t)TOF_EDGE_BACKOFF_STEPS);
        }
    }

    if (edge_hit != 0u)
    {
        LinkSvc_SetFault(FAULT_EDGE);
    }
    else
    {
        LinkSvc_ClearFaultCode(FAULT_EDGE);
    }

    /* ---------- 前脸一颗：障碍 ---------- */
    {
        const tof_dev_t *d = Tof_GetDev(TOF_EDGE_NUM);

        if ((d != 0) && (d->enabled != 0u) && (d->online != 0u))
        {
            if (d->sample_seq != s_last_seq[TOF_EDGE_NUM])
            {
                s_last_seq[TOF_EDGE_NUM] = d->sample_seq;
                (void)tof_math_guard_obstacle(&s_obstacle, d->valid, d->mm);
            }

            if (s_obstacle.active != 0u)
            {
                if (s_obstacle_latched == 0u)
                {
                    uint8_t p[2];
                    s_obstacle_latched = 1u;
                    p[0] = (uint8_t)(((uint32_t)d->mm >> 8) & 0xFFu);
                    p[1] = (uint8_t)((uint32_t)d->mm & 0xFFu);
                    LinkSvc_SendEvent(OP_EVT_OBSTACLE_DETECTED, p, 2u);
                }
                LinkSvc_SetFault(FAULT_OBSTACLE);
            }
            else
            {
                s_obstacle_latched = 0u;
                LinkSvc_ClearFaultCode(FAULT_OBSTACLE);
            }
        }
    }

    /* ---------- 周期上报距离 ---------- */
    if ((uint32_t)(now_ms - s_last_data_ms) >= TOF_DATA_PERIOD_MS)
    {
        s_last_data_ms = now_ms;
        tof_send_data_event();
    }
}

/* ============================ 运动闸门 ============================ */

uint8_t TofSvc_MotionAllowed(uint8_t dir)
{
    uint8_t i;
    uint8_t edge = 0u;

    if (s_ready == 0u)
    {
        return 1u;      /* 未启用 TOF：不拦（盲走模式另有闸门）*/
    }

    for (i = 0u; i < TOF_EDGE_NUM; i++)
    {
        if (s_edge[i].active != 0u)
        {
            edge = 1u;
        }
    }

    if (edge != 0u)
    {
        /* 桌沿故障：只允许后退脱困，禁止前进与转向 */
        return (uint8_t)((dir == MOVE_DIR_BACKWARD) ? 1u : 0u);
    }

    if ((s_obstacle.active != 0u) && (dir == MOVE_DIR_FORWARD))
    {
        return 0u;
    }

    return 1u;
}

/* ============================ 状态与清除 ============================ */

uint8_t TofSvc_IsBackoffActive(void)
{
    return s_backoff_active;
}

uint8_t TofSvc_GetEdgeLatchMask(void)
{
    return s_edge_latch_mask;
}

uint8_t TofSvc_IsObstacleActive(void)
{
    return s_obstacle.active;
}

uint8_t TofSvc_IsReady(void)
{
    return s_ready;
}

void TofSvc_ClearEdge(void)
{
    uint8_t i;

    for (i = 0u; i < TOF_EDGE_NUM; i++)
    {
        tof_math_guard_reset(&s_edge[i]);
    }
    s_edge_latch_mask = 0u;
    s_backoff_active  = 0u;
    LinkSvc_ClearFaultCode(FAULT_EDGE);
}

void TofSvc_ClearObstacle(void)
{
    tof_math_guard_reset(&s_obstacle);
    s_obstacle_latched = 0u;
    LinkSvc_ClearFaultCode(FAULT_OBSTACLE);
}

void TofSvc_OnUserStop(void)
{
    s_backoff_active = 0u;
}
