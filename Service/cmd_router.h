/**
  ******************************************************************************
  * @file    Service/cmd_router.h
  * @brief   双命令源仲裁：A7Z（USART1）与蓝牙（USART3）
  *
  * ★ 为什么需要仲裁
  *   接上蓝牙以后有两个东西能发 MOVE。如果两边都放行：
  *     - 两个源同时驱动电机，脉冲会互相覆盖，位置计数直接乱掉
  *     - 一边发"前进"另一边发"后退"，电机来回抖
  *   所以必须有且只有一个"控制权拥有者"。
  *
  * ★ 优先级设计（这是"防卡死"的另一半）
  *   SAFETY  >  QUERY  >  CONTROL
  *
  *   SAFETY 类（停止 / 清故障）**不看控制权，任何来源、任何时候都执行**。
  *   理由很现实：手动遥控器上的"停"必须永远有效，
  *   哪怕控制权在 A7Z 手上。这条如果也要仲裁，紧急情况就停不下来了。
  *
  *   QUERY 类（PING / 诊断）总是执行 —— 代价小、不改执行器状态，
  *   而且两路都要能保活。
  *
  *   CONTROL 类（运动 / 配置）只有拥有者能执行。
  *
  * ★ 控制权怎么选
  *   A7Z 在场 → A7Z 拥有（它是"大脑"，优先级天然更高）
  *   A7Z 失联 → 蓝牙接管（这就是"单独由蓝牙遥控"）
  *   两个都不在 → 无人拥有（上层会置心跳故障并停车）
  *
  *   切换瞬间上层必须停车，否则会"接着刚才的动作继续跑"。
  ******************************************************************************
  */
#ifndef __CMD_ROUTER_H
#define __CMD_ROUTER_H

#include <stdint.h>
#include "app_config.h"

/* 命令来源 */
#define CMD_SRC_A7Z     0u
#define CMD_SRC_BT      1u
#define CMD_SRC_COUNT   2u
#define CMD_SRC_NONE    0xFFu

/**
  * @brief  按操作码取优先级分类
  * @return CMD_CLASS_SAFETY / CMD_CLASS_QUERY / CMD_CLASS_CONTROL
  * @note   未知操作码归到 CONTROL：它会被 dispatch 回 NAK(UNKNOWN_OP)，
  *         不需要为它单独开一条仲裁路径。
  */
static inline uint8_t cmd_class_of(uint8_t op)
{
    switch (op)
    {
    case OP_MOTOR_STOP:
    case OP_CLEAR_FAULT:
        return CMD_CLASS_SAFETY;

    case OP_PING:
    case OP_GET_CONFIG:
    case OP_STEPPER_STAT:
    case OP_LINK_STAT:
        return CMD_CLASS_QUERY;

    default:
        return CMD_CLASS_CONTROL;
    }
}

/**
  * @brief  带载荷的分类（处理"MOVE + dir=STOP"这种"名为运动实为停止"的情况）
  * @note   MOVE dir=4 是停止命令，必须按 SAFETY 处理，
  *         否则控制权在 A7Z 时蓝牙就连"停"都发不出去。
  */
static inline uint8_t cmd_class_of_frame(uint8_t op,
                                         const uint8_t *payload,
                                         uint8_t len)
{
    if ((op == OP_MOVE) && (len >= 1u) && (payload != 0) &&
        (payload[0] == MOVE_DIR_STOP))
    {
        return CMD_CLASS_SAFETY;
    }
    return cmd_class_of(op);
}

/**
  * @brief  选出控制权拥有者
  * @param  a7z_alive  A7Z 链路是否在心跳窗口内
  * @param  bt_alive   蓝牙链路是否在心跳窗口内
  * @return CMD_SRC_A7Z / CMD_SRC_BT / CMD_SRC_NONE
  */
static inline uint8_t cmd_owner(uint8_t a7z_alive, uint8_t bt_alive)
{
    if (a7z_alive != 0u)
    {
        return CMD_SRC_A7Z;
    }
    if (bt_alive != 0u)
    {
        return CMD_SRC_BT;
    }
    return CMD_SRC_NONE;
}

/**
  * @brief  这条命令是否允许执行
  * @param  src       命令来源
  * @param  cls       已算好的优先级分类（避免重复计算）
  * @param  owner     当前控制权拥有者
  * @return 1 = 放行，0 = 应回 NAK
  */
static inline uint8_t cmd_allowed(uint8_t src, uint8_t cls, uint8_t owner)
{
    /* SAFETY / QUERY 不看控制权 */
    if (cls != CMD_CLASS_CONTROL)
    {
        return 1u;
    }

    /* CONTROL：必须由拥有者发起 */
    return (uint8_t)((src == owner) ? 1u : 0u);
}

#endif /* __CMD_ROUTER_H */
