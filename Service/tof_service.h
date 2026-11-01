/**
  ******************************************************************************
  * @file    Service/tof_service.h
  * @brief   TOF 保护服务：桌沿 / 障碍判定、故障联动、自动回退
  *
  * 分层：
  *   Hardware/tof.c       只管把距离读出来
  *   Hardware/tof_math.h  纯判定（可在 PC 上单测）
  *   本文件              把两者接起来，并联动故障 / 事件 / 运动闸门
  ******************************************************************************
  */
#ifndef __TOF_SERVICE_H
#define __TOF_SERVICE_H

#include <stdint.h>

/** @brief 初始化：按配置挑出启用的传感器、建判定器、下发给驱动 */
void     TofSvc_Init(void);

/** @brief 主循环调用：驱动轮询 + 判定 + 故障联动 + 周期上报 */
void     TofSvc_Poll(uint32_t now_ms);

/**
  * @brief  该方向的运动是否允许（MOVE 分发前的闸门）
  * @param  dir MOVE_DIR_xxx
  * @return 1 = 允许
  * @note   桌沿故障期间只允许后退；障碍故障期间只禁止前进。
  */
uint8_t  TofSvc_MotionAllowed(uint8_t dir);

/** @brief 是否正在执行桌沿自动回退（安全联锁要放行它）*/
uint8_t  TofSvc_IsBackoffActive(void);

/** @brief 桌沿已锁存的传感器位图（bit n）*/
uint8_t  TofSvc_GetEdgeLatchMask(void);

/** @brief 障碍是否正处于触发状态 */
uint8_t  TofSvc_IsObstacleActive(void);

/** @brief 清除桌沿故障（CLEAR_FAULT bit0）*/
void     TofSvc_ClearEdge(void);

/** @brief 清除障碍故障（CLEAR_FAULT bit1；障碍消失也会自动清）*/
void     TofSvc_ClearObstacle(void);

/** @brief 用户主动 STOP：放弃自动回退标记 */
void     TofSvc_OnUserStop(void);

/** @brief 是否已完成过至少一轮完整扫描（没扫完之前不要采信判定结果）*/
uint8_t  TofSvc_IsReady(void);

#endif /* __TOF_SERVICE_H */
