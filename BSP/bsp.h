/**
  ******************************************************************************
  * @file    BSP/bsp.h
  * @brief   板级支撑：LED / 看门狗 / TIM3 1ms 调度时基 / 时钟自检
  *
  * 约定：
  *   - 所有 GPIO / 定时器 / 看门狗操作集中在本模块，其它模块不直接碰寄存器
  *   - 1 ms 调度时基用 TIM3（不用 SysTick，原因见说明书 §13.1）
  ******************************************************************************
  */
#ifndef __BSP_H
#define __BSP_H

#include <stdint.h>
#include "stm32f10x.h"

/* ---------- 初始化 ---------- */
void     BSP_NvicInit(void);      /* 优先级分组，必须最先调用一次 */
void     BSP_LedInit(void);
void     BSP_IwdgInit(void);      /* 一旦启动不可关闭 */
void     BSP_IwdgFeed(void);
void     BSP_TickInit(void);

/* ---------- 时钟 ---------- */
uint32_t BSP_GetSysClock(void);
uint8_t  BSP_ClockIsAsExpected(uint32_t expected_hz);

/* ---------- 调度时基 ---------- */
uint32_t BSP_GetTickMs(void);          /* 上电以来的毫秒数（溢出约 49 天）*/

/* ---------- LED ---------- */
void     BSP_LedOn(void);
void     BSP_LedOff(void);
void     BSP_LedToggle(void);

/* LED 提示模式：便于上板时一眼分辨状态 */
typedef enum
{
    BSP_LED_PATTERN_OK = 0,   /* 1 Hz 慢闪：已与上位机正常通信 */
    BSP_LED_PATTERN_WAIT,     /* 双闪 + 长停：上电后从未收到过上位机帧 */
    BSP_LED_PATTERN_FAULT     /* 5 Hz 快闪：曾通信过又丢失，或其它故障 */
} bsp_led_pattern_t;

void     BSP_LedRunPattern(uint32_t now_ms, bsp_led_pattern_t pattern);

/* 兼容用：固定周期翻转 */
void     BSP_LedHeartbeatTask(uint32_t now_ms, uint32_t period_ms);

#endif /* __BSP_H */
