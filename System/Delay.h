/**
  ******************************************************************************
  * @file    System/Delay.h
  * @brief   微秒/毫秒/秒级延时（SysTick 实现）
  *
  * 来源：参考文件\小车代码完整实现 寒假版\System\Delay.c
  *
  * ⚠️ 重要（见项目说明书 §13.1）：
  *   本模块**直接改写并关闭 SysTick**。因此 1 ms 调度时基绝对不能用 SysTick，
  *   本项目改用 TIM3（BSP_TickInit）。若把调度放回 SysTick，Delay_ms 会把
  *   调度中断永久关掉，表现为"LED 不闪、电机不动、但程序没死"。
  *
  * 适用范围：仅用于初始化阶段与调试日志，**不得用于实时路径**。
  ******************************************************************************
  */
#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>

void Delay_us(uint32_t xus);
void Delay_ms(uint32_t xms);
void Delay_s(uint32_t xs);

#endif /* __DELAY_H */
