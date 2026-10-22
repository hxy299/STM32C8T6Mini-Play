/**
  ******************************************************************************
  * @file    Hardware/servo.h
  * @brief   M3：SG90 舵机（PA0 / TIM2_CH1 / 50 Hz）
  *
  * 关键约束（见项目说明书 §5.1）：
  *   - 角度【硬限幅】±45°，即使上位机发 180° 也必须钳位并上报 clamped
  *   - 受 CFG_SERVO_EN 控制：关闭时不初始化 TIM2、不输出 PWM、命令返回 NAK
  *   - 缓动由 1 ms 调度驱动（Servo_Update），不得阻塞主循环
  ******************************************************************************
  */
#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

/* 返回值 */
#define SERVO_OK            0
#define SERVO_ERR_DISABLED -1      /* 功能未启用（CFG_SERVO_EN=0） */
#define SERVO_ERR_PARAM    -2
#define SERVO_ERR_NOTIM    -3

/* ---------- 初始化 ----------
 * 若 CFG_SERVO_EN=0，本函数直接返回 SERVO_ERR_DISABLED 且不碰任何寄存器。
 * 返回 SERVO_OK 表示 TIM2/PA0 已就绪并开始输出中位脉宽。
 */
int      Servo_Init(void);

/* ---------- 运行/停止（用于运行时开关切换） ---------- */
int      Servo_Enable(void);
void     Servo_Disable(void);
uint8_t  Servo_IsEnabled(void);

/* ---------- 角度控制 ----------
 * deg : -45..+45（超限会被钳位）
 * ms  : 缓动时长；0 = 立即到位；>0 = 线性插值（以 SERVO_EASE_TICK_MS 为步）
 * 返回 : SERVO_OK / SERVO_ERR_DISABLED
 * clamped 输出：1 表示入参被钳位过
 */
int      Servo_MoveTo(int8_t deg, uint16_t ms, uint8_t *clamped);

/* ---------- 1 ms 调度调用 ---------- */
void     Servo_Update(uint32_t now_ms);

/* ---------- 查询 ---------- */
int8_t   Servo_GetCurrentDeg(void);     /* 当前（含缓动中间态）角度 */
int8_t   Servo_GetTargetDeg(void);
uint16_t Servo_DegToPulseUs(int8_t deg);
uint16_t Servo_GetPulseUs(void);
uint32_t Servo_GetMoveCount(void);

#endif /* __SERVO_H */
