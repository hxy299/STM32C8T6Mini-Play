/**
  ******************************************************************************
  * @file    Hardware/stepper.h
  * @brief   M4/M5：28BYJ-48 步进电机 ×2（经 ULN2003 驱动）
  *
  * 设计要点（对应项目说明书 §5.2、§5.5）：
  *   1) 8 拍半步相序，精确步数（不靠延时，靠定时器计数）
  *   2) **非阻塞**：加减速在 1 ms 任务里推进，步进在定时器中断里打拍
  *   3) 每个电机独立时基（TIM4 / TIM1），这样才能做差速
  *   4) 静止时四相全断电 —— 12 V 直驱时通电保持会持续发热
  *   5) 三重热保护：静止断电 / 单机连续运转上限 / 双机重叠上限
  *
  * ⚠️ 机械参数需要上板实测标定：
  *      STEPPER_HALFSTEPS_PER_REV（默认 4076）
  *      TRACK_WHEEL_DIAMETER_MM   （默认 30）
  ******************************************************************************
  */
#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h>

/* ---------- 电机标识 ---------- */
typedef enum
{
    STEPPER_L = 0,      /* 左（PA11/PA12/PA15/PB0） */
    STEPPER_R = 1,      /* 右（PB1/PB3/PB4/PB8）  */
    STEPPER_COUNT = 2
} stepper_id_t;

/* ---------- 返回值 ---------- */
#define STEPPER_OK            0
#define STEPPER_ERR_BUSY     -1      /* 电机正在运动，需先 STOP */
#define STEPPER_ERR_THERMAL  -2      /* 冷却中 / 热保护跳闸 */
#define STEPPER_ERR_PARAM    -3
#define STEPPER_ERR_STEPS    -4      /* 步数为 0 */

/* ---------- 速度档（MOVE 命令的 speed 字节）----------
 * 定义已移到 User/app_config.h（因为 stepper_math.h 也要用，
 * 而它不能包含本头文件 —— 那会造成循环依赖）。
 * 这里加 #ifndef 兜底，单独包含本头文件时也能编译。
 */
#ifndef STEPPER_SPEED_0
#define STEPPER_SPEED_0       0      /* 120 Hz 蜗速 */
#endif
#ifndef STEPPER_SPEED_1
#define STEPPER_SPEED_1       1      /* 200 Hz */
#endif
#ifndef STEPPER_SPEED_2
#define STEPPER_SPEED_2       2      /* 300 Hz 默认 */
#endif
#ifndef STEPPER_SPEED_3
#define STEPPER_SPEED_3       3      /* 600 Hz 上限 */
#endif
#ifndef STEPPER_SPEED_DEFAULT
#define STEPPER_SPEED_DEFAULT 2
#endif

/* ============================ 初始化 ============================ */
/* 配置 8 路 GPIO、JATAG 重映射、两个步进时基定时器（不启动） */
void     Stepper_Init(void);

/* 1 ms 调度调用：加减速斜坡 + 热保护 + 重叠保护 */
void     Stepper_Task1ms(uint32_t now_ms);

/* ============================ 运动控制 ============================ */
/**
  * @brief  单电机相对移动
  * @param  id         电机
  * @param  halfsteps  半步数（正 = 正转，负 = 反转，0 = 无效）
  * @param  hz         目标巡航步频（会被钳到 [MIN, MAX]）
  * @retval STEPPER_OK / ERR_BUSY / ERR_THERMAL / ERR_STEPS
  */
int      Stepper_Move(stepper_id_t id, int32_t halfsteps, uint16_t hz);

/**
  * @brief  双电机同时移动（差速用）
  * @note   步数为 0 的一侧保持不动
  */
int      Stepper_MoveBoth(int32_t left_steps, int32_t right_steps, uint16_t hz);

int      Stepper_Stop(stepper_id_t id);

/* 清除热保护冷却（台架调试用；正式运行不要调，否则失去过热保护） */
void     Stepper_ClearCooldown(void);
void     Stepper_StopAll(void);

/* ============================ 查询 ============================ */
uint8_t  Stepper_IsBusy(stepper_id_t id);
uint8_t  Stepper_IsAnyBusy(void);
int32_t  Stepper_GetPosition(stepper_id_t id);
int32_t  Stepper_GetRemaining(stepper_id_t id);
uint16_t Stepper_GetCurrentHz(stepper_id_t id);
uint8_t  Stepper_IsCoolingDown(stepper_id_t id);
uint32_t Stepper_GetThermalTripCount(void);

/* 完成事件：主循环轮询，自上次调用后新完成则返回 1 */
uint8_t  Stepper_PollDone(stepper_id_t id);

/* 速度档 -> Hz；越界返回默认档 */
uint16_t Stepper_SpeedToHz(uint8_t speed);

/* 半步 -> 距离，单位 0.1 mm（避免浮点） */
uint32_t Stepper_StepsToMm10(int32_t halfsteps);

#endif /* __STEPPER_H */
