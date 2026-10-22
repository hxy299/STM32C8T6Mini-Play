/**
  ******************************************************************************
  * @file    tools/hosttest/app_config.h
  * @brief   主机测试专用桩
  *
  * 说明：真实的 User\app_config.h 依赖 stm32f10x.h（寄存器/外设宏），
  *       无法在 x86 主机上编译。主机测试时用本桩替换，
  *       搜索路径上 tools/hosttest 必须排在 User/ 之前。
  *
  * 只提供被测模块真正用到的宏：
  *   - Protocol/link_proto.c  -> LINK_*
  *   - Hardware/servo_math.h  -> SERVO_*
  ******************************************************************************
  */
#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/* ---- 协议 ---- */
#define LINK_SOF                    0xA5u
#define LINK_MAX_PAYLOAD            200u
#define LINK_MAX_FRAME              (6u + LINK_MAX_PAYLOAD + 2u)
#define LINK_INTERBYTE_TIMEOUT_MS   100u

/* ---- 舵机（必须与 User/app_config.h 保持一致）---- */
#define SERVO_CENTER_US             1500u
#define SERVO_HALF_RANGE_US         500u
#define SERVO_MAX_DEG               45
#define SERVO_MIN_DEG               (-45)

/* ---- 步进电机（必须与 User/app_config.h 保持一致）---- */
#define STEPPER_FULLSTEPS_PER_REV   2038u
#define STEPPER_HALFSTEPS_PER_REV   4076u
#define TRACK_WHEEL_DIAMETER_MM     30u

#define STEPPER_START_HZ            120u
#define STEPPER_HZ_SLOW             200u
#define STEPPER_HZ_NORMAL           300u
#define STEPPER_HZ_FAST             600u
#define STEPPER_HZ_MIN              60u
#define STEPPER_HZ_MAX              700u
#define STEPPER_ACCEL_HZ_PER_MS     4u
#define STEPPER_FULLSTEP_EN         1
#define STEPPER_TIM_TICK_HZ         1000000u

#define STEPPER_SPEED_0             0
#define STEPPER_SPEED_1             1
#define STEPPER_SPEED_2             2
#define STEPPER_SPEED_3             3

#endif /* __APP_CONFIG_H */
