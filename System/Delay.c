/**
  ******************************************************************************
  * @file    System/Delay.c
  * @brief   SysTick 延时实现（参考旧小车工程，保持行为一致）
  *
  * ⚠️ 见 Delay.h 的警告：本文件会改写/关闭 SysTick，
  *    因此调度时基必须用 TIM3（BSP_TickInit），不能用 SysTick。
  ******************************************************************************
  */
#include "stm32f10x.h"
#include "Delay.h"

/* 72 MHz 下每微秒 72 个 SysTick 计数 */
#define DELAY_TICKS_PER_US   72u

/**
  * @brief  微秒级延时（范围 0 ~ 233015 us）
  * @note   会临时接管 SysTick，结束后关闭。不要与调度时基混用。
  */
void Delay_us(uint32_t xus)
{
    SysTick->LOAD = DELAY_TICKS_PER_US * xus;
    SysTick->VAL  = 0x00u;
    SysTick->CTRL = 0x00000005u;              /* HCLK 时钟源 + 启动 */
    while (!(SysTick->CTRL & 0x00010000u));   /* 等 COUNTFLAG */
    SysTick->CTRL = 0x00000004u;              /* 关闭 */
}

/**
  * @brief  毫秒级延时
  */
void Delay_ms(uint32_t xms)
{
    while (xms--)
    {
        Delay_us(1000u);
    }
}

/**
  * @brief  秒级延时
  */
void Delay_s(uint32_t xs)
{
    while (xs--)
    {
        Delay_ms(1000u);
    }
}
