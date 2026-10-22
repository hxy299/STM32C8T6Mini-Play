/**
  ******************************************************************************
  * @file    BSP/bsp.c
  * @brief   板级支撑实现
  ******************************************************************************
  */
#include "stm32f10x.h"
#include "app_config.h"
#include "bsp.h"

/* 调度时基计数（TIM3 中断里自增） */
static volatile uint32_t g_tick_ms = 0u;

/* ============================ 系统级 ============================ */

/**
  * @brief  NVIC 优先级分组（必须在任何 NVIC_Init 之前调用一次）
  * @note   用 Group_2：2 位抢占、2 位子优先级。
  *         抢占优先级数值：TIM3 = 2，USART1 = 3（时基优先级更高）。
  */
void BSP_NvicInit(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

/* ============================ LED ============================ */

void BSP_LedInit(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(LED_GPIO_CLK, ENABLE);

    gpio.GPIO_Pin   = LED_GPIO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(LED_GPIO_PORT, &gpio);

    BSP_LedOff();   /* 上电先灭，避免误判 */
}

void BSP_LedOn(void)
{
#if LED_ACTIVE_LOW
    GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PIN);
#else
    GPIO_SetBits(LED_GPIO_PORT, LED_GPIO_PIN);
#endif
}

void BSP_LedOff(void)
{
#if LED_ACTIVE_LOW
    GPIO_SetBits(LED_GPIO_PORT, LED_GPIO_PIN);
#else
    GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PIN);
#endif
}

void BSP_LedToggle(void)
{
    if (GPIO_ReadOutputDataBit(LED_GPIO_PORT, LED_GPIO_PIN))
    {
        GPIO_ResetBits(LED_GPIO_PORT, LED_GPIO_PIN);
    }
    else
    {
        GPIO_SetBits(LED_GPIO_PORT, LED_GPIO_PIN);
    }
}

/**
  * @brief  LED 提示模式
  *
  * 三种可肉眼区分的节奏：
  *   OK    : 亮 500 / 灭 500                      （1 Hz 均闪）
  *   WAIT  : 亮 100 / 灭 150 / 亮 100 / 灭 850    （双闪后长停，1.2 s 一轮）
  *   FAULT : 亮 100 / 灭 100                      （5 Hz 快闪）
  *
  * 为什么要区分 WAIT 与 OK：上电后若从未收到上位机帧，固件按安全策略
  * 会置 heartbeat 故障；但"没接上位机"和"接上又掉了"是两件事，
  * 肉眼能分开可以省很多排查时间。
  */
void BSP_LedRunPattern(uint32_t now_ms, bsp_led_pattern_t pattern)
{
    static uint32_t s_last = 0u;
    static uint8_t  s_init = 0u;
    static uint8_t  s_wait_on = 0u;

    if (!s_init)
    {
        s_last = now_ms;
        s_init = 1u;
        BSP_LedOff();
        return;
    }

    switch (pattern)
    {
    case BSP_LED_PATTERN_OK:
        s_wait_on = 0u;
        if ((uint32_t)(now_ms - s_last) >= 500u)
        {
            s_last = now_ms;
            BSP_LedToggle();
        }
        break;

    case BSP_LED_PATTERN_FAULT:
        s_wait_on = 0u;
        if ((uint32_t)(now_ms - s_last) >= 100u)
        {
            s_last = now_ms;
            BSP_LedToggle();
        }
        break;

    case BSP_LED_PATTERN_WAIT:
    default:
    {
        /* 1.2 s 一轮：0..100 亮 / 100..250 灭 / 250..350 亮 / 350..1200 灭 */
        uint32_t elapsed = (uint32_t)(now_ms - s_last);

        if (elapsed >= 1200u)
        {
            s_last += (elapsed / 1200u) * 1200u;
            elapsed = (uint32_t)(now_ms - s_last);
            s_wait_on = 0u;
        }

        if ((elapsed < 100u) || (elapsed >= 250u && elapsed < 350u))
        {
            if (!s_wait_on)
            {
                s_wait_on = 1u;
                BSP_LedOn();
            }
        }
        else
        {
            if (s_wait_on)
            {
                s_wait_on = 0u;
                BSP_LedOff();
            }
        }
        break;
    }
    }
}

/**
  * @brief  LED 心跳任务：每 period_ms 翻转一次（保留兼容）
  * @note   由主循环调用，非阻塞
  */
void BSP_LedHeartbeatTask(uint32_t now_ms, uint32_t period_ms)
{
    static uint32_t last = 0u;
    static uint8_t  inited = 0u;

    if (!inited)
    {
        last = now_ms;
        inited = 1u;
        return;
    }

    if ((uint32_t)(now_ms - last) >= period_ms)
    {
        last = now_ms;
        BSP_LedToggle();
    }
}

/* ============================ 看门狗 ============================ */

void BSP_IwdgInit(void)
{
    /* LSI 约 40 kHz；预分频 64 -> 625 Hz；重装 1000 -> 约 1.6 s
     * 注意：IWDG 一旦启动无法关闭，只能在复位后重新配置。
     */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(APP_IWDG_PRESCALER);
    IWDG_SetReload(APP_IWDG_RELOAD);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

void BSP_IwdgFeed(void)
{
    IWDG_ReloadCounter();
}

/* ============================ TIM3 1 ms 调度时基 ============================ */

void BSP_TickInit(void)
{
    TIM_TimeBaseInitTypeDef tb;
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(APP_TICK_TIM_CLK, ENABLE);

    /* APB1 = 36 MHz，但定时器时钟翻倍 = 72 MHz
     * PSC = 72-1 -> 1 MHz；ARR = 1000-1 -> 1 ms
     */
    tb.TIM_Prescaler         = APP_TICK_PRESCALER - 1u;
    tb.TIM_Period            = APP_TICK_PERIOD - 1u;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(APP_TICK_TIM, &tb);

    TIM_ClearFlag(APP_TICK_TIM, TIM_FLAG_Update);
    TIM_ITConfig(APP_TICK_TIM, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel                   = APP_TICK_TIM_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2u;   /* 高于串口，保证时序 */
    nvic.NVIC_IRQChannelSubPriority        = 0u;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    TIM_Cmd(APP_TICK_TIM, ENABLE);
}

uint32_t BSP_GetTickMs(void)
{
    return g_tick_ms;
}

/* TIM3 中断：1 ms 一次
 * ⚠️ 中断里只做最轻的工作，重活放主循环（见说明书 §8.1）
 */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(APP_TICK_TIM, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(APP_TICK_TIM, TIM_IT_Update);
        g_tick_ms++;
    }
}

/* ============================ 时钟 ============================ */

uint32_t BSP_GetSysClock(void)
{
    return SystemCoreClock;
}

uint8_t BSP_ClockIsAsExpected(uint32_t expected_hz)
{
    return (BSP_GetSysClock() == expected_hz) ? 1u : 0u;
}
