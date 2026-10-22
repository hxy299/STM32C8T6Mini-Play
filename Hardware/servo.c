/**
  ******************************************************************************
  * @file    Hardware/servo.c
  * @brief   M3：SG90 舵机驱动（PA0 / TIM2_CH1 / 50 Hz / 限幅 ±45°）
  ******************************************************************************
  */
#include "stm32f10x.h"
#include "app_config.h"
#include "config.h"
#include "servo_math.h"
#include "servo.h"

/* ============================ 内部状态 ============================ */
static uint8_t  s_enabled    = 0u;      /* TIM2 是否已启动并输出 */
static int8_t   s_cur_deg    = 0;       /* 当前角度（缓动中间态） */
static int8_t   s_target_deg = 0;       /* 目标角度 */
static uint16_t s_pulse_us   = SERVO_CENTER_US;

/* 缓动状态机 */
static uint8_t  s_easing     = 0u;
static int8_t   s_ease_from  = 0;
static int8_t   s_ease_to    = 0;
static uint16_t s_ease_total_ms = 0u;
static uint16_t s_ease_done_ms  = 0u;
static uint16_t s_ease_steps    = 0u;
static uint16_t s_ease_step     = 0u;

static uint32_t s_move_count = 0u;

/* ============================ 工具 ============================ */

static int8_t clamp_deg(int16_t deg, uint8_t *clamped)
{
    return servo_math_clamp(deg, clamped);
}

/**
  * @brief  角度 -> 脉宽（微秒）
  * @note   实现在 servo_math.h（纯整数、可在主机上测试）
  */
uint16_t Servo_DegToPulseUs(int8_t deg)
{
    return servo_math_deg_to_us((int16_t)deg);
}

static void servo_write_pulse(uint16_t us)
{
    if (!s_enabled)
    {
        return;
    }
    s_pulse_us = us;
    TIM_SetCompare1(SERVO_TIM, (uint16_t)us);
}

static void servo_apply_current(void)
{
    servo_write_pulse(Servo_DegToPulseUs(s_cur_deg));
}

/* ============================ 初始化 / 开关 ============================ */

int Servo_Init(void)
{
    /* 功能关闭时完全不碰寄存器（说明书 §5.6.2） */
    if (!Config_IsEnabled(CFG_SERVO_EN))
    {
        s_enabled = 0u;
        return SERVO_ERR_DISABLED;
    }
    return Servo_Enable();
}

int Servo_Enable(void)
{
    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef       oc;

    if (s_enabled)
    {
        return SERVO_OK;
    }

    RCC_APB2PeriphClockCmd(SERVO_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(SERVO_TIM_CLK, ENABLE);

    /* PA0 = TIM2_CH1，复用推挽 */
    gpio.GPIO_Pin   = SERVO_GPIO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SERVO_GPIO_PORT, &gpio);

    /* PSC = 72-1 -> 1 MHz；ARR = 20000-1 -> 20 ms (50 Hz) */
    tb.TIM_Prescaler         = SERVO_TIM_PRESCALER - 1u;
    tb.TIM_Period            = SERVO_TIM_PERIOD - 1u;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(SERVO_TIM, &tb);

    /* PWM1 模式，高电平有效 */
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_Pulse       = SERVO_CENTER_US;
    TIM_OC1Init(SERVO_TIM, &oc);

    /* 立刻装载 CCR，避免第一个周期用旧值 */
    TIM_OC1PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(SERVO_TIM, ENABLE);

    TIM_SetCompare1(SERVO_TIM, SERVO_CENTER_US);
    TIM_Cmd(SERVO_TIM, ENABLE);

    s_enabled    = 1u;
    s_cur_deg    = 0;
    s_target_deg = 0;
    s_pulse_us   = SERVO_CENTER_US;
    s_easing     = 0u;

    return SERVO_OK;
}

void Servo_Disable(void)
{
    if (!s_enabled)
    {
        return;
    }

    /* 停止 PWM 输出：舵机失去驱动，自然卸力（不会持续堵转发热） */
    TIM_Cmd(SERVO_TIM, DISABLE);
    TIM_SetCompare1(SERVO_TIM, 0u);

    /* 把 PA0 拉回普通推挽输出低电平，避免悬空 */
    {
        GPIO_InitTypeDef gpio;
        GPIO_ResetBits(SERVO_GPIO_PORT, SERVO_GPIO_PIN);
        gpio.GPIO_Pin   = SERVO_GPIO_PIN;
        gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
        gpio.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(SERVO_GPIO_PORT, &gpio);
    }

    s_enabled = 0u;
    s_easing  = 0u;
}

uint8_t Servo_IsEnabled(void)
{
    return s_enabled;
}

/* ============================ 角度控制 ============================ */

int Servo_MoveTo(int8_t deg, uint16_t ms, uint8_t *clamped)
{
    int8_t target;

    if (clamped)
    {
        *clamped = 0u;
    }

    if (!s_enabled)
    {
        return SERVO_ERR_DISABLED;
    }

    target = clamp_deg((int16_t)deg, clamped);
    s_target_deg = target;

    /* ms = 0 或目标等于当前：立即到位 */
    if (ms == 0u || target == s_cur_deg)
    {
        s_cur_deg = target;
        s_easing  = 0u;
        servo_apply_current();
        s_move_count++;
        return SERVO_OK;
    }

    /* 启动线性缓动 */
    s_ease_from     = s_cur_deg;
    s_ease_to       = target;
    s_ease_total_ms = ms;
    s_ease_done_ms  = 0u;

    /* 步数 = 总时长 / 步进周期，至少 1 步 */
    s_ease_steps = (uint16_t)(ms / SERVO_EASE_TICK_MS);
    if (s_ease_steps == 0u)
    {
        s_ease_steps = 1u;
    }
    s_ease_step = (uint16_t)(ms / s_ease_steps);

    s_easing = 1u;
    s_move_count++;
    return SERVO_OK;
}

/**
  * @brief  缓动更新，由 1 ms 调度调用
  * @note   用"已过时间 / 总时间"做线性插值（整数运算），
  *         不使用浮点，也不会累积误差。
  */
void Servo_Update(uint32_t now_ms)
{
    static uint32_t s_last = 0u;
    static uint8_t  s_init = 0u;

    (void)s_ease_step;   /* 保留字段，当前用时间比例插值 */

    if (!s_enabled || !s_easing)
    {
        s_init = 0u;
        return;
    }

    if (!s_init)
    {
        s_last = now_ms;
        s_init = 1u;
        return;
    }

    if ((uint32_t)(now_ms - s_last) < SERVO_EASE_TICK_MS)
    {
        return;
    }

    /* 落后太多（例如主循环被 Flash 擦写阻塞过）时重新对齐，
     * 避免为了"追帧"而连续猛推舵机。*/
    if ((uint32_t)(now_ms - s_last) > (SERVO_EASE_TICK_MS * 5u))
    {
        s_last = (uint32_t)(now_ms - SERVO_EASE_TICK_MS);
    }

    s_last += SERVO_EASE_TICK_MS;

    s_ease_done_ms = (uint16_t)(s_ease_done_ms + SERVO_EASE_TICK_MS);

    if (s_ease_done_ms >= s_ease_total_ms)
    {
        /* 到位 */
        s_cur_deg = s_ease_to;
        s_easing  = 0u;
        s_init    = 0u;
        servo_apply_current();
        return;
    }

    /* 线性插值：cur = from + (to - from) * done / total */
    {
        int32_t span = (int32_t)s_ease_to - (int32_t)s_ease_from;
        int32_t off  = (span * (int32_t)s_ease_done_ms) / (int32_t)s_ease_total_ms;

        s_cur_deg = (int8_t)((int32_t)s_ease_from + off);
        servo_apply_current();
    }
}

/* ============================ 查询 ============================ */

int8_t Servo_GetCurrentDeg(void)
{
    return s_cur_deg;
}

int8_t Servo_GetTargetDeg(void)
{
    return s_target_deg;
}

uint16_t Servo_GetPulseUs(void)
{
    return s_pulse_us;
}

uint32_t Servo_GetMoveCount(void)
{
    return s_move_count;
}
