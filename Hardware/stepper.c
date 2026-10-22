/**
  ******************************************************************************
  * @file    Hardware/stepper.c
  * @brief   M4/M5：28BYJ-48 ×2 步进驱动（8 拍半步 + 精确步数 + 加减速 + 热保护）
  *
  * 时基分配：
  *   TIM4 -> 左电机步进节拍
  *   TIM1 -> 右电机步进节拍（仅用更新中断，不用其输出通道，故 PA8 仍可作普通 IO）
  *   TIM3 -> 1 ms 调度（喂加减速斜坡）
  *   TIM2 -> 舵机 50 Hz
  *
  * 步进节拍定时器：PSC=72-1 -> 1 MHz；ARR = 1000000/hz - 1
  ******************************************************************************
  */
#include "stm32f10x.h"
#include "app_config.h"
#include "config.h"
#include "bsp.h"
#include "stepper_math.h"
#include "stepper.h"

/* ============================ 相序表 ============================ */
/* 相序表在 stepper_math.h 里（STEPPER_HALF_SEQ），便于主机测试。 */

/* ============================ 每个电机的状态 ============================ */
typedef struct
{
    GPIO_TypeDef *port[4];
    uint16_t      pin[4];
    TIM_TypeDef  *tim;
    uint8_t       irq;

    volatile uint8_t  phase;          /* 0..7 当前拍号 */
    volatile int32_t  remaining;      /* 剩余半步（带符号 = 方向） */
    volatile int32_t  position;       /* 累计位置（半步） */
    volatile uint8_t  running;
    volatile uint8_t  done_pending;   /* 刚跑完，待主循环取走 */
    volatile uint16_t cur_hz;         /* 当前步频 */

    uint16_t target_hz;               /* 巡航步频 */
    uint8_t  decelerating;            /* 已进入减速段（防止来回抖） */
    uint32_t run_start_ms;            /* 本轮连续运转起点 */
    uint32_t cool_until_ms;           /* 冷却结束时刻 */
    uint8_t  thermal_trip;            /* 因过热被强制停 */
} stepper_t;

static stepper_t s_motors[STEPPER_COUNT];

/* 双机重叠计时 */
static uint32_t s_overlap_start = 0u;
static uint8_t  s_overlap_active = 0u;

static uint32_t s_thermal_trip_count = 0u;

/* 是否已完成初始化。未初始化时拒绝一切运动，
 * 避免 STEPPER_ENABLE=0 时仍去碰未配置的定时器。 */
static uint8_t s_ready = 0u;

/* ============================ 底层输出 ============================ */

static void apply_phase(stepper_t *m, uint8_t beat)
{
    /* 全步/半步由 STEPPER_FULLSTEP_EN 决定，见 stepper_math.h */
    uint8_t mask = stepper_math_phase_bits(beat);
    uint8_t i;

    for (i = 0u; i < 4u; i++)
    {
        if (mask & (uint8_t)(1u << i))
        {
            GPIO_SetBits(m->port[i], m->pin[i]);
        }
        else
        {
            GPIO_ResetBits(m->port[i], m->pin[i]);
        }
    }
}

static void coils_off(stepper_t *m)
{
    uint8_t i;
    for (i = 0u; i < 4u; i++)
    {
        GPIO_ResetBits(m->port[i], m->pin[i]);
    }
}

/* 立即停：关定时器 + 四相断电（静止必须断电，否则 12V 下持续发热）*/
static void stepper_hard_stop(stepper_t *m)
{
    TIM_ITConfig(m->tim, TIM_IT_Update, DISABLE);
    TIM_Cmd(m->tim, DISABLE);
    TIM_ClearITPendingBit(m->tim, TIM_IT_Update);

    coils_off(m);

    m->running = 0u;
    m->cur_hz  = 0u;
    m->decelerating = 0u;
}

/* 设置步进频率（1 MHz 基准）*/
static void stepper_set_rate(stepper_t *m, uint16_t hz)
{
    if (hz < STEPPER_HZ_MIN) { hz = STEPPER_HZ_MIN; }
    if (hz > STEPPER_HZ_MAX) { hz = STEPPER_HZ_MAX; }

    /* ARPE 已在初始化时关闭，直接写 ARR 立即生效。
     * 具体 ARR 计算在 stepper_math.h，已被主机测试覆盖。 */
    m->tim->ARR = stepper_math_hz_to_arr(hz);
    m->cur_hz = hz;
}

/* ============================ 步进中断体 ============================ */

static void stepper_isr(stepper_t *m)
{
    if (!m->running || (m->remaining == 0))
    {
        stepper_hard_stop(m);
        return;
    }

    /* 相位环长度随模式变化：全步 4 拍、半步 8 拍 */
    {
        uint8_t pmask = stepper_math_phase_mask();

        if (m->remaining > 0)
        {
            m->phase = (uint8_t)((m->phase + 1u) & pmask);
            m->position++;
            m->remaining--;
        }
        else
        {
            m->phase = (uint8_t)((m->phase + pmask) & pmask);  /* -1 mod (pmask+1) */
            m->position--;
            m->remaining++;
        }
    }

    apply_phase(m, m->phase);

    if (m->remaining == 0)
    {
        stepper_hard_stop(m);
        m->done_pending = 1u;
    }
}

/* ============================ 初始化 ============================ */

void Stepper_Init(void)
{
    /* 用局部自动数组初始化（而非 static）：GPIOx 是 (GPIO_TypeDef*)整数常量
     * 的强制转换，放在静态初始化里部分编译器不接受，放函数体内最稳妥。*/
    GPIO_TypeDef *a_ports[4] = STEPPER_A_PORTS;
    uint16_t      a_pins[4]  = STEPPER_A_PINS;
    GPIO_TypeDef *b_ports[4] = STEPPER_B_PORTS;
    uint16_t      b_pins[4]  = STEPPER_B_PINS;

    GPIO_InitTypeDef        gpio;
    TIM_TimeBaseInitTypeDef tb;
    NVIC_InitTypeDef        nvic;
    uint8_t i;

    /* ---------- 1) 打开 GPIO 与 AFIO 时钟 ---------- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO, ENABLE);

    /* ---------- 2) 释放 JTAG 引脚（PA15 / PB3 / PB4）----------
     * 只关 JTAG，保留 SWD（PA13/PA14），否则下载调试会断。
     */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* ---------- 3) 配置 8 路推挽输出 ---------- */
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;

    for (i = 0u; i < 4u; i++)
    {
        gpio.GPIO_Pin = a_pins[i];
        GPIO_Init(a_ports[i], &gpio);

        gpio.GPIO_Pin = b_pins[i];
        GPIO_Init(b_ports[i], &gpio);
    }

    /* ---------- 4) 记录到电机结构体 ---------- */
    for (i = 0u; i < 4u; i++)
    {
        s_motors[STEPPER_L].port[i] = a_ports[i];
        s_motors[STEPPER_L].pin[i]  = a_pins[i];
        s_motors[STEPPER_R].port[i] = b_ports[i];
        s_motors[STEPPER_R].pin[i]  = b_pins[i];
    }

    s_motors[STEPPER_L].tim = STEPPER_A_TIM;
    s_motors[STEPPER_L].irq = (uint8_t)STEPPER_A_IRQn;
    s_motors[STEPPER_R].tim = STEPPER_B_TIM;
    s_motors[STEPPER_R].irq = (uint8_t)STEPPER_B_IRQn;

    for (i = 0u; i < STEPPER_COUNT; i++)
    {
        stepper_t *m = &s_motors[i];

        m->phase         = 0u;
        m->remaining     = 0;
        m->position      = 0;
        m->running       = 0u;
        m->done_pending  = 0u;
        m->cur_hz        = 0u;
        m->target_hz     = STEPPER_HZ_NORMAL;
        m->decelerating  = 0u;
        m->run_start_ms  = 0u;
        m->cool_until_ms = 0u;
        m->thermal_trip  = 0u;

        coils_off(m);

        /* ---------- 5) 步进时基定时器（只开更新中断，不配输出通道）---------- */
        if (m->tim == STEPPER_A_TIM)
        {
            RCC_APB1PeriphClockCmd(STEPPER_A_TIM_CLK, ENABLE);
        }
        else
        {
            RCC_APB2PeriphClockCmd(STEPPER_B_TIM_CLK, ENABLE);
        }

        tb.TIM_Prescaler         = STEPPER_TIM_PRESCALER - 1u;
        tb.TIM_Period            = (uint16_t)((STEPPER_TIM_TICK_HZ / STEPPER_HZ_NORMAL) - 1u);
        tb.TIM_CounterMode       = TIM_CounterMode_Up;
        tb.TIM_ClockDivision     = TIM_CKD_DIV1;
        tb.TIM_RepetitionCounter = 0;
        TIM_TimeBaseInit(m->tim, &tb);

        /* 关 ARPE：这样 1 ms 任务里改 ARR 立刻生效，便于平滑变速 */
        TIM_ARRPreloadConfig(m->tim, DISABLE);

        TIM_ClearFlag(m->tim, TIM_FLAG_Update);
        TIM_ITConfig(m->tim, TIM_IT_Update, DISABLE);   /* 先不开，等运动时再开 */
        TIM_Cmd(m->tim, DISABLE);

        nvic.NVIC_IRQChannel                   = m->irq;
        nvic.NVIC_IRQChannelPreemptionPriority = 1u;    /* 步进最高，保证不丢拍 */
        nvic.NVIC_IRQChannelSubPriority        = 0u;
        nvic.NVIC_IRQChannelCmd                = ENABLE;
        NVIC_Init(&nvic);
    }

    s_overlap_active = 0u;
    s_overlap_start  = 0u;
    s_ready = 1u;
}

/* ============================ 启动一次运动 ============================ */

static int stepper_start(stepper_t *m, int32_t halfsteps, uint16_t hz, uint32_t now_ms)
{
    if (!s_ready)
    {
        return STEPPER_ERR_PARAM;      /* 步进被禁用（STEPPER_ENABLE=0） */
    }
    if (halfsteps == 0)
    {
        return STEPPER_ERR_STEPS;
    }
    if (m->running)
    {
        return STEPPER_ERR_BUSY;
    }
    if ((m->cool_until_ms != 0u) && (now_ms < m->cool_until_ms))
    {
        return STEPPER_ERR_THERMAL;
    }

    m->phase        = (uint8_t)(m->phase & stepper_math_phase_mask());
    m->remaining    = halfsteps;
    m->decelerating = 0u;
    m->thermal_trip = 0u;
    m->done_pending = 0u;
    m->target_hz    = hz;

    /* 起步必须从低速开始，直接给高频率会丢步 */
    stepper_set_rate(m, STEPPER_START_HZ);

    m->run_start_ms = now_ms;
    m->running = 1u;

    apply_phase(m, m->phase);          /* 先把当前拍输出，避免第一拍毛刺 */
    TIM_ClearITPendingBit(m->tim, TIM_IT_Update);
    TIM_ITConfig(m->tim, TIM_IT_Update, ENABLE);
    TIM_Cmd(m->tim, ENABLE);

    return STEPPER_OK;
}

int Stepper_Move(stepper_id_t id, int32_t halfsteps, uint16_t hz)
{
    if (id >= STEPPER_COUNT)
    {
        return STEPPER_ERR_PARAM;
    }
    return stepper_start(&s_motors[id], halfsteps, hz, BSP_GetTickMs());
}

int Stepper_MoveBoth(int32_t left_steps, int32_t right_steps, uint16_t hz)
{
    uint32_t now = BSP_GetTickMs();
    int r1 = STEPPER_OK;
    int r2 = STEPPER_OK;

    if (left_steps != 0)
    {
        r1 = stepper_start(&s_motors[STEPPER_L], left_steps, hz, now);
    }
    if (right_steps != 0)
    {
        r2 = stepper_start(&s_motors[STEPPER_R], right_steps, hz, now);
    }

    if (r1 != STEPPER_OK || r2 != STEPPER_OK)
    {
        /* ⚠️ 任一侧失败必须【回滚另一侧】，不能"只让一侧动"。
         * 差速车左右各走各的会立刻分叉：直线走歪、转向角度全错，
         * 而且上位机收到的是 ACK，根本发现不了。
         * 实测症状：MOTION_DONE 里 L/R 位置不相等（例如 8914 vs 8895）。
         */
        if (r1 == STEPPER_OK)
        {
            stepper_hard_stop(&s_motors[STEPPER_L]);
        }
        if (r2 == STEPPER_OK)
        {
            stepper_hard_stop(&s_motors[STEPPER_R]);
        }

        return (r1 != STEPPER_OK) ? r1 : r2;
    }
    return STEPPER_OK;
}

int Stepper_Stop(stepper_id_t id)
{
    if (id >= STEPPER_COUNT)
    {
        return STEPPER_ERR_PARAM;
    }
    stepper_hard_stop(&s_motors[id]);
    return STEPPER_OK;
}

void Stepper_StopAll(void)
{
    stepper_hard_stop(&s_motors[STEPPER_L]);
    stepper_hard_stop(&s_motors[STEPPER_R]);
    s_overlap_active = 0u;
}

/**
  * @brief  清除热保护冷却
  * @note   仅台架调试用。会一并清掉 thermal_trip 标志与冷却截止时间，
  *         让下一次 MOVE 可以立刻执行。正式运行不要调用。
  */
void Stepper_ClearCooldown(void)
{
    uint8_t i;
    for (i = 0u; i < STEPPER_COUNT; i++)
    {
        s_motors[i].cool_until_ms = 0u;
        s_motors[i].thermal_trip  = 0u;
    }
}

/* ============================ 1 ms 任务 ============================ */

void Stepper_Task1ms(uint32_t now_ms)
{
    uint8_t i;
    uint8_t busy_l;
    uint8_t busy_r;

    for (i = 0u; i < STEPPER_COUNT; i++)
    {
        stepper_t *m = &s_motors[i];

        /* ---------- 热保护 1：单机连续运转上限 ----------
         * ⚠️ 必须防 uint32 下溢：
         *    run_start_ms 取自 Stepper_MoveBoth 内部重新读的 tick，
         *    而 Task1ms 的 now_ms 是主循环开头读的，两者可能差 1 ms。
         *    若 run_start_ms > now_ms，(uint32_t)(now - start) 会变成
         *    约 42 亿，于是"刚启动就热跳闸"。必须先判大小再相减。
         */
        if (m->running)
        {
            uint32_t run_elapsed = (now_ms >= m->run_start_ms)
                                   ? (uint32_t)(now_ms - m->run_start_ms)
                                   : 0u;

            if (run_elapsed >= STEPPER_MAX_RUN_MS)
            {
                stepper_hard_stop(m);
                m->cool_until_ms = now_ms + STEPPER_COOLDOWN_MS;
                m->thermal_trip  = 1u;
                s_thermal_trip_count++;
                continue;
            }
        }

        if (!m->running)
        {
            continue;
        }

        /* ---------- 加减速斜坡 ---------- */
        {
            int32_t  rem = m->remaining;
            uint32_t absrem = (rem >= 0) ? (uint32_t)rem : (uint32_t)(-rem);
            uint32_t decel_need;
            uint16_t desired;

            /* 从当前频率降到 0 需要的步数（公式见 stepper_math.h） */
            decel_need = stepper_math_decel_steps(m->cur_hz);

            if (!m->decelerating && (absrem <= decel_need))
            {
                m->decelerating = 1u;
            }

            desired = m->decelerating ? STEPPER_HZ_MIN : m->target_hz;

            if (m->cur_hz < desired)
            {
                uint32_t v = (uint32_t)m->cur_hz + STEPPER_ACCEL_HZ_PER_MS;
                stepper_set_rate(m, (uint16_t)((v > desired) ? desired : v));
            }
            else if (m->cur_hz > desired)
            {
                uint32_t v = (uint32_t)m->cur_hz - STEPPER_ACCEL_HZ_PER_MS;
                stepper_set_rate(m, (uint16_t)((v < desired) ? desired : v));
            }
        }
    }

    /* ---------- 热保护 2：双机同时运转的重叠时间上限 ---------- */
    busy_l = s_motors[STEPPER_L].running;
    busy_r = s_motors[STEPPER_R].running;

    if (busy_l && busy_r)
    {
        if (!s_overlap_active)
        {
            s_overlap_active = 1u;
            s_overlap_start  = now_ms;
        }
        else
        {
            /* 同样防下溢（见上面热保护 1 的说明） */
            uint32_t ov_elapsed = (now_ms >= s_overlap_start)
                                  ? (uint32_t)(now_ms - s_overlap_start)
                                  : 0u;

            if (ov_elapsed >= STEPPER_MAX_OVERLAP_MS)
            {
                /* 两电机同时长时间跑会把 ULN2003 烧了，强制都停 */
                stepper_hard_stop(&s_motors[STEPPER_L]);
                stepper_hard_stop(&s_motors[STEPPER_R]);
                s_motors[STEPPER_L].cool_until_ms = now_ms + STEPPER_COOLDOWN_MS;
                s_motors[STEPPER_R].cool_until_ms = now_ms + STEPPER_COOLDOWN_MS;
                s_motors[STEPPER_L].thermal_trip  = 1u;
                s_motors[STEPPER_R].thermal_trip  = 1u;
                s_thermal_trip_count++;
                s_overlap_active = 0u;
            }
        }
    }
    else
    {
        s_overlap_active = 0u;
    }
}

/* ============================ 查询 ============================ */

uint8_t Stepper_IsBusy(stepper_id_t id)
{
    if (id >= STEPPER_COUNT)
    {
        return 0u;
    }
    return s_motors[id].running;
}

uint8_t Stepper_IsAnyBusy(void)
{
    return (uint8_t)(s_motors[STEPPER_L].running || s_motors[STEPPER_R].running);
}

/**
  * @brief  取某个电机的装配方向符号（+1 / -1）
  * @note   左右电机面对面安装，通同样的电流时转子电气方向相同、
  *         但物理转向相反。STEPPER_x_DIR_SIGN 就是用来抵消这个镜像的。
  */
static int8_t stepper_dir_sign(stepper_id_t id)
{
    return (id == STEPPER_L) ? (int8_t)STEPPER_L_DIR_SIGN
                             : (int8_t)STEPPER_R_DIR_SIGN;
}

/**
  * @brief  读取位置
  * @note   ⚠️ 返回值已换算到【机器人坐标】：正 = 机器人前进方向。
  *         如果不做这个换算，一侧的 DIR_SIGN 为 -1 时，
  *         前进会得到 L=+2000 / R=-2000，看起来像"左右不同步"，
  *         实际上是完全同步的。诊断数据必须用同一个坐标系。
  */
int32_t Stepper_GetPosition(stepper_id_t id)
{
    if (id >= STEPPER_COUNT)
    {
        return 0;
    }
    return s_motors[id].position * (int32_t)stepper_dir_sign(id);
}

/**
  * @brief  读取剩余步数
  * @note   同样换算到机器人坐标系，保证正负含义与位置一致。
  *         内部减速计算用的是原始 remaining 字段，不受影响。
  */
int32_t Stepper_GetRemaining(stepper_id_t id)
{
    if (id >= STEPPER_COUNT)
    {
        return 0;
    }
    return s_motors[id].remaining * (int32_t)stepper_dir_sign(id);
}

uint16_t Stepper_GetCurrentHz(stepper_id_t id)
{
    if (id >= STEPPER_COUNT)
    {
        return 0u;
    }
    return s_motors[id].cur_hz;
}

uint8_t Stepper_IsCoolingDown(stepper_id_t id)
{
    if (id >= STEPPER_COUNT)
    {
        return 0u;
    }
    return (uint8_t)((s_motors[id].cool_until_ms != 0u) &&
                     (BSP_GetTickMs() < s_motors[id].cool_until_ms));
}

uint32_t Stepper_GetThermalTripCount(void)
{
    return s_thermal_trip_count;
}

uint8_t Stepper_PollDone(stepper_id_t id)
{
    if (id >= STEPPER_COUNT)
    {
        return 0u;
    }
    if (s_motors[id].done_pending)
    {
        s_motors[id].done_pending = 0u;
        return 1u;
    }
    return 0u;
}

/* ============================ 换算 ============================ */

uint16_t Stepper_SpeedToHz(uint8_t speed)
{
    return stepper_math_speed_to_hz(speed);
}

uint32_t Stepper_StepsToMm10(int32_t halfsteps)
{
    return stepper_math_steps_to_mm10(halfsteps);
}
/* ============================ 中断入口 ============================ */

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(STEPPER_A_TIM, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(STEPPER_A_TIM, TIM_IT_Update);
        stepper_isr(&s_motors[STEPPER_L]);
    }
}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(STEPPER_B_TIM, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(STEPPER_B_TIM, TIM_IT_Update);
        stepper_isr(&s_motors[STEPPER_R]);
    }
}
