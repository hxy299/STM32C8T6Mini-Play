/**
  ******************************************************************************
  * @file    User/main.c
  * @brief   A733 桌宠 下位机运动控制 —— M0 ~ M2.5 阶段骨架
  *
  * 本阶段目标（对应项目说明书 §10）：
  *   M0   时钟 72 MHz / GPIO / TIM3 时基 / IWDG / LED 心跳
  *   M1   USART1 收帧 + CRC16 + ACK/PONG
  *   M2   心跳超时检测 + 看门狗
  *   M2.5 Flash 配置持久化 + SET_CONFIG / GET_CONFIG
  *
  * 本阶段**不驱动任何执行器**（步进/舵机/TOF/TTS 均未初始化）。
  *
  * 纪律：编译通过不等于通过。验收判据见说明书 §10 的 M0/M1/M2/M2.5 四行。
  ******************************************************************************
  */
#include "stm32f10x.h"
#include <stdio.h>
#include "app_config.h"
#include "Delay.h"
#include "bsp.h"
#include "serial.h"
#include "link_proto.h"
#include "config.h"
#include "servo.h"
#include "stepper.h"
#include "link_service.h"

/* 调试状态行：每 5 秒打印一次链路统计。
 * ⚠️ 已改为 0：Serial_Printf 阻塞发送，会打断 1ms 调度节奏，
 *    协议联调阶段关掉；单独排查硬件时再临时打开。 */
#define DEBUG_STATUS_PRINT      0
#define DEBUG_STATUS_PERIOD_MS  5000u

static void print_banner(void)
{
    Serial_Printf("\r\n");
    Serial_Printf("========================================\r\n");
    Serial_Printf(" A733 DeskPet  Motion MCU  (STM32F103C8T6)\r\n");
    Serial_Printf(" stage: M0 ~ M2.5  (skeleton, no actuator)\r\n");
    Serial_Printf("----------------------------------------\r\n");
}

static void print_selftest(void)
{
    uint32_t clk = BSP_GetSysClock();

    Serial_Printf(" SYSCLK : %lu Hz  %s\r\n",
                  (unsigned long)clk,
                  BSP_ClockIsAsExpected(APP_EXPECTED_SYSCLK) ? "[OK]" : "[!!]");
    Serial_Printf(" link   : USART1 %lu 8N1  PA9(TX)/PA10(RX)\r\n",
                  (unsigned long)LINK_UART_BAUDRATE);
    Serial_Printf(" tick   : TIM3 1ms   IWDG: ~1.6s\r\n");
    Serial_Printf(" config : 0x%08lX  (%s)\r\n",
                  (unsigned long)Config_Get(),
                  Config_IsDefault() ? "default" : "from flash");
    Serial_Printf("   SERVO=%u TTS=%u BT=%u  TOF0/1/2/3=%u/%u/%u/%u\r\n",
                  Config_IsEnabled(CFG_SERVO_EN),
                  Config_IsEnabled(CFG_TTS_EN),
                  Config_IsEnabled(CFG_BT_EN),
                  Config_IsEnabled(CFG_TOF0_EN),
                  Config_IsEnabled(CFG_TOF1_EN),
                  Config_IsEnabled(CFG_TOF2_EN),
                  Config_IsEnabled(CFG_TOF3_EN));

    if (Config_IsBottomTofAllDisabled())
    {
        Serial_Printf(" WARN   : bottom TOF all disabled -> BLIND MODE\r\n");
    }

    /* 执行器状态 */
    if (Servo_IsEnabled())
    {
        Serial_Printf(" servo  : ON  PA0/TIM2_CH1 50Hz  center=%uus pulse=%uus "
                      "range=\xC2\xB1%d deg\r\n",
                      (unsigned)SERVO_CENTER_US, (unsigned)Servo_GetPulseUs(),
                      (int)SERVO_MAX_DEG);
    }
    else
    {
        Serial_Printf(" servo  : OFF (CFG_SERVO_EN=0, TIM2 not started)\r\n");
    }

    Serial_Printf(" stepper: %s  L=PB1/3/4/8  R=PA11/12/15/PB0  TIM4+TIM1\r\n",
                  STEPPER_ENABLE ? "ENABLED" : "DISABLED(!)");
    Serial_Printf("   mode=%s  %lu steps/rev  wheel=%umm  start=%uHz max=%uHz\r\n",
                  (STEPPER_FULLSTEP_EN ? "FULL-4" : "HALF-8"),
                  (unsigned long)(STEPPER_FULLSTEP_EN
                                  ? STEPPER_FULLSTEPS_PER_REV
                                  : STEPPER_HALFSTEPS_PER_REV),
                  (unsigned)TRACK_WHEEL_DIAMETER_MM,
                  (unsigned)STEPPER_START_HZ,
                  (unsigned)STEPPER_HZ_MAX);
    Serial_Printf("   thermal: run<=%ums cooldown=%ums overlap<=%ums\r\n",
                  (unsigned)STEPPER_MAX_RUN_MS,
                  (unsigned)STEPPER_COOLDOWN_MS,
                  (unsigned)STEPPER_MAX_OVERLAP_MS);

    Serial_Printf("========================================\r\n");
    Serial_Printf(" ready: waiting for frame A5 01/08/06 ...\r\n\r\n");
}

static void status_task(uint32_t now_ms)
{
#if DEBUG_STATUS_PRINT
    static uint32_t last = 0u;
    static uint8_t  inited = 0u;

    if (!inited)
    {
        last = now_ms;
        inited = 1u;
        return;
    }

    if ((uint32_t)(now_ms - last) >= DEBUG_STATUS_PERIOD_MS)
    {
        last = now_ms;
        Serial_Printf("[stat] up=%lus good=%lu bad=%lu rx=%lu ovf=%lu fault=0x%02X %s\r\n",
                      (unsigned long)(now_ms / 1000u),
                      (unsigned long)LinkSvc_GetGoodFrameCount(),
                      (unsigned long)LinkSvc_GetBadFrameCount(),
                      (unsigned long)Serial_GetRxTotalCount(),
                      (unsigned long)Serial_GetRxOverflowCount(),
                      LinkSvc_GetFault(),
                      LinkSvc_IsCommLost() ? "LOST" : "LINK");
    }
#else
    (void)now_ms;
#endif
}

int main(void)
{
    uint32_t now;

    /* ---------- M0：板级基础 ---------- */
    BSP_NvicInit();          /* 优先级分组必须先设 */
    BSP_LedInit();
    BSP_IwdgInit();          /* 尽早启动看门狗 */

    /* ---------- M1：链路（硬件初始化）---------- */
    Serial_Init();

    /* ---------- M2.5：配置 ----------
     * ⚠️ 必须在 LinkSvc_Init 之前加载，因为
     *    LinkSvc_Init 要依据配置判断是否进入盲走模式。*/
    Config_Load();

    /* ---------- M1：链路（服务层，依赖配置）---------- */
    LinkSvc_Init();

    /* ---------- M3：舵机（受 CFG_SERVO_EN 控制）----------
     * 关闭时 Servo_Init 内部直接返回，不初始化 TIM2、不占用 PA0。 */
    (void)Servo_Init();

    /* ---------- M4/M5：步进 ×2 ----------
     * 会做 JTAG 重映射释放 PA15/PB3/PB4；四相输出全部拉低（静止断电）。
     * STEPPER_ENABLE=0 时完全不初始化，电机绝对不会动。 */
#if STEPPER_ENABLE
    Stepper_Init();
#endif

    /* ---------- M0：1 ms 调度时基 ---------- */
    BSP_TickInit();

    /* 开机自检（此时 SysTick 未被占用，可用 Delay）*/
    print_banner();
    print_selftest();

    /* 上电指示：快闪 3 次，便于肉眼确认固件跑起来了。
     * 循环里喂狗，避免串口打印耗时不确定时误复位。*/
    {
        uint8_t i;
        for (i = 0u; i < 3u; i++)
        {
            BSP_IwdgFeed();
            BSP_LedOn();
            Delay_ms(80u);
            BSP_IwdgFeed();
            BSP_LedOff();
            Delay_ms(120u);
        }
        BSP_IwdgFeed();
    }

    /* ---------- 主循环 ---------- */
    while (1)
    {
        now = BSP_GetTickMs();

        /* 看门狗：每轮喂一次。若这里卡死，IWDG 约 1.6 s 复位 */
        BSP_IwdgFeed();

        /* 收帧 -> 解析 -> 分发 */
        LinkSvc_Poll(now);

        /* 心跳超时 -> 置故障 */
        LinkSvc_HeartbeatTask(now);

        /* 步进加减速斜坡 + 热保护（1 ms 粒度）*/
        Stepper_Task1ms(now);

        /* 上报 MOTION_DONE / 热保护跳闸 */
        LinkSvc_TaskMotion();

        /* 舵机缓动（非阻塞，内部按 SERVO_EASE_TICK_MS 推进） */
        Servo_Update(now);

        /* ★ 安全联锁（说明书 §5.3）：只要存在任何故障，立刻停止一切运动。
         *   覆盖：心跳丢失 / 桌沿 / 障碍 / 热保护 / 盲走模式。
         *   步进是"停在原地"而不是"回中"，因为桌沿附近回退更危险；
         *   只有舵机在 M11 才会做回中处理。*/
        if (LinkSvc_GetFault() != FAULT_NONE)
        {
            if (Stepper_IsAnyBusy())
            {
                Stepper_StopAll();
            }
        }

        /* LED 提示（三种节奏，肉眼可分辨）：
         *   上电后从未收到过上位机帧 -> 双闪+长停（WAIT，等待上位机）
         *   曾通信过又丢失 / 其它故障 -> 5 Hz 快闪（FAULT）
         *   通信正常                  -> 1 Hz 慢闪（OK）
         * 说明：上电不接上位机时固件会按安全策略置 heartbeat 故障，
         *       但"没接"和"掉了"是两件事，因此这里用不同节奏区分。*/
        if (LinkSvc_GetFault() != FAULT_NONE)
        {
            if (LinkSvc_HasEverCommed())
            {
                BSP_LedRunPattern(now, BSP_LED_PATTERN_FAULT);
            }
            else
            {
                BSP_LedRunPattern(now, BSP_LED_PATTERN_WAIT);
            }
        }
        else
        {
            BSP_LedRunPattern(now, BSP_LED_PATTERN_OK);
        }

        /* 调试状态行 */
        status_task(now);
    }
}
