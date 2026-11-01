/**
  ******************************************************************************
  * @file    User/main.c
  * @brief   A733 桌宠 下位机运动控制
  *
  * 已完成阶段（对应项目说明书 §10）：
  *   M0   时钟 72 MHz / GPIO / TIM3 时基 / IWDG / LED 心跳
  *   M1   USART1 收帧 + CRC16 + ACK/PONG
  *   M2   心跳超时检测 + 看门狗
  *   M2.5 Flash 配置持久化 + SET_CONFIG / GET_CONFIG
  *   M3   SG90 舵机（TIM2_CH1，软件限幅 ±45°）
  *   M4/M5 28BYJ-48 双步进（TIM4 + TIM1_UP，差速/加减速/热保护）
  *   M6/M7 VL53L0X ×4（I2C1 + XSHUT 逐颗改址，桌沿/障碍）
  *   BT    USART3 蓝牙独立遥控 + 双命令源仲裁
  *   M9   USART2 上的 TW-TTS 转发（中断发送，不阻塞主循环）
  *
  * 纪律：编译通过不等于通过。验收判据见说明书 §10 与 docs/ 下各阶段说明。
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
#include "tts.h"
#include "tof.h"
#include "tof_service.h"
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
    Serial_Printf(" stage: M0 ~ M9  (motion + TOF + BT + TTS)\r\n");
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
    /* 蓝牙：打印【实际初始化结果】而不是配置期望值 ——
     * 配置开了但初始化没做，说明 BT_LINK_ENABLE=0，一眼能看出来。 */
    if (SerialBt_IsReady() != 0u)
    {
        Serial_Printf(" bt     : USART3 %lu 8N1  PB10(TX)/PB11(RX)  [READY]\r\n",
                      (unsigned long)BT_UART_BAUDRATE);
    }
    else
    {
        Serial_Printf(" bt     : disabled (CFG_BT_EN=%u, BT_LINK_ENABLE=%u)\r\n",
                      Config_IsEnabled(CFG_BT_EN), (unsigned)BT_LINK_ENABLE);
    }
    /* TTS：同样打印实际初始化结果。
     * 单独一行，因为它是【单向输出】通道，接线错了不会报错，
     * 只会"没声音" —— 这行是排查 TTS 哑巴问题的第一手信息。 */
    if (Tts_IsReady() != 0u)
    {
        Serial_Printf(" tts    : USART2 %lu 8N1  PA2(TX)/PA3(RX)  [READY]\r\n",
                      (unsigned long)TTS_UART_BAUDRATE);
    }
    else
    {
        Serial_Printf(" tts    : disabled (CFG_TTS_EN=%u, TTS_LINK_ENABLE=%u)\r\n",
                      Config_IsEnabled(CFG_TTS_EN), (unsigned)TTS_LINK_ENABLE);
    }
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
    else if (Config_GetBottomTofCount() < 3u)
    {
        /* 只有 1~2 颗底部传感器：桌沿保护有方向盲区。
         * 必须显式提示 —— 否则容易误以为整圈都有保护。 */
        Serial_Printf(" WARN   : bottom TOF only %u/3 -> "
                      "BACKWARD IS UNPROTECTED\r\n",
                      (unsigned)Config_GetBottomTofCount());
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

    /* TOF：这里打印的是【实测】的启用/在线位图，不是编译期常量。
     * 如果 enable 有 bit 但 online 对应 bit 为 0，说明那颗没起来
     * （接线 / 供电 / XSHUT 下拉 / 地址分配失败）。 */
    if (Tof_GetEnableMask() == 0u)
    {
        Serial_Printf(" tof    : DISABLED (all 4 off)\r\n");
    }
    else
    {
        Serial_Printf(" tof    : en=0x%X online=0x%X  poll=%ums  edge>%dmm obs<%dmm\r\n",
                      (unsigned)Tof_GetEnableMask(),
                      (unsigned)Tof_GetOnlineMask(),
                      (unsigned)Tof_GetPollMs(),
                      (int)TOF_EDGE_MM, (int)TOF_OBSTACLE_MM);
        if (Tof_GetOnlineMask() == 0u)
        {
            Serial_Printf(" WARN   : TOF enabled but NONE online -> fault 0x05\r\n");
        }
    }

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

    /* ---------- 蓝牙链路（USART3，PB10/PB11）----------
     * 目的：A7Z 不在场时仍能遥控。
     * 只有配置字里 BT_EN=1 才初始化；关掉时 PB10/PB11 保持普通 IO，
     * 不占中断、不占缓冲，也不会因为模块没插而产生噪声中断。
     *
     * ⚠️ 模块必须先把波特率改到 115200（AT+UART=115200,0,0），
     *    出厂默认 9600 会与本配置不匹配，表现为收到乱码。 */
#if BT_LINK_ENABLE
    if (Config_IsEnabled(CFG_BT_EN) != 0u)
    {
        SerialBt_Init();
    }
#endif

    /* ---------- M9：TTS 转发（USART2，PA2/PA3）----------
     * A7Z 的 UART4 已交给运动链路，TTS 模块改挂 MCU。
     * 关闭时完全不初始化 USART2，PA2/PA3 保持普通 IO。
     * 9600 波特下用中断发送，Relay 只入队，不阻塞主循环。 */
#if TTS_LINK_ENABLE
    if (Config_IsEnabled(CFG_TTS_EN) != 0u)
    {
        Tts_Init();
    }
#endif

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

    /* ---------- M6/M7：TOF ×4 ----------
     * ⚠️ 必须放在 BSP_TickInit 之后：VL53L0X 的初始化与参考校准
     *    用 BSP_GetTickMs() 做超时判断，没有 tick 会死等。
     * 整个过程约 1 秒（4 颗 × 地址分配 + 完整调参 + 两次参考校准），
     * 内部会反复喂看门狗。 */
    TofSvc_Init();

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

        /* TOF 轮询 + 桌沿/障碍判定 + 故障联动（非阻塞，每次只推进一颗） */
        TofSvc_Poll(now);

        /* TTS 兜底重启（正常情况下 TXE 中断自己抽干缓冲，这里只防异常） */
        Tts_Poll();

        /* ★ 安全联锁（说明书 §5.3）：只要存在任何故障，立刻停止一切运动。
         *   覆盖：心跳丢失 / 桌沿 / 障碍 / 热保护 / 盲走模式。
         *   步进是"停在原地"而不是"回中"，因为桌沿附近回退更危险；
         *   只有舵机在 M11 才会做回中处理。*/
        if (Stepper_IsAnyBusy())
        {
            uint8_t f     = LinkSvc_GetFault();
            uint8_t d     = LinkSvc_GetLastDir();
            uint8_t do_stop = 1u;

            if (f == FAULT_NONE)
            {
                do_stop = 0u;
            }
            else if (f == FAULT_EDGE)
            {
                /* 桌沿故障：允许"后退脱困"，也允许 TOF 自己的自动回退。
                 * 其余方向一律停 —— 再往前走就是掉下去。 */
                if ((d == MOVE_DIR_BACKWARD) ||
                    (TofSvc_IsBackoffActive() != 0u))
                {
                    do_stop = 0u;
                }
            }
            else if (f == FAULT_OBSTACLE)
            {
                /* 障碍故障：只禁止前进，后退与转向允许（便于自行脱困）*/
                if (d != MOVE_DIR_FORWARD)
                {
                    do_stop = 0u;
                }
            }

            if (do_stop != 0u)
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
