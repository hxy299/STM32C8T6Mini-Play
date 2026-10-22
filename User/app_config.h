/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   全局可调参数集中处（M0-M2.5 阶段）
  *
  * 说明：
  *   - 本文件只放"编译期常量与默认值"，运行时开关（舵机/TOF/蓝牙/TTS）
  *     走 Service/config.c 的配置字，不放这里。
  *   - 任何引脚改动必须先回到项目说明书 §四 的引脚分配表确认无冲突。
  ******************************************************************************
  */
#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/* ============================ 系统时钟 ============================ */
/* HSE 8 MHz × 9 = 72 MHz（system_stm32f10x.c 已默认 SYSCLK_FREQ_72MHz）*/
#define APP_EXPECTED_SYSCLK     72000000u

/* ============================ A7Z 链路 ============================ */
/* 与 A7Z 的 UART4 对接：USART1, PA9(TX)/PA10(RX) */
#define LINK_USART              USART1
#define LINK_USART_CLK          RCC_APB2Periph_USART1
#define LINK_UART_BAUDRATE      115200u
#define LINK_USART_IRQn         USART1_IRQn
#define LINK_RX_RING_SIZE       256u    /* 2 的幂，中断写入 */

/* ============================ 协议常量 ============================ */
#define LINK_SOF                0xA5u
#define LINK_MAX_PAYLOAD        200u
#define LINK_MAX_FRAME          (6u + LINK_MAX_PAYLOAD + 2u)  /* 208 */

/* 帧间超时：超过该时间没有新字节，则丢弃半帧。
 * 作用：防止噪声里的假 SOF（后面跟着一个大 LEN）把紧随其后的真帧吞掉。
 * 115200 下一个 13 字节帧约 1.2 ms，100 ms 余量极大，不影响正常通信。 */
#define LINK_INTERBYTE_TIMEOUT_MS  100u

/* 帧类型 */
#define LINK_TYPE_CMD           0x01u
#define LINK_TYPE_ACK           0x02u
#define LINK_TYPE_EVT           0x03u

/* 命令操作码（A7Z -> MCU）*/
#define OP_MOVE                 0x01u
#define OP_SERVO                0x02u
#define OP_TTS                  0x03u
#define OP_TOF_CFG              0x04u
#define OP_CLEAR_FAULT          0x05u
#define OP_PING                 0x06u
#define OP_SET_CONFIG           0x07u
#define OP_GET_CONFIG           0x08u
#define OP_STEPPER_STAT         0x09u   /* 诊断：读回电机运行状态 */
#define OP_LINK_STAT            0x0Au   /* 诊断：链路健康计数器 */
#define OP_STEPPER_TEST         0x0Bu   /* 标定：只驱动单侧电机（原始方向） */
#define OP_MOTOR_STOP           0x0Cu   /* 独立停止命令：立即切断两路步进 */

/* ===== 功能实现位图（PONG 的第 11 字节）=====
 * 作用：让上位机一眼看出"板子里跑的固件到底实现了哪些命令"，
 *       彻底避免"以为刷了新固件、其实还是旧的"这种误判。
 * 新增功能时在这里加一位。
 */
#define IMPL_SERVO              (1u << 0)   /* M3：SERVO 已实现 */
#define IMPL_MOVE               (1u << 1)   /* M4/M5：MOVE 已实现 */
#define IMPL_TTS                (1u << 2)   /* M9：TTS 已实现 */
#define IMPL_TOF                (1u << 3)   /* M7：TOF_CFG 已实现 */
#define IMPL_CONFIG             (1u << 4)   /* M2.5：SET/GET_CONFIG */
#define IMPL_CLEAR_FAULT        (1u << 5)   /* M2.5：CLEAR_FAULT */

/* 当前固件实现的能力集合 */
#define IMPL_MASK_CURRENT       (IMPL_SERVO | IMPL_MOVE | IMPL_CONFIG | IMPL_CLEAR_FAULT)

/* MOVE 命令的方向字节 */
#define MOVE_DIR_FORWARD        0x00u
#define MOVE_DIR_BACKWARD       0x01u
#define MOVE_DIR_TURN_LEFT      0x02u
#define MOVE_DIR_TURN_RIGHT     0x03u
#define MOVE_DIR_STOP           0x04u

/* MOVE 的 steps 字段取该值时表示"持续运动，直到 STOP 或热保护截断" */
#define MOVE_STEPS_CONTINUOUS   0xFFFFu

/* 应答操作码（MCU -> A7Z）*/
#define OP_ACK                  0x81u
#define OP_NAK                  0x82u
#define OP_PONG                 0x86u
#define OP_CONFIG               0x87u
#define OP_EVT_FAULT            0x93u
#define OP_EVT_CONFIG_CHANGED   0x96u
#define OP_EVT_MOTION_DONE      0x94u

/* 状态码 */
#define ST_OK                   0x00u
#define ST_CLAMPED              0x01u
#define ST_BUSY                 0x02u
#define ST_FAULT_LATCHED        0x03u
#define ST_CRC_ERROR            0x04u
#define ST_UNKNOWN_OP           0x05u
#define ST_FEATURE_DISABLED     0x06u
#define ST_FLASH_FAIL           0x07u

/* 故障码 */
#define FAULT_NONE              0x00u
#define FAULT_HEARTBEAT_LOST    0x01u
#define FAULT_EDGE              0x02u
#define FAULT_OBSTACLE          0x03u
#define FAULT_THERMAL           0x04u
#define FAULT_TOF_INIT          0x05u
#define FAULT_NO_EDGE_PROTECT   0x0Au
#define FAULT_BLIND_ACK         0x5Au  /* 盲走确认魔数 */

/* ============================ 心跳 / 超时（ms）============================ */
#define HB_TIMEOUT_STOP_MS      1000u   /* 无有效帧 -> 停止动作 */
#define HB_TIMEOUT_BLINK_MS     1500u   /* 无有效帧 -> LED 快闪 */
#define APP_BOOT_BANNER_MS      3000u   /* 上电后放宽心跳的宽限时间 */

/* ============================ 调试 ============================ */
/* 1 = 在二进制协议帧之外额外打印可读文本（仅 bring-up 用）。
 * 联调 A7Z 时改为 0，避免文本混进二进制流。
 *
 * ⚠️ 2026-10-22 起改为 0：Serial_Printf 是【阻塞发送】，
 *    在 handle_move 里且在 ACK 之前执行，会推迟 ACK 并占用主循环。
 *    协议联调阶段必须关掉，只在单独排查硬件时临时打开。 */
#define APP_DEBUG_TEXT          0

/* ============================ LED ============================ */
/* 板载 LED：PC13，低电平点亮（Blue Pill 典型接法）*/
#define LED_GPIO_PORT           GPIOC
#define LED_GPIO_CLK            RCC_APB2Periph_GPIOC
#define LED_GPIO_PIN            GPIO_Pin_13
#define LED_ACTIVE_LOW          1

#define LED_HEARTBEAT_MS        500u    /* 正常：1 Hz 闪烁（500ms 翻转） */
#define LED_FAULT_MS            100u    /* 故障：5 Hz 快闪 */

/* ============================ M3：SG90 舵机 ============================ */
/* 引脚：PA0 = TIM2_CH1（默认复用功能，无需重映射）
 * 定时器：TIM2，APB1=36MHz 但定时器时钟翻倍 72MHz
 *   PSC = 72-1  -> 1 MHz（1 tick = 1 us）
 *   ARR = 20000-1 -> 20 ms -> 50 Hz
 *   CCR 直接就是脉宽微秒数
 */
#define SERVO_TIM               TIM2
#define SERVO_TIM_CLK           RCC_APB1Periph_TIM2
#define SERVO_TIM_CHANNEL       TIM_Channel_1
#define SERVO_GPIO_PORT         GPIOA
#define SERVO_GPIO_CLK          RCC_APB2Periph_GPIOA
#define SERVO_GPIO_PIN          GPIO_Pin_0

#define SERVO_TIM_PRESCALER     72u       /* -> 1 MHz */
#define SERVO_TIM_PERIOD        20000u    /* -> 20 ms (50 Hz) */

#define SERVO_CENTER_US         1500u     /* 中位脉宽 */
#define SERVO_HALF_RANGE_US     500u      /* ±45° 对应 ±500 us */
#define SERVO_MAX_DEG           45        /* 软件限幅上限 */
#define SERVO_MIN_DEG           (-45)     /* 软件限幅下限 */

#define SERVO_EASE_TICK_MS      10u       /* 缓动插值步进周期 */
#define SERVO_EASE_DEFAULT_MS   300u      /* 未指定 ms 时的默认缓动时长 */

/* ============================ M4/M5：28BYJ-48 步进 ×2 ============================ */
/* 相序线：每个电机 4 根，经 ULN2003 驱动
 *   电机 A（左）：PB1  / PB3  / PB4  / PB8    ← 实测确认
 *   电机 B（右）：PA11 / PA12 / PA15 / PB0    ← 实测确认
 *
 * ⚠️ PA15 / PB3 / PB4 默认是 JTAG 引脚，必须执行
 *    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE)
 *    才能当普通 IO 用；PA13/PA14 的 SWD 仍保留，不影响下载调试。
 */
/*  ★ 2026-10-22 实测修正：原先把 L/R 的物理归属标反了。
 *    单侧标定结果（dcal）：
 *      PA11/12/15/PB0 这组 "+" = 后  -> 它其实是【右】电机
 *      PB1/3/4/8   这组 "+" = 前  -> 它其实是【左】电机
 *    所以把两组对调，让 STEPPER_L 真正对应机器人左侧。
 */
#define STEPPER_A_PORTS     { GPIOB, GPIOB, GPIOB, GPIOB }   /* 左 */
#define STEPPER_A_PINS      { GPIO_Pin_1, GPIO_Pin_3, GPIO_Pin_4, GPIO_Pin_8 }
#define STEPPER_B_PORTS     { GPIOA, GPIOA, GPIOA, GPIOB }   /* 右 */
#define STEPPER_B_PINS      { GPIO_Pin_11, GPIO_Pin_12, GPIO_Pin_15, GPIO_Pin_0 }

/* 步进节拍定时器（每个电机一个独立时基，便于差速）*/
#define STEPPER_A_TIM       TIM4            /* APB1，72 MHz 定时器时钟 */
#define STEPPER_A_TIM_CLK   RCC_APB1Periph_TIM4
#define STEPPER_A_IRQn      TIM4_IRQn
#define STEPPER_B_TIM       TIM1            /* APB2，72 MHz */
#define STEPPER_B_TIM_CLK   RCC_APB2Periph_TIM1
#define STEPPER_B_IRQn      TIM1_UP_IRQn

/* 定时器基准：PSC = 72-1 -> 1 MHz，ARR = 1000000/hz - 1 */
#define STEPPER_TIM_PRESCALER   72u
#define STEPPER_TIM_TICK_HZ     1000000u

/* ---------- 机械与运动参数（★ 上板后需要实测标定）---------- */
/* 28BYJ-48：内部 32 整步/转 × 减速比 63.68 ≈ 2038 整步/输出圈
 * 8 拍半步模式下为 4076 半步/输出圈
 *
 * ⚠️ 修正：我早先在说明书里写的 "2048 半步/圈" 是错的（少了 2 倍），
 *    那会把实际速度算成 2 倍。正确值见下。
 */
#define STEPPER_FULLSTEPS_PER_REV   2038u
#define STEPPER_HALFSTEPS_PER_REV   4076u

/* ---------- 步进总开关 ----------
 * 0 = 完全不初始化步进：不配 GPIO、不开定时器、不输出任何相位。
 *     用于"电机乱转/停不住"时先断电排查，烧进去保证电机绝对不动。
 * 1 = 正常使用。
 */
#define STEPPER_ENABLE              1

/* ---------- 步进模式 ----------
 * 0 = 8 拍半步：4076 步/圈，平滑、分辨率高、**慢**
 * 1 = 4 拍全步：2038 步/圈，**速度翻倍**、两相通电扭矩更大、分辨率减半
 *
 * 桌宠嫌慢就把这里设 1。实测推荐值。
 */
#define STEPPER_FULLSTEP_EN         1

/* 履带驱动轮直径（mm），用于把半步换算成距离；装好后实测填这里 */
#define TRACK_WHEEL_DIAMETER_MM     30u

/* ---------- 左右电机的装配方向符号 ----------
 * 左右电机面对面安装，通同样电流时【电气方向相同、物理转向相反】。
 * 这个符号用来抵消镜像，让两侧朝同一个物理方向走。
 *
 * 判断方法：
 *   发 fwd 200，如果两个轮子【反向转】（一个前一个后），
 *   就把其中一侧改成 -1。改哪一侧都可以，效果一样。
 *
 * 注意：Stepper_GetPosition() 会把读数换算到"机器人前进为正"的坐标系，
 *       所以改了这个符号之后，sst / MOTION_DONE 里的左右位置仍然相等。
 */
/*  ★ 实测标定结果（2026-10-22，本机装配）：
 *      左 = -1，右 = +1
 *
 *  含义：STEPPER_x_DIR_SIGN 表示"该侧电机的原始正方向"对应
 *        机器人的【前进】方向。取 -1 就是"该侧电机正转其实是后退"。
 *
 *  为什么两个值都要定、不能只翻一个：
 *    左右电机面对面安装，镜像效应本身需要一次取反；
 *    而"前进到底是哪一边"又取决于你接线时哪根是 IN1。
 *    两者叠加出来的最终结果是 (-1, +1)。
 *
 *  改了这对符号后：
 *    · fwd / bck      前后方向正确
 *    · lft / rgt      转向方向【不受影响】（两符号同时翻转，抵消了）
 *    · sst 的 position 仍换算到"前进为正"的坐标系，左右恒相等
 *
 *  如果以后重新接线，判断方法：
 *    发 fwd 500，看机器人是前进还是后退；
 *    后退就把【两个】符号同时取反。
 */
#define STEPPER_L_DIR_SIGN          (+1)   /* 左侧：原始 + 即前进 */
#define STEPPER_R_DIR_SIGN          (-1)   /* 右侧：原始 + 是后退，故取反 */

/* ---------- 速度档位（半步/秒）---------- */
#define STEPPER_START_HZ            120u    /* 起步频率：必须从低速开始，否则丢步 */
#define STEPPER_HZ_SLOW             200u
#define STEPPER_HZ_NORMAL           300u    /* 默认档 */
#define STEPPER_HZ_FAST             600u    /* 上限：再高会丢步且 ULN2003 更热 */
#define STEPPER_HZ_MIN              60u
#define STEPPER_HZ_MAX              700u    /* 只给扫频留头，正常档位到 600 */

/* ---------- 速度档编号（MOVE 命令的 speed 字节）----------
 * 放在这里而不是 stepper.h：因为 stepper_math.h 也要用这几个宏，
 * 而 stepper_math.h 不能包含 stepper.h（会循环依赖）。
 */
#define STEPPER_SPEED_0             0       /* 120 Hz 蜗速 */
#define STEPPER_SPEED_1             1       /* 200 Hz */
#define STEPPER_SPEED_2             2       /* 300 Hz 默认 */
#define STEPPER_SPEED_3             3       /* 600 Hz 上限 */
#define STEPPER_SPEED_DEFAULT       2

/* 加减速：每毫秒改变多少 Hz（2 -> 120Hz 到 300Hz 约 90ms）*/
#define STEPPER_ACCEL_HZ_PER_MS     4u      /* 原 2 太保守，履带起步没那么娇气 */

/* ---------- 热保护（12V 直驱 ULN2003 的硬约束）----------
 * 12V/50Ω ≈ 240mA/相，ULN2003 单通道约 0.26W；
 * 两电机同时步进约 1.06W，温升 74~95°C，会超限。
 * 参见说明书 §3.3。这里做三重限制：
 *   1) 静止时四相全断电（stepper_hard_stop 里立即拉低）
 *   2) 单个电机连续运转上限，超过强制停 + 冷却
 *   3) 双电机同时运转的重叠时间上限
 */
/* ---------- 电机供电电压：直接决定热保护阈值 ----------
 * ★ 换了电机供电之后必须改这里！
 *
 * 12V 直驱：单相 240 mA / 2.9 W（额定 5V 的 5.8 倍）
 *           两个电机同时跑，ULN2003 温升可达 74~95 °C
 *           → 必须严格限时，所以双机只给 3 秒（默认档仅 20.8 mm！）
 *
 *  5V 降压：单相 100 mA / 0.5 W
 *           温升 25~36 °C，可以几乎连续跑
 *           → 限时放宽到 60 秒，双机不再受限
 *
 * 强烈建议换成 5V（12V→5V 降压模块，≥2A，约 ¥3）：
 * 否则每次直线动作只有 2 cm，桌宠基本没法用。
 */
/* #define STEPPER_SUPPLY_12V          1 */
#define STEPPER_SUPPLY_5V        1 

#ifdef STEPPER_SUPPLY_5V
  #define STEPPER_MAX_RUN_MS        60000u  /* 单次连续运转上限 */
  #define STEPPER_COOLDOWN_MS       500u    /* 强制冷却时间 */
  #define STEPPER_MAX_OVERLAP_MS    60000u  /* 双电机同时运转上限 */
#else
  #define STEPPER_MAX_RUN_MS        8000u
  #define STEPPER_COOLDOWN_MS       2000u
  #define STEPPER_MAX_OVERLAP_MS    3000u
#endif

/* ============================ 看门狗 ============================ */
/* LSI 约 40 kHz，预分频 64 -> 625 Hz；重装 1000 -> 约 1.6 s */
#define APP_IWDG_PRESCALER      IWDG_Prescaler_64
#define APP_IWDG_RELOAD         1000u

/* ============================ 调度时基 ============================ */
/* TIM3：APB1=36MHz，但定时器时钟翻倍为 72MHz
 * PSC=71 -> 1 MHz；ARR=999 -> 1 ms
 * 注意：调度时基用 TIM3，SysTick 留给 Delay_us/ms（见说明书 §13.1）
 */
#define APP_TICK_TIM            TIM3
#define APP_TICK_TIM_CLK        RCC_APB1Periph_TIM3
#define APP_TICK_TIM_IRQn       TIM3_IRQn
#define APP_TICK_PRESCALER      72u
#define APP_TICK_PERIOD         1000u   /* (ARR+1) -> 1 ms */

/* ============================ 配置存储（Flash page 63）============================ */
/* STM32F103C8T6：64 KB Flash，1 KB/page，最后一页 = page 63 */
#define CFG_FLASH_ADDR          0x0800FC00u
#define CFG_FLASH_MAGIC         0x43385436u  /* "C8T6" */
#define CFG_FLASH_VERSION       1u

/* 运行时功能开关位（配置字低 7 位）*/
#define CFG_SERVO_EN            (1u << 0)
#define CFG_TTS_EN              (1u << 1)
#define CFG_BT_EN               (1u << 2)
#define CFG_TOF0_EN             (1u << 3)   /* 底部前：桌沿 */
#define CFG_TOF1_EN             (1u << 4)   /* 底部左：桌沿 */
#define CFG_TOF2_EN             (1u << 5)   /* 底部右：桌沿 */
#define CFG_TOF3_EN             (1u << 6)   /* 前脸：障碍 */

/* 默认：舵机/蓝牙关闭，TTS/TOF 开启（见说明书 §2.3、§5.6.1）*/
#define CFG_DEFAULT             (CFG_TTS_EN | CFG_TOF0_EN | CFG_TOF1_EN | \
                                 CFG_TOF2_EN | CFG_TOF3_EN)

/* 底部三颗 TOF 的掩码，用于盲走判定 */
#define CFG_BOTTOM_TOF_MASK     (CFG_TOF0_EN | CFG_TOF1_EN | CFG_TOF2_EN)

#endif /* __APP_CONFIG_H */
