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

/* ============================ 蓝牙链路（USART3）============================
 * 目的：A7Z 不在场时，STM32 仍能被蓝牙遥控独立操作。
 *
 * 引脚：PB10 = USART3_TX → 模块 RX
 *       PB11 = USART3_RX ← 模块 TX
 *       两者都是 USART3 的默认引脚，无需重映射。
 *
 * ⚠️ 波特率：HC-05 / HC-06 出厂默认 9600，必须先用 AT 命令改到 115200：
 *      AT+UART=115200,0,0
 *    原因：9600 下发送一个 20 字节应答帧要 20 ms 阻塞，
 *    这是双链路场景下"程序卡死"最直接的来源。
 */
#define BT_LINK_ENABLE          1
#define BT_USART                USART3
#define BT_USART_CLK            RCC_APB1Periph_USART3
#define BT_USART_IRQn           USART3_IRQn
#define BT_GPIO_PORT            GPIOB
#define BT_GPIO_CLK             RCC_APB2Periph_GPIOB
#define BT_TX_PIN               GPIO_Pin_10
#define BT_RX_PIN               GPIO_Pin_11

/* 115200：一个 20 字节应答帧约 1.7 ms 阻塞发送，可接受。
 * 若模块改不了、被迫用 9600，必须保持 BT_VERBOSE_EVENTS = 0。 */
#define BT_UART_BAUDRATE        115200u
#define BT_RX_RING_SIZE         128u    /* 2 的幂 */

/* 蓝牙多久没有有效帧就算"失联"（释放控制权）。
 * 比 A7Z 的 1000 ms 宽松：蓝牙是人工遥控，节奏比机器慢。 */
#define BT_TIMEOUT_MS           2000u

/* 是否给蓝牙端口发高频事件（TOF_DATA 每 200 ms 一条）。
 * 0 = 不发。蓝牙只是遥控通道，不需要遥测流；
 *     而且低波特率下这条流会把发送端堵死。 */
#define BT_VERBOSE_EVENTS       0

/* ============================ M9：TTS 语音模块（USART2）============================
 * 架构（说明书 §1.1「TTS 帧转发」）：
 *   A7Z 的 UART4 已经完全交给运动链路，TTS 模块改挂到 STM32 上，
 *   A7Z 只把 TW-TTS 原始帧塞进 OP_TTS 的负载，由 MCU 原样转发。
 *
 * 引脚：PA2 = USART2_TX → 模块 RX
 *       PA3 = USART2_RX ← 模块 TX（模块通常不回数据，留着以便以后读状态）
 *
 * ★ 9600 8N1 是 TW-TTS 模块的固定波特率，只有 960 字节/秒。
 *   一条 60 字的中文句 ≈ 190 字节 ≈ 200 ms 发完。
 *
 *   ⚠️ 所以【绝对不能同步发送】。同步发一帧就把 1 ms 步进节拍和心跳
 *      全卡住 200 ms，直接违反说明书 §1.2 原则 4「MCU 不阻塞」。
 *      本模块用 TX 环形缓冲 + 中断发送，Relay 只做入队（微秒级）。
 */
#define TTS_LINK_ENABLE         1
#define TTS_USART               USART2
#define TTS_USART_CLK           RCC_APB1Periph_USART2
#define TTS_USART_IRQn          USART2_IRQn
#define TTS_GPIO_PORT           GPIOA
#define TTS_GPIO_CLK            RCC_APB2Periph_GPIOA
#define TTS_TX_PIN              GPIO_Pin_2
#define TTS_RX_PIN              GPIO_Pin_3
#define TTS_UART_BAUDRATE       9600u

/* TX 环大小必须 2 的幂。512 字节约等于 2.5 条长句，
 * 用来吸收 A7Z 分词后连续下发的几句话之间的抖动。 */
#define TTS_TX_RING_SIZE        512u
#define TTS_RX_RING_SIZE        64u

/* 单帧上限 = LINK_MAX_PAYLOAD（200）。
 * TW-TTS 文本帧头 5 字节，因此能承载的 UTF-8 正文最长 195 字节
 * （约 65 个汉字）。A7Z 侧的 SentenceStream 本来就是按句切的。 */
#define TTS_FRAME_MAX           200u
#define TTS_TEXT_MAX            190u    /* 正文上限，留 5 字节头 */

/* 返回码 */
#define TTS_OK                  0
#define TTS_ERR_NOTREADY       (-1)
#define TTS_ERR_TOOLONG        (-2)
#define TTS_ERR_BUSY           (-3)

/* ============================ 双端口调度预算 ============================
 * 每个端口每次主循环最多处理多少字节。
 *
 * ★ 这是"防止程序卡死"的核心：
 *   如果蓝牙端口被噪声刷屏，没有配额限制时主循环会一直在这里解析，
 *   A7Z 处理和步进任务全被饿死 —— 表现就是整机卡住。
 *   加配额后单个端口再吵也最多吃掉这么多，剩下的留到下一轮。
 */
#define LINK_PORT_BYTE_BUDGET   32u

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
#define IMPL_MASK_CURRENT       (IMPL_SERVO | IMPL_MOVE | IMPL_CONFIG | \
                                 IMPL_CLEAR_FAULT | IMPL_TOF | IMPL_TTS)

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
#define OP_EVT_EDGE_DETECTED    0x91u   /* M7：桌沿触发  sensor(1) dist_mm(2) */
#define OP_EVT_TOF_DATA         0x92u   /* M7：周期距离  mask(1) d0..d3(2 each) */
#define OP_EVT_OBSTACLE_DETECTED 0x95u  /* M8：障碍触发  dist_mm(2) */

/* 状态码 */
#define ST_OK                   0x00u
#define ST_CLAMPED              0x01u
#define ST_BUSY                 0x02u
#define ST_FAULT_LATCHED        0x03u
#define ST_CRC_ERROR            0x04u
#define ST_UNKNOWN_OP           0x05u
#define ST_FEATURE_DISABLED     0x06u
#define ST_FLASH_FAIL           0x07u
#define ST_ARBITRATION          0x09u   /* 当前控制权不在本通道（另一路在控制）*/

/* ============================ 命令优先级分类 ============================
 * 双命令源（A7Z / 蓝牙）仲裁用。分类只看操作码，与来源无关。
 *
 *   SAFETY  停止、清故障 —— 任何来源、任何时候都执行，不受控制权限制。
 *           设计理由：手动遥控器必须永远能按下"停"。
 *   QUERY   保活/诊断 —— 总是执行，代价小且不改变执行器状态。
 *   CONTROL 运动/配置 —— 只有"当前控制权拥有者"能执行。
 */
#define CMD_CLASS_SAFETY        0u
#define CMD_CLASS_QUERY         1u
#define CMD_CLASS_CONTROL       2u

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

/* ============================ M6/M7：VL53L0X TOF ×4 ============================ */
/* 接口：I2C1，PB6 = SCL / PB7 = SDA，标准模式 100 kHz，4 颗共用一条总线。
 *
 * ⚠️ 4 颗 VL53L0X 出厂地址全是 0x29，上电后同时响应同一地址，
 *    必须靠 XSHUT 逐个唤醒、逐个改地址（见 tof.c 的 Tof_Init）。
 * ⚠️ 每个 XSHUT 必须 10 kΩ 下拉，否则上电瞬间芯片状态不确定，
 *    地址分配会随机失败。
 */
#define I2C1_GPIO_PORT          GPIOB
#define I2C1_GPIO_CLK           RCC_APB2Periph_GPIOB
#define I2C1_SCL_PIN            GPIO_Pin_6
#define I2C1_SDA_PIN            GPIO_Pin_7
#define I2C1_CLK                RCC_APB1Periph_I2C1
#define I2C1_PERIPH             I2C1
#define I2C1_SPEED_HZ           100000u     /* 100 kHz：4 颗共用时更稳 */
#define I2C1_TIMEOUT_LOOPS      60000u      /* 单个事件轮询的超时计数（约 3 ms @72MHz）*/

/* ---------- 传感器拓扑（说明书 §5.3）----------
 *                     [TOF3]      前脸：测障碍（朝前）
 *              [TOF1]       [TOF2] 底部左/右：测桌面（朝下）
 *                     [TOF0]      底部前：测桌面（朝下）
 *
 *  ⚠️ 底部三颗危险方向是【距离变远】（桌沿），
 *     前脸那颗危险方向是【距离变近】（障碍）—— 语义相反，不能共用阈值。
 */
#define TOF_COUNT               4u
#define TOF_EDGE_NUM            3u          /* 0..2 是底部桌沿传感器 */

/* XSHUT 必须接在互相独立的 GPIO 上 */
#define TOF_XSHUT_PORTS         { GPIOA, GPIOA, GPIOA, GPIOB }
#define TOF_XSHUT_PINS          { GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_8, GPIO_Pin_5 }

#define TOF_ADDR_DEFAULT        0x29u       /* 出厂地址（4 颗全同）*/
#define TOF_ADDR_BASE           0x30u       /* 改地址后：TOF0..3 = 0x30..0x33 */

/* 按物理编号固定分配地址（说明书 §5.3.4）：
 * 未启用的传感器只是不初始化、不参与地址分配，已启用的仍保持固定地址，
 * 这样排查时"哪颗是 TOF2"永远是 0x32，不会因为改动启用集而漂移。 */
#define TOF_CFG_BIT(n)          (CFG_TOF0_EN << (n))
#define TOF_IS_EDGE(n)          ((n) < TOF_EDGE_NUM)

/* ---------- 测量参数 ---------- */
#define TOF_TIMING_BUDGET_US    20000u      /* 20 ms（VL53L0X 允许的最小值）*/
#define TOF_MEASURE_TIMEOUT_MS  120u        /* 单次测量超时 */
#define TOF_POLL_DEFAULT_MS     10u         /* 两颗之间的间隔（TOF_CFG 可调）*/
#define TOF_POLL_MIN_MS         10u
#define TOF_POLL_MAX_MS         255u

#define TOF_RANGE_MAX_MM        2000        /* 超过此值视为无效 */
#define TOF_RANGE_INVALID_MM    8190        /* 无回波时寄存器常给出的值 */

/* ---------- 桌沿（底部三颗）：距离变远 = 危险 ---------- */
#define TOF_EDGE_MM             80          /* 超过 -> 判定桌沿 */
#define TOF_EDGE_RECOVER_MM     60          /* 回到此值以下才解除 */
#define TOF_EDGE_DEBOUNCE       3u          /* 连续 3 次确认 */

/* ---------- 障碍（前脸一颗）：距离变近 = 危险 ---------- */
#define TOF_OBSTACLE_MM         150         /* 小于 -> 判定障碍 */
#define TOF_OBSTACLE_RECOVER_MM 200         /* 远离到此值以上才解除 */
#define TOF_OBSTACLE_DEBOUNCE   2u          /* 连续 2 次确认 */

/* ---------- 桌沿触发后的自动回退 ---------- */
#define TOF_EDGE_BACKOFF_STEPS  200         /* 反向回退的步数 */
#define TOF_EDGE_BACKOFF_HZ     120u        /* 回退慢速，避免二次失控 */

/* ---------- 距离上报 ---------- */
/* TOF_DATA 事件周期。太密会占用 115200 链路，太稀上位机看不到实时距离。
 * 18 字节/次 @200ms ≈ 90 B/s，可接受。 */
#define TOF_DATA_PERIOD_MS      200u

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

/* 默认：舵机关闭；TTS/蓝牙/TOF 开启。
 *
 * 蓝牙默认开启的理由：它只是"多一种控制入口"，没插模块时
 * USART3 收不到任何数据，对系统完全无副作用（RX 上拉，无噪声中断）。
 *
 * ⚠️ 已经烧过固件的板子，配置存在 Flash 里，改这个宏【不会生效】。
 *    需要手动发一次：  s 0000004E
 *    或者先清空 Flash 配置页再上电。
 */
/* ★ 2026-11-01 按实际硬件改为【两颗 TOF】：
 *      TOF0 = 底部前 → 桌沿
 *      TOF3 = 前脸   → 障碍
 *    TOF1 / TOF2（底部左 / 右）没有模块，默认关闭。
 *
 *    配置字 = TTS(0x02) | BT(0x04) | TOF0(0x08) | TOF3(0x40) = 0x4E
 *
 *   ⚠️ 只有一颗底部传感器时，桌沿保护【有方向盲区】：
 *      · 前进  → 有保护（TOF0 就在前方底面）
 *      · 后退  → 没有保护！尾部先掉下去，而 TOF0 仍在桌面上方
 *      · 转向  → 只有前向参考，侧向越界发现不了
 *   详见 docs/TOF两传感器配置说明.md
 */
#define CFG_DEFAULT             (CFG_TTS_EN | CFG_BT_EN | \
                                 CFG_TOF0_EN | CFG_TOF3_EN)

/* 补上 TOF1/TOF2 之后（三颗底部全开）改成：
 *   #define CFG_DEFAULT  (CFG_TTS_EN | CFG_BT_EN | CFG_TOF0_EN | \
 *                         CFG_TOF1_EN | CFG_TOF2_EN | CFG_TOF3_EN)   // 0x7E
 */

/* 底部三颗 TOF 的掩码，用于盲走判定 */
#define CFG_BOTTOM_TOF_MASK     (CFG_TOF0_EN | CFG_TOF1_EN | CFG_TOF2_EN)

#endif /* __APP_CONFIG_H */
