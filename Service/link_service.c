/**
  ******************************************************************************
  * @file    Service/link_service.c
  * @brief   帧分发 + 心跳 + 应答
  ******************************************************************************
  */
#include "stm32f10x.h"
#include <string.h>
#include "app_config.h"
#include "serial.h"
#include "link_proto.h"
#include "config.h"
#include "bsp.h"
#include "servo.h"
#include "stepper.h"
#include "link_service.h"

/* 本阶段临时状态码：功能尚未实现（M3+ 逐个替换）*/
#define ST_NOT_IMPLEMENTED   0x08u

/* 运行状态 */
static uint32_t s_last_frame_ms = 0u;
static uint8_t  s_comm_lost = 1u;      /* 上电默认认为未建立通信 */
static uint8_t  s_ever_comm = 0u;      /* 上电后是否收到过至少一帧 */
static uint8_t  s_fault = FAULT_NONE;
static uint8_t  s_fault_reported = 0u;
static uint32_t s_boot_ms = 0u;
static uint8_t  s_boot_captured = 0u;

/* ============================ 内部：发送辅助 ============================ */

static void send_frame(uint8_t type, uint16_t seq, uint8_t op,
                       const uint8_t *payload, uint8_t len)
{
    uint8_t  buf[LINK_MAX_FRAME];
    int      n;

    n = Link_BuildFrame(buf, (uint16_t)sizeof(buf), type, seq, op, payload, len);
    if (n > 0)
    {
        Serial_SendBuffer(buf, (uint16_t)n);
    }
}

void LinkSvc_SendAck(uint16_t seq, uint8_t status)
{
    send_frame(LINK_TYPE_ACK, seq, OP_ACK, &status, 1u);
}

void LinkSvc_SendNak(uint16_t seq, uint8_t status, uint8_t detail)
{
    uint8_t p[2];
    p[0] = status;
    p[1] = detail;
    send_frame(LINK_TYPE_ACK, seq, OP_NAK, p, 2u);
}

void LinkSvc_SendFaultEvent(uint8_t code)
{
    send_frame(LINK_TYPE_EVT, 0u, OP_EVT_FAULT, &code, 1u);
}

/* ============================ 初始化 ============================ */

void LinkSvc_Init(void)
{
    Link_ParserReset();
    s_last_frame_ms = 0u;
    s_comm_lost = 1u;
    s_ever_comm = 0u;
    s_fault = FAULT_NONE;
    s_fault_reported = 0u;
    s_boot_captured = 0u;
    s_boot_ms = 0u;

    /* 若配置里底部三颗 TOF 全关，上电即进入盲走模式（说明书 §5.3.3）*/
    if (Config_IsBottomTofAllDisabled())
    {
        s_fault = FAULT_NO_EDGE_PROTECT;
    }
}

/* ============================ 命令处理 ============================ */

static void handle_ping(const link_frame_t *f)
{
    uint8_t  p[11];
    uint32_t uptime = BSP_GetTickMs();
    uint32_t cfg = Config_Get();
    uint8_t  flags = 0u;

    /* flags: 用于让上位机在一次读里判断启动状态，
     *        不必依赖"看开机横幅"或"对比 uptime"这种容易误判的方式。
     *   bit0 = 配置来自 Flash（0 表示本次上电用的是默认值）
     *   bit1 = 处于盲走模式（底部三颗 TOF 全关）
     */
    if (!Config_IsDefault())
    {
        flags |= 0x01u;
    }
    if (Config_IsBottomTofAllDisabled())
    {
        flags |= 0x02u;
    }

    p[0] = (uint8_t)(uptime & 0xFFu);
    p[1] = (uint8_t)((uptime >> 8) & 0xFFu);
    p[2] = (uint8_t)((uptime >> 16) & 0xFFu);
    p[3] = (uint8_t)((uptime >> 24) & 0xFFu);
    p[4] = s_fault;
    p[5] = (uint8_t)(cfg & 0xFFu);
    p[6] = (uint8_t)((cfg >> 8) & 0xFFu);
    p[7] = (uint8_t)((cfg >> 16) & 0xFFu);
    p[8] = (uint8_t)((cfg >> 24) & 0xFFu);
    p[9] = flags;
    /* 第 11 字节：功能实现位图。
     * 加了这一字节之后，上位机可以立刻判断板子跑的是哪一阶段的固件，
     * 不会再把"没刷进去"误当成"接线/硬件问题"。*/
    p[10] = (uint8_t)IMPL_MASK_CURRENT;

    send_frame(LINK_TYPE_ACK, f->seq, OP_PONG, p, 11u);
}

/**
  * @brief  M3：舵机命令
  * @note   payload: deg(int8) + ms(uint16 LE)  = 3 字节
  *         deg 是【有符号】，范围 ±45，超限钳位并返回 ST_CLAMPED
  */
static void handle_servo(const link_frame_t *f)
{
    int8_t   deg;
    uint16_t ms;
    uint8_t  clamped = 0u;
    int      ret;

    if (f->len < 3u)
    {
        LinkSvc_SendNak(f->seq, ST_UNKNOWN_OP, f->op);
        return;
    }

    if (!Servo_IsEnabled())
    {
        LinkSvc_SendNak(f->seq, ST_FEATURE_DISABLED, f->op);
        return;
    }

    deg = (int8_t)f->payload[0];
    ms  = (uint16_t)((uint16_t)f->payload[1] | ((uint16_t)f->payload[2] << 8));

    ret = Servo_MoveTo(deg, ms, &clamped);
    if (ret != SERVO_OK)
    {
        LinkSvc_SendNak(f->seq, (ret == SERVO_ERR_DISABLED)
                                  ? ST_FEATURE_DISABLED : ST_UNKNOWN_OP,
                        f->op);
        return;
    }

#if APP_DEBUG_TEXT
    Serial_Printf("[servo] deg=%d ms=%u clamped=%u -> target-pulse=%uus\r\n",
                  (int)Servo_GetTargetDeg(), (unsigned)ms, (unsigned)clamped,
                  (unsigned)Servo_DegToPulseUs(Servo_GetTargetDeg()));
#endif

    LinkSvc_SendAck(f->seq, clamped ? ST_CLAMPED : ST_OK);
}

/* MOVE 的方向 -> 左右电机半步数
 * 左右电机装配镜像，因此"前进"对两侧的电气方向由
 * STEPPER_L_DIR_SIGN / STEPPER_R_DIR_SIGN 决定（见 app_config.h）。
 */
static void move_dir_to_steps(uint8_t dir, int32_t steps,
                              int32_t *left, int32_t *right)
{
    *left  = 0;
    *right = 0;

    switch (dir)
    {
    case MOVE_DIR_FORWARD:
        *left  = steps * STEPPER_L_DIR_SIGN;
        *right = steps * STEPPER_R_DIR_SIGN;
        break;

    case MOVE_DIR_BACKWARD:
        *left  = -steps * STEPPER_L_DIR_SIGN;
        *right = -steps * STEPPER_R_DIR_SIGN;
        break;

    case MOVE_DIR_TURN_LEFT:      /* 原地左转：左轮后退，右轮前进 */
        *left  = -steps * STEPPER_L_DIR_SIGN;
        *right = steps * STEPPER_R_DIR_SIGN;
        break;

    case MOVE_DIR_TURN_RIGHT:     /* 原地右转：左轮前进，右轮后退 */
        *left  = steps * STEPPER_L_DIR_SIGN;
        *right = -steps * STEPPER_R_DIR_SIGN;
        break;

    default:
        break;
    }
}

/**
  * @brief  M4/M5：履带运动命令
  * @note   payload: dir(1) + speed(1) + steps(2 LE) = 4 字节
  *           dir   : 0 前进 / 1 后退 / 2 左转 / 3 右转 / 4 停止
  *           speed : 0..3 -> 120/200/300/600 Hz（越界取默认 300）
  *           steps : 半步数；0xFFFF 表示持续运动（由热保护/STOP 截断）
  */
static void handle_move(const link_frame_t *f)
{
    uint8_t  dir;
    uint8_t  speed;
    uint16_t steps_raw;
    int32_t  steps;
    int32_t  left;
    int32_t  right;
    int      ret;

    if (f->len < 4u)
    {
        LinkSvc_SendNak(f->seq, ST_UNKNOWN_OP, f->op);
        return;
    }

    dir       = f->payload[0];
    speed     = f->payload[1];
    steps_raw = (uint16_t)((uint16_t)f->payload[2] |
                           ((uint16_t)f->payload[3] << 8));

    /* ---- STOP ---- */
    if (dir == MOVE_DIR_STOP)
    {
        Stepper_StopAll();
        LinkSvc_SendAck(f->seq, ST_OK);
#if APP_DEBUG_TEXT
        Serial_Printf("[move] STOP\r\n");
#endif
        return;
    }

    if (dir > MOVE_DIR_TURN_RIGHT)
    {
        LinkSvc_SendNak(f->seq, ST_UNKNOWN_OP, f->op);
        return;
    }

    /* ---- 步数 ---- */
    if (steps_raw == MOVE_STEPS_CONTINUOUS)
    {
        steps = 0x3FFFFFFF;      /* 当作"很长"，靠热保护或 STOP 截断 */
    }
    else
    {
        steps = (int32_t)steps_raw;
    }

    if (steps == 0)
    {
        LinkSvc_SendNak(f->seq, ST_UNKNOWN_OP, f->op);
        return;
    }

    move_dir_to_steps(dir, steps, &left, &right);

    ret = Stepper_MoveBoth(left, right, Stepper_SpeedToHz(speed));
    if (ret != STEPPER_OK)
    {
        uint8_t st;
        switch (ret)
        {
        case STEPPER_ERR_BUSY:    st = ST_BUSY;     break;
        case STEPPER_ERR_THERMAL: st = ST_FAULT_LATCHED; break;
        default:                  st = ST_UNKNOWN_OP;    break;
        }
        LinkSvc_SendNak(f->seq, st, f->op);
        return;
    }

#if APP_DEBUG_TEXT
    Serial_Printf("[move] dir=%u speed=%u(%uHz) steps=%ld -> L=%ld R=%ld\r\n",
                  (unsigned)dir, (unsigned)speed,
                  (unsigned)Stepper_SpeedToHz(speed),
                  (long)steps, (long)left, (long)right);
#endif

    /* ACK 统一格式：payload[0] = 状态码，payload[1] = 附加信息（可选）。
     * MOVE 的附加信息是预测距离（0.1 mm），超过 254 时饱和为 0xFF。
     *
     * ⚠️ 早期版本把距离直接放在 payload[0]，导致上位机把它当成状态码，
     *    显示成 status=0x2E / 0x45 / 0xB8 这种鬼值。已修正。 */
    {
        uint8_t  p[2];
        uint32_t mm10 = Stepper_StepsToMm10(steps);

        p[0] = ST_OK;
        p[1] = (mm10 > 254u) ? 0xFFu : (uint8_t)mm10;
        send_frame(LINK_TYPE_ACK, f->seq, OP_ACK, p, 2u);
    }
}

/**
  * @brief  运动完成 / 热保护事件上报（主循环调用）
  */
void LinkSvc_TaskMotion(void)
{
    uint8_t i;

    for (i = 0u; i < (uint8_t)STEPPER_COUNT; i++)
    {
        if (Stepper_PollDone((stepper_id_t)i))
        {
            /* payload: motor_id(1) + position(4 LE) */
            uint8_t p[5];
            int32_t pos = Stepper_GetPosition((stepper_id_t)i);
            p[0] = i;
            p[1] = (uint8_t)(pos & 0xFF);
            p[2] = (uint8_t)((pos >> 8) & 0xFF);
            p[3] = (uint8_t)((pos >> 16) & 0xFF);
            p[4] = (uint8_t)((pos >> 24) & 0xFF);
            send_frame(LINK_TYPE_EVT, 0u, OP_EVT_MOTION_DONE, p, 5u);
        }
    }

    /* ---------- 热故障：按【当前状态】判定，不锁存 ----------
     * 过热是瞬时状态，冷却结束就该自动恢复。
     *
     * ⚠️ 早期版本是"跳闸次数变化就置 FAULT_THERMAL"，而主循环的安全联锁是
     *    "只要 fault != NONE 就 Stepper_StopAll()"，于是：
     *      一次热跳闸 → 故障永久锁存 → 之后每一次 MOVE 都被立刻杀掉
     *    实测表现为"发 MOVE 有 ACK，但电机只动一下/不到 1 秒就停"。
     *
     * 现在改成：只有"确实还在冷却中"才报热故障，冷却一结束自动清除。
     */
    if (Stepper_IsCoolingDown(STEPPER_L) || Stepper_IsCoolingDown(STEPPER_R))
    {
        if (s_fault != FAULT_THERMAL)
        {
            s_fault = FAULT_THERMAL;
            s_fault_reported = 0u;
        }
    }
    else if (s_fault == FAULT_THERMAL)
    {
        s_fault = FAULT_NONE;
    }
}

/**
  * @brief  诊断：读回两个电机的实时运行状态
  * @note   用于排查"发 MOVE 后电机只动一下"这类问题，
  *         可以区分：没启动 / 卡在剩余步数 / 被热保护停 / 正常完成。
  *
  * payload（24 字节）：
  *   [0..3]   L.position   int32（累计半步）
  *   [4..7]   L.remaining  int32（剩余半步，0 = 已停）
  *   [8..9]   L.cur_hz     uint16
  *   [10]     L.flags      bit0 running, bit1 cooling
  *   [11..21] R.*          同 L（11 字节）
  *   [22]     thermal_trip_count（饱和到 255）
  *   [23]     fault 当前值
  */
static void handle_stepper_stat(const link_frame_t *f)
{
    uint8_t  p[24];
    uint8_t  i;
    uint8_t  off;

    memset(p, 0, sizeof(p));

    for (i = 0u; i < (uint8_t)STEPPER_COUNT; i++)
    {
        int32_t  pos = Stepper_GetPosition((stepper_id_t)i);
        int32_t  rem = Stepper_GetRemaining((stepper_id_t)i);
        uint16_t hz  = Stepper_GetCurrentHz((stepper_id_t)i);
        uint8_t  fl  = 0u;

        off = (uint8_t)(i * 11u);

        if (Stepper_IsBusy((stepper_id_t)i))          { fl |= 0x01u; }
        if (Stepper_IsCoolingDown((stepper_id_t)i))   { fl |= 0x02u; }

        p[off + 0u] = (uint8_t)(pos & 0xFF);
        p[off + 1u] = (uint8_t)((pos >> 8) & 0xFF);
        p[off + 2u] = (uint8_t)((pos >> 16) & 0xFF);
        p[off + 3u] = (uint8_t)((pos >> 24) & 0xFF);
        p[off + 4u] = (uint8_t)(rem & 0xFF);
        p[off + 5u] = (uint8_t)((rem >> 8) & 0xFF);
        p[off + 6u] = (uint8_t)((rem >> 16) & 0xFF);
        p[off + 7u] = (uint8_t)((rem >> 24) & 0xFF);
        p[off + 8u] = (uint8_t)(hz & 0xFF);
        p[off + 9u] = (uint8_t)((hz >> 8) & 0xFF);
        p[off + 10u] = fl;
    }

    {
        uint32_t trips = Stepper_GetThermalTripCount();
        p[22] = (trips > 255u) ? 255u : (uint8_t)trips;
    }
    p[23] = s_fault;

    send_frame(LINK_TYPE_ACK, f->seq, OP_STEPPER_STAT, p, 24u);
}

/**
  * @brief  诊断：链路健康计数器
  * @note   用来回答"上位机明明在发帧，为什么下位机还报心跳丢失"这类问题。
  *           rx_bytes 在涨但 good 不涨      -> 帧被 CRC/解析丢掉
  *           rx_overflow 在涨               -> 主循环太慢，环形缓冲溢出
  *           last_frame_age 一直很大        -> 根本没收到
  *
  * payload（26 字节）：
  *   [0..3]   uptime ms
  *   [4..7]   好帧数
  *   [8..11]  坏帧数
  *   [12..15] 收到的字节总数
  *   [16..19] 环形缓冲溢出次数
  *   [20..23] 距上一帧的毫秒数（从未收到过则为 0xFFFFFFFF）
  *   [24]     当前 fault
  *   [25]     comm_lost 标志
  */
static void handle_link_stat(const link_frame_t *f)
{
    uint8_t  p[26];
    uint32_t v;
    uint32_t age;

    if (s_comm_lost)
    {
        age = 0xFFFFFFFFu;
    }
    else
    {
        uint32_t now = BSP_GetTickMs();
        age = (now >= s_last_frame_ms) ? (now - s_last_frame_ms) : 0u;
    }

    v = BSP_GetTickMs();
    p[0]  = (uint8_t)(v & 0xFF);          p[1]  = (uint8_t)((v >> 8) & 0xFF);
    p[2]  = (uint8_t)((v >> 16) & 0xFF);  p[3]  = (uint8_t)((v >> 24) & 0xFF);

    v = Link_GetGoodFrames();
    p[4]  = (uint8_t)(v & 0xFF);          p[5]  = (uint8_t)((v >> 8) & 0xFF);
    p[6]  = (uint8_t)((v >> 16) & 0xFF);  p[7]  = (uint8_t)((v >> 24) & 0xFF);

    v = Link_GetBadFrames();
    p[8]  = (uint8_t)(v & 0xFF);          p[9]  = (uint8_t)((v >> 8) & 0xFF);
    p[10] = (uint8_t)((v >> 16) & 0xFF);  p[11] = (uint8_t)((v >> 24) & 0xFF);

    v = Serial_GetRxTotalCount();
    p[12] = (uint8_t)(v & 0xFF);          p[13] = (uint8_t)((v >> 8) & 0xFF);
    p[14] = (uint8_t)((v >> 16) & 0xFF);  p[15] = (uint8_t)((v >> 24) & 0xFF);

    v = Serial_GetRxOverflowCount();
    p[16] = (uint8_t)(v & 0xFF);          p[17] = (uint8_t)((v >> 8) & 0xFF);
    p[18] = (uint8_t)((v >> 16) & 0xFF);  p[19] = (uint8_t)((v >> 24) & 0xFF);

    p[20] = (uint8_t)(age & 0xFF);        p[21] = (uint8_t)((age >> 8) & 0xFF);
    p[22] = (uint8_t)((age >> 16) & 0xFF); p[23] = (uint8_t)((age >> 24) & 0xFF);

    p[24] = s_fault;
    p[25] = s_comm_lost;

    send_frame(LINK_TYPE_ACK, f->seq, OP_LINK_STAT, p, 26u);
}

/**
  * @brief  标定：只驱动单侧电机，使用【原始相位方向】
  * @note   这是方向标定唯一可靠的手段。
  *
  *         为什么需要它：
  *           fwd/bck/lft/rgt 都是双电机复合运动，一旦任一符号错，
  *           观察到的就是混合结果，无法反推是哪一处错。
  *           只驱动一个电机时，"正数让它往哪走"是唯一确定的事实，
  *           四个事实就能解出全部符号。
  *
  *         payload: motor(1) + steps(int32 LE) = 5 字节
  *           motor : 0 = 左，1 = 右
  *           steps : 有符号，正 = 原始相位递增方向（不经过 DIR_SIGN）
  */
static void handle_stepper_test(const link_frame_t *f)
{
    uint8_t id;
    int32_t steps;
    int     ret;

    if (f->len < 5u)
    {
        LinkSvc_SendNak(f->seq, ST_UNKNOWN_OP, f->op);
        return;
    }

    id = f->payload[0];
    steps = (int32_t)((uint32_t)f->payload[1]
                    | ((uint32_t)f->payload[2] << 8)
                    | ((uint32_t)f->payload[3] << 16)
                    | ((uint32_t)f->payload[4] << 24));

    if ((id > 1u) || (steps == 0))
    {
        LinkSvc_SendNak(f->seq, ST_UNKNOWN_OP, f->op);
        return;
    }

    /* 用 200 Hz（档位 1）：慢一点，方便肉眼判断轮子往哪走 */
    ret = Stepper_Move((stepper_id_t)id, steps,
                       Stepper_SpeedToHz(STEPPER_SPEED_1));
    if (ret != STEPPER_OK)
    {
        uint8_t st;
        switch (ret)
        {
        case STEPPER_ERR_BUSY:    st = ST_BUSY;           break;
        case STEPPER_ERR_THERMAL: st = ST_FAULT_LATCHED;  break;
        default:                  st = ST_UNKNOWN_OP;     break;
        }
        LinkSvc_SendNak(f->seq, st, f->op);
        return;
    }

    LinkSvc_SendAck(f->seq, ST_OK);
}

/**
  * @brief  独立停止命令
  * @note   为什么单独做一个操作码，而不是复用 MOVE(dir=4)：
  *           停止是安全动作，不能依赖"上位机把 dir 算对"；
  *           而且它必须无条件生效 —— 不论当前是否在运动、是否故障。
  *
  *         payload（可省略）：
  *           [0] bit0 = 1 → 同时清除热保护冷却（台架调试用）
  */
static void handle_motor_stop(const link_frame_t *f)
{
    Stepper_StopAll();

    if ((f->len >= 1u) && ((f->payload[0] & 0x01u) != 0u))
    {
        Stepper_ClearCooldown();
    }

    LinkSvc_SendAck(f->seq, ST_OK);
}

static void handle_set_config(const link_frame_t *f)
{
    uint32_t cfg;
    int ret;

    if (f->len != 4u)
    {
        LinkSvc_SendNak(f->seq, ST_UNKNOWN_OP, f->op);
        return;
    }

    cfg = (uint32_t)f->payload[0]
        | ((uint32_t)f->payload[1] << 8)
        | ((uint32_t)f->payload[2] << 16)
        | ((uint32_t)f->payload[3] << 24);

    ret = Config_Set(cfg);
    if (ret != CONFIG_OK)
    {
        LinkSvc_SendNak(f->seq, ST_FLASH_FAIL, 0u);
        return;
    }

    LinkSvc_SendAck(f->seq, ST_OK);

    /* 上报配置变更，便于 A7Z 侧看到下位机形态变了 */
    {
        uint8_t p[4];
        uint32_t now = Config_Get();
        p[0] = (uint8_t)(now & 0xFFu);
        p[1] = (uint8_t)((now >> 8) & 0xFFu);
        p[2] = (uint8_t)((now >> 16) & 0xFFu);
        p[3] = (uint8_t)((now >> 24) & 0xFFu);
        send_frame(LINK_TYPE_EVT, 0u, OP_EVT_CONFIG_CHANGED, p, 4u);
    }

    /* 若底部三颗 TOF 全关 -> 盲走模式，报故障 */
    if (Config_IsBottomTofAllDisabled())
    {
        s_fault = FAULT_NO_EDGE_PROTECT;
        s_fault_reported = 0u;
    }
    else if (s_fault == FAULT_NO_EDGE_PROTECT)
    {
        s_fault = FAULT_NONE;
    }

    /* 配置变更后立刻应用到执行器（说明书 §5.6.4）：
     * 舵机开关切换必须在运行期生效，不能等重启。 */
    if (Config_IsEnabled(CFG_SERVO_EN))
    {
        if (!Servo_IsEnabled())
        {
            (void)Servo_Init();
#if APP_DEBUG_TEXT
            Serial_Printf("[cfg] servo enabled at runtime\r\n");
#endif
        }
    }
    else
    {
        if (Servo_IsEnabled())
        {
            Servo_Disable();
#if APP_DEBUG_TEXT
            Serial_Printf("[cfg] servo disabled at runtime\r\n");
#endif
        }
    }
}

static void handle_get_config(const link_frame_t *f)
{
    uint8_t p[4];
    uint32_t cfg = Config_Get();

    p[0] = (uint8_t)(cfg & 0xFFu);
    p[1] = (uint8_t)((cfg >> 8) & 0xFFu);
    p[2] = (uint8_t)((cfg >> 16) & 0xFFu);
    p[3] = (uint8_t)((cfg >> 24) & 0xFFu);

    send_frame(LINK_TYPE_ACK, f->seq, OP_CONFIG, p, 4u);
}

static void handle_clear_fault(const link_frame_t *f)
{
    uint8_t flag;
    uint8_t clear_ok = 1u;

    if (f->len < 1u)
    {
        LinkSvc_SendNak(f->seq, ST_UNKNOWN_OP, f->op);
        return;
    }

    flag = f->payload[0];

    /* 盲走模式：必须附 blind_ack = 0x5A 才解除 */
    if (s_fault == FAULT_NO_EDGE_PROTECT)
    {
        if (f->len < 2u || f->payload[1] != FAULT_BLIND_ACK)
        {
            clear_ok = 0u;
        }
    }

    if (!clear_ok)
    {
        LinkSvc_SendNak(f->seq, ST_FAULT_LATCHED, FAULT_NO_EDGE_PROTECT);
        return;
    }

    /* bit0 = 清边缘，bit1 = 清障碍（本阶段实现为整体清除）*/
    (void)flag;
    s_fault = FAULT_NONE;
    s_fault_reported = 0u;

    LinkSvc_SendAck(f->seq, ST_OK);
}

/**
  * @brief  命令分发
  * @note   M0-M2.5 只实现 PING / SET_CONFIG / GET_CONFIG / CLEAR_FAULT，
  *         运动与音频相关操作码明确返回"未实现"，便于上位机识别当前阶段。
  */
static void LinkSvc_Dispatch(const link_frame_t *f)
{
    /* 任何合法帧都算"通信正常" */
    s_last_frame_ms = BSP_GetTickMs();
    s_comm_lost = 0u;

    if (!s_ever_comm)
    {
        s_ever_comm = 1u;
#if APP_DEBUG_TEXT
        Serial_Printf("[link] first frame OK (op=0x%02X seq=%u)\r\n",
                      f->op, (unsigned)f->seq);
#endif
    }

    /* 只要收到帧，就认为心跳恢复 */
    if (s_fault == FAULT_HEARTBEAT_LOST)
    {
        s_fault = FAULT_NONE;
        s_fault_reported = 0u;
    }

    switch (f->op)
    {
    case OP_PING:
        handle_ping(f);
        break;

    case OP_SET_CONFIG:
        handle_set_config(f);
        break;

    case OP_GET_CONFIG:
        handle_get_config(f);
        break;

    case OP_STEPPER_STAT:      /* 诊断：电机实时状态 */
        handle_stepper_stat(f);
        break;

    case OP_LINK_STAT:         /* 诊断：链路健康计数器 */
        handle_link_stat(f);
        break;

    case OP_STEPPER_TEST:      /* 标定：单侧电机原始方向 */
        handle_stepper_test(f);
        break;

    case OP_MOTOR_STOP:        /* 独立停止：无条件生效 */
        handle_motor_stop(f);
        break;

    case OP_CLEAR_FAULT:
        handle_clear_fault(f);
        break;

    case OP_SERVO:         /* M3：已实现 */
        handle_servo(f);
        break;

    case OP_MOVE:          /* M4/M5：已实现 */
        handle_move(f);
        break;

    case OP_TTS:           /* M9 实现 */
    case OP_TOF_CFG:       /* M7 实现 */
        LinkSvc_SendNak(f->seq, ST_NOT_IMPLEMENTED, f->op);
        break;

    default:
        LinkSvc_SendNak(f->seq, ST_UNKNOWN_OP, f->op);
        break;
    }
}

/* ============================ 主循环任务 ============================ */

void LinkSvc_Poll(uint32_t now_ms)
{
    uint8_t        b;
    link_frame_t   frame;
    int            r;

    if (!s_boot_captured)
    {
        s_boot_ms = BSP_GetTickMs();
        s_boot_captured = 1u;
    }

    /* 一次最多处理 32 个字节，避免长时间占住主循环 */
    {
        uint8_t guard = 0u;
        while (guard++ < 32u && Serial_ReadByte(&b))
        {
            r = Link_FeedByte(b, now_ms, &frame);
            if (r == 1)
            {
                LinkSvc_Dispatch(&frame);
            }
            /* r == -1 是坏帧，已计入统计，继续解析后续字节 */
        }
    }
}

void LinkSvc_HeartbeatTask(uint32_t now_ms)
{
    uint32_t elapsed;
    uint32_t now;

    if (!s_boot_captured)
    {
        return;
    }

    /* ★ 必须重新取一次 tick，不能用调用者传进来的 now_ms。
     *
     * 原因（这是一个真实踩到的 uint32 下溢 bug）：
     *   主循环的 now 是在循环开头取的；
     *   而 LinkSvc_Poll -> LinkSvc_Dispatch 里又调了一次 BSP_GetTickMs()
     *   来更新 s_last_frame_ms。如果这中间跨过了 1 ms 边界：
     *
     *       now_ms = T,  s_last_frame_ms = T+1
     *       (uint32_t)(T - (T+1)) = 4294967295
     *       4294967295 >= 1000  ->  每收到一帧就误报一次心跳丢失！
     *
     *   实测症状：日志里心跳丢失刷屏；每帧约 1~5% 概率误判；
     *   安全联锁因此把正在跑的 MOVE 反复停掉（trips=0、左右不分叉，
     *   但动作被截断 8 次）。
     */
    now = BSP_GetTickMs();

    /* 上电宽限期：未接 A7Z 也不应立刻报故障（同样防下溢） */
    {
        uint32_t boot_elapsed = (now >= s_boot_ms) ? (now - s_boot_ms) : 0u;
        if (boot_elapsed < APP_BOOT_BANNER_MS)
        {
            return;
        }
    }

    /* now_ms 参数保留只为兼容调用方，内部一律用新取的 now */
    (void)now_ms;

    if (s_comm_lost)
    {
        elapsed = 0xFFFFFFFFu;      /* 从未收到过帧 */
    }
    else
    {
        elapsed = (now >= s_last_frame_ms) ? (now - s_last_frame_ms) : 0u;
    }

    if (elapsed >= HB_TIMEOUT_STOP_MS && s_fault != FAULT_HEARTBEAT_LOST)
    {
        s_fault = FAULT_HEARTBEAT_LOST;
        s_fault_reported = 0u;
        s_comm_lost = 1u;
        /* M3+ 在此处停止步进；本阶段无执行器，仅置故障 */
#if APP_DEBUG_TEXT
        Serial_Printf("[link] heartbeat lost (%s)\r\n",
                      s_ever_comm ? "was connected" : "never connected");
#endif
    }

    /* 故障事件只主动上报一次，避免刷屏 */
    if (s_fault != FAULT_NONE && !s_fault_reported)
    {
        LinkSvc_SendFaultEvent(s_fault);
        s_fault_reported = 1u;
#if APP_DEBUG_TEXT
        Serial_Printf("[link] -> sent EVT_FAULT code=0x%02X\r\n", s_fault);
#endif
    }
}

/* ============================ 查询 ============================ */

uint8_t LinkSvc_GetFault(void)
{
    return s_fault;
}

uint8_t LinkSvc_IsCommLost(void)
{
    return s_comm_lost;
}

uint8_t LinkSvc_HasEverCommed(void)
{
    return s_ever_comm;
}

uint32_t LinkSvc_GetLastFrameMs(void)
{
    return s_last_frame_ms;
}

uint32_t LinkSvc_GetGoodFrameCount(void)
{
    return Link_GetGoodFrames();
}

uint32_t LinkSvc_GetBadFrameCount(void)
{
    return Link_GetBadFrames();
}

void LinkSvc_ClearFault(void)
{
    s_fault = FAULT_NONE;
    s_fault_reported = 0u;
}
