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
#include "tof.h"
#include "tof_service.h"
#include "tts.h"
#include "tts_math.h"
#include "cmd_router.h"
#include "link_service.h"

/* 0x08 当前没有命令会返回它（M0-M9 的操作码已全部实现）。
 * 保留定义是因为上位机 pc_link_test.py 的 STATUS 表里有这个码，
 * 删掉会让那个表悄悄对不上号。 */
#define ST_NOT_IMPLEMENTED   0x08u

/* 运行状态 */
static uint8_t  s_comm_lost = 1u;      /* 上电默认认为未建立通信 */
static uint8_t  s_ever_comm = 0u;      /* 上电后是否收到过至少一帧 */
static uint8_t  s_fault = FAULT_NONE;
static uint8_t  s_fault_reported = 0u;
static uint32_t s_boot_ms = 0u;
static uint8_t  s_boot_captured = 0u;
static uint8_t  s_last_dir = 0xFFu;    /* 最近一次成功的 MOVE 方向 */

/* 盲走模式确认（每次冷启动都要重新确认，不持久化）*/
static uint8_t  s_blind_ack = 0u;

/* ============================ 双端口链路状态 ============================ */
/**
  * @brief  一条链路的运行状态
  * @note   每条链路有【自己的】解析器、时间戳与统计。
  *         共用任意一项都会导致两条链路互相破坏。
  */
typedef struct
{
    link_parser_t parser;
    uint32_t      last_frame_ms;
    uint8_t       ever_comm;
    uint8_t       alive;
} link_port_t;

static link_port_t s_ports[CMD_SRC_COUNT];

/* 当前正在处理哪条链路的帧 —— 应答要发回这条链路。
 * dispatch 是同步的（没有重入），所以用一个模块变量足够，
 * 这样所有 handler 里的 LinkSvc_SendAck/Nak 都不用改签名。 */
static uint8_t s_reply_src = CMD_SRC_A7Z;

/* 当前控制权拥有者；NONE 表示两路都不在线 */
static uint8_t s_owner      = CMD_SRC_NONE;
static uint8_t s_owner_prev = CMD_SRC_NONE;

/* ============================ 内部：故障优先级 ============================ */

/**
  * @brief  故障优先级：数值越大越"紧急"，用来决定同时出现时上报哪一个
  * @note   桌沿（会掉下去）> 障碍 > 心跳丢失 > 过热 > TOF 初始化失败
  */
static uint8_t fault_priority(uint8_t code)
{
    switch (code)
    {
    case FAULT_NO_EDGE_PROTECT: return 6u;   /* 盲走：连保护都没有 */
    case FAULT_EDGE:            return 5u;
    case FAULT_OBSTACLE:        return 4u;
    case FAULT_HEARTBEAT_LOST:  return 3u;
    case FAULT_THERMAL:         return 2u;
    case FAULT_TOF_INIT:        return 1u;
    default:                    return 0u;
    }
}

void LinkSvc_SetFault(uint8_t code)
{
    if (code == FAULT_NONE)
    {
        return;
    }

    if ((s_fault == FAULT_NONE) || (fault_priority(code) > fault_priority(s_fault)))
    {
        s_fault = code;
        s_fault_reported = 0u;
    }
}

void LinkSvc_ClearFaultCode(uint8_t code)
{
    if (s_fault == code)
    {
        s_fault = FAULT_NONE;
        s_fault_reported = 0u;
    }
}

/* ============================ 内部：发送辅助 ============================ */

/**
  * @brief  往指定链路发一帧
  * @note   蓝牙未初始化时静默丢弃：没插模块不应导致异常。
  */
static void send_to_src(uint8_t src, uint8_t type, uint16_t seq, uint8_t op,
                        const uint8_t *payload, uint8_t len)
{
    uint8_t  buf[LINK_MAX_FRAME];
    int      n;

    n = Link_BuildFrame(buf, (uint16_t)sizeof(buf), type, seq, op, payload, len);
    if (n <= 0)
    {
        return;
    }

    if (src == CMD_SRC_BT)
    {
        if (SerialBt_IsReady() != 0u)
        {
            SerialBt_SendBuffer(buf, (uint16_t)n);
        }
        return;
    }

    Serial_SendBuffer(buf, (uint16_t)n);
}

/**
  * @brief  应答：发回"发起本次请求的那条链路"
  * @note   s_reply_src 在 poll_port 里设置。dispatch 是同步调用，
  *         所以这里不需要传参，所有 handler 保持原样。
  */
static void send_frame(uint8_t type, uint16_t seq, uint8_t op,
                       const uint8_t *payload, uint8_t len)
{
    send_to_src(s_reply_src, type, seq, op, payload, len);
}

/**
  * @brief  主动事件：广播给所有在线链路
  * @note   一路都不在线时也往 A7Z 发一次，便于串口抓到现场。
  */
static void send_broadcast(uint8_t op, const uint8_t *payload, uint8_t len)
{
    uint8_t i;
    uint8_t any = 0u;

    for (i = 0u; i < CMD_SRC_COUNT; i++)
    {
        if (s_ports[i].alive == 0u)
        {
            continue;
        }
        /* 高频遥测默认不给蓝牙：蓝牙只是遥控通道，且低波特率下会把发送端堵死 */
        if ((op == OP_EVT_TOF_DATA) && (i == CMD_SRC_BT) &&
            (BT_VERBOSE_EVENTS == 0))
        {
            continue;
        }
        send_to_src(i, LINK_TYPE_EVT, 0u, op, payload, len);
        any = 1u;
    }

    if (any == 0u)
    {
        send_to_src(CMD_SRC_A7Z, LINK_TYPE_EVT, 0u, op, payload, len);
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
    send_broadcast(OP_EVT_FAULT, &code, 1u);
}

/* ============================ 初始化 ============================ */

void LinkSvc_Init(void)
{
    uint8_t i;

    /* 每条链路一个独立解析器 */
    for (i = 0u; i < CMD_SRC_COUNT; i++)
    {
        Link_ParserInit(&s_ports[i].parser);
        s_ports[i].last_frame_ms = 0u;
        s_ports[i].ever_comm     = 0u;
        s_ports[i].alive         = 0u;
    }

    s_reply_src     = CMD_SRC_A7Z;
    s_owner         = CMD_SRC_NONE;
    s_owner_prev    = CMD_SRC_NONE;
    s_comm_lost     = 1u;
    s_ever_comm     = 0u;
    s_fault         = FAULT_NONE;
    s_fault_reported = 0u;
    s_boot_captured = 0u;
    s_boot_ms       = 0u;

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
  * @brief  统一的"请求一次运动"入口
  * @param  dir   MOVE_DIR_xxx
  * @param  hz    步进频率
  * @param  steps 步数（正数；方向由 dir 决定）
  * @return STEPPER_OK / STEPPER_ERR_xxx
  * @note   内部与 TOF 的自动回退共用，保证方向换算只有一处。
  */
int LinkSvc_RequestMove(uint8_t dir, uint16_t hz, int32_t steps)
{
    int32_t left  = 0;
    int32_t right = 0;
    int     ret;

    if ((dir > MOVE_DIR_TURN_RIGHT) || (steps <= 0))
    {
        return STEPPER_ERR_PARAM;
    }

    move_dir_to_steps(dir, steps, &left, &right);

    ret = Stepper_MoveBoth(left, right, hz);
    if (ret == STEPPER_OK)
    {
        s_last_dir = dir;
    }
    return ret;
}

uint8_t LinkSvc_GetLastDir(void)
{
    return s_last_dir;
}

/**
  * @brief  统一的主动事件上报入口
  * @note   MOTION_DONE / EDGE_DETECTED / OBSTACLE_DETECTED / TOF_DATA 都走这里。
  *         事件是广播给所有在线链路的（上位机和遥控端都应看到状态），
  *         但 TOF_DATA 这类高频遥测默认只发 A7Z。
  */
void LinkSvc_SendEvent(uint8_t op, const uint8_t *payload, uint8_t len)
{
    send_broadcast(op, payload, len);
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

    /* ---------- 安全闸门 ----------
     * 1) 盲走模式：底部三颗 TOF 全关时必须先显式确认（不持久化）
     * 2) 桌沿 / 障碍：按方向分别放行
     */
    if ((Config_IsBottomTofAllDisabled() != 0u) && (s_blind_ack == 0u))
    {
        LinkSvc_SendNak(f->seq, ST_FAULT_LATCHED, FAULT_NO_EDGE_PROTECT);
        return;
    }

    if (TofSvc_MotionAllowed(dir) == 0u)
    {
        LinkSvc_SendNak(f->seq, ST_FAULT_LATCHED,
                        (TofSvc_IsObstacleActive() != 0u) ? FAULT_OBSTACLE
                                                          : FAULT_EDGE);
        return;
    }

    ret = LinkSvc_RequestMove(dir, Stepper_SpeedToHz(speed), steps);
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
    Serial_Printf("[move] dir=%u speed=%u(%uHz) steps=%ld\r\n",
                  (unsigned)dir, (unsigned)speed,
                  (unsigned)Stepper_SpeedToHz(speed),
                  (long)steps);
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
  * ★ [0..25] 是【发起本次查询的那条链路】的统计：
  *     A7Z 来问就是 A7Z 的数据，蓝牙来问就是蓝牙的数据。
  *     [26..29] 才是全局的双链路状态。
  *
  * payload（32 字节）：
  *   [0..3]   uptime ms
  *   [4..7]   好帧数（本链路）
  *   [8..11]  坏帧数（本链路）
  *   [12..15] 收到的字节总数（本链路）
  *   [16..19] 环形缓冲溢出次数（本链路）
  *   [20..23] 距上一帧的毫秒数（从未收到过则为 0xFFFFFFFF）
  *   [24]     当前 fault
  *   [25]     comm_lost 标志（两路都失联才为 1）
  *   [26]     当前控制权：0=A7Z 1=BT 0xFF=无人
  *   [27]     A7Z alive
  *   [28]     BT alive
  *   [29]     BT 端口是否已初始化
  *   [30..31] 保留（0）
  */
static void handle_link_stat(const link_frame_t *f)
{
    uint8_t        p[32];
    uint32_t       v;
    uint32_t       age;
    uint8_t        src = s_reply_src;
    link_port_t   *port = &s_ports[src];

    if (port->ever_comm == 0u)
    {
        age = 0xFFFFFFFFu;
    }
    else
    {
        uint32_t now = BSP_GetTickMs();
        age = (now >= port->last_frame_ms) ? (now - port->last_frame_ms) : 0u;
    }

    v = BSP_GetTickMs();
    p[0]  = (uint8_t)(v & 0xFF);          p[1]  = (uint8_t)((v >> 8) & 0xFF);
    p[2]  = (uint8_t)((v >> 16) & 0xFF);  p[3]  = (uint8_t)((v >> 24) & 0xFF);

    v = port->parser.good;
    p[4]  = (uint8_t)(v & 0xFF);          p[5]  = (uint8_t)((v >> 8) & 0xFF);
    p[6]  = (uint8_t)((v >> 16) & 0xFF);  p[7]  = (uint8_t)((v >> 24) & 0xFF);

    v = port->parser.bad;
    p[8]  = (uint8_t)(v & 0xFF);          p[9]  = (uint8_t)((v >> 8) & 0xFF);
    p[10] = (uint8_t)((v >> 16) & 0xFF);  p[11] = (uint8_t)((v >> 24) & 0xFF);

    v = (src == CMD_SRC_BT) ? SerialBt_GetRxTotalCount()
                            : Serial_GetRxTotalCount();
    p[12] = (uint8_t)(v & 0xFF);          p[13] = (uint8_t)((v >> 8) & 0xFF);
    p[14] = (uint8_t)((v >> 16) & 0xFF);  p[15] = (uint8_t)((v >> 24) & 0xFF);

    v = (src == CMD_SRC_BT) ? SerialBt_GetRxOverflowCount()
                            : Serial_GetRxOverflowCount();
    p[16] = (uint8_t)(v & 0xFF);          p[17] = (uint8_t)((v >> 8) & 0xFF);
    p[18] = (uint8_t)((v >> 16) & 0xFF);  p[19] = (uint8_t)((v >> 24) & 0xFF);

    p[20] = (uint8_t)(age & 0xFF);        p[21] = (uint8_t)((age >> 8) & 0xFF);
    p[22] = (uint8_t)((age >> 16) & 0xFF); p[23] = (uint8_t)((age >> 24) & 0xFF);

    p[24] = s_fault;
    p[25] = s_comm_lost;

    p[26] = s_owner;
    p[27] = s_ports[CMD_SRC_A7Z].alive;
    p[28] = s_ports[CMD_SRC_BT].alive;
    p[29] = SerialBt_IsReady();
    p[30] = 0u;
    p[31] = 0u;

    send_frame(LINK_TYPE_ACK, f->seq, OP_LINK_STAT, p, 32u);
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

    if (f->len < 1u)
    {
        LinkSvc_SendNak(f->seq, ST_UNKNOWN_OP, f->op);
        return;
    }

    flag = f->payload[0];

    /* ---------- 盲走模式确认 ----------
     * 底部三颗 TOF 全关时，必须显式带 blind_ack = 0x5A。
     * 这份许可是【内存态】，每次冷启动都要重新确认 ——
     * 目的是让"关掉安全传感器"永远是一个刻意动作。 */
    if (s_fault == FAULT_NO_EDGE_PROTECT)
    {
        if ((f->len < 2u) || (f->payload[1] != FAULT_BLIND_ACK))
        {
            LinkSvc_SendNak(f->seq, ST_FAULT_LATCHED, FAULT_NO_EDGE_PROTECT);
            return;
        }

        s_blind_ack = 1u;
        s_fault = FAULT_NONE;
        s_fault_reported = 0u;
        LinkSvc_SendAck(f->seq, ST_OK);
        return;
    }

    /* ---------- 按位清除 ----------
     * bit0 = 清桌沿，bit1 = 清障碍。
     * 只有对应的故障真的存在时才清除，避免误清别的故障。 */
    if ((flag & 0x01u) != 0u)
    {
        TofSvc_ClearEdge();
    }
    if ((flag & 0x02u) != 0u)
    {
        TofSvc_ClearObstacle();
    }

    /* flag == 0 视为"清全部可清故障"（兼容旧上位机）*/
    if (flag == 0u)
    {
        TofSvc_ClearEdge();
        TofSvc_ClearObstacle();
    }

    /* 过热故障由冷却状态自动恢复，这里只把上报标志复位一次 */
    if (s_fault == FAULT_THERMAL)
    {
        s_fault = FAULT_NONE;
        s_fault_reported = 0u;
    }

    LinkSvc_SendAck(f->seq, ST_OK);
}

/**
  * @brief  M9：TTS —— 把语音数据转发到 USART2 上的 TW-TTS 模块
  *
  * payload 有两种形态：
  *   ① 以 0xFD 开头 -> 视为 TW-TTS 原始帧，原样转发（说明书 §6.3 的定义）
  *   ② 其他        -> 视为 UTF-8 正文，由 MCU 组 "FD len 01 04 text" 再发
  *
  * ② 是为了台架自测：上位机不用自己算帧头，直接发一句中文即可。
  * A7Z 正式接入时按 ① 走，协议与它已验证过的 uart_tts 完全一致。
  *
  * ⚠️ 只入队不等发完。9600 波特下一条长句要 200 ms，
  *    同步发送会把 1 ms 步进节拍和心跳全卡住。
  */
static void handle_tts(const link_frame_t *f)
{
    int r;

    if ((Config_IsEnabled(CFG_TTS_EN) == 0u) || (Tts_IsReady() == 0u))
    {
        LinkSvc_SendNak(f->seq, ST_FEATURE_DISABLED, f->op);
        return;
    }

    if (f->len == 0u)
    {
        LinkSvc_SendNak(f->seq, ST_UNKNOWN_OP, f->op);
        return;
    }

    if (tts_is_raw_frame(f->payload[0]) != 0)
    {
        r = Tts_Relay(f->payload, f->len);
    }
    else
    {
        r = Tts_SpeakUtf8(f->payload, f->len);
    }

    if (r == TTS_OK)
    {
        /* 环里还有积压就顺带告诉上位机，方便它自己限速 */
        LinkSvc_SendAck(f->seq, (Tts_GetPendingBytes() > 0u) ? ST_BUSY : ST_OK);
    }
    else if (r == TTS_ERR_BUSY)
    {
        /* 环满：让上位机稍后重发，不要半截入队把数据流截断 */
        LinkSvc_SendNak(f->seq, ST_BUSY, f->op);
    }
    else
    {
        /* 超长或未初始化 */
        LinkSvc_SendNak(f->seq, ST_CLAMPED, f->op);
    }
}

/**
  * @brief  M7：TOF_CFG —— 运行时调整轮询周期
  * @note   payload: poll_ms(1)，10..255。不改启用集（那要走 SET_CONFIG）。
  */
static void handle_tof_cfg(const link_frame_t *f)
{
    uint8_t ms;

    if (f->len < 1u)
    {
        LinkSvc_SendNak(f->seq, ST_UNKNOWN_OP, f->op);
        return;
    }

    if (Tof_GetOnlineMask() == 0u)
    {
        /* 一颗都不在线：没得调 */
        LinkSvc_SendNak(f->seq, ST_FEATURE_DISABLED, f->op);
        return;
    }

    ms = Tof_SetPollMs(f->payload[0]);

    /* 若超出范围已被钳位，按说明书用 0x01 告知"参数越界已钳位" */
    {
        uint8_t st = (ms == f->payload[0]) ? ST_OK : 0x01u;
        LinkSvc_SendAck(f->seq, st);
    }
}

/**
  * @brief  命令分发
  * @note   已实现：PING / SET_CONFIG / GET_CONFIG / CLEAR_FAULT /
  *         MOVE / SERVO / TOF_CFG / TTS / MOTOR_STOP 及三个诊断命令。
  *         未识别的操作码返回 NAK(0x05)。
  */
static void LinkSvc_Dispatch(const link_frame_t *f)
{
    uint8_t cls;

    /* 任何合法帧都算"本链路通信正常"。
     * 时间戳已由 poll_port 记在各自端口里（用的是主循环开头的 now_ms），
     * 这里不再取 BSP_GetTickMs() —— 从根上避免"时间戳比心跳的 now 还新"的下溢。 */

    if (!s_ever_comm)
    {
        s_ever_comm = 1u;
#if APP_DEBUG_TEXT
        Serial_Printf("[link] first frame OK (src=%u op=0x%02X seq=%u)\r\n",
                      (unsigned)s_reply_src, f->op, (unsigned)f->seq);
#endif
    }

    /* 只要收到帧，就认为心跳恢复 */
    if (s_fault == FAULT_HEARTBEAT_LOST)
    {
        s_fault = FAULT_NONE;
        s_fault_reported = 0u;
    }

    /* ---------- 双命令源仲裁 ----------
     * SAFETY / QUERY 一律放行；CONTROL 只有控制权拥有者才能执行。
     * 这一层放在 switch 之前，任何 handler 都不会被越权调用。
     */
    cls = cmd_class_of_frame(f->op, f->payload, f->len);
    if (cmd_allowed(s_reply_src, cls, s_owner) == 0u)
    {
        LinkSvc_SendNak(f->seq, ST_ARBITRATION, f->op);
        return;
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

    case OP_TTS:           /* M9：TW-TTS 转发（★ 不可与 TOF_CFG 共用分支）*/
        handle_tts(f);
        break;

    case OP_TOF_CFG:       /* M7：运行时调整轮询周期 */
        handle_tof_cfg(f);
        break;

    default:
        LinkSvc_SendNak(f->seq, ST_UNKNOWN_OP, f->op);
        break;
    }
}

/* ============================ 主循环任务 ============================ */

/**
  * @brief  该链路的存活超时窗口
  * @note   蓝牙比 A7Z 宽松：人工遥控的节奏比机器慢，
  *         用 1000 ms 会让用户稍微停顿就被判失联。
  */
static uint32_t port_alive_window(uint8_t src)
{
    return (src == CMD_SRC_BT) ? BT_TIMEOUT_MS : HB_TIMEOUT_STOP_MS;
}

/**
  * @brief  更新两条链路的 alive 标志与控制权
  * @note   ★ 控制权发生变化时立刻停车。
  *
  *         不这么做会出现这种情况：A7Z 正在让它前进，A7Z 掉线，
  *         蓝牙接管 —— 但蓝牙还没发任何命令，机器人会"接着 A7Z 的动作继续跑"。
  *         这在没有桌沿保护的情况下非常危险。
  */
static void link_update_alive(uint32_t now)
{
    uint8_t i;
    uint8_t owner;

    for (i = 0u; i < CMD_SRC_COUNT; i++)
    {
        uint32_t el;

        if (s_ports[i].ever_comm == 0u)
        {
            s_ports[i].alive = 0u;
            continue;
        }

        el = (now >= s_ports[i].last_frame_ms)
             ? (now - s_ports[i].last_frame_ms) : 0u;

        s_ports[i].alive = (uint8_t)((el < port_alive_window(i)) ? 1u : 0u);
    }

    owner = cmd_owner(s_ports[CMD_SRC_A7Z].alive, s_ports[CMD_SRC_BT].alive);
    s_owner = owner;

    if (owner != s_owner_prev)
    {
        s_owner_prev = owner;

        /* 控制权切换（包括"从有到无"）：停车，等新拥有者重新下令 */
        Stepper_StopAll();
        TofSvc_OnUserStop();

#if APP_DEBUG_TEXT
        Serial_Printf("[link] owner -> %s\r\n",
                      (owner == CMD_SRC_A7Z) ? "A7Z" :
                      (owner == CMD_SRC_BT)  ? "BT"  : "NONE");
#endif
        (void)owner;
    }
}

/**
  * @brief  把一条链路的字节喂进它自己的解析器
  * @note   ★ 每条链路每次调用最多处理 LINK_PORT_BYTE_BUDGET 个字节。
  *         没有这个配额时，一路被噪声刷屏会把主循环彻底占死
  *         （A7Z 侧和步进任务全部饿死），这就是"程序卡死"的主要来源。
  */
static void poll_port(uint8_t src, uint32_t now_ms)
{
    uint8_t      b;
    link_frame_t frame;
    int          r;
    uint8_t      guard = 0u;

    for (;;)
    {
        if (guard >= LINK_PORT_BYTE_BUDGET)
        {
            break;              /* 配额用完，剩下的留到下一轮 */
        }

        if (src == CMD_SRC_BT)
        {
            if (SerialBt_IsReady() == 0u)
            {
                break;          /* 蓝牙未初始化：这一路整体跳过 */
            }
            if (SerialBt_ReadByte(&b) == 0u)
            {
                break;
            }
        }
        else
        {
            if (Serial_ReadByte(&b) == 0u)
            {
                break;
            }
        }

        guard++;

        r = Link_FeedByteTo(&s_ports[src].parser, b, now_ms, &frame);
        if (r == 1)
        {
            s_ports[src].last_frame_ms = now_ms;   /* 用循环开头的 now，防下溢 */
            s_ports[src].ever_comm     = 1u;

            /* ★ 必须在分发前刷新存活与控制权。
             *   否则刚上电的第一次控制命令会因为 owner 还是 NONE
             *   被仲裁拒掉（NAK 0x09），表现为"上电第一条命令不生效"。 */
            link_update_alive(now_ms);

            s_reply_src = src;
            LinkSvc_Dispatch(&frame);
        }
        /* r == -1 是坏帧，已计入统计，继续解析后续字节 */
    }
}

void LinkSvc_Poll(uint32_t now_ms)
{
    if (!s_boot_captured)
    {
        s_boot_ms = BSP_GetTickMs();
        s_boot_captured = 1u;
    }

    /* 先刷新存活与控制权：新连上的链路在这一轮就能拿到控制权 */
    link_update_alive(BSP_GetTickMs());

    /* 两路都处理，各自有配额；顺序固定（A7Z 优先）保证行为可预期 */
    poll_port(CMD_SRC_A7Z, now_ms);
    poll_port(CMD_SRC_BT,  now_ms);
}

void LinkSvc_HeartbeatTask(uint32_t now_ms)
{
    uint32_t now;
    uint8_t  a7z;
    uint8_t  bt;

    if (!s_boot_captured)
    {
        return;
    }

    /* ★ 必须重新取一次 tick，不能用调用者传进来的 now_ms。
     *
     * 原因（这是一个真实踩到的 uint32 下溢 bug）：
     *   now 是主循环开头取的，而各处的 last_frame_ms 可能是稍后写入的。
     *   如果这中间跨过了 1 ms 边界：
     *       now = T,  last_frame_ms = T+1
     *       (uint32_t)(T - (T+1)) = 4294967295 >= 1000 -> 误报心跳丢失
     *   实测症状：心跳丢失刷屏，安全联锁把正在跑的 MOVE 反复停掉。
     */
    now = BSP_GetTickMs();

    /* 上电宽限期：未接上位机也不应立刻报故障（同样防下溢） */
    {
        uint32_t boot_elapsed = (now >= s_boot_ms) ? (now - s_boot_ms) : 0u;
        if (boot_elapsed < APP_BOOT_BANNER_MS)
        {
            return;
        }
    }

    (void)now_ms;

    link_update_alive(now);

    a7z = s_ports[CMD_SRC_A7Z].alive;
    bt  = s_ports[CMD_SRC_BT].alive;

    /* ---------- 双链路心跳 ----------
     * ★ 只有【两路都失联】才算心跳丢失。
     *
     *   旧逻辑只认 A7Z：A7Z 一掉线就停车 —— 那"单独用蓝牙遥控"
     *   根本跑不起来（1 秒后就报失联）。现在只要有一路在线就继续工作。
     */
    if ((a7z != 0u) || (bt != 0u))
    {
        s_comm_lost = 0u;

        if (s_fault == FAULT_HEARTBEAT_LOST)
        {
            s_fault = FAULT_NONE;
            s_fault_reported = 0u;
        }
    }
    else
    {
        s_comm_lost = 1u;

        if (s_fault != FAULT_HEARTBEAT_LOST)
        {
            s_fault = FAULT_HEARTBEAT_LOST;
            s_fault_reported = 0u;
#if APP_DEBUG_TEXT
            Serial_Printf("[link] heartbeat lost (a7z=%u bt=%u ever=%u/%u)\r\n",
                          (unsigned)a7z, (unsigned)bt,
                          (unsigned)s_ports[CMD_SRC_A7Z].ever_comm,
                          (unsigned)s_ports[CMD_SRC_BT].ever_comm);
#endif
        }
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
    return s_ports[CMD_SRC_A7Z].last_frame_ms;
}

uint32_t LinkSvc_GetGoodFrameCount(void)
{
    return s_ports[CMD_SRC_A7Z].parser.good;
}

uint32_t LinkSvc_GetBadFrameCount(void)
{
    return s_ports[CMD_SRC_A7Z].parser.bad;
}

void LinkSvc_ClearFault(void)
{
    s_fault = FAULT_NONE;
    s_fault_reported = 0u;
}

/* ============================ 双链路状态查询 ============================ */

uint8_t LinkSvc_GetOwner(void)
{
    return s_owner;
}

uint8_t LinkSvc_IsPortAlive(uint8_t src)
{
    if (src >= CMD_SRC_COUNT)
    {
        return 0u;
    }
    return s_ports[src].alive;
}

uint8_t LinkSvc_IsBtEnabled(void)
{
    return SerialBt_IsReady();
}
