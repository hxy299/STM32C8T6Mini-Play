/**
  ******************************************************************************
  * @file    tools/hosttest/test_cmd_router.c
  * @brief   双命令源仲裁的单元测试（无需硬件）
  *
  * 编译：
  *   gcc -std=c99 -Wall -Wextra -Werror -O2 -I tools/hosttest -I Service \
  *       tools/hosttest/test_cmd_router.c -o t && ./t
  ******************************************************************************
  */
#include <stdio.h>
#include "app_config.h"
#include "cmd_router.h"

static int g_pass = 0;
static int g_fail = 0;

static void chk(int cond, const char *name)
{
    if (cond)
    {
        g_pass++;
        printf("  [PASS] %s\n", name);
    }
    else
    {
        g_fail++;
        printf("  [FAIL] %s\n", name);
    }
}

/* ---------------------------------------------------------------- */

static void test_class(void)
{
    printf("\n[1] 操作码 -> 优先级分类\n");

    chk(cmd_class_of(OP_MOTOR_STOP)   == CMD_CLASS_SAFETY, "MOTOR_STOP = SAFETY");
    chk(cmd_class_of(OP_CLEAR_FAULT)  == CMD_CLASS_SAFETY, "CLEAR_FAULT = SAFETY");

    chk(cmd_class_of(OP_PING)         == CMD_CLASS_QUERY,  "PING = QUERY");
    chk(cmd_class_of(OP_GET_CONFIG)   == CMD_CLASS_QUERY,  "GET_CONFIG = QUERY");
    chk(cmd_class_of(OP_STEPPER_STAT) == CMD_CLASS_QUERY,  "STEPPER_STAT = QUERY");
    chk(cmd_class_of(OP_LINK_STAT)    == CMD_CLASS_QUERY,  "LINK_STAT = QUERY");

    chk(cmd_class_of(OP_MOVE)         == CMD_CLASS_CONTROL, "MOVE = CONTROL");
    chk(cmd_class_of(OP_SERVO)        == CMD_CLASS_CONTROL, "SERVO = CONTROL");
    chk(cmd_class_of(OP_TTS)          == CMD_CLASS_CONTROL, "TTS = CONTROL");
    chk(cmd_class_of(OP_TOF_CFG)      == CMD_CLASS_CONTROL, "TOF_CFG = CONTROL");
    chk(cmd_class_of(OP_SET_CONFIG)   == CMD_CLASS_CONTROL, "SET_CONFIG = CONTROL");
    chk(cmd_class_of(OP_STEPPER_TEST) == CMD_CLASS_CONTROL, "STEPPER_TEST = CONTROL");

    /* 未知操作码归 CONTROL：dispatch 会回 NAK(UNKNOWN_OP)，
     * 但也不该让它绕过控制权检查 */
    chk(cmd_class_of(0xEE)            == CMD_CLASS_CONTROL, "未知码归 CONTROL");
}

static void test_class_of_frame(void)
{
    uint8_t mstop[4] = { MOVE_DIR_STOP, 2u, 0u, 0u };
    uint8_t mfwd[4]  = { MOVE_DIR_FORWARD, 2u, 0xE8u, 0x03u };
    uint8_t mbck[4]  = { MOVE_DIR_BACKWARD, 2u, 0xE8u, 0x03u };
    uint8_t empty[1] = { 0u };

    printf("\n[2] 带载荷分类：MOVE dir=STOP 必须算 SAFETY\n");

    chk(cmd_class_of_frame(OP_MOVE, mstop, 4u) == CMD_CLASS_SAFETY,
        "MOVE dir=STOP -> SAFETY（关键：否则蓝牙停不了车）");
    chk(cmd_class_of_frame(OP_MOVE, mfwd, 4u)  == CMD_CLASS_CONTROL,
        "MOVE dir=FWD -> CONTROL");
    chk(cmd_class_of_frame(OP_MOVE, mbck, 4u)  == CMD_CLASS_CONTROL,
        "MOVE dir=BCK -> CONTROL（后退仍然是控制类，不是安全类）");

    /* 载荷不足时不能误判成 SAFETY */
    chk(cmd_class_of_frame(OP_MOVE, empty, 0u) == CMD_CLASS_CONTROL,
        "MOVE 载荷为空 -> CONTROL（不能误判成停止）");
    chk(cmd_class_of_frame(OP_MOVE, 0, 4u)     == CMD_CLASS_CONTROL,
        "MOVE payload=NULL -> CONTROL（空指针防护）");

    chk(cmd_class_of_frame(OP_MOTOR_STOP, 0, 0u) == CMD_CLASS_SAFETY,
        "MOTOR_STOP 无载荷仍为 SAFETY");
}

static void test_owner(void)
{
    printf("\n[3] 控制权选择\n");

    chk(cmd_owner(1u, 1u) == CMD_SRC_A7Z, "两路都在 -> A7Z 优先");
    chk(cmd_owner(1u, 0u) == CMD_SRC_A7Z, "只有 A7Z -> A7Z");
    chk(cmd_owner(0u, 1u) == CMD_SRC_BT,  "只有蓝牙 -> 蓝牙接管（单独遥控）");
    chk(cmd_owner(0u, 0u) == CMD_SRC_NONE, "两路都无 -> 无人");
}

static void test_allowed(void)
{
    printf("\n[4] 放行规则\n");

    /* CONTROL：只有拥有者放行 */
    chk(cmd_allowed(CMD_SRC_A7Z, CMD_CLASS_CONTROL, CMD_SRC_A7Z) == 1u,
        "A7Z 拥有时，A7Z 的 CONTROL 放行");
    chk(cmd_allowed(CMD_SRC_BT,  CMD_CLASS_CONTROL, CMD_SRC_A7Z) == 0u,
        "A7Z 拥有时，蓝牙的 CONTROL 被拒");
    chk(cmd_allowed(CMD_SRC_BT,  CMD_CLASS_CONTROL, CMD_SRC_BT)  == 1u,
        "蓝牙拥有时，蓝牙的 CONTROL 放行");
    chk(cmd_allowed(CMD_SRC_A7Z, CMD_CLASS_CONTROL, CMD_SRC_BT)  == 0u,
        "蓝牙拥有时，A7Z 的 CONTROL 被拒");
    chk(cmd_allowed(CMD_SRC_BT,  CMD_CLASS_CONTROL, CMD_SRC_NONE) == 0u,
        "无拥有者时 CONTROL 被拒");

    /* SAFETY：任何来源、任何拥有者都放行 */
    chk(cmd_allowed(CMD_SRC_BT,  CMD_CLASS_SAFETY, CMD_SRC_A7Z)  == 1u,
        "A7Z 拥有时，蓝牙的 SAFETY 仍放行（遥控器永远能停车）");
    chk(cmd_allowed(CMD_SRC_A7Z, CMD_CLASS_SAFETY, CMD_SRC_BT)   == 1u,
        "蓝牙拥有时，A7Z 的 SAFETY 仍放行");
    chk(cmd_allowed(CMD_SRC_BT,  CMD_CLASS_SAFETY, CMD_SRC_NONE) == 1u,
        "无拥有者时 SAFETY 仍放行");

    /* QUERY：总是放行 */
    chk(cmd_allowed(CMD_SRC_BT,  CMD_CLASS_QUERY, CMD_SRC_A7Z)   == 1u,
        "A7Z 拥有时，蓝牙的 QUERY 放行（保活）");
    chk(cmd_allowed(CMD_SRC_A7Z, CMD_CLASS_QUERY, CMD_SRC_NONE)  == 1u,
        "无拥有者时 QUERY 放行");
}

static void test_scenario(void)
{
    uint8_t mstop[1] = { MOVE_DIR_STOP };
    uint8_t mfwd[1]  = { MOVE_DIR_FORWARD };
    uint8_t owner;

    printf("\n[5] 场景：A7Z 在线，蓝牙想抢控制\n");

    owner = cmd_owner(1u, 1u);                       /* A7Z 在线 */
    chk(owner == CMD_SRC_A7Z, "拥有者是 A7Z");

    chk(cmd_allowed(CMD_SRC_BT,
                    cmd_class_of_frame(OP_MOVE, mfwd, 1u), owner) == 0u,
        "蓝牙发 MOVE 前进 -> 被拒（NAK 0x09）");

    chk(cmd_allowed(CMD_SRC_BT,
                    cmd_class_of_frame(OP_MOVE, mstop, 1u), owner) == 1u,
        "蓝牙发 MOVE 停止 -> 放行（紧急停车不受仲裁限制）");

    chk(cmd_allowed(CMD_SRC_BT, cmd_class_of(OP_MOTOR_STOP), owner) == 1u,
        "蓝牙发 MOTOR_STOP -> 放行");

    chk(cmd_allowed(CMD_SRC_BT, cmd_class_of(OP_PING), owner) == 1u,
        "蓝牙发 PING -> 放行（要能保活）");

    printf("\n[6] 场景：A7Z 掉线，蓝牙接管\n");

    owner = cmd_owner(0u, 1u);
    chk(owner == CMD_SRC_BT, "拥有者变成蓝牙");
    chk(cmd_allowed(CMD_SRC_BT,
                    cmd_class_of_frame(OP_MOVE, mfwd, 1u), owner) == 1u,
        "蓝牙发 MOVE 前进 -> 现在放行（这就是单独遥控）");
    chk(cmd_allowed(CMD_SRC_A7Z,
                    cmd_class_of_frame(OP_MOVE, mfwd, 1u), owner) == 0u,
        "A7Z 此时发 MOVE -> 被拒（它已经失联，帧不该出现）");
}

static void test_priority_order(void)
{
    printf("\n[7] 优先级数值关系\n");

    /* 数值越小越优先，便于将来加层级 */
    chk(CMD_CLASS_SAFETY  < CMD_CLASS_QUERY,   "SAFETY 优先级高于 QUERY");
    chk(CMD_CLASS_QUERY   < CMD_CLASS_CONTROL, "QUERY 优先级高于 CONTROL");
}

/* ---------------------------------------------------------------- */

int main(void)
{
    printf("==============================================\n");
    printf(" 双命令源仲裁 单元测试\n");
    printf("==============================================\n");

    test_class();
    test_class_of_frame();
    test_owner();
    test_allowed();
    test_scenario();
    test_priority_order();

    printf("\n==============================================\n");
    printf(" 结果: %d PASS / %d FAIL\n", g_pass, g_fail);
    printf("==============================================\n");

    return (g_fail == 0) ? 0 : 1;
}
