/**
  ******************************************************************************
  * @file    tools/hosttest/test_stepper_math.c
  * @brief   步进电机纯算法主机测试（M4/M5 核心逻辑验证）
  *
  * 覆盖：
  *   1. 8 拍相序表是否合法（每次只有 1 或 2 相通电、相邻拍只差 1 位）
  *   2. 相位推进（正/反方向，含 mod 8 回绕）
  *   3. 步频 -> 定时器 ARR
  *   4. 减速所需步数
  *   5. 半步 -> 距离换算
  *   6. 速度档映射
  *   7. 模拟一次完整梯形运动（加速-巡航-减速-停止），验证步数精确
  *
  * 编译运行（WSL）：
  *   gcc -std=c99 -Wall -Wextra -Werror -O2 \
  *       -I tools/hosttest -I Hardware \
  *       tools/hosttest/test_stepper_math.c -o /tmp/t_step && /tmp/t_step
  ******************************************************************************
  */
#include <stdio.h>
#include <stdint.h>
#include "stepper_math.h"

static int g_pass = 0;
static int g_fail = 0;

static void chk(int cond, const char *name)
{
    if (cond) { g_pass++; printf("  [PASS] %s\n", name); }
    else      { g_fail++; printf("  [FAIL] %s\n", name); }
}

/* ---------------------------------------------------------------- */
/* 1. 相序表合法性                                                   */
/* ---------------------------------------------------------------- */
static void test_sequence_table(void)
{
    int i;
    int valid_count = 1;
    int adjacent_ok = 1;

    printf("\n[1] 8 拍半步相序表\n");
    printf("  beat : ");
    for (i = 0; i < 8; i++) { printf("%d ", i); }
    printf("\n  mask : ");
    for (i = 0; i < 8; i++) { printf("%02X ", STEPPER_HALF_SEQ[i]); }
    printf("\n");

    /* 每拍必须只接通 1 或 2 相 */
    for (i = 0; i < 8; i++)
    {
        uint8_t m = STEPPER_HALF_SEQ[i];
        int bits = 0;
        int b;
        for (b = 0; b < 4; b++) { if (m & (1u << b)) { bits++; } }
        if (bits != 1 && bits != 2) { valid_count = 0; }
    }
    chk(valid_count == 1, "每拍只接通 1 或 2 相");

    /* 相邻两拍必须只差 1 个相位（半步的定义） */
    for (i = 0; i < 8; i++)
    {
        uint8_t a = STEPPER_HALF_SEQ[i];
        uint8_t b = STEPPER_HALF_SEQ[(i + 1) & 7];
        uint8_t diff = (uint8_t)(a ^ b);
        int bits = 0;
        int k;
        for (k = 0; k < 4; k++) { if (diff & (1u << k)) { bits++; } }
        if (bits != 1) { adjacent_ok = 0; }
    }
    chk(adjacent_ok == 1, "相邻拍只差 1 个相位");

    /* 四个单相拍必须都出现一次（A/B/C/D 各一次） */
    {
        int singles = 0;
        for (i = 0; i < 8; i++)
        {
            uint8_t m = STEPPER_HALF_SEQ[i];
            if (m == 0x01u || m == 0x02u || m == 0x04u || m == 0x08u)
            {
                singles++;
            }
        }
        printf("  单相拍数量 = %d\n", singles);
        chk(singles == 4, "单相拍恰好 4 个（A/B/C/D 各一次）");
    }

    /* 不能出现 0x00（全断电）或 0x0F（全通电） */
    {
        int bad = 0;
        for (i = 0; i < 8; i++)
        {
            if (STEPPER_HALF_SEQ[i] == 0x00u || STEPPER_HALF_SEQ[i] == 0x0Fu)
            {
                bad = 1;
            }
        }
        chk(bad == 0, "不含 0x00（全断）或 0x0F（全通）");
    }
}

/* ---------------------------------------------------------------- */
/* 2. 相位推进                                                       */
/* ---------------------------------------------------------------- */
static void test_phase_advance(void)
{
    uint8_t p;
    int i;

    printf("\n[2] 相位推进\n");

    /* 正转 8 次应回到原点 */
    p = 0u;
    for (i = 0; i < 8; i++) { p = stepper_math_next_phase(p, +1); }
    chk(p == 0u, "正转 8 拍回到原位");

    /* 反转 8 次应回到原点 */
    p = 0u;
    for (i = 0; i < 8; i++) { p = stepper_math_next_phase(p, -1); }
    chk(p == 0u, "反转 8 拍回到原位");

    /* 从 0 反转应先到 7（而不是 -1） */
    p = stepper_math_next_phase(0u, -1);
    printf("  0 反转 -> %u (期望 7)\n", (unsigned)p);
    chk(p == 7u, "0 反转 -> 7（负数 mod 8 正确）");

    /* 正反转互为逆运算 */
    {
        int ok = 1;
        for (i = 0; i < 8; i++)
        {
            uint8_t a = stepper_math_next_phase((uint8_t)i, +1);
            uint8_t b = stepper_math_next_phase(a, -1);
            if (b != (uint8_t)i) { ok = 0; }
        }
        chk(ok == 1, "正转再反转回到原点");
    }
}

/* ---------------------------------------------------------------- */
/* 3. 步频 -> ARR                                                    */
/* ---------------------------------------------------------------- */
static void test_hz_to_arr(void)
{
    printf("\n[3] 步频 -> 定时器 ARR\n");
    printf("  120 Hz -> ARR=%u (期望 8332 = 1000000/120 - 1)\n",
           (unsigned)stepper_math_hz_to_arr(120));
    printf("  300 Hz -> ARR=%u (期望 3332 = 1000000/300 - 1)\n",
           (unsigned)stepper_math_hz_to_arr(300));
    printf("  600 Hz -> ARR=%u (期望 1665 = 1000000/600 - 1)\n",
           (unsigned)stepper_math_hz_to_arr(600));

    /* 注意：1000000/600 = 1666.67，整除得 1666，减 1 后 ARR = 1665。
     * 实际周期 = (1665+1) tick = 1666 us -> 600.24 Hz，误差 0.04%，可接受。 */
    chk(stepper_math_hz_to_arr(120) == 8332u, "120 Hz -> ARR 8332");
    chk(stepper_math_hz_to_arr(300) == 3332u, "300 Hz -> ARR 3332");
    chk(stepper_math_hz_to_arr(600) == 1665u, "600 Hz -> ARR 1665");

    /* 越界钳位 */
    chk(stepper_math_hz_to_arr(0)   == stepper_math_hz_to_arr(STEPPER_HZ_MIN),
        "0 Hz 被钳到最小频率");
    chk(stepper_math_hz_to_arr(9999)== stepper_math_hz_to_arr(STEPPER_HZ_MAX),
        "9999 Hz 被钳到最大频率");

    /* 频率越高 ARR 越小（单调） */
    {
        uint16_t prev = 0xFFFF;
        int mono = 1;
        int hz;
        for (hz = (int)STEPPER_HZ_MIN; hz <= (int)STEPPER_HZ_MAX; hz += 10)
        {
            uint16_t a = stepper_math_hz_to_arr((uint16_t)hz);
            if (a >= prev) { mono = 0; }
            prev = a;
        }
        chk(mono == 1, "频率升高时 ARR 单调减小");
    }
}

/* ---------------------------------------------------------------- */
/* 4. 减速步数                                                       */
/* ---------------------------------------------------------------- */
static void test_decel_steps(void)
{
    printf("\n[4] 减速所需步数 (a = %u Hz/ms)\n",
           (unsigned)STEPPER_ACCEL_HZ_PER_MS);
    printf("  120 Hz -> %u 步\n", (unsigned)stepper_math_decel_steps(120));
    printf("  300 Hz -> %u 步\n", (unsigned)stepper_math_decel_steps(300));
    printf("  600 Hz -> %u 步\n", (unsigned)stepper_math_decel_steps(600));

    chk(stepper_math_decel_steps(0)   == 0u, "0 Hz 不需要减速步");
    chk(stepper_math_decel_steps(120) == 1u, "120 Hz -> 1 步 (a=4)");
    chk(stepper_math_decel_steps(300) == 11u, "300 Hz -> 11 步 (a=4)");
    chk(stepper_math_decel_steps(600) == 45u, "600 Hz -> 45 步 (a=4)");

    /* 速度越高，需要越多步才能停下（平方关系） */
    chk(stepper_math_decel_steps(600) > stepper_math_decel_steps(300),
        "高速需要更多减速步");
    chk(stepper_math_decel_steps(300) > stepper_math_decel_steps(120),
        "中速需要更多减速步");
}

/* ---------------------------------------------------------------- */
/* 5. 距离换算                                                       */
/* ---------------------------------------------------------------- */
static void test_distance(void)
{
    uint32_t one_rev = stepper_math_steps_to_mm10(
        (int32_t)stepper_math_steps_per_rev());
    uint32_t expect = ((uint32_t)TRACK_WHEEL_DIAMETER_MM * 31416u) / 1000u;

    printf("\n[5] 半步 -> 距离 (轮径 %u mm)\n",
           (unsigned)TRACK_WHEEL_DIAMETER_MM);
    printf("  一圈 (%lu 步) = %lu.%lu mm (期望 %lu.%lu mm)\n",
           (unsigned long)stepper_math_steps_per_rev(),
           (unsigned long)(one_rev / 10u), (unsigned long)(one_rev % 10u),
           (unsigned long)(expect / 10u), (unsigned long)(expect % 10u));

    chk(one_rev == expect, "一圈的周长与 π·D 一致");

    /* 正负对称 */
    chk(stepper_math_steps_to_mm10(-1000) == stepper_math_steps_to_mm10(1000),
        "正负步数得到相同距离");
    chk(stepper_math_steps_to_mm10(0) == 0u, "0 步距离为 0");

    /* 90 度约 1/4 圈 */
    {
        uint32_t q = stepper_math_steps_to_mm10(
            (int32_t)(stepper_math_steps_per_rev() / 4u));
        printf("  1/4 圈 = %lu.%lu mm\n",
               (unsigned long)(q / 10u), (unsigned long)(q % 10u));
        chk((q + 2u >= one_rev / 4u) && (q <= one_rev / 4u + 2u),
            "1/4 圈约为整圈周长的 1/4");
    }

    /* 300 Hz 下跑 1 秒的距离 */
    {
        uint32_t d = stepper_math_steps_to_mm10(300);
        printf("  300 步 = %lu.%lu mm\n",
               (unsigned long)(d / 10u), (unsigned long)(d % 10u));
        chk(d > 0u, "300 步距离大于 0");
    }
}

/* ---------------------------------------------------------------- */
/* 6. 速度档映射                                                     */
/* ---------------------------------------------------------------- */
static void test_speed_map(void)
{
    printf("\n[6] 速度档映射\n");
    printf("  speed0=%u speed1=%u speed2=%u speed3=%u\n",
           (unsigned)stepper_math_speed_to_hz(0),
           (unsigned)stepper_math_speed_to_hz(1),
           (unsigned)stepper_math_speed_to_hz(2),
           (unsigned)stepper_math_speed_to_hz(3));

    chk(stepper_math_speed_to_hz(0) == STEPPER_START_HZ, "speed0 -> 120 Hz");
    chk(stepper_math_speed_to_hz(1) == STEPPER_HZ_SLOW,  "speed1 -> 200 Hz");
    chk(stepper_math_speed_to_hz(2) == STEPPER_HZ_NORMAL,"speed2 -> 300 Hz");
    chk(stepper_math_speed_to_hz(3) == STEPPER_HZ_FAST,  "speed3 -> 600 Hz");
    chk(stepper_math_speed_to_hz(99) == STEPPER_HZ_NORMAL,
        "越界速度取默认 300 Hz");
}

/* ---------------------------------------------------------------- */
/* 7. 模拟一次完整梯形运动                                           */
/* ---------------------------------------------------------------- */
static void test_motion_profile(void)
{
    /* 用纯整数模拟："每毫秒执行一次 Task1ms" 的逻辑，
     * 检查步数是否精确、是否出现减速段、总耗时是否合理。 */
    const int32_t TOTAL = 600;             /* 目标 600 半步 */
    const uint16_t TARGET_HZ = 300u;

    int32_t  remaining = TOTAL;
    int32_t  position  = 0;
    uint16_t cur_hz    = STEPPER_START_HZ;
    uint8_t  decel     = 0u;
    uint32_t t_ms      = 0u;
    double   step_acc  = 0.0;              /* 小数步累计（模拟定时器节拍）*/
    int      saw_accel = 0;
    int      saw_decel = 0;

    while (remaining != 0 && t_ms < 20000u)
    {
        uint32_t absrem = (uint32_t)(remaining > 0 ? remaining : -remaining);
        uint32_t need   = stepper_math_decel_steps(cur_hz);

        if (!decel && absrem <= need) { decel = 1u; }

        {
            uint16_t desired = decel ? STEPPER_HZ_MIN : TARGET_HZ;
            if (cur_hz < desired)
            {
                uint32_t v = (uint32_t)cur_hz + STEPPER_ACCEL_HZ_PER_MS;
                cur_hz = (uint16_t)((v > desired) ? desired : v);
                if (!decel) { saw_accel = 1; }
            }
            else if (cur_hz > desired)
            {
                uint32_t v = (uint32_t)cur_hz - STEPPER_ACCEL_HZ_PER_MS;
                cur_hz = (uint16_t)((v < desired) ? desired : v);
                saw_decel = 1;
            }
        }

        /* 1 ms 内应该走多少步 */
        step_acc += (double)cur_hz / 1000.0;
        while (step_acc >= 1.0 && remaining != 0)
        {
            step_acc -= 1.0;
            if (remaining > 0) { position++; remaining--; }
            else               { position--; remaining++; }
        }

        t_ms++;
    }

    printf("\n[7] 梯形运动模拟：目标 %ld 半步 @ %u Hz\n",
           (long)TOTAL, (unsigned)TARGET_HZ);
    printf("  实际步数 = %ld   位置 = %ld\n", (long)(TOTAL - remaining),
           (long)position);
    printf("  耗时 = %lu ms   (产生加速度段=%d, 减速段=%d)\n",
           (unsigned long)t_ms, saw_accel, saw_decel);

    chk(remaining == 0, "最终精确走完目标步数（不丢步/不多步）");
    chk(position == TOTAL, "位置累计与目标一致");
    chk(saw_accel == 1, "存在加速段");
    chk(saw_decel == 1, "存在减速段（不是硬停）");

    /* 理论最短时间：600 步 / 300 Hz = 2000 ms，加减速会略微增加 */
    printf("  理论匀速耗时 = %.0f ms\n", 600.0 / 300.0 * 1000.0);
    chk(t_ms > 1900u && t_ms < 2600u, "总耗时在合理范围（2000~2600 ms）");
}

int main(void)
{
    printf("==============================================\n");
    printf(" stepper_math 主机测试\n");
    printf(" (相序 / 相位推进 / ARR / 减速 / 距离 / 梯形速度)\n");
    printf("==============================================\n");

    test_sequence_table();
    test_phase_advance();
    test_hz_to_arr();
    test_decel_steps();
    test_distance();
    test_speed_map();
    test_motion_profile();

    printf("\n==============================================\n");
    printf(" 结果: %d PASS / %d FAIL\n", g_pass, g_fail);
    printf("==============================================\n");
    return (g_fail == 0) ? 0 : 1;
}
