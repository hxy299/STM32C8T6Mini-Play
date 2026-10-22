/**
  ******************************************************************************
  * @file    tools/hosttest/test_servo_math.c
  * @brief   舵机角度->脉宽 主机测试（M3 核心算法验证）
  *
  * 编译运行（WSL）：
  *   gcc -std=c99 -Wall -Wextra -Werror -O2 \
  *       -I tools/hosttest -I Hardware \
  *       tools/hosttest/test_servo_math.c -o /tmp/t_servo && /tmp/t_servo
  ******************************************************************************
  */
#include <stdio.h>
#include <stdint.h>
#include "servo_math.h"

static int g_pass = 0;
static int g_fail = 0;

static void chk(int cond, const char *name)
{
    if (cond) { g_pass++; printf("  [PASS] %s\n", name); }
    else      { g_fail++; printf("  [FAIL] %s\n", name); }
}

int main(void)
{
    uint8_t  clamped;
    int8_t   d;
    uint16_t us;

    printf("==============================================\n");
    printf(" servo_math 主机测试\n");
    printf("==============================================\n");

    /* ---- 1. 关键点位 ---- */
    printf("\n[1] 关键点位\n");
    printf("  center  = %u us\n", (unsigned)servo_math_deg_to_us(0));
    printf("  +45 deg = %u us\n", (unsigned)servo_math_deg_to_us(45));
    printf("  -45 deg = %u us\n", (unsigned)servo_math_deg_to_us(-45));
    chk(servo_math_deg_to_us(0)   == 1500u, "0 deg   -> 1500 us");
    chk(servo_math_deg_to_us(45)  == 2000u, "+45 deg -> 2000 us");
    chk(servo_math_deg_to_us(-45) == 1000u, "-45 deg -> 1000 us");

    /* ---- 2. 中间点（线性） ---- */
    printf("\n[2] 中间点线性\n");
    d = 90;  /* 会被钳到 45 */
    us = servo_math_deg_to_us(d);
    printf("  90 deg(钳位) = %u us\n", (unsigned)us);
    chk(us == 2000u, "90 deg 被钳到 45 -> 2000 us");

    chk(servo_math_deg_to_us(9)  == 1600u, "+9 deg  -> 1600 us");
    chk(servo_math_deg_to_us(18) == 1700u, "+18 deg -> 1700 us");
    chk(servo_math_deg_to_us(27) == 1800u, "+27 deg -> 1800 us");
    chk(servo_math_deg_to_us(-9) == 1400u, "-9 deg  -> 1400 us");
    chk(servo_math_deg_to_us(-18)== 1300u, "-18 deg -> 1300 us");

    /* ---- 3. 越界钳位 ---- */
    printf("\n[3] 越界钳位\n");
    clamped = 0u;
    d = servo_math_clamp(180, &clamped);
    printf("  clamp(180) -> %d, clamped=%u\n", (int)d, (unsigned)clamped);
    chk(d == 45 && clamped == 1u, "180 -> 45 且标记 clamped");

    clamped = 0u;
    d = servo_math_clamp(-180, &clamped);
    printf("  clamp(-180) -> %d, clamped=%u\n", (int)d, (unsigned)clamped);
    chk(d == -45 && clamped == 1u, "-180 -> -45 且标记 clamped");

    clamped = 0u;
    d = servo_math_clamp(30, &clamped);
    chk(d == 30 && clamped == 0u, "30 不钳位且 clamped=0");

    clamped = 0u;
    d = servo_math_clamp(45, &clamped);
    chk(d == 45 && clamped == 0u, "45 边界不标记钳位");

    /* ---- 4. 全范围单调性 + 输出边界 ---- */
    printf("\n[4] 全范围单调性 (-90..90)\n");
    {
        int16_t a;
        int     mono = 1;
        int     inrange = 1;
        uint16_t prev = 0u;

        for (a = -90; a <= 90; a++)
        {
            uint16_t cur = servo_math_deg_to_us(a);
            if (a > -90 && cur < prev) { mono = 0; }
            if (cur < 1000u || cur > 2000u) { inrange = 0; }
            prev = cur;
        }
        chk(mono == 1, "角度增大时脉宽单调不减");
        chk(inrange == 1, "任意输入下脉宽都在 1000..2000 us 内");
    }

    /* ---- 5. 负角度不能出现浮点/符号错误 ---- */
    printf("\n[5] 负角度符号处理\n");
    chk(servo_math_deg_to_us(-1) == 1488u || servo_math_deg_to_us(-1) == 1489u,
        "-1 deg 接近 1500（符号正确，不跳变到 1500+）");
    printf("  -1 deg = %u us\n", (unsigned)servo_math_deg_to_us(-1));
    chk(servo_math_deg_to_us(-45) < servo_math_deg_to_us(0),
        "-45 的脉宽小于 0 的脉宽");

    printf("\n==============================================\n");
    printf(" 结果: %d PASS / %d FAIL\n", g_pass, g_fail);
    printf("==============================================\n");
    return (g_fail == 0) ? 0 : 1;
}
