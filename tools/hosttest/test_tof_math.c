/**
  ******************************************************************************
  * @file    tools/hosttest/test_tof_math.c
  * @brief   TOF 保护判定纯函数的单元测试（在 PC 上用 gcc 跑，不需要硬件）
  *
  * 编译：
  *   gcc -std=c99 -Wall -Wextra -Werror -O2 -I tools/hosttest -I Hardware \
  *       tools/hosttest/test_tof_math.c -o t && ./t
  ******************************************************************************
  */
#include <stdio.h>
#include <string.h>
#include "app_config.h"
#include "tof_math.h"

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

static void test_range_ok(void)
{
    printf("\n[1] 读数有效性\n");

    chk(tof_math_range_ok(0, 2000)    == 0u, "0 mm 无效");
    chk(tof_math_range_ok(-5, 2000)   == 0u, "负值无效");
    chk(tof_math_range_ok(1, 2000)    == 1u, "1 mm 有效");
    chk(tof_math_range_ok(20, 2000)   == 1u, "20 mm（底盘离桌高度）有效");
    chk(tof_math_range_ok(2000, 2000) == 1u, "2000 mm 边界有效");
    chk(tof_math_range_ok(2001, 2000) == 0u, "2001 mm 超出量程无效");
    chk(tof_math_range_ok(8190, 2000) == 0u, "8190（无回波）无效");
}

static void test_median3(void)
{
    printf("\n[2] 三点中值滤波\n");

    chk(tof_math_median3(10, 20, 30) == 20, "升序取中");
    chk(tof_math_median3(30, 20, 10) == 20, "降序取中");
    chk(tof_math_median3(10, 999, 12) == 12, "尖峰被压掉");
    chk(tof_math_median3(10, -999, 12) == 10, "负向尖峰被压掉");
    chk(tof_math_median3(5, 5, 5) == 5, "全等");
    chk(tof_math_median3(7, 7, 99) == 7, "两个相同取那个");
}

/* ---------------------------------------------------------------- */

static void test_edge_basic(void)
{
    tof_guard_t g;
    int i;

    printf("\n[3] 桌沿判定：距离变远 = 危险\n");

    tof_math_guard_init(&g, TOF_EDGE_MM, TOF_EDGE_RECOVER_MM, TOF_EDGE_DEBOUNCE);

    /* 桌面上：20 mm，远低于 80 mm，不该触发 */
    for (i = 0; i < 10; i++)
    {
        (void)tof_math_guard_edge(&g, 1u, 20);
    }
    chk(g.active == 0u, "桌面上 20 mm 不触发");

    /* 突然变远到 300 mm，需要 3 次确认 */
    chk(tof_math_guard_edge(&g, 1u, 300) == 0u, "第 1 次变远还不触发");
    chk(tof_math_guard_edge(&g, 1u, 300) == 0u, "第 2 次变远还不触发");
    chk(tof_math_guard_edge(&g, 1u, 300) == 1u, "第 3 次变远触发（去抖=3）");
}

static void test_edge_no_return(void)
{
    tof_guard_t g;
    int i;

    printf("\n[4] 桌沿判定：无回波必须算危险（最关键的一条）\n");

    tof_math_guard_init(&g, TOF_EDGE_MM, TOF_EDGE_RECOVER_MM, TOF_EDGE_DEBOUNCE);

    /* 正下方没有桌面时 VL53L0X 会给无效读数。
     * 如果这里按"无效=安全"处理，机器人会直接掉下去。 */
    for (i = 0; i < (int)TOF_EDGE_DEBOUNCE; i++)
    {
        (void)tof_math_guard_edge(&g, 0u, 0);
    }
    chk(g.active == 1u, "连续无回波 -> 判定为桌沿");
}

static void test_edge_hysteresis(void)
{
    tof_guard_t g;
    int i;

    printf("\n[5] 桌沿判定：回滞区间\n");

    tof_math_guard_init(&g, TOF_EDGE_MM, TOF_EDGE_RECOVER_MM, TOF_EDGE_DEBOUNCE);

    for (i = 0; i < (int)TOF_EDGE_DEBOUNCE; i++)
    {
        (void)tof_math_guard_edge(&g, 1u, 500);
    }
    chk(g.active == 1u, "先触发");

    /* 回到 70 mm：低于 80 的触发值，但高于 60 的解除值 -> 仍保持触发 */
    for (i = 0; i < 10; i++)
    {
        (void)tof_math_guard_edge(&g, 1u, 70);
    }
    chk(g.active == 1u, "70 mm 在回滞区间内，不解锁");

    /* 回到 50 mm：低于 60 -> 解除 */
    for (i = 0; i < (int)TOF_EDGE_DEBOUNCE; i++)
    {
        (void)tof_math_guard_edge(&g, 1u, 50);
    }
    chk(g.active == 0u, "50 mm 低于解除值，解锁");
}

static void test_edge_single_spike(void)
{
    tof_guard_t g;

    printf("\n[6] 桌沿判定：单次跳变不应误报\n");

    tof_math_guard_init(&g, TOF_EDGE_MM, TOF_EDGE_RECOVER_MM, TOF_EDGE_DEBOUNCE);

    (void)tof_math_guard_edge(&g, 1u, 20);
    (void)tof_math_guard_edge(&g, 1u, 900);   /* 单次尖峰 */
    (void)tof_math_guard_edge(&g, 1u, 20);

    chk(g.active == 0u, "单次尖峰被去抖挡住");
    chk(g.confirm_cnt == 0u, "计数在安全样本后清零");
}

/* ---------------------------------------------------------------- */

static void test_obstacle_basic(void)
{
    tof_guard_t g;
    int i;

    printf("\n[7] 障碍判定：距离变近 = 危险\n");

    tof_math_guard_init(&g, TOF_OBSTACLE_MM, TOF_OBSTACLE_RECOVER_MM,
                        TOF_OBSTACLE_DEBOUNCE);

    for (i = 0; i < 5; i++)
    {
        (void)tof_math_guard_obstacle(&g, 1u, 800);
    }
    chk(g.active == 0u, "前方 800 mm 空旷，不触发");

    chk(tof_math_guard_obstacle(&g, 1u, 100) == 0u, "第 1 次变近还不触发");
    chk(tof_math_guard_obstacle(&g, 1u, 100) == 1u, "第 2 次变近触发（去抖=2）");
}

static void test_obstacle_no_return(void)
{
    tof_guard_t g;
    int i;

    printf("\n[8] 障碍判定：无回波必须算【安全】（与桌沿相反）\n");

    tof_math_guard_init(&g, TOF_OBSTACLE_MM, TOF_OBSTACLE_RECOVER_MM,
                        TOF_OBSTACLE_DEBOUNCE);

    /* 前方空旷 -> 没有回波。如果这里按"无效=危险"处理，
     * 桌宠在空地上会永远动不了。 */
    for (i = 0; i < 20; i++)
    {
        (void)tof_math_guard_obstacle(&g, 0u, 0);
    }
    chk(g.active == 0u, "连续无回波 -> 不触发障碍");
}

static void test_obstacle_auto_recover(void)
{
    tof_guard_t g;
    int i;

    printf("\n[9] 障碍判定：远离后自动解除（不需要 CLEAR_FAULT）\n");

    tof_math_guard_init(&g, TOF_OBSTACLE_MM, TOF_OBSTACLE_RECOVER_MM,
                        TOF_OBSTACLE_DEBOUNCE);

    for (i = 0; i < (int)TOF_OBSTACLE_DEBOUNCE; i++)
    {
        (void)tof_math_guard_obstacle(&g, 1u, 100);
    }
    chk(g.active == 1u, "先触发障碍");

    /* 170 mm 在回滞区间（>=150 但 <200）-> 保持 */
    for (i = 0; i < 10; i++)
    {
        (void)tof_math_guard_obstacle(&g, 1u, 170);
    }
    chk(g.active == 1u, "170 mm 在回滞区间内，保持触发");

    /* 250 mm -> 解除 */
    for (i = 0; i < (int)TOF_OBSTACLE_DEBOUNCE; i++)
    {
        (void)tof_math_guard_obstacle(&g, 1u, 250);
    }
    chk(g.active == 0u, "250 mm 远离后自动解除");
}

/* ---------------------------------------------------------------- */

static void test_guard_reset(void)
{
    tof_guard_t g;
    int i;

    printf("\n[10] 判定器复位\n");

    tof_math_guard_init(&g, TOF_EDGE_MM, TOF_EDGE_RECOVER_MM, TOF_EDGE_DEBOUNCE);
    for (i = 0; i < (int)TOF_EDGE_DEBOUNCE; i++)
    {
        (void)tof_math_guard_edge(&g, 1u, 500);
    }
    chk(g.active == 1u, "触发后再复位");

    tof_math_guard_reset(&g);
    chk(g.active == 0u, "复位后 active=0");
    chk(g.confirm_cnt == 0u, "复位后 confirm_cnt=0");
    chk(g.trig_mm == TOF_EDGE_MM, "复位保留阈值配置");

    /* 复位后必须重新累计确认次数 */
    chk(tof_math_guard_edge(&g, 1u, 500) == 0u, "复位后第 1 次不立即触发");
}

static void test_confirm_n_zero(void)
{
    tof_guard_t g;

    printf("\n[11] confirm_n=0 按 1 处理（除零/永不适配防护）\n");

    tof_math_guard_init(&g, TOF_EDGE_MM, TOF_EDGE_RECOVER_MM, 0u);
    chk(g.confirm_n == 1u, "confirm_n=0 被修正为 1");
    chk(tof_math_guard_edge(&g, 1u, 500) == 1u, "单次即触发");
}

static void test_value_invariance(void)
{
    tof_guard_t g;
    int i;

    printf("\n[12] 阈值边界\n");

    tof_math_guard_init(&g, TOF_EDGE_MM, TOF_EDGE_RECOVER_MM, 1u);

    /* 恰好等于触发阈值不算触发（判定用 > ）*/
    (void)tof_math_guard_edge(&g, 1u, TOF_EDGE_MM);
    chk(g.active == 0u, "恰好 == 80 mm 不触发（严格大于）");

    tof_math_guard_reset(&g);
    (void)tof_math_guard_edge(&g, 1u, TOF_EDGE_MM + 1);
    chk(g.active == 1u, "81 mm 触发");
    (void)i;
}

/* ---------------------------------------------------------------- */

int main(void)
{
    printf("==============================================\n");
    printf(" TOF 保护判定 单元测试\n");
    printf("==============================================\n");
    printf(" 配置：EDGE>%d/<=%d x%u   OBST<%d/>=%d x%u\n",
           (int)TOF_EDGE_MM, (int)TOF_EDGE_RECOVER_MM,
           (unsigned)TOF_EDGE_DEBOUNCE,
           (int)TOF_OBSTACLE_MM, (int)TOF_OBSTACLE_RECOVER_MM,
           (unsigned)TOF_OBSTACLE_DEBOUNCE);

    test_range_ok();
    test_median3();
    test_edge_basic();
    test_edge_no_return();
    test_edge_hysteresis();
    test_edge_single_spike();
    test_obstacle_basic();
    test_obstacle_no_return();
    test_obstacle_auto_recover();
    test_guard_reset();
    test_confirm_n_zero();
    test_value_invariance();

    printf("\n==============================================\n");
    printf(" 结果: %d PASS / %d FAIL\n", g_pass, g_fail);
    printf("==============================================\n");

    return (g_fail == 0) ? 0 : 1;
}
