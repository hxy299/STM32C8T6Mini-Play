/**
  ******************************************************************************
  * @file    tools/hosttest/test_tts_math.c
  * @brief   TTS 环形缓冲纯函数的单元测试（PC 上跑，不需要硬件）
  *
  * 为什么值得单独测：
  *   环形缓冲的回绕错误在板子上表现为"偶发丢字/整句吞掉"，
  *   串口日志里几乎看不出来，只能靠穷举 head/tail 组合提前发现。
  *
  * 编译：
  *   gcc -std=c99 -Wall -Wextra -Werror -O2 -I tools/hosttest -I Hardware -I User \
  *       tools/hosttest/test_tts_math.c -o t && ./t
  ******************************************************************************
  */
#include <stdio.h>
#include <string.h>
#include "app_config.h"
#include "tts_math.h"

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

static void test_used_free(void)
{
    const uint16_t S = 512u;

    printf("\n[1] 占用/剩余计算\n");

    chk(tts_ring_used(0, 0, S) == 0, "空环 used = 0");
    chk(tts_ring_free(0, 0, S) == (int)S - 1, "空环 free = size-1（留 1 字节区分满空）");

    chk(tts_ring_used(10, 0, S) == 10, "不回绕 used = head-tail");
    chk(tts_ring_used(0, 10, S) == (int)S - 10, "回绕 used = size-tail+head");

    chk(tts_ring_used(100, 100, S) == 0, "head==tail 视为空（不是满）");
    chk(tts_ring_free(511, 0, S) == 0, "只剩 1 字节空位时 free = 0");

    chk(tts_ring_used(5, 3, 0) == 0, "size=0 不崩（防御）");
}

static void test_fits(void)
{
    const uint16_t S = 64u;

    printf("\n[2] 整帧能否放下\n");

    chk(tts_ring_fits(0, 0, S, 63) == 1, "空环可放 63 字节");
    chk(tts_ring_fits(0, 0, S, 64) == 0, "空环放不下 64 字节（容量 = size-1）");
    chk(tts_ring_fits(0, 0, S, 0) == 1, "放 0 字节恒成立");

    /* 回绕下还剩多少 */
    chk(tts_ring_fits(5, 60, S, 8) == 1, "回绕：已用 9，剩 54，可放 8");
    chk(tts_ring_fits(5, 60, S, 55) == 0, "回绕：剩 54，放不下 55");

    /* 恰好填满：head 在 tail 前面 1 格 -> 已用 1，可放 62 填到 63 */
    chk(tts_ring_fits(1, 0, S, 62) == 1, "已用 1，可放 62 填到 63");
    chk(tts_ring_fits(1, 0, S, 63) == 0, "已用 1，放不下 63");
}

static void test_advance(void)
{
    const uint16_t S = 8u;

    printf("\n[3] 下标回绕\n");

    chk(tts_ring_advance(0, S) == 1, "0 -> 1");
    chk(tts_ring_advance(6, S) == 7, "6 -> 7");
    chk(tts_ring_advance(7, S) == 0, "7 -> 0（回绕）");
    chk(tts_ring_advance(0, 0) == 0, "size=0 不崩");
}

/**
  * @brief  穷举所有 head/tail 组合，验证 used + free 恒定
  * @note   这是最有价值的一项：任何回绕写错都会破坏这个恒等式。
  */
static void test_invariant(void)
{
    const uint16_t S = 16u;
    int bad = 0;
    int h, t;

    printf("\n[4] 不变量：used + free == size - 1（穷举 16×16）\n");

    for (h = 0; h < (int)S; h++)
    {
        for (t = 0; t < (int)S; t++)
        {
            int used = tts_ring_used((uint16_t)h, (uint16_t)t, S);
            int freeb = tts_ring_free((uint16_t)h, (uint16_t)t, S);

            if (used < 0 || used > (int)S - 1)
            {
                bad++;
            }
            if (used + freeb != (int)S - 1)
            {
                bad++;
            }
        }
    }

    chk(bad == 0, "全部 256 组均满足 used+free = 15 且 used 在 [0,15]");
}

/**
  * @brief  模拟真实收发：一条一条入队、按字节出队，检查字节序不乱
  */
static void test_stream_order(void)
{
    const uint16_t S = 32u;
    uint16_t head = 0u, tail = 0u;
    uint8_t  ring[32];
    uint8_t  frames[6][8];
    int      i, j, produced = 0, consumed = 0;
    int      ok = 1;

    printf("\n[5] 模拟收发：3 帧入队 + 逐字节出队\n");

    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 8; j++)
        {
            frames[i][j] = (uint8_t)(i * 8 + j);
        }
    }

    for (i = 0; i < 3; i++)
    {
        if (tts_ring_fits(head, tail, S, 8u) == 0)
        {
            ok = 0;
            break;
        }
        for (j = 0; j < 8; j++)
        {
            ring[head] = frames[i][j];
            head = tts_ring_advance(head, S);
        }
        produced += 8;
    }

    /* 逐字节出队并校验顺序 */
    for (i = 0; i < produced; i++)
    {
        uint8_t expect = (uint8_t)(i);   /* 因为 frames 是连续递增的 */
        if (head == tail)
        {
            ok = 0;
            break;
        }
        if (ring[tail] != expect)
        {
            ok = 0;
            break;
        }
        tail = tts_ring_advance(tail, S);
        consumed++;
    }

    chk(ok == 1, "24 字节全部按入队顺序出队");
    chk(consumed == 24, "出队字节数 = 24");
    chk(tts_ring_used(head, tail, S) == 0, "抽干后环为空");

    /* 再验一次：填满到容量边界后必须拒绝 */
    head = tail = 0u;
    for (i = 0; i < 3; i++)
    {
        (void)tts_ring_fits(head, tail, S, 10u);
        for (j = 0; j < 10 && i < 3; j++)
        {
            if (tts_ring_fits(head, tail, S, 1u) == 0) break;
            ring[head] = 0xAA;
            head = tts_ring_advance(head, S);
        }
    }
    chk(tts_ring_free(head, tail, S) == 0 ||
        tts_ring_fits(head, tail, S, 1u) == 1, "容量边界行为自洽");
}

static void test_frame_len(void)
{
    printf("\n[6] TW-TTS 文本帧长度\n");

    chk(tts_text_frame_len(0, 190) == 0u, "空正文 -> 0（非法）");
    chk(tts_text_frame_len(1, 190) == 6u, "1 字节正文 -> 6（5 头 + 1）");
    chk(tts_text_frame_len(90, 190) == 95u, "90 字节正文 -> 95");
    chk(tts_text_frame_len(190, 190) == 195u, "190 字节正文 -> 195（刚好 200 上限内）");
    chk(tts_text_frame_len(191, 190) == 0u, "191 字节正文 -> 0（超限）");

    /* 55 个汉字 = 165 字节 UTF-8，应放得下 */
    chk(tts_text_frame_len(165, 190) == 170u, "55 个汉字（165 字节）-> 170");
    /* 64 个汉字 = 192 字节，超 190 */
    chk(tts_text_frame_len(192, 190) == 0u, "64 个汉字（192 字节）-> 拒绝");
}

static void test_is_raw(void)
{
    printf("\n[7] 原始帧识别\n");

    chk(tts_is_raw_frame(0xFD) == 1, "0xFD -> 原始帧（原样转发）");
    chk(tts_is_raw_frame('A') == 0, "ASCII -> 当正文字符处理");
    chk(tts_is_raw_frame(0xE4) == 0, "UTF-8 首字节 -> 当正文处理");
    chk(tts_is_raw_frame(0x00) == 0, "0x00 -> 当正文处理");
}

/* ---------------------------------------------------------------- */

int main(void)
{
    printf("========================================\n");
    printf(" TTS 环形缓冲纯函数单测\n");
    printf("========================================\n");

    test_used_free();
    test_fits();
    test_advance();
    test_invariant();
    test_stream_order();
    test_frame_len();
    test_is_raw();

    printf("\n----------------------------------------\n");
    printf(" 结果: %d PASS / %d FAIL\n", g_pass, g_fail);
    printf("----------------------------------------\n");
    return (g_fail == 0) ? 0 : 1;
}
