/**
  ******************************************************************************
  * @file    tools/hosttest/test_link_proto.c
  * @brief   协议逻辑主机测试（CRC16 / 组帧 / 拆帧 / 重同步 / 帧间超时）
  *
  * 目的：在没有 ARM 工具链、没有硬件的情况下，
  *       先把最容易出错也最不该出错的部分验证到位。
  *
  * 编译运行（WSL）：
  *   gcc -std=c99 -Wall -Wextra -Werror -O2 \
  *       -I tools/hosttest -I Protocol \
  *       tools/hosttest/test_link_proto.c Protocol/link_proto.c \
  *       -o /tmp/t_link && /tmp/t_link
  ******************************************************************************
  */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "link_proto.h"

static int      g_pass = 0;
static int      g_fail = 0;

/* 模拟时间：每喂一个字节前进 1 ms（115200 下约 0.087 ms/字节，留足余量）*/
static uint32_t g_now = 0u;

static void check(int cond, const char *name)
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

/* 喂一个字节（自动推进模拟时间 1 ms）*/
static int feed(uint8_t b, link_frame_t *f)
{
    g_now += 1u;
    return Link_FeedByte(b, g_now, f);
}

/* 推进模拟时间（模拟线路空闲 gap 毫秒）*/
static void idle_gap(uint32_t ms)
{
    g_now += ms;
}

/* 把一段字节全部喂进去，返回成功解析的帧数 */
static int feed_all(const uint8_t *buf, int n, link_frame_t *last)
{
    int i;
    int got = 0;

    for (i = 0; i < n; i++)
    {
        if (feed(buf[i], last) == 1)
        {
            got++;
        }
    }
    return got;
}

/* ---------------------------------------------------------------- */
/* 1. CRC16-CCITT-FALSE 标准检验值                                   */
/* ---------------------------------------------------------------- */
static void test_crc_known_answer(void)
{
    const uint8_t v[] = "123456789";
    uint16_t crc = Link_Crc16(v, 9);

    printf("\n[1] CRC16-CCITT-FALSE 已知答案\n");
    printf("  \"123456789\" -> 0x%04X (期望 0x29B1)\n", crc);
    check(crc == 0x29B1, "CRC 标准检验值 0x29B1");
    check(Link_Crc16(v, 0) == 0xFFFF, "空输入返回初值 0xFFFF");
}

/* ---------------------------------------------------------------- */
/* 2. 组帧 -> 拆帧 往返                                              */
/* ---------------------------------------------------------------- */
static void test_roundtrip(void)
{
    uint8_t      buf[240];
    uint8_t      payload[5] = { 0x01, 0x02, 0x03, 0x04, 0x05 };
    link_frame_t f;
    int          n;
    int          got;

    printf("\n[2] 组帧 -> 拆帧 往返\n");

    n = Link_BuildFrame(buf, sizeof(buf), 0x01, 0x1234, 0x06, payload, 5);
    printf("  帧长 = %d (期望 13 = 6+5+2)\n", n);
    check(n == 13, "帧长度 13");
    check(buf[0] == 0xA5, "SOF = 0xA5");
    check(buf[1] == 0x01, "TYPE = 0x01");
    check(buf[2] == 0x34 && buf[3] == 0x12, "SEQ 小端 = 0x1234");
    check(buf[4] == 0x06, "OP = 0x06");
    check(buf[5] == 0x05, "LEN = 5");

    memset(&f, 0, sizeof(f));
    Link_ParserReset();
    got = feed_all(buf, n, &f);
    check(got == 1, "解析出 1 帧");
    check(f.type == 0x01 && f.seq == 0x1234 && f.op == 0x06 && f.len == 5,
          "type/seq/op/len 全部正确");
    check(memcmp(f.payload, payload, 5) == 0, "payload 逐字节一致");
}

/* ---------------------------------------------------------------- */
/* 3. 零长度 payload（PING 就是这种）                                 */
/* ---------------------------------------------------------------- */
static void test_zero_payload(void)
{
    uint8_t      buf[32];
    link_frame_t f;
    int          n;
    int          got;

    printf("\n[3] 零长度 payload（PING 场景）\n");

    n = Link_BuildFrame(buf, sizeof(buf), 0x01, 7, 0x06, NULL, 0);
    check(n == 8, "帧长 8");

    memset(&f, 0, sizeof(f));
    Link_ParserReset();
    got = feed_all(buf, n, &f);
    check(got == 1 && f.len == 0 && f.seq == 7, "解析成功且 len=0");
}

/* ---------------------------------------------------------------- */
/* 4. CRC 损坏必须被拒                                               */
/* ---------------------------------------------------------------- */
static void test_crc_corruption(void)
{
    uint8_t      buf[64];
    link_frame_t f;
    uint32_t     bad_before;
    int          n;
    int          got;

    printf("\n[4] CRC 损坏 -> 必须拒绝\n");

    n = Link_BuildFrame(buf, sizeof(buf), 0x01, 1, 0x06, NULL, 0);
    buf[4] ^= 0xFF;

    memset(&f, 0, sizeof(f));
    Link_ParserReset();
    bad_before = Link_GetBadFrames();
    got = feed_all(buf, n, &f);
    check(got == 0, "坏帧未被当成好帧");
    check(Link_GetBadFrames() > bad_before, "坏帧计数增加");
}

/* ---------------------------------------------------------------- */
/* 5. 前导垃圾 + 空闲间隔 -> 自动重同步（关键用例）                    */
/*    垃圾里含假 SOF，且后面跟着非法结构；靠帧间超时恢复               */
/* ---------------------------------------------------------------- */
static void test_resync_with_gap(void)
{
    uint8_t      junk[7] = { 0x00, 0xFF, 0x11, 0x22, 0xA5, 0x99, 0x33 };
    uint8_t      buf[32];
    link_frame_t f;
    int          n;
    uint32_t     i;
    int          got;

    printf("\n[5] 前导垃圾 + 空闲间隔 -> 重同步\n");

    n = Link_BuildFrame(buf, sizeof(buf), 0x01, 0x55, 0x06, NULL, 0);

    memset(&f, 0, sizeof(f));
    Link_ParserReset();

    for (i = 0; i < sizeof(junk); i++)
    {
        (void)feed(junk[i], &f);
    }

    /* 真实环境里两帧之间必然有空闲（PING 500ms 一次），这里模拟 200ms */
    idle_gap(200u);

    got = feed_all(buf, n, &f);
    check(got == 1, "垃圾之后仍能解析出正确帧（帧间超时生效）");
    check(f.seq == 0x55, "帧内容正确");
}

/* ---------------------------------------------------------------- */
/* 6. payload 里出现 0xA5 不能被误判为帧头                            */
/* ---------------------------------------------------------------- */
static void test_a5_inside_payload(void)
{
    uint8_t      payload[8] = { 0xA5, 0xA5, 0x00, 0xA5, 0xFF, 0xA5, 0x01, 0xA5 };
    uint8_t      buf[32];
    link_frame_t f;
    int          n;
    int          got;

    printf("\n[6] payload 内含 0xA5\n");

    n = Link_BuildFrame(buf, sizeof(buf), 0x01, 0xAA, 0x03, payload, 8);
    memset(&f, 0, sizeof(f));
    Link_ParserReset();
    got = feed_all(buf, n, &f);
    check(got == 1, "只解析出 1 帧（没有在 payload 里错误切分）");
    check(f.len == 8 && memcmp(f.payload, payload, 8) == 0, "payload 完整无误");
}

/* ---------------------------------------------------------------- */
/* 7. 长度边界                                                       */
/* ---------------------------------------------------------------- */
static void test_length_overflow(void)
{
    uint8_t      out[300];
    uint8_t      big[250];
    link_frame_t f;
    int          n;
    int          r;

    printf("\n[7] 长度边界\n");

    memset(big, 0x00, sizeof(big));

    n = Link_BuildFrame(out, sizeof(out), 0x01, 1, 0x03, big, 201);
    check(n < 0, "payload=201 组帧被拒");

    n = Link_BuildFrame(out, sizeof(out), 0x01, 1, 0x03, big, 200);
    check(n > 0, "payload=200 组帧允许");

    memset(&f, 0, sizeof(f));
    Link_ParserReset();
    (void)feed(0xA5, &f);
    (void)feed(0x01, &f);
    (void)feed(0x00, &f);
    (void)feed(0x00, &f);
    (void)feed(0x03, &f);
    r = feed(201, &f);
    check(r == -1, "解析侧 LEN=201 返回 -1");

    n = Link_BuildFrame(out, 5, 0x01, 1, 0x06, NULL, 0);
    check(n < 0, "输出缓冲不足被拒");
}

/* ---------------------------------------------------------------- */
/* 8. 连续多帧                                                        */
/* ---------------------------------------------------------------- */
static void test_back_to_back(void)
{
    uint8_t      buf[128];
    link_frame_t f;
    int          off = 0;
    int          i;
    int          k;
    int          frames = 0;

    printf("\n[8] 连续多帧背靠背\n");

    for (k = 0; k < 5; k++)
    {
        int n = Link_BuildFrame(&buf[off], (uint16_t)(sizeof(buf) - (size_t)off),
                                0x01, (uint16_t)k, 0x06, NULL, 0);
        if (n < 0)
        {
            break;
        }
        off += n;
    }

    memset(&f, 0, sizeof(f));
    Link_ParserReset();
    for (i = 0; i < off; i++)
    {
        if (feed(buf[i], &f) == 1)
        {
            if (f.seq == (uint16_t)frames)
            {
                frames++;
            }
        }
    }
    check(frames == 5, "连续解析出 5 帧且 seq 递增正确");
}

/* ---------------------------------------------------------------- */
/* 9. 截断帧不应产生半帧                                              */
/* ---------------------------------------------------------------- */
static void test_truncated(void)
{
    uint8_t      buf[64];
    link_frame_t f;
    int          n;
    int          got;

    printf("\n[9] 截断帧\n");

    n = Link_BuildFrame(buf, sizeof(buf), 0x01, 9, 0x06, NULL, 0);
    memset(&f, 0, sizeof(f));
    Link_ParserReset();
    got = feed_all(buf, n - 1, &f);     /* 故意少一个字节 */
    check(got == 0, "少一个字节不会产生帧");
}

/* ---------------------------------------------------------------- */
/* 10. 假 SOF 吞数据后，靠超时恢复（核心健壮性验证）                  */
/* ---------------------------------------------------------------- */
static void test_false_sof_recovery(void)
{
    uint8_t      phantom[6] = { 0xA5, 0x01, 0x00, 0x00, 0x03, 0x55 }; /* LEN=85 */
    uint8_t      buf[32];
    link_frame_t f;
    int          n;
    int          got;

    printf("\n[10] 假 SOF 吞数据 -> 超时恢复\n");

    n = Link_BuildFrame(buf, sizeof(buf), 0x01, 0x77, 0x06, NULL, 0);

    memset(&f, 0, sizeof(f));
    Link_ParserReset();

    /* 假帧头声称有 85 字节 payload，会进入等待状态 */
    (void)feed_all(phantom, (int)sizeof(phantom), &f);

    /* 真实帧在 200 ms 后才到（正常节奏），超时保护应丢弃那个半帧 */
    idle_gap(200u);
    got = feed_all(buf, n, &f);

    check(got == 1, "超时后真实帧被正确解析");
    check(f.seq == 0x77, "帧内容正确");
    check(Link_GetTimeoutDrops() > 0, "超时丢弃计数增加");
}

/* ---------------------------------------------------------------- */
/* 11. 统计一致性                                                     */
/* ---------------------------------------------------------------- */
static void test_stats(void)
{
    printf("\n[11] 统计计数\n");
    printf("  good=%lu bad=%lu drop=%lu timeout=%lu\n",
           (unsigned long)Link_GetGoodFrames(),
           (unsigned long)Link_GetBadFrames(),
           (unsigned long)Link_GetByteDrops(),
           (unsigned long)Link_GetTimeoutDrops());
    check(Link_GetGoodFrames() > 0, "good 计数大于 0");
    check(Link_GetBadFrames() > 0, "bad 计数大于 0");
}

int main(void)
{
    printf("==============================================\n");
    printf(" link_proto 主机测试\n");
    printf(" (CRC16 / 组帧 / 拆帧 / 重同步 / 帧间超时)\n");
    printf("==============================================\n");

    test_crc_known_answer();
    test_roundtrip();
    test_zero_payload();
    test_crc_corruption();
    test_resync_with_gap();
    test_a5_inside_payload();
    test_length_overflow();
    test_back_to_back();
    test_truncated();
    test_false_sof_recovery();
    test_stats();

    printf("\n==============================================\n");
    printf(" 结果: %d PASS / %d FAIL\n", g_pass, g_fail);
    printf("==============================================\n");

    return (g_fail == 0) ? 0 : 1;
}
