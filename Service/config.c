/**
  ******************************************************************************
  * @file    Service/config.c
  * @brief   配置字 + Flash 持久化（STM32F103C8T6 page 63）
  *
  * ⚠️ 关于擦写期间的时序影响（重要，见说明书 §5.6.3）：
  *   Flash 擦除约 20~40 ms，期间 CPU 取指会被挂起，TIM3 的 1 ms 中断也会被推迟。
  *   后果：tick 会"少走"若干毫秒（不是漂移，是暂停）。
  *   本项目可接受，因为：
  *     - 心跳超时 1000 ms，40 ms 误差无影响
  *     - IWDG 1.6 s，40 ms 不足以复位
  *     - 只在 SET_CONFIG 时发生，频率极低
  *   后续接入步进后，必须在调用 Config_Set 前先停止运动（说明书写明）。
  ******************************************************************************
  */
#include "stm32f10x.h"
#include <string.h>
#include "app_config.h"
#include "link_proto.h"
#include "config.h"

/* Flash 中的布局（12 字节，按半字编程）*/
#define CFG_OFF_MAGIC_LO   0u
#define CFG_OFF_MAGIC_HI   2u
#define CFG_OFF_VERSION    4u
#define CFG_OFF_CONF_LO    6u
#define CFG_OFF_CONF_HI    8u
#define CFG_OFF_CRC        10u

/* 运行期副本 */
static uint32_t s_config = CFG_DEFAULT;
static uint8_t  s_is_default = 1u;
static uint32_t s_write_count = 0u;
static uint32_t s_error_count = 0u;

/* ============================ 内部工具 ============================ */

static uint16_t cfg_rd16(uint32_t off)
{
    return *(volatile uint16_t *)(CFG_FLASH_ADDR + off);
}

static uint32_t cfg_rd32(uint32_t off)
{
    uint32_t lo = (uint32_t)cfg_rd16(off);
    uint32_t hi = (uint32_t)cfg_rd16(off + 2u);
    return (lo | (hi << 16));
}

/* 计算前 10 字节的 CRC16（magic + version + config）*/
static uint16_t cfg_calc_crc(uint32_t magic, uint16_t version, uint32_t config)
{
    uint8_t  buf[10];
    buf[0] = (uint8_t)(magic & 0xFFu);
    buf[1] = (uint8_t)((magic >> 8) & 0xFFu);
    buf[2] = (uint8_t)((magic >> 16) & 0xFFu);
    buf[3] = (uint8_t)((magic >> 24) & 0xFFu);
    buf[4] = (uint8_t)(version & 0xFFu);
    buf[5] = (uint8_t)((version >> 8) & 0xFFu);
    buf[6] = (uint8_t)(config & 0xFFu);
    buf[7] = (uint8_t)((config >> 8) & 0xFFu);
    buf[8] = (uint8_t)((config >> 16) & 0xFFu);
    buf[9] = (uint8_t)((config >> 24) & 0xFFu);
    /* 复用协议模块的 CRC16，避免两份实现不一致 */
    return Link_Crc16(buf, 10u);
}

/* ============================ 加载 ============================ */

void Config_Load(void)
{
    uint32_t magic;
    uint16_t version;
    uint32_t config;
    uint16_t crc_stored;
    uint16_t crc_calc;

    magic   = cfg_rd32(CFG_OFF_MAGIC_LO);
    version = cfg_rd16(CFG_OFF_VERSION);
    config  = cfg_rd32(CFG_OFF_CONF_LO);
    crc_stored = cfg_rd16(CFG_OFF_CRC);

    /* 全 0xFF 表示从未写过（擦除态）*/
    if (magic == 0xFFFFFFFFu)
    {
        s_config = CFG_DEFAULT;
        s_is_default = 1u;
        return;
    }

    if (magic != CFG_FLASH_MAGIC || version != CFG_FLASH_VERSION)
    {
        s_config = CFG_DEFAULT;
        s_is_default = 1u;
        return;
    }

    crc_calc = cfg_calc_crc(magic, version, config);
    if (crc_calc != crc_stored)
    {
        s_config = CFG_DEFAULT;
        s_is_default = 1u;
        return;
    }

    /* 只接受已知的功能位，其余位一律清掉（防脏数据）*/
    s_config = config & 0x7Fu;
    s_is_default = 0u;
}

/* ============================ 保存 ============================ */

static int cfg_flash_save(uint32_t config)
{
    uint16_t crc;
    FLASH_Status st;
    uint32_t i;

    crc = cfg_calc_crc(CFG_FLASH_MAGIC, CFG_FLASH_VERSION, config);

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    st = FLASH_ErasePage(CFG_FLASH_ADDR);
    if (st != FLASH_COMPLETE)
    {
        FLASH_Lock();
        return CONFIG_ERR_FLASH;
    }

    /* 按半字写入 6 个半字 = 12 字节 */
    {
        uint16_t words[6];
        words[0] = (uint16_t)(CFG_FLASH_MAGIC & 0xFFFFu);
        words[1] = (uint16_t)((CFG_FLASH_MAGIC >> 16) & 0xFFFFu);
        words[2] = (uint16_t)(CFG_FLASH_VERSION & 0xFFFFu);
        words[3] = (uint16_t)(config & 0xFFFFu);
        words[4] = (uint16_t)((config >> 16) & 0xFFFFu);
        words[5] = crc;

        for (i = 0u; i < 6u; i++)
        {
            st = FLASH_ProgramHalfWord(CFG_FLASH_ADDR + (i * 2u), words[i]);
            if (st != FLASH_COMPLETE)
            {
                FLASH_Lock();
                return CONFIG_ERR_FLASH;
            }
        }
    }

    FLASH_Lock();

    /* 回读校验：确保真的写进去了 */
    if (cfg_rd32(CFG_OFF_MAGIC_LO) != CFG_FLASH_MAGIC ||
        cfg_rd32(CFG_OFF_CONF_LO)  != config)
    {
        return CONFIG_ERR_FLASH;
    }

    return CONFIG_OK;
}

int Config_Set(uint32_t new_config)
{
    uint32_t sanitized = new_config & 0x7Fu;

    if (sanitized == s_config)
    {
        return CONFIG_OK;          /* 没变，不擦写 Flash（保护寿命）*/
    }

    /* 先更新内存，保证即使 Flash 失败，本次运行也按新配置走 */
    s_config = sanitized;
    s_is_default = 0u;

    s_write_count++;
    if (cfg_flash_save(sanitized) != CONFIG_OK)
    {
        s_error_count++;
        return CONFIG_ERR_FLASH;
    }

    return CONFIG_OK;
}

/* ============================ 查询 ============================ */

uint32_t Config_Get(void)
{
    return s_config;
}

uint8_t Config_IsEnabled(uint32_t mask)
{
    return ((s_config & mask) != 0u) ? 1u : 0u;
}

uint8_t Config_IsDefault(void)
{
    return s_is_default;
}

uint8_t Config_IsBottomTofAllDisabled(void)
{
    return ((s_config & CFG_BOTTOM_TOF_MASK) == 0u) ? 1u : 0u;
}

/**
  * @brief  启用了几颗底部桌沿传感器（0..3）
  * @note   用于区分"完整覆盖"和"只有前向一颗"：
  *         只有一颗时，后退方向完全没有保护。
  */
uint8_t Config_GetBottomTofCount(void)
{
    uint8_t n = 0u;

    if ((s_config & CFG_TOF0_EN) != 0u) { n++; }
    if ((s_config & CFG_TOF1_EN) != 0u) { n++; }
    if ((s_config & CFG_TOF2_EN) != 0u) { n++; }

    return n;
}

uint32_t Config_GetFlashWriteCount(void)
{
    return s_write_count;
}

uint32_t Config_GetFlashErrorCount(void)
{
    return s_error_count;
}
