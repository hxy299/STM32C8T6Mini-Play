/**
  ******************************************************************************
  * @file    Service/config.h
  * @brief   运行时功能开关（配置字）+ Flash 持久化
  *
  * 配置字布局（见说明书 §5.6.1）：
  *   bit0 SERVO_EN   bit1 TTS_EN   bit2 BT_EN
  *   bit3 TOF0_EN    bit4 TOF1_EN  bit5 TOF2_EN   bit6 TOF3_EN
  *
  * 存储：Flash page 63 @ 0x0800FC00
  *   +0  magic  4B  "C8T6" = 0x43385436
  *   +4  version 2B
  *   +6  config  4B
  *   +10 crc16   2B（覆盖 +0..+9）
  ******************************************************************************
  */
#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdint.h>

/* 返回值 */
#define CONFIG_OK            0
#define CONFIG_ERR_FLASH    -1

/* ---------- 初始化：从 Flash 加载，失败则用默认值 ---------- */
void     Config_Load(void);

/* ---------- 查询 ---------- */
uint32_t Config_Get(void);
uint8_t  Config_IsEnabled(uint32_t mask);
uint8_t  Config_IsDefault(void);
uint8_t  Config_IsBottomTofAllDisabled(void);   /* 触发盲走模式的判据 */
uint8_t  Config_GetBottomTofCount(void);        /* 启用的桌沿传感器数量 0..3 */

/* ---------- 修改 ----------
 * 返回 CONFIG_OK 或 CONFIG_ERR_FLASH。
 * 值未变化时不擦写 Flash（保护寿命），直接返回 OK。
 * 若 Flash 写失败，内存中的配置仍会更新，但会在返回码里体现。
 */
int      Config_Set(uint32_t new_config);

/* ---------- 诊断 ---------- */
uint32_t Config_GetFlashWriteCount(void);
uint32_t Config_GetFlashErrorCount(void);

#endif /* __CONFIG_H */
