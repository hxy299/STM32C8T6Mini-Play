/**
  ******************************************************************************
  * @file    Hardware/tof.h
  * @brief   VL53L0X ×4 驱动（共用 I2C1，靠 XSHUT 逐个分配地址）
  *
  * 分工：
  *   本文件只负责"把距离读出来"，不做任何保护判断。
  *   保护策略在 Service/tof_service.c，纯判定在 Hardware/tof_math.h。
  *
  * 为什么初始化要分地址：
  *   4 颗 VL53L0X 出厂地址全为 0x29，上电后同时响应同一地址，无法区分。
  *   标准做法是 XSHUT 全部拉低 -> 逐颗唤醒 -> 唤醒一颗就立刻改地址 -> 保持唤醒。
  ******************************************************************************
  */
#ifndef __TOF_H
#define __TOF_H

#include <stdint.h>

/* 单颗传感器的运行状态 */
typedef struct
{
    uint8_t  enabled;       /* 配置里是否启用 */
    uint8_t  online;        /* 初始化是否成功 */
    uint8_t  valid;         /* 最近一次读数是否有效 */
    uint8_t  stop_var;      /* 初始化时读到的 stop_variable，单次测距要用 */
    uint8_t  addr;          /* 当前 I2C 地址 */
    int32_t  mm;            /* 最近一次距离（mm），无效时保留上次值 */
    uint16_t err_count;     /* 累计错误次数（诊断用）*/
    uint16_t sample_seq;    /* 新样本序号：每完成一次测量 +1
                             * ★ 上层必须靠它判断"有没有新数据"，
                             *   否则把判定器放在主循环里每轮都喂，
                             *   去抖计数会在微秒级跑满，等于没有去抖。 */
} tof_dev_t;

/**
  * @brief  初始化（阻塞，含 4 颗地址分配 + 完整调参 + 参考校准）
  * @param  enable_mask  bit n = TOF n 启用（0..3）
  * @note   整个过程约 1 秒，内部会喂看门狗。
  *         未启用的传感器不拉 XSHUT、不分配地址、不参与轮询。
  */
void     Tof_Init(uint8_t enable_mask);

/** @brief 初始化成功的位图（bit n = TOF n 在线）*/
uint8_t  Tof_GetOnlineMask(void);

/** @brief 启用位图 */
uint8_t  Tof_GetEnableMask(void);

/** @brief 取某颗的状态（索引越界返回 0）*/
const tof_dev_t *Tof_GetDev(uint8_t idx);

/** @brief 全部读数是否至少有一遍完整轮询过 */
uint8_t  Tof_HasFirstSweep(void);

/**
  * @brief  非阻塞轮询
  * @param  now_ms  BSP_GetTickMs()
  * @note   每次只推进一颗：启动单次测距 -> 后续调用里等结果 -> 有结果后
  *         间隔 poll_ms 再动下一颗。绝不阻塞主循环。
  */
void     Tof_Poll(uint32_t now_ms);

/** @brief 设置两颗之间的间隔（TOF_CFG 命令用），返回实际生效值 */
uint8_t  Tof_SetPollMs(uint8_t ms);
uint8_t  Tof_GetPollMs(void);

/** @brief 是否正在等某颗的结果（用于状态显示）*/
uint8_t  Tof_IsBusy(void);

#endif /* __TOF_H */
