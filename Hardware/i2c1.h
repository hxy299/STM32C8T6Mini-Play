/**
  ******************************************************************************
  * @file    Hardware/i2c1.h
  * @brief   I2C1 主机驱动（PB6=SCL / PB7=SDA，标准模式 100 kHz）
  *
  * 设计取舍：
  *   - **全阻塞 + 超时**，不做中断/DMA。
  *     4 颗 TOF 共用一条 100 kHz 总线，单次事务最长也就几毫秒，
  *     而且 TOF 轮询本身是"每 10 ms 动一颗"，阻塞用不影响 1 ms 调度。
  *   - 每个等待事件都有超时，**绝不无限等待**（I2C 卡死会让看门狗复位）。
  *   - 提供 I2C1_BusRecover()：STM32F1 的 I2C 外设有个经典缺陷，
  *     从机在传输中被复位会把 SDA 一直拉低，导致总线锁死。
  *     此时必须手工打 9 个 SCL 时钟把它顶出来。
  ******************************************************************************
  */
#ifndef __I2C1_H
#define __I2C1_H

#include <stdint.h>

/* ---------- 返回码 ---------- */
#define I2C1_OK             0
#define I2C1_ERR_ADDR      (-1)     /* 地址无应答（器件不在线）*/
#define I2C1_ERR_BUS       (-2)     /* 总线忙 / 起始或地址阶段超时 */
#define I2C1_ERR_TX        (-3)     /* 发送阶段超时 */
#define I2C1_ERR_RX        (-4)     /* 接收阶段超时 */
#define I2C1_ERR_PARAM     (-5)     /* 参数非法 */

/* ---------- 初始化 ---------- */
void I2C1_Init(void);

/* ---------- 原始传输（addr 是 7 位地址，例如 0x29）---------- */
int  I2C1_Write(uint8_t addr, const uint8_t *buf, uint16_t len);
int  I2C1_Read(uint8_t addr, uint8_t *buf, uint16_t len);

/* ---------- 8 位寄存器索引（VL53L0X 用这个）---------- */
int  I2C1_WriteReg8(uint8_t addr, uint8_t reg, uint8_t val);
int  I2C1_ReadReg8(uint8_t addr, uint8_t reg, uint8_t *val);
int  I2C1_WriteReg8Buf(uint8_t addr, uint8_t reg,
                       const uint8_t *buf, uint16_t len);
int  I2C1_ReadReg8Buf(uint8_t addr, uint8_t reg,
                      uint8_t *buf, uint16_t len);

/* ---------- 16 位寄存器索引（保留通用性，VL53L0X 不用）---------- */
int  I2C1_WriteReg16(uint8_t addr, uint16_t reg, uint8_t val);
int  I2C1_ReadReg16(uint8_t addr, uint16_t reg, uint8_t *val);
int  I2C1_ReadReg16Buf(uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len);
int  I2C1_WriteReg16Buf(uint8_t addr, uint16_t reg,
                        const uint8_t *buf, uint16_t len);

/* ---------- 探测器件是否存在（只发地址，不传数据）---------- */
int  I2C1_Probe(uint8_t addr);

/* ---------- 总线恢复：SDA 被拉低时手工打 9 个 SCL ---------- */
void I2C1_BusRecover(void);

#endif /* __I2C1_H */
