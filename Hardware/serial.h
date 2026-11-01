/**
  ******************************************************************************
  * @file    Hardware/serial.h
  * @brief   双串口驱动
  *
  *   USART1（PA9=TX / PA10=RX）  与 A7Z 通信，115200
  *   USART3（PB10=TX / PB11=RX） 蓝牙模块，115200（模块需先改波特率）
  *
  * 两者各自独立的环形缓冲与中断，互不影响。
  ******************************************************************************
  */
#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>

/* ============================ USART1（A7Z）============================ */
void     Serial_Init(void);

void     Serial_SendByte(uint8_t b);
void     Serial_SendBuffer(const uint8_t *buf, uint16_t len);
int      Serial_Printf(const char *fmt, ...);   /* 调试用，会阻塞 */

uint8_t  Serial_ReadByte(uint8_t *b);           /* 1 = 取到，0 = 暂无 */
uint16_t Serial_Available(void);

uint32_t Serial_GetRxOverflowCount(void);       /* 环形缓冲溢出次数 */
uint32_t Serial_GetRxTotalCount(void);          /* 累计收到字节数 */

/* ============================ USART3（蓝牙）============================ */
void     SerialBt_Init(void);
uint8_t  SerialBt_IsReady(void);

void     SerialBt_SendByte(uint8_t b);
void     SerialBt_SendBuffer(const uint8_t *buf, uint16_t len);

uint8_t  SerialBt_ReadByte(uint8_t *b);
uint16_t SerialBt_Available(void);

uint32_t SerialBt_GetRxOverflowCount(void);
uint32_t SerialBt_GetRxTotalCount(void);

#endif /* __SERIAL_H */
