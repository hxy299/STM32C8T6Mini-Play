/**
  ******************************************************************************
  * @file    tools/stub/stm32f10x.h
  * @brief   最小 SPL 桩 —— 仅供 PC 上做 -fsyntax-only 用
  *
  * ⚠️ 这不是可用的外设库：没有任何实现，寄存器地址是占位值。
  *    它唯一的用途是让 gcc 能在没有 ARM 工具链、没有 Keil 的机器上
  *    检查"源码语法 + 类型 + 函数声明"是否正确。
  *
  *    能挡掉的错误：拼写、漏分号、未声明函数、结构体字段写错、类型不匹配。
  *    挡不掉的：链接错误、外设真实行为、优化相关的问题。
  *
  *    符号清单是按本工程实际用到的写的，新增外设时在这里补。
  ******************************************************************************
  */
#ifndef __STM32F10X_STUB_H
#define __STM32F10X_STUB_H

#include <stdint.h>

/* ============================ 基础类型 ============================ */
typedef enum { RESET = 0, SET = 1 } FlagStatus, ITStatus;
typedef enum { DISABLE = 0, ENABLE = 1 } FunctionalState;
typedef enum { ERROR = 0, SUCCESS = 1 } ErrorStatus;

/* ============================ 外设结构（不透明）============================ */
typedef struct { volatile uint32_t r; } GPIO_TypeDef;
typedef struct { volatile uint32_t r; } USART_TypeDef;
/* TIM 有寄存器被直接访问（stepper.c 里 m->tim->ARR = ...），
 * 所以这里必须把用到的寄存器做成真实字段。 */
typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
} TIM_TypeDef;
typedef struct { volatile uint32_t r; } I2C_TypeDef;
typedef struct { volatile uint32_t r; } SPI_TypeDef;
typedef struct { volatile uint32_t r; } FLASH_TypeDef;
typedef struct { volatile uint32_t r; } IWDG_TypeDef;

extern uint32_t SystemCoreClock;

/* 基址必须是【编译期常量】：serial.c 用它们做静态初始化 */
#define PERIPH_BASE      ((uint32_t)0x40000000)
#define GPIOA ((GPIO_TypeDef *)(PERIPH_BASE + 0x10800u))
#define GPIOB ((GPIO_TypeDef *)(PERIPH_BASE + 0x10C00u))
#define GPIOC ((GPIO_TypeDef *)(PERIPH_BASE + 0x11000u))
#define GPIOD ((GPIO_TypeDef *)(PERIPH_BASE + 0x11400u))
#define USART1 ((USART_TypeDef *)(PERIPH_BASE + 0x13800u))
#define USART2 ((USART_TypeDef *)(PERIPH_BASE + 0x04400u))
#define USART3 ((USART_TypeDef *)(PERIPH_BASE + 0x04800u))
#define TIM1 ((TIM_TypeDef *)(PERIPH_BASE + 0x12C00u))
#define TIM2 ((TIM_TypeDef *)(PERIPH_BASE + 0x00000u))
#define TIM3 ((TIM_TypeDef *)(PERIPH_BASE + 0x00400u))
#define TIM4 ((TIM_TypeDef *)(PERIPH_BASE + 0x00800u))
#define I2C1 ((I2C_TypeDef *)(PERIPH_BASE + 0x05400u))
#define I2C2 ((I2C_TypeDef *)(PERIPH_BASE + 0x05800u))

/* ============================ GPIO ============================ */
#define GPIO_Pin_0  ((uint16_t)0x0001)
#define GPIO_Pin_1  ((uint16_t)0x0002)
#define GPIO_Pin_2  ((uint16_t)0x0004)
#define GPIO_Pin_3  ((uint16_t)0x0008)
#define GPIO_Pin_4  ((uint16_t)0x0010)
#define GPIO_Pin_5  ((uint16_t)0x0020)
#define GPIO_Pin_6  ((uint16_t)0x0040)
#define GPIO_Pin_7  ((uint16_t)0x0080)
#define GPIO_Pin_8  ((uint16_t)0x0100)
#define GPIO_Pin_9  ((uint16_t)0x0200)
#define GPIO_Pin_10 ((uint16_t)0x0400)
#define GPIO_Pin_11 ((uint16_t)0x0800)
#define GPIO_Pin_12 ((uint16_t)0x1000)
#define GPIO_Pin_13 ((uint16_t)0x2000)
#define GPIO_Pin_14 ((uint16_t)0x4000)
#define GPIO_Pin_15 ((uint16_t)0x8000)

typedef enum
{
    GPIO_Mode_AIN = 0, GPIO_Mode_IN_FLOATING, GPIO_Mode_IPD, GPIO_Mode_IPU,
    GPIO_Mode_Out_OD, GPIO_Mode_Out_PP, GPIO_Mode_AF_OD, GPIO_Mode_AF_PP
} GPIOMode_TypeDef;

typedef enum { GPIO_Speed_10MHz = 1, GPIO_Speed_2MHz, GPIO_Speed_50MHz } GPIOSpeed_TypeDef;

typedef struct
{
    uint16_t          GPIO_Pin;
    GPIOSpeed_TypeDef GPIO_Speed;
    GPIOMode_TypeDef  GPIO_Mode;
} GPIO_InitTypeDef;

#define GPIO_Remap_SWJ_NoJTRST      ((uint32_t)0x00300100)
#define GPIO_Remap_SWJ_JTAGDisable  ((uint32_t)0x00300200)

void    GPIO_Init(GPIO_TypeDef *g, GPIO_InitTypeDef *i);
void    GPIO_SetBits(GPIO_TypeDef *g, uint16_t p);
void    GPIO_ResetBits(GPIO_TypeDef *g, uint16_t p);
void    GPIO_WriteBit(GPIO_TypeDef *g, uint16_t p, uint8_t v);
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *g, uint16_t p);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef *g, uint16_t p);
void    GPIO_PinRemapConfig(uint32_t remap, FunctionalState s);
void    GPIO_DeInit(GPIO_TypeDef *g);

/* ============================ RCC ============================ */
#define RCC_APB2Periph_AFIO   ((uint32_t)0x00000001)
#define RCC_APB2Periph_GPIOA  ((uint32_t)0x00000004)
#define RCC_APB2Periph_GPIOB  ((uint32_t)0x00000008)
#define RCC_APB2Periph_GPIOC  ((uint32_t)0x00000010)
#define RCC_APB2Periph_GPIOD  ((uint32_t)0x00000020)
#define RCC_APB2Periph_TIM1   ((uint32_t)0x00000800)
#define RCC_APB2Periph_USART1 ((uint32_t)0x00004000)
#define RCC_APB1Periph_TIM2   ((uint32_t)0x00000001)
#define RCC_APB1Periph_TIM3   ((uint32_t)0x00000002)
#define RCC_APB1Periph_TIM4   ((uint32_t)0x00000004)
#define RCC_APB1Periph_USART2 ((uint32_t)0x00020000)
#define RCC_APB1Periph_USART3 ((uint32_t)0x00040000)
#define RCC_APB1Periph_I2C1   ((uint32_t)0x00200000)
#define RCC_APB1Periph_I2C2   ((uint32_t)0x00400000)
#define RCC_APB1Periph_PWR    ((uint32_t)0x10000000)

void RCC_APB2PeriphClockCmd(uint32_t p, FunctionalState s);
void RCC_APB1PeriphClockCmd(uint32_t p, FunctionalState s);
void RCC_APB2PeriphResetCmd(uint32_t p, FunctionalState s);

/* ============================ NVIC ============================ */
typedef enum
{
    USART1_IRQn = 37, USART2_IRQn = 38, USART3_IRQn = 39,
    TIM1_UP_IRQn = 25, TIM1_CC_IRQn = 27, TIM2_IRQn = 28,
    TIM3_IRQn = 29, TIM4_IRQn = 30,
    I2C1_EV_IRQn = 31, I2C1_ER_IRQn = 32,
    EXTI0_IRQn = 6, EXTI1_IRQn = 7
} IRQn_Type;

typedef struct
{
    uint8_t         NVIC_IRQChannel;
    uint8_t         NVIC_IRQChannelPreemptionPriority;
    uint8_t         NVIC_IRQChannelSubPriority;
    FunctionalState NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;

#define NVIC_PriorityGroup_2 ((uint32_t)0x500)

void NVIC_Init(NVIC_InitTypeDef *n);
void NVIC_PriorityGroupConfig(uint32_t g);
void NVIC_SystemReset(void);

/* ============================ USART ============================ */
typedef struct
{
    uint32_t USART_BaudRate;
    uint16_t USART_WordLength;
    uint16_t USART_StopBits;
    uint16_t USART_Parity;
    uint16_t USART_Mode;
    uint16_t USART_HardwareFlowControl;
} USART_InitTypeDef;

#define USART_WordLength_8b ((uint16_t)0x0000)
#define USART_StopBits_1    ((uint16_t)0x0000)
#define USART_Parity_No     ((uint16_t)0x0000)
#define USART_Mode_Rx       ((uint16_t)0x0004)
#define USART_Mode_Tx       ((uint16_t)0x0008)
#define USART_HardwareFlowControl_None ((uint16_t)0x0000)
#define USART_IT_TXE   ((uint16_t)0x0727)
#define USART_IT_RXNE  ((uint16_t)0x0525)
#define USART_FLAG_TXE  ((uint16_t)0x0080)
#define USART_FLAG_TC   ((uint16_t)0x0040)
#define USART_FLAG_RXNE ((uint16_t)0x0020)
#define USART_FLAG_ORE  ((uint16_t)0x0208)

void       USART_Init(USART_TypeDef *u, USART_InitTypeDef *i);
void       USART_Cmd(USART_TypeDef *u, FunctionalState s);
void       USART_ITConfig(USART_TypeDef *u, uint16_t it, FunctionalState s);
void       USART_SendData(USART_TypeDef *u, uint16_t d);
uint16_t   USART_ReceiveData(USART_TypeDef *u);
ITStatus   USART_GetITStatus(USART_TypeDef *u, uint16_t it);
void       USART_ClearITPendingBit(USART_TypeDef *u, uint16_t it);
FlagStatus USART_GetFlagStatus(USART_TypeDef *u, uint16_t f);
void       USART_ClearFlag(USART_TypeDef *u, uint16_t f);

/* ============================ TIM ============================ */
typedef struct
{
    uint16_t TIM_Prescaler;
    uint16_t TIM_CounterMode;
    uint32_t TIM_Period;
    uint16_t TIM_ClockDivision;
    uint8_t  TIM_RepetitionCounter;
} TIM_TimeBaseInitTypeDef;

typedef struct
{
    uint16_t TIM_OCMode;
    uint16_t TIM_OutputState;
    uint16_t TIM_OutputNState;
    uint32_t TIM_Pulse;
    uint16_t TIM_OCPolarity;
    uint16_t TIM_OCNPolarity;
    uint16_t TIM_OCIdleState;
    uint16_t TIM_OCNIdleState;
} TIM_OCInitTypeDef;

#define TIM_IT_Update   ((uint16_t)0x0001)
#define TIM_FLAG_Update ((uint16_t)0x0001)
#define TIM_CounterMode_Up ((uint16_t)0x0000)
#define TIM_CKD_DIV1    ((uint16_t)0x0000)
#define TIM_OCMode_PWM1 ((uint16_t)0x0060)
#define TIM_OutputState_Enable ((uint16_t)0x0001)
#define TIM_OCPolarity_High    ((uint16_t)0x0000)
#define TIM_OCPreload_Enable   ((uint16_t)0x0008)
#define TIM_Channel_1   ((uint16_t)0x0000)
#define TIM_Channel_2   ((uint16_t)0x0004)
#define TIM_Channel_3   ((uint16_t)0x0008)
#define TIM_Channel_4   ((uint16_t)0x000C)

void       TIM_TimeBaseInit(TIM_TypeDef *t, TIM_TimeBaseInitTypeDef *i);
void       TIM_OC1Init(TIM_TypeDef *t, TIM_OCInitTypeDef *i);
void       TIM_OC2Init(TIM_TypeDef *t, TIM_OCInitTypeDef *i);
void       TIM_OC3Init(TIM_TypeDef *t, TIM_OCInitTypeDef *i);
void       TIM_OC4Init(TIM_TypeDef *t, TIM_OCInitTypeDef *i);
void       TIM_OCStructInit(TIM_OCInitTypeDef *i);
void       TIM_OC1PreloadConfig(TIM_TypeDef *t, uint16_t preload);
void       TIM_ARRPreloadConfig(TIM_TypeDef *t, FunctionalState s);
void       TIM_Cmd(TIM_TypeDef *t, FunctionalState s);
void       TIM_ITConfig(TIM_TypeDef *t, uint16_t it, FunctionalState s);
ITStatus   TIM_GetITStatus(TIM_TypeDef *t, uint16_t it);
void       TIM_ClearITPendingBit(TIM_TypeDef *t, uint16_t it);
FlagStatus TIM_GetFlagStatus(TIM_TypeDef *t, uint16_t f);
void       TIM_ClearFlag(TIM_TypeDef *t, uint16_t f);
void       TIM_SetCompare1(TIM_TypeDef *t, uint16_t v);
void       TIM_SetCompare2(TIM_TypeDef *t, uint16_t v);
void       TIM_SetCompare3(TIM_TypeDef *t, uint16_t v);
void       TIM_SetCompare4(TIM_TypeDef *t, uint16_t v);
void       TIM_SetAutoreload(TIM_TypeDef *t, uint16_t v);

/* ============================ I2C ============================ */
typedef struct
{
    uint32_t I2C_ClockSpeed;
    uint16_t I2C_Mode;
    uint16_t I2C_DutyCycle;
    uint16_t I2C_OwnAddress1;
    uint16_t I2C_Ack;
    uint16_t I2C_AcknowledgedAddress;
} I2C_InitTypeDef;

#define I2C_Mode_I2C                 ((uint16_t)0x0002)
#define I2C_DutyCycle_2              ((uint16_t)0x0000)
#define I2C_Ack_Enable               ((uint16_t)0x0400)
#define I2C_AcknowledgedAddress_7bit ((uint16_t)0x4000)
#define I2C_Direction_Transmitter    ((uint8_t)0x00)
#define I2C_Direction_Receiver       ((uint8_t)0x01)

#define I2C_FLAG_AF    ((uint32_t)0x00000400)
#define I2C_FLAG_BTF   ((uint32_t)0x00000004)
#define I2C_FLAG_BUSY  ((uint32_t)0x00000002)
#define I2C_FLAG_RXNE  ((uint32_t)0x00000040)
#define I2C_FLAG_TXE   ((uint32_t)0x00000080)

#define I2C_EVENT_MASTER_MODE_SELECT                ((uint32_t)0x00030001)
#define I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED  ((uint32_t)0x00070082)
#define I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED     ((uint32_t)0x00030002)
#define I2C_EVENT_MASTER_BYTE_TRANSMITTING          ((uint32_t)0x00070080)
#define I2C_EVENT_MASTER_BYTE_TRANSMITTED           ((uint32_t)0x00070084)
#define I2C_EVENT_MASTER_BYTE_RECEIVED              ((uint32_t)0x00030040)

void        I2C_DeInit(I2C_TypeDef *i);
void        I2C_Init(I2C_TypeDef *i, I2C_InitTypeDef *cfg);
void        I2C_Cmd(I2C_TypeDef *i, FunctionalState s);
void        I2C_GenerateSTART(I2C_TypeDef *i, FunctionalState s);
void        I2C_GenerateSTOP(I2C_TypeDef *i, FunctionalState s);
void        I2C_AcknowledgeConfig(I2C_TypeDef *i, FunctionalState s);
void        I2C_Send7bitAddress(I2C_TypeDef *i, uint8_t addr, uint8_t dir);
void        I2C_SendData(I2C_TypeDef *i, uint8_t d);
uint8_t     I2C_ReceiveData(I2C_TypeDef *i);
FlagStatus  I2C_GetFlagStatus(I2C_TypeDef *i, uint32_t f);
void        I2C_ClearFlag(I2C_TypeDef *i, uint32_t f);
ErrorStatus I2C_CheckEvent(I2C_TypeDef *i, uint32_t ev);
uint32_t    I2C_GetLastEvent(I2C_TypeDef *i);
void        I2C_SoftwareResetCmd(I2C_TypeDef *i, FunctionalState s);

/* ============================ IWDG ============================ */
#define IWDG_WriteAccess_Enable ((uint16_t)0x5555)
#define IWDG_Prescaler_4        ((uint8_t)0x00)
#define IWDG_Prescaler_8        ((uint8_t)0x01)
#define IWDG_Prescaler_16       ((uint8_t)0x02)
#define IWDG_Prescaler_32       ((uint8_t)0x03)
#define IWDG_Prescaler_64       ((uint8_t)0x04)
#define IWDG_Prescaler_128      ((uint8_t)0x05)
#define IWDG_Prescaler_256      ((uint8_t)0x06)

void IWDG_WriteAccessCmd(uint16_t w);
void IWDG_SetPrescaler(uint8_t p);
void IWDG_SetReload(uint16_t r);
void IWDG_ReloadCounter(void);
void IWDG_Enable(void);

/* ============================ FLASH ============================ */
typedef enum
{
    FLASH_BUSY = 1, FLASH_ERROR_PG, FLASH_ERROR_WRP,
    FLASH_COMPLETE, FLASH_TIMEOUT
} FLASH_Status;

#define FLASH_FLAG_BSY      ((uint32_t)0x00000001)
#define FLASH_FLAG_EOP      ((uint32_t)0x00000020)
#define FLASH_FLAG_PGERR    ((uint32_t)0x00000004)
#define FLASH_FLAG_WRPRTERR ((uint32_t)0x00000010)

void         FLASH_Unlock(void);
void         FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(uint32_t a);
FLASH_Status FLASH_ProgramHalfWord(uint32_t a, uint16_t d);
FLASH_Status FLASH_WaitForLastOperation(uint32_t t);
FLASH_Status FLASH_GetStatus(void);
void         FLASH_ClearFlag(uint32_t f);

/* ============================ 杂项 ============================ */
void __disable_irq(void);
void __enable_irq(void);

#endif /* __STM32F10X_STUB_H */
