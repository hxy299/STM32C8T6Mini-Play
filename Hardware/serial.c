/**
  ******************************************************************************
  * @file    Hardware/serial.c
  * @brief   USART1 + RX 中断环形缓冲 + 阻塞发送 + printf 重定向
  ******************************************************************************
  */
#include "stm32f10x.h"
#include <stdarg.h>
#include <stdio.h>
#include "app_config.h"
#include "serial.h"

/* ============================ 环形缓冲 ============================ */
/* 大小必须是 2 的幂，用位与代替取模 */
#define RX_RING_SIZE   LINK_RX_RING_SIZE
#define RX_RING_MASK   (RX_RING_SIZE - 1u)

static volatile uint8_t  s_rx_ring[RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0u;   /* 中断写 */
static volatile uint16_t s_rx_tail = 0u;   /* 主循环读 */
static volatile uint32_t s_rx_overflow = 0u;
static volatile uint32_t s_rx_total = 0u;

/* ============================ 初始化 ============================ */

void Serial_Init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef  nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | LINK_USART_CLK, ENABLE);

    /* PA9 = USART1_TX：复用推挽 */
    gpio.GPIO_Pin   = GPIO_Pin_9;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* PA10 = USART1_RX：上拉输入 */
    gpio.GPIO_Pin  = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

    uart.USART_BaudRate            = LINK_UART_BAUDRATE;
    uart.USART_WordLength          = USART_WordLength_8b;
    uart.USART_StopBits            = USART_StopBits_1;
    uart.USART_Parity              = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(LINK_USART, &uart);

    /* 只开 RXNE 中断，TX 走轮询 */
    USART_ITConfig(LINK_USART, USART_IT_RXNE, ENABLE);

    /* 串口中断优先级低于调度时基（TIM3=2），本项为 3。
     * 注意：NVIC 优先级分组由 BSP_NvicInit() 在 main 开头统一设置。 */
    nvic.NVIC_IRQChannel                   = LINK_USART_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3u;
    nvic.NVIC_IRQChannelSubPriority        = 0u;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(LINK_USART, ENABLE);
}

/* ============================ 发送（阻塞）============================ */

void Serial_SendByte(uint8_t b)
{
    while (USART_GetFlagStatus(LINK_USART, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(LINK_USART, b);
}

void Serial_SendBuffer(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    if (buf == 0)
    {
        return;
    }
    for (i = 0u; i < len; i++)
    {
        Serial_SendByte(buf[i]);
    }
    while (USART_GetFlagStatus(LINK_USART, USART_FLAG_TC) == RESET)
    {
    }
}

/* ============================ 调试 printf ============================ */
/* 注意：用 vsnprintf 限长，避免旧工程的 vsprintf + char[100] 溢出风险 */
int Serial_Printf(const char *fmt, ...)
{
    char    buf[128];
    va_list ap;
    int     n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0)
    {
        uint16_t len = (uint16_t)((n < (int)sizeof(buf)) ? n : (int)sizeof(buf) - 1);
        Serial_SendBuffer((const uint8_t *)buf, len);
    }
    return n;
}

/* fputc 重定向，支持 printf() */
int fputc(int ch, FILE *f)
{
    (void)f;
    Serial_SendByte((uint8_t)ch);
    return ch;
}

/* ============================ 接收 ============================ */

uint8_t Serial_ReadByte(uint8_t *b)
{
    if (b == 0)
    {
        return 0u;
    }
    if (s_rx_tail == s_rx_head)
    {
        return 0u;                     /* 空 */
    }
    *b = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1u) & RX_RING_MASK);
    return 1u;
}

uint16_t Serial_Available(void)
{
    return (uint16_t)((s_rx_head - s_rx_tail) & RX_RING_MASK);
}

uint32_t Serial_GetRxOverflowCount(void)
{
    return s_rx_overflow;
}

uint32_t Serial_GetRxTotalCount(void)
{
    return s_rx_total;
}

/* ============================ USART1 中断 ============================ */
/* 只做搬字节，绝不在这里解析帧（保证中断短） */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(LINK_USART, USART_IT_RXNE) != RESET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(LINK_USART);
        uint16_t next = (uint16_t)((s_rx_head + 1u) & RX_RING_MASK);

        s_rx_total++;

        if (next == s_rx_tail)
        {
            /* 满了：丢弃新字节并计数（不覆盖旧数据，便于发现上位机发太快）*/
            s_rx_overflow++;
        }
        else
        {
            s_rx_ring[s_rx_head] = data;
            s_rx_head = next;
        }

        USART_ClearITPendingBit(LINK_USART, USART_IT_RXNE);
    }
}
