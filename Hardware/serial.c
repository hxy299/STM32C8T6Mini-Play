/**
  ******************************************************************************
  * @file    Hardware/serial.c
  * @brief   双串口：USART1（A7Z）+ USART3（蓝牙）+ RX 中断环形缓冲
  *
  * 设计：
  *   - 接收：RXNE 中断 -> 各自独立的环形缓冲（只搬运，不解析）
  *   - 解析：主循环从环形缓冲取字节，喂给各自的 link_parser_t 实例
  *   - 发送：阻塞发送（应答帧都很小；大帧由上层避免）
  *
  * ★ 两个端口必须是【完全独立的实例】：
  *   共用环形缓冲或共用一个解析器都会导致"接了蓝牙以后 A7Z 也失灵"。
  *   这里用 uart_port_t 表驱动，两条链路各一份状态。
  ******************************************************************************
  */
#include "stm32f10x.h"
#include <stdarg.h>
#include <stdio.h>
#include "app_config.h"
#include "serial.h"

/* ============================ 端口实例 ============================ */
#define PORT_A7Z   0u
#define PORT_BT    1u
#define PORT_COUNT 2u

typedef struct
{
    USART_TypeDef    *usart;
    volatile uint8_t *ring;
    volatile uint16_t head;        /* 中断写 */
    volatile uint16_t tail;        /* 主循环读 */
    volatile uint32_t overflow;
    volatile uint32_t total;
    uint16_t          mask;        /* 环形长度 - 1（长度必须是 2 的幂）*/
    uint8_t           ready;       /* 是否已初始化 */
} uart_port_t;

static volatile uint8_t s_ring_a7z[LINK_RX_RING_SIZE];
static volatile uint8_t s_ring_bt[BT_RX_RING_SIZE];

static uart_port_t s_port[PORT_COUNT] =
{
    { LINK_USART, s_ring_a7z, 0u, 0u, 0u, 0u,
      (uint16_t)(LINK_RX_RING_SIZE - 1u), 0u },
    { BT_USART,   s_ring_bt,  0u, 0u, 0u, 0u,
      (uint16_t)(BT_RX_RING_SIZE - 1u),   0u }
};

/* ============================ 内部：通用实现 ============================ */

static void uart_isr(uart_port_t *p)
{
    if (USART_GetITStatus(p->usart, USART_IT_RXNE) != RESET)
    {
        uint8_t  data = (uint8_t)USART_ReceiveData(p->usart);
        uint16_t next = (uint16_t)((p->head + 1u) & p->mask);

        p->total++;

        if (next == p->tail)
        {
            /* 满了：丢弃新字节并计数（不覆盖旧数据，便于发现对端发太快）*/
            p->overflow++;
        }
        else
        {
            p->ring[p->head] = data;
            p->head = next;
        }

        USART_ClearITPendingBit(p->usart, USART_IT_RXNE);
    }
}

static void uart_send_byte(uart_port_t *p, uint8_t b)
{
    while (USART_GetFlagStatus(p->usart, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(p->usart, b);
}

static void uart_send_buffer(uart_port_t *p, const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (buf == 0)
    {
        return;
    }
    for (i = 0u; i < len; i++)
    {
        uart_send_byte(p, buf[i]);
    }
    /* 等最后一个字节真正发完，避免调用者连续发帧时粘包 */
    while (USART_GetFlagStatus(p->usart, USART_FLAG_TC) == RESET)
    {
    }
}

static uint8_t uart_read_byte(uart_port_t *p, uint8_t *b)
{
    if (b == 0)
    {
        return 0u;
    }
    if (p->tail == p->head)
    {
        return 0u;                     /* 空 */
    }
    *b = p->ring[p->tail];
    p->tail = (uint16_t)((p->tail + 1u) & p->mask);
    return 1u;
}

static uint16_t uart_available(uart_port_t *p)
{
    return (uint16_t)((p->head - p->tail) & p->mask);
}

/**
  * @brief  通用串口硬件初始化
  * @param  p       端口实例
  * @param  tx_port/tx_pin  TX 引脚
  * @param  rx_port/rx_pin  RX 引脚
  * @param  gpio_clk GPIO 时钟
  * @param  usart_clk USART 时钟
  * @param  baud     波特率
  * @param  irqn     中断号
  */
static void uart_hw_init(uart_port_t *p,
                         GPIO_TypeDef *tx_port, uint16_t tx_pin,
                         GPIO_TypeDef *rx_port, uint16_t rx_pin,
                         uint32_t gpio_clk, uint32_t usart_clk,
                         uint32_t baud, uint8_t irqn)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef  nvic;

    RCC_APB2PeriphClockCmd(gpio_clk | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(usart_clk, ENABLE);
    if (usart_clk == RCC_APB2Periph_USART1)
    {
        /* USART1 在 APB2 上，上面那行对它是无效参数，这里补上 */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    }

    /* TX：复用推挽 */
    gpio.GPIO_Pin   = tx_pin;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(tx_port, &gpio);

    /* RX：上拉输入。上拉很重要 —— 模块没插时悬空的 RX 会收到噪声，
     * 解析器会不断重同步，白白占用主循环配额。 */
    gpio.GPIO_Pin  = rx_pin;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(rx_port, &gpio);

    uart.USART_BaudRate            = baud;
    uart.USART_WordLength          = USART_WordLength_8b;
    uart.USART_StopBits            = USART_StopBits_1;
    uart.USART_Parity              = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(p->usart, &uart);

    /* 只开 RXNE 中断，TX 走轮询 */
    USART_ITConfig(p->usart, USART_IT_RXNE, ENABLE);

    /* 串口中断优先级低于调度时基（TIM3=2），本项为 3。
     * NVIC 优先级分组由 BSP_NvicInit() 在 main 开头统一设置。 */
    nvic.NVIC_IRQChannel                   = irqn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3u;
    nvic.NVIC_IRQChannelSubPriority        = 0u;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(p->usart, ENABLE);

    p->head = 0u;
    p->tail = 0u;
    p->ready = 1u;
}

/* ============================ USART1：A7Z ============================ */

void Serial_Init(void)
{
    uart_hw_init(&s_port[PORT_A7Z],
                 GPIOA, GPIO_Pin_9,      /* PA9  = USART1_TX */
                 GPIOA, GPIO_Pin_10,     /* PA10 = USART1_RX */
                 RCC_APB2Periph_GPIOA,
                 RCC_APB2Periph_USART1,
                 LINK_UART_BAUDRATE,
                 (uint8_t)LINK_USART_IRQn);
}

void Serial_SendByte(uint8_t b)
{
    uart_send_byte(&s_port[PORT_A7Z], b);
}

void Serial_SendBuffer(const uint8_t *buf, uint16_t len)
{
    uart_send_buffer(&s_port[PORT_A7Z], buf, len);
}

uint8_t Serial_ReadByte(uint8_t *b)
{
    return uart_read_byte(&s_port[PORT_A7Z], b);
}

uint16_t Serial_Available(void)
{
    return uart_available(&s_port[PORT_A7Z]);
}

uint32_t Serial_GetRxOverflowCount(void)
{
    return s_port[PORT_A7Z].overflow;
}

uint32_t Serial_GetRxTotalCount(void)
{
    return s_port[PORT_A7Z].total;
}

/* ============================ USART3：蓝牙 ============================ */

void SerialBt_Init(void)
{
    uart_hw_init(&s_port[PORT_BT],
                 BT_GPIO_PORT, BT_TX_PIN,    /* PB10 = USART3_TX */
                 BT_GPIO_PORT, BT_RX_PIN,    /* PB11 = USART3_RX */
                 BT_GPIO_CLK,
                 BT_USART_CLK,
                 BT_UART_BAUDRATE,
                 (uint8_t)BT_USART_IRQn);
}

uint8_t SerialBt_IsReady(void)
{
    return s_port[PORT_BT].ready;
}

void SerialBt_SendByte(uint8_t b)
{
    uart_send_byte(&s_port[PORT_BT], b);
}

void SerialBt_SendBuffer(const uint8_t *buf, uint16_t len)
{
    uart_send_buffer(&s_port[PORT_BT], buf, len);
}

uint8_t SerialBt_ReadByte(uint8_t *b)
{
    return uart_read_byte(&s_port[PORT_BT], b);
}

uint16_t SerialBt_Available(void)
{
    return uart_available(&s_port[PORT_BT]);
}

uint32_t SerialBt_GetRxOverflowCount(void)
{
    return s_port[PORT_BT].overflow;
}

uint32_t SerialBt_GetRxTotalCount(void)
{
    return s_port[PORT_BT].total;
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
        uart_send_buffer(&s_port[PORT_A7Z], (const uint8_t *)buf, len);
    }
    return n;
}

/* fputc 重定向，支持 printf() */
int fputc(int ch, FILE *f)
{
    (void)f;
    uart_send_byte(&s_port[PORT_A7Z], (uint8_t)ch);
    return ch;
}

/* ============================ 中断入口 ============================ */
/* 只做搬字节，绝不在这里解析帧（保证中断短） */

void USART1_IRQHandler(void)
{
    uart_isr(&s_port[PORT_A7Z]);
}

void USART3_IRQHandler(void)
{
    uart_isr(&s_port[PORT_BT]);
}
