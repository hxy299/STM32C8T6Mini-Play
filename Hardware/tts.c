/**
  ******************************************************************************
  * @file    Hardware/tts.c
  * @brief   M9：TW-TTS 语音模块转发（USART2，9600 8N1，中断发送）
  *
  * ★ 为什么要中断发送而不是 while 等 TXE：
  *
  *   9600 8N1 每字节约 1.04 ms。一条 60 字的中文句 = 195 字节，
  *   同步发完要 203 ms。而主循环同时要跑：
  *     · 步进 1 ms 节拍（Stepper_Task1ms）
  *     · 心跳与双链路轮询（LinkSvc_Poll / HeartbeatTask）
  *     · TOF 轮询（TofSvc_Poll）
  *   同步发送 = 这些全部停摆 200 ms，心跳会被判超时，电机节拍顿挫。
  *
  *   本实现里 Tts_Relay 只做一次 memcpy + 下标推进（几微秒），
  *   真正的移位由 USART2 TXE 中断在后台完成。
  *
  * 并发模型（单写者，Cortex-M3 上 16 位访问是原子的）：
  *   s_tx_head  只由主循环写（Tts_Relay）
  *   s_tx_tail  只由中断写（USART2_IRQHandler）
  *   两边只读对方的下标，且读到的"旧值"只会让判断更保守，不会越界。
  ******************************************************************************
  */
#include "stm32f10x.h"
#include <string.h>
#include "app_config.h"
#include "tts.h"
#include "tts_math.h"

/* ============================ 环形缓冲 ============================ */

static uint8_t  s_tx[TTS_TX_RING_SIZE];
static volatile uint16_t s_tx_head = 0u;   /* 主循环写 */
static volatile uint16_t s_tx_tail = 0u;   /* 中断写 */

static uint8_t  s_rx[TTS_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0u;
static volatile uint16_t s_rx_tail = 0u;

static volatile uint8_t  s_ready = 0u;
static volatile uint8_t  s_tx_active = 0u; /* 1 = TXE 中断已开，正在抽环 */

static volatile uint32_t s_frames = 0u;
static volatile uint32_t s_rejected = 0u;

/* 组帧用的静态缓冲。
 * 不放栈上：200 字节对 Keil 默认 1 KB 栈来说太大，
 * 叠在 send_to_src 的 208 字节缓冲之上有溢出风险。*/
static uint8_t s_build[TTS_FRAME_MAX];

/* ============================ 内部 ============================ */

/**
  * @brief  启动或续跑发送：把队首字节塞进 DR，然后开 TXE 中断
  * @note   只在主循环上下文调用（中断里走另一条简短路径）。
  */
static void tts_kick(void)
{
    uint16_t tail;

    if (s_ready == 0u)
    {
        return;
    }

    tail = s_tx_tail;
    if (s_tx_head == tail)
    {
        s_tx_active = 0u;
        USART_ITConfig(TTS_USART, USART_IT_TXE, DISABLE);
        return;
    }

    s_tx_active = 1u;
    USART_SendData(TTS_USART, s_tx[tail]);
    s_tx_tail = tts_ring_advance(tail, TTS_TX_RING_SIZE);
    USART_ITConfig(TTS_USART, USART_IT_TXE, ENABLE);
}

/* ============================ 初始化 ============================ */

void Tts_Init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef  nvic;

    s_tx_head = 0u;
    s_tx_tail = 0u;
    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_tx_active = 0u;
    s_frames = 0u;
    s_rejected = 0u;

    /* 时钟 */
    RCC_APB2PeriphClockCmd(TTS_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(TTS_USART_CLK, ENABLE);

    /* PA2 = TX（复用推挽）*/
    gpio.GPIO_Pin   = TTS_TX_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TTS_GPIO_PORT, &gpio);

    /* PA3 = RX（上拉输入）。
     * 用上拉而不是浮空：模块不回数据时引脚不会漂，
     * 避免浮空引脚收噪声把 ORE 置起来。 */
    gpio.GPIO_Pin  = TTS_RX_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(TTS_GPIO_PORT, &gpio);

    /* USART2：9600 8N1，只开发送中断。
     * RX 中断也开着，但收到的字节只进环不参与协议
     * （TW-TTS 通常不回复，留着是为了以后读模块状态）。*/
    uart.USART_BaudRate            = TTS_UART_BAUDRATE;
    uart.USART_WordLength          = USART_WordLength_8b;
    uart.USART_StopBits            = USART_StopBits_1;
    uart.USART_Parity              = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(TTS_USART, &uart);

    /* 只使能 RXNE；TXE 由 tts_kick 按需打开，
     * 避免"环空但 TXE 一直中断"把 CPU 占满。 */
    USART_ITConfig(TTS_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(TTS_USART, ENABLE);

    /* NVIC：与 serial.c 保持一致（抢占 3 / 子 0）。
     * 分组由 BSP_NvicInit() 在 main 开头统一设置。 */
    nvic.NVIC_IRQChannel                   = TTS_USART_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3u;
    nvic.NVIC_IRQChannelSubPriority        = 0u;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    s_ready = 1u;
}

uint8_t Tts_IsReady(void)
{
    return s_ready;
}

/* ============================ 发送 ============================ */

int Tts_Relay(const uint8_t *frame, uint16_t len)
{
    uint16_t i;
    uint16_t h;

    if (s_ready == 0u)
    {
        return TTS_ERR_NOTREADY;
    }
    if ((frame == 0) || (len == 0u) || (len > TTS_FRAME_MAX))
    {
        return TTS_ERR_TOOLONG;
    }

    /* ★ 整帧原子性：放不下就整帧拒绝，绝不半截入队。
     *   半截入队会让 TW-TTS 收到残缺帧，模块念到一半停住且无法自愈。*/
    if (tts_ring_fits(s_tx_head, s_tx_tail, TTS_TX_RING_SIZE, len) == 0)
    {
        s_rejected++;
        return TTS_ERR_BUSY;
    }

    h = s_tx_head;
    for (i = 0u; i < len; i++)
    {
        s_tx[h] = frame[i];
        h = tts_ring_advance(h, TTS_TX_RING_SIZE);
    }
    s_tx_head = h;

    s_frames++;

    /* 放进去了但中断没在跑（上一帧刚好发完）—— 重新踢一脚。
     * 不踢的话数据会一直躺在环里，直到下一次 Tts_Poll。*/
    if (s_tx_active == 0u)
    {
        tts_kick();
    }

    return TTS_OK;
}

int Tts_SpeakUtf8(const uint8_t *text, uint16_t len)
{
    uint16_t flen;
    uint16_t i;

    if ((text == 0) || (len == 0u) || (len > TTS_TEXT_MAX))
    {
        return TTS_ERR_TOOLONG;
    }

    /* TW-TTS 文本帧：FD | len_hi | len_lo | 01 | 04(UTF-8) | 正文
     * len 字段 = 正文长度 + 2（编码 + 保留字节）*/
    flen = (uint16_t)(len + 2u);
    s_build[0] = 0xFDu;
    s_build[1] = (uint8_t)((flen >> 8) & 0xFFu);
    s_build[2] = (uint8_t)(flen & 0xFFu);
    s_build[3] = 0x01u;
    s_build[4] = 0x04u;
    for (i = 0u; i < len; i++)
    {
        s_build[5u + i] = text[i];
    }

    return Tts_Relay(s_build, (uint16_t)(len + 5u));
}

void Tts_Poll(void)
{
    if (s_ready == 0u)
    {
        return;
    }

    /* 异常兜底：环里有数据，但中断没在跑 —— 拉起来。 */
    if ((s_tx_active == 0u) && (s_tx_head != s_tx_tail))
    {
        tts_kick();
    }
}

/* ============================ 中断 ============================ */

void USART2_IRQHandler(void)
{
    /* ---- 发送：DR 空，继续下一个字节 ---- */
    if (USART_GetITStatus(TTS_USART, USART_IT_TXE) != RESET)
    {
        uint16_t tail = s_tx_tail;

        if (s_tx_head != tail)
        {
            USART_SendData(TTS_USART, s_tx[tail]);
            s_tx_tail = tts_ring_advance(tail, TTS_TX_RING_SIZE);
        }
        else
        {
            /* 环抽干了：关掉 TXE，交回主循环控制 */
            USART_ITConfig(TTS_USART, USART_IT_TXE, DISABLE);
            s_tx_active = 0u;
        }
    }

    /* ---- 接收：只进环，不解析 ---- */
    if (USART_GetITStatus(TTS_USART, USART_IT_RXNE) != RESET)
    {
        uint8_t  b  = (uint8_t)USART_ReceiveData(TTS_USART);
        uint16_t nh = tts_ring_advance(s_rx_head, TTS_RX_RING_SIZE);

        if (nh != s_rx_tail)          /* 满了就丢，模块回复不在关键路径上 */
        {
            s_rx[s_rx_head] = b;
            s_rx_head = nh;
        }
    }

    /* ---- 溢出：读一下 DR 清标志，否则中断会一直进来 ---- */
    if (USART_GetFlagStatus(TTS_USART, USART_FLAG_ORE) != RESET)
    {
        (void)USART_ReceiveData(TTS_USART);
    }
}

/* ============================ 诊断 ============================ */

uint32_t Tts_GetSentFrames(void)
{
    return s_frames;
}

uint32_t Tts_GetRejectedFrames(void)
{
    return s_rejected;
}

uint32_t Tts_GetPendingBytes(void)
{
    return (uint32_t)tts_ring_used(s_tx_head, s_tx_tail, TTS_TX_RING_SIZE);
}

uint8_t Tts_IsBusy(void)
{
    if ((s_tx_active != 0u) || (s_tx_head != s_tx_tail))
    {
        return 1u;
    }
    return 0u;
}
