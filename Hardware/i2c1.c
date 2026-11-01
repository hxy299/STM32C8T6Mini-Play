/**
  ******************************************************************************
  * @file    Hardware/i2c1.c
  * @brief   I2C1 主机驱动实现
  ******************************************************************************
  */
#include "stm32f10x.h"
#include "app_config.h"
#include "i2c1.h"
#include "Delay.h"

/* ============================ 内部工具 ============================ */

/**
  * @brief  轮询等待某个 I2C 事件，带超时
  * @return I2C1_OK / I2C1_ERR_BUS
  * @note   超时用简单循环计数。72 MHz 下 60000 次大约 3 ms，
  *         足够覆盖 100 kHz 下最长的一次事务（几字节）。
  */
static int i2c_wait_event(uint32_t event)
{
    uint32_t guard = I2C1_TIMEOUT_LOOPS;

    while (I2C_CheckEvent(I2C1_PERIPH, event) == ERROR)
    {
        if (guard-- == 0u)
        {
            return I2C1_ERR_BUS;
        }
    }
    return I2C1_OK;
}

static int i2c_wait_flag(uint32_t flag)
{
    uint32_t guard = I2C1_TIMEOUT_LOOPS;

    while (I2C_GetFlagStatus(I2C1_PERIPH, flag) == RESET)
    {
        if (guard-- == 0u)
        {
            return I2C1_ERR_BUS;
        }
    }
    return I2C1_OK;
}

/**
  * @brief  等总线空闲；超时则尝试恢复一次再等
  */
static int i2c_wait_idle(void)
{
    uint32_t guard = I2C1_TIMEOUT_LOOPS;

    while (I2C_GetFlagStatus(I2C1_PERIPH, I2C_FLAG_BUSY) != RESET)
    {
        if (guard-- == 0u)
        {
            return I2C1_ERR_BUS;
        }
    }
    return I2C1_OK;
}

/**
  * @brief  收尾：清 AF 并释放总线
  */
static void i2c_abort(void)
{
    I2C_ClearFlag(I2C1_PERIPH, I2C_FLAG_AF);
    I2C_GenerateSTOP(I2C1_PERIPH, ENABLE);
}

/* ============================ 初始化 ============================ */

void I2C1_Init(void)
{
    GPIO_InitTypeDef gpio;
    I2C_InitTypeDef  i2c;

    RCC_APB2PeriphClockCmd(I2C1_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(I2C1_CLK, ENABLE);

    /* PB6/PB7 默认就是 I2C1 的复用开漏，不需要重映射 */
    gpio.GPIO_Pin   = I2C1_SCL_PIN | I2C1_SDA_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C1_GPIO_PORT, &gpio);

    I2C_DeInit(I2C1_PERIPH);

    /* APB1 = 36 MHz，标准模式 100 kHz
     *   CCR   = 36e6 / (2 * 100e3) = 180
     *   TRISE = 36e6 * 300e-9 + 1  = 11.8 -> 标准模式取 (36 + 1) = 37
     */
    i2c.I2C_Mode                = I2C_Mode_I2C;
    i2c.I2C_DutyCycle           = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1         = 0x00u;
    i2c.I2C_Ack                 = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed          = I2C1_SPEED_HZ;
    I2C_Init(I2C1_PERIPH, &i2c);

    I2C_Cmd(I2C1_PERIPH, ENABLE);
}

/* ============================ 写 ============================ */

int I2C1_Write(uint8_t addr, const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if ((buf == 0) || (len == 0u))
    {
        return I2C1_ERR_PARAM;
    }

    if (i2c_wait_idle() != I2C1_OK)
    {
        return I2C1_ERR_BUS;
    }

    I2C_GenerateSTART(I2C1_PERIPH, ENABLE);
    if (i2c_wait_event(I2C_EVENT_MASTER_MODE_SELECT) != I2C1_OK)
    {
        i2c_abort();
        return I2C1_ERR_BUS;
    }

    /* ⚠️ StdPeriph 的 I2C_Send7bitAddress 期望【已左移一位】的地址 */
    I2C_Send7bitAddress(I2C1_PERIPH, (uint8_t)(addr << 1),
                        I2C_Direction_Transmitter);
    if (i2c_wait_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != I2C1_OK)
    {
        i2c_abort();
        return I2C1_ERR_ADDR;      /* 器件没应答：不在线或地址错 */
    }

    for (i = 0u; i < len; i++)
    {
        if (i2c_wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTING) != I2C1_OK)
        {
            i2c_abort();
            return I2C1_ERR_TX;
        }
        I2C_SendData(I2C1_PERIPH, buf[i]);
    }

    /* 最后一个字节发完（BTF）才发 STOP，否则会截断 */
    if (i2c_wait_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != I2C1_OK)
    {
        i2c_abort();
        return I2C1_ERR_TX;
    }
    I2C_GenerateSTOP(I2C1_PERIPH, ENABLE);

    return I2C1_OK;
}

/* ============================ 读 ============================ */

int I2C1_Read(uint8_t addr, uint8_t *buf, uint16_t len)
{
    uint16_t i = 0u;
    uint16_t left = len;

    if ((buf == 0) || (len == 0u))
    {
        return I2C1_ERR_PARAM;
    }

    if (i2c_wait_idle() != I2C1_OK)
    {
        return I2C1_ERR_BUS;
    }

    I2C_AcknowledgeConfig(I2C1_PERIPH, ENABLE);

    I2C_GenerateSTART(I2C1_PERIPH, ENABLE);
    if (i2c_wait_event(I2C_EVENT_MASTER_MODE_SELECT) != I2C1_OK)
    {
        i2c_abort();
        return I2C1_ERR_BUS;
    }

    I2C_Send7bitAddress(I2C1_PERIPH, (uint8_t)(addr << 1),
                        I2C_Direction_Receiver);
    if (i2c_wait_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != I2C1_OK)
    {
        i2c_abort();
        return I2C1_ERR_ADDR;
    }

    while (left > 0u)
    {
        if (left == 1u)
        {
            /* 最后一个字节：先关 ACK 再收，然后 STOP */
            I2C_AcknowledgeConfig(I2C1_PERIPH, DISABLE);
            I2C_GenerateSTOP(I2C1_PERIPH, ENABLE);

            if (i2c_wait_event(I2C_EVENT_MASTER_BYTE_RECEIVED) != I2C1_OK)
            {
                I2C_AcknowledgeConfig(I2C1_PERIPH, ENABLE);
                return I2C1_ERR_RX;
            }
            buf[i++] = I2C_ReceiveData(I2C1_PERIPH);
            left--;
        }
        else if (left == 2u)
        {
            /* 剩两个：关 ACK，等 BTF（两字节都到齐），再 STOP，然后连读两个 */
            I2C_AcknowledgeConfig(I2C1_PERIPH, DISABLE);

            if (i2c_wait_event(I2C_EVENT_MASTER_BYTE_RECEIVED) != I2C1_OK)
            {
                I2C_AcknowledgeConfig(I2C1_PERIPH, ENABLE);
                return I2C1_ERR_RX;
            }
            /* 等 BTF：DR 和移位寄存器都满 */
            {
                uint32_t guard = I2C1_TIMEOUT_LOOPS;
                while (I2C_GetFlagStatus(I2C1_PERIPH, I2C_FLAG_BTF) == RESET)
                {
                    if (guard-- == 0u)
                    {
                        I2C_AcknowledgeConfig(I2C1_PERIPH, ENABLE);
                        i2c_abort();
                        return I2C1_ERR_RX;
                    }
                }
            }
            I2C_GenerateSTOP(I2C1_PERIPH, ENABLE);
            buf[i++] = I2C_ReceiveData(I2C1_PERIPH);
            buf[i++] = I2C_ReceiveData(I2C1_PERIPH);
            left -= 2u;
        }
        else
        {
            if (i2c_wait_event(I2C_EVENT_MASTER_BYTE_RECEIVED) != I2C1_OK)
            {
                I2C_AcknowledgeConfig(I2C1_PERIPH, ENABLE);
                i2c_abort();
                return I2C1_ERR_RX;
            }
            buf[i++] = I2C_ReceiveData(I2C1_PERIPH);
            left--;
        }
    }

    I2C_AcknowledgeConfig(I2C1_PERIPH, ENABLE);
    return I2C1_OK;
}

/* ============================ 寄存器读写 ============================ */

int I2C1_WriteReg8(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t b[2];
    b[0] = reg;
    b[1] = val;
    return I2C1_Write(addr, b, 2u);
}

int I2C1_ReadReg8(uint8_t addr, uint8_t reg, uint8_t *val)
{
    return I2C1_ReadReg8Buf(addr, reg, val, 1u);
}

int I2C1_ReadReg8Buf(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    int r;

    r = I2C1_Write(addr, &reg, 1u);
    if (r != I2C1_OK)
    {
        return r;
    }
    return I2C1_Read(addr, buf, len);
}

int I2C1_WriteReg8Buf(uint8_t addr, uint8_t reg,
                      const uint8_t *buf, uint16_t len)
{
    uint8_t tmp[1u + 8u];
    uint16_t i;

    /* VL53L0X 的 SPAD 表必须一次连续写完，中间不能插新的 START */
    if (len > 8u)
    {
        return I2C1_ERR_PARAM;
    }

    tmp[0] = reg;
    for (i = 0u; i < len; i++)
    {
        tmp[1u + i] = buf[i];
    }
    return I2C1_Write(addr, tmp, (uint16_t)(1u + len));
}

int I2C1_WriteReg16(uint8_t addr, uint16_t reg, uint8_t val)
{
    uint8_t b[3];
    b[0] = (uint8_t)(reg >> 8);
    b[1] = (uint8_t)(reg & 0xFFu);
    b[2] = val;
    return I2C1_Write(addr, b, 3u);
}

int I2C1_ReadReg16(uint8_t addr, uint16_t reg, uint8_t *val)
{
    return I2C1_ReadReg16Buf(addr, reg, val, 1u);
}

int I2C1_ReadReg16Buf(uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t r8[2];
    int r;

    r8[0] = (uint8_t)(reg >> 8);
    r8[1] = (uint8_t)(reg & 0xFFu);

    r = I2C1_Write(addr, r8, 2u);
    if (r != I2C1_OK)
    {
        return r;
    }
    return I2C1_Read(addr, buf, len);
}

int I2C1_WriteReg16Buf(uint8_t addr, uint16_t reg,
                       const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    int r;

    /* VL53L0X 的 SPAD 表要一次连续写 6 字节，中间不能插新的 START，
     * 所以这里拼一个临时缓冲一次发完。 */
    {
        uint8_t tmp[2 + 8];

        if (len > 8u)
        {
            return I2C1_ERR_PARAM;
        }
        tmp[0] = (uint8_t)(reg >> 8);
        tmp[1] = (uint8_t)(reg & 0xFFu);
        for (i = 0u; i < len; i++)
        {
            tmp[2u + i] = buf[i];
        }
        r = I2C1_Write(addr, tmp, (uint16_t)(2u + len));
    }
    return r;
}

/* ============================ 探测 ============================ */

int I2C1_Probe(uint8_t addr)
{
    if (i2c_wait_idle() != I2C1_OK)
    {
        return I2C1_ERR_BUS;
    }

    I2C_GenerateSTART(I2C1_PERIPH, ENABLE);
    if (i2c_wait_event(I2C_EVENT_MASTER_MODE_SELECT) != I2C1_OK)
    {
        i2c_abort();
        return I2C1_ERR_BUS;
    }

    I2C_Send7bitAddress(I2C1_PERIPH, (uint8_t)(addr << 1),
                        I2C_Direction_Transmitter);
    if (i2c_wait_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != I2C1_OK)
    {
        i2c_abort();
        return I2C1_ERR_ADDR;
    }

    I2C_GenerateSTOP(I2C1_PERIPH, ENABLE);
    return I2C1_OK;
}

/* ============================ 总线恢复 ============================ */

/**
  * @brief  从总线锁死中恢复
  * @note   STM32F1 的 I2C 有个经典缺陷：从机在主机传输中途掉电/复位，
  *         会把 SDA 一直拉低，主机此后永远等不到总线空闲。
  *         唯一的办法是**临时把引脚切成普通 GPIO**，
  *         手工打 9 个 SCL 脉冲让从机把剩余的数据位移完，
  *         再补一个 STOP，最后重新初始化 I2C 外设。
  */
void I2C1_BusRecover(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t i;

    /* 1) 关掉 I2C 外设，引脚切回普通开漏输出 */
    I2C_Cmd(I2C1_PERIPH, DISABLE);

    gpio.GPIO_Pin   = I2C1_SCL_PIN | I2C1_SDA_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C1_GPIO_PORT, &gpio);

    /* 2) 两条线都释放为高 */
    GPIO_SetBits(I2C1_GPIO_PORT, I2C1_SCL_PIN | I2C1_SDA_PIN);
    Delay_us(10u);

    /* 3) 9 个 SCL 时钟，把从机移位寄存器顶空 */
    for (i = 0u; i < 9u; i++)
    {
        GPIO_ResetBits(I2C1_GPIO_PORT, I2C1_SCL_PIN);
        Delay_us(10u);
        GPIO_SetBits(I2C1_GPIO_PORT, I2C1_SCL_PIN);
        Delay_us(10u);
    }

    /* 4) 生成 STOP：SCL 高电平期间 SDA 由低变高 */
    GPIO_ResetBits(I2C1_GPIO_PORT, I2C1_SDA_PIN);
    Delay_us(10u);
    GPIO_SetBits(I2C1_GPIO_PORT, I2C1_SCL_PIN);
    Delay_us(10u);
    GPIO_SetBits(I2C1_GPIO_PORT, I2C1_SDA_PIN);
    Delay_us(10u);

    /* 5) 重新初始化 I2C 外设（引脚也会被 I2C1_Init 改回复用开漏）*/
    I2C1_Init();
}
