/**
  ******************************************************************************
  * @file    Hardware/tof.c
  * @brief   VL53L0X ×4 驱动实现
  *
  * 初始化序列来自 ST 官方 API（VL53L0X_DataInit / StaticInit /
  * load_tuning_settings / PerformRefCalibration）的移植，
  * 参考实现见 Pololu vl53l0x-arduino 的 VL53L0X::init()。
  *
  * ⚠️ 不要试图"精简"调参表。VL53L0X 的相位校准 / VHV 校准 / SPAD 表
  *    缺任何一段都会导致读数严重偏移或干脆不出数。
  ******************************************************************************
  */
#include "stm32f10x.h"
#include "app_config.h"
#include "i2c1.h"
#include "tof.h"
#include "tof_math.h"
#include "Delay.h"
#include "bsp.h"

/* ============================ VL53L0X 寄存器索引 ============================ */
#define VL_SYSRANGE_START                   0x00u
#define VL_SYSTEM_SEQUENCE_CONFIG           0x01u
#define VL_SYSTEM_INTERRUPT_CONFIG_GPIO     0x0Au
#define VL_SYSTEM_INTERRUPT_CLEAR           0x0Bu
#define VL_RESULT_INTERRUPT_STATUS          0x13u
#define VL_RESULT_RANGE_STATUS              0x14u
#define VL_RANGE_MM_REG                     (VL_RESULT_RANGE_STATUS + 10u) /* 0x1E */
#define VL_MSRC_CONFIG_CONTROL              0x60u
#define VL_MSRC_CONFIG_TIMEOUT_MACROP       0x46u
#define VL_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44u
#define VL_FINAL_RANGE_CONFIG_VCSEL_PERIOD  0x70u
#define VL_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x71u
#define VL_PRE_RANGE_CONFIG_VCSEL_PERIOD    0x50u
#define VL_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x51u
#define VL_GPIO_HV_MUX_ACTIVE_HIGH          0x84u
#define VL_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV 0x89u
#define VL_IDENTIFICATION_MODEL_ID          0xC0u
#define VL_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 0xB0u
#define VL_GLOBAL_CONFIG_REF_EN_START_SELECT 0xB6u
#define VL_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD 0x4Eu
#define VL_DYNAMIC_SPAD_REF_EN_START_OFFSET  0x4Fu
#define VL_I2C_SLAVE_DEVICE_ADDRESS         0x8Au

#define VL_MODEL_ID_VAL                     0xEEu

/* 序列步骤开销（ST 官方常数，单位 us）*/
#define VL_START_OVERHEAD       1910u
#define VL_END_OVERHEAD          960u
#define VL_MSRC_OVERHEAD         660u
#define VL_TCC_OVERHEAD          590u
#define VL_DSS_OVERHEAD          690u
#define VL_PRE_RANGE_OVERHEAD    660u
#define VL_FINAL_RANGE_OVERHEAD  550u

/* ============================ ST 官方调参表 ============================ */
typedef struct
{
    uint8_t reg;
    uint8_t val;
} vl_regval_t;

static const vl_regval_t VL_TUNING[] =
{
    /* DefaultTuningSettings（vl53l0x_tuning.h）*/
    { 0xFFu, 0x01u }, { 0x00u, 0x00u },

    { 0xFFu, 0x00u }, { 0x09u, 0x00u }, { 0x10u, 0x00u }, { 0x11u, 0x00u },

    { 0x24u, 0x01u }, { 0x25u, 0xFFu }, { 0x75u, 0x00u },

    { 0xFFu, 0x01u }, { 0x4Eu, 0x2Cu }, { 0x48u, 0x00u }, { 0x30u, 0x20u },

    { 0xFFu, 0x00u }, { 0x30u, 0x09u }, { 0x54u, 0x00u }, { 0x31u, 0x04u },
    { 0x32u, 0x03u }, { 0x40u, 0x83u }, { 0x46u, 0x25u }, { 0x60u, 0x00u },
    { 0x27u, 0x00u }, { 0x50u, 0x06u }, { 0x51u, 0x00u }, { 0x52u, 0x96u },
    { 0x56u, 0x08u }, { 0x57u, 0x30u }, { 0x61u, 0x00u }, { 0x62u, 0x00u },
    { 0x64u, 0x00u }, { 0x65u, 0x00u }, { 0x66u, 0xA0u },

    { 0xFFu, 0x01u }, { 0x22u, 0x32u }, { 0x47u, 0x14u }, { 0x49u, 0xFFu },
    { 0x4Au, 0x00u },

    { 0xFFu, 0x00u }, { 0x7Au, 0x0Au }, { 0x7Bu, 0x00u }, { 0x78u, 0x21u },

    { 0xFFu, 0x01u }, { 0x23u, 0x34u }, { 0x42u, 0x00u }, { 0x44u, 0xFFu },
    { 0x45u, 0x26u }, { 0x46u, 0x05u }, { 0x40u, 0x40u }, { 0x0Eu, 0x06u },
    { 0x20u, 0x1Au }, { 0x43u, 0x40u },

    { 0xFFu, 0x00u }, { 0x34u, 0x03u }, { 0x35u, 0x44u },

    { 0xFFu, 0x01u }, { 0x31u, 0x04u }, { 0x4Bu, 0x09u }, { 0x4Cu, 0x05u },
    { 0x4Du, 0x04u },

    { 0xFFu, 0x00u }, { 0x44u, 0x00u }, { 0x45u, 0x20u }, { 0x47u, 0x08u },
    { 0x48u, 0x28u }, { 0x67u, 0x00u }, { 0x70u, 0x04u }, { 0x71u, 0x01u },
    { 0x72u, 0xFEu }, { 0x76u, 0x00u }, { 0x77u, 0x00u },

    { 0xFFu, 0x01u }, { 0x0Du, 0x01u },

    { 0xFFu, 0x00u }, { 0x80u, 0x01u }, { 0x01u, 0xF8u },

    { 0xFFu, 0x01u }, { 0x8Eu, 0x01u }, { 0x00u, 0x01u }, { 0xFFu, 0x00u },
    { 0x80u, 0x00u }
};

#define VL_TUNING_LEN   (sizeof(VL_TUNING) / sizeof(VL_TUNING[0]))

/* ============================ 内部状态 ============================ */
static GPIO_TypeDef *const s_xshut_port[TOF_COUNT] = TOF_XSHUT_PORTS;
static const uint16_t      s_xshut_pin[TOF_COUNT]  = TOF_XSHUT_PINS;

static tof_dev_t s_dev[TOF_COUNT];

static uint8_t  s_enable_mask = 0u;
static uint8_t  s_online_mask  = 0u;
static uint8_t  s_online_count = 0u;
static uint8_t  s_poll_ms     = TOF_POLL_DEFAULT_MS;

/* 轮询状态机 */
#define TOF_ST_IDLE   0u
#define TOF_ST_WAIT   1u

static uint8_t  s_state      = TOF_ST_IDLE;
static uint8_t  s_cur        = 0u;    /* 当前正在测的索引 */
static uint32_t s_start_ms   = 0u;
static uint32_t s_next_ms    = 0u;    /* 下一次允许启动的时刻 */
static uint8_t  s_first_sweep = 0u;
static uint8_t  s_sweep_done  = 0u;   /* 本轮已测完几颗 */

/* I2C 失败标志：初始化期间累计，最后统一判 */
static uint8_t  s_i2c_fail = 0u;

/* ============================ 低层寄存器访问 ============================ */

static void vl_write(uint8_t addr, uint8_t reg, uint8_t val)
{
    if (I2C1_WriteReg8(addr, reg, val) != I2C1_OK)
    {
        s_i2c_fail = 1u;
    }
}

static uint8_t vl_read(uint8_t addr, uint8_t reg)
{
    uint8_t v = 0u;

    if (I2C1_ReadReg8(addr, reg, &v) != I2C1_OK)
    {
        s_i2c_fail = 1u;
        return 0u;
    }
    return v;
}

/* 16 位数据读（寄存器索引仍是 8 位，数据大端）*/
static uint16_t vl_read16w(uint8_t addr, uint8_t reg)
{
    uint8_t b[2] = { 0u, 0u };

    if (I2C1_ReadReg8Buf(addr, reg, b, 2u) != I2C1_OK)
    {
        s_i2c_fail = 1u;
        return 0u;
    }
    return (uint16_t)(((uint16_t)b[0] << 8) | (uint16_t)b[1]);
}

static void vl_write16w(uint8_t addr, uint8_t reg, uint16_t v)
{
    uint8_t b[2];

    b[0] = (uint8_t)(v >> 8);
    b[1] = (uint8_t)(v & 0xFFu);
    if (I2C1_WriteReg8Buf(addr, reg, b, 2u) != I2C1_OK)
    {
        s_i2c_fail = 1u;
    }
}

/* ============================ 超时换算 ============================ */

/* 宏周期（ns）：2304 * vcsel_period_pclks * 1655 / 1000 */
static uint32_t vl_calc_macro_period_ns(uint8_t pclks)
{
    return ((2304ul * (uint32_t)pclks * 1655ul) + 500ul) / 1000ul;
}

static uint32_t vl_mclks_to_us(uint16_t mclks, uint8_t pclks)
{
    uint32_t ns = vl_calc_macro_period_ns(pclks);
    return (((uint32_t)mclks * ns) + 500ul) / 1000ul;
}

static uint32_t vl_us_to_mclks(uint32_t us, uint8_t pclks)
{
    uint32_t ns = vl_calc_macro_period_ns(pclks);
    return (((us * 1000ul) + (ns / 2ul)) / ns);
}

/* 寄存器编码格式："(LSByte * 2^MSByte) + 1" */
static uint16_t vl_decode_timeout(uint16_t reg_val)
{
    return (uint16_t)(((uint16_t)(reg_val & 0x00FFu)
                       << (uint16_t)((reg_val & 0xFF00u) >> 8)) + 1u);
}

static uint16_t vl_encode_timeout(uint32_t mclks)
{
    uint32_t ls = 0u;
    uint16_t ms = 0u;

    if (mclks > 0u)
    {
        ls = mclks - 1u;
        while ((ls & 0xFFFFFF00ul) > 0ul)
        {
            ls >>= 1;
            ms++;
        }
        return (uint16_t)((ms << 8) | (uint16_t)(ls & 0xFFu));
    }
    return 0u;
}

static uint8_t vl_decode_vcsel(uint8_t reg_val)
{
    return (uint8_t)((reg_val + 1u) << 1);
}

/* ============================ 序列步骤 ============================ */
typedef struct
{
    uint8_t tcc;
    uint8_t dss;
    uint8_t msrc;
    uint8_t pre_range;
    uint8_t final_range;
} vl_enables_t;

typedef struct
{
    uint8_t  pre_range_pclks;
    uint16_t msrc_dss_tcc_mclks;
    uint32_t msrc_dss_tcc_us;
    uint16_t pre_range_mclks;
    uint32_t pre_range_us;
    uint8_t  final_range_pclks;
    uint16_t final_range_mclks;
    uint32_t final_range_us;
} vl_timeouts_t;

static void vl_get_enables(uint8_t addr, vl_enables_t *e)
{
    uint8_t sc = vl_read(addr, VL_SYSTEM_SEQUENCE_CONFIG);

    e->tcc         = (uint8_t)((sc >> 4) & 0x01u);
    e->dss         = (uint8_t)((sc >> 3) & 0x01u);
    e->msrc        = (uint8_t)((sc >> 2) & 0x01u);
    e->pre_range   = (uint8_t)((sc >> 6) & 0x01u);
    e->final_range = (uint8_t)((sc >> 7) & 0x01u);
}

static void vl_get_timeouts(uint8_t addr, const vl_enables_t *e,
                            vl_timeouts_t *t)
{
    t->pre_range_pclks = vl_decode_vcsel(
        vl_read(addr, VL_PRE_RANGE_CONFIG_VCSEL_PERIOD));

    t->msrc_dss_tcc_mclks = (uint16_t)(vl_read(addr, VL_MSRC_CONFIG_TIMEOUT_MACROP) + 1u);
    t->msrc_dss_tcc_us = vl_mclks_to_us(t->msrc_dss_tcc_mclks, t->pre_range_pclks);

    t->pre_range_mclks = vl_decode_timeout(
        vl_read16w(addr, VL_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    t->pre_range_us = vl_mclks_to_us(t->pre_range_mclks, t->pre_range_pclks);

    t->final_range_pclks = vl_decode_vcsel(
        vl_read(addr, VL_FINAL_RANGE_CONFIG_VCSEL_PERIOD));

    t->final_range_mclks = vl_decode_timeout(
        vl_read16w(addr, VL_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (e->pre_range != 0u)
    {
        t->final_range_mclks = (uint16_t)(t->final_range_mclks - t->pre_range_mclks);
    }

    t->final_range_us = vl_mclks_to_us(t->final_range_mclks, t->final_range_pclks);
}

/**
  * @brief  按目标总时长重算并写入 final range 的 timeout
  * @return 1 = 成功，0 = 目标时长比固定开销还小（物理上做不到）
  */
static uint8_t vl_set_timing_budget(uint8_t addr, uint32_t budget_us)
{
    vl_enables_t  e;
    vl_timeouts_t t;
    uint32_t used;

    vl_get_enables(addr, &e);
    vl_get_timeouts(addr, &e, &t);

    used = VL_START_OVERHEAD + VL_END_OVERHEAD;

    if (e.tcc != 0u)
    {
        used += (t.msrc_dss_tcc_us + VL_TCC_OVERHEAD);
    }
    if (e.dss != 0u)
    {
        used += (2ul * (t.msrc_dss_tcc_us + VL_DSS_OVERHEAD));
    }
    else if (e.msrc != 0u)
    {
        used += (t.msrc_dss_tcc_us + VL_MSRC_OVERHEAD);
    }
    if (e.pre_range != 0u)
    {
        used += (t.pre_range_us + VL_PRE_RANGE_OVERHEAD);
    }
    if (e.final_range != 0u)
    {
        uint32_t final_us;
        uint32_t final_mclks;

        used += VL_FINAL_RANGE_OVERHEAD;

        if (used > budget_us)
        {
            return 0u;              /* 目标太短 */
        }

        final_us = budget_us - used;
        final_mclks = vl_us_to_mclks(final_us, t.final_range_pclks);

        if (e.pre_range != 0u)
        {
            final_mclks += t.pre_range_mclks;
        }

        vl_write16w(addr, VL_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                    vl_encode_timeout(final_mclks));
    }
    return 1u;
}

/* ============================ SPAD 信息 ============================ */

static uint8_t vl_get_spad_info(uint8_t addr, uint8_t *count,
                                uint8_t *is_aperture)
{
    uint8_t tmp;
    uint32_t t0;

    vl_write(addr, 0x80u, 0x01u);
    vl_write(addr, 0xFFu, 0x01u);
    vl_write(addr, 0x00u, 0x00u);

    vl_write(addr, 0xFFu, 0x06u);
    vl_write(addr, 0x83u, (uint8_t)(vl_read(addr, 0x83u) | 0x04u));
    vl_write(addr, 0xFFu, 0x07u);
    vl_write(addr, 0x81u, 0x01u);

    vl_write(addr, 0x80u, 0x01u);

    vl_write(addr, 0x94u, 0x6Bu);
    vl_write(addr, 0x83u, 0x00u);

    t0 = BSP_GetTickMs();
    while (vl_read(addr, 0x83u) == 0x00u)
    {
        BSP_IwdgFeed();
        if ((uint32_t)(BSP_GetTickMs() - t0) > 100u)
        {
            return 0u;
        }
    }
    vl_write(addr, 0x83u, 0x01u);
    tmp = vl_read(addr, 0x92u);

    *count       = (uint8_t)(tmp & 0x7Fu);
    *is_aperture = (uint8_t)((tmp >> 7) & 0x01u);

    vl_write(addr, 0x81u, 0x00u);
    vl_write(addr, 0xFFu, 0x06u);
    vl_write(addr, 0x83u, (uint8_t)(vl_read(addr, 0x83u) & (uint8_t)~0x04u));
    vl_write(addr, 0xFFu, 0x01u);
    vl_write(addr, 0x00u, 0x01u);

    vl_write(addr, 0xFFu, 0x00u);
    vl_write(addr, 0x80u, 0x00u);

    return 1u;
}

/* ============================ 参考校准 ============================ */

static uint8_t vl_ref_calibration(uint8_t addr, uint8_t vhv_init_byte)
{
    uint32_t t0;

    vl_write(addr, VL_SYSRANGE_START, (uint8_t)(0x01u | vhv_init_byte));

    t0 = BSP_GetTickMs();
    while ((vl_read(addr, VL_RESULT_INTERRUPT_STATUS) & 0x07u) == 0u)
    {
        BSP_IwdgFeed();
        if ((uint32_t)(BSP_GetTickMs() - t0) > 300u)
        {
            return 0u;
        }
    }

    vl_write(addr, VL_SYSTEM_INTERRUPT_CLEAR, 0x01u);
    vl_write(addr, VL_SYSRANGE_START, 0x00u);
    return 1u;
}

/* ============================ 单颗完整初始化 ============================ */

/**
  * @brief  VL53L0X 完整初始化（DataInit + StaticInit + RefCalibration）
  * @param  stop_var_out 输出 stop_variable，单次测距时必须回写
  * @return 1 = 成功
  */
static uint8_t vl_sensor_init(uint8_t addr, uint8_t *stop_var_out)
{
    uint8_t  spad_count = 0u;
    uint8_t  spad_aperture = 0u;
    uint8_t  ref_spad_map[6];
    uint8_t  first_spad;
    uint8_t  spads_enabled;
    uint8_t  stop_var;
    uint8_t  i;
    uint16_t budget;

    s_i2c_fail = 0u;

    /* ---- 1) 型号确认 ---- */
    if (vl_read(addr, VL_IDENTIFICATION_MODEL_ID) != VL_MODEL_ID_VAL)
    {
        return 0u;
    }

    /* ---- 2) DataInit ---- */
    /* 默认是 1V8 模式，模块板上一般有电平转换；这里按 2V8 配，
     * 因为我们的 MCU 是 3.3V 逻辑。若模块自带 LDO+电平转换也无影响。 */
    vl_write(addr, VL_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
             (uint8_t)(vl_read(addr, VL_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01u));

    /* "Set I2C standard mode" */
    vl_write(addr, 0x88u, 0x00u);

    vl_write(addr, 0x80u, 0x01u);
    vl_write(addr, 0xFFu, 0x01u);
    vl_write(addr, 0x00u, 0x00u);
    stop_var = vl_read(addr, 0x91u);
    vl_write(addr, 0x00u, 0x01u);
    vl_write(addr, 0xFFu, 0x00u);
    vl_write(addr, 0x80u, 0x00u);

    /* 关掉 SIGNAL_RATE_MSRC / PRE_RANGE 的限值检查 */
    vl_write(addr, VL_MSRC_CONFIG_CONTROL,
             (uint8_t)(vl_read(addr, VL_MSRC_CONFIG_CONTROL) | 0x12u));

    /* 信号率限值 0.25 MCPS：Q9.7 定点 = 0.25 * 128 = 32 */
    vl_write16w(addr, VL_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32u);

    vl_write(addr, VL_SYSTEM_SEQUENCE_CONFIG, 0xFFu);

    /* ---- 3) StaticInit：SPAD ---- */
    if (vl_get_spad_info(addr, &spad_count, &spad_aperture) == 0u)
    {
        return 0u;
    }

    if (I2C1_ReadReg8Buf(addr, VL_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                         ref_spad_map, 6u) != I2C1_OK)
    {
        return 0u;
    }

    vl_write(addr, 0xFFu, 0x01u);
    vl_write(addr, VL_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00u);
    vl_write(addr, VL_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2Cu);
    vl_write(addr, 0xFFu, 0x00u);
    vl_write(addr, VL_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4u);

    /* 12 号是第一个 aperture SPAD */
    first_spad = (spad_aperture != 0u) ? 12u : 0u;
    spads_enabled = 0u;

    for (i = 0u; i < 48u; i++)
    {
        uint8_t bit_hi = (uint8_t)(i / 8u);
        uint8_t bit_lo = (uint8_t)(i % 8u);

        if ((i < first_spad) || (spads_enabled == spad_count))
        {
            ref_spad_map[bit_hi] &= (uint8_t)~(1u << bit_lo);
        }
        else if (((ref_spad_map[bit_hi] >> bit_lo) & 0x01u) != 0u)
        {
            spads_enabled++;
        }
    }

    if (I2C1_WriteReg8Buf(addr, VL_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                          ref_spad_map, 6u) != I2C1_OK)
    {
        return 0u;
    }

    /* ---- 4) 官方调参表 ---- */
    for (i = 0u; i < (uint8_t)VL_TUNING_LEN; i++)
    {
        vl_write(addr, VL_TUNING[i].reg, VL_TUNING[i].val);
        if ((i & 0x0Fu) == 0u)
        {
            BSP_IwdgFeed();
        }
    }

    /* ---- 5) GPIO 中断配置：新样本就绪 ---- */
    vl_write(addr, VL_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04u);
    vl_write(addr, VL_GPIO_HV_MUX_ACTIVE_HIGH,
             (uint8_t)(vl_read(addr, VL_GPIO_HV_MUX_ACTIVE_HIGH) & (uint8_t)~0x10u));
    vl_write(addr, VL_SYSTEM_INTERRUPT_CLEAR, 0x01u);

    /* ---- 6) 关掉 MSRC / TCC，重算时序预算 ---- */
    vl_write(addr, VL_SYSTEM_SEQUENCE_CONFIG, 0xE8u);

    budget = (uint16_t)TOF_TIMING_BUDGET_US;
    if (vl_set_timing_budget(addr, budget) == 0u)
    {
        /* 20 ms 都做不到就不要这颗了；理论上不会发生 */
        return 0u;
    }

    /* ---- 7) 参考校准（VHV + 相位）---- */
    vl_write(addr, VL_SYSTEM_SEQUENCE_CONFIG, 0x01u);
    if (vl_ref_calibration(addr, 0x40u) == 0u)
    {
        return 0u;
    }

    vl_write(addr, VL_SYSTEM_SEQUENCE_CONFIG, 0x02u);
    if (vl_ref_calibration(addr, 0x00u) == 0u)
    {
        return 0u;
    }

    /* "restore the previous Sequence Config" */
    vl_write(addr, VL_SYSTEM_SEQUENCE_CONFIG, 0xE8u);

    if (s_i2c_fail != 0u)
    {
        return 0u;
    }

    *stop_var_out = stop_var;
    return 1u;
}

/* ============================ 单次测距 ============================ */

static uint8_t vl_start_single(uint8_t addr, uint8_t stop_var)
{
    s_i2c_fail = 0u;

    vl_write(addr, 0x80u, 0x01u);
    vl_write(addr, 0xFFu, 0x01u);
    vl_write(addr, 0x00u, 0x00u);
    vl_write(addr, 0x91u, stop_var);
    vl_write(addr, 0x00u, 0x01u);
    vl_write(addr, 0xFFu, 0x00u);
    vl_write(addr, 0x80u, 0x00u);

    vl_write(addr, VL_SYSRANGE_START, 0x01u);
    return (s_i2c_fail == 0u) ? 1u : 0u;
}

/* ============================ 初始化：地址分配 ============================ */

void Tof_Init(uint8_t enable_mask)
{
    GPIO_InitTypeDef g;
    uint8_t i;

    s_enable_mask = (uint8_t)(enable_mask & 0x0Fu);
    s_online_mask  = 0u;
    s_online_count = 0u;
    s_state       = TOF_ST_IDLE;
    s_cur         = 0u;
    s_first_sweep = 0u;
    s_sweep_done  = 0u;

    for (i = 0u; i < TOF_COUNT; i++)
    {
        s_dev[i].enabled   = (uint8_t)(((s_enable_mask >> i) & 0x01u));
        s_dev[i].online    = 0u;
        s_dev[i].valid     = 0u;
        s_dev[i].stop_var  = 0u;
        s_dev[i].addr      = (uint8_t)(TOF_ADDR_BASE + i);
        s_dev[i].mm        = 0;
        s_dev[i].err_count = 0u;
        s_dev[i].sample_seq = 0u;
    }

    if (s_enable_mask == 0u)
    {
        return;     /* 一颗都不启用：什么都不做 */
    }

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    /* 1) XSHUT 全部拉低 = 全部关断。
     *    必须先设成推挽输出并输出低，再等电源稳定，
     *    否则 4 颗同时上电会全部抢 0x29。 */
    for (i = 0u; i < TOF_COUNT; i++)
    {
        GPIO_ResetBits(s_xshut_port[i], s_xshut_pin[i]);

        g.GPIO_Pin   = s_xshut_pin[i];
        g.GPIO_Mode  = GPIO_Mode_Out_PP;
        g.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(s_xshut_port[i], &g);
    }
    Delay_ms(10u);
    BSP_IwdgFeed();

    I2C1_Init();

    /* 2) 逐颗唤醒 -> 改地址 -> 完整初始化 -> 保持唤醒 */
    for (i = 0u; i < TOF_COUNT; i++)
    {
        uint8_t new_addr;

        if (s_dev[i].enabled == 0u)
        {
            continue;       /* 未启用：保持 XSHUT 低，绝不参与地址分配 */
        }

        new_addr = (uint8_t)(TOF_ADDR_BASE + i);

        GPIO_SetBits(s_xshut_port[i], s_xshut_pin[i]);
        Delay_ms(3u);       /* 等内部 boot（数据手册约 1.2 ms，留余量）*/
        BSP_IwdgFeed();

        /* 此时总线上只有这一颗醒着，它一定在 0x29 */
        if (I2C1_Probe(TOF_ADDR_DEFAULT) != I2C1_OK)
        {
            /* 在线上探不到：可能是接线/供电问题。
             * 先尝试恢复总线，再探一次。 */
            I2C1_BusRecover();
            Delay_ms(2u);
            if (I2C1_Probe(TOF_ADDR_DEFAULT) != I2C1_OK)
            {
                s_dev[i].online = 0u;
                GPIO_ResetBits(s_xshut_port[i], s_xshut_pin[i]);
                continue;
            }
        }

        /* 改地址（I2C_SLAVE_DEVICE_ADDRESS = 0x8A，写 7 位地址）*/
        if (I2C1_WriteReg8(TOF_ADDR_DEFAULT, VL_I2C_SLAVE_DEVICE_ADDRESS,
                           new_addr) != I2C1_OK)
        {
            s_dev[i].online = 0u;
            GPIO_ResetBits(s_xshut_port[i], s_xshut_pin[i]);
            continue;
        }
        Delay_ms(1u);

        /* 确认新地址有应答 */
        if (I2C1_Probe(new_addr) != I2C1_OK)
        {
            s_dev[i].online = 0u;
            GPIO_ResetBits(s_xshut_port[i], s_xshut_pin[i]);
            continue;
        }

        /* 在新地址上做完整初始化 */
        if (vl_sensor_init(new_addr, &s_dev[i].stop_var) == 0u)
        {
            s_dev[i].online = 0u;
            GPIO_ResetBits(s_xshut_port[i], s_xshut_pin[i]);
            continue;
        }

        s_dev[i].addr   = new_addr;
        s_dev[i].online = 1u;
        s_online_mask   = (uint8_t)(s_online_mask | (1u << i));
        s_online_count++;

        BSP_IwdgFeed();
    }

    s_next_ms = BSP_GetTickMs();
}

/* ============================ 非阻塞轮询 ============================ */

static uint8_t tof_next_enabled(uint8_t from)
{
    uint8_t k;

    for (k = 1u; k <= TOF_COUNT; k++)
    {
        uint8_t idx = (uint8_t)((from + k) % TOF_COUNT);

        if ((s_online_mask >> idx) & 0x01u)
        {
            return idx;
        }
    }
    return 0xFFu;
}

void Tof_Poll(uint32_t now_ms)
{
    if (s_online_mask == 0u)
    {
        return;
    }

    if (s_state == TOF_ST_IDLE)
    {
        uint8_t idx;

        /* 还没到下一次启动时刻 */
        if ((int32_t)(now_ms - s_next_ms) < 0)
        {
            return;
        }

        idx = tof_next_enabled(s_cur);
        if (idx == 0xFFu)
        {
            return;
        }

        s_cur = idx;

        if (vl_start_single(s_dev[idx].addr, s_dev[idx].stop_var) == 0u)
        {
            s_dev[idx].err_count++;
            s_dev[idx].valid = 0u;
            s_next_ms = now_ms + 50u;
            return;
        }

        s_start_ms = now_ms;
        s_state    = TOF_ST_WAIT;
    }
    else
    {
        uint8_t  idx = s_cur;
        uint8_t  started;
        int32_t  mm;

        /* 1) 启动位还在 -> 继续等 */
        started = vl_read(s_dev[idx].addr, VL_SYSRANGE_START);
        if ((started & 0x01u) != 0u)
        {
            if ((uint32_t)(now_ms - s_start_ms) > TOF_MEASURE_TIMEOUT_MS)
            {
                s_dev[idx].valid = 0u;
                s_dev[idx].err_count++;
                s_state   = TOF_ST_IDLE;
                s_next_ms = now_ms + s_poll_ms;
            }
            return;
        }

        /* 2) 结果就绪？ */
        if ((vl_read(s_dev[idx].addr, VL_RESULT_INTERRUPT_STATUS) & 0x07u) == 0u)
        {
            if ((uint32_t)(now_ms - s_start_ms) > TOF_MEASURE_TIMEOUT_MS)
            {
                s_dev[idx].valid = 0u;
                s_dev[idx].err_count++;
                s_state   = TOF_ST_IDLE;
                s_next_ms = now_ms + s_poll_ms;
            }
            return;
        }

        /* 3) 读距离（0x1E，大端 16 位）*/
        {
            uint8_t b[2] = { 0u, 0u };

            if (I2C1_ReadReg8Buf(s_dev[idx].addr, VL_RANGE_MM_REG, b, 2u) != I2C1_OK)
            {
                s_dev[idx].valid = 0u;
                s_dev[idx].err_count++;
                s_dev[idx].sample_seq++;   /* 读失败也算一次"有结论"的采样 */
            }
            else
            {
                mm = (int32_t)(((uint16_t)b[0] << 8) | (uint16_t)b[1]);

                s_dev[idx].mm    = mm;
                s_dev[idx].valid = tof_math_range_ok(mm, TOF_RANGE_MAX_MM);

                /* 每出一颗新结果就 +1，上层据此只喂一次判定器 */
                s_dev[idx].sample_seq++;
            }
        }

        /* 4) 清中断，允许下一次 */
        vl_write(s_dev[idx].addr, VL_SYSTEM_INTERRUPT_CLEAR, 0x01u);
        vl_write(s_dev[idx].addr, VL_SYSRANGE_START, 0x00u);

        s_state   = TOF_ST_IDLE;
        s_next_ms = now_ms + s_poll_ms;

        /* 轮询进度：全部在线传感器都测过一遍就算完成首次扫描 */
        s_sweep_done++;
        if (s_sweep_done >= s_online_count)
        {
            s_first_sweep = 1u;
            s_sweep_done  = 0u;
        }
    }
}

/* ============================ 访问器 ============================ */

uint8_t Tof_GetOnlineMask(void)
{
    return s_online_mask;
}

uint8_t Tof_GetEnableMask(void)
{
    return s_enable_mask;
}

const tof_dev_t *Tof_GetDev(uint8_t idx)
{
    if (idx >= TOF_COUNT)
    {
        return 0;
    }
    return &s_dev[idx];
}

uint8_t Tof_HasFirstSweep(void)
{
    return s_first_sweep;
}

uint8_t Tof_SetPollMs(uint8_t ms)
{
    /* 下限要夹；上限 255 就是 uint8_t 的天然上界，不需要再判
     * （否则编译器会报 comparison is always false）。 */
    if (ms < TOF_POLL_MIN_MS)
    {
        ms = TOF_POLL_MIN_MS;
    }
    s_poll_ms = ms;
    return s_poll_ms;
}

uint8_t Tof_GetPollMs(void)
{
    return s_poll_ms;
}

uint8_t Tof_IsBusy(void)
{
    return (uint8_t)((s_state == TOF_ST_WAIT) ? 1u : 0u);
}
