/**
 ******************************************************************************
 * @file    tmp102.c
 * @brief   TI TMP102 digital temperature sensor over I2C2.
 * @note    See tmp102.h for the register map and data format.
 ******************************************************************************
 */
#include "tmp102.h"
#include <stddef.h>

#define TMP102_TIMEOUT_MS   50u

static I2C_HandleTypeDef *tmp_i2c  = NULL;
static uint16_t           tmp_addr = TMP102_HAL_ADDR(TMP102_ADDR_GND);
static uint8_t            tmp_extended = 0u;   /* mirrors the EM bit */

/* ------------------------------------------------------------------ */
/* Low-level 16-bit register access (TMP102 is big-endian on the wire) */
/* ------------------------------------------------------------------ */
static TMP102_Status tmp_read16(uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];

    if (tmp_i2c == NULL) return TMP102_ERR_IO;

    /* Mem_Read writes the pointer byte, issues a repeated start and
       reads back - exactly the transaction the TMP102 expects.       */
    if (HAL_I2C_Mem_Read(tmp_i2c, tmp_addr, reg, I2C_MEMADD_SIZE_8BIT,
                         buf, 2u, TMP102_TIMEOUT_MS) != HAL_OK) {
        return TMP102_ERR_IO;
    }

    *value = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    return TMP102_OK;
}

static TMP102_Status tmp_write16(uint8_t reg, uint16_t value)
{
    uint8_t buf[2];

    if (tmp_i2c == NULL) return TMP102_ERR_IO;

    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)(value & 0xFFu);

    if (HAL_I2C_Mem_Write(tmp_i2c, tmp_addr, reg, I2C_MEMADD_SIZE_8BIT,
                          buf, 2u, TMP102_TIMEOUT_MS) != HAL_OK) {
        return TMP102_ERR_IO;
    }
    return TMP102_OK;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
void TMP102_Init(I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    tmp_i2c      = hi2c;
    tmp_addr     = TMP102_HAL_ADDR(addr7);
    tmp_extended = 0u;
}

HAL_StatusTypeDef TMP102_IsReady(uint32_t trials)
{
    if (tmp_i2c == NULL) return HAL_ERROR;
    return HAL_I2C_IsDeviceReady(tmp_i2c, tmp_addr, trials, TMP102_TIMEOUT_MS);
}

TMP102_Status TMP102_ReadRaw(int16_t *raw)
{
    uint16_t word;
    TMP102_Status st;

    if (raw == NULL) return TMP102_ERR_PARAM;

    st = tmp_read16(TMP102_REG_TEMP, &word);
    if (st != TMP102_OK) return st;

    /* The reading is LEFT justified in the 16-bit word, so casting to
       a signed type and shifting right sign-extends it for free:
         normal   mode -> 12 bits in [15:4]
         extended mode -> 13 bits in [15:3]                            */
    if (tmp_extended) {
        *raw = (int16_t)((int16_t)word >> 3);
    } else {
        *raw = (int16_t)((int16_t)word >> 4);
    }
    return TMP102_OK;
}

TMP102_Status TMP102_ReadMilliC(int32_t *milli_c)
{
    int16_t raw;
    TMP102_Status st;

    if (milli_c == NULL) return TMP102_ERR_PARAM;

    st = TMP102_ReadRaw(&raw);
    if (st != TMP102_OK) return st;

    /* One LSB is 0.0625 C = 62.5 mC, so mC = raw * 125 / 2.
       Worst-case truncation is 0.5 mC, far below the +/-0.5 C
       accuracy of the part itself.                                    */
    *milli_c = ((int32_t)raw * 125) / 2;
    return TMP102_OK;
}

TMP102_Status TMP102_ReadCelsius(float *celsius)
{
    int16_t raw;
    TMP102_Status st;

    if (celsius == NULL) return TMP102_ERR_PARAM;

    st = TMP102_ReadRaw(&raw);
    if (st != TMP102_OK) return st;

    *celsius = (float)raw * 0.0625f;
    return TMP102_OK;
}

TMP102_Status TMP102_ReadConfig(uint16_t *cfg)
{
    if (cfg == NULL) return TMP102_ERR_PARAM;
    return tmp_read16(TMP102_REG_CONFIG, cfg);
}

TMP102_Status TMP102_WriteConfig(uint16_t cfg)
{
    TMP102_Status st = tmp_write16(TMP102_REG_CONFIG, cfg);
    if (st == TMP102_OK) {
        /* Keep our view of the data format in sync with the hardware. */
        tmp_extended = (cfg & TMP102_CFG_EM) ? 1u : 0u;
    }
    return st;
}

TMP102_Status TMP102_SetRate(TMP102_Rate rate)
{
    uint16_t cfg;
    TMP102_Status st = TMP102_ReadConfig(&cfg);
    if (st != TMP102_OK) return st;

    cfg &= (uint16_t)~(TMP102_CFG_CR1 | TMP102_CFG_CR0);
    cfg |= (uint16_t)(((uint16_t)rate & 0x3u) << 6);

    return TMP102_WriteConfig(cfg);
}

TMP102_Status TMP102_SetExtendedMode(uint8_t enable)
{
    uint16_t cfg;
    TMP102_Status st = TMP102_ReadConfig(&cfg);
    if (st != TMP102_OK) return st;

    if (enable) cfg |=  (uint16_t)TMP102_CFG_EM;
    else        cfg &= (uint16_t)~TMP102_CFG_EM;

    return TMP102_WriteConfig(cfg);
}

/* ------------------------------------------------------------------ */
/* One-shot mode                                                       */
/* ------------------------------------------------------------------ */
TMP102_Status TMP102_Shutdown(void)
{
    uint16_t cfg;
    TMP102_Status st = TMP102_ReadConfig(&cfg);
    if (st != TMP102_OK) return st;

    cfg |= (uint16_t)TMP102_CFG_SD;
    return TMP102_WriteConfig(cfg);
}

TMP102_Status TMP102_StartOneShot(void)
{
    uint16_t cfg;
    TMP102_Status st = TMP102_ReadConfig(&cfg);
    if (st != TMP102_OK) return st;

    /* OS only has an effect while the device is shut down. Writing 1
       kicks off a single conversion; it reads back 0 until done.      */
    cfg |= (uint16_t)(TMP102_CFG_SD | TMP102_CFG_OS);
    return TMP102_WriteConfig(cfg);
}

TMP102_Status TMP102_IsConversionDone(uint8_t *done)
{
    uint16_t cfg;
    TMP102_Status st;

    if (done == NULL) return TMP102_ERR_PARAM;

    st = TMP102_ReadConfig(&cfg);
    if (st != TMP102_OK) return st;

    *done = (cfg & TMP102_CFG_OS) ? 1u : 0u;
    return TMP102_OK;
}
