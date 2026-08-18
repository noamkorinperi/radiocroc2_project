/**
 ******************************************************************************
 * @file    tmp102.h
 * @brief   TI TMP102 digital temperature sensor over I2C2 (PF0/PF1).
 *
 * Unlike the RADIOROC2, the TMP102 is a completely standard I2C device:
 * a pointer register selects one of four 16-bit registers, so the HAL's
 * memory-access helpers can be used directly.
 *
 *   Pointer  Register
 *   0x00     Temperature (read only)
 *   0x01     Configuration
 *   0x02     T_LOW
 *   0x03     T_HIGH
 *
 * Temperature format: 12 bits by default, LEFT justified inside the
 * 16-bit word (bits 15:4), two's complement, 0.0625 C per LSB.
 * In extended mode (EM = 1) it becomes 13 bits in bits 15:3, same LSB
 * weight, extending the top of the range to +150 C.
 *
 * Address selection (7-bit): ADD0 to GND = 0x48 (this board),
 * V+ = 0x49, SDA = 0x4A, SCL = 0x4B.
 *
 * The sensor runs directly at 3.3 V on its own bus, so unlike the
 * RADIOROC2 side there is no level shifter in the path.
 ******************************************************************************
 */
#ifndef TMP102_H
#define TMP102_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Addressing                                                          */
/* ------------------------------------------------------------------ */
#define TMP102_ADDR_GND        0x48u   /* ADD0 -> GND (default)  */
#define TMP102_ADDR_VPLUS      0x49u
#define TMP102_ADDR_SDA        0x4Au
#define TMP102_ADDR_SCL        0x4Bu

/* HAL wants the 7-bit address shifted up by one. */
#define TMP102_HAL_ADDR(a7)    ((uint16_t)((a7) << 1))

/* ------------------------------------------------------------------ */
/* Register pointers                                                   */
/* ------------------------------------------------------------------ */
#define TMP102_REG_TEMP        0x00u
#define TMP102_REG_CONFIG      0x01u
#define TMP102_REG_TLOW        0x02u
#define TMP102_REG_THIGH       0x03u

/* ------------------------------------------------------------------ */
/* Configuration register bits (MSB byte is the high half)             */
/* ------------------------------------------------------------------ */
#define TMP102_CFG_OS          (1u << 15)  /* one-shot / conversion ready */
#define TMP102_CFG_R1          (1u << 14)  /* converter resolution, RO    */
#define TMP102_CFG_R0          (1u << 13)
#define TMP102_CFG_F1          (1u << 12)  /* fault queue                 */
#define TMP102_CFG_F0          (1u << 11)
#define TMP102_CFG_POL         (1u << 10)  /* ALERT polarity              */
#define TMP102_CFG_TM          (1u <<  9)  /* thermostat mode             */
#define TMP102_CFG_SD          (1u <<  8)  /* shutdown                    */
#define TMP102_CFG_CR1         (1u <<  7)  /* conversion rate             */
#define TMP102_CFG_CR0         (1u <<  6)
#define TMP102_CFG_AL          (1u <<  5)  /* alert flag, RO              */
#define TMP102_CFG_EM          (1u <<  4)  /* extended (13-bit) mode      */

#define TMP102_CFG_DEFAULT     0x60A0u     /* power-on value              */

/** Continuous conversion rate. Reading faster than this just returns
 *  the same sample again. */
typedef enum {
    TMP102_RATE_0P25HZ = 0,
    TMP102_RATE_1HZ    = 1,
    TMP102_RATE_4HZ    = 2,   /* default */
    TMP102_RATE_8HZ    = 3
} TMP102_Rate;

typedef enum {
    TMP102_OK = 0,
    TMP102_ERR_IO,        /* NACK or timeout on the bus        */
    TMP102_ERR_PARAM      /* bad argument                      */
} TMP102_Status;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/** Bind the driver to an I2C handle and a 7-bit address.
 *  @param addr7 use TMP102_ADDR_GND (0x48) for this board.            */
void TMP102_Init(I2C_HandleTypeDef *hi2c, uint8_t addr7);

/** Does the sensor ACK? Useful as a bring-up probe.                   */
HAL_StatusTypeDef TMP102_IsReady(uint32_t trials);

/** Raw signed count. Multiply by 0.0625 to get degrees Celsius.       */
TMP102_Status TMP102_ReadRaw(int16_t *raw);

/** Temperature in milli-degrees Celsius (25.0 C -> 25000).
 *  Integer only - safe to call from anywhere, no FPU needed.          */
TMP102_Status TMP102_ReadMilliC(int32_t *milli_c);

/** Temperature in degrees Celsius as a float (convenience wrapper).   */
TMP102_Status TMP102_ReadCelsius(float *celsius);

/** Read / write the 16-bit configuration register.                    */
TMP102_Status TMP102_ReadConfig(uint16_t *cfg);
TMP102_Status TMP102_WriteConfig(uint16_t cfg);

/** Set the continuous conversion rate.                                */
TMP102_Status TMP102_SetRate(TMP102_Rate rate);

/** Enable or disable 13-bit extended mode (range up to +150 C).       */
TMP102_Status TMP102_SetExtendedMode(uint8_t enable);

/* ---- One-shot (low power) ---------------------------------------- */
/* Put the sensor in shutdown, then trigger single conversions on
 * demand. A conversion takes about 26 ms typical. This is the mode to
 * use if you want the temperature sampled at a known instant rather
 * than free running.                                                  */

/** Enter shutdown mode (no continuous conversions).                   */
TMP102_Status TMP102_Shutdown(void);

/** Trigger one conversion while in shutdown.                          */
TMP102_Status TMP102_StartOneShot(void);

/** 1 when the one-shot conversion has finished.                       */
TMP102_Status TMP102_IsConversionDone(uint8_t *done);

#endif /* TMP102_H */
