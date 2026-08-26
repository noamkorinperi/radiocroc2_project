/**
 ******************************************************************************
 * @file    radioroc2.h
 * @brief   RADIOROC2 ASIC - I2C Slow Control driver (custom protocol)
 *
 * The RADIOROC2 uses a NON-standard I2C protocol (datasheet v2.0, p.8-10).
 * The 8-bit address byte is built as:
 *
 *        [ CHIP_ID (4b) ][ internal register (3b) ][ R/W (1b) ]
 *
 * so each internal slave register (R0..R7) is a *separate* I2C address.
 * A single Slow-Control write is therefore THREE independent I2C frames:
 *
 *        R0 <- sub-address   (LSB of the full address)
 *        R1 <- address       (MSB: 0-63 = channel, 64-67 = global blocks)
 *        R2 <- data          (commits the write)
 *
 * REQUIREMENT: clk_sm_i2c (PE9 / TIM1_CH1) must run at EXACTLY 20 x SCL
 * and be synchronous with it. With SCL = 100 kHz -> clk_sm_i2c = 2 MHz.
 * Start that PWM before calling any function here.
 ******************************************************************************
 */
#ifndef RADIOROC2_H
#define RADIOROC2_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Chip identity                                                       */
/* ------------------------------------------------------------------ */
/* CHIP_ID<3:0> is hard-strapped on the test board. Datasheet default  */
/* is 0b0001. VERIFY the strapping on YOUR board and change if needed. */
#ifndef RR2_CHIP_ID
#define RR2_CHIP_ID            0x1u
#endif

/* ------------------------------------------------------------------ */
/* Internal I2C slave-core registers (Table 3)                         */
/* Selected by the 3 address bits, NOT by a data byte.                 */
/* ------------------------------------------------------------------ */
#define RR2_REG_ADDR_LSB       0u   /* R0: full address LSB = SUB-ADDRESS */
#define RR2_REG_ADDR_MSB       1u   /* R1: full address MSB = ADDRESS     */
#define RR2_REG_DATA           2u   /* R2: data (read / write)            */
#define RR2_REG_DATA_AINC      3u   /* R3: data + auto-increment subadd   */
#define RR2_REG_STATUS         7u   /* R7: status (error, parity)         */

/* 8-bit HAL device address for a given internal register.             */
/* Wire byte = [CHIP_ID(4)][reg(3)][R/W(1)].                           */
#define RR2_HAL_ADDR(reg)  ((uint8_t)((((RR2_CHIP_ID << 3) | ((reg) & 0x7u)) << 1)))

/* ------------------------------------------------------------------ */
/* Common ASIC "address" values (R1 field)                             */
/* ------------------------------------------------------------------ */
/* 0..63  -> per-channel configuration (address == channel number)     */
#define RR2_ADDR_CH(n)         ((uint8_t)(n))   /* n = 0..63            */
#define RR2_ADDR_BIASING       64u              /* ASIC biasing         */
#define RR2_ADDR_COMMON        65u              /* common blocks        */
#define RR2_ADDR_OUTING        66u              /* outing               */
#define RR2_ADDR_EVENTGATE     67u              /* event validation     */

/* ------------------------------------------------------------------ */
/* Return codes                                                        */
/* ------------------------------------------------------------------ */
typedef enum {
    RR2_OK = 0,
    RR2_ERR_LSB,       /* NACK / timeout while writing R0 (sub-address) */
    RR2_ERR_MSB,       /* NACK / timeout while writing R1 (address)     */
    RR2_ERR_DATA,      /* NACK / timeout on R2 (data), or bad argument  */
    RR2_ERR_READBACK,  /* read-back value did not match written value   */
    RR2_ERR_ADC        /* ADC conversion failed or timed out (DAQ)      */
} RR2_Status;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/** Bind the driver to an I2C handle. Call once, after MX_I2C1_Init(). */
void RR2_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Free the bus if a slave is still holding it. Call before use.
 *
 * @return 1 if the bus is usable (already was, or has been freed),
 *         0 if SDA is still held low and only power will clear it.
 */
uint8_t RR2_I2C_BusRecover(void);

/** Stage-1 bring-up: does the ASIC ACK on its Chip ID? (HAL_OK = yes) */
HAL_StatusTypeDef RR2_IsReady(uint32_t trials);

/** Write one Slow-Control byte to (address, subadd). 3 I2C frames.    */
RR2_Status RR2_Write(uint8_t address, uint8_t subadd, uint8_t data);

/** Read one Slow-Control byte from (address, subadd).                 */
RR2_Status RR2_Read(uint8_t address, uint8_t subadd, uint8_t *data);

/** Write then read back and compare (handy self-test).                */
RR2_Status RR2_WriteVerify(uint8_t address, uint8_t subadd, uint8_t data);

/** Optional burst write using R3 auto-increment. VERIFY with read-back
 *  before trusting it - the auto-increment semantics in the datasheet
 *  are described only in prose (p.9).                                  */
RR2_Status RR2_WriteBurst(uint8_t address, uint8_t start_subadd,
                          const uint8_t *data, uint8_t len);

/** Read the status register R7 (error / parity flags).                */
RR2_Status RR2_ReadStatus(uint8_t *status);

#endif /* RADIOROC2_H */
