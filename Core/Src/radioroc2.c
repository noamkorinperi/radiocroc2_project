/**
 ******************************************************************************
 * @file    radioroc2.c
 * @brief   RADIOROC2 ASIC - I2C Slow Control driver (custom protocol)
 * @note    See radioroc2.h for the protocol description.
 ******************************************************************************
 */
#include "radioroc2.h"
#include <stddef.h>   /* NULL (usually via CMSIS, kept explicit for portability) */

/* Timeout per single I2C frame, in ms. 100 ms is generous for bring-up. */
#define RR2_I2C_TIMEOUT   100u

static I2C_HandleTypeDef *rr2_i2c = NULL;

/* The chip id every transaction is addressed to. Starts at the
   compile-time default and is overridden once the board is scanned. */
static uint8_t rr2_chip_id = (uint8_t)RR2_CHIP_ID;

/* ------------------------------------------------------------------ */
/* Low-level: one byte to / from a given internal register            */
/* ------------------------------------------------------------------ */
static HAL_StatusTypeDef rr2_reg_write(uint8_t reg, uint8_t value)
{
    /* One full START..STOP frame addressed to the chosen internal reg. */
    return HAL_I2C_Master_Transmit(rr2_i2c, RR2_HAL_ADDR(reg),
                                   &value, 1u, RR2_I2C_TIMEOUT);
}

static HAL_StatusTypeDef rr2_reg_read(uint8_t reg, uint8_t *value)
{
    /* HAL sets the R/W bit to 1 for us on the wire. */
    return HAL_I2C_Master_Receive(rr2_i2c, RR2_HAL_ADDR(reg),
                                  value, 1u, RR2_I2C_TIMEOUT);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
void RR2_Init(I2C_HandleTypeDef *hi2c)
{
    rr2_i2c = hi2c;
}

void RR2_SetChipId(uint8_t id)
{
    rr2_chip_id = (uint8_t)(id & 0x0Fu);
}

uint8_t RR2_GetChipId(void)
{
    return rr2_chip_id;
}

uint8_t RR2_ScanChipId(uint16_t *map)
{
    uint16_t hits  = 0u;
    uint8_t  first = RR2_CHIP_ID_NONE;

    if (rr2_i2c == NULL) {
        if (map != NULL) *map = 0u;
        return RR2_CHIP_ID_NONE;
    }

    for (uint8_t id = 0u; id < 16u; ++id) {
        /* Demand an ACK on two different internal registers before
           believing a hit - one stray address is easy to collide with,
           two consecutive ones much less so. */
        const uint8_t a0 = RR2_HAL_ADDR_OF(id, RR2_REG_ADDR_LSB);
        const uint8_t a1 = RR2_HAL_ADDR_OF(id, RR2_REG_ADDR_MSB);

        if ((HAL_I2C_IsDeviceReady(rr2_i2c, a0, 2u, RR2_I2C_TIMEOUT) == HAL_OK) &&
            (HAL_I2C_IsDeviceReady(rr2_i2c, a1, 2u, RR2_I2C_TIMEOUT) == HAL_OK)) {

            hits |= (uint16_t)(1u << id);
            if (first == RR2_CHIP_ID_NONE) first = id;
        }
    }

    if (map != NULL) *map = hits;
    return first;
}

HAL_StatusTypeDef RR2_IsReady(uint32_t trials)
{
    /* Probe R0's address. The ASIC ACKs when CHIP_ID matches, which
       tells us the PCA9306 level-shifter and the I2C link are alive. */
    return HAL_I2C_IsDeviceReady(rr2_i2c, RR2_HAL_ADDR(RR2_REG_ADDR_LSB),
                                 trials, RR2_I2C_TIMEOUT);
}

RR2_Status RR2_Write(uint8_t address, uint8_t subadd, uint8_t data)
{
    /* Custom protocol (Figure 3): three separate START..STOP frames. */
    if (rr2_reg_write(RR2_REG_ADDR_LSB, subadd)  != HAL_OK) return RR2_ERR_LSB;
    if (rr2_reg_write(RR2_REG_ADDR_MSB, address) != HAL_OK) return RR2_ERR_MSB;
    if (rr2_reg_write(RR2_REG_DATA,     data)    != HAL_OK) return RR2_ERR_DATA;
    return RR2_OK;
}

RR2_Status RR2_Read(uint8_t address, uint8_t subadd, uint8_t *data)
{
    if (data == NULL) return RR2_ERR_DATA;
    /* Point R0/R1 at the target, then read R2. */
    if (rr2_reg_write(RR2_REG_ADDR_LSB, subadd)  != HAL_OK) return RR2_ERR_LSB;
    if (rr2_reg_write(RR2_REG_ADDR_MSB, address) != HAL_OK) return RR2_ERR_MSB;
    if (rr2_reg_read (RR2_REG_DATA,     data)    != HAL_OK) return RR2_ERR_DATA;
    return RR2_OK;
}

RR2_Status RR2_WriteVerify(uint8_t address, uint8_t subadd, uint8_t data)
{
    uint8_t readback = 0u;
    RR2_Status st = RR2_Write(address, subadd, data);
    if (st != RR2_OK) return st;

    st = RR2_Read(address, subadd, &readback);
    if (st != RR2_OK) return st;

    return (readback == data) ? RR2_OK : RR2_ERR_READBACK;
}

RR2_Status RR2_WriteBurst(uint8_t address, uint8_t start_subadd,
                          const uint8_t *data, uint8_t len)
{
    if ((data == NULL) || (len == 0u)) return RR2_OK;

    /* Load the starting (address, subadd) once... */
    if (rr2_reg_write(RR2_REG_ADDR_LSB, start_subadd) != HAL_OK) return RR2_ERR_LSB;
    if (rr2_reg_write(RR2_REG_ADDR_MSB, address)      != HAL_OK) return RR2_ERR_MSB;

    /* ...then stream data through R3, which auto-increments the subadd. */
    for (uint8_t i = 0u; i < len; ++i) {
        if (rr2_reg_write(RR2_REG_DATA_AINC, data[i]) != HAL_OK) return RR2_ERR_DATA;
    }
    return RR2_OK;
}

RR2_Status RR2_ReadStatus(uint8_t *status)
{
    if (status == NULL) return RR2_ERR_DATA;
    return (rr2_reg_read(RR2_REG_STATUS, status) == HAL_OK) ? RR2_OK : RR2_ERR_DATA;
}
