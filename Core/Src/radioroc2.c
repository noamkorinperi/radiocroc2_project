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

/**
 * @brief Free a bus that a slave is still holding, and say whether it was.
 *
 * A reset landing in the middle of a Slow Control frame resets the
 * STM32 and nothing else. The ASIC is left part way through clocking a
 * byte out, holding SDA low and waiting on SCL edges the restarted
 * firmware never sends. From then on no START is valid and every
 * transaction fails - which presents as a chip that still ACKs its ID
 * and refuses every write, with i2ctest reporting SDA_STUCK_LOW.
 * Nothing in software cleared it, so the only cure was to pull power,
 * which on a board wired into a temperature soak is not a small ask.
 *
 * The way out is to drive SCL by hand: every falling edge advances the
 * slave one bit, and within nine it reaches a bit it does not drive and
 * lets go. A hand-made STOP afterwards leaves it at idle rather than
 * stranded mid-transfer.
 *
 * Costs nothing when the bus is already idle, and is safe with no ASIC
 * on the other end. Clocked from HAL_Delay, so about 500 Hz - unjamming
 * has no minimum speed, and this needs no calibrated timer.
 */
uint8_t RR2_I2C_BusRecover(void)
{
    GPIO_InitTypeDef g = {0};
    uint8_t freed;

    if (rr2_i2c == NULL) return 0u;

    /* Idle already. Do not touch a working bus. */
    if ((GPIOB->IDR & GPIO_PIN_9) != 0u) return 1u;

    /* Take the pads off the peripheral and drive them the way I2C does:
       open drain, no internal pull, the board's own pull-ups making the
       high level. */
    (void)HAL_I2C_DeInit(rr2_i2c);

    g.Pin   = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(1u);

    for (uint8_t i = 0u; i < 9u; ++i) {
        if ((GPIOB->IDR & GPIO_PIN_9) != 0u) break;   /* let go already */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_Delay(1u);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(1u);
    }
    freed = ((GPIOB->IDR & GPIO_PIN_9) != 0u) ? 1u : 0u;

    /* STOP by hand - SDA released while SCL is high. Without it the
       slave is free but still inside a transfer, and the next START
       would read as a repeated one. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_Delay(1u);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(1u);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(1u);

    /* Hand the pads back. DeInit left the handle in RESET, so this runs
       HAL_I2C_MspInit() again and the pins return to AF4. */
    (void)HAL_I2C_Init(rr2_i2c);

    return freed;
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
