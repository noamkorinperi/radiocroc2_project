/**
 ******************************************************************************
 * @file    rr2_test_i2c.c
 * @brief   Scope test 2 of 2 - Slow Control I2C bus (PB8 SCL / PB9 SDA).
 * @note    See rr2_test_i2c.h for the probe setup and the variable list.
 ******************************************************************************
 */
#include "rr2_test_i2c.h"
#include "radioroc2.h"
#include "radioroc2_ctrl.h"
#include "main.h"
#include "i2c.h"
#include "tim.h"

#define TEST_I2C_TIMEOUT_MS   50u
#define TEST_CPU_HZ           216000000u

/* SCL clocks in one single-byte write frame: 8 address bits + ACK, then
   8 data bits + ACK. START and STOP are not full clock periods and are
   deliberately left out - they would bias the derived frequency high. */
#define TEST_I2C_CLOCKS_PER_FRAME  18u

/* TIM1 is fed from the APB2 timer clock, which the PLL puts at 216 MHz. */
#define TEST_TIM1_CLK_KHZ     216000u

/* ------------------------------------------------------------------ */
/* Controls                                                            */
/* ------------------------------------------------------------------ */
volatile uint8_t  g_test_i2c_mode       = 0u;
volatile uint8_t  g_test_i2c_pattern    = RR2_TEST_I2C_SINGLE;
volatile uint16_t g_test_i2c_gap_ms     = 1u;
volatile uint8_t  g_test_i2c_scan_apply = 0u;

/* ------------------------------------------------------------------ */
/* Measurements                                                        */
/* ------------------------------------------------------------------ */
volatile uint8_t  g_test_i2c_status    = 0u;
volatile uint32_t g_test_i2c_scl_khz   = 0u;
volatile uint32_t g_test_i2c_frame_us  = 0u;
volatile uint32_t g_test_i2c_ratio_x10 = 0u;
volatile uint32_t g_test_i2c_ok        = 0u;
volatile uint32_t g_test_i2c_fail      = 0u;

volatile uint16_t g_test_i2c_scan_map    = 0u;
volatile uint8_t  g_test_i2c_scan_found  = RR2_CHIP_ID_NONE;
volatile uint8_t  g_test_i2c_scan_active = 0u;
volatile uint32_t g_test_i2c_scan_passes = 0u;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */
static uint32_t cyc_to_ns(uint32_t cycles)
{
    return (cycles * 1000u) / (TEST_CPU_HZ / 1000000u);
}

/**
 * @brief Send one byte to one of the ASIC's internal registers and time it.
 * @param reg     internal register R0..R7
 * @param value   the byte to put on the wire
 * @param out_ns  wire time of the whole START..STOP frame
 * @retval HAL status - HAL_TIMEOUT / HAL_ERROR both mean "no ACK"
 */
static HAL_StatusTypeDef timed_frame(uint8_t reg, uint8_t value, uint32_t *out_ns)
{
    const uint32_t t0 = DWT->CYCCNT;

    const HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1,
                                                         RR2_HAL_ADDR(reg),
                                                         &value, 1u,
                                                         TEST_I2C_TIMEOUT_MS);

    *out_ns = cyc_to_ns(DWT->CYCCNT - t0);
    return st;
}

/**
 * @brief Turn one measured frame time into an effective SCL frequency,
 *        and into the clk_sm_i2c : SCL ratio the ASIC cares about.
 *
 * The frame time includes a few microseconds of HAL state machine on
 * top of the wire time, so the answer reads slightly low. That is fine
 * for the question being asked - whether the ratio is near 20 or not.
 */
static void derive_frequencies(uint32_t frame_ns)
{
    if (frame_ns == 0u) return;

    /* f_SCL = clocks / frame_time. Kept in kHz to stay in 32 bits. */
    g_test_i2c_scl_khz = (TEST_I2C_CLOCKS_PER_FRAME * 1000000u) / frame_ns;

    /* clk_sm_i2c comes straight from the timer config rather than a
       hard-coded 2 MHz, so this stays honest if the period changes. */
    const uint32_t period  = htim1.Init.Period + 1u;
    const uint32_t clk_khz = (period > 0u) ? (TEST_TIM1_CLK_KHZ / period) : 0u;

    if (g_test_i2c_scl_khz > 0u) {
        g_test_i2c_ratio_x10 = (clk_khz * 10u) / g_test_i2c_scl_khz;
    }
}

static void account(HAL_StatusTypeDef st)
{
    g_test_i2c_status = (uint8_t)st;

    if (st == HAL_OK) g_test_i2c_ok++;
    else              g_test_i2c_fail++;
}

/* ------------------------------------------------------------------ */
/* Patterns                                                            */
/* ------------------------------------------------------------------ */
/**
 * @brief One frame: load the sub-address latch (R0).
 *
 * Harmless by construction - R0 only latches which sub-address a later
 * data write would target, and nothing follows it here.
 */
static void pattern_single(void)
{
    uint32_t ns = 0u;

    const HAL_StatusTypeDef st = timed_frame(RR2_REG_ADDR_LSB,
                                             RR2_COM_SUB_DELAY, &ns);
    account(st);

    g_test_i2c_frame_us = ns / 1000u;
    derive_frequencies(ns);
}

/**
 * @brief Three frames: a complete Slow Control write, R0 -> R1 -> R2.
 *
 * The data byte is the hold-delay register rewritten with the value the
 * shadow already holds, so a test run leaves the ASIC configuration
 * exactly as it found it.
 *
 * All three frames are timed, but only the first feeds the frequency
 * calculation - the gaps between frames are software, not wire time,
 * and would drag the derived SCL down.
 */
static void pattern_slowctrl(void)
{
    const RR2_Shadow *sh = RR2_Ctrl_GetShadow();
    uint32_t ns0 = 0u, ns1 = 0u, ns2 = 0u;
    HAL_StatusTypeDef st;

    st = timed_frame(RR2_REG_ADDR_LSB, RR2_COM_SUB_DELAY, &ns0);
    if (st == HAL_OK) {
        st = timed_frame(RR2_REG_ADDR_MSB, RR2_ADDR_COMMON, &ns1);
    }
    if (st == HAL_OK) {
        st = timed_frame(RR2_REG_DATA, sh->com_delay, &ns2);
    }
    account(st);

    g_test_i2c_frame_us = (ns0 + ns1 + ns2) / 1000u;
    derive_frequencies(ns0);
}

/**
 * @brief Sweep all 16 chip ids and report which ones answer.
 *
 * This is the pattern to run when the board's CHIP_ID strapping is
 * unknown. On the scope you will see 32 short probe frames march up
 * through the address space; the one (or ones) that ACK stand out
 * because SDA is pulled low on the 9th clock instead of floating high.
 *
 * Slower than the other patterns - a sweep costs up to 32 timeouts
 * when nothing is connected - so it deliberately runs one sweep per
 * pass with the usual gap after it.
 */
static void pattern_scan(void)
{
    uint16_t map = 0u;

    const uint8_t found = RR2_ScanChipId(&map);

    g_test_i2c_scan_map   = map;
    g_test_i2c_scan_found = found;

    if (found == RR2_CHIP_ID_NONE) {
        g_test_i2c_fail++;
        g_test_i2c_status = (uint8_t)HAL_ERROR;
    } else {
        g_test_i2c_ok++;
        g_test_i2c_status = (uint8_t)HAL_OK;

        /* Opt-in: retarget the driver so the other patterns, and the
           DAQ afterwards, talk to the chip that actually answered. */
        if (g_test_i2c_scan_apply) {
            RR2_SetChipId(found);
        }
    }

    g_test_i2c_scan_active = RR2_GetChipId();
    g_test_i2c_scan_passes++;

    /* A sweep says nothing about bus timing, so leave the frequency
       figures from whatever pattern measured them last. */
}

/* ------------------------------------------------------------------ */
/* Entry point                                                         */
/* ------------------------------------------------------------------ */
void RR2_TestI2C_Task(void)
{
    if (!g_test_i2c_mode) return;

    /* The ASIC's I2C slave core is clocked by CLK_SM_I2C. Without it the
       chip cannot answer, and the whole test would report a dead bus for
       the wrong reason. Make sure PA8 is running before touching SCL. */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    g_test_i2c_ok   = 0u;
    g_test_i2c_fail = 0u;

    while (g_test_i2c_mode) {
        switch (g_test_i2c_pattern) {
        case RR2_TEST_I2C_SCAN:
            pattern_scan();
            break;

        case RR2_TEST_I2C_SLOWCTRL:
            pattern_slowctrl();
            break;

        case RR2_TEST_I2C_SINGLE:
        default:
            pattern_single();
            break;
        }

        /* Separates one transaction from the next so the scope has a
           quiet stretch to trigger against. */
        HAL_Delay(g_test_i2c_gap_ms);
    }
}
