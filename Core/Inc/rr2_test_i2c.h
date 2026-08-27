/**
 ******************************************************************************
 * @file    rr2_test_i2c.h
 * @brief   Scope test 2 of 2 - Slow Control I2C bus (PB8 SCL / PB9 SDA).
 *
 * WHAT THIS TEST ANSWERS
 * Three separate questions, all of which look identical from software
 * (everything just NACKs) and are trivial to tell apart on a scope:
 *
 *   1. Is SCL the frequency we think it is?
 *      The ASIC requires clk_sm_i2c = 20 x SCL. PA9 runs at exactly
 *      2.000 MHz, so SCL must be 100 kHz. But the CubeMX timing word
 *      0x20404768 gives tSCLH = 4.00 us and tSCLL = 5.83 us, i.e. about
 *      101.7 kHz nominal - a ratio of 19.7, just UNDER the required 20.
 *      Sync and rise time pull it back to roughly 20.3. It sits exactly
 *      on the boundary, and only a scope can settle it.
 *
 *   2. Are the pull-ups doing their job?
 *      Both buses are configured GPIO_NOPULL (correct for I2C), so the
 *      pull-ups must be on the test board. Too weak and SCL/SDA turn
 *      into sawtooths instead of square edges.
 *
 *   3. Does the ASIC actually ACK?
 *      The 9th clock of each byte is the ACK bit. Low = the chip
 *      answered, high = nobody home. g_test_i2c_status reports the same
 *      thing, but the scope shows you WHICH byte failed.
 *
 * PROBE SETUP
 *   CH1 -> PB8  (SCL)
 *   CH2 -> PB9  (SDA)
 *   CH3 -> PA9  (CLK_SM_I2C), if you have a third channel
 *
 * With CH3 connected you can read the 20:1 ratio directly off the
 * screen: count PA9 edges inside one SCL period. That measurement is
 * the whole point of this test.
 *
 * Trigger on CH2 falling edge (START condition, SDA falling while SCL
 * is high). The test repeats the same transaction forever, so the
 * picture stands still.
 *
 * HOW TO RUN IT FROM THE IDE
 * Add these to Live Expressions and edit them while running:
 *
 *   g_test_i2c_mode     0 = off, 1 = run             <- the on/off switch
 *   g_test_i2c_pattern  1 = SINGLE, 2 = SLOWCTRL
 *   g_test_i2c_gap_ms   idle gap between transactions
 *
 * Results, refreshed after every transaction:
 *
 *   g_test_i2c_status    0 = HAL_OK, 1 = ERROR, 3 = TIMEOUT (NACK)
 *   g_test_i2c_scl_khz   effective SCL, derived from the wire time
 *   g_test_i2c_frame_us  duration of one transaction
 *   g_test_i2c_ratio_x10 clk_sm_i2c : SCL ratio, x10 (200 = exactly 20)
 *   g_test_i2c_ok        transactions that ACKed
 *   g_test_i2c_fail      transactions that did not
 *
 * NOTE g_test_i2c_scl_khz is derived from timing the HAL call, so it
 *      carries a few percent of software overhead and reads slightly
 *      LOW. Treat it as a sanity check on the scope, not a replacement.
 *      g_test_i2c_ratio_x10 below 200 means the ASIC is being clocked
 *      too slowly for its own I2C core.
 ******************************************************************************
 */
#ifndef RR2_TEST_I2C_H
#define RR2_TEST_I2C_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Patterns                                                            */
/* ------------------------------------------------------------------ */
/* One START..STOP frame, one data byte. The shortest repeating picture
   you can get, which makes it the easiest to measure frequency on.    */
#define RR2_TEST_I2C_SINGLE     1u

/* A full Slow Control write: three back-to-back frames (R0 sub-address,
   R1 address, R2 data). This is what production traffic looks like, and
   it is the pattern to use when hunting an ASIC that ACKs the first
   frame but not the rest.                                             */
#define RR2_TEST_I2C_SLOWCTRL   2u

/* ------------------------------------------------------------------ */
/* Controls - write these from Live Expressions                        */
/* ------------------------------------------------------------------ */
extern volatile uint8_t  g_test_i2c_mode;      /* 0 = off, 1 = run     */
extern volatile uint8_t  g_test_i2c_pattern;   /* RR2_TEST_I2C_*       */
extern volatile uint16_t g_test_i2c_gap_ms;    /* gap between frames   */

/* ------------------------------------------------------------------ */
/* Measurements - read these from Live Expressions                     */
/* ------------------------------------------------------------------ */
extern volatile uint8_t  g_test_i2c_status;     /* HAL_StatusTypeDef   */
extern volatile uint32_t g_test_i2c_scl_khz;    /* effective SCL       */
extern volatile uint32_t g_test_i2c_frame_us;   /* one transaction     */
extern volatile uint32_t g_test_i2c_ratio_x10;  /* clk:SCL ratio x10   */
extern volatile uint32_t g_test_i2c_ok;         /* ACKed               */
extern volatile uint32_t g_test_i2c_fail;       /* NACKed / timed out  */

/**
 * @brief Run the I2C test for as long as g_test_i2c_mode stays non-zero.
 *
 * Blocks on purpose, same reasoning as the clock test: the scope needs
 * a repeating transaction, and nothing else may drive the bus while you
 * are probing it. Clearing g_test_i2c_mode returns to the main loop.
 *
 * Writes are aimed at a harmless target - the hold-delay register of the
 * common block, rewritten with the value it already holds - so a test
 * run never disturbs the ASIC configuration.
 */
void RR2_TestI2C_Task(void);

#endif /* RR2_TEST_I2C_H */
