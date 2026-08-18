/**
 ******************************************************************************
 * @file    rr2_test_clocks.h
 * @brief   Scope test 1 of 2 - CLK_SM_I2C (PA8) and CK_READ (PA1) timing.
 *
 * WHAT THIS TEST ANSWERS
 * The readout clock is bit-banged in software (radioroc2_daq.c), so the
 * 100 ns constants in radioroc2_daq.h are a REQUEST, not a guarantee.
 * At 216 MHz a 100 ns delay is only ~21 CPU cycles, which is the same
 * order as the cost of one HAL_GPIO_WritePin() call. This test drives
 * the real production code path in a repeating pattern so a scope can
 * measure what CK_READ actually looks like, and reports the same numbers
 * back through the debugger as a cross-check.
 *
 * PROBE SETUP
 *   CH1 -> PA8  (CLK_SM_I2C)  free-running 2.000 MHz, the known-good ref
 *   CH2 -> PA1  (CK_READ)     the signal under test
 *   Trigger on CH2. In BURST pattern the inter-burst gap gives a stable
 *   trigger point; in SQUARE pattern trigger on either edge.
 *
 * HOW TO RUN IT FROM THE IDE
 * No terminal and no host software needed. Add these to Live Expressions
 * (Window > Show View > Live Expressions) and edit them while running:
 *
 *   g_test_clk_mode      0 = off, 1 = run          <- the on/off switch
 *   g_test_clk_pattern   1 = SQUARE, 2 = BURST     <- what to emit
 *   g_test_clk_burst_len pulses per burst (BURST)
 *   g_test_clk_gap_ms    idle gap between bursts
 *
 * Results, refreshed continuously while the test runs:
 *
 *   g_test_clk_period_ns   measured CK_READ period, averaged
 *   g_test_clk_freq_khz    the same as a frequency
 *   g_test_clk_burst_us    time for one full burst
 *   g_test_clk_gpio_ns     cost of ONE HAL_GPIO_WritePin call
 *   g_test_clk_passes      increments every pass, proves it is alive
 *
 * Live Expressions updates while the core runs, so you can change the
 * pattern with one hand on the probe. No breakpoints, no halting.
 *
 * NOTE g_test_clk_gpio_ns is the interesting one. If it is anywhere near
 *      100 ns then the delay constants are being swamped by the GPIO call
 *      itself and CK_READ cannot be as fast as the header claims. See the
 *      I-cache / ART settings in stm32f7xx_hal_conf.h.
 ******************************************************************************
 */
#ifndef RR2_TEST_CLOCKS_H
#define RR2_TEST_CLOCKS_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Patterns                                                            */
/* ------------------------------------------------------------------ */
#define RR2_TEST_CLK_SQUARE   1u   /* CK_READ pulses back to back      */
#define RR2_TEST_CLK_BURST    2u   /* full readout pattern, then a gap */

/* ------------------------------------------------------------------ */
/* Controls - write these from Live Expressions                        */
/* ------------------------------------------------------------------ */
extern volatile uint8_t  g_test_clk_mode;        /* 0 = off, 1 = run   */
extern volatile uint8_t  g_test_clk_pattern;     /* RR2_TEST_CLK_*     */
extern volatile uint16_t g_test_clk_burst_len;   /* pulses per burst   */
extern volatile uint16_t g_test_clk_gap_ms;      /* gap between bursts */

/* ------------------------------------------------------------------ */
/* Measurements - read these from Live Expressions                     */
/* ------------------------------------------------------------------ */
extern volatile uint32_t g_test_clk_period_ns;   /* CK_READ period     */
extern volatile uint32_t g_test_clk_freq_khz;    /* derived frequency  */
extern volatile uint32_t g_test_clk_burst_us;    /* one burst duration */
extern volatile uint32_t g_test_clk_gpio_ns;     /* one GPIO write     */
extern volatile uint32_t g_test_clk_passes;      /* liveness counter   */

/**
 * @brief Run the clock test for as long as g_test_clk_mode stays non-zero.
 *
 * Blocks on purpose: a scope needs a repeating waveform to trigger on,
 * and the DAQ must not fight for CK_READ while you are probing it.
 * Clearing g_test_clk_mode from the debugger returns control to the
 * main loop, with the readout lines parked in their idle state.
 */
void RR2_TestClocks_Task(void);

#endif /* RR2_TEST_CLOCKS_H */
