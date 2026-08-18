/**
 ******************************************************************************
 * @file    rr2_test_clocks.c
 * @brief   Scope test 1 of 2 - CLK_SM_I2C (PE9) and CK_READ (PA1) timing.
 * @note    See rr2_test_clocks.h for the probe setup and the variable list.
 ******************************************************************************
 */
#include "rr2_test_clocks.h"
#include "radioroc2_daq.h"
#include "main.h"
#include "tim.h"

/* Averaging depth. 1000 pulses at a few hundred ns each is well under a
   millisecond, so the numbers refresh fast enough to watch them move
   while turning a knob on the scope.                                   */
#define TEST_CLK_SAMPLES   1000u

#define TEST_CPU_HZ        216000000u

/* ------------------------------------------------------------------ */
/* Controls                                                            */
/* ------------------------------------------------------------------ */
volatile uint8_t  g_test_clk_mode      = 0u;
volatile uint8_t  g_test_clk_pattern   = RR2_TEST_CLK_SQUARE;
volatile uint16_t g_test_clk_burst_len = RR2_NUM_CHANNELS;   /* 64 */
volatile uint16_t g_test_clk_gap_ms    = 1u;

/* ------------------------------------------------------------------ */
/* Measurements                                                        */
/* ------------------------------------------------------------------ */
volatile uint32_t g_test_clk_period_ns = 0u;
volatile uint32_t g_test_clk_freq_khz  = 0u;
volatile uint32_t g_test_clk_burst_us  = 0u;
volatile uint32_t g_test_clk_gpio_ns   = 0u;
volatile uint32_t g_test_clk_passes    = 0u;

/* ------------------------------------------------------------------ */
/* Cycle counting                                                      */
/* ------------------------------------------------------------------ */
/* RR2_DAQ_Init() already enabled and validated the DWT. If it is dead
   every measurement here would read zero, so the results are simply
   left untouched and only the waveform itself stays useful.           */
static uint32_t cyc_to_ns(uint32_t cycles)
{
    /* 216 cycles per us. Scaled to keep the division in 32 bits for
       the magnitudes this test deals with (< ~19 s).                  */
    return (cycles * 1000u) / (TEST_CPU_HZ / 1000000u);
}

/* ------------------------------------------------------------------ */
/* Measure the cost of the GPIO call itself                            */
/* ------------------------------------------------------------------ */
/**
 * @brief Time one HAL_GPIO_WritePin, with no delay around it.
 *
 * This is the number that decides whether the ns constants in
 * radioroc2_daq.h are achievable at all. CK_READ toggles during the
 * measurement, which is harmless - the read pointer is rewound before
 * every burst anyway.
 */
static void measure_gpio_cost(void)
{
    const uint32_t t0 = DWT->CYCCNT;

    for (uint32_t i = 0u; i < TEST_CLK_SAMPLES; ++i) {
        HAL_GPIO_WritePin(CK_READ_GPIO_Port, CK_READ_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CK_READ_GPIO_Port, CK_READ_Pin, GPIO_PIN_RESET);
    }

    const uint32_t dt = DWT->CYCCNT - t0;

    /* Two writes per iteration. */
    g_test_clk_gpio_ns = cyc_to_ns(dt / (TEST_CLK_SAMPLES * 2u));
}

/* ------------------------------------------------------------------ */
/* Patterns                                                            */
/* ------------------------------------------------------------------ */
/**
 * @brief Back-to-back CK_READ pulses, no gap.
 *
 * The simplest thing to measure: put a cursor on two adjacent rising
 * edges and read the period straight off the scope. Compare against
 * g_test_clk_period_ns, and against the 100 ns high + 100 ns low that
 * RR2_CKREAD_HIGH_NS / RR2_CKREAD_LOW_NS ask for.
 */
static void pattern_square(void)
{
    const uint32_t t0 = DWT->CYCCNT;

    for (uint32_t i = 0u; i < TEST_CLK_SAMPLES; ++i) {
        RR2_DAQ_ClockOnce();
    }

    const uint32_t dt     = DWT->CYCCNT - t0;
    const uint32_t per_ns = cyc_to_ns(dt / TEST_CLK_SAMPLES);

    g_test_clk_period_ns = per_ns;
    g_test_clk_freq_khz  = (per_ns > 0u) ? (1000000u / per_ns) : 0u;
}

/**
 * @brief The real readout pattern: rewind, N pulses, end-of-readout, gap.
 *
 * This is what the ASIC actually sees during an event, so it also puts
 * RSTN_READ (PD1) and RESET_N (PD4) on the wire - worth a third probe
 * if you have one. The idle gap is what makes the burst easy to trigger
 * on; shrink g_test_clk_gap_ms once you have a stable picture.
 */
static void pattern_burst(void)
{
    const uint16_t len = g_test_clk_burst_len;

    const uint32_t t0 = DWT->CYCCNT;

    RR2_DAQ_ResetReadPointer();
    for (uint16_t i = 0u; i < len; ++i) {
        RR2_DAQ_ClockOnce();
    }
    RR2_DAQ_EndOfReadout();

    const uint32_t dt = DWT->CYCCNT - t0;

    g_test_clk_burst_us = cyc_to_ns(dt) / 1000u;

    if (len > 0u) {
        const uint32_t per_ns = cyc_to_ns(dt / len);
        g_test_clk_period_ns  = per_ns;
        g_test_clk_freq_khz   = (per_ns > 0u) ? (1000000u / per_ns) : 0u;
    }

    /* The gap is what the scope triggers on. */
    HAL_Delay(g_test_clk_gap_ms);
}

/* ------------------------------------------------------------------ */
/* Entry point                                                         */
/* ------------------------------------------------------------------ */
void RR2_TestClocks_Task(void)
{
    if (!g_test_clk_mode) return;

    /* PE9 carries the 2 MHz reference. It is started during boot, but
       start it again in case the test is entered after something else
       stopped it - HAL is happy to be told twice.                     */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    /* Once per entry, not per pass: it does not change and re-measuring
       it would only add jitter to the waveform.                       */
    measure_gpio_cost();

    while (g_test_clk_mode) {
        switch (g_test_clk_pattern) {
        case RR2_TEST_CLK_BURST:
            pattern_burst();
            break;

        case RR2_TEST_CLK_SQUARE:
        default:
            pattern_square();
            break;
        }
        g_test_clk_passes++;
    }

    /* Park the lines exactly as RR2_DAQ_Init() leaves them, so the DAQ
       can resume without a stray edge. */
    HAL_GPIO_WritePin(CK_READ_GPIO_Port,   CK_READ_Pin,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RSTN_READ_GPIO_Port, RSTN_READ_Pin, GPIO_PIN_SET);
}
