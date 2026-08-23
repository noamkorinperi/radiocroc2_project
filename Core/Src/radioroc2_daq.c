/**
 ******************************************************************************
 * @file    radioroc2_daq.c
 * @brief   RADIOROC2 - Stage 2 DAQ: analog-mux readout of the 64 channels.
 * @note    See radioroc2_daq.h for the readout sequence.
 ******************************************************************************
 */
#include "radioroc2_daq.h"
#include "main.h"     /* pin macros generated from the CubeMX User Labels */
#include <stddef.h>

/* ------------------------------------------------------------------ */
/* Module state                                                        */
/* ------------------------------------------------------------------ */
static ADC_HandleTypeDef *rr2_adc_hg = NULL;   /* OUT_AMUXHG -> PA4 */
static ADC_HandleTypeDef *rr2_adc_lg = NULL;   /* OUT_AMUXLG -> PA5 */
static uint32_t rr2_seq = 0u;

/* Per-conversion timeout. One conversion at 144+15 cycles / 27 MHz is
   about 5.9 us, so 2 ms is a very generous ceiling.                   */
#define RR2_ADC_TIMEOUT_MS   2u

/* ------------------------------------------------------------------ */
/* Sub-microsecond timing via the DWT cycle counter                    */
/* ------------------------------------------------------------------ */
#define RR2_CPU_HZ            216000000u
#define RR2_CYCLES_PER_US     (RR2_CPU_HZ / 1000000u)              /* 216 */
#define RR2_NS_TO_CYCLES(ns)  (((ns) * RR2_CYCLES_PER_US) / 1000u)

/* Cortex-M7 adds a CoreSight software lock that Cortex-M3/M4 do not
   have. On some cores the DWT stays locked and CYCCNT never advances
   until this magic key is written to the Lock Access Register. The
   address is used literally because older CMSIS headers do not expose
   a LAR field in DWT_Type.                                           */
#define RR2_DWT_LAR         (*(volatile uint32_t *)0xE0001FB0u)
#define RR2_DWT_UNLOCK_KEY  0xC5ACCE55u

/* 1 once CYCCNT has been observed to actually increment. */
static uint8_t rr2_dwt_ok = 0u;

static void RR2_DWT_Enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    RR2_DWT_LAR = RR2_DWT_UNLOCK_KEY;   /* harmless if already unlocked */
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    /* Prove the counter runs. Without this check a dead DWT would make
       every delay loop below spin forever and hang the firmware.      */
    const uint32_t t0 = DWT->CYCCNT;
    __NOP(); __NOP(); __NOP(); __NOP();
    rr2_dwt_ok = (DWT->CYCCNT != t0) ? 1u : 0u;
}

/** Busy-wait for a number of CPU cycles. Wrap-around safe, and never
 *  blocks indefinitely even if the DWT is unavailable.               */
static void RR2_DelayCycles(uint32_t cycles)
{
    if (rr2_dwt_ok) {
        const uint32_t start = DWT->CYCCNT;
        while ((DWT->CYCCNT - start) < cycles) {
            /* spin */
        }
    } else {
        /* Fallback: coarse volatile loop. One iteration costs several
           cycles, so looping "cycles" times deliberately overshoots.
           Every timing constraint in this driver is a minimum, so a
           longer wait is safe - only the event rate suffers.          */
        for (volatile uint32_t i = 0u; i < cycles; ++i) {
            __NOP();
        }
    }
}

uint8_t RR2_DAQ_IsTimingOk(void)
{
    return rr2_dwt_ok;
}

/* ------------------------------------------------------------------ */
/* Low-level line control                                              */
/* ------------------------------------------------------------------ */
void RR2_DAQ_ClockOnce(void)
{
    /* The RISING edge advances the read register (Figure 26). */
    HAL_GPIO_WritePin(CK_READ_GPIO_Port, CK_READ_Pin, GPIO_PIN_SET);
    RR2_DelayCycles(RR2_NS_TO_CYCLES(RR2_CKREAD_HIGH_NS));
    HAL_GPIO_WritePin(CK_READ_GPIO_Port, CK_READ_Pin, GPIO_PIN_RESET);
    RR2_DelayCycles(RR2_NS_TO_CYCLES(RR2_CKREAD_LOW_NS));
}

void RR2_DAQ_ResetReadPointer(void)
{
    /* RSTN_READ is active low. Asserting it rewinds the shift register
       so that the next CK_READ rising edge presents channel 0.        */
    HAL_GPIO_WritePin(RSTN_READ_GPIO_Port, RSTN_READ_Pin, GPIO_PIN_RESET);
    RR2_DelayCycles(RR2_NS_TO_CYCLES(RR2_RSTN_READ_NS));
    HAL_GPIO_WritePin(RSTN_READ_GPIO_Port, RSTN_READ_Pin, GPIO_PIN_SET);
    RR2_DelayCycles(RR2_NS_TO_CYCLES(RR2_RSTN_READ_NS));
}

void RR2_DAQ_EndOfReadout(void)
{
    /* RESET_N must be asserted for ~20 ns at the end of the readout to
       clear the peak detectors and the delay cell (datasheet p.46).   */
    HAL_GPIO_WritePin(RESET_N_GPIO_Port, RESET_N_Pin, GPIO_PIN_RESET);
    RR2_DelayCycles(RR2_NS_TO_CYCLES(RR2_RESET_N_NS));
    HAL_GPIO_WritePin(RESET_N_GPIO_Port, RESET_N_Pin, GPIO_PIN_SET);
}

void RR2_DAQ_WaitHold(void)
{
    RR2_DelayCycles(RR2_NS_TO_CYCLES(RR2_HOLD_DELAY_NS));
}

/* ------------------------------------------------------------------ */
/* Analog sampling                                                     */
/* ------------------------------------------------------------------ */
RR2_Status RR2_DAQ_SampleBothGains(uint16_t *hg, uint16_t *lg)
{
    if ((hg == NULL) || (lg == NULL))               return RR2_ERR_DATA;
    if ((rr2_adc_hg == NULL) || (rr2_adc_lg == NULL)) return RR2_ERR_ADC;

    /* ADC1 and ADC2 are independent peripherals, so starting both back
       to back lets the two gains convert in parallel: one conversion
       time for both samples instead of two.                          */
    if (HAL_ADC_Start(rr2_adc_hg) != HAL_OK) return RR2_ERR_ADC;
    if (HAL_ADC_Start(rr2_adc_lg) != HAL_OK) {
        HAL_ADC_Stop(rr2_adc_hg);
        return RR2_ERR_ADC;
    }

    if (HAL_ADC_PollForConversion(rr2_adc_hg, RR2_ADC_TIMEOUT_MS) != HAL_OK) {
        HAL_ADC_Stop(rr2_adc_hg);
        HAL_ADC_Stop(rr2_adc_lg);
        return RR2_ERR_ADC;
    }
    if (HAL_ADC_PollForConversion(rr2_adc_lg, RR2_ADC_TIMEOUT_MS) != HAL_OK) {
        HAL_ADC_Stop(rr2_adc_hg);
        HAL_ADC_Stop(rr2_adc_lg);
        return RR2_ERR_ADC;
    }

    *hg = (uint16_t)HAL_ADC_GetValue(rr2_adc_hg);
    *lg = (uint16_t)HAL_ADC_GetValue(rr2_adc_lg);

    HAL_ADC_Stop(rr2_adc_hg);
    HAL_ADC_Stop(rr2_adc_lg);
    return RR2_OK;
}

/* ------------------------------------------------------------------ */
/* Public entry points                                                 */
/* ------------------------------------------------------------------ */
void RR2_DAQ_Init(ADC_HandleTypeDef *adc_hg, ADC_HandleTypeDef *adc_lg)
{
    rr2_adc_hg = adc_hg;
    rr2_adc_lg = adc_lg;
    rr2_seq    = 0u;
    RR2_DWT_Enable();

    /* Park the readout lines in their idle state. */
    HAL_GPIO_WritePin(CK_READ_GPIO_Port,   CK_READ_Pin,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RSTN_READ_GPIO_Port, RSTN_READ_Pin, GPIO_PIN_SET);
}

RR2_Status RR2_DAQ_ReadWindow(RR2_Event *evt, uint8_t first_ch, uint8_t count)
{
    if (evt == NULL) return RR2_ERR_DATA;
    if ((first_ch >= RR2_NUM_CHANNELS) ||
        ((uint16_t)first_ch + count > RR2_NUM_CHANNELS)) {
        return RR2_ERR_DATA;
    }

    evt->first_ch = first_ch;
    evt->count    = count;
    evt->seq      = ++rr2_seq;

    RR2_DAQ_ResetReadPointer();

    /* Fast-forward: advance past the channels we do not digitise.
       No settling wait is needed because nothing is sampled here.     */
    for (uint8_t i = 0u; i < first_ch; ++i) {
        RR2_DAQ_ClockOnce();
    }

    /* Digitise the requested window. */
    for (uint8_t i = 0u; i < count; ++i) {
        const uint8_t ch = (uint8_t)(first_ch + i);

        RR2_DAQ_ClockOnce();                                  /* select ch */
        RR2_DelayCycles(RR2_NS_TO_CYCLES(RR2_MUX_SETTLE_NS)); /* let it settle */

        RR2_Status st = RR2_DAQ_SampleBothGains(&evt->hg[ch], &evt->lg[ch]);
        if (st != RR2_OK) {
            RR2_DAQ_EndOfReadout();   /* always re-arm the ASIC */
            return st;
        }
    }

    RR2_DAQ_EndOfReadout();
    return RR2_OK;
}

RR2_Status RR2_DAQ_ReadEvent(RR2_Event *evt)
{
    return RR2_DAQ_ReadWindow(evt, 0u, RR2_NUM_CHANNELS);
}
