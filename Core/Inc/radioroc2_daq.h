/**
 ******************************************************************************
 * @file    radioroc2_daq.h
 * @brief   RADIOROC2 - Stage 2 DAQ: analog-mux readout of the 64 channels.
 *
 * Readout sequence (datasheet Figure 25 / 26, Example 1):
 *
 *   1. An acquisition trigger fires inside the ASIC. NOR_T1OC goes low,
 *      which we catch on EXTI0.
 *   2. The internal delay cell asserts "hold", freezing the peak
 *      detectors. Wait out that delay before touching CK_READ.
 *   3. Pulse RSTN_READ low to reset the read pointer.
 *   4. Each RISING edge of CK_READ advances the read register by one
 *      channel; OUT_AMUXHG / OUT_AMUXLG then present that channel's
 *      peak value. Allow >200 ns of settling before sampling.
 *   5. After channel 63, pulse RESET_N (>20 ns) to clear the peak
 *      detectors and the delay cell, arming the ASIC for the next event.
 *
 * IMPORTANT - CK_READ must be BURST generated, not free-running.
 * A continuously running PWM on CK_READ would keep shifting the read
 * pointer between events, so PA1 is driven as GPIO_Output and pulsed
 * in software. See PROJECT_DOCUMENTATION section 3 for the rationale.
 ******************************************************************************
 */
#ifndef RADIOROC2_DAQ_H
#define RADIOROC2_DAQ_H

#include "radioroc2_regs.h"   /* also provides RR2_NUM_CHANNELS */

/* ------------------------------------------------------------------ */
/* Timing constants (ns). Derived from the datasheet, tune on hardware. */
/* ------------------------------------------------------------------ */
#define RR2_CKREAD_HIGH_NS      100u   /* CK_READ high time            */
#define RR2_CKREAD_LOW_NS       100u   /* CK_READ low time             */
#define RR2_MUX_SETTLE_NS       250u   /* datasheet asks for >200 ns   */
#define RR2_RSTN_READ_NS        100u   /* datasheet asks for >20 ns    */
#define RR2_RESET_N_NS          100u   /* datasheet asks for ~20 ns    */

/* Worst-case internal hold delay: delay(0xFF) * 0.85 ns * slopeTrim(4)
   ~= 870 ns with our defaults. 3 us gives comfortable margin.         */
#define RR2_HOLD_DELAY_NS      3000u

/* ------------------------------------------------------------------ */
/* Channel selection                                                   */
/* ------------------------------------------------------------------ */
/* Which of the 64 inputs actually carry a detector. Bit N = channel N.
 *
 * The sensors are modular rather than soldered down, so which inputs
 * are populated changes between setups and the selection has to be a
 * runtime value, not a compile-time one. It is also NOT assumed to be
 * contiguous: the readout clocks past unselected channels without
 * sampling them, which costs a couple of hundred nanoseconds each
 * instead of the ~6 us an ADC conversion takes.
 *
 * Default is all 64, so out of the box the firmware behaves exactly as
 * it did before any selection is made.                                */
#define RR2_MASK_ALL      (~(uint64_t)0)
#define RR2_MASK_NONE     ((uint64_t)0)
#define RR2_MASK_CH(n)    ((uint64_t)1u << (n))

/* ------------------------------------------------------------------ */
/* One captured event                                                  */
/* ------------------------------------------------------------------ */
typedef struct {
    /* Indexed by ABSOLUTE channel number, so hg[7] is always channel 7
       whatever the mask happens to be. Only the channels set in mask
       hold meaningful data; the rest keep whatever they had.          */
    uint16_t hg[RR2_NUM_CHANNELS];  /* High Gain peak, ADC1_IN4 counts */
    uint16_t lg[RR2_NUM_CHANNELS];  /* Low  Gain peak, ADC2_IN5 counts */
    uint64_t mask;                  /* which channels are valid        */
    uint32_t seq;                   /* trigger sequence number         */
} RR2_Event;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/** Bind the ADCs and enable the DWT cycle counter used for ns delays.
 *  Call once after MX_ADC1_Init() / MX_ADC2_Init().                   */
void RR2_DAQ_Init(ADC_HandleTypeDef *adc_hg, ADC_HandleTypeDef *adc_lg);

/** 1 if the DWT cycle counter is running and delays are cycle-accurate,
 *  0 if the driver fell back to a coarse software loop. Check this once
 *  after RR2_DAQ_Init(): a 0 means every readout delay overshoots, which
 *  is safe but slower. */
uint8_t RR2_DAQ_IsTimingOk(void);

/** Busy-wait for the ASIC's internal hold delay to elapse.            */
void RR2_DAQ_WaitHold(void);

/** Pulse RSTN_READ to rewind the read pointer to before channel 0.    */
void RR2_DAQ_ResetReadPointer(void);

/** Pulse RESET_N to clear the peak detectors and delay cell.
 *  Call once at the end of every readout.                             */
void RR2_DAQ_EndOfReadout(void);

/** Emit one CK_READ pulse, advancing the read register by one channel. */
void RR2_DAQ_ClockOnce(void);

/* ------------------------------------------------------------------ */
/* Channel selection                                                   */
/* ------------------------------------------------------------------ */

/** Choose which channels to digitise. Takes effect on the next event. */
void RR2_DAQ_SetChannelMask(uint64_t mask);

/** The mask currently in force. */
uint64_t RR2_DAQ_GetChannelMask(void);

/** How many channels the current mask selects. */
uint8_t RR2_DAQ_GetChannelCount(void);

/**
 * @brief Build a mask from an explicit list of channel numbers.
 *
 * The readable way to say "my five detectors are on 3, 9, 20, 41, 55":
 *
 *      static const uint8_t mine[] = { 3, 9, 20, 41, 55 };
 *      RR2_DAQ_SelectChannels(mine, 5);
 *
 * @retval RR2_OK, or RR2_ERR_DATA if any entry is >= RR2_NUM_CHANNELS.
 *         On error the current mask is left untouched.
 */
RR2_Status RR2_DAQ_SelectChannels(const uint8_t *list, uint8_t n);

/* ------------------------------------------------------------------ */
/* Readout                                                             */
/* ------------------------------------------------------------------ */

/**
 * @brief Digitise one event, sampling only the channels in the mask.
 *
 * Walks the read register from channel 0 up to the highest selected
 * channel, clocking past the ones that are not selected and stopping
 * as soon as the last selected one has been read. With five scattered
 * detectors that is 64 clock pulses but only five pairs of ADC
 * conversions - roughly 50 us instead of the 400-500 us a full sweep
 * costs, because the conversions dominate, not the clocking.
 */
RR2_Status RR2_DAQ_ReadEvent(RR2_Event *evt);

/** Sample both gains once, without touching CK_READ. Useful to probe
 *  a single channel or to check the analog path during bring-up.      */
RR2_Status RR2_DAQ_SampleBothGains(uint16_t *hg, uint16_t *lg);

#endif /* RADIOROC2_DAQ_H */
