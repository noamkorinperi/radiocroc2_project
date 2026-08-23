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
 *      channel; OUT_AMUXLG then presents that channel's peak value.
 *      Allow >200 ns of settling before sampling.
 *   5. After channel 63, pulse RESET_N (>20 ns) to clear the peak
 *      detectors and the delay cell, arming the ASIC for the next event.
 *
 * IMPORTANT - CK_READ must be BURST generated, not free-running.
 * A continuously running PWM on CK_READ would keep shifting the read
 * pointer between events, so PA1 is driven as GPIO_Output and pulsed
 * in software.
 *
 * ONE GAIN - only OUT_AMUXLG is instrumented on this board. It lands on
 * PA5 / ADC2_IN5; OUT_AMUXHG is not wired anywhere, and its mux buffer
 * is powered down in Slow Control (EN_aMuxHG = 0). So an event carries
 * exactly one code per channel, all the way out to the host.
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

/* The internal hold delay is NOT a constant: it is delay * 0.85 ns *
   slopeTrim, and both terms are host settable. A fixed wait here was
   sized for the power-on defaults (delay=255, slopeTrim=4 -> 870 ns)
   and quietly became too short the moment anything stretched it -
   PresetCsI asks for delay=255, slopeTrim=15, which is 3251 ns.
   Starting CK_READ before hold is asserted samples peak detectors that
   are still tracking, which smears the photopeak without ever failing.

   So the wait is derived from the shadow instead, at every event. Only
   these two margins are fixed:                                        */
#define RR2_HOLD_DELAY_MIN_NS  1000u   /* floor, if the config says ~0 */
#define RR2_HOLD_DELAY_PAD_NS  1000u   /* pad, on top of a +50% margin */

/* ------------------------------------------------------------------ */
/* One captured event                                                  */
/* ------------------------------------------------------------------ */
typedef struct {
    uint16_t lg[RR2_NUM_CHANNELS];  /* OUT_AMUXLG peak, ADC2_IN5 counts */
    uint8_t  first_ch;              /* first channel stored in lg      */
    uint8_t  count;                 /* how many channels are valid     */
    uint32_t seq;                   /* trigger sequence number         */
} RR2_Event;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/** Bind the ADC and enable the DWT cycle counter used for ns delays.
 *  Call once after MX_ADC2_Init().                                    */
void RR2_DAQ_Init(ADC_HandleTypeDef *adc_lg);

/** 1 if the DWT cycle counter is running and delays are cycle-accurate,
 *  0 if the driver fell back to a coarse software loop. Check this once
 *  after RR2_DAQ_Init(): a 0 means every readout delay overshoots, which
 *  is safe but slower. */
uint8_t RR2_DAQ_IsTimingOk(void);

/** Busy-wait for the ASIC's internal hold delay to elapse. Reads the
 *  currently configured delay, so it stays correct after 'delay' or
 *  'preset csi' change it at runtime.                                 */
void RR2_DAQ_WaitHold(void);

/** The wait RR2_DAQ_WaitHold() will perform, in ns, for the delay
 *  currently held in the Slow Control shadow. Exposed so the host can
 *  see what the readout is actually waiting for.                      */
uint32_t RR2_DAQ_HoldDelayNs(void);

/** Pulse RSTN_READ to rewind the read pointer to before channel 0.    */
void RR2_DAQ_ResetReadPointer(void);

/** Pulse RESET_N to clear the peak detectors and delay cell.
 *  Call once at the end of every readout.                             */
void RR2_DAQ_EndOfReadout(void);

/** Emit one CK_READ pulse, advancing the read register by one channel. */
void RR2_DAQ_ClockOnce(void);

/** Full readout: reset pointer, digitise all 64 channels, reset ASIC. */
RR2_Status RR2_DAQ_ReadEvent(RR2_Event *evt);

/** Partial readout. Channels before @p first_ch are fast-forwarded
 *  without settling or sampling, which is much quicker when only a
 *  known sub-range is instrumented (Example 1 "fast forward").        */
RR2_Status RR2_DAQ_ReadWindow(RR2_Event *evt, uint8_t first_ch, uint8_t count);

/** Sample OUT_AMUXLG once, without touching CK_READ. Useful to probe
 *  a single channel or to check the analog path during bring-up.      */
RR2_Status RR2_DAQ_SampleLG(uint16_t *lg);

#endif /* RADIOROC2_DAQ_H */
