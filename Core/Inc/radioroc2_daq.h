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
 * pointer between events, so PA2 is driven as GPIO_Output and pulsed
 * in software.
 *
 * TWO GAINS, ONE SWITCH - both mux outputs are wired to an ADC:
 * OUT_AMUXLG on PA5 / ADC2_IN5 and OUT_AMUXHG on PA4 / ADC1_IN4. LG is
 * always digitised; HG is opt-in at runtime through the 'hg' host
 * command, which powers the HG mux buffer in Slow Control AND flips
 * this driver into sampling both pins per channel. The default stays
 * LG-only, so a host that never asks sees exactly the old stream. When
 * HG is on, the two conversions run in parallel on their separate
 * ADCs - see RR2_DAQ_SamplePair() - and the event grows by an appended
 * HG block on the wire, not a new frame type (usb_stream.h).
 ******************************************************************************
 */
#ifndef RADIOROC2_DAQ_H
#define RADIOROC2_DAQ_H

#include "radioroc2_regs.h"   /* also provides RR2_NUM_CHANNELS */

/* ------------------------------------------------------------------ */
/* Timing constants (ns). Derived from the datasheet, tune on hardware. */
/*                                                                      */
/* REQUESTED, NOT DELIVERED. Measured on the bench 2026-08-26: at -O0   */
/* one HAL_GPIO_WritePin costs ~245 ns, which swamps the 100 ns below,  */
/* so CK_READ really runs at ~732 kHz - a 1365 ns period, roughly 680   */
/* ns per half. That is safe: every constraint here is a MINIMUM and a  */
/* longer pulse only costs dead time. It is not the bottleneck either:  */
/* the four-channel window clocks out in ~5.5 us against ~24 us of ADC  */
/* conversions in the same event.                                       */
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
    uint16_t hg[RR2_NUM_CHANNELS];  /* OUT_AMUXHG peak, ADC1_IN4 counts */
    uint8_t  first_ch;              /* first channel stored in lg/hg   */
    uint8_t  count;                 /* how many channels are valid     */
    uint8_t  has_hg;                /* 1 when hg[] was digitised too   */
    uint32_t seq;                   /* trigger sequence number         */
} RR2_Event;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/** Bind the ADCs and enable the DWT cycle counter used for ns delays.
 *  Call once after the MX_ADCx_Init() calls. adc_hg may be NULL: the
 *  HG switch then refuses to turn on and everything else behaves
 *  exactly as the LG-only build did.                                  */
void RR2_DAQ_Init(ADC_HandleTypeDef *adc_lg, ADC_HandleTypeDef *adc_hg);

/** Digitise OUT_AMUXHG alongside OUT_AMUXLG on every event (1), or go
 *  back to LG only (0). Refuses to enable when no HG ADC was bound at
 *  init - accepting would stream events whose HG block never converted.
 *  This is the DAQ half of the 'hg' command; the ASIC half, powering
 *  the HG mux buffer, is RR2_Ctrl_SetAnalogMuxHG().                   */
RR2_Status RR2_DAQ_SetHG(uint8_t on);

/** Current state of the HG readout switch.                            */
uint8_t RR2_DAQ_GetHG(void);

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

/** Sample OUT_AMUXLG and OUT_AMUXHG once, without touching CK_READ.
 *  The two conversions overlap - LG on ADC2, HG on ADC1 - so a pair
 *  costs the same wall time as one gain alone.                        */
RR2_Status RR2_DAQ_SamplePair(uint16_t *lg, uint16_t *hg);

#endif /* RADIOROC2_DAQ_H */
