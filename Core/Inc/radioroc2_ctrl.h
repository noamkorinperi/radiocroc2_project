/**
 ******************************************************************************
 * @file    radioroc2_ctrl.h
 * @brief   RADIOROC2 runtime control with a RAM shadow of the Slow Control.
 *
 * WHY A SHADOW
 * A Slow Control write always replaces a whole byte, and most bytes pack
 * several unrelated fields. Changing one enable bit with a plain write
 * would silently clobber the other seven. Reading the byte back first
 * would work, but the read path is not verified on hardware yet.
 *
 * So the driver keeps its own copy of every register it manages. A setter
 * edits the shadow, then pushes exactly the byte(s) that changed. This is
 * true read-modify-write semantics without depending on the ASIC's read
 * path, and it also lets the host dump the current configuration.
 *
 * SCOPE
 * Only the parameters that matter for a CsI(Tl) + SiPM spectroscopy setup
 * are exposed. Deliberately NOT covered:
 *   - per-channel outing routing (address 66, subadd 0..63): the 128
 *     direct outputs are unused, readout goes through the analog mux
 *   - probe switches: debug only
 *   - bias currents (address 64, subadd 0..12): left at silicon defaults
 *   - extended output / CLPS driver settings: differential outputs unused
 ******************************************************************************
 */
#ifndef RADIOROC2_CTRL_H
#define RADIOROC2_CTRL_H

#include "radioroc2_regs.h"

#define RR2_CH_ALL      0xFFu   /* broadcast sentinel for the ch argument */

/* Sentinel for the hg half of the paired setters below: leave that field
   at whatever the shadow already holds. The host passes it whenever the
   hg argument is omitted, so a control that is no longer offered cannot
   silently rewrite the HG chain behind the back of the operator.      */
#define RR2_KEEP        0xFFu

/* ------------------------------------------------------------------ */
/* Shadow image                                                        */
/* ------------------------------------------------------------------ */
typedef struct {
    /* Address 0..63, subadd 0..7 */
    uint8_t ch[RR2_NUM_CHANNELS][8];

    /* Address 65 - common blocks (only the bytes we manage) */
    uint8_t com_dac1_lo;      /* subadd 1  */
    uint8_t com_dac2_dac1;    /* subadd 2  */
    uint8_t com_dacq_dac2;    /* subadd 3  */
    uint8_t com_dacq_hi;      /* subadd 4  */
    uint8_t com_vref_en;      /* subadd 7  */
    uint8_t com_delay;        /* subadd 8  */
    uint8_t com_slope;        /* subadd 9  */
    uint8_t com_hyst_trig;    /* subadd 12 */

    /* Address 66 - outing power */
    uint8_t out_power;        /* subadd 70 */

    /* Address 64 - block power on/off */
    uint8_t bias_on1;         /* subadd 13 */
    uint8_t bias_on2;         /* subadd 14 */
} RR2_Shadow;

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

/** Load the datasheet power-on defaults into the shadow. No I2C. */
void RR2_Ctrl_ResetShadow(void);

/** Write the entire shadow to the ASIC. ~0.4 s for all 64 channels. */
RR2_Status RR2_Ctrl_PushAll(void);

/** Write only the global blocks (fast, no channel loop). */
RR2_Status RR2_Ctrl_PushGlobal(void);

/** Write one channel's eight sub-addresses. */
RR2_Status RR2_Ctrl_PushChannel(uint8_t ch);

/** Read-only view of the shadow, for dumping to the host. */
const RR2_Shadow *RR2_Ctrl_GetShadow(void);

/* ------------------------------------------------------------------ */
/* Per-channel setters. Pass RR2_CH_ALL to apply to every channel.     */
/* Each one edits the shadow and immediately pushes the changed byte.  */
/* ------------------------------------------------------------------ */

/** Input DAC, 0..255. Trims the SiPM overvoltage channel by channel -
 *  this is the knob for equalising gain across an array.             */
RR2_Status RR2_Ctrl_SetInDac(uint8_t ch, uint8_t value);

/** Charge preamp gains. lg 0..15 (span 0.5-8), hg 0..15 (span 5-80).
 *  Only the LG path is digitised - see the note on SetAnalogMux - so hg
 *  is kept for the charge trigger, which still runs off the HG chain.
 *  Pass RR2_KEEP for either half to leave it exactly as it is.       */
RR2_Status RR2_Ctrl_SetChargeGain(uint8_t ch, uint8_t lg, uint8_t hg);

/** Shaper peaking time index, 0..15 for each gain path, or RR2_KEEP.
 *  Step is 20 ns normally, 120 ns when slow shaping is enabled.      */
RR2_Status RR2_Ctrl_SetShapingTime(uint8_t ch, uint8_t tau_lg, uint8_t tau_hg);

/** Slow shaping toggles. 0 -> 20 ns steps (max 300 ns),
 *  1 -> 120 ns steps (max 1.8 us), RR2_KEEP -> unchanged. Needed for
 *  slow scintillators.                                               */
RR2_Status RR2_Ctrl_SetSlowShaping(uint8_t ch, uint8_t slow_lg, uint8_t slow_hg);

/** Time preamp gain, 0..63 (closed-loop gain 15..100).               */
RR2_Status RR2_Ctrl_SetPatGain(uint8_t ch, uint8_t gain);

/** Per-channel threshold trims, 0..63 each (0-15 mV below the global
 *  threshold). Used to equalise trigger points across channels.      */
RR2_Status RR2_Ctrl_SetThresholdTrim(uint8_t ch, uint8_t trim1, uint8_t trim2);

/** Enable or disable a whole channel: preamps, discriminators,
 *  shapers and peak detectors together. Disable unused channels so
 *  they stop contributing to the NOR triggers.                       */
RR2_Status RR2_Ctrl_SetChannelEnabled(uint8_t ch, uint8_t enable);

/** The three discriminators of a channel, without touching anything
 *  else it does. A channel with them off still shapes its input and
 *  still presents a peak-detector code to the readout - it simply
 *  cannot fire the NOR trigger.
 *
 *  That is what a baseline reference channel wants: an unconnected
 *  input is floating, and a spurious hit there raises the same trigger
 *  as a real gamma, which looks exactly like a detector that ignores
 *  its source.                                                       */
RR2_Status RR2_Ctrl_SetDiscriminators(uint8_t ch, uint8_t enable);

/** Internal charge injection, for exercising the analog chain with no
 *  radiation source. use_ctest selects the 1.5 pF test capacitor.    */
RR2_Status RR2_Ctrl_SetChargeInjection(uint8_t ch, uint8_t enable, uint8_t use_ctest);

/* ------------------------------------------------------------------ */
/* Global setters (address 64 / 65 / 66)                               */
/* ------------------------------------------------------------------ */

/** The three 10-bit threshold DACs (0..1023).
 *  dac1 = low time trigger, dac2 = high time trigger, dacq = charge. */
RR2_Status RR2_Ctrl_SetThresholds(uint16_t dac1, uint16_t dac2, uint16_t dacq);

/** Peak-detector hold delay. total = delay * 0.85 ns * slope_trim.
 *  delay 0..255, slope_trim 0..15. Set this to land the hold near the
 *  shaper's peak, i.e. close to the peaking time.                    */
RR2_Status RR2_Ctrl_SetHoldDelay(uint8_t delay, uint8_t slope_trim);

/** Acquisition trigger source, selTrig[3:0]. Use the RR2_SELTRIG_*
 *  constants from radioroc2_regs.h.                                  */
RR2_Status RR2_Ctrl_SetTriggerSource(uint8_t sel_trig);

/** Hold source: 0 = internal delay cell, 1 = external HOLDEXT pin.   */
RR2_Status RR2_Ctrl_SetHoldExternal(uint8_t external);

/** Power the LG analog multiplexer buffer. It must be on or the ADC
 *  reads nothing from OUT_AMUXLG.
 *
 *  There is no HG argument on purpose: OUT_AMUXHG goes nowhere on this
 *  board, so its buffer is always written off. The HG chain inside the
 *  ASIC stays powered - the charge trigger is derived from it.       */
RR2_Status RR2_Ctrl_SetAnalogMux(uint8_t enable);

/* ------------------------------------------------------------------ */
/* Convenience preset                                                  */
/* ------------------------------------------------------------------ */

/** Starting point for CsI(Tl), whose light decay is around 1 us:
 *  slow shaping on, long peaking time, and a hold delay stretched to
 *  match. This is a STARTING POINT, not a calibrated configuration -
 *  scan the delay while watching the pulse height to refine it.      */
RR2_Status RR2_Ctrl_PresetCsI(void);

#endif /* RADIOROC2_CTRL_H */
