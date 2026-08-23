/**
 ******************************************************************************
 * @file    radioroc2_config.c
 * @brief   RADIOROC2 - high-level Slow Control configuration sequences.
 *
 * Brings the ASIC into a reproducible baseline for charge/energy readout
 * via the OUT_AMUXLG analog multiplexer output, matching the
 * datasheet "Example 1: default configuration":
 *      - all analog blocks powered / enabled
 *      - shapers + peak detectors on
 *      - LG analog-mux buffer on (so the ADC on PA5 sees a signal)
 *      - acquisition trigger = global Low-threshold Time Trigger (selTrig=0100)
 *
 * The ASIC already powers up at these defaults (Table 4), but writing them
 * explicitly makes the state reproducible and independent of power-on order.
 ******************************************************************************
 */
#include "radioroc2_regs.h"

/* ------------------------------------------------------------------ */
/* One channel -> datasheet defaults (subadd 0..7)                     */
/* ------------------------------------------------------------------ */
RR2_Status RR2_ConfigChannel(uint8_t ch)
{
    const uint8_t a = RR2_ADDR_CH(ch);
    RR2_Status st;

    if ((st = RR2_Write(a, RR2_CH_SUB_INDAC,   RR2_CH_INDAC_DEFAULT)) != RR2_OK) return st;
    if ((st = RR2_Write(a, RR2_CH_SUB_PAT,     RR2_CH_PAT_DEFAULT))   != RR2_OK) return st;
    if ((st = RR2_Write(a, RR2_CH_SUB_GAIN,    RR2_CH_GAIN_DEFAULT))  != RR2_OK) return st;
    if ((st = RR2_Write(a, RR2_CH_SUB_TAU,     RR2_CH_TAU_DEFAULT))   != RR2_OK) return st;
    if ((st = RR2_Write(a, RR2_CH_SUB_CALIBT1, 0x00u))                != RR2_OK) return st;
    if ((st = RR2_Write(a, RR2_CH_SUB_CALIBT2, 0x00u))                != RR2_OK) return st;
    if ((st = RR2_Write(a, RR2_CH_SUB_EN,      RR2_CH_EN_DEFAULT))    != RR2_OK) return st;
    if ((st = RR2_Write(a, RR2_CH_SUB_EN2,     RR2_CH_EN2_DEFAULT))   != RR2_OK) return st;
    return RR2_OK;
}

RR2_Status RR2_ConfigAllChannels(void)
{
    for (uint8_t ch = 0u; ch < 64u; ++ch) {
        RR2_Status st = RR2_ConfigChannel(ch);
        if (st != RR2_OK) return st;   /* stop on first failing channel */
    }
    return RR2_OK;
}

RR2_Status RR2_WriteAllChannels(uint8_t subadd, uint8_t data)
{
    for (uint8_t ch = 0u; ch < 64u; ++ch) {
        RR2_Status st = RR2_Write(RR2_ADDR_CH(ch), subadd, data);
        if (st != RR2_OK) return st;
    }
    return RR2_OK;
}

/* ------------------------------------------------------------------ */
/* Global blocks: biasing (64), common (65), outing power (66),        */
/* event gating (67)                                                   */
/* ------------------------------------------------------------------ */
RR2_Status RR2_ConfigCommon(void)
{
    RR2_Status st;
    uint8_t s;

    /* ---- Address 64: ASIC biasing ---- */
    /* subadd 0..12 are all nibble pairs at the 0x44 default */
    for (s = RR2_BIAS_SUB_INDAC0; s <= RR2_BIAS_SUB_DISCRICHG; ++s) {
        if ((st = RR2_Write(RR2_ADDR_BIASING, s, RR2_BIAS_NIBBLE_DEFAULT)) != RR2_OK) return st;
    }
    /* subadd 13/14: power-on (ON_*) bits -> everything enabled */
    if ((st = RR2_Write(RR2_ADDR_BIASING, RR2_BIAS_SUB_ON1, RR2_BIAS_ON1_DEFAULT)) != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_BIASING, RR2_BIAS_SUB_ON2, RR2_BIAS_ON2_DEFAULT)) != RR2_OK) return st;

    /* ---- Address 65: common blocks ---- */
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_BG,        RR2_COM_BG_DEFAULT))        != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_THDAC_BIAS,  RR2_BIAS_NIBBLE_DEFAULT)) != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_THDACQ_BIAS, RR2_BIAS_NIBBLE_DEFAULT)) != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_VREF_EN,   RR2_COM_VREF_EN_DEFAULT))   != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_DELAY,     RR2_COM_DELAY_DEFAULT))     != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_SLOPE,     RR2_BIAS_NIBBLE_DEFAULT))   != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_DISCRIDLY, RR2_BIAS_NIBBLE_DEFAULT))   != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_DELAYDAC,  RR2_BIAS_NIBBLE_DEFAULT))   != RR2_OK) return st;
    /* selTrig=0100 (global T1), hysteresis on, delay cell on */
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_HYST_TRIG, RR2_COM_HYST_TRIG_DEFAULT)) != RR2_OK) return st;

    /* ---- Address 66: outing ---- power the LG analog-mux buffer.
       Without this the ADC on PA5 reads nothing meaningful.           */
    if ((st = RR2_Write(RR2_ADDR_OUTING, RR2_OUT_SUB_POWER, RR2_OUT_POWER_DEFAULT)) != RR2_OK) return st;

    /* ---- Address 67: event validation gating ---- */
    if ((st = RR2_Write(RR2_ADDR_EVENTGATE, RR2_GATE_SUB_RX, RR2_GATE_RX_DEFAULT)) != RR2_OK) return st;

    return RR2_OK;
}

/* ------------------------------------------------------------------ */
/* Full baseline configuration                                         */
/* ------------------------------------------------------------------ */
RR2_Status RR2_ConfigDefault(uint16_t th_low, uint16_t th_high, uint16_t th_charge)
{
    RR2_Status st;

    if ((st = RR2_ConfigCommon())      != RR2_OK) return st;
    if ((st = RR2_ConfigAllChannels()) != RR2_OK) return st;

    /* Threshold DACs are detector-dependent - caller decides the codes. */
    if ((st = RR2_SetTimeThresholds(th_low, th_high, th_charge)) != RR2_OK) return st;

    return RR2_OK;
}
