/**
 ******************************************************************************
 * @file    radioroc2_ctrl.c
 * @brief   RADIOROC2 runtime control with a RAM shadow of the Slow Control.
 * @note    See radioroc2_ctrl.h for the rationale and the covered scope.
 ******************************************************************************
 */
#include "radioroc2_ctrl.h"
#include <stddef.h>

static RR2_Shadow sh;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

/** Push one shadowed channel byte to the ASIC. */
static RR2_Status push_ch_sub(uint8_t ch, uint8_t sub)
{
    return RR2_Write(RR2_ADDR_CH(ch), sub, sh.ch[ch][sub]);
}

/** Apply a per-channel edit to one channel or to all of them.
 *  @param edit  callback that mutates sh.ch[ch][...]                 */
static RR2_Status for_each_channel(uint8_t ch, uint8_t sub,
                                   void (*edit)(uint8_t ch, uint32_t a, uint32_t b),
                                   uint32_t a, uint32_t b)
{
    if (ch == RR2_CH_ALL) {
        for (uint8_t c = 0u; c < RR2_NUM_CHANNELS; ++c) {
            edit(c, a, b);
            RR2_Status st = push_ch_sub(c, sub);
            if (st != RR2_OK) return st;
        }
        return RR2_OK;
    }

    if (ch >= RR2_NUM_CHANNELS) return RR2_ERR_DATA;
    edit(ch, a, b);
    return push_ch_sub(ch, sub);
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */
void RR2_Ctrl_ResetShadow(void)
{
    for (uint8_t c = 0u; c < RR2_NUM_CHANNELS; ++c) {
        sh.ch[c][RR2_CH_SUB_INDAC]   = RR2_CH_INDAC_DEFAULT;  /* 0x80 */
        sh.ch[c][RR2_CH_SUB_PAT]     = RR2_CH_PAT_DEFAULT;    /* 0xA0 */
        sh.ch[c][RR2_CH_SUB_GAIN]    = RR2_CH_GAIN_DEFAULT;   /* 0x44 */
        sh.ch[c][RR2_CH_SUB_TAU]     = RR2_CH_TAU_DEFAULT;    /* 0x11 */
        sh.ch[c][RR2_CH_SUB_CALIBT1] = 0x00u;
        sh.ch[c][RR2_CH_SUB_CALIBT2] = 0x00u;
        sh.ch[c][RR2_CH_SUB_EN]      = RR2_CH_EN_DEFAULT;     /* 0x7F */
        sh.ch[c][RR2_CH_SUB_EN2]     = RR2_CH_EN2_DEFAULT;    /* 0x0F */
    }

    sh.com_dac1_lo   = 0x00u;
    sh.com_dac2_dac1 = 0x00u;
    sh.com_dacq_dac2 = 0x00u;
    sh.com_dacq_hi   = 0x00u;
    sh.com_vref_en   = RR2_COM_VREF_EN_DEFAULT;    /* 0x8F */
    sh.com_delay     = RR2_COM_DELAY_DEFAULT;      /* 0xFF */
    sh.com_slope     = RR2_BIAS_NIBBLE_DEFAULT;    /* 0x44 */
    sh.com_hyst_trig = RR2_COM_HYST_TRIG_DEFAULT;  /* 0xE4 */

    sh.out_power     = RR2_OUT_POWER_DEFAULT;      /* 0x09 */
    sh.bias_on1      = RR2_BIAS_ON1_DEFAULT;       /* 0xFF */
    sh.bias_on2      = RR2_BIAS_ON2_DEFAULT;       /* 0xFF */
}

RR2_Status RR2_Ctrl_PushChannel(uint8_t ch)
{
    if (ch >= RR2_NUM_CHANNELS) return RR2_ERR_DATA;

    for (uint8_t sub = 0u; sub < 8u; ++sub) {
        RR2_Status st = push_ch_sub(ch, sub);
        if (st != RR2_OK) return st;
    }
    return RR2_OK;
}

RR2_Status RR2_Ctrl_PushGlobal(void)
{
    RR2_Status st;

    /* Address 64 - block power */
    if ((st = RR2_Write(RR2_ADDR_BIASING, RR2_BIAS_SUB_ON1, sh.bias_on1)) != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_BIASING, RR2_BIAS_SUB_ON2, sh.bias_on2)) != RR2_OK) return st;

    /* Address 65 - thresholds, references, delay, trigger */
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_DAC1_LO,   sh.com_dac1_lo))   != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_DAC2_DAC1, sh.com_dac2_dac1)) != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_DACQ_DAC2, sh.com_dacq_dac2)) != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_DACQ_HI,   sh.com_dacq_hi))   != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_VREF_EN,   sh.com_vref_en))   != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_DELAY,     sh.com_delay))     != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_SLOPE,     sh.com_slope))     != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_HYST_TRIG, sh.com_hyst_trig)) != RR2_OK) return st;

    /* Address 66 - analog mux / buffer power */
    if ((st = RR2_Write(RR2_ADDR_OUTING, RR2_OUT_SUB_POWER, sh.out_power)) != RR2_OK) return st;

    return RR2_OK;
}

RR2_Status RR2_Ctrl_PushAll(void)
{
    RR2_Status st = RR2_Ctrl_PushGlobal();
    if (st != RR2_OK) return st;

    for (uint8_t c = 0u; c < RR2_NUM_CHANNELS; ++c) {
        st = RR2_Ctrl_PushChannel(c);
        if (st != RR2_OK) return st;
    }
    return RR2_OK;
}

const RR2_Shadow *RR2_Ctrl_GetShadow(void)
{
    return &sh;
}

/* ------------------------------------------------------------------ */
/* Per-channel edit callbacks                                          */
/* ------------------------------------------------------------------ */
static void ed_indac(uint8_t c, uint32_t v, uint32_t unused)
{
    (void)unused;
    sh.ch[c][RR2_CH_SUB_INDAC] = (uint8_t)v;
}

/* RR2_KEEP in either half means "read that field back out of the
   shadow", which is what lets a caller set one gain path without
   touching the other. Resolved per channel, so it stays correct under
   RR2_CH_ALL even if the channels have drifted apart.                */
static void ed_gain(uint8_t c, uint32_t lg, uint32_t hg)
{
    const uint8_t cur = sh.ch[c][RR2_CH_SUB_GAIN];

    if (lg == RR2_KEEP) lg = RR2_GET(RR2_LGGAIN, cur);
    if (hg == RR2_KEEP) hg = RR2_GET(RR2_HGGAIN, cur);

    sh.ch[c][RR2_CH_SUB_GAIN] = RR2_NIBBLES((uint8_t)lg, (uint8_t)hg);
}

static void ed_tau(uint8_t c, uint32_t lg, uint32_t hg)
{
    /* Datasheet naming quirk: bits [7:4] are tauLG, [3:0] are tauHG. */
    const uint8_t cur = sh.ch[c][RR2_CH_SUB_TAU];

    if (lg == RR2_KEEP) lg = RR2_GET(RR2_TAULG, cur);
    if (hg == RR2_KEEP) hg = RR2_GET(RR2_TAUHG, cur);

    sh.ch[c][RR2_CH_SUB_TAU] = RR2_NIBBLES((uint8_t)lg, (uint8_t)hg);
}

static void ed_slow(uint8_t c, uint32_t lg, uint32_t hg)
{
    uint8_t b = sh.ch[c][RR2_CH_SUB_EN2];

    if (lg == RR2_KEEP) lg = (uint32_t)((b >> RR2_SLOWSHAPINGLG_Pos) & 1u);
    if (hg == RR2_KEEP) hg = (uint32_t)((b >> RR2_SLOWSHAPINGHG_Pos) & 1u);

    b &= (uint8_t)~((1u << RR2_SLOWSHAPINGLG_Pos) | (1u << RR2_SLOWSHAPINGHG_Pos));
    if (lg) b |= (uint8_t)(1u << RR2_SLOWSHAPINGLG_Pos);
    if (hg) b |= (uint8_t)(1u << RR2_SLOWSHAPINGHG_Pos);
    sh.ch[c][RR2_CH_SUB_EN2] = b;
}

static void ed_patgain(uint8_t c, uint32_t g, uint32_t unused)
{
    (void)unused;
    uint8_t b = sh.ch[c][RR2_CH_SUB_PAT];
    b &= (uint8_t)~RR2_PATGAIN_Msk;          /* keep patComp untouched */
    b |= RR2_SET(RR2_PATGAIN, (uint8_t)g);
    sh.ch[c][RR2_CH_SUB_PAT] = b;
}

static void ed_trim1(uint8_t c, uint32_t t, uint32_t unused)
{
    (void)unused;
    sh.ch[c][RR2_CH_SUB_CALIBT1] = RR2_SET(RR2_CALIBT1, (uint8_t)t);
}

static void ed_trim2(uint8_t c, uint32_t t, uint32_t unused)
{
    (void)unused;
    sh.ch[c][RR2_CH_SUB_CALIBT2] = RR2_SET(RR2_CALIBT2, (uint8_t)t);
}

static void ed_enable6(uint8_t c, uint32_t en, uint32_t unused)
{
    (void)unused;
    /* subadd 6: input DAC, time preamp, both discriminators, both
       charge preamps. DAC_select stays at its shadowed value.        */
    const uint8_t mask = (uint8_t)((1u << RR2_EN_INDAC_Pos) |
                                   (1u << RR2_EN_PAT_Pos) |
                                   (1u << RR2_EN_DISCRI1_Pos) |
                                   (1u << RR2_EN_DISCRI2_Pos) |
                                   (1u << RR2_EN_DISCRICHARGE_Pos) |
                                   (1u << RR2_EN_PALG_Pos) |
                                   (1u << RR2_EN_PAHG_Pos));
    if (en) sh.ch[c][RR2_CH_SUB_EN] |=  mask;
    else    sh.ch[c][RR2_CH_SUB_EN] &= (uint8_t)~mask;
}

static void ed_enable7(uint8_t c, uint32_t en, uint32_t unused)
{
    (void)unused;
    /* subadd 7: shapers and peak detectors. slowShaping bits are left
       alone so enabling a channel never changes its timing.          */
    const uint8_t mask = (uint8_t)((1u << RR2_EN_SHLG_Pos) |
                                   (1u << RR2_EN_SHHG_Pos) |
                                   (1u << RR2_EN_PDETLG_Pos) |
                                   (1u << RR2_EN_PDETHG_Pos));
    if (en) sh.ch[c][RR2_CH_SUB_EN2] |=  mask;
    else    sh.ch[c][RR2_CH_SUB_EN2] &= (uint8_t)~mask;
}

static void ed_discri(uint8_t c, uint32_t en, uint32_t unused)
{
    (void)unused;
    /* subadd 6: the three discriminators only. Preamps, input DAC and
       time preamp are left alone, so a channel can keep producing a
       shaped baseline while being unable to fire a trigger.          */
    const uint8_t mask = (uint8_t)((1u << RR2_EN_DISCRI1_Pos) |
                                   (1u << RR2_EN_DISCRI2_Pos) |
                                   (1u << RR2_EN_DISCRICHARGE_Pos));
    if (en) sh.ch[c][RR2_CH_SUB_EN] |=  mask;
    else    sh.ch[c][RR2_CH_SUB_EN] &= (uint8_t)~mask;
}

static void ed_ctest(uint8_t c, uint32_t en, uint32_t use_ctest)
{
    uint8_t b = sh.ch[c][RR2_CH_SUB_EN2];
    b &= (uint8_t)~((1u << RR2_EN_TEST_Pos) | (1u << RR2_USECTEST_Pos));
    if (en)        b |= (uint8_t)(1u << RR2_EN_TEST_Pos);
    if (use_ctest) b |= (uint8_t)(1u << RR2_USECTEST_Pos);
    sh.ch[c][RR2_CH_SUB_EN2] = b;
}

/* ------------------------------------------------------------------ */
/* Per-channel public setters                                          */
/* ------------------------------------------------------------------ */
RR2_Status RR2_Ctrl_SetInDac(uint8_t ch, uint8_t value)
{
    return for_each_channel(ch, RR2_CH_SUB_INDAC, ed_indac, value, 0u);
}

RR2_Status RR2_Ctrl_SetChargeGain(uint8_t ch, uint8_t lg, uint8_t hg)
{
    if (((lg > 15u) && (lg != RR2_KEEP)) ||
        ((hg > 15u) && (hg != RR2_KEEP))) return RR2_ERR_DATA;
    return for_each_channel(ch, RR2_CH_SUB_GAIN, ed_gain, lg, hg);
}

RR2_Status RR2_Ctrl_SetShapingTime(uint8_t ch, uint8_t tau_lg, uint8_t tau_hg)
{
    if (((tau_lg > 15u) && (tau_lg != RR2_KEEP)) ||
        ((tau_hg > 15u) && (tau_hg != RR2_KEEP))) return RR2_ERR_DATA;
    return for_each_channel(ch, RR2_CH_SUB_TAU, ed_tau, tau_lg, tau_hg);
}

RR2_Status RR2_Ctrl_SetSlowShaping(uint8_t ch, uint8_t slow_lg, uint8_t slow_hg)
{
    return for_each_channel(ch, RR2_CH_SUB_EN2, ed_slow, slow_lg, slow_hg);
}

RR2_Status RR2_Ctrl_SetPatGain(uint8_t ch, uint8_t gain)
{
    if (gain > 63u) return RR2_ERR_DATA;
    return for_each_channel(ch, RR2_CH_SUB_PAT, ed_patgain, gain, 0u);
}

RR2_Status RR2_Ctrl_SetThresholdTrim(uint8_t ch, uint8_t trim1, uint8_t trim2)
{
    RR2_Status st;

    if ((trim1 > 63u) || (trim2 > 63u)) return RR2_ERR_DATA;

    st = for_each_channel(ch, RR2_CH_SUB_CALIBT1, ed_trim1, trim1, 0u);
    if (st != RR2_OK) return st;

    return for_each_channel(ch, RR2_CH_SUB_CALIBT2, ed_trim2, trim2, 0u);
}

RR2_Status RR2_Ctrl_SetChannelEnabled(uint8_t ch, uint8_t enable)
{
    RR2_Status st = for_each_channel(ch, RR2_CH_SUB_EN, ed_enable6, enable, 0u);
    if (st != RR2_OK) return st;

    return for_each_channel(ch, RR2_CH_SUB_EN2, ed_enable7, enable, 0u);
}

RR2_Status RR2_Ctrl_SetDiscriminators(uint8_t ch, uint8_t enable)
{
    return for_each_channel(ch, RR2_CH_SUB_EN, ed_discri, enable, 0u);
}

RR2_Status RR2_Ctrl_SetChargeInjection(uint8_t ch, uint8_t enable, uint8_t use_ctest)
{
    return for_each_channel(ch, RR2_CH_SUB_EN2, ed_ctest, enable, use_ctest);
}

/* ------------------------------------------------------------------ */
/* Global setters                                                      */
/* ------------------------------------------------------------------ */
RR2_Status RR2_Ctrl_SetThresholds(uint16_t dac1, uint16_t dac2, uint16_t dacq)
{
    RR2_Status st;

    if ((dac1 > 1023u) || (dac2 > 1023u) || (dacq > 1023u)) return RR2_ERR_DATA;

    /* The three 10-bit DACs are interleaved across four sub-addresses,
       so all four bytes are rebuilt together to stay consistent.      */
    sh.com_dac1_lo   = (uint8_t)(dac1 & 0xFFu);
    sh.com_dac2_dac1 = (uint8_t)(((dac2 & 0x3Fu) << 2) | ((dac1 >> 8) & 0x03u));
    sh.com_dacq_dac2 = (uint8_t)(((dacq & 0x0Fu) << 4) | ((dac2 >> 6) & 0x0Fu));
    sh.com_dacq_hi   = (uint8_t)((dacq >> 4) & 0x3Fu);

    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_DAC1_LO,   sh.com_dac1_lo))   != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_DAC2_DAC1, sh.com_dac2_dac1)) != RR2_OK) return st;
    if ((st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_DACQ_DAC2, sh.com_dacq_dac2)) != RR2_OK) return st;
    return RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_DACQ_HI, sh.com_dacq_hi);
}

RR2_Status RR2_Ctrl_SetHoldDelay(uint8_t delay, uint8_t slope_trim)
{
    RR2_Status st;

    if (slope_trim > 15u) return RR2_ERR_DATA;

    sh.com_delay = delay;
    sh.com_slope = (uint8_t)((sh.com_slope & 0x0Fu) |
                             RR2_SET(RR2_SLOPETRIM, slope_trim));

    st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_DELAY, sh.com_delay);
    if (st != RR2_OK) return st;

    return RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_SLOPE, sh.com_slope);
}

RR2_Status RR2_Ctrl_SetTriggerSource(uint8_t sel_trig)
{
    if (sel_trig > 15u) return RR2_ERR_DATA;

    sh.com_hyst_trig = (uint8_t)((sh.com_hyst_trig & (uint8_t)~RR2_SELTRIG_Msk) |
                                 RR2_SET(RR2_SELTRIG, sel_trig));

    return RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_HYST_TRIG, sh.com_hyst_trig);
}

RR2_Status RR2_Ctrl_SetHoldExternal(uint8_t external)
{
    if (external) sh.com_hyst_trig |=  (uint8_t)(1u << RR2_SELHOLDEXT_Pos);
    else          sh.com_hyst_trig &= (uint8_t)~(1u << RR2_SELHOLDEXT_Pos);

    return RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_HYST_TRIG, sh.com_hyst_trig);
}

RR2_Status RR2_Ctrl_SetAnalogMux(uint8_t enable)
{
    uint8_t b = sh.out_power;

    /* Only the LG enable is touched. This used to clear EN_aMuxHG on
       the way past, which was fine while nothing read HG - but now that
       'hg' is a runtime switch, keeping that habit would let 'mux 1'
       (and 'preset csi', which ends in it) silently unpower a buffer
       the DAQ is still appending codes from. Each gain path owns its
       own bit.                                                        */
    b &= (uint8_t)~(1u << RR2_EN_AMUXLG_Pos);

    if (enable) {
        b |= (uint8_t)(1u << RR2_EN_AMUXLG_Pos);
        /* The buffer supply the mux output shares. */
        b |= (uint8_t)(1u << RR2_ON_ABUFFER_Pos);
    }

    sh.out_power = b;
    return RR2_Write(RR2_ADDR_OUTING, RR2_OUT_SUB_POWER, sh.out_power);
}

RR2_Status RR2_Ctrl_SetAnalogMuxHG(uint8_t enable)
{
    uint8_t b = sh.out_power;

    if (enable) {
        b |= (uint8_t)(1u << RR2_EN_AMUXHG_Pos);
        /* Same shared buffer supply the LG path switches on. */
        b |= (uint8_t)(1u << RR2_ON_ABUFFER_Pos);
    } else {
        /* Drop only the HG enable. ON_aBuffer stays up because the LG
           mux output rides on it too - turning HG off must not take
           the working gain path down with it.                         */
        b &= (uint8_t)~(1u << RR2_EN_AMUXHG_Pos);
    }

    sh.out_power = b;
    return RR2_Write(RR2_ADDR_OUTING, RR2_OUT_SUB_POWER, sh.out_power);
}

/* ------------------------------------------------------------------ */
/* Preset                                                              */
/* ------------------------------------------------------------------ */
RR2_Status RR2_Ctrl_PresetCsI(void)
{
    RR2_Status st;

    /* CsI(Tl) light decay is around 1 us, so the 20 ns shaping steps
       are far too fast to collect the charge. Switch both paths to the
       120 ns step range and pick a long peaking time.                */
    st = RR2_Ctrl_SetSlowShaping(RR2_CH_ALL, 1u, 1u);
    if (st != RR2_OK) return st;

    /* Index 14 with 120 ns steps lands near 1.8 us. */
    st = RR2_Ctrl_SetShapingTime(RR2_CH_ALL, 14u, 14u);
    if (st != RR2_OK) return st;

    /* Hold must fire near the shaper peak. delay * 0.85 ns * slopeTrim
       with delay=255 and slopeTrim=15 gives roughly 2.6 us, which is
       the right order for a 1.8 us peaking time. Scan it on hardware. */
    st = RR2_Ctrl_SetHoldDelay(255u, 15u);
    if (st != RR2_OK) return st;

    /* Make sure the ADC can actually see the LG peak detectors. */
    return RR2_Ctrl_SetAnalogMux(1u);
}
