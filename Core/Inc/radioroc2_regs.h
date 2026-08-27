/**
 ******************************************************************************
 * @file    radioroc2_regs.h
 * @brief   RADIOROC2 Slow Control register map (datasheet v2.0, Table 4)
 *
 * A Slow-Control parameter is identified by THREE things:
 *      - address  (R1): 0-63 channel, 64 biasing, 65 common, 66 outing, 67 gate
 *      - subadd   (R0): the register within that address
 *      - a bit-field inside the data byte (many bytes pack several params)
 *
 * Bit-fields use CMSIS style: <NAME>_Pos (LSB position) and <NAME>_Msk (mask).
 * Compose a byte with RR2_SET(); extract with RR2_GET() (once reads work).
 *
 *      uint8_t b = RR2_SET(RR2_PATCOMP, 2) | RR2_SET(RR2_PATGAIN, 0x30);
 *      RR2_Write(RR2_ADDR_CH(5), RR2_CH_SUB_PAT, b);
 *
 * NOTE: This is a WRITE-oriented map. It never reads-modifies-writes, so a
 * byte you don't fully specify is written as 0. Where the datasheet default
 * matters, a <...>_DEFAULT byte is provided.
 ******************************************************************************
 */
#ifndef RADIOROC2_REGS_H
#define RADIOROC2_REGS_H

#include "radioroc2.h"   /* RR2_ADDR_*, RR2_Write(), RR2_Status */

/* ================================================================== */
/*  Field helpers                                                      */
/* ================================================================== */
/* The ASIC has 64 analog channels. Defined here rather than in the DAQ
   header because it is a property of the chip, not of the readout.   */
#define RR2_NUM_CHANNELS      64u

/* Place a value into a field:  RR2_SET(RR2_PATGAIN, 0x20)            */
#define RR2_SET(field, val)   ((uint8_t)(((uint8_t)(val) << field##_Pos) & field##_Msk))
/* Extract a field from a byte:  RR2_GET(RR2_PATGAIN, byte)           */
#define RR2_GET(field, byte)  ((uint8_t)(((byte) & field##_Msk) >> field##_Pos))
/* Pack two nibbles [7:4]=hi, [3:0]=lo (used all over address 64/66). */
#define RR2_NIBBLES(hi, lo)   ((uint8_t)((((uint8_t)(hi) & 0xFu) << 4) | ((uint8_t)(lo) & 0xFu)))


/* ================================================================== */
/*  ADDRESS 0-63  -  Per-channel configuration                         */
/*  (write the same subadd to each channel address you want to set)    */
/* ================================================================== */
#define RR2_CH_SUB_INDAC     0u   /* inDac[7:0]                        */
#define RR2_CH_SUB_PAT       1u   /* patComp[7:6], patGain[5:0]        */
#define RR2_CH_SUB_GAIN      2u   /* lgGain[7:4], hgGain[3:0]          */
#define RR2_CH_SUB_TAU       3u   /* tauLG[7:4], tauHG[3:0]            */
#define RR2_CH_SUB_CALIBT1   4u   /* calibDacT1[5:0]                   */
#define RR2_CH_SUB_CALIBT2   5u   /* calibDacT2[5:0]                   */
#define RR2_CH_SUB_EN        6u   /* enable bits (see below)           */
#define RR2_CH_SUB_EN2       7u   /* shaper/pdet enables + toggles     */
#define RR2_CH_SUB_PROBE     66u  /* per-channel probe switches (dbg)  */

/* subadd 0 */
#define RR2_INDAC_Pos        0u
#define RR2_INDAC_Msk        (0xFFu << RR2_INDAC_Pos)
#define RR2_CH_INDAC_DEFAULT 0x80u

/* subadd 1 */
#define RR2_PATGAIN_Pos      0u
#define RR2_PATGAIN_Msk      (0x3Fu << RR2_PATGAIN_Pos)   /* gain span 15..100 */
#define RR2_PATCOMP_Pos      6u
#define RR2_PATCOMP_Msk      (0x03u << RR2_PATCOMP_Pos)   /* Ccomp 50..150 fF  */
#define RR2_CH_PAT_DEFAULT   0xA0u                        /* patComp=10, gain=100000 */

/* subadd 2 */
#define RR2_HGGAIN_Pos       0u
#define RR2_HGGAIN_Msk       (0x0Fu << RR2_HGGAIN_Pos)    /* HG gain 5..80  */
#define RR2_LGGAIN_Pos       4u
#define RR2_LGGAIN_Msk       (0x0Fu << RR2_LGGAIN_Pos)    /* LG gain 0.5..8 */
#define RR2_CH_GAIN_DEFAULT  0x44u

/* subadd 3  (naming quirk kept from datasheet) */
#define RR2_TAUHG_Pos        0u
#define RR2_TAUHG_Msk        (0x0Fu << RR2_TAUHG_Pos)     /* shaping 20..300 ns */
#define RR2_TAULG_Pos        4u
#define RR2_TAULG_Msk        (0x0Fu << RR2_TAULG_Pos)
#define RR2_CH_TAU_DEFAULT   0x11u

/* subadd 4 / 5  (channel-wise threshold trims, top 2 bits NC) */
#define RR2_CALIBT1_Pos      0u
#define RR2_CALIBT1_Msk      (0x3Fu << RR2_CALIBT1_Pos)
#define RR2_CALIBT2_Pos      0u
#define RR2_CALIBT2_Msk      (0x3Fu << RR2_CALIBT2_Pos)

/* subadd 6  -  single-bit enables (default 0x7F: all enabled, DAC_select=0) */
#define RR2_DAC_SELECT_Pos     7u   /* 0=100ohm input, 1=HiZ (ext R+C)  */
#define RR2_EN_INDAC_Pos       6u
#define RR2_EN_PAT_Pos         5u
#define RR2_EN_DISCRI1_Pos     4u
#define RR2_EN_DISCRI2_Pos     3u
#define RR2_EN_DISCRICHARGE_Pos 2u
#define RR2_EN_PALG_Pos        1u
#define RR2_EN_PAHG_Pos        0u
#define RR2_CH_EN_DEFAULT      0x7Fu

/* subadd 7  -  shaper/peak-detector enables + shaping toggles (default 0x0F) */
#define RR2_SLOWSHAPINGLG_Pos  7u   /* 0=20ns LSB, 1=120ns LSB */
#define RR2_SLOWSHAPINGHG_Pos  6u
#define RR2_USECTEST_Pos       5u
#define RR2_EN_TEST_Pos        4u
#define RR2_EN_SHLG_Pos        3u
#define RR2_EN_SHHG_Pos        2u
#define RR2_EN_PDETLG_Pos      1u
#define RR2_EN_PDETHG_Pos      0u
#define RR2_CH_EN2_DEFAULT     0x0Fu

/* subadd 66 (probe switches, all default 0 - debug only) */
#define RR2_CMD_TQ_Pos         4u
#define RR2_CMD_SHHG_Pos       3u
#define RR2_CMD_SHLG_Pos       2u
#define RR2_CMD_PAHG_Pos       1u
#define RR2_CMD_PALG_Pos       0u


/* ================================================================== */
/*  ADDRESS 64  -  ASIC biasing (all nibble pairs, default 0x44)       */
/*  byte = RR2_NIBBLES(<hi param [7:4]>, <lo param [3:0]>)             */
/* ================================================================== */
#define RR2_BIAS_SUB_INDAC0    0u   /* Ibo_inDac0 | Ibi_inDac0        */
#define RR2_BIAS_SUB_INDAC1    1u   /* Ibo_inDac1 | Ibi_inDac1        */
#define RR2_BIAS_SUB_CALIB_PAT 2u   /* Ib_calibDac | Ib_paT           */
#define RR2_BIAS_SUB_PA_HGLG   3u   /* Ib_paHG | Ib_paLG              */
#define RR2_BIAS_SUB_SHHG      4u   /* Ibi_shHG | Ibo_shHG            */
#define RR2_BIAS_SUB_SHLG      5u   /* Ibi_shLG | Ibo_shLG            */
#define RR2_BIAS_SUB_PDET      6u   /* Ibi_pdetector | Ibi_pdbuffer   */
#define RR2_BIAS_SUB_FC_PDET   7u   /* Ib_FCP_pdetector | Ib_FCN_pd.  */
#define RR2_BIAS_SUB_FC_PDBUF  8u   /* Ib_FCP_pdbuffer | Ib_FCN_pdb.  */
#define RR2_BIAS_SUB_DISCRI1A  9u   /* Ibi_discri1 | Ibm1_discri1     */
#define RR2_BIAS_SUB_DISCRI1B  10u  /* Ibm2_discri1 | Ibi_discri2     */
#define RR2_BIAS_SUB_DISCRI2   11u  /* Ibm1_discri2 | Ibm2_discri2    */
#define RR2_BIAS_SUB_DISCRICHG 12u  /* Ibi_discricharge | Ibo_disch.  */
#define RR2_BIAS_SUB_ON1       13u  /* ON_* enables (default 0xFF)    */
#define RR2_BIAS_SUB_ON2       14u  /* ON_* enables (default 0xFF)    */
#define RR2_BIAS_NIBBLE_DEFAULT 0x44u

/* subadd 13 ON bits (default all 1 -> 0xFF) */
#define RR2_ON_INDAC0_Pos      7u
#define RR2_ON_INDAC1_Pos      6u
#define RR2_ON_PAT_Pos         5u
#define RR2_ON_PAHG_Pos        4u
#define RR2_ON_PALG_Pos        3u
#define RR2_ON_SHHG_Pos        2u
#define RR2_ON_SHLG_Pos        1u
#define RR2_ON_PDETECTOR_Pos   0u
#define RR2_BIAS_ON1_DEFAULT   0xFFu
/* subadd 14 ON bits ([7:5] NC=111, default 0xFF) */
#define RR2_ON_CALIBDAC_Pos    4u
#define RR2_ON_DISCRICHARGE_Pos 3u
#define RR2_ON_PDBUFFER_Pos    2u
#define RR2_ON_DISCRI1_Pos     1u
#define RR2_ON_DISCRI2_Pos     0u
#define RR2_BIAS_ON2_DEFAULT   0xFFu


/* ================================================================== */
/*  ADDRESS 65  -  Common blocks                                       */
/* ================================================================== */
#define RR2_COM_SUB_BG         0u   /* bg[5:0] bandgap trim (dflt 0x20) */
#define RR2_COM_SUB_DAC1_LO    1u   /* Dac1[7:0]                        */
#define RR2_COM_SUB_DAC2_DAC1  2u   /* Dac2[5:0]<<2 | Dac1[9:8]         */
#define RR2_COM_SUB_DACQ_DAC2  3u   /* DacQ[3:0]<<4 | Dac2[9:6]         */
#define RR2_COM_SUB_DACQ_HI    4u   /* DacQ[9:4]                        */
#define RR2_COM_SUB_THDAC_BIAS 5u   /* Ibi_thresholdDac | Ibo_...       */
#define RR2_COM_SUB_THDACQ_BIAS 6u  /* Ibi_thresholdDacQ | Ibo_...      */
#define RR2_COM_SUB_VREF_EN    7u   /* vref[7:4] + EN_th* bits          */
#define RR2_COM_SUB_DELAY      8u   /* delay[7:0] (default 0xFF)        */
#define RR2_COM_SUB_SLOPE      9u   /* slopeTrim[7:4] | ibi_discri_dly  */
#define RR2_COM_SUB_DISCRIDLY  10u  /* ibm_discri_delay | ibo_...       */
#define RR2_COM_SUB_DELAYDAC   11u  /* ibi_delayDac | ibo_delayDac      */
#define RR2_COM_SUB_HYST_TRIG  12u  /* hysteresis, EN_delay, selHold... */
#define RR2_COM_SUB_PROBE      66u  /* cmd_globalTrigger, cmd_hold (dbg)*/

/* subadd 0 */
#define RR2_BG_Pos             0u
#define RR2_BG_Msk             (0x3Fu << RR2_BG_Pos)
#define RR2_COM_BG_DEFAULT     0x20u

/* subadd 7 */
#define RR2_VREF_Pos           4u
#define RR2_VREF_Msk           (0x0Fu << RR2_VREF_Pos)   /* Vref_1v trim, dflt 1000 */
#define RR2_EN_TH1_Pos         3u
#define RR2_EN_TH2_Pos         2u
#define RR2_EN_THQ_Pos         1u
#define RR2_EN_BG_Pos          0u
#define RR2_COM_VREF_EN_DEFAULT 0x8Fu  /* vref=1000, all EN=1 */

/* subadd 8 */
#define RR2_DELAY_Pos          0u
#define RR2_DELAY_Msk          (0xFFu << RR2_DELAY_Pos)
#define RR2_COM_DELAY_DEFAULT  0xFFu

/* subadd 9 */
#define RR2_SLOPETRIM_Pos      4u
#define RR2_SLOPETRIM_Msk      (0x0Fu << RR2_SLOPETRIM_Pos)

/* subadd 12 */
#define RR2_HYSTERESIS1_Pos    7u
#define RR2_HYSTERESIS2_Pos    6u
#define RR2_EN_DELAY_Pos       5u
#define RR2_SELHOLDEXT_Pos     4u   /* 0=internal hold, 1=external (HOLDEXT pin) */
#define RR2_SELTRIG_Pos        0u
#define RR2_SELTRIG_Msk        (0x0Fu << RR2_SELTRIG_Pos)
/* selTrig<1:0> (peak-detector acquisition source) */
#define RR2_SELTRIG_GLOBAL     0x0u
#define RR2_SELTRIG_T1         0x1u
#define RR2_SELTRIG_T2         0x2u
#define RR2_SELTRIG_CHARGE     0x3u
/* selTrig<3:2> (which global trigger, when <1:0>=00) */
#define RR2_SELTRIG_G_GND      (0x0u << 2)  /* use with TRIGEXT */
#define RR2_SELTRIG_G_T1       (0x1u << 2)
#define RR2_SELTRIG_G_T2       (0x2u << 2)
#define RR2_SELTRIG_G_TQ       (0x3u << 2)
#define RR2_COM_HYST_TRIG_DEFAULT 0xE4u  /* hyst1=1,hyst2=1,EN_delay=1,selTrig=0100 */

/* subadd 66 probe (debug) */
#define RR2_CMD_GLOBALTRIGGER_Pos 1u
#define RR2_CMD_HOLD_Pos          0u

/**
 * @brief  Program the three time/charge threshold DACs (10-bit each).
 *         Writes subadd 1..4 of address 65 in one consistent group, so
 *         the bytes shared between DACs are never clobbered.
 * @param  dac1  Time Trigger Low threshold  (0..1023)
 * @param  dac2  Time Trigger High threshold (0..1023)
 * @param  dacQ  Charge Trigger threshold    (0..1023)
 */
static inline RR2_Status RR2_SetTimeThresholds(uint16_t dac1, uint16_t dac2, uint16_t dacQ)
{
    RR2_Status st;
    /* subadd1 = Dac1[7:0] */
    st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_DAC1_LO, (uint8_t)(dac1 & 0xFFu));
    if (st != RR2_OK) return st;
    /* subadd2 = Dac2[5:0]<<2 | Dac1[9:8] */
    st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_DAC2_DAC1,
                   (uint8_t)(((dac2 & 0x3Fu) << 2) | ((dac1 >> 8) & 0x03u)));
    if (st != RR2_OK) return st;
    /* subadd3 = DacQ[3:0]<<4 | Dac2[9:6] */
    st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_DACQ_DAC2,
                   (uint8_t)(((dacQ & 0x0Fu) << 4) | ((dac2 >> 6) & 0x0Fu)));
    if (st != RR2_OK) return st;
    /* subadd4 = DacQ[9:4] */
    st = RR2_Write(RR2_ADDR_COMMON, RR2_COM_SUB_DACQ_HI,
                   (uint8_t)((dacQ >> 4) & 0x3Fu));
    return st;
}


/* ================================================================== */
/*  ADDRESS 66  -  Outing (per-channel outputs + global driver config) */
/* ================================================================== */
/* subadd 0..63 : per-channel output routing. subadd == channel number */
#define RR2_OUT_SUB_CH(n)      ((uint8_t)(n))   /* n = 0..63 */
#define RR2_OUTPAD_Pos         0u   /* [1:0]  OUT1 routing            */
#define RR2_OUTPAD_Msk         (0x03u << RR2_OUTPAD_Pos)
#define RR2_OUTPAD2_Pos        2u   /* [3:2]  OUT2 routing            */
#define RR2_OUTPAD2_Msk        (0x03u << RR2_OUTPAD2_Pos)
#define RR2_LVDSOUT_Pos        4u   /* [5:4]  differential select     */
#define RR2_LVDSOUT_Msk        (0x03u << RR2_LVDSOUT_Pos)
#define RR2_CM_Pos             6u   /* [7:6]  common-mode select      */
#define RR2_CM_Msk             (0x03u << RR2_CM_Pos)
/* outPad1 / outPad2 values */
#define RR2_OUTPAD_T1          0x0u
#define RR2_OUTPAD_T2          0x1u
#define RR2_OUTPAD_SLOW_HG     0x2u
#define RR2_OUTPAD_SLOW_LG     0x3u
/* lvdsOut values (must be 00 to use single-ended outPad settings) */
#define RR2_LVDS_OFF           0x0u
#define RR2_LVDS_T1_DIFF       0x1u
#define RR2_LVDS_T2_DIFF       0x2u
#define RR2_LVDS_DISABLE_ALL   0x3u

/* global driver config */
#define RR2_OUT_SUB_EXTENDED   64u  /* extendedOutput(7), delayPE[6:5], bufSize[4:0] */
#define RR2_OUT_SUB_BUFSIZE    65u  /* bufSizeClps[7:4], bufSizePE[3:0]  */
#define RR2_OUT_SUB_COMPCAP    66u  /* cmProbe[5:4], cmLG[3:2], cmHG[1:0] */
#define RR2_OUT_SUB_ABUF1      67u  /* ibi_aBuffer | ibFCP_aBuffer       */
#define RR2_OUT_SUB_ABUF2      68u  /* ibFCN_aBuffer | ibi_outing        */
#define RR2_OUT_SUB_OUTING     69u  /* ibFCP_outing | ibFCN_outing       */
#define RR2_OUT_SUB_POWER      70u  /* ON_outing, ON_aBuffer, EN_* below */

#define RR2_EXTENDEDOUTPUT_Pos 7u
/* subadd 70 power/enable bits */
#define RR2_ON_OUTING_Pos      4u
#define RR2_ON_ABUFFER_Pos     3u
#define RR2_EN_PROBE_Pos       2u
#define RR2_EN_AMUXHG_Pos      1u   /* HG analog-mux buffer - PA4 / ADC1_IN4 */
#define RR2_EN_AMUXLG_Pos      0u   /* enable LG analog-mux buffer (needed for ADC read) */
/* Both mux outputs reach an ADC. LG is always read; HG is opt-in at
   runtime through the 'hg' host command, so the default keeps its
   buffer unpowered and a host that never asks sees no change at all. */
#define RR2_OUT_POWER_DEFAULT  0x09u /* ON_aBuffer=1, EN_aMuxHG=0, EN_aMuxLG=1 */


/* ================================================================== */
/*  ADDRESS 67  -  Event validation gating                             */
/* ================================================================== */
#define RR2_GATE_SUB_RX        0u
#define RR2_EN_RX_Pos          1u   /* differential valid-event receiver, dflt 1 */
#define RR2_FORCED_VALEVT_Pos  0u   /* 0=external, 1=internal                     */
#define RR2_GATE_RX_DEFAULT    0x02u

/* ================================================================== */
/*  High-level configuration (implemented in radioroc2_config.c)       */
/* ================================================================== */

/** Configure one channel (subadd 0..7) to datasheet defaults. */
RR2_Status RR2_ConfigChannel(uint8_t ch);

/** Configure all 64 channels to defaults (~0.4 s at 100 kHz I2C). */
RR2_Status RR2_ConfigAllChannels(void);

/** Write the same byte to a given subadd of every channel (broadcast). */
RR2_Status RR2_WriteAllChannels(uint8_t subadd, uint8_t data);

/** Configure the global blocks: biasing (64), common (65),
 *  analog-mux power (66) and event gating (67). */
RR2_Status RR2_ConfigCommon(void);

/** Full baseline for charge/energy readout (Example 1):
 *  common blocks + all channels + the three threshold DACs.
 *  th_* are raw 10-bit codes (0..1023 -> 256..534 mV). TUNE to your SiPM. */
RR2_Status RR2_ConfigDefault(uint16_t th_low, uint16_t th_high, uint16_t th_charge);

#endif /* RADIOROC2_REGS_H */
