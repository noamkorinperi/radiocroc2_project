/**
 ******************************************************************************
 * @file    rr2_i2ctest.c
 * @brief   Slow Control link test. See rr2_i2ctest.h for what and why.
 ******************************************************************************
 */
#include "rr2_i2ctest.h"

#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "radioroc2.h"
#include "radioroc2_regs.h"
#include "radioroc2_ctrl.h"
#include "usb_stream.h"

#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* The two speed settings                                              */
/* ------------------------------------------------------------------ */
/* Pass 1 is whatever the board boots with. Pass 2 divides both clocks
   by ten, which leaves the 20:1 ratio the ASIC needs untouched while
   giving the level translator's pull-up ten times as long to charge
   the line. Only the speed changes between the passes - SCLDEL and
   SDADEL are deliberately identical, so a difference in the result can
   only be a difference in speed.

   TIM1 runs from 216 MHz (APB2 doubled for the timers):
       ARR 107  -> 216e6 / 108  = 2.000 MHz
       ARR 1079 -> 216e6 / 1080 = 200.0 kHz

   I2C1 runs from PCLK1 = 54 MHz. TIMINGR is
   [PRESC][-][SCLDEL][SDADEL][SCLH][SCLL]:
       0x20404768 -> PRESC 2,  SCLH 71,  SCLL 104 -> 101.7 kHz nominal
       0xB040C7F9 -> PRESC 11, SCLH 199, SCLL 249 ->  10.00 kHz nominal

   "Nominal" is the register arithmetic only. The real SCL period is
   longer - the peripheral adds its synchronisation time and the line's
   own rise time on top - so the figures printed below are labelled
   nominal and nothing is concluded from them. What the bus actually
   does is settled by whether the ASIC answers, which is the entire
   point of this file.                                                */
/* Only the slow pair is ever programmed. Pass 1's values are not named
   here because they are never written - RR2_I2CTest_Run() saves the
   live ARR and TIMINGR on the way in and restores those on the way out,
   so whatever the board booted with is what pass 1 measures and what
   the hardware is left holding. */
#define CLK_ARR_SLOW      1079u
#define TIMINGR_SLOW      0xB040C7F9u

/* How many times the round trip is repeated per pass. One success can
   be luck; a bus that is marginal rather than broken shows up as a
   count somewhere between 0 and this.                                */
#define ROUNDTRIPS        32u

/* The byte the round trip writes. Two patterns, not one: 0x55 and 0xAA
   are bitwise complements, so between them every data bit is driven
   both high and low. A single pattern would pass happily with a bit
   stuck at the value that pattern happens to want.                   */
#define PATTERN_A         0x55u
#define PATTERN_B         0xAAu

/* Harmless target: the common block's hold-delay byte. Restored from
   the RAM shadow when the test finishes.                             */
#define TEST_ADDR         RR2_ADDR_COMMON
#define TEST_SUB          RR2_COM_SUB_DELAY

/* ------------------------------------------------------------------ */
/* Results of one pass                                                 */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t clk_sm_hz;      /* CLK_SM_I2C, computed from TIM1        */
    uint32_t scl_nominal_hz; /* register arithmetic only - see above  */

    uint8_t  ack_count;      /* how many 7-bit addresses ACKed        */
    uint8_t  ack_first;      /* the lowest one, 0xFF if none          */
    int16_t  chip_id;        /* derived from an aligned run of 8, -1  */

    uint32_t w_ok;           /* round trips whose three writes ACKed  */
    uint32_t r_ok;           /* ...and whose read frame also ACKed    */
    uint32_t match;          /* ...and whose value came back intact   */
    uint8_t  first_err;      /* first non-OK RR2_Status seen          */
} PassResult;

/* ------------------------------------------------------------------ */
/* Output                                                              */
/* ------------------------------------------------------------------ */
/* The report is a few hundred bytes and the transmit ring is 8 KB, so
   it fits - but USBStream_Task() only runs from the main loop, and
   this function IS the main loop for as long as it is running. So the
   ring is pumped here, and drained deliberately when it fills.       */
static void emit(const char *s)
{
    (void)USBStream_SendText(s);

    USBStream_Task();

    /* Half full is early enough that the next few lines cannot
       overflow before the DMA gets another chance. */
    while (USBStream_GetPending() > (USBSTREAM_RING_SIZE / 2u)) {
        USBStream_Task();
    }
}

static void emit_kv(const char *key, int32_t value)
{
    char b[48];
    (void)snprintf(b, sizeof(b), "i2ctest.%s=%ld\r\n", key, (long)value);
    emit(b);
}

static void emit_kvs(const char *key, const char *value)
{
    char b[64];
    (void)snprintf(b, sizeof(b), "i2ctest.%s=%s\r\n", key, value);
    emit(b);
}

/* ------------------------------------------------------------------ */
/* Clock and timing control                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief CLK_SM_I2C in Hz, read back from the TIM1 registers.
 *
 * TIM1 sits on APB2, whose clock is doubled for the timers unless the
 * APB2 prescaler is 1. Bit 2 of PPRE2 is the "is divided" bit.
 */
static uint32_t clk_sm_hz(void)
{
    const uint32_t src = ((RCC->CFGR & RCC_CFGR_PPRE2_2) == 0u)
                       ? HAL_RCC_GetPCLK2Freq()
                       : (HAL_RCC_GetPCLK2Freq() * 2u);

    const uint32_t div = (TIM1->PSC + 1u) * (TIM1->ARR + 1u);

    return (div > 0u) ? (src / div) : 0u;
}

/**
 * @brief Retune CLK_SM_I2C without stopping it.
 *
 * ARR and CCR1 are both preloaded (CubeMX enables auto-reload preload),
 * so writing them alone would leave the old values in the shadow
 * registers until the next update event. Forcing UG transfers them
 * immediately and restarts the period cleanly.
 */
static void clk_set_arr(uint32_t arr)
{
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (arr + 1u) / 2u);

    TIM1->EGR = TIM_EGR_UG;
}

/**
 * @brief What actually clocks I2C1, in Hz.
 *
 * Resolved from DCKCFGR2 rather than asked of the HAL. On STM32F7,
 * HAL_RCCEx_GetPeriphCLKFreq() only knows how to answer for the SAI
 * clocks - hand it RCC_PERIPHCLK_I2C1 and it returns 0, silently, which
 * makes every frequency derived from it come out as 0 too.
 */
static uint32_t i2c1_kernel_hz(void)
{
    switch (__HAL_RCC_GET_I2C1_SOURCE()) {
        case RCC_I2C1CLKSOURCE_SYSCLK: return HAL_RCC_GetSysClockFreq();
        case RCC_I2C1CLKSOURCE_HSI:    return (uint32_t)HSI_VALUE;
        case RCC_I2C1CLKSOURCE_PCLK1:
        default:                       return HAL_RCC_GetPCLK1Freq();
    }
}

/**
 * @brief Nominal SCL from TIMINGR. Register arithmetic only.
 *
 * Printed for context, never used to judge anything - the real period
 * is longer by the peripheral's synchronisation time plus the line's
 * rise time, and neither is knowable from a register.
 */
static uint32_t scl_nominal_hz(void)
{
    const uint32_t kernel = i2c1_kernel_hz();
    const uint32_t t      = I2C1->TIMINGR;

    const uint32_t presc = ((t >> 28) & 0x0Fu) + 1u;
    const uint32_t sclh  = ((t >>  8) & 0xFFu) + 1u;
    const uint32_t scll  = ( t        & 0xFFu) + 1u;

    const uint32_t ticks = presc * (sclh + scll);

    return ((kernel > 0u) && (ticks > 0u)) ? (kernel / ticks) : 0u;
}

/**
 * @brief Reprogram TIMINGR.
 *
 * TIMINGR is write-protected while PE is set, and the reference manual
 * asks for PE to stay low for at least three APB cycles before it is
 * raised again. The read-back of CR1 enforces that ordering.
 */
static void i2c_set_timing(uint32_t timingr)
{
    const uint32_t pe = I2C1->CR1 & I2C_CR1_PE;

    I2C1->CR1 &= ~I2C_CR1_PE;
    (void)I2C1->CR1;
    __DSB();

    I2C1->TIMINGR   = timingr;
    hi2c1.Init.Timing = timingr;   /* keep the HAL handle honest */

    I2C1->CR1 |= pe;
    (void)I2C1->CR1;
}

/* ------------------------------------------------------------------ */
/* Step 1 - is the bus usable at all?                                  */
/* ------------------------------------------------------------------ */
/* Everything after this assumes the bus can move. A line held low
   makes an address scan return "nothing answered", which would be a
   true statement and a completely misleading one.                    */
static RR2_I2CTestVerdict check_bus(void)
{
    const uint8_t scl_af = (uint8_t)((GPIOB->AFR[1] >> (4u * 0u)) & 0xFu);
    const uint8_t sda_af = (uint8_t)((GPIOB->AFR[1] >> (4u * 1u)) & 0xFu);
    const uint8_t scl_md = (uint8_t)((GPIOB->MODER  >> (2u * 8u)) & 3u);
    const uint8_t sda_md = (uint8_t)((GPIOB->MODER  >> (2u * 9u)) & 3u);

    const uint8_t scl_idle = ((GPIOB->IDR & GPIO_PIN_8) != 0u) ? 1u : 0u;
    const uint8_t sda_idle = ((GPIOB->IDR & GPIO_PIN_9) != 0u) ? 1u : 0u;

    emit_kv("scl_idle", scl_idle);
    emit_kv("sda_idle", sda_idle);

    if ((scl_md != 2u) || (sda_md != 2u) ||
        (scl_af != GPIO_AF4_I2C1) || (sda_af != GPIO_AF4_I2C1) ||
        ((RCC->APB1ENR & RCC_APB1ENR_I2C1EN) == 0u) ||
        ((I2C1->CR1 & I2C_CR1_PE) == 0u)) {
        return RR2_I2CT_BUS_NOT_READY;
    }

    if (!scl_idle) return RR2_I2CT_SCL_STUCK_LOW;
    if (!sda_idle) return RR2_I2CT_SDA_STUCK_LOW;

    return RR2_I2CT_OK;
}

/* ------------------------------------------------------------------ */
/* Step 2 - who is out there?                                          */
/* ------------------------------------------------------------------ */
/**
 * @brief Probe every 7-bit address and record what answers.
 *
 * A probe is an address byte and a STOP, with no data, so it cannot
 * move the ASIC's sub-address latch or disturb any configuration no
 * matter which address it lands on.
 *
 * The ASIC answers on eight consecutive addresses, (CHIP_ID << 3) | reg
 * for reg 0..7. Finding that aligned run of eight is what turns "some
 * device is there" into "the ASIC is there, and its chip id is N" -
 * which is exactly the question RR2_CHIP_ID in radioroc2.h guesses at.
 */
static void scan(PassResult *p)
{
    uint8_t seen[128];
    char    b[80];
    uint8_t n = 0u;

    memset(seen, 0, sizeof(seen));

    p->ack_first = 0xFFu;
    p->chip_id   = -1;

    for (uint8_t a = 0x08u; a <= 0x77u; ++a) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 1u, 5u) == HAL_OK) {
            seen[a] = 1u;
            n++;
            if (p->ack_first == 0xFFu) p->ack_first = a;

            /* Listed one per line so nothing has to be parsed out of a
               comma-separated field on the Python side. */
            (void)snprintf(b, sizeof(b), "i2ctest.ack_addr=0x%02X\r\n", a);
            emit(b);
        }
    }

    p->ack_count = n;

    /* An aligned run of eight is the ASIC's fingerprint. */
    for (uint8_t id = 0u; id < 16u; ++id) {
        const uint8_t base = (uint8_t)(id << 3);
        uint8_t all = 1u;

        for (uint8_t reg = 0u; reg < 8u; ++reg) {
            if (!seen[base + reg]) { all = 0u; break; }
        }
        if (all) { p->chip_id = (int16_t)id; break; }
    }
}

/* ------------------------------------------------------------------ */
/* Step 3 - the real protocol                                          */
/* ------------------------------------------------------------------ */
/**
 * @brief One write / read / compare cycle, counted in three parts.
 *
 * Kept separate on purpose. "The ASIC ACKs everything we write but
 * never reads back" and "the ASIC is not there" look identical if the
 * three are collapsed into one pass/fail, and they are entirely
 * different problems - radioroc2_ctrl.h notes the read path has never
 * been confirmed on hardware, so it is the more likely of the two to
 * be the one at fault.
 */
static void roundtrip_once(PassResult *p, uint8_t pattern)
{
    uint8_t readback = 0u;

    const RR2_Status w = RR2_Write(TEST_ADDR, TEST_SUB, pattern);

    if (w != RR2_OK) {
        if (p->first_err == RR2_OK) p->first_err = (uint8_t)w;
        return;
    }
    p->w_ok++;

    const RR2_Status r = RR2_Read(TEST_ADDR, TEST_SUB, &readback);

    if (r != RR2_OK) {
        if (p->first_err == RR2_OK) p->first_err = (uint8_t)r;
        return;
    }
    p->r_ok++;

    if (readback == pattern) {
        p->match++;
    } else if (p->first_err == RR2_OK) {
        p->first_err = (uint8_t)RR2_ERR_READBACK;
    }
}

/* ------------------------------------------------------------------ */
/* One full pass at the current speed                                  */
/* ------------------------------------------------------------------ */
static void run_pass(PassResult *p, const char *label)
{
    memset(p, 0, sizeof(*p));
    p->first_err = (uint8_t)RR2_OK;

    p->clk_sm_hz      = clk_sm_hz();
    p->scl_nominal_hz = scl_nominal_hz();

    emit_kvs("pass", label);
    emit_kv("clk_sm_hz", (int32_t)p->clk_sm_hz);
    emit_kv("scl_nominal_hz", (int32_t)p->scl_nominal_hz);

    scan(p);

    emit_kv("ack_count", p->ack_count);
    emit_kv("chip_id", p->chip_id);

    /* No point running the protocol against a bus where nothing
       answered - every round trip would fail on its first frame and
       take its timeout doing it. */
    if (p->ack_count == 0u) {
        emit_kv("w_ok", 0);
        emit_kv("r_ok", 0);
        emit_kv("match", 0);
        return;
    }

    for (uint32_t i = 0u; i < ROUNDTRIPS; ++i) {
        roundtrip_once(p, ((i & 1u) == 0u) ? PATTERN_A : PATTERN_B);
    }

    emit_kv("roundtrips", (int32_t)ROUNDTRIPS);
    emit_kv("w_ok",  (int32_t)p->w_ok);
    emit_kv("r_ok",  (int32_t)p->r_ok);
    emit_kv("match", (int32_t)p->match);
    emit_kv("first_err", p->first_err);
}

/**
 * @brief Turn one pass's counters into a verdict.
 */
static RR2_I2CTestVerdict judge(const PassResult *p)
{
    if (p->ack_count == 0u)          return RR2_I2CT_NO_DEVICE;
    if (p->chip_id   <  0)           return RR2_I2CT_WRONG_ID;
    if (p->match     == ROUNDTRIPS)  return RR2_I2CT_OK;
    if (p->w_ok      == ROUNDTRIPS)  return RR2_I2CT_WRITE_ONLY;

    return RR2_I2CT_NO_DEVICE;
}

static const char *verdict_name(RR2_I2CTestVerdict v)
{
    switch (v) {
        case RR2_I2CT_OK:            return "OK";
        case RR2_I2CT_WRITE_ONLY:    return "WRITE_ONLY";
        case RR2_I2CT_NO_DEVICE:     return "NO_DEVICE";
        case RR2_I2CT_WRONG_ID:      return "WRONG_ID";
        case RR2_I2CT_SCL_STUCK_LOW: return "SCL_STUCK_LOW";
        case RR2_I2CT_SDA_STUCK_LOW: return "SDA_STUCK_LOW";
        case RR2_I2CT_BUS_NOT_READY: return "BUS_NOT_READY";
        default:                     return "?";
    }
}

/* ------------------------------------------------------------------ */
/* Public entry point                                                  */
/* ------------------------------------------------------------------ */
RR2_I2CTestVerdict RR2_I2CTest_Run(void)
{
    PassResult p;
    RR2_I2CTestVerdict v;

    /* Everything that gets changed, so it can all go back. */
    const uint32_t saved_arr     = TIM1->ARR;
    const uint32_t saved_timingr = I2C1->TIMINGR;

    emit("i2ctest.begin=1\r\n");

    v = check_bus();
    if (v != RR2_I2CT_OK) {
        emit_kvs("verdict_name", verdict_name(v));
        emit_kv("verdict", (int32_t)v);
        emit("i2ctest.done=1\r\n");
        return v;
    }

    /* ---- pass 1: the speed the board boots with ------------------ */
    run_pass(&p, "fast");
    v = judge(&p);

    /* ---- pass 2: both clocks divided by ten, ratio preserved -----
       Only worth running if pass 1 found nothing at all. If the ASIC
       answered and merely failed to read back, the speed is clearly
       not what is wrong and repeating the whole thing slower would
       only add noise to the report. */
    if (v == RR2_I2CT_NO_DEVICE) {
        emit("i2ctest.retry_slow=1\r\n");

        clk_set_arr(CLK_ARR_SLOW);
        i2c_set_timing(TIMINGR_SLOW);
        HAL_Delay(2);           /* let the retuned clock settle */

        run_pass(&p, "slow");
        v = judge(&p);
    }

    /* ---- put the hardware back ----------------------------------- */
    clk_set_arr(saved_arr);
    i2c_set_timing(saved_timingr);
    HAL_Delay(2);

    /* ---- and put the ASIC back ----------------------------------- */
    /* From the shadow, not from a read: if the read path is the thing
       that is broken, a read-based restore would write back whatever
       garbage it returned. */
    if (p.ack_count > 0u) {
        (void)RR2_Write(TEST_ADDR, TEST_SUB, RR2_Ctrl_GetShadow()->com_delay);
    }

    emit_kvs("verdict_name", verdict_name(v));
    emit_kv("verdict", (int32_t)v);
    emit("i2ctest.done=1\r\n");

    return v;
}
