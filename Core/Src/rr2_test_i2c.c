/**
 ******************************************************************************
 * @file    rr2_test_i2c.c
 * @brief   Slow Control I2C health check - staged, self-reporting.
 * @note    See rr2_test_i2c.h for what each stage proves and how to run it.
 ******************************************************************************
 */
#include "rr2_test_i2c.h"
#include "radioroc2.h"
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usb_stream.h"
#include <stdio.h>

/* ------------------------------------------------------------------ */
/* Board facts this test checks against                                */
/* ------------------------------------------------------------------ */
/* SCL / SDA of I2C1. Not in main.h because CubeMX configures them
   inside HAL_I2C_MspInit() rather than as named pins - keep these two
   in step with that function if the bus ever moves. */
#define TEST_SCL_PORT           GPIOB
#define TEST_SCL_PIN            GPIO_PIN_8
#define TEST_SDA_PORT           GPIOB
#define TEST_SDA_PIN            GPIO_PIN_9

/* CLK_SM_I2C must be exactly 20 x SCL, and the bus is a 100 kHz bus,
   so TIM1 has to emit 2 MHz on PE9. This is checked against what the
   timer is programmed to produce, which is exact and repeatable -
   never against the SCL derived below, which is a software estimate
   and would turn a hard requirement into a flaky one. */
#define TEST_CLK_TARGET_HZ      2000000u
#define TEST_CLK_TOL_PCT        5u

/* SCL clocks in one address-only probe: 8 address bits plus the ACK
   bit. START and STOP are not full clock periods and are left out -
   counting them would bias the derived frequency high. */
#define TEST_CLOCKS_PER_PROBE   9u

/* Room for one report line. */
#define TEST_LINE_MAX           96u

/* How long to spend pushing the report out before giving up, in ms.
   Only relevant when nothing is draining the host link. */
#define TEST_FLUSH_MS           200u

/* ------------------------------------------------------------------ */
/* Controls                                                            */
/* ------------------------------------------------------------------ */
volatile uint8_t g_test_run     = 0u;
volatile uint8_t g_test_repeat  = 0u;
volatile uint8_t g_test_apply   = 1u;
volatile uint8_t g_test_verbose = 1u;

/* ------------------------------------------------------------------ */
/* Results                                                             */
/* ------------------------------------------------------------------ */
volatile uint8_t  g_test_verdict = RR2_TEST_PENDING;
volatile uint8_t  g_test_stage   = RR2_TEST_STAGE_NONE;
volatile uint8_t  g_test_reason  = RR2_TEST_REASON_NONE;
volatile uint32_t g_test_runs    = 0u;

volatile uint32_t g_test_clk_hz      = 0u;
volatile uint8_t  g_test_clk_running = 0u;
volatile uint8_t  g_test_scl_idle    = 0u;
volatile uint8_t  g_test_sda_idle    = 0u;
volatile uint8_t  g_test_resets_ok   = 0u;
volatile uint8_t  g_test_errorn_sc   = 0u;
volatile uint8_t  g_test_i2c_state   = 0u;

volatile uint8_t  g_test_chipid       = RR2_CHIP_ID_NONE;
volatile uint16_t g_test_chipid_map   = 0u;
volatile uint16_t g_test_any_map      = 0u;
volatile uint8_t  g_test_chipid_count = 0u;
volatile uint8_t  g_test_reg_mask[16] = {0};
volatile uint8_t  g_test_stable       = 0u;
volatile uint32_t g_test_probes       = 0u;
volatile uint32_t g_test_acks         = 0u;
volatile uint32_t g_test_nacks        = 0u;
volatile uint32_t g_test_timeouts     = 0u;
volatile uint32_t g_test_busy         = 0u;
volatile uint32_t g_test_reinits      = 0u;
volatile uint32_t g_test_sweep_ms     = 0u;

volatile uint32_t g_test_probe_ns  = 0u;
volatile uint32_t g_test_scl_khz   = 0u;
volatile uint32_t g_test_ratio_x10 = 0u;

/* One-line summary, kept for RR2_TestI2C_ResultText(). */
static char s_result[TEST_LINE_MAX] = "not run";

/* ------------------------------------------------------------------ */
/* Reporting over the host link                                        */
/* ------------------------------------------------------------------ */
/**
 * @brief Queue one report line, and pump it out.
 *
 * The ring is drained here rather than left to the main loop, because
 * the whole point of the boot-time run is that the report is already on
 * the wire by the time anything else happens - and on a failing board
 * the main loop may never get that far.
 */
static void say(const char *line)
{
    if (!g_test_verbose) return;

    const uint32_t deadline = HAL_GetTick() + TEST_FLUSH_MS;

    /* The link takes text verbatim - terminating a line is the callers
       job here, the same as it is in usb_cmd.c. Adding it once in this
       function rather than to every format string is what keeps the
       report from arriving as one unbroken run-on line. */
    (void)USBStream_SendText(line);
    (void)USBStream_SendText("\r\n");

    while ((USBStream_GetPending() > 0u) && (HAL_GetTick() < deadline)) {
        USBStream_Task();
    }
}

/* Format straight into the shared buffer and send it. Static rather
   than a local because these run at boot, where the stack is shared
   with the HAL init path. */
static char s_line[TEST_LINE_MAX];

/* ------------------------------------------------------------------ */
/* Small helpers                                                       */
/* ------------------------------------------------------------------ */
/**
 * @brief TIM1s own clock, which is not the APB2 bus clock.
 *
 * Whenever the APB2 prescaler is anything other than 1 the timer clock
 * is doubled. Here APB2 runs at 108 MHz and TIM1 therefore at 216 MHz,
 * which is what makes ARR = 107 come out as 2.000 MHz on PE9.
 */
static uint32_t tim1_clk_hz(void)
{
    uint32_t hz = HAL_RCC_GetPCLK2Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) {
        hz *= 2u;
    }
    return hz;
}

/**
 * @brief Is the cycle counter actually counting?
 *
 * RR2_DAQ_Init() turns it on, but this test has to survive being run
 * before that or on a part where the DWT stayed locked. Everything it
 * feeds is informational, so a dead counter costs the timing figures
 * and nothing else.
 */
static uint8_t dwt_ready(void)
{
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u) return 0u;

    const uint32_t t0 = DWT->CYCCNT;
    __NOP(); __NOP(); __NOP(); __NOP();
    return (DWT->CYCCNT != t0) ? 1u : 0u;
}

static uint32_t cyc_to_ns(uint32_t cycles)
{
    const uint32_t mhz = SystemCoreClock / 1000000u;

    if (mhz == 0u)                      return 0u;
    if (cycles > (0xFFFFFFFFu / 1000u)) return 0xFFFFFFFFu;

    return (cycles * 1000u) / mhz;
}

static uint8_t popcount16(uint16_t v)
{
    uint8_t n = 0u;

    while (v != 0u) {
        v &= (uint16_t)(v - 1u);
        n++;
    }
    return n;
}

/* ------------------------------------------------------------------ */
/* Stage 1 - BUS                                                       */
/* ------------------------------------------------------------------ */
/**
 * @brief Everything that has to be true before an address probe means
 *        anything.
 *
 * Nothing here drives the bus. Each check reads back state the firmware
 * already set up, so the whole stage costs microseconds and can only
 * fail for reasons that would otherwise masquerade as "the ASIC did not
 * answer".
 *
 * @retval RR2_TEST_REASON_NONE when the bus is fit to probe
 */
static RR2_TestReason stage_bus(void)
{
    /* Recorded first, and never judged. The ASIC drives ERRORN_SC low
       after a bad Slow Control frame and holds it until the error is
       read, so a low here usually reflects earlier traffic rather than
       the state of the bus - which is why it must not fail the stage.
       Read up front so it is populated no matter where the stage bails
       out below. */
    g_test_errorn_sc =
        (HAL_GPIO_ReadPin(ERRORN_SC_GPIO_Port, ERRORN_SC_Pin) == GPIO_PIN_SET)
        ? 1u : 0u;

    /* --- CLK_SM_I2C ---------------------------------------------- */
    /* The ASICs I2C slave core is clocked from PE9. Without it the chip
       physically cannot answer, and a perfectly healthy bus reads as a
       dead one. TIM1 is an advanced timer, so a running counter is not
       enough: the channel output and the master output enable both have
       to be on or the pin stays flat. */
    g_test_clk_running =
        (((htim1.Instance->CR1  & TIM_CR1_CEN)   != 0u) &&
         ((htim1.Instance->CCER & TIM_CCER_CC1E) != 0u) &&
         ((htim1.Instance->BDTR & TIM_BDTR_MOE)  != 0u)) ? 1u : 0u;

    const uint32_t psc = htim1.Instance->PSC + 1u;
    const uint32_t arr = htim1.Instance->ARR + 1u;

    g_test_clk_hz = ((psc * arr) > 0u) ? (tim1_clk_hz() / (psc * arr)) : 0u;

    if (!g_test_clk_running) return RR2_TEST_REASON_NO_CLK;

    const uint32_t tol = (TEST_CLK_TARGET_HZ / 100u) * TEST_CLK_TOL_PCT;

    if ((g_test_clk_hz < (TEST_CLK_TARGET_HZ - tol)) ||
        (g_test_clk_hz > (TEST_CLK_TARGET_HZ + tol))) {
        return RR2_TEST_REASON_CLK_FREQ;
    }

    /* --- Reset lines --------------------------------------------- */
    /* All active low, so released means high. RSTN_READ is deliberately
       not checked - it gates the readout register, not Slow Control,
       and the DAQ is entitled to hold it. */
    g_test_resets_ok =
        ((HAL_GPIO_ReadPin(RESET_N_GPIO_Port,  RESET_N_Pin)  == GPIO_PIN_SET) &&
         (HAL_GPIO_ReadPin(RSTN_I2C_GPIO_Port, RSTN_I2C_Pin) == GPIO_PIN_SET) &&
         (HAL_GPIO_ReadPin(RSTN_SC_GPIO_Port,  RSTN_SC_Pin)  == GPIO_PIN_SET))
        ? 1u : 0u;

    if (!g_test_resets_ok) return RR2_TEST_REASON_RESET_HELD;

    /* --- The two wires ------------------------------------------- */
    /* Both are open drain with the pull-ups on the test board, so an
       idle bus sits high. Reading the input register works even though
       the pins are in alternate function mode, because IDR always
       reflects the real pin level.

       A line stuck low is worth catching here: the sweep would still
       run, but every one of its 384 probes would burn the full timeout
       and the answer would be "nothing answered" for a reason that has
       nothing to do with the ASIC. */
    g_test_scl_idle =
        (HAL_GPIO_ReadPin(TEST_SCL_PORT, TEST_SCL_PIN) == GPIO_PIN_SET) ? 1u : 0u;
    g_test_sda_idle =
        (HAL_GPIO_ReadPin(TEST_SDA_PORT, TEST_SDA_PIN) == GPIO_PIN_SET) ? 1u : 0u;

    if (!g_test_scl_idle) return RR2_TEST_REASON_SCL_LOW;
    if (!g_test_sda_idle) return RR2_TEST_REASON_SDA_LOW;

    /* --- The peripheral ------------------------------------------ */
    const HAL_I2C_StateTypeDef state = HAL_I2C_GetState(&hi2c1);

    g_test_i2c_state = (uint8_t)state;

    if (state != HAL_I2C_STATE_READY) return RR2_TEST_REASON_I2C_BUSY;

    return RR2_TEST_REASON_NONE;
}

/* ------------------------------------------------------------------ */
/* Stage 2 - CHIPID                                                    */
/* ------------------------------------------------------------------ */
/**
 * @brief Put one address on the wire and see whether anything ACKs.
 *
 * Address byte then STOP - no data byte is ever sent, which is what
 * makes the whole sweep free of side effects on the ASIC.
 *
 * @param addr8   8-bit HAL address
 * @param out_ns  wire time, only meaningful on an ACK
 * @retval 1 if the address ACKed
 */
static uint8_t probe(uint8_t addr8, uint32_t *out_ns)
{
    const uint32_t t0 = DWT->CYCCNT;

    const HAL_StatusTypeDef st =
        HAL_I2C_IsDeviceReady(&hi2c1, addr8, 1u, RR2_TEST_PROBE_MS);

    *out_ns = cyc_to_ns(DWT->CYCCNT - t0);

    g_test_probes++;

    if (st == HAL_OK) {
        g_test_acks++;
        return 1u;
    }

    /* Which KIND of "no" this was is the whole diagnosis when a sweep
       comes back empty. A fast NACK means the bus clocked all nine bits
       and nobody pulled SDA down - healthy wires, absent chip. A
       timeout or a BUSY means the transfer never completed at all, so
       the sweep result says nothing about who is on the bus. Without
       these three counters both look identical from g_test_acks == 0. */
    if      (st == HAL_ERROR)   g_test_nacks++;
    else if (st == HAL_TIMEOUT) g_test_timeouts++;
    else if (st == HAL_BUSY)    g_test_busy++;

    /* HAL_ERROR is the ordinary NACK - the expected answer for 15 of
       the 16 ids - and it leaves the peripheral clean, so it must not
       trigger the recovery below or the sweep would re-initialise 120
       times for nothing.

       The other two do need it. A timeout abandons the state machine
       mid-transfer, and a BUSY is the peripheral latching that the bus
       was not idle when the probe started; both persist, so without a
       re-init every probe after the first bad one would fail for that
       reason instead of its own and the whole sweep would read as an
       empty bus. Only cycling PE clears a stuck BUSY. */
    if ((st == HAL_TIMEOUT) || (st == HAL_BUSY)) {
        (void)HAL_I2C_DeInit(&hi2c1);
        MX_I2C1_Init();
        g_test_reinits++;
    }
    return 0u;
}

/**
 * @brief One full sweep of all 16 ids x 8 internal registers.
 *
 * @param regs  out, per id, bit R set = register R ACKed
 * @param any   out, bit N set = id N ACKed on at least one address
 * @retval bitmap of ids that ACKed on R0, R1 and R2 together
 */
static uint16_t sweep_once(uint8_t regs[16], uint16_t *any)
{
    uint16_t qualified = 0u;
    uint16_t answered  = 0u;

    for (uint8_t id = 0u; id < 16u; ++id) {
        uint8_t mask = 0u;

        /* Id 0 lands on I2C addresses 0x00..0x07, which the I2C spec
           reserves - 0x00 is the general call address. Probing them is
           harmless, and an ASIC strapped to 0 would be found nowhere
           else, so the sweep asks anyway and lets the report show what
           came back. */
        for (uint8_t reg = 0u; reg < 8u; ++reg) {
            uint32_t ns = 0u;

            if (probe(RR2_HAL_ADDR_OF(id, reg), &ns)) {
                mask |= (uint8_t)(1u << reg);

                /* Only an ACK carries wire time worth reading - a NACK
                   is mostly HAL state machine. Keep the first one. */
                if ((g_test_probe_ns == 0u) && (ns > 0u)) {
                    g_test_probe_ns = ns;
                }
            }
        }

        regs[id] = mask;

        if (mask != 0u) answered |= (uint16_t)(1u << id);

        if ((mask & RR2_TEST_REGS_REQUIRED) == RR2_TEST_REGS_REQUIRED) {
            qualified |= (uint16_t)(1u << id);
        }
    }

    *any = answered;
    return qualified;
}

/**
 * @brief Turn the one timed probe into a bus speed, for the report.
 *
 * Informational only. It times the HAL call, so it carries a few
 * percent of software overhead on top of the wire time and reads
 * slightly low - the scope stays the authority on both numbers.
 */
static void derive_bus_speed(void)
{
    if (g_test_probe_ns == 0u) return;

    g_test_scl_khz =
        (TEST_CLOCKS_PER_PROBE * 1000000u) / g_test_probe_ns;

    if ((g_test_scl_khz > 0u) && (g_test_clk_hz > 0u)) {
        g_test_ratio_x10 = ((g_test_clk_hz / 1000u) * 10u) / g_test_scl_khz;
    }
}

/**
 * @brief Sweep for the chip id, RR2_TEST_SWEEPS times over.
 *
 * The repeats are the point. One sweep tells you who answered; several
 * identical sweeps tell you the bus answers the same way every time,
 * which is the difference between a chip that works and a chip that is
 * about to produce intermittent Slow Control errors.
 *
 * @retval RR2_TEST_REASON_NONE when exactly one id answered, every time
 */
static RR2_TestReason stage_chipid(void)
{
    uint8_t  regs[16] = {0};
    uint16_t any      = 0u;
    uint16_t map      = 0u;
    uint16_t first    = 0u;
    uint16_t first_any = 0u;

    /* Everything else was cleared by the caller. Stability starts out
       true and can only be taken away by a sweep that disagrees. */
    g_test_stable = 1u;

    /* Wall time over all the sweeps. The single most telling number
       when nothing answers: 384 fast NACKs are tens of milliseconds,
       384 timeouts are nearly two seconds. */
    const uint32_t t_start = HAL_GetTick();

    for (uint8_t pass = 0u; pass < RR2_TEST_SWEEPS; ++pass) {
        map = sweep_once(regs, &any);

        if (pass == 0u) {
            first     = map;
            first_any = any;
        } else if ((map != first) || (any != first_any)) {
            /* Keep going rather than bailing out - the later sweeps
               still populate the maps, and seeing WHICH id came and
               went is more useful than an early exit. */
            g_test_stable = 0u;
        }
    }

    g_test_sweep_ms     = HAL_GetTick() - t_start;
    g_test_chipid_map   = map;
    g_test_any_map      = any;
    g_test_chipid_count = popcount16(map);

    for (uint8_t id = 0u; id < 16u; ++id) {
        g_test_reg_mask[id] = regs[id];
    }

    /* Lowest qualified id. Which one it is only matters when several
       answered, and that is a failure in its own right. */
    g_test_chipid = RR2_CHIP_ID_NONE;
    for (uint8_t id = 0u; id < 16u; ++id) {
        if ((map & (uint16_t)(1u << id)) != 0u) {
            g_test_chipid = id;
            break;
        }
    }

    derive_bus_speed();

    /* Instability is reported ahead of everything else on purpose. The
       maps hold the LAST sweep, so a chip that answered twice and then
       vanished would otherwise be reported as simply absent - and "it
       comes and goes" is a completely different fault from "it is not
       there" to whoever has to fix it. */
    if (!g_test_stable) return RR2_TEST_REASON_UNSTABLE;

    if (g_test_chipid_count == 0u) {
        /* Something ACKed, just not on the three registers a Slow
           Control write needs. Worth its own reason: it means the bus
           and the addressing are alive and the chip is not. */
        return (any != 0u) ? RR2_TEST_REASON_PARTIAL
                           : RR2_TEST_REASON_NO_ANSWER;
    }
    if (g_test_chipid_count > 1u) return RR2_TEST_REASON_MULTI;

    return RR2_TEST_REASON_NONE;
}

/* ------------------------------------------------------------------ */
/* Report                                                              */
/* ------------------------------------------------------------------ */
static const char *reason_text(uint8_t reason)
{
    switch (reason) {
    case RR2_TEST_REASON_NONE:        return "ok";
    case RR2_TEST_REASON_NO_CLK:      return "CLK_SM_I2C not running (PE9/TIM1)";
    case RR2_TEST_REASON_CLK_FREQ:    return "CLK_SM_I2C off target (need 2 MHz)";
    case RR2_TEST_REASON_RESET_HELD:  return "a reset line is still asserted";
    case RR2_TEST_REASON_SCL_LOW:     return "SCL stuck low";
    case RR2_TEST_REASON_SDA_LOW:     return "SDA stuck low";
    case RR2_TEST_REASON_I2C_BUSY:    return "I2C1 not idle";
    case RR2_TEST_REASON_NO_ANSWER:   return "no chip id answered";
    case RR2_TEST_REASON_PARTIAL:     return "answered, but not on R0/R1/R2";
    case RR2_TEST_REASON_MULTI:       return "more than one chip id answered";
    case RR2_TEST_REASON_UNSTABLE:    return "answer changed between sweeps";
    default:                          return "?";
    }
}

/**
 * @brief Print what the run found, one line per thing worth knowing.
 *
 * The per-id lines are only emitted for ids that answered on something,
 * so a healthy board prints one of them and a dead board prints none -
 * either way the report stays short enough to read at a glance.
 */
static void report(void)
{
    say("[i2c] ---- RADIOROC2 Slow Control health check ----");

    (void)snprintf(s_line, sizeof s_line,
                   "[i2c] bus    clk=%luHz run=%u scl=%u sda=%u rst=%u errn=%u",
                   (unsigned long)g_test_clk_hz,
                   (unsigned)g_test_clk_running, (unsigned)g_test_scl_idle,
                   (unsigned)g_test_sda_idle,    (unsigned)g_test_resets_ok,
                   (unsigned)g_test_errorn_sc);
    say(s_line);

    if (g_test_stage >= RR2_TEST_STAGE_CHIPID) {
        (void)snprintf(s_line, sizeof s_line,
                       "[i2c] sweep  probes=%lu acks=%lu sweeps=%u stable=%u",
                       (unsigned long)g_test_probes,
                       (unsigned long)g_test_acks,
                       (unsigned)RR2_TEST_SWEEPS,
                       (unsigned)g_test_stable);
        say(s_line);

        (void)snprintf(s_line, sizeof s_line,
                       "[i2c] why    nack=%lu tmo=%lu busy=%lu reinit=%lu %lums",
                       (unsigned long)g_test_nacks,
                       (unsigned long)g_test_timeouts,
                       (unsigned long)g_test_busy,
                       (unsigned long)g_test_reinits,
                       (unsigned long)g_test_sweep_ms);
        say(s_line);

        for (uint8_t id = 0u; id < 16u; ++id) {
            if (g_test_reg_mask[id] == 0u) continue;

            (void)snprintf(s_line, sizeof s_line,
                           "[i2c] id %2u  regs=0x%02X %s",
                           (unsigned)id, (unsigned)g_test_reg_mask[id],
                           ((g_test_reg_mask[id] & RR2_TEST_REGS_REQUIRED)
                              == RR2_TEST_REGS_REQUIRED)
                             ? "<- RADIOROC2" : "(incomplete)");
            say(s_line);
        }

        if (g_test_scl_khz > 0u) {
            (void)snprintf(s_line, sizeof s_line,
                           "[i2c] speed  probe=%luns scl~%lukHz ratio=%lu/200",
                           (unsigned long)g_test_probe_ns,
                           (unsigned long)g_test_scl_khz,
                           (unsigned long)g_test_ratio_x10);
            say(s_line);
        }
    }

    say(s_result);
}

/* ------------------------------------------------------------------ */
/* Entry points                                                        */
/* ------------------------------------------------------------------ */
uint8_t RR2_TestI2C_Run(void)
{
    RR2_TestReason reason;

    g_test_verdict = RR2_TEST_PENDING;
    g_test_reason  = RR2_TEST_REASON_NONE;
    g_test_stage   = RR2_TEST_STAGE_BUS;

    /* Wipe every result before anything else. A run that bails out
       early must not leave the previous run figures sitting there
       looking like fresh ones - on a health check that is the one kind
       of stale reading that would actually mislead someone. Stage 1
       bails at the first thing it finds wrong, so most of these would
       otherwise keep whatever the last healthy run put there. */
    g_test_clk_hz       = 0u;
    g_test_clk_running  = 0u;
    g_test_scl_idle     = 0u;
    g_test_sda_idle     = 0u;
    g_test_resets_ok    = 0u;
    g_test_errorn_sc    = 0u;
    g_test_i2c_state    = 0u;

    g_test_chipid       = RR2_CHIP_ID_NONE;
    g_test_chipid_map   = 0u;
    g_test_any_map      = 0u;
    g_test_chipid_count = 0u;
    g_test_stable       = 0u;
    g_test_probes       = 0u;
    g_test_acks         = 0u;
    g_test_nacks        = 0u;
    g_test_timeouts     = 0u;
    g_test_busy         = 0u;
    g_test_reinits      = 0u;
    g_test_sweep_ms     = 0u;
    g_test_probe_ns     = 0u;
    g_test_scl_khz      = 0u;
    g_test_ratio_x10    = 0u;

    for (uint8_t id = 0u; id < 16u; ++id) {
        g_test_reg_mask[id] = 0u;
    }

    /* The timing figures are a nicety; the test has to work without
       them, so a dead cycle counter just leaves them at zero. */
    const uint8_t timed = dwt_ready();

    reason = stage_bus();

    if (reason == RR2_TEST_REASON_NONE) {
        g_test_stage = RR2_TEST_STAGE_CHIPID;
        reason = stage_chipid();
    }

    if (!timed) {
        g_test_probe_ns  = 0u;
        g_test_scl_khz   = 0u;
        g_test_ratio_x10 = 0u;
    }

    g_test_reason  = (uint8_t)reason;
    g_test_verdict = (reason == RR2_TEST_REASON_NONE) ? RR2_TEST_PASS
                                                      : RR2_TEST_FAIL;

    if (g_test_verdict == RR2_TEST_PASS) {
        (void)snprintf(s_result, sizeof s_result,
                       "[i2c] PASS   chip id %u%s",
                       (unsigned)g_test_chipid,
                       g_test_apply ? " (driver now targets it)" : "");

        /* Point the driver at what was found, so the configuration push
           and the DAQ afterwards address the chip that actually exists
           rather than the compile-time default. */
        if (g_test_apply) {
            RR2_SetChipId(g_test_chipid);
        }
    } else {
        (void)snprintf(s_result, sizeof s_result,
                       "[i2c] FAIL   stage %u: %s",
                       (unsigned)g_test_stage,
                       reason_text(g_test_reason));
    }

    g_test_runs++;
    report();

    return g_test_verdict;
}

void RR2_TestI2C_Task(void)
{
    if (!g_test_run && !g_test_repeat) return;

    g_test_run = 0u;   /* one-shot; g_test_repeat is what re-arms it */

    (void)RR2_TestI2C_Run();
}

const char *RR2_TestI2C_ResultText(void)
{
    return s_result;
}
