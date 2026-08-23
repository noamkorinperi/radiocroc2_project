/**
 ******************************************************************************
 * @file    rr2_test_i2c.h
 * @brief   Slow Control I2C health check - staged, self-reporting.
 *
 * WHAT THIS IS FOR
 * The signals themselves have already been verified on a scope, so this
 * test does not measure edges. It asks the next question: with all the
 * wires proven good, does the RADIOROC2 actually answer, and who is it?
 *
 * THE STAGES
 * They run in order and the run stops at the first one that fails, so
 * g_test_stage always names the thing that needs attention.
 *
 *   1  BUS     Preconditions. None of them touch the wire - they read
 *              back the state the firmware believes it set up. The point
 *              is that a stage-2 failure should never be ambiguous: if
 *              the clock is dead or a reset is still asserted, you learn
 *              that here instead of guessing at a silent bus.
 *
 *   2  CHIPID  Who is on the bus. CHIP_ID<3:0> is strapped in copper on
 *              the test board and there is no register to read it from,
 *              because the id IS the top half of the I2C address. The
 *              only way to learn it is to ask all 16 and see which one
 *              answers, which is exactly what this stage does.
 *
 * More stages belong after CHIPID - a register write with read-back, the
 * R7 status flags - but they are only meaningful once the chip has been
 * positively identified, so they are deliberately not here yet.
 *
 * WHY THE CHIPID STAGE IS SAFE TO RUN AT ANY TIME
 * It never writes. Every probe is an address byte followed by STOP, so
 * no data byte ever reaches the ASIC and no configuration can be
 * disturbed - not even the sub-address latch. Re-running it in the
 * middle of a session cannot cost you the chip state.
 *
 * HOW THE SWEEP DECIDES
 * Each of the 16 candidate ids owns 8 consecutive I2C addresses, one per
 * internal register R0..R7. All 8 are probed and the ACK pattern is kept
 * in g_test_reg_mask[id]. An id counts as a RADIOROC2 only if R0, R1 and
 * R2 all answer - those three are what a Slow Control write is made of,
 * so a chip that misses any of them cannot be driven anyway. Demanding
 * three specific consecutive addresses also makes a false positive from
 * some other device very unlikely.
 *
 * The whole sweep is then repeated RR2_TEST_SWEEPS times and the answer
 * must come out identical every time. An ASIC that answers on one sweep
 * and not the next is not a working ASIC, and that is precisely the
 * failure a marginal pull-up or a borderline clock ratio produces - it
 * would otherwise surface much later as random Slow Control errors.
 *
 * HOW TO RUN IT
 * It runs once by itself at start-up, before the ASIC is configured, and
 * the result decides whether the configuration is attempted at all.
 *
 * To run it again without resetting the board, from Live Expressions
 * (Window > Show View > Live Expressions):
 *
 *   g_test_run     write 1 - runs the whole sequence once, clears itself
 *   g_test_repeat  1 = keep re-running, for watching an intermittent bus
 *   g_test_apply   1 = adopt the id found (default), 0 = only report it
 *   g_test_verbose 1 = print the report to the host link too (default)
 *
 * WHAT TO READ AFTERWARDS
 * Start with these three. They are the whole result:
 *
 *   g_test_verdict  RR2_TEST_PASS / RR2_TEST_FAIL
 *   g_test_stage    which stage the run got to
 *   g_test_reason   why it stopped, RR2_TEST_REASON_*
 *
 * then the per-stage detail listed further down. With g_test_verbose set
 * the same thing arrives on the host link as plain text, which is easier
 * to read and easy to paste into a log.
 ******************************************************************************
 */
#ifndef RR2_TEST_I2C_H
#define RR2_TEST_I2C_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Tunables                                                            */
/* ------------------------------------------------------------------ */
/* How many times the 16-id sweep is repeated. The sweeps must all
   agree, so this is the depth of the stability check rather than a
   retry count - raising it makes the test stricter, not more lenient. */
#ifndef RR2_TEST_SWEEPS
#define RR2_TEST_SWEEPS         3u
#endif

/* Per-probe timeout, ms. A probe is 9 SCL clocks - 90 us at 100 kHz -
   and a chip that is not there NACKs at the end of the address byte
   rather than running the clock out, so this only ever elapses when
   the bus itself is stuck. Kept small so a dead bus fails quickly
   instead of spending 128 full timeouts on it. */
#ifndef RR2_TEST_PROBE_MS
#define RR2_TEST_PROBE_MS       5u
#endif

/* The internal registers a Slow Control write is built from: R0
   sub-address, R1 address, R2 data. All three must ACK for an id to
   count as a working chip. */
#define RR2_TEST_REGS_REQUIRED  0x07u

/* ------------------------------------------------------------------ */
/* Verdicts and stages                                                 */
/* ------------------------------------------------------------------ */
#define RR2_TEST_PENDING        0u   /* not run yet, or still running  */
#define RR2_TEST_PASS           1u
#define RR2_TEST_FAIL           2u

#define RR2_TEST_STAGE_NONE     0u
#define RR2_TEST_STAGE_BUS      1u
#define RR2_TEST_STAGE_CHIPID   2u

/**
 * @brief Why a run stopped. Ordered by stage, so the number alone tells
 *        you roughly where to look.
 */
typedef enum {
    RR2_TEST_REASON_NONE = 0,   /* passed                               */

    /* Stage 1 - BUS */
    RR2_TEST_REASON_NO_CLK,     /* CLK_SM_I2C is not being emitted      */
    RR2_TEST_REASON_CLK_FREQ,   /* it runs, but not at 20 x SCL         */
    RR2_TEST_REASON_RESET_HELD, /* a reset line is still asserted low   */
    RR2_TEST_REASON_SCL_LOW,    /* SCL stuck low - no pull-up, or held  */
    RR2_TEST_REASON_SDA_LOW,    /* SDA stuck low - a slave mid-byte     */
    RR2_TEST_REASON_I2C_BUSY,   /* the peripheral is not idle           */

    /* Stage 2 - CHIPID */
    RR2_TEST_REASON_NO_ANSWER,  /* nothing on the bus answered at all   */
    RR2_TEST_REASON_PARTIAL,    /* something answered, but not R0/R1/R2 */
    RR2_TEST_REASON_MULTI,      /* more than one id qualified           */
    RR2_TEST_REASON_UNSTABLE    /* the sweeps disagreed with each other */
} RR2_TestReason;

/* ------------------------------------------------------------------ */
/* Controls - write these from Live Expressions                        */
/* ------------------------------------------------------------------ */
extern volatile uint8_t g_test_run;      /* 1 = run once, self-clearing */
extern volatile uint8_t g_test_repeat;   /* 1 = run continuously        */
extern volatile uint8_t g_test_apply;    /* 1 = adopt the id found      */
extern volatile uint8_t g_test_verbose;  /* 1 = print over the host link*/

/* ------------------------------------------------------------------ */
/* Overall result                                                      */
/* ------------------------------------------------------------------ */
extern volatile uint8_t  g_test_verdict; /* RR2_TEST_PASS / _FAIL       */
extern volatile uint8_t  g_test_stage;   /* RR2_TEST_STAGE_*            */
extern volatile uint8_t  g_test_reason;  /* RR2_TestReason              */
extern volatile uint32_t g_test_runs;    /* completed runs, a liveness  */

/* ------------------------------------------------------------------ */
/* Stage 1 - BUS                                                       */
/* ------------------------------------------------------------------ */
/* CLK_SM_I2C as the timer is actually programmed, not as the comments
   claim. The ASIC needs 20 x SCL, so 2 MHz against a 100 kHz bus.     */
extern volatile uint32_t g_test_clk_hz;
/* TIM1 is an advanced timer: a running counter is not enough, the
   channel output and the master output enable have to be on as well,
   and a missing MOE is the classic cause of a flat line on an
   otherwise correct PWM setup. 1 only when all three are set.         */
extern volatile uint8_t  g_test_clk_running;
extern volatile uint8_t  g_test_scl_idle;    /* PB8 reads high = good  */
extern volatile uint8_t  g_test_sda_idle;    /* PB9 reads high = good  */
extern volatile uint8_t  g_test_resets_ok;   /* all reset lines high   */
extern volatile uint8_t  g_test_errorn_sc;   /* PD3, 0 = ASIC flagging */
extern volatile uint8_t  g_test_i2c_state;   /* HAL_I2C_STATE_*        */

/* ------------------------------------------------------------------ */
/* Stage 2 - CHIPID                                                    */
/* ------------------------------------------------------------------ */
/* The answer. 0xFF when nothing qualified.                            */
extern volatile uint8_t  g_test_chipid;
/* bit N set = id N answered on R0, R1 and R2, i.e. a usable chip.     */
extern volatile uint16_t g_test_chipid_map;
/* bit N set = id N answered on at least one of its 8 addresses. Wider
   than the map above on purpose: an id that shows up here but not
   there is something on the bus that is not a working RADIOROC2, and
   that distinction is the difference between "wrong chip" and "half
   dead chip".                                                         */
extern volatile uint16_t g_test_any_map;
/* How many ids qualified. Anything other than 1 is a failure.         */
extern volatile uint8_t  g_test_chipid_count;
/* Per id, which of R0..R7 ACKed - bit 0 = R0. The raw evidence behind
   the two maps above, and the first thing to look at when an id lands
   in g_test_any_map but not in g_test_chipid_map.                     */
extern volatile uint8_t  g_test_reg_mask[16];
/* 1 when every sweep produced the same map. 0 means an intermittent
   bus, which is a failure even if a chip was found.                   */
extern volatile uint8_t  g_test_stable;
/* Total address probes issued, and how many of them ACKed. On a healthy
   single-chip board that is 16 x 8 x RR2_TEST_SWEEPS probes and
   8 x RR2_TEST_SWEEPS acks.                                           */
extern volatile uint32_t g_test_probes;
extern volatile uint32_t g_test_acks;

/* What the probes that did NOT ack actually did. Read these first
   whenever g_test_acks is 0, because "nothing answered" has two
   completely different causes that look identical without them:
     g_test_nacks     the bus clocked all nine bits and nobody pulled
                      SDA down. Healthy wires, absent or unpowered
                      chip. On an empty bus this equals g_test_probes.
     g_test_timeouts  the transfer never finished. The wires, not the
                      chip, are the problem - the sweep result says
                      nothing about who is out there.
     g_test_busy      the peripheral found the bus non-idle before it
                      even started. Usually a slave holding SDA low.
     g_test_reinits   how often I2C1 had to be cycled to recover.
     g_test_sweep_ms  wall time for all the sweeps. The quickest tell
                      of all: fast NACKs are tens of ms, a sweep full
                      of timeouts is close to two seconds.            */
extern volatile uint32_t g_test_nacks;
extern volatile uint32_t g_test_timeouts;
extern volatile uint32_t g_test_busy;
extern volatile uint32_t g_test_reinits;
extern volatile uint32_t g_test_sweep_ms;

/* Informational, measured on an ACKing probe only - a NACK is mostly
   HAL overhead and would read as nonsense. Derived from software
   timing, so it carries a few percent of call overhead and reads
   slightly low. The scope is the authority; these are here so that a
   report from a board you cannot reach still says something about the
   bus speed.
     g_test_probe_ns   wire time of one 9-clock address probe
     g_test_scl_khz    the same, as a frequency
     g_test_ratio_x10  CLK_SM_I2C : SCL, x10. The ASIC needs 20, so
                       200 is the target and below 200 means the chip
                       is having its own I2C core clocked too slowly. */
extern volatile uint32_t g_test_probe_ns;
extern volatile uint32_t g_test_scl_khz;
extern volatile uint32_t g_test_ratio_x10;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/**
 * @brief Run the whole sequence once, synchronously.
 *
 * Takes a few milliseconds on a healthy board: the sweep is address
 * probes only, and a chip that is not there NACKs immediately rather
 * than running the timeout out. A stuck bus is the slow case, and
 * stage 1 catches that before the sweep starts.
 *
 * On a pass, and with g_test_apply set, the driver is pointed at the id
 * that was found so everything afterwards - the configuration push, the
 * DAQ - talks to the right chip.
 *
 * @retval RR2_TEST_PASS or RR2_TEST_FAIL
 */
uint8_t RR2_TestI2C_Run(void);

/**
 * @brief Main-loop hook. Does nothing until g_test_run or g_test_repeat
 *        is set from the debugger, then runs the sequence.
 */
void RR2_TestI2C_Task(void);

/**
 * @brief One-line summary of the last run, for logging.
 * @retval a pointer to a static string, never NULL
 */
const char *RR2_TestI2C_ResultText(void);

#endif /* RR2_TEST_I2C_H */
