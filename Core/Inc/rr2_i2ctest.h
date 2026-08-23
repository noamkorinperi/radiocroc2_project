/**
 ******************************************************************************
 * @file    rr2_i2ctest.h
 * @brief   One command that answers "do the STM32 and the ASIC talk?"
 *
 * WHAT IT IS
 * Not another scope aid. This runs the real Slow Control protocol against
 * the real chip and reports, in one pass, whether the link works well
 * enough to build the host tools on top of.
 *
 * It is meant to be run from the host console (`i2ctest`) and read by the
 * Python side, so everything it learns comes out over the link as
 * key=value lines. No debugger, no Live Expressions, no breakpoints.
 *
 * WHAT IT CHECKS, IN ORDER
 *   1. The bus is usable at all - pads configured, peripheral enabled,
 *      both lines idle high. A line held low makes every later result
 *      meaningless, so the test stops there and says so.
 *   2. A full 7-bit address scan, 0x08..0x77. Answers "does anything
 *      answer" and "is CHIP_ID what we assumed" in one sweep. The ASIC
 *      occupies eight consecutive addresses - one per internal register -
 *      so an aligned run of eight is its signature, and the chip id falls
 *      straight out of where that run starts.
 *   3. The real three-frame Slow Control round trip, repeated, with the
 *      write path and the read path counted SEPARATELY. Writes working
 *      while reads do not is a completely different situation from
 *      nothing working, and radioroc2_ctrl.h notes the read path has
 *      never been confirmed on hardware - so the two must not be
 *      collapsed into one pass/fail.
 *
 * THE SECOND PASS
 * If nothing answers at the configured speed, the whole sequence is run
 * again with CLK_SM_I2C and SCL scaled down together by ten - 200 kHz and
 * 10 kHz, which keeps the 20:1 ratio the ASIC needs. This is not a
 * separate experiment to go and run; it is the same test covering the
 * other hypothesis in the same flash cycle, and it costs about a second.
 * The reason it is worth the second: scope captures taken on 2026-08-23
 * show CLK_SM_I2C arriving at the ASIC at only 1.04 V, rounded and
 * ringing, while SCL and SDA arrive at 1.38 V. A passive open-drain
 * level translator cannot slew 2 MHz through its pull-up; at 200 kHz it
 * has ten times as long and should make it. If pass 2 talks and pass 1
 * does not, that is the answer, and the fix is a push-pull translator on
 * the clock.
 *
 * SAFETY
 * The round trip writes to the common block's hold-delay byte (address
 * 65, subadd 8) and restores it from the RAM shadow afterwards, so a run
 * leaves the ASIC configuration exactly as it found it. The clock and
 * timing registers are likewise restored on the way out, whichever pass
 * succeeded.
 ******************************************************************************
 */
#ifndef RR2_I2CTEST_H
#define RR2_I2CTEST_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Verdicts                                                            */
/* ------------------------------------------------------------------ */
/* Reported as i2ctest.verdict, both as a number and as a name.        */
typedef enum {
    RR2_I2CT_OK            = 0,  /* writes and reads both round-trip   */
    RR2_I2CT_WRITE_ONLY    = 1,  /* the ASIC ACKs writes, reads fail   */
    RR2_I2CT_NO_DEVICE     = 2,  /* nothing ACKed at any address       */
    RR2_I2CT_WRONG_ID      = 3,  /* something answered, not at our id  */
    RR2_I2CT_SCL_STUCK_LOW = 4,  /* SCL held low - bus unusable        */
    RR2_I2CT_SDA_STUCK_LOW = 5,  /* SDA held low - bus jammed          */
    RR2_I2CT_BUS_NOT_READY = 6   /* pads or peripheral not configured  */
} RR2_I2CTestVerdict;

/**
 * @brief Run the whole test and print the report over the host link.
 *
 * Blocks for up to about two seconds (two full address scans plus the
 * round trips). Call from the main loop, never from an ISR - it uses the
 * blocking HAL I2C calls and pumps the output ring as it goes.
 *
 * @return the verdict, also printed as i2ctest.verdict.
 */
RR2_I2CTestVerdict RR2_I2CTest_Run(void);

#endif /* RR2_I2CTEST_H */
