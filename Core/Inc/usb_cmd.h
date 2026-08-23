/**
 ******************************************************************************
 * @file    usb_cmd.h
 * @brief   Line-based command interface over USB CDC.
 *
 * Configuration traffic is tiny and interactive, so it is plain text:
 * type commands in any serial terminal, or script them from Python.
 * Only the event stream is binary.
 *
 * THREADING
 * CDC_Receive_FS runs in USB interrupt context, where blocking I2C is
 * not allowed. USBCmd_Feed() therefore only copies bytes into a ring
 * (ISR safe), and USBCmd_Task() does the parsing and the ASIC writes
 * from the main loop.
 *
 * COMMAND SET
 *   help                       list commands
 *   stat                       print counters and health flags
 *   fmt bin|txt                event stream format
 *
 *   ch <n|all> indac <0-255>           SiPM overvoltage trim
 *   ch <n|all> gain <lg 0-15> <hg 0-15>   charge preamp gains
 *   ch <n|all> tau  <lg 0-15> <hg 0-15>   shaper peaking time index
 *   ch <n|all> slow <lg 0|1> <hg 0|1>     120 ns shaping steps
 *   ch <n|all> patgain <0-63>          time preamp gain
 *   ch <n|all> trim <t1 0-63> <t2 0-63>   per-channel threshold trim
 *   ch <n|all> on|off                  enable / disable the channel
 *   ch <n|all> ctest <0|1> <use_cap 0|1>  internal charge injection
 *   ch <n> dump                        show the shadow for one channel
 *
 *   th <dac1> <dac2> <dacq>            global thresholds, 0..1023
 *   delay <0-255> <slope 0-15>         peak-detector hold delay
 *   trig <0-15>                        selTrig[3:0]
 *   hold int|ext                       hold source
 *   mux <hg 0|1> <lg 0|1>              analog mux buffer power
 *
 *   preset csi                         starting point for CsI(Tl)
 *   defaults                           reload datasheet defaults
 *   push                               rewrite the whole shadow
 *
 *   w <addr> <sub> <data>              raw Slow Control write
 *   r <addr> <sub>                     raw Slow Control read
 *
 *   i2ctest                            Slow Control link test: address
 *                                      scan + repeated write/read/verify,
 *                                      reported as i2ctest.* key=value
 *                                      lines. See rr2_i2ctest.h.
 ******************************************************************************
 */
#ifndef USB_CMD_H
#define USB_CMD_H

#include <stdint.h>

#ifndef USBCMD_RX_RING_SIZE
#define USBCMD_RX_RING_SIZE   256u
#endif

#ifndef USBCMD_LINE_MAX
#define USBCMD_LINE_MAX        80u
#endif

/** Clear the receive ring and line buffer. */
void USBCmd_Init(void);

/** Push received bytes. ISR safe - call from CDC_Receive_FS. */
void USBCmd_Feed(const uint8_t *data, uint32_t len);

/** Parse and execute complete lines. Call from the main loop. */
void USBCmd_Task(void);

/** Lines discarded because the receive ring overflowed. */
uint32_t USBCmd_GetOverruns(void);

#endif /* USB_CMD_H */
