/**
 ******************************************************************************
 * @file    usb_stream.h
 * @brief   Event streaming to the host over USB CDC (virtual COM port).
 *
 * Two output formats share one transmit path:
 *
 *   BINARY (default) - framed, compact, CRC protected. A full 64-channel
 *                      event is 277 bytes, so 1 kHz of events is about
 *                      277 kB/s, comfortable for USB Full Speed.
 *   TEXT             - human readable CSV, for bring-up only. The same
 *                      event is roughly 1.3 kB, so it only keeps up at
 *                      low rates. Handy with any serial terminal.
 *
 * Binary frame layout (little endian):
 *
 *   off  size  field
 *   0    2     sync, 0xA5 0x5A
 *   2    1     frame type (see USBStream_FrameType)
 *   3    2     payload length
 *   5    N     payload
 *   5+N  2     CRC16-CCITT over bytes [2 .. 4+N]
 *
 * Event payload:
 *   0   4   sequence number
 *   4   4   timestamp, ms since boot
 *   8   4   temperature, milli-Celsius (signed)
 *   12  1   first channel
 *   13  1   channel count
 *   14  2*count  high-gain ADC codes
 *   ..  2*count  low-gain ADC codes
 *
 * Transmission is buffered. CDC_Transmit_FS refuses a new transfer while
 * one is in flight, so frames are queued in a ring and pumped out by
 * USBStream_Task(). If the host stops reading the ring fills, and new
 * frames are DROPPED rather than blocking the DAQ - the drop counter
 * makes that visible instead of silently stretching dead time.
 ******************************************************************************
 */
#ifndef USB_STREAM_H
#define USB_STREAM_H

#include "radioroc2_daq.h"
#include <stdint.h>

/* Ring size. 8 KB holds roughly 30 full events of burst. */
#ifndef USBSTREAM_RING_SIZE
#define USBSTREAM_RING_SIZE     8192u
#endif

/* Bytes handed to the USB stack per Task() call. */
#ifndef USBSTREAM_CHUNK_MAX
#define USBSTREAM_CHUNK_MAX     512u
#endif

#define USBSTREAM_SYNC0         0xA5u
#define USBSTREAM_SYNC1         0x5Au

typedef enum {
    USBSTREAM_FRAME_EVENT  = 1,
    USBSTREAM_FRAME_STATUS = 2
} USBStream_FrameType;

typedef enum {
    USBSTREAM_FMT_BINARY = 0,   /* default */
    USBSTREAM_FMT_TEXT   = 1
} USBStream_Format;

/** Status snapshot sent alongside the event stream. */
typedef struct {
    uint32_t uptime_ms;
    uint32_t trigger_count;
    uint32_t events_ok;
    uint32_t events_bad;
    uint32_t frames_dropped;
    int32_t  temp_milli_c;
    uint8_t  rr2_online;
    uint8_t  temp_online;
    uint8_t  timing_ok;
    uint8_t  cfg_status;      /* RR2_Status of the config sequence  */
    uint8_t  read_status;     /* RR2_Status of the last readout     */
} USBStream_Status;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

/** Reset the ring and counters. Call once at start-up. */
void USBStream_Init(void);

/** Select the wire format. Safe to call at runtime. */
void USBStream_SetFormat(USBStream_Format fmt);
USBStream_Format USBStream_GetFormat(void);

/** Queue one digitised event.
 *  @return 1 if queued, 0 if dropped for lack of buffer space.       */
uint8_t USBStream_SendEvent(const RR2_Event *evt,
                            int32_t temp_milli_c,
                            uint32_t timestamp_ms);

/** Queue a status frame. @return 1 if queued, 0 if dropped.          */
uint8_t USBStream_SendStatus(const USBStream_Status *st);

/** Queue a plain text line (always sent verbatim, both formats).
 *  Useful for log messages during bring-up.                          */
uint8_t USBStream_SendText(const char *str);

/** Pump queued bytes into the USB stack. Call every main-loop pass.  */
void USBStream_Task(void);

/** Number of frames discarded because the ring was full.             */
uint32_t USBStream_GetDropped(void);

/** Bytes currently waiting to be sent - useful to spot backpressure. */
uint32_t USBStream_GetPending(void);

#endif /* USB_STREAM_H */
