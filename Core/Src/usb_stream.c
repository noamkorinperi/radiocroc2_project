/**
 ******************************************************************************
 * @file    usb_stream.c
 * @brief   Event streaming to the host over USB CDC.
 * @note    See usb_stream.h for the frame layout.
 *
 * Stack note: the F722 project links with a 1 KB main stack, so this
 * module never builds a whole frame in a local array. Text formatting
 * goes through a 24-byte scratch buffer, one number at a time.
 ******************************************************************************
 */
#include "usb_stream.h"
#include "usbd_cdc_if.h"     /* CDC_Transmit_FS */
#include "usb_device.h"      /* MX_USB_DEVICE_Init */
#include "usbd_core.h"       /* USBD_HandleTypeDef, USBD_STATE_CONFIGURED */
#include <stdio.h>
#include <string.h>

/* hUsbDeviceFS is defined in usb_device.c. Depending on the CubeMX
   version its extern declaration lives in usb_device.h or usbd_cdc_if.h,
   so declare it explicitly here to be independent of that choice. */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* ------------------------------------------------------------------ */
/* Ring buffer                                                         */
/* ------------------------------------------------------------------ */
static uint8_t  ring[USBSTREAM_RING_SIZE];
static volatile uint32_t ring_head = 0u;   /* next write position */
static volatile uint32_t ring_tail = 0u;   /* next read  position */

static uint32_t frames_dropped = 0u;
static USBStream_Format tx_format = USBSTREAM_FMT_BINARY;

static uint32_t ring_used(void)
{
    return (ring_head - ring_tail) & (uint32_t)(USBSTREAM_RING_SIZE - 1u);
}

static uint32_t ring_free(void)
{
    /* One slot is always kept empty so head == tail means "empty". */
    return (uint32_t)(USBSTREAM_RING_SIZE - 1u) - ring_used();
}

/** Unchecked push - callers must reserve space first. */
static void ring_put(uint8_t b)
{
    ring[ring_head] = b;
    ring_head = (ring_head + 1u) & (uint32_t)(USBSTREAM_RING_SIZE - 1u);
}

/* ------------------------------------------------------------------ */
/* CRC16-CCITT (poly 0x1021, init 0xFFFF)                              */
/* ------------------------------------------------------------------ */
static uint16_t crc_acc = 0xFFFFu;

static void crc_reset(void)
{
    crc_acc = 0xFFFFu;
}

static void crc_update(uint8_t b)
{
    crc_acc ^= (uint16_t)((uint16_t)b << 8);
    for (uint8_t i = 0u; i < 8u; ++i) {
        crc_acc = (crc_acc & 0x8000u) ? (uint16_t)((crc_acc << 1) ^ 0x1021u)
                                      : (uint16_t)(crc_acc << 1);
    }
}

/** Push one byte and fold it into the running CRC. */
static void put_crc(uint8_t b)
{
    crc_update(b);
    ring_put(b);
}

static void put_u16_crc(uint16_t v)
{
    put_crc((uint8_t)(v & 0xFFu));
    put_crc((uint8_t)(v >> 8));
}

static void put_u32_crc(uint32_t v)
{
    put_crc((uint8_t)(v & 0xFFu));
    put_crc((uint8_t)((v >> 8) & 0xFFu));
    put_crc((uint8_t)((v >> 16) & 0xFFu));
    put_crc((uint8_t)((v >> 24) & 0xFFu));
}

/** Emit sync + type + length and start the CRC over type/length. */
static void put_header(uint8_t type, uint16_t payload_len)
{
    ring_put(USBSTREAM_SYNC0);      /* sync is outside the CRC */
    ring_put(USBSTREAM_SYNC1);
    crc_reset();
    put_crc(type);
    put_u16_crc(payload_len);
}

static void put_footer(void)
{
    const uint16_t c = crc_acc;
    ring_put((uint8_t)(c & 0xFFu));
    ring_put((uint8_t)(c >> 8));
}

/* ------------------------------------------------------------------ */
/* Text helpers (no large stack buffers)                               */
/* ------------------------------------------------------------------ */
static uint8_t text_push(const char *s)
{
    const uint32_t n = (uint32_t)strlen(s);
    if (ring_free() < n) return 0u;
    for (uint32_t i = 0u; i < n; ++i) {
        ring_put((uint8_t)s[i]);
    }
    return 1u;
}

static uint8_t text_push_i32(int32_t v)
{
    char tmp[16];
    (void)snprintf(tmp, sizeof(tmp), "%ld", (long)v);
    return text_push(tmp);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
void USBStream_Init(void)
{
    ring_head      = 0u;
    ring_tail      = 0u;
    frames_dropped = 0u;
    tx_format      = USBSTREAM_FMT_BINARY;
}

void USBStream_SetFormat(USBStream_Format fmt)
{
    tx_format = fmt;
}

USBStream_Format USBStream_GetFormat(void)
{
    return tx_format;
}

uint32_t USBStream_GetDropped(void)
{
    return frames_dropped;
}

uint32_t USBStream_GetPending(void)
{
    return ring_used();
}

uint8_t USBStream_SendText(const char *str)
{
    if (str == NULL) return 0u;
    if (!text_push(str)) {
        frames_dropped++;
        return 0u;
    }
    return 1u;
}

uint8_t USBStream_SendEvent(const RR2_Event *evt,
                            int32_t temp_milli_c,
                            uint32_t timestamp_ms)
{
    if (evt == NULL) return 0u;

    const uint8_t  first = evt->first_ch;
    const uint8_t  count = evt->count;

    if (tx_format == USBSTREAM_FMT_TEXT) {
        /* E,seq,ms,tempmC,first,count,hg,lg,hg,lg,...  */
        if (!text_push("E,"))                    { frames_dropped++; return 0u; }
        if (!text_push_i32((int32_t)evt->seq))   { frames_dropped++; return 0u; }
        if (!text_push(","))                     { frames_dropped++; return 0u; }
        if (!text_push_i32((int32_t)timestamp_ms)) { frames_dropped++; return 0u; }
        if (!text_push(","))                     { frames_dropped++; return 0u; }
        if (!text_push_i32(temp_milli_c))        { frames_dropped++; return 0u; }
        if (!text_push(","))                     { frames_dropped++; return 0u; }
        if (!text_push_i32((int32_t)first))      { frames_dropped++; return 0u; }
        if (!text_push(","))                     { frames_dropped++; return 0u; }
        if (!text_push_i32((int32_t)count))      { frames_dropped++; return 0u; }

        for (uint8_t i = 0u; i < count; ++i) {
            const uint8_t ch = (uint8_t)(first + i);
            if (!text_push(","))                          { frames_dropped++; return 0u; }
            if (!text_push_i32((int32_t)evt->hg[ch]))     { frames_dropped++; return 0u; }
            if (!text_push(","))                          { frames_dropped++; return 0u; }
            if (!text_push_i32((int32_t)evt->lg[ch]))     { frames_dropped++; return 0u; }
        }
        if (!text_push("\r\n")) { frames_dropped++; return 0u; }
        return 1u;
    }

    /* ---- Binary ---- */
    const uint16_t payload_len = (uint16_t)(14u + (4u * (uint32_t)count));
    const uint32_t need        = 2u + 1u + 2u + payload_len + 2u;

    if (ring_free() < need) {
        frames_dropped++;
        return 0u;
    }

    put_header((uint8_t)USBSTREAM_FRAME_EVENT, payload_len);
    put_u32_crc(evt->seq);
    put_u32_crc(timestamp_ms);
    put_u32_crc((uint32_t)temp_milli_c);
    put_crc(first);
    put_crc(count);
    for (uint8_t i = 0u; i < count; ++i) {
        put_u16_crc(evt->hg[(uint8_t)(first + i)]);
    }
    for (uint8_t i = 0u; i < count; ++i) {
        put_u16_crc(evt->lg[(uint8_t)(first + i)]);
    }
    put_footer();
    return 1u;
}

uint8_t USBStream_SendStatus(const USBStream_Status *st)
{
    if (st == NULL) return 0u;

    if (tx_format == USBSTREAM_FMT_TEXT) {
        if (!text_push("S,"))                                { frames_dropped++; return 0u; }
        if (!text_push_i32((int32_t)st->uptime_ms))          { frames_dropped++; return 0u; }
        if (!text_push(","))                                 { frames_dropped++; return 0u; }
        if (!text_push_i32((int32_t)st->trigger_count))      { frames_dropped++; return 0u; }
        if (!text_push(","))                                 { frames_dropped++; return 0u; }
        if (!text_push_i32((int32_t)st->events_ok))          { frames_dropped++; return 0u; }
        if (!text_push(","))                                 { frames_dropped++; return 0u; }
        if (!text_push_i32((int32_t)st->events_bad))         { frames_dropped++; return 0u; }
        if (!text_push(","))                                 { frames_dropped++; return 0u; }
        if (!text_push_i32((int32_t)st->frames_dropped))     { frames_dropped++; return 0u; }
        if (!text_push(","))                                 { frames_dropped++; return 0u; }
        if (!text_push_i32(st->temp_milli_c))                { frames_dropped++; return 0u; }
        if (!text_push(","))                                 { frames_dropped++; return 0u; }
        if (!text_push_i32((int32_t)st->rr2_online))         { frames_dropped++; return 0u; }
        if (!text_push(","))                                 { frames_dropped++; return 0u; }
        if (!text_push_i32((int32_t)st->temp_online))        { frames_dropped++; return 0u; }
        if (!text_push("\r\n"))                              { frames_dropped++; return 0u; }
        return 1u;
    }

    const uint16_t payload_len = 27u;
    const uint32_t need        = 2u + 1u + 2u + payload_len + 2u;

    if (ring_free() < need) {
        frames_dropped++;
        return 0u;
    }

    put_header((uint8_t)USBSTREAM_FRAME_STATUS, payload_len);
    put_u32_crc(st->uptime_ms);
    put_u32_crc(st->trigger_count);
    put_u32_crc(st->events_ok);
    put_u32_crc(st->events_bad);
    put_u32_crc(st->frames_dropped);
    put_u32_crc((uint32_t)st->temp_milli_c);
    put_crc((uint8_t)((st->rr2_online ? 0x01u : 0u) |
                      (st->temp_online ? 0x02u : 0u) |
                      (st->timing_ok  ? 0x04u : 0u)));
    put_crc(st->cfg_status);
    put_crc(st->read_status);
    put_footer();
    return 1u;
}

/* ------------------------------------------------------------------ */
/* Transmit pump                                                       */
/* ------------------------------------------------------------------ */
void USBStream_Task(void)
{
    uint32_t used;
    uint32_t chunk;

    used = ring_used();
    if (used == 0u) return;

    /* Nothing to do until the host has enumerated and configured us.
       Without this check frames would pile up and be dropped while the
       cable is unplugged. */
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;

    /* Send at most one contiguous span, capped, so we never block the
       main loop for long. */
    chunk = USBSTREAM_RING_SIZE - ring_tail;   /* to end of buffer */
    if (chunk > used)                 chunk = used;
    if (chunk > USBSTREAM_CHUNK_MAX)  chunk = USBSTREAM_CHUNK_MAX;

    if (CDC_Transmit_FS(&ring[ring_tail], (uint16_t)chunk) == USBD_OK) {
        ring_tail = (ring_tail + chunk) & (uint32_t)(USBSTREAM_RING_SIZE - 1u);
    }
    /* USBD_BUSY means a transfer is still in flight - just retry next
       pass, the data stays queued. */
}
