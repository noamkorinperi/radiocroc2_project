/**
 ******************************************************************************
 * @file    usb_stream.c
 * @brief   Event streaming to the host over USART3 -> the ST-Link VCP.
 * @note    See usb_stream.h for the frame layout.
 *
 * Stack note: the F722 project links with a 1 KB main stack, so this
 * module never builds a whole frame in a local array. Text formatting
 * goes through a 24-byte scratch buffer, one number at a time.
 ******************************************************************************
 */
#include "usb_stream.h"
#include "usb_cmd.h"         /* USBCmd_Feed, fed from the RX interrupt */
#include "usart.h"           /* huart3 - the ST-Link VCP */
#include <stdio.h>
#include <string.h>

/* Single byte landing zone for the receive interrupt. Commands are typed
   by a human, so there is no case for DMA on this side - one interrupt
   per character costs nothing at the rates a keyboard produces. */
static uint8_t rx_byte;

/* Length of the DMA transfer currently in flight, 0 when idle. The ring
   tail is NOT advanced when the transfer starts, only when it finishes,
   so the bytes being sent stay reserved and the producer cannot walk
   over data that is still on the wire. */
static volatile uint32_t tx_len = 0u;

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
    tx_len         = 0u;
    frames_dropped = 0u;
    tx_format      = USBSTREAM_FMT_BINARY;

    /* Arm the receive side. From here on every character re-arms itself
       from the interrupt, so this is the only place it has to be kicked
       off - except after an error, which the error callback handles. */
    (void)HAL_UART_Receive_IT(&huart3, &rx_byte, 1u);
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
        /* E,seq,ms,tempmC,first,count,lg,lg,...  */
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
            if (!text_push_i32((int32_t)evt->lg[ch]))     { frames_dropped++; return 0u; }
        }
        /* HG columns after ALL the LG ones, never interleaved - the
           text row mirrors the binary payload, so a reader that takes
           the first 'count' values after the header still gets pure LG
           whether or not the event carries the second gain. */
        if (evt->has_hg) {
            for (uint8_t i = 0u; i < count; ++i) {
                const uint8_t ch = (uint8_t)(first + i);
                if (!text_push(","))                      { frames_dropped++; return 0u; }
                if (!text_push_i32((int32_t)evt->hg[ch])) { frames_dropped++; return 0u; }
            }
        }
        if (!text_push("\r\n")) { frames_dropped++; return 0u; }
        return 1u;
    }

    /* ---- Binary ---- */
    /* The HG block is announced by nothing but the payload length: the
       host tests plen against 14 + 4*count. Appended after the LG
       codes, not interleaved and not a new frame type, so a decoder
       that predates it keeps working untouched. */
    const uint16_t payload_len = (uint16_t)(14u + (2u * (uint32_t)count)
                                     + (evt->has_hg ? (2u * (uint32_t)count) : 0u));
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
        put_u16_crc(evt->lg[(uint8_t)(first + i)]);
    }
    if (evt->has_hg) {
        for (uint8_t i = 0u; i < count; ++i) {
            put_u16_crc(evt->hg[(uint8_t)(first + i)]);
        }
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

    const uint16_t payload_len = 30u;
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
                      (st->timing_ok  ? 0x04u : 0u) |
                      (st->bus_jam    ? 0x08u : 0u)));
    put_crc(st->cfg_status);
    put_crc(st->read_status);
    put_crc(st->cmd_done);
    put_crc(st->cmd_failed);
    put_crc(st->cmd_last);
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

    /* A transfer is already on the wire. Nothing to do - the completion
       interrupt will free the space and the next pass will pick up. */
    if (tx_len != 0u) return;

    used = ring_used();
    if (used == 0u) return;

    /* One contiguous span only: the DMA walks memory linearly and has
       no idea the buffer wraps. Capped so a burst cannot monopolise
       the link and starve the command channel of its replies. */
    chunk = USBSTREAM_RING_SIZE - ring_tail;   /* to end of buffer */
    if (chunk > used)                 chunk = used;
    if (chunk > USBSTREAM_CHUNK_MAX)  chunk = USBSTREAM_CHUNK_MAX;

    /* Claim the span BEFORE handing it to the DMA. Once tx_len is set,
       ring_free() counts these bytes as occupied, so a producer running
       between here and the completion interrupt cannot overwrite them. */
    tx_len = chunk;

    if (HAL_UART_Transmit_DMA(&huart3, &ring[ring_tail],
                              (uint16_t)chunk) != HAL_OK) {
        /* Could not start. Release the claim and retry next pass; the
           data stays queued either way. */
        tx_len = 0u;
    }
}

/**
 * @brief DMA finished pushing a span out of the UART.
 *
 * Only now is it safe to release those bytes back to the ring. Doing it
 * when the transfer STARTED - which is what the USB CDC path used to do
 * - let a fast burst of events overwrite bytes that were still being
 * clocked out, corrupting frames under exactly the load where you least
 * want to lose them.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;

    ring_tail = (ring_tail + tx_len) & (uint32_t)(USBSTREAM_RING_SIZE - 1u);
    tx_len    = 0u;
}

/* ------------------------------------------------------------------ */
/* Receive - host commands                                             */
/* ------------------------------------------------------------------ */
/**
 * @brief One character arrived from the host.
 *
 * Re-arms before doing anything else. At 921600 baud the next byte is
 * only 11 us behind this one and USBCmd_Feed() is not free, so arming
 * after the copy leaves a window with no receive pending - which is how
 * a burst of typed commands turns into an overrun. The byte is taken
 * into a local first, because re-arming hands rx_byte back to the ISR.
 *
 * The parsing happens in the main loop, because acting on a command can
 * mean hundreds of milliseconds of blocking I2C and that must never run
 * in interrupt context.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;

    const uint8_t byte = rx_byte;
    (void)HAL_UART_Receive_IT(huart, &rx_byte, 1u);
    USBCmd_Feed(&byte, 1u);
}

/**
 * @brief A receive error aborted the pending read - restart it.
 *
 * Framing and overrun errors are expected on a line with no flow
 * control, especially while the host opens or closes the port. Without
 * re-arming here a single glitch would silently kill the command
 * channel for the rest of the session.
 *
 * What must not happen here is touching the transmit claim. A receive
 * overrun says nothing about the span the DMA is still clocking out,
 * and releasing it on the way past did two things at once: ring_free()
 * began counting bytes still on the wire as reusable, and clearing
 * tx_len opened the guard at the top of USBStream_Task(), which then
 * started a second transfer over the first, got HAL_BUSY, and left the
 * span unsent with ring_tail already past it. Neither loss increments
 * frames_dropped, so the reply to whichever command had just blocked
 * the main loop long enough to cause the overrun - 'push' above all -
 * went missing with every indicator still green.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;

    /* The DMA gave up on the transmit. UART_DMAError() has already
       ended that transfer, so those bytes are never going out and the
       pump would wait on them forever - this is the one case where the
       claim is ours to release. */
    if ((tx_len != 0u) &&
        ((huart->ErrorCode & HAL_UART_ERROR_DMA) != 0u) &&
        (huart->gState == HAL_UART_STATE_READY)) {
        ring_tail = (ring_tail + tx_len) & (uint32_t)(USBSTREAM_RING_SIZE - 1u);
        tx_len    = 0u;
    }

    /* Re-arm only if the HAL really ended the read. It does that for an
       overrun, which is the error that matters here; framing and noise
       leave the receive live, and this would just return HAL_BUSY. */
    if (huart->RxState == HAL_UART_STATE_READY) {
        (void)HAL_UART_Receive_IT(huart, &rx_byte, 1u);
    }
}
