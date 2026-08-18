/**
 ******************************************************************************
 * @file    usb_cmd.c
 * @brief   Line-based command interface over USB CDC.
 * @note    See usb_cmd.h for the command set.
 ******************************************************************************
 */
#include "usb_cmd.h"
#include "radioroc2_ctrl.h"
#include "usb_stream.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/* Receive ring - written from the USB ISR, drained from the main loop */
/* ------------------------------------------------------------------ */
static uint8_t  rx_ring[USBCMD_RX_RING_SIZE];
static volatile uint32_t rx_head = 0u;
static volatile uint32_t rx_tail = 0u;
static uint32_t overruns = 0u;

/* Line assembly and tokenising buffers live in .bss: the main stack is
   only 1 KB, so nothing large is placed on it. */
static char  line[USBCMD_LINE_MAX];
static uint32_t line_len = 0u;
static char *tok[8];
static uint8_t  ntok = 0u;

#define RX_MASK   (USBCMD_RX_RING_SIZE - 1u)

/* ------------------------------------------------------------------ */
/* Small output helpers                                                */
/* ------------------------------------------------------------------ */
static void reply(const char *s)
{
    (void)USBStream_SendText(s);
}

static void reply_status(RR2_Status st)
{
    reply((st == RR2_OK) ? "ok\r\n" : "ERR: ASIC write failed\r\n");
}

static void reply_kv(const char *key, int32_t value)
{
    char b[32];
    (void)snprintf(b, sizeof(b), "%s=%ld\r\n", key, (long)value);
    reply(b);
}

/* ------------------------------------------------------------------ */
/* Parsing helpers                                                     */
/* ------------------------------------------------------------------ */
static int32_t arg_i(uint8_t index, int32_t fallback)
{
    if (index >= ntok) return fallback;
    return (int32_t)strtol(tok[index], NULL, 0);
}

static uint8_t arg_is(uint8_t index, const char *what)
{
    if (index >= ntok) return 0u;
    return (strcmp(tok[index], what) == 0) ? 1u : 0u;
}

/** Resolve the channel argument: a number, or "all". */
static uint8_t parse_channel(uint8_t index, uint8_t *ok)
{
    *ok = 1u;
    if (arg_is(index, "all")) return RR2_CH_ALL;

    const int32_t v = arg_i(index, -1);
    if ((v < 0) || (v >= (int32_t)RR2_NUM_CHANNELS)) {
        *ok = 0u;
        return 0u;
    }
    return (uint8_t)v;
}

/* ------------------------------------------------------------------ */
/* Command handlers                                                    */
/* ------------------------------------------------------------------ */
static void cmd_help(void)
{
    reply("commands:\r\n");
    reply("  stat | fmt bin|txt | defaults | push\r\n");
    reply("  sel <ch> [ch ...] | sel all | sel   (detector channels)\r\n");
    reply("  ch <n|all> indac <0-255>\r\n");
    reply("  ch <n|all> gain <lg> <hg>      (0-15)\r\n");
    reply("  ch <n|all> tau <lg> <hg>       (0-15)\r\n");
    reply("  ch <n|all> slow <lg> <hg>      (0|1)\r\n");
    reply("  ch <n|all> patgain <0-63>\r\n");
    reply("  ch <n|all> trim <t1> <t2>      (0-63)\r\n");
    reply("  ch <n|all> on|off\r\n");
    reply("  ch <n|all> ctest <0|1> <cap 0|1>\r\n");
    reply("  ch <n> dump\r\n");
    reply("  th <dac1> <dac2> <dacq>        (0-1023)\r\n");
    reply("  delay <0-255> <slope 0-15>\r\n");
    reply("  trig <0-15> | hold int|ext | mux <hg> <lg>\r\n");
    reply("  preset csi\r\n");
    reply("  w <addr> <sub> <data> | r <addr> <sub>\r\n");
}

static void cmd_dump(uint8_t ch)
{
    const RR2_Shadow *s = RR2_Ctrl_GetShadow();
    char b[64];

    if (ch >= RR2_NUM_CHANNELS) {
        reply("ERR: dump needs a single channel\r\n");
        return;
    }

    (void)snprintf(b, sizeof(b),
                   "ch%u: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                   (unsigned)ch,
                   s->ch[ch][0], s->ch[ch][1], s->ch[ch][2], s->ch[ch][3],
                   s->ch[ch][4], s->ch[ch][5], s->ch[ch][6], s->ch[ch][7]);
    reply(b);

    (void)snprintf(b, sizeof(b),
                   "glob: dac %02X %02X %02X %02X  dly %02X %02X  trig %02X  mux %02X\r\n",
                   s->com_dac1_lo, s->com_dac2_dac1, s->com_dacq_dac2,
                   s->com_dacq_hi, s->com_delay, s->com_slope,
                   s->com_hyst_trig, s->out_power);
    reply(b);
}

static void cmd_ch(void)
{
    uint8_t ok = 0u;
    const uint8_t ch = parse_channel(1u, &ok);

    if (!ok) {
        reply("ERR: channel must be 0-63 or 'all'\r\n");
        return;
    }
    if (ntok < 3u) {
        reply("ERR: missing sub-command\r\n");
        return;
    }

    if (arg_is(2u, "indac")) {
        reply_status(RR2_Ctrl_SetInDac(ch, (uint8_t)arg_i(3u, 128)));
    } else if (arg_is(2u, "gain")) {
        reply_status(RR2_Ctrl_SetChargeGain(ch, (uint8_t)arg_i(3u, 4),
                                                (uint8_t)arg_i(4u, 4)));
    } else if (arg_is(2u, "tau")) {
        reply_status(RR2_Ctrl_SetShapingTime(ch, (uint8_t)arg_i(3u, 1),
                                                 (uint8_t)arg_i(4u, 1)));
    } else if (arg_is(2u, "slow")) {
        reply_status(RR2_Ctrl_SetSlowShaping(ch, (uint8_t)arg_i(3u, 0),
                                                 (uint8_t)arg_i(4u, 0)));
    } else if (arg_is(2u, "patgain")) {
        reply_status(RR2_Ctrl_SetPatGain(ch, (uint8_t)arg_i(3u, 32)));
    } else if (arg_is(2u, "trim")) {
        reply_status(RR2_Ctrl_SetThresholdTrim(ch, (uint8_t)arg_i(3u, 0),
                                                   (uint8_t)arg_i(4u, 0)));
    } else if (arg_is(2u, "on")) {
        reply_status(RR2_Ctrl_SetChannelEnabled(ch, 1u));
    } else if (arg_is(2u, "off")) {
        reply_status(RR2_Ctrl_SetChannelEnabled(ch, 0u));
    } else if (arg_is(2u, "ctest")) {
        reply_status(RR2_Ctrl_SetChargeInjection(ch, (uint8_t)arg_i(3u, 0),
                                                     (uint8_t)arg_i(4u, 0)));
    } else if (arg_is(2u, "dump")) {
        cmd_dump(ch);
    } else {
        reply("ERR: unknown ch sub-command\r\n");
    }
}

/**
 * @brief "sel 3 9 20 41 55"  - pick the channels that carry a detector.
 *        "sel all"           - back to all 64.
 *        "sel"               - just report the current selection.
 *
 * Applies to BOTH sides: the readout stops digitising the others, and
 * the ASIC stops them triggering. The sensors are modular, so this is
 * expected to change between runs and lives here rather than in a
 * #define.
 */
static void cmd_sel(void)
{
    RR2_Status st;

    if (ntok >= 2u) {
        if (arg_is(1u, "all")) {
            RR2_DAQ_SetChannelMask(RR2_MASK_ALL);
        } else {
            uint8_t list[8];
            uint8_t n = 0u;

            for (uint8_t i = 1u; (i < ntok) && (n < (uint8_t)sizeof(list)); ++i) {
                const int32_t v = arg_i(i, -1);
                if ((v < 0) || (v >= (int32_t)RR2_NUM_CHANNELS)) {
                    reply("ERR: channels must be 0-63\r\n");
                    return;
                }
                list[n++] = (uint8_t)v;
            }

            st = RR2_DAQ_SelectChannels(list, n);
            if (st != RR2_OK) {
                reply("ERR: bad channel list\r\n");
                return;
            }
        }

        /* Keep the ASIC in step, otherwise the deselected channels keep
           firing the NOR trigger on their own noise. ~70 ms. */
        st = RR2_Ctrl_ApplyChannelMask(RR2_DAQ_GetChannelMask());
        if (st != RR2_OK) {
            reply("ERR: selection set, but the ASIC refused the enables\r\n");
            return;
        }
    }

    /* Always echo what is now in force, so "sel" alone is a query. */
    const uint64_t m = RR2_DAQ_GetChannelMask();
    char b[64];
    (void)snprintf(b, sizeof(b), "sel=%08lX%08lX n=%u\r\n",
                   (unsigned long)(uint32_t)(m >> 32),
                   (unsigned long)(uint32_t)(m & 0xFFFFFFFFu),
                   (unsigned)RR2_DAQ_GetChannelCount());
    reply(b);
}

static void execute(void)
{
    if (ntok == 0u) return;

    if (arg_is(0u, "help")) {
        cmd_help();
    }
    else if (arg_is(0u, "sel")) {
        cmd_sel();
    }
    else if (arg_is(0u, "stat")) {
        reply_kv("pending", (int32_t)USBStream_GetPending());
        reply_kv("dropped", (int32_t)USBStream_GetDropped());
        reply_kv("rx_overruns", (int32_t)overruns);
        reply_kv("format", (int32_t)USBStream_GetFormat());
    }
    else if (arg_is(0u, "fmt")) {
        if (arg_is(1u, "txt")) {
            USBStream_SetFormat(USBSTREAM_FMT_TEXT);
            reply("ok, text\r\n");
        } else {
            USBStream_SetFormat(USBSTREAM_FMT_BINARY);
            reply("ok, binary\r\n");
        }
    }
    else if (arg_is(0u, "ch")) {
        cmd_ch();
    }
    else if (arg_is(0u, "th")) {
        reply_status(RR2_Ctrl_SetThresholds((uint16_t)arg_i(1u, 300),
                                            (uint16_t)arg_i(2u, 500),
                                            (uint16_t)arg_i(3u, 200)));
    }
    else if (arg_is(0u, "delay")) {
        reply_status(RR2_Ctrl_SetHoldDelay((uint8_t)arg_i(1u, 255),
                                           (uint8_t)arg_i(2u, 4)));
    }
    else if (arg_is(0u, "trig")) {
        reply_status(RR2_Ctrl_SetTriggerSource((uint8_t)arg_i(1u, 4)));
    }
    else if (arg_is(0u, "hold")) {
        reply_status(RR2_Ctrl_SetHoldExternal(arg_is(1u, "ext")));
    }
    else if (arg_is(0u, "mux")) {
        reply_status(RR2_Ctrl_SetAnalogMux((uint8_t)arg_i(1u, 1),
                                           (uint8_t)arg_i(2u, 1)));
    }
    else if (arg_is(0u, "preset")) {
        if (arg_is(1u, "csi")) {
            reply_status(RR2_Ctrl_PresetCsI());
        } else {
            reply("ERR: known presets: csi\r\n");
        }
    }
    else if (arg_is(0u, "defaults")) {
        RR2_Ctrl_ResetShadow();
        reply("shadow reset, run 'push' to apply\r\n");
    }
    else if (arg_is(0u, "push")) {
        reply("pushing all channels, ~0.4 s\r\n");
        reply_status(RR2_Ctrl_PushAll());
    }
    else if (arg_is(0u, "w")) {
        reply_status(RR2_Write((uint8_t)arg_i(1u, 0),
                               (uint8_t)arg_i(2u, 0),
                               (uint8_t)arg_i(3u, 0)));
    }
    else if (arg_is(0u, "r")) {
        uint8_t v = 0u;
        RR2_Status st = RR2_Read((uint8_t)arg_i(1u, 0),
                                 (uint8_t)arg_i(2u, 0), &v);
        if (st == RR2_OK) reply_kv("val", (int32_t)v);
        else              reply_status(st);
    }
    else {
        reply("ERR: unknown command, try 'help'\r\n");
    }
}

/* ------------------------------------------------------------------ */
/* Line assembly                                                       */
/* ------------------------------------------------------------------ */
static void tokenize(void)
{
    char *p = line;
    ntok = 0u;

    while ((*p != '\0') && (ntok < (uint8_t)(sizeof(tok) / sizeof(tok[0])))) {
        while ((*p == ' ') || (*p == '\t')) { *p = '\0'; ++p; }
        if (*p == '\0') break;
        tok[ntok++] = p;
        while ((*p != '\0') && (*p != ' ') && (*p != '\t')) { ++p; }
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
void USBCmd_Init(void)
{
    rx_head  = 0u;
    rx_tail  = 0u;
    line_len = 0u;
    overruns = 0u;
}

void USBCmd_Feed(const uint8_t *data, uint32_t len)
{
    if (data == NULL) return;

    for (uint32_t i = 0u; i < len; ++i) {
        const uint32_t next = (rx_head + 1u) & RX_MASK;
        if (next == rx_tail) {      /* full - drop the byte */
            overruns++;
            return;
        }
        rx_ring[rx_head] = data[i];
        rx_head = next;
    }
}

void USBCmd_Task(void)
{
    while (rx_tail != rx_head) {
        const char c = (char)rx_ring[rx_tail];
        rx_tail = (rx_tail + 1u) & RX_MASK;

        if ((c == '\r') || (c == '\n')) {
            if (line_len > 0u) {
                line[line_len] = '\0';
                tokenize();
                execute();          /* may block on I2C - main loop only */
                line_len = 0u;
            }
        } else if (line_len < (USBCMD_LINE_MAX - 1u)) {
            line[line_len++] = c;
        } else {
            /* Overlong line: drop it and resynchronise on the newline. */
            line_len = 0u;
            reply("ERR: line too long\r\n");
        }
    }
}

uint32_t USBCmd_GetOverruns(void)
{
    return overruns;
}
