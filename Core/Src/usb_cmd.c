/**
 ******************************************************************************
 * @file    usb_cmd.c
 * @brief   Line-based command interface over the host link.
 * @note    See usb_cmd.h for the command set.
 ******************************************************************************
 */
#include "usb_cmd.h"
#include "radioroc2_ctrl.h"
#include "radioroc2_daq.h"
#include "rr2_i2ctest.h"
#include "usb_stream.h"
#include "main.h"            /* the pin macros, for the line report */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/* Bring-up state owned by main.c                                      */
/* ------------------------------------------------------------------ */
/* These live in main.c and have no header of their own. They are read
   here so 'stat' can answer the question a stalled bring-up actually
   raises - did the ASIC ever get configured, and is the trigger line
   moving - without a debugger attached. Declared locally rather than
   in main.h, which is CubeMX generated and does not know RR2_Status. */
extern volatile uint8_t    g_rr2_online;
extern volatile uint8_t    g_rr2_bus_jam;
extern volatile uint8_t    g_rr2_timing_ok;
extern volatile uint8_t    g_rr2_sc_error;
extern volatile RR2_Status g_rr2_cfg_status;
extern volatile RR2_Status g_rr2_read_status;
extern volatile uint32_t   g_rr2_trigger_count;
extern volatile uint32_t   g_rr2_events_ok;
extern volatile uint32_t   g_rr2_events_bad;

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

/* Every command that produces a result is counted here, and the counts
   ride in the status frame. The reply below travels as bare text with
   no framing of its own and can be lost - by a full ring, by a resync,
   by anything that eats bytes - and a lost reply is indistinguishable
   from a command that never ran. The counts cannot be lost the same
   way: they sit inside a CRC protected frame that is resent every
   second, so the host can ask "did my six writes complete" and get a
   real answer even when not one of the six replies arrived. Written and
   read from the main loop only, so the reads need no protection. */
static uint8_t cmd_done   = 0u;   /* commands completed, wraps at 256  */
static uint8_t cmd_failed = 0u;   /* of those, how many returned error */
static uint8_t cmd_last   = 0u;   /* RR2_Status of the most recent one */

static void reply_status(RR2_Status st)
{
    cmd_last = (uint8_t)st;
    if (st != RR2_OK) cmd_failed++;
    cmd_done++;                   /* bumped last, so a completion is
                                     never visible to the host before
                                     the failure that belongs to it */
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
    reply("  stat | lines | fmt bin|txt | defaults | push\r\n");
    reply("  ch <n|all> indac <0-255>\r\n");
    reply("  ch <n|all> gain <lg> [hg]      (0-15)\r\n");
    reply("  ch <n|all> tau <lg> [hg]       (0-15)\r\n");
    reply("  ch <n|all> slow <lg> [hg]      (0|1)\r\n");
    reply("  ch <n|all> patgain <0-63>\r\n");
    reply("  ch <n|all> trim <t1> <t2>      (0-63)\r\n");
    reply("  ch <n|all> on|off\r\n");
    reply("  ch <n|all> discri <0|1>        trigger, without the rest\r\n");
    reply("  ch <n|all> ctest <0|1> <cap 0|1>\r\n");
    reply("  ch <n> dump\r\n");
    reply("  th <dac1> <dac2> <dacq>        (0-1023)\r\n");
    reply("  delay <0-255> <slope 0-15>\r\n");
    reply("  trig <0-15> | hold int|ext | mux <0|1>  (LG buffer)\r\n");
    reply("  hg <0|1>                       HG buffer + HG readout\r\n");
    reply("  preset csi\r\n");
    reply("  w <addr> <sub> <data> | r <addr> <sub>\r\n");
    reply("  i2ctest                        Slow Control link test\r\n");
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

    /* mux shows what the ASIC was asked to power (the OUT_POWER shadow,
       EN_aMuxHG is bit 1), hg shows what the DAQ actually samples. They
       normally agree; a failed Slow Control write is the case where
       they will not, and this line is where that shows. */
    (void)snprintf(b, sizeof(b),
                   "glob: dac %02X %02X %02X %02X  dly %02X %02X  trig %02X  mux %02X  hg %u\r\n",
                   s->com_dac1_lo, s->com_dac2_dac1, s->com_dacq_dac2,
                   s->com_dacq_hi, s->com_delay, s->com_slope,
                   s->com_hyst_trig, s->out_power, (unsigned)RR2_DAQ_GetHG());
    reply(b);
}

/**
 * @brief Report the static level of every ASIC control line, then watch
 *        the trigger line for a fixed window.
 *
 * This exists because "no events" has two causes that look identical
 * from the counters: an ASIC that never fires, and an EXTI armed on the
 * edge the line does not produce. Both leave trigger_count at zero.
 *
 * NOR_T1OC is assumed active low everywhere in this firmware - EXTI0 is
 * armed on the falling edge and PA0 carries a pull-up. If the open
 * collector driver in the ASIC conducts while idle, the line sits LOW
 * between events and rises on a trigger, and every one of them is
 * invisible. nor= below is that answer: 1 means the assumption holds,
 * 0 means the interrupt is watching the wrong direction.
 *
 * The two counters are complementary and neither is redundant. The
 * polled ones only see a pulse wider than the sampling loop, which at
 * -O0 is a few hundred ns; exti= is incremented from the interrupt and
 * catches pulses far shorter than that, but only in one direction.
 * Polled edges with no exti= movement is the wrong-edge signature.
 *
 * Blocks the main loop for the length of the window. Triggers arriving
 * meanwhile still latch their flag and are serviced afterwards.
 */
static void cmd_lines(void)
{
    const uint32_t window_ms = 200u;
    uint32_t rise = 0u, fall = 0u, low = 0u, samples = 0u;

    /* One key=value per line, exactly as 'stat' answers. The GUI only
       recognises a reply as data when the line holds a single pair and
       no spaces, so packing several onto a line would print in the
       console and reach nothing that saves it. */
    reply_kv("nor",       (int32_t)HAL_GPIO_ReadPin(NOR_T1OC_GPIO_Port,  NOR_T1OC_Pin));
    reply_kv("errorn",    (int32_t)HAL_GPIO_ReadPin(ERRORN_SC_GPIO_Port, ERRORN_SC_Pin));
    reply_kv("reset_n",   (int32_t)HAL_GPIO_ReadPin(RESET_N_GPIO_Port,   RESET_N_Pin));
    reply_kv("rstn_read", (int32_t)HAL_GPIO_ReadPin(RSTN_READ_GPIO_Port, RSTN_READ_Pin));
    reply_kv("rstn_i2c",  (int32_t)HAL_GPIO_ReadPin(RSTN_I2C_GPIO_Port,  RSTN_I2C_Pin));
    reply_kv("rstn_sc",   (int32_t)HAL_GPIO_ReadPin(RSTN_SC_GPIO_Port,   RSTN_SC_Pin));
    reply_kv("holdext",   (int32_t)HAL_GPIO_ReadPin(HOLDEXT_GPIO_Port,   HOLDEXT_Pin));
    reply_kv("ck_read",   (int32_t)HAL_GPIO_ReadPin(CK_READ_GPIO_Port,   CK_READ_Pin));

    /* Watch NOR_T1OC. */
    {
        const uint32_t exti_before = g_rr2_trigger_count;
        const uint32_t t_start = HAL_GetTick();   /* difference form: wrap safe */
        uint8_t prev = (uint8_t)HAL_GPIO_ReadPin(NOR_T1OC_GPIO_Port, NOR_T1OC_Pin);

        while ((HAL_GetTick() - t_start) < window_ms) {
            const uint8_t now =
                (uint8_t)HAL_GPIO_ReadPin(NOR_T1OC_GPIO_Port, NOR_T1OC_Pin);
            if (now != prev) {
                if (now) rise++; else fall++;
                prev = now;
            }
            if (!now) low++;
            samples++;
        }

        reply_kv("nor_ms",   (int32_t)window_ms);
        reply_kv("nor_rise", (int32_t)rise);
        reply_kv("nor_fall", (int32_t)fall);
        reply_kv("nor_exti", (int32_t)(g_rr2_trigger_count - exti_before));
    }

    /* Permille rather than percent: a source at a few hundred counts a
       second holds the line down for well under 1% of the window, and
       percent would round every one of those to a flat zero.

       Divided down first rather than scaling low by 1000. The loop is
       untimed and its sample count depends on the optimisation level -
       at -O2 it can pass ten million, and ten million times a thousand
       does not fit in the 32 bits this arithmetic runs in. */
    {
        const uint32_t per_permille = (samples + 999u) / 1000u;
        reply_kv("nor_low_permille",
                 (per_permille != 0u) ? (int32_t)(low / per_permille) : 0);
        reply_kv("nor_samples", (int32_t)samples);
    }
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
        /* No hg argument -> RR2_KEEP -> the HG nibble is left alone. */
        reply_status(RR2_Ctrl_SetChargeGain(ch, (uint8_t)arg_i(3u, 4),
                                                (uint8_t)arg_i(4u, RR2_KEEP)));
    } else if (arg_is(2u, "tau")) {
        reply_status(RR2_Ctrl_SetShapingTime(ch, (uint8_t)arg_i(3u, 1),
                                                 (uint8_t)arg_i(4u, RR2_KEEP)));
    } else if (arg_is(2u, "slow")) {
        reply_status(RR2_Ctrl_SetSlowShaping(ch, (uint8_t)arg_i(3u, 0),
                                                 (uint8_t)arg_i(4u, RR2_KEEP)));
    } else if (arg_is(2u, "patgain")) {
        reply_status(RR2_Ctrl_SetPatGain(ch, (uint8_t)arg_i(3u, 32)));
    } else if (arg_is(2u, "trim")) {
        reply_status(RR2_Ctrl_SetThresholdTrim(ch, (uint8_t)arg_i(3u, 0),
                                                   (uint8_t)arg_i(4u, 0)));
    } else if (arg_is(2u, "on")) {
        reply_status(RR2_Ctrl_SetChannelEnabled(ch, 1u));
    } else if (arg_is(2u, "off")) {
        reply_status(RR2_Ctrl_SetChannelEnabled(ch, 0u));
    } else if (arg_is(2u, "discri")) {
        reply_status(RR2_Ctrl_SetDiscriminators(ch, (uint8_t)arg_i(3u, 1)));
    } else if (arg_is(2u, "ctest")) {
        reply_status(RR2_Ctrl_SetChargeInjection(ch, (uint8_t)arg_i(3u, 0),
                                                     (uint8_t)arg_i(4u, 0)));
    } else if (arg_is(2u, "dump")) {
        cmd_dump(ch);
    } else {
        reply("ERR: unknown ch sub-command\r\n");
    }
}

static void execute(void)
{
    if (ntok == 0u) return;

    if (arg_is(0u, "help")) {
        cmd_help();
    }
    else if (arg_is(0u, "stat")) {
        /* Bring-up first. All of this already rides in the status frame,
           but that frame is binary by default, so on a terminal it is
           the noise you scroll past - and the one number that splits
           "the ASIC never triggered" from "the readout failed" was
           only ever legible to a debugger. online=0 in particular is
           worth reading before anything else: main() skips the whole
           configuration sequence when the chip does not ACK, and then
           streams nothing while every other indicator stays green. */
        reply_kv("online",    (int32_t)g_rr2_online);
        reply_kv("cfg_st",    (int32_t)g_rr2_cfg_status);
        reply_kv("bus_jam",   (int32_t)g_rr2_bus_jam);
        reply_kv("sc_error",  (int32_t)g_rr2_sc_error);
        reply_kv("timing_ok", (int32_t)g_rr2_timing_ok);
        reply_kv("triggers",  (int32_t)g_rr2_trigger_count);
        reply_kv("events_ok", (int32_t)g_rr2_events_ok);
        reply_kv("events_bad",(int32_t)g_rr2_events_bad);
        reply_kv("read_st",   (int32_t)g_rr2_read_status);
        /* Which of the three Slow Control frames a failed write died
           on, as an RR2_Status: 1 = R0 sub-address, 2 = R1 address,
           3 = R2 data. "ERR: ASIC write failed" says only that one of
           them NACKed, and the three mean different faults - a chip
           that never saw the frame at all, against one that took the
           address and refused the payload. Kept next to the counts so
           a failure and the command it belongs to are read together. */
        reply_kv("cmd_last",   (int32_t)cmd_last);
        reply_kv("cmd_done",   (int32_t)cmd_done);
        reply_kv("cmd_failed", (int32_t)cmd_failed);
        reply_kv("pending", (int32_t)USBStream_GetPending());
        reply_kv("dropped", (int32_t)USBStream_GetDropped());
        reply_kv("rx_overruns", (int32_t)overruns);
        reply_kv("format", (int32_t)USBStream_GetFormat());
        /* What the readout will actually wait for before clocking
           CK_READ, for the delay currently configured. Sanity check it
           after 'preset csi': it must grow, not stay put.            */
        reply_kv("hold_ns", (int32_t)RR2_DAQ_HoldDelayNs());
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
    else if (arg_is(0u, "lines")) {
        cmd_lines();
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
        reply_status(RR2_Ctrl_SetAnalogMux((uint8_t)arg_i(1u, 1)));
    }
    else if (arg_is(0u, "hg")) {
        /* Two halves that must move together: EN_aMuxHG powers the mux
           buffer inside the ASIC, the DAQ switch makes the readout
           digitise PA4 and the stream append the codes. The DAQ goes
           first because it is the only half that can refuse locally
           (no ADC1 bound). If the Slow Control write then fails, the
           shadow still holds the intent and the next 'push' applies
           it - the same recovery every other setter here relies on -
           and 'glob' shows the two halves disagreeing meanwhile. */
        const uint8_t on = (uint8_t)(arg_i(1u, 1) != 0);
        RR2_Status st = RR2_DAQ_SetHG(on);
        if (st == RR2_OK) st = RR2_Ctrl_SetAnalogMuxHG(on);
        reply_status(st);
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
    else if (arg_is(0u, "i2ctest")) {
        /* Blocks for up to about two seconds. Nothing else may drive
           the bus while it runs, which is exactly what running it from
           here guarantees - the main loop is inside this call. */
        (void)RR2_I2CTest_Run();
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
    rx_head    = 0u;
    rx_tail    = 0u;
    line_len   = 0u;
    overruns   = 0u;
    cmd_done   = 0u;
    cmd_failed = 0u;
    cmd_last   = 0u;
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

uint8_t USBCmd_GetDone(void)       { return cmd_done; }
uint8_t USBCmd_GetFailed(void)     { return cmd_failed; }
uint8_t USBCmd_GetLastStatus(void) { return cmd_last; }
