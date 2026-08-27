#!/usr/bin/env python3
"""
rr2_thscan.py - automatic DAC1 threshold scan for RADIOROC2.

    python rr2_thscan.py                          # 320 -> 80, step 20
    python rr2_thscan.py --start 300 --stop 150 --step 10
    python rr2_thscan.py --values 300,280,260,240,220
    python rr2_thscan.py --label T30_HV27.14 --max-rate 20000

For every DAC1 value it sets the threshold, waits for the board to
confirm it took, dwells, and records the trigger rate. One CSV row per
point plus a JSON record of the whole run land in scans/.

The board's threshold is put back where it started when the scan ends -
including on Ctrl-C, and including on a crash. Leaving the chip parked
at a threshold buried in its own noise is how the next person's run
gets quietly ruined.

ONE PORT AT A TIME
This opens the same ST-Link VCP the GUI uses, and a serial port has one
owner. Close rr2_gui.py before starting a scan.

WHERE THE RATE COMES FROM
Not from counting event frames. Below the noise wall the ASIC fires far
faster than 921600 baud can carry 29-byte frames - about 3100 events a
second - and every event past that is dropped inside the firmware's
ring. Counting arrivals would flatten the top of the S-curve at exactly
the place the scan exists to find.

The rate comes from the status frame's own counters instead.
trigger_count is incremented in the NOR_T1OC interrupt, so it counts
every trigger the ASIC produced: the ones whose readout was skipped
because the previous one was still being digitised, and the ones whose
frame never fit on the wire. Dividing by the board's uptime_ms rather
than the host clock means a status frame that was itself dropped only
widens the window - it cannot corrupt it.

READING THE RESULT
Four columns tell the story together:

    trig_cps   what NOR_T1 actually did          <- the scan
    read_cps   what the firmware digitised       <- dead time
    host_cps   what survived the link            <- bandwidth
    excess     median ADC code of the signal channel, above pedestal

Sweeping downwards, trig_cps creeps up as more of the Compton continuum
clears the discriminator, then climbs by orders of magnitude within a
few DAC codes. Rate alone cannot say which of those is which: a real
source and a noise wall both make the number go up.

`excess` is what separates them, and it is why the scan reads the whole
window rather than just the signal channel. Channels 1-3 are the dark
references - same front end, no detector - so the median code across
them IS the pedestal, measured in the same events, at the same
temperature, with no separate run. A trigger from a gamma puts the
signal channel well above that. A trigger from noise does not. When
excess collapses toward zero, the rate has stopped being physics.

read_cps falling behind trig_cps is not a fault - it is the readout's
own dead time, about 38 us an event. host_cps falling behind read_cps
is the link, not the physics.

DWELL AND STATISTICS
A point stops early once it has collected --counts triggers, so a quiet
threshold gets its full --dwell while a noisy one is done as soon as
--min-dwell allows. The relative error on a point is 1/sqrt(counts),
which is why that column is in the CSV: 300 counts is 5.8%, and a point
that timed out at 9 counts is 33% and is not a measurement. At a 3 cps
background even 30 s buys only about 90 counts, so raise --dwell if the
quiet end has to be tight.
"""

import argparse
import csv
import json
import math
import os
import statistics
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError:
    serial = None

from rr2_decode import Decoder, LINK_BAUD, describe_ports, find_link_port

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SCAN_DIR = os.path.join(SCRIPT_DIR, "scans")

# The board sends status once a second, and a command that touches the
# ASIC can block the main loop for a good fraction of one. Four seconds
# is two beats plus slack.
CONFIRM_TIMEOUT_S = 4.0

# 'ch N dump' answers immediately - this only has to cover the round
# trip and whatever is queued ahead of it on the wire.
DUMP_TIMEOUT_S = 3.0

# Words in the dump lines that label the hex rather than being hex.
# Left in, "dac" itself parses as 0xDAC.
_GLOB_LABELS = {"glob:", "dac", "dly", "trig", "mux"}


# ===================================================================
#  Link
# ===================================================================
class Link:
    """Serial port plus decoder, pumped from the main thread.

    No reader thread on purpose: a scan is a strictly sequential thing
    - set, confirm, dwell, record - and the moment there is no GUI to
    keep responsive, a thread only buys race conditions.
    """

    def __init__(self, ser):
        self.ser = ser
        self.dec = Decoder()
        self.last_status = None     # most recent status frame
        self.status_frames = []     # since the last clear()
        self.events = []            # since the last clear()
        self.lines = []             # text replies, oldest first
        self.errors = []            # every ERR: line the board sent

    # -- transport ---------------------------------------------------
    def send(self, line):
        self.ser.write((line + "\n").encode("ascii", "ignore"))

    def pump(self, seconds=0.0):
        """Read and sort whatever arrives, for at least `seconds`."""
        end = time.monotonic() + seconds
        while True:
            waiting = self.ser.in_waiting
            # read(1) blocks up to the port timeout, which is what keeps
            # an idle link from spinning this loop at full speed.
            chunk = self.ser.read(waiting) if waiting else self.ser.read(1)
            if chunk:
                for frame in self.dec.feed(chunk):
                    kind = frame.get("type")
                    if kind == "status":
                        self.last_status = frame
                        self.status_frames.append(frame)
                    elif kind == "event":
                        self.events.append(frame)
                for line in self.dec.take_text():
                    self.lines.append(line)
                    if line.startswith("ERR:"):
                        self.errors.append(line)
            if time.monotonic() >= end:
                return

    def clear(self):
        self.status_frames.clear()
        self.events.clear()
        self.lines.clear()

    # -- helpers -----------------------------------------------------
    def wait_status(self, timeout=CONFIRM_TIMEOUT_S):
        """Block until a status frame lands. False if none does.

        Everything else here compares counters against a baseline, and
        there is no baseline until the first heartbeat has arrived.
        """
        end = time.monotonic() + timeout
        seen = self.last_status
        while time.monotonic() < end:
            self.pump(0.05)
            if self.last_status is not seen:
                return True
        return self.last_status is not None and seen is not None


def send_confirmed(link, cmd, timeout=CONFIRM_TIMEOUT_S):
    """Send one ASIC-touching command and wait for the board to own it.

    The 'ok' a command replies with is bare text sharing the wire with
    the binary frames, so a lost reply and a command that never ran look
    identical from here. The status frame carries a completed-commands
    counter that cannot be lost the same way - CRC protected, resent
    every second - so the question worth asking is whether that counter
    moved. Same reasoning as CmdWatch in rr2_gui.py.

    Returns (ok, message).
    """
    base = link.last_status
    if base is None or base.get("cmd_done") is None:
        return False, "no status frame to compare against"

    done0 = base["cmd_done"]
    failed0 = base.get("cmd_failed", 0)

    link.send(cmd)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        link.pump(0.05)
        st = link.last_status
        if st is None or st.get("cmd_done") is None:
            continue
        # Both counters wrap at 256, so only differences mean anything.
        if ((st["cmd_done"] - done0) & 0xFF) >= 1:
            if ((st.get("cmd_failed", 0) - failed0) & 0xFF):
                return False, ("the board ran it and the ASIC refused the "
                               "write (cmd_last=%s)" % st.get("cmd_last", "?"))
            return True, "confirmed by the board"
    return False, "not confirmed within %.0f s" % timeout


def verify_dac1(link, want, ch):
    """Read the threshold back and check it is the one asked for.

    The completed-commands counter moving proves the board ran A
    command; it cannot prove it ran THIS one. A line that lost a byte on
    the wire still parses as something, and something is what gets
    counted. Only the register coming back the way it was sent closes
    that gap.

    It does not close the other one. 'ch N dump' answers out of the
    firmware's RAM shadow, and RR2_Ctrl_SetThresholds() updates that
    shadow BEFORE it pushes to the ASIC - so a threshold the chip
    NACKed still reads back correctly here. That is what the failed
    counter in send_confirmed() is for. Neither check subsumes the
    other: one says the value is right, the other says the chip took it.
    """
    _, raw = read_dump(link, ch)
    if not raw:
        return False, "no readback - the dump never came back"
    got = decode_globals(raw).get("dac1")
    if got != want:
        return False, "readback says dac1=%s, not %d" % (got, want)
    return True, "confirmed by the board"


def restore_threshold(link, dac1, dac2, dacq, ch, attempts=2):
    """Put the threshold back, and do not take 'ok' for an answer.

    This runs from a finally block, which is exactly where the second
    Ctrl-C lands when someone holds the key down. An interrupt here
    would leave the chip parked wherever the sweep stopped - most
    likely at the bottom of it, buried in its own noise - and the next
    run would inherit that silently. So an interrupt during the restore
    is swallowed and the write retried.
    """
    msg = "not attempted"
    for _ in range(attempts):
        try:
            ok, msg = send_confirmed(link, "th %d %d %d" % (dac1, dac2, dacq))
            if ok:
                ok, msg = verify_dac1(link, dac1, ch)
            if ok:
                return True, msg
        except KeyboardInterrupt:
            msg = "interrupted - retrying, the board does not stay where "\
                  "the scan left it"
            print("  " + msg)
        except Exception as exc:                    # noqa: BLE001
            msg = "%s: %s" % (type(exc).__name__, exc)
    return False, msg


def _hex_after(text, drop_words):
    """Hex bytes of a dump line, ignoring the words that label them."""
    out = []
    for word in text.split():
        if word.lower() in drop_words:
            continue
        try:
            out.append(int(word, 16))
        except ValueError:
            pass
    return out


def read_dump(link, ch, timeout=DUMP_TIMEOUT_S):
    """'ch <n> dump' -> (channel bytes, global bytes), either may be None."""
    tag = "ch%d:" % ch
    link.lines.clear()
    link.send("ch %d dump" % ch)

    chb = globb = None
    end = time.monotonic() + timeout
    while time.monotonic() < end and (chb is None or globb is None):
        link.pump(0.05)
        for line in link.lines:
            if line.startswith(tag):
                chb = _hex_after(line, {tag})
            elif line.startswith("glob:"):
                globb = _hex_after(line, _GLOB_LABELS)
        link.lines.clear()
    return chb, globb


def decode_globals(b):
    """The three threshold DACs out of the eight global shadow bytes.

    Order matches cmd_dump() in usb_cmd.c: dac1_lo, dac2_dac1,
    dacq_dac2, dacq_hi, delay, slope, hyst_trig, out_power. The three
    10-bit DACs are interleaved across the first four, which is why a
    threshold cannot be read back one byte at a time.

    Only the fields a scan needs are decoded here; rr2_gui.py has the
    full version. A wrong answer here fails loudly - the readback stops
    matching what was sent and every point is marked unconfirmed.
    """
    if len(b) < 8:
        return {}
    return {
        "dac1": b[0] | ((b[1] & 0x03) << 8),
        "dac2": ((b[1] >> 2) & 0x3F) | ((b[2] & 0x0F) << 6),
        "dacq": ((b[2] >> 4) & 0x0F) | ((b[3] & 0x3F) << 4),
        "hold_delay": b[4],
        "slope_trim": (b[5] >> 4) & 0x0F,
        "selTrig": b[6] & 0x0F,
        "amux_lg": bool(b[7] & 0x01),
    }


def read_stat(link, timeout=DUMP_TIMEOUT_S):
    """'stat' as a dict. Values are ints; missing keys just stay out."""
    link.lines.clear()
    link.send("stat")
    out = {}
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        link.pump(0.05)
        for line in link.lines:
            if "=" in line and " " not in line:
                key, _, val = line.partition("=")
                try:
                    out[key] = int(val)
                except ValueError:
                    pass
        link.lines.clear()
        if "hold_ns" in out:        # the last key 'stat' prints
            break
    return out


# ===================================================================
#  One point
# ===================================================================
def reduce_events(frames, sig_ch):
    """Event frames -> (t_ms, signal code, pedestal code) triples.

    Every channel in the readout window other than the signal one is a
    dark reference: same front end, nothing connected. Their median is
    the pedestal for that very event - same temperature, no separate
    run - which is what makes the energy of a trigger judgeable at all.
    """
    out = []
    for e in frames:
        lo, n = e["first_ch"], e["count"]
        if not lo <= sig_ch < lo + n:
            continue
        dark = [v for i, v in enumerate(e["lg"][:n]) if lo + i != sig_ch]
        out.append((e["t_ms"], e["lg"][sig_ch - lo],
                    statistics.median(dark) if dark else None))
    return out


def window_counts(frames):
    """Triggers accumulated across the status frames collected so far.

    The stop condition of a dwell, and the only honest way to ask it:
    the counters are the board's, so a status frame that never arrived
    costs resolution, not correctness.
    """
    if len(frames) < 2:
        return 0
    return (frames[-1]["triggers"] - frames[0]["triggers"]) & 0xFFFFFFFF


def summarise_window(frames, seen):
    """One point's row, from its status frames and its reduced events.

    Pure: no link, no waiting. The CLI collects `frames` and `seen` in a
    blocking loop and the GUI collects them from its Tk pump, and both
    hand them here so there is one definition of what a point means.
    """
    row = {"status_frames": len(frames)}

    if len(frames) < 2:
        # Either the link went away or the ring was so full of events
        # that both heartbeats inside the window were dropped. Neither
        # is a rate, and neither should be written as one.
        row["note"] = "fewer than two status frames - no window to divide by"
        return row

    first, last = frames[0], frames[-1]
    window_ms = (last["uptime_ms"] - first["uptime_ms"]) & 0xFFFFFFFF
    if window_ms == 0:
        row["note"] = "zero-length window"
        return row
    window_s = window_ms / 1000.0

    def delta(key):
        return (last[key] - first[key]) & 0xFFFFFFFF

    trig = delta("triggers")
    read = delta("events_ok")

    # Keep only the events whose timestamp falls inside the same window
    # the counters were differenced over, so host_cps is comparable to
    # the other two rather than to a slightly different slice of time.
    inside = [t for t in seen
              if first["uptime_ms"] <= t[0] <= last["uptime_ms"]]
    sig = [t[1] for t in inside]
    ped = [t[2] for t in inside if t[2] is not None]

    sig_med = float(statistics.median(sig)) if sig else None
    ped_med = float(statistics.median(ped)) if ped else None

    row.update({
        "window_s": round(window_s, 3),
        "trig_counts": trig,
        "trig_cps": round(trig / window_s, 3),
        "read_cps": round(read / window_s, 3),
        "host_cps": round(len(sig) / window_s, 3),
        "dropped": delta("dropped"),
        "events_bad": delta("events_bad"),
        # What fraction of triggers never became an event. Readout dead
        # time, not a fault, until it approaches 1.
        "dead_frac": round(1.0 - (read / trig), 4) if trig else "",
        "sig_n": len(sig),
        "sig_med": round(sig_med, 1) if sig_med is not None else "",
        "sig_mean": round(float(statistics.fmean(sig)), 1) if sig else "",
        "ped_med": round(ped_med, 1) if ped_med is not None else "",
        "excess": (round(sig_med - ped_med, 1)
                   if sig_med is not None and ped_med is not None else ""),
        "temp_c": last.get("temp_c", ""),
        "note": "",
    })
    return row


def measure_point(link, sig_ch, min_dwell, max_dwell, target_counts):
    """Dwell at the current threshold and return what the board counted.

    Stops once `target_counts` triggers have accumulated, but never
    before `min_dwell` - a window shorter than a couple of status beats
    has too coarse a denominator to divide by.
    """
    link.clear()
    t0 = time.monotonic()

    # Reduced as they arrive rather than held as frames: half a minute
    # at a few thousand events a second is a lot of decoded dicts to be
    # sitting on for the sake of two medians.
    seen = []

    while True:
        link.pump(0.1)
        seen.extend(reduce_events(link.events, sig_ch))
        link.events.clear()

        elapsed = time.monotonic() - t0
        if elapsed >= max_dwell:
            break
        if (elapsed >= min_dwell
                and window_counts(link.status_frames) >= target_counts):
            break

    return summarise_window(list(link.status_frames), seen)


# ===================================================================
#  Output
# ===================================================================
COLUMNS = ["dac1", "trig_cps", "trig_counts", "read_cps", "host_cps",
           "dead_frac", "dropped", "events_bad", "excess", "sig_med",
           "sig_mean", "ped_med", "sig_n", "window_s", "status_frames",
           "temp_c", "confirmed", "note"]


def write_outputs(path_base, header, rows):
    """CSV for plotting, JSON for everything the CSV has no column for."""
    csv_path = path_base + ".csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in COLUMNS})

    json_path = path_base + ".json"
    with open(json_path, "w", encoding="utf-8") as fh:
        json.dump({"header": header, "points": rows}, fh, indent=1)

    return csv_path, json_path


def print_table_header():
    print()
    print("  dac1   trig_cps  counts   read_cps   host_cps  dead   excess")
    print("  " + "-" * 62)


def _num(value, fmt="%.1f", dash="   -"):
    return (fmt % value) if value != "" and value is not None else dash


def print_row(row):
    if "trig_cps" not in row:
        print("  %4d   %s" % (row["dac1"], row.get("note", "no data")))
        return
    dead = row["dead_frac"]
    print("  %4d  %9.2f  %6d  %9.2f  %9.2f  %5s  %7s%s"
          % (row["dac1"], row["trig_cps"], row["trig_counts"],
             row["read_cps"], row["host_cps"],
             _num(dead * 100 if dead != "" else "", "%.0f%%"),
             _num(row.get("excess", "")),
             "" if row["confirmed"] else "   <- NOT CONFIRMED"))


def print_chart(rows):
    """Log-scale bars. The knee is the point of the whole exercise, and
    on a linear axis a jump from 2 to 3000 cps hides every point below
    the last one."""
    pts = [r for r in rows if r.get("trig_cps", 0) > 0]
    if len(pts) < 2:
        return
    top = max(r["trig_cps"] for r in pts)
    lo = math.log10(min(r["trig_cps"] for r in pts))
    span = math.log10(top) - lo

    # A log axis normalised to the span turns third-decimal noise into
    # full-width bars when every point is the same rate. Under a factor
    # of two, scale from zero instead and let a flat scan look flat.
    flat = span < 0.3

    print()
    print("  trigger rate, %s" % ("linear" if flat else "log scale"))
    for row in rows:
        rate = row.get("trig_cps", 0)
        if not rate:
            print("  %4d  |" % row["dac1"])
            continue
        frac = rate / top if flat else (math.log10(rate) - lo) / span
        print("  %4d  |%s %.1f"
              % (row["dac1"], "#" * max(int(round(46 * frac)), 1), rate))


# A point with fewer events than this has no median worth trusting.
MIN_EVENTS_FOR_EXCESS = 20

# Below this fraction of the best excess seen, the triggers are landing
# on the pedestal and are not carrying energy any more.
EXCESS_FLOOR_FRAC = 0.3


def steepest_lines(rows):
    """The steepest step between neighbouring points, in decades.

    Cross-check only. On an exponential noise floor the steepest step
    lands well past the wall, deep inside a region that is already all
    noise, which is why it is not what the recommendation is built on.
    """
    pts = [r for r in rows if r.get("trig_cps", 0) > 0]
    if len(pts) < 2:
        return ["Fewer than two points produced a rate - nothing to compare."]

    decades, a, b = max(
        ((abs(math.log10(b["trig_cps"] / a["trig_cps"])), a, b)
         for a, b in zip(pts, pts[1:])), key=lambda t: t[0])

    if decades < 0.5:
        return ["Rate never moved by half a decade between neighbouring "
                "points.",
                "Either the sweep did not reach the noise, or nothing is "
                "triggering at all."]
    return ["Steepest rate step: dac1 %d -> %d, %.1f -> %.1f cps "
            "(%.1f decades)" % (a["dac1"], b["dac1"], a["trig_cps"],
                                b["trig_cps"], decades)]


def recommend(rows, dac2, dacq):
    """Which threshold to actually use and why, as lines of text.

    Rate alone cannot answer this. Lowering the threshold admits more of
    the Compton continuum AND more noise, and both show up as the number
    going up. The energy excess over the dark reference channels is what
    separates them, so that is what picks the wall: a point whose median
    signal has collapsed onto the pedestal is noise, however impressive
    its rate.

    The pick is then the clean point with the highest rate - the lowest
    threshold that still buys real events, which is the whole objective.

    Returns lines rather than printing them, so the CLI and the GUI say
    the same thing in their own furniture.
    """
    usable = [r for r in rows
              if r.get("trig_cps", 0) > 0 and r.get("excess", "") != ""
              and r.get("sig_n", 0) >= MIN_EVENTS_FOR_EXCESS]

    if len(usable) < 2:
        return (["Not enough events to judge which triggers carried energy "
                 "(a point needs %d" % MIN_EVENTS_FOR_EXCESS,
                 "events for a median). Rate-only view:"]
                + steepest_lines(rows))

    healthy = max(r["excess"] for r in usable)
    floor = EXCESS_FLOOR_FRAC * healthy
    clean = [r for r in usable if r["excess"] >= floor]
    noisy = [r for r in usable if r["excess"] < floor]

    if not noisy:
        return (["No noise wall in this range: every point still carries "
                 "energy (excess >= %.0f" % floor,
                 "ADC codes throughout). Sweep further down to find it."]
                + steepest_lines(rows))

    # The noisy point nearest the boundary is the slowest one, whichever
    # way the sweep ran.
    wall = min(noisy, key=lambda r: r["trig_cps"])
    out = ["Wall: at dac1 %d the median signal is %.0f codes above the "
           "pedestal, against %.0f" % (wall["dac1"], wall["excess"], healthy),
           "at the top of the scan - those %.0f cps are noise, not counts."
           % wall["trig_cps"]]

    if not clean:
        return out + ["", "No point in this sweep stayed clean. Start higher."]

    pick = max(clean, key=lambda r: r["trig_cps"])
    return out + [
        "",
        "Best clean point: dac1=%d, %.1f cps, excess %.0f codes"
        % (pick["dac1"], pick["trig_cps"], pick["excess"]),
        "    th %d %d %d" % (pick["dac1"], dac2, dacq),
        "Leave margin above it: the wall moves with temperature and with "
        "HV, and a",
        "threshold that was clean at 25 C can be inside the noise at 60 C.",
    ] + steepest_lines(rows)


# ===================================================================
#  Main
# ===================================================================
def sweep_values(start, stop, step, values=None):
    """The DAC1 values to visit. An explicit list wins over the range.

    Raises ValueError on anything a sweep cannot be made of, so the CLI
    and the GUI reject the same input with the same words.
    """
    if values:
        out = [int(v) for v in str(values).replace(",", " ").split()]
    else:
        step = abs(int(step)) or 1
        start, stop = int(start), int(stop)
        out = (list(range(start, stop - 1, -step)) if stop <= start
               else list(range(start, stop + 1, step)))

    if not out:
        raise ValueError("empty sweep - check start / stop / step")
    if any(not 0 <= v <= 1023 for v in out):
        raise ValueError("dac1 is a 10-bit code: every value must be 0..1023")
    return out


def build_values(args):
    return sweep_values(args.start, args.stop, args.step, args.values)


def main():
    ap = argparse.ArgumentParser(
        description="Automatic DAC1 threshold scan: rate at every threshold.")
    ap.add_argument("--port", help="serial port. Omit and the ST-Link is found")
    ap.add_argument("--baud", type=int, default=LINK_BAUD)
    ap.add_argument("--list", action="store_true", help="list ports and exit")

    ap.add_argument("--start", type=int, default=320, help="first dac1 (320)")
    ap.add_argument("--stop", type=int, default=80, help="last dac1 (80)")
    ap.add_argument("--step", type=int, default=20, help="dac1 step (20)")
    ap.add_argument("--values", help="explicit list instead, e.g. 300,280,260")

    ap.add_argument("--dwell", type=float, default=30.0,
                    help="seconds per point, upper bound (30)")
    ap.add_argument("--min-dwell", type=float, default=6.0,
                    help="seconds per point, lower bound (6)")
    ap.add_argument("--counts", type=int, default=300,
                    help="triggers per point before moving on (300)")
    ap.add_argument("--max-rate", type=float, default=None,
                    help="stop the sweep once a point exceeds this cps")

    ap.add_argument("--channel", type=int, default=0,
                    help="the channel carrying the SiPM (0)")
    ap.add_argument("--restore", type=int, default=None,
                    help="dac1 to leave the board at (default: where it was)")
    ap.add_argument("--label", default="", help="goes into the file names")
    args = ap.parse_args()

    if args.list:
        ports = describe_ports()
        if ports is None:
            print("pyserial is not installed:  pip install pyserial")
            return 2
        for dev, desc, is_link in ports:
            print("%-12s %s%s" % (dev, desc, "   <- ST-Link" if is_link else ""))
        return 0

    if serial is None:
        print("pyserial is not installed:  pip install pyserial")
        return 2

    try:
        values = build_values(args)
    except ValueError as exc:
        print(exc)
        return 2

    port = args.port or find_link_port()
    if not port:
        print("No ST-Link found. Plug the board in, or pass --port. "
              "'--list' shows what is there.")
        return 2

    try:
        ser = serial.Serial(port, args.baud, timeout=0.05)
    except Exception as exc:
        print("Could not open %s: %s" % (port, exc))
        print("A serial port has one owner - close rr2_gui.py first.")
        return 2

    with ser:
        return run_scan(Link(ser), port, args, values)


def run_scan(link, port, args, values):
    print("RADIOROC2 threshold scan on %s at %d baud" % (port, args.baud))
    print("%d points: %s" % (len(values), ", ".join(str(v) for v in values)))

    # Nothing below can be trusted until the board has been heard from:
    # every confirmation here is a counter differenced against a
    # baseline, and there is no baseline yet.
    print("waiting for the first status frame...")
    if not link.wait_status(6.0) or link.last_status is None:
        print("No status frame in six seconds. The board is not streaming - "
              "wrong baud, firmware not running, or the wrong port.")
        return 2

    stat = read_stat(link)
    if stat.get("online") == 0:
        print("ASIC is offline (online=0): main() skipped the whole "
              "configuration sequence and no threshold will apply.")
        return 2

    _, glob_raw = read_dump(link, args.channel)
    if not glob_raw:
        print("Could not read the global registers back. Without a readback "
              "there is no way to tell a threshold that applied from one "
              "that did not - refusing to scan blind.")
        return 2

    glob0 = decode_globals(glob_raw)
    dac1_0 = glob0["dac1"]
    dac2, dacq = glob0["dac2"], glob0["dacq"]
    restore_to = args.restore if args.restore is not None else dac1_0

    print("board: dac1=%d dac2=%d dacq=%d  hold_ns=%s  selTrig=0x%X"
          % (dac1_0, dac2, dacq, stat.get("hold_ns", "?"), glob0["selTrig"]))
    if glob0["selTrig"] not in (0x4, 0x1):
        print("  NOTE: selTrig=0x%X is not the T1 global trigger. This scan "
              "moves DAC1, which is the T1 threshold - if the acquisition "
              "trigger is sourced elsewhere the rate will not follow it."
              % glob0["selTrig"])
    print("dac2 and dacq are carried through unchanged; only dac1 moves.")
    print("the board goes back to dac1=%d when this ends." % restore_to)

    header = {
        "started": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "port": port,
        "baud": args.baud,
        "label": args.label,
        "signal_channel": args.channel,
        "values": values,
        "dwell_s": args.dwell,
        "min_dwell_s": args.min_dwell,
        "target_counts": args.counts,
        "max_rate_cps": args.max_rate,
        "dac1_at_start": dac1_0,
        "dac2": dac2,
        "dacq": dacq,
        "restore_to": restore_to,
        "global_at_start": glob0,
        "global_raw_at_start": " ".join("%02X" % v for v in glob_raw),
        "stat_at_start": stat,
    }

    rows = []
    stopped = ""
    print_table_header()

    try:
        for dac1 in values:
            ok, msg = send_confirmed(link, "th %d %d %d" % (dac1, dac2, dacq))
            if ok:
                ok, msg = verify_dac1(link, dac1, args.channel)

            row = {"dac1": dac1, "confirmed": bool(ok)}
            if not ok:
                row["note"] = msg
                row["status_frames"] = 0
                rows.append(row)
                print_row(row)
                continue

            row.update(measure_point(link, args.channel, args.min_dwell,
                                     args.dwell, args.counts))
            row["dac1"] = dac1
            row["confirmed"] = True
            rows.append(row)
            print_row(row)

            if args.max_rate is not None and row.get("trig_cps", 0) > args.max_rate:
                stopped = ("stopped at dac1=%d: %.0f cps is past --max-rate %.0f"
                           % (dac1, row["trig_cps"], args.max_rate))
                print("\n  " + stopped)
                break

    except KeyboardInterrupt:
        stopped = "interrupted by the operator"
        print("\n  interrupted - keeping the %d points already taken" % len(rows))

    finally:
        # Whatever happened above, the chip does not stay where the scan
        # left it.
        restore_ok, restore_msg = restore_threshold(link, restore_to, dac2,
                                                    dacq, args.channel)
        print("\n  restore dac1=%d: %s" % (restore_to, restore_msg))
        if not restore_ok:
            print("  THE BOARD MAY STILL BE AT A SCAN THRESHOLD. Check with "
                  "'ch %d dump' before the next run." % args.channel)

    header["stopped"] = stopped
    header["finished"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    header["restore_confirmed"] = restore_ok
    header["board_errors"] = link.errors

    os.makedirs(SCAN_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = "thscan_%s%s" % (stamp, ("_" + args.label) if args.label else "")
    csv_path, json_path = write_outputs(os.path.join(SCAN_DIR, name),
                                        header, rows)

    print_chart(rows)
    print()
    for line in recommend(rows, dac2, dacq):
        print("  " + line if line else "")

    if link.errors:
        print()
        print("  the board sent %d ERR line(s) during the scan:" % len(link.errors))
        for line in link.errors[:5]:
            print("    " + line)

    print()
    print("  %s" % csv_path)
    print("  %s" % json_path)

    # Exit non-zero when the run is not one you can act on: a sweep that
    # measured nothing, or a board left somewhere other than where it
    # started. Both are easy to miss in a wall of table output, and both
    # matter to whatever runs next.
    measured = sum(1 for r in rows if "trig_cps" in r)
    if measured == 0:
        print("\n  NO POINT PRODUCED A RATE - nothing here is a measurement.")
        return 1
    if not restore_ok:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
