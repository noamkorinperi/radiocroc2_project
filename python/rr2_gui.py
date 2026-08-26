#!/usr/bin/env python3
"""
rr2_gui.py - desktop front end for the RADIOROC2 gamma spectrometer.

Pages
    Main       navigation hub
    Settings   push Slow Control parameters to the ASIC
    Measure    run an acquisition and watch the spectrum build up
    History    saved measurements, with Excel export
    Help       static instructions

Requirements
    pyserial   pip install pyserial      (hardware link)
    openpyxl   pip install openpyxl      (Excel export; CSV fallback otherwise)
    pillow     pip install pillow        (PNG screenshots)

rr2_decode.py must sit next to this file - it owns the wire protocol so
there is only one place to change if the framing ever moves.

Run without hardware:
    python rr2_gui.py --sim
"""

import argparse
import csv
import ctypes
import json
import os
import queue
import random
import sys
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import filedialog, font as tkfont, messagebox, ttk

# ---------------------------------------------------------------- protocol
try:
    from rr2_decode import Decoder, LINK_BAUD, find_link_port
except ImportError:
    sys.exit("rr2_decode.py must be in the same folder as rr2_gui.py")

try:
    import serial
    from serial.tools import list_ports
    HAVE_SERIAL = True
except ImportError:
    HAVE_SERIAL = False

try:
    from openpyxl import Workbook
    from openpyxl.chart import LineChart, Reference
    HAVE_XLSX = True
except ImportError:
    HAVE_XLSX = False

try:
    from PIL import ImageGrab
    HAVE_PIL = True
except ImportError:
    HAVE_PIL = False


ADC_MAX = 4095                      # 12-bit ADC
ADC_VREF_V = 3.3                    # VREF+ is tied to VDDA on the Nucleo

# OUT_AMUXLG cannot swing past 1.32 V. That is the ASIC output buffer's
# ceiling, not a setting, and nothing amplifies the line between the
# ASIC and PA5 - so the top 60% of the ADC's range is unreachable and a
# clipped event lands at LG_CEIL_CODE, never at 4095. Histogramming to
# 4095 spent 60% of the axis on codes that cannot occur and, worse, put
# the saturation detector somewhere the signal can never reach.
LG_MAX_V = 1.32
LG_CEIL_CODE = int(LG_MAX_V / ADC_VREF_V * (ADC_MAX + 1))       # 1638

# A peak detector pinned at the ceiling reads the ceiling plus noise,
# not more than it, so saturation shows up as a pile-up just below
# LG_CEIL_CODE rather than as anything out of range. Call the top 1%
# saturated.
LG_SAT_CODE = LG_CEIL_CODE - max(8, LG_CEIL_CODE // 100)        # 1622
STORE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "measurements")
RAW_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "raw")
SHOT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "screenshots")
PREFS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "gui_prefs.json")

# A thermal sweep is six temperature points, and it gets repeated. Six
# slots meant the pedestal run was deleted on the way to the last point,
# silently, at the end of a day in the chamber. Histograms are a few kB.
MAX_STORED = 200

# A restrained instrument palette: light background, one accent.
C_BG = "#f4f6f8"
C_PANEL = "#ffffff"
C_ACCENT = "#1f6feb"
C_ACCENT_DK = "#12457a"
C_TEXT = "#1c2430"
C_MUTED = "#6b7684"
C_GRID = "#dfe4ea"
C_BAR = "#1f6feb"
C_OK = "#1a7f37"
C_WARN = "#b26a00"
C_BAD = "#c0392b"


# ===================================================================
#  Preferences - the few choices that should outlive a session
# ===================================================================
def load_prefs():
    """Whatever was saved last time, or nothing. Never raises."""
    try:
        with open(PREFS_PATH, encoding="utf-8") as fh:
            prefs = json.load(fh)
        return prefs if isinstance(prefs, dict) else {}
    except (OSError, ValueError):
        return {}


def save_prefs(prefs):
    """Best effort. A preference is not worth an error dialog."""
    try:
        with open(PREFS_PATH, "w", encoding="utf-8") as fh:
            json.dump(prefs, fh, ensure_ascii=False, indent=1)
    except OSError:
        pass


# ===================================================================
#  Link: serial transport plus a simulator that speaks the same API
# ===================================================================
class Link:
    """Owns the port, a reader thread and the decoded-frame queue."""

    def __init__(self):
        self.ser = None
        self.sim = False
        self.frames = queue.Queue()
        self.text = queue.Queue()          # replies to typed commands
        self._stop = threading.Event()
        self._thread = None
        self._dec = Decoder()
        self._sim_state = {"rate": 180.0, "centre": 1100, "sigma": 41}
        # An open port proves nothing - the ST-Link enumerates whether or
        # not the firmware is running, and a wrong baud rate delivers
        # bytes that are pure noise. These are what tell the difference.
        self.rx_bytes = 0
        self.rx_at = 0.0                   # monotonic time of the last byte

    def stats(self):
        """Counters that say whether the wire is healthy, not just open."""
        return {
            "rx_bytes": self.rx_bytes,
            "rx_at": self.rx_at,
            "bad_crc": self._dec.bad_crc,
            "resyncs": self._dec.resyncs,
        }

    # ---- connection -------------------------------------------------
    @staticmethod
    def ports():
        """Available ports, ST-Link first so it lands under the cursor."""
        if not HAVE_SERIAL:
            return []
        devs = [p.device for p in list_ports.comports()]
        link = find_link_port()
        if link and link in devs:            # float it to the top
            devs.remove(link)
            devs.insert(0, link)
        return devs

    def open(self, port):
        self.close()
        if not HAVE_SERIAL:
            raise RuntimeError("pyserial is not installed")
        # A real UART now, not a CDC endpoint - the baud rate matters and
        # has to match RR2_LINK_BAUD in the firmware or every byte is junk.
        self.ser = serial.Serial(port, LINK_BAUD, timeout=0.05)
        self.sim = False
        self._start_reader(self._read_serial)

    def open_sim(self):
        self.close()
        self.sim = True
        self._start_reader(self._read_sim)

    def _start_reader(self, target):
        self.rx_bytes = 0
        self.rx_at = 0.0
        self._dec = Decoder()
        self._stop.clear()
        self._thread = threading.Thread(target=target, daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.sim = False

    @property
    def connected(self):
        return self.sim or (self.ser is not None and self.ser.is_open)

    # ---- transmit ---------------------------------------------------
    def send(self, line):
        """Send one text command. Returns True if it went out."""
        if self.sim:
            self.text.put(f"[sim] {line}")
            return True
        if not self.ser:
            return False
        try:
            self.ser.write((line + "\n").encode("ascii", "ignore"))
            return True
        except Exception as exc:
            self.text.put(f"TX error: {exc}")
            return False

    # ---- reader threads ---------------------------------------------
    def _read_serial(self):
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(4096)
            except Exception as exc:
                self.text.put(f"RX error: {exc}")
                return
            if not chunk:
                continue
            self.rx_bytes += len(chunk)
            self.rx_at = time.monotonic()
            # Text replies and binary frames share the pipe. The decoder
            # sorts them: frames come back from feed(), and the bytes it
            # rejected are recovered as lines by take_text(). Draining
            # both is what puts stat / sel / dump output - and every ERR:
            # the firmware sends - into the console instead of nowhere.
            for frame in self._dec.feed(chunk):
                self.frames.put(frame)
            for line in self._dec.take_text():
                self.text.put(line)

    def _read_sim(self):
        """Poisson-ish event generator with a Gaussian photopeak."""
        seq = 0
        t0 = time.time()
        next_status = t0 + 1.0
        while not self._stop.is_set():
            time.sleep(0.02)
            now = time.time()
            self.rx_bytes += 64
            self.rx_at = time.monotonic()
            n = max(0, int(random.gauss(self._sim_state["rate"] * 0.02,
                                        self._sim_state["rate"] * 0.02 * 0.4)))
            for _ in range(n):
                seq += 1
                if random.random() < 0.72:
                    lg = random.gauss(self._sim_state["centre"],
                                      self._sim_state["sigma"])
                else:
                    lg = random.expovariate(1 / 320.0)      # Compton-ish tail
                # Clipped at the ASIC's ceiling, not the ADC's full
                # scale, so the simulator can reproduce saturation.
                lg = int(max(0, min(LG_CEIL_CODE, lg)))
                self.frames.put({
                    "type": "event", "seq": seq,
                    "t_ms": int((now - t0) * 1000),
                    "temp_c": 24.8 + 0.4 * random.random(),
                    "first_ch": 0, "count": 1,
                    "lg": [lg],
                })
            if now >= next_status:
                next_status = now + 1.0
                self.frames.put({
                    "type": "status", "uptime_ms": int((now - t0) * 1000),
                    "triggers": seq, "events_ok": seq, "events_bad": 0,
                    "dropped": 0, "temp_c": 25.0,
                    "rr2_online": True, "temp_online": True,
                    "timing_ok": True, "cfg_status": 0, "read_status": 0,
                })


# ===================================================================
#  Shadow register decoding
# ===================================================================
# Field positions mirror radioroc2_regs.h. Slow Control is write-only on
# this board, so the firmware's shadow - what "ch N dump" prints - is the
# only readback of the ASIC there is. Decoding it here is what lets a
# saved run carry the settings it was taken under, instead of a promise
# that the form on screen was the form that got pushed.

def decode_channel(b):
    """The eight per-channel shadow bytes, as named settings."""
    if len(b) < 8:
        return {}
    sub6, sub7 = b[6], b[7]
    return {
        "inDac": b[0],
        "patGain": b[1] & 0x3F,
        "lgGain": (b[2] >> 4) & 0x0F,
        "hgGain": b[2] & 0x0F,
        "tauLG": (b[3] >> 4) & 0x0F,
        "tauHG": b[3] & 0x0F,
        "trimT1": b[4] & 0x3F,
        "trimT2": b[5] & 0x3F,
        "slowLG": bool(sub7 & 0x80),
        "slowHG": bool(sub7 & 0x40),
        "ctest": bool(sub7 & 0x10),
        # The LG readout path end to end: preamp, shaper, peak detector.
        # Any one of them off and the channel reports only a pedestal.
        "lg_path_on": (bool(sub6 & 0x02) and bool(sub7 & 0x08)
                       and bool(sub7 & 0x02)),
        # Separate on purpose - a reference channel keeps its readout and
        # loses only its right to fire the trigger.
        "discri_on": bool(sub6 & 0x1C),
    }


def decode_globals(b):
    """The eight global shadow bytes from the "glob:" line.

    Order matches cmd_dump(): dac1_lo, dac2_dac1, dacq_dac2, dacq_hi,
    delay, slope, hyst_trig, out_power.
    """
    if len(b) < 8:
        return {}
    dac1 = b[0] | ((b[1] & 0x03) << 8)
    dac2 = ((b[1] >> 2) & 0x3F) | ((b[2] & 0x0F) << 6)
    dacq = ((b[2] >> 4) & 0x0F) | ((b[3] & 0x3F) << 4)
    return {
        "dac1": dac1,
        "dac2": dac2,
        "dacq": dacq,
        "hold_delay": b[4],
        "slope_trim": (b[5] >> 4) & 0x0F,
        "selTrig": b[6] & 0x0F,
        "hold_external": bool(b[6] & 0x10),
        "amux_lg": bool(b[7] & 0x01),
        "abuffer": bool(b[7] & 0x08),
    }


class DeviceState:
    """What the board last said about itself, in named values.

    "stat" and "ch N dump" answer as bare text on the same UART as the
    frames, so the recovered line is the only place those numbers exist
    on this side. Parsing them once, here, means the console and the
    saved record can both have them.
    """

    def __init__(self):
        self.stat = {}                 # key=value pairs from "stat"
        self.ch_raw = {}               # channel -> [8 shadow bytes]
        self.glob_raw = []
        self.stamp = 0.0               # monotonic time of the last update

    def feed(self, line):
        """Absorb one recovered text line. True if it was one of ours."""
        line = line.strip()
        if not line:
            return False

        # stat answers in bare key=value, one per line.
        if "=" in line and " " not in line:
            key, _, val = line.partition("=")
            try:
                self.stat[key] = int(val)
            except ValueError:
                self.stat[key] = val
            self.stamp = time.monotonic()
            return True

        if line.startswith("glob:"):
            vals = self._hex_after(line, ("glob:", "dac", "dly", "trig", "mux"))
            if len(vals) >= 8:
                self.glob_raw = vals[:8]
                self.stamp = time.monotonic()
                return True
            return False

        # "ch7: 80 20 40 EE 00 00 7F 8F"
        if line.startswith("ch") and ":" in line:
            head, _, rest = line.partition(":")
            try:
                ch = int(head[2:])
            except ValueError:
                return False
            vals = self._hex_list(rest)
            if len(vals) >= 8:
                self.ch_raw[ch] = vals[:8]
                self.stamp = time.monotonic()
                return True
        return False

    @staticmethod
    def _hex_list(text):
        out = []
        for word in text.split():
            try:
                out.append(int(word, 16))
            except ValueError:
                pass
        return out

    @classmethod
    def _hex_after(cls, text, drop_words):
        """Hex bytes of a line, ignoring the words that label them.

        The glob line interleaves names with values - "dac 2C 4B ... dly
        FF F0 ..." - so the words have to come out before the hex does.
        Left in, "dac" itself parses as 0xDAC.
        """
        keep = " ".join(w for w in text.split()
                        if w.lower() not in drop_words)
        return cls._hex_list(keep)

    def snapshot(self, channel=None):
        """A frozen, JSON-safe copy of everything known about the board."""
        out = {"stat": dict(self.stat)}
        if self.glob_raw:
            out["global"] = decode_globals(self.glob_raw)
            out["global_raw"] = " ".join("%02X" % v for v in self.glob_raw)
        if channel is not None and channel in self.ch_raw:
            raw = self.ch_raw[channel]
            out["channel"] = channel
            out["channel_settings"] = decode_channel(raw)
            out["channel_raw"] = " ".join("%02X" % v for v in raw)
        return out

    def has_dump(self, channel):
        return channel in self.ch_raw and bool(self.glob_raw)


# ===================================================================
#  Link health
# ===================================================================
class LinkHealth:
    """Is the link actually carrying data, or is the port merely open?

    An open port proves only that the ST-Link enumerated. The firmware
    is not necessarily running, the baud rate may be wrong, and a loose
    wire delivers bytes that decode to nothing. The firmware sends a
    status frame once a second whatever else is happening, so a missing
    heartbeat is the one unambiguous sign that the link is down rather
    than the source being quiet.
    """

    DEAD_S = 3.5          # heartbeats are 1 s apart; three missed is dead
    LATE_S = 2.2          # one missed, and something is wrong
    MISSED_BEAT_S = 1.8   # by the board's clock: a beat that never came
    ERR_WINDOW_S = 10.0   # how long an error still counts against the link

    OFFLINE, DEAD, UNSTABLE, OK = "offline", "dead", "unstable", "ok"

    def __init__(self):
        self.reset()

    def reset(self):
        self.started_at = time.monotonic()   # when this link began
        self.status_at = 0.0
        self.event_at = 0.0
        self.status_count = 0
        self.event_count = 0
        self.gap_worst = 0.0           # worst gap, by the board's clock
        self.last_err_at = 0.0
        self.err_note = ""
        self.board = {}                # last status frame, as sent
        self.dropped_delta = 0
        self.bad_delta = 0
        self.bus_jam_seen = False
        self._crc = 0
        self._byte_at = 0.0

    # -- inputs
    def on_frame(self, frame):
        now = time.monotonic()
        kind = frame.get("type")
        if kind == "event":
            self.event_at = now
            self.event_count += 1
            return
        if kind != "status":
            return

        self.status_at = now
        self.status_count += 1

        prev, self.board = self.board, dict(frame)

        # Time the gap by the board's own clock, not this one. The frame
        # carries the uptime it was built at, so a host that was busy
        # repainting for two seconds and drained three frames at once
        # does not get reported as a link that skipped a beat.
        if prev:
            gap = (frame.get("uptime_ms", 0) - prev.get("uptime_ms", 0)) / 1000.0
            if gap < 0:
                self._flag(now, "the board reset - uptime went backwards")
                self.bus_jam_seen = False      # a new boot, a new verdict
            else:
                self.gap_worst = max(self.gap_worst, gap)
                if gap > self.MISSED_BEAT_S:
                    self._flag(now, "the board skipped %.1f s of heartbeats"
                                    % gap)

        # Counters the firmware keeps for itself. Rising means the board
        # is losing data on its own side, which no amount of port health
        # on this side would ever show.
        self.dropped_delta = (max(0, frame.get("dropped", 0)
                                  - prev.get("dropped", 0)) if prev else 0)
        self.bad_delta = (max(0, frame.get("events_bad", 0)
                              - prev.get("events_bad", 0)) if prev else 0)
        if self.dropped_delta:
            self._flag(now, "%d frames dropped by the board"
                            % self.dropped_delta)
        if self.bad_delta:
            self._flag(now, "%d failed readouts" % self.bad_delta)
        if not frame.get("rr2_online", True):
            self._flag(now, "ASIC offline - Slow Control is not answering")
        if not frame.get("timing_ok", True):
            self._flag(now, "DWT timing unavailable - hold delay is wrong")
        # Once per boot, not once per frame. It says something happened
        # before the firmware started, and the firmware dealt with it -
        # raising it every second would pin the lamp amber for a fault
        # that is already over, which is how cfg_status used to read.
        if frame.get("bus_jam") and not self.bus_jam_seen:
            self.bus_jam_seen = True
            self._flag(now, "I2C was jammed at boot and was clocked free "
                            "- something reset mid-transfer")
        if frame.get("cfg_status") or frame.get("read_status"):
            self._flag(now, "ASIC error cfg=%s read=%s"
                            % (frame.get("cfg_status"),
                               frame.get("read_status")))

    def on_stats(self, st):
        """Decoder counters. A rising CRC count is a corrupt wire.

        Resyncs are not counted: every text reply the firmware sends is
        a resync, so they say nothing about the link's health.
        """
        self._byte_at = st.get("rx_at", 0.0)
        crc = st.get("bad_crc", 0)
        if crc > self._crc:
            self._flag(time.monotonic(),
                       "%d frames failed CRC" % (crc - self._crc))
        self._crc = crc

    def _flag(self, now, note):
        self.last_err_at = now
        self.err_note = note

    # -- output
    def level(self, connected):
        if not connected:
            return self.OFFLINE
        now = time.monotonic()
        if not self.status_at:
            # No heartbeat yet. For the first few seconds after opening
            # the port that is not a verdict, it is not knowing - the
            # board may simply not have reached its next second.
            if now - self.started_at < self.DEAD_S:
                return self.UNSTABLE
            return self.DEAD
        age = now - self.status_at
        if age > self.DEAD_S:
            return self.DEAD
        if age > self.LATE_S:
            return self.UNSTABLE
        if self.last_err_at and (now - self.last_err_at) < self.ERR_WINDOW_S:
            return self.UNSTABLE
        return self.OK

    def heartbeat_age(self):
        return (time.monotonic() - self.status_at) if self.status_at else None

    def event_age(self):
        return (time.monotonic() - self.event_at) if self.event_at else None

    def summary(self, connected):
        """The verdict, and the number standing behind it."""
        lvl = self.level(connected)
        age = self.heartbeat_age()
        if lvl == self.OFFLINE:
            return lvl, "no port open"
        if lvl == self.DEAD:
            if age is None:
                # Bytes with no frame in them is a link that is talking
                # and not being understood, which is a different fault
                # from a link that is silent.
                if self._byte_at > self.started_at:
                    return lvl, ("bytes are arriving but none of them "
                                 "decode - check the baud rate")
                return lvl, ("nothing received - wrong port, or the "
                             "firmware is not running")
            return lvl, "no heartbeat for %.1f s" % age
        if lvl == self.UNSTABLE:
            if age is None:
                return lvl, "waiting for the first heartbeat"
            if self.err_note:
                return lvl, self.err_note
            return lvl, "heartbeat %.1f s late" % age
        return lvl, ("heartbeat %.1f s ago, %d received"
                     % (age, self.status_count))

    def snapshot(self, connected):
        """The link's own vital signs, to be saved with a measurement."""
        lvl, why = self.summary(connected)
        return {
            "level": lvl,
            "detail": why,
            "status_frames": self.status_count,
            "events_seen": self.event_count,
            "worst_heartbeat_gap_s": round(self.gap_worst, 2),
            "bad_crc": self._crc,
            "board": {k: v for k, v in self.board.items() if k != "type"},
        }


# ===================================================================
#  Histogram
# ===================================================================
class Histogram:
    def __init__(self, bins=512, lo=0, hi=LG_CEIL_CODE):
        self.reset(bins, lo, hi)

    def reset(self, bins=None, lo=None, hi=None):
        if bins is not None:
            self.bins = int(bins)
        if lo is not None:
            self.lo = int(lo)
        if hi is not None:
            self.hi = int(hi)
        self.counts = [0] * self.bins
        self.total = 0
        self.overflow = 0
        self.saturated = 0

    @property
    def width(self):
        return (self.hi - self.lo) / float(self.bins)

    def add(self, value):
        if value >= LG_SAT_CODE:
            self.saturated += 1
        if value < self.lo or value > self.hi:
            # Clamped into the end bin, not dropped. A clipped event is
            # still an event, and a counter that only ran when the
            # histogram threw data away is what hid saturation before:
            # the driver already clamps to 0..4095 and the old range
            # ended at 4095, so it could never fire at all.
            self.overflow += 1
            value = min(max(value, self.lo), self.hi)
        idx = int((value - self.lo) / self.width)
        if idx >= self.bins:
            idx = self.bins - 1
        self.counts[idx] += 1
        self.total += 1

    def peak_bin(self):
        if not self.total:
            return None
        m = max(self.counts)
        return self.counts.index(m)

    def centre_of(self, idx):
        return self.lo + (idx + 0.5) * self.width


# ===================================================================
#  Histogram canvas - drawn by hand so matplotlib is not required
# ===================================================================
class HistCanvas(tk.Canvas):
    PAD_L, PAD_R, PAD_T, PAD_B = 58, 14, 14, 38

    def __init__(self, master, **kw):
        super().__init__(master, bg=C_PANEL, highlightthickness=1,
                         highlightbackground=C_GRID, **kw)
        self.hist = None
        self.log = False
        self.bind("<Configure>", lambda e: self.redraw())
        self.bind("<Motion>", self._hover)
        self._tip = None

    def set_hist(self, hist):
        self.hist = hist
        self.redraw()

    def set_log(self, on):
        self.log = bool(on)
        self.redraw()

    # -- geometry helpers
    def _plot_box(self):
        w = self.winfo_width()
        h = self.winfo_height()
        return (self.PAD_L, self.PAD_T, w - self.PAD_R, h - self.PAD_B)

    def clear_tip(self):
        """Drop the hover readout - it has no business in a saved PNG."""
        if self._tip:
            self.delete(self._tip)
            self._tip = None

    def _hover(self, event):
        if not self.hist or not self.hist.total:
            return
        x0, y0, x1, y1 = self._plot_box()
        if not (x0 <= event.x <= x1 and y0 <= event.y <= y1):
            self.clear_tip()
            return
        frac = (event.x - x0) / max(1, (x1 - x0))
        idx = min(self.hist.bins - 1, int(frac * self.hist.bins))
        adc = self.hist.centre_of(idx)
        self.clear_tip()
        self._tip = self.create_text(
            x1 - 6, y0 + 6, anchor="ne", fill=C_MUTED,
            text=f"ADC {adc:.0f}   counts {self.hist.counts[idx]}",
            font=("Segoe UI", 9))

    # -- drawing
    def redraw(self):
        self.delete("all")
        self._tip = None
        w, h = self.winfo_width(), self.winfo_height()
        if w < 60 or h < 60:
            return
        x0, y0, x1, y1 = self._plot_box()

        self.create_rectangle(x0, y0, x1, y1, outline=C_GRID, fill=C_PANEL)

        if not self.hist or self.hist.total == 0:
            self.create_text((x0 + x1) / 2, (y0 + y1) / 2, fill=C_MUTED,
                             text="no data yet", font=("Segoe UI", 11))
            self._axis_labels(x0, y0, x1, y1, 0, 0, LG_CEIL_CODE)
            return

        hist = self.hist
        peak = max(hist.counts) or 1

        def to_y(c):
            if self.log:
                import math
                num = math.log10(c + 1)
                den = math.log10(peak + 1) or 1
                frac = num / den
            else:
                frac = c / peak
            return y1 - frac * (y1 - y0)

        # horizontal grid
        for i in range(1, 5):
            gy = y0 + (y1 - y0) * i / 5.0
            self.create_line(x0, gy, x1, gy, fill=C_GRID)

        bw = (x1 - x0) / float(hist.bins)
        # Draw as a filled outline when bins are sub-pixel, bars otherwise.
        if bw < 1.6:
            pts = []
            for i, c in enumerate(hist.counts):
                px = x0 + i * bw
                pts.extend([px, to_y(c)])
            if len(pts) >= 4:
                self.create_line(*pts, fill=C_BAR, width=1)
        else:
            for i, c in enumerate(hist.counts):
                if c == 0:
                    continue
                px = x0 + i * bw
                self.create_rectangle(px, to_y(c), px + max(1, bw - 1), y1,
                                      fill=C_BAR, outline="")

        # peak marker
        pb = hist.peak_bin()
        if pb is not None and hist.counts[pb] > 0:
            px = x0 + (pb + 0.5) * bw
            self.create_line(px, y0, px, y1, fill=C_BAD, dash=(3, 3))
            self.create_text(min(px + 4, x1 - 4), y0 + 4, anchor="nw",
                             fill=C_BAD, font=("Segoe UI", 9),
                             text=f"peak {hist.centre_of(pb):.0f}")

        self._axis_labels(x0, y0, x1, y1, peak, hist.lo, hist.hi)

    def _axis_labels(self, x0, y0, x1, y1, peak, lo, hi):
        self.create_text((x0 + x1) / 2, y1 + 24, text="ADC channel",
                         fill=C_MUTED, font=("Segoe UI", 9))
        self.create_text(14, (y0 + y1) / 2, text="counts", angle=90,
                         fill=C_MUTED, font=("Segoe UI", 9))
        for i in range(6):
            xv = lo + (hi - lo) * i / 5.0
            px = x0 + (x1 - x0) * i / 5.0
            self.create_text(px, y1 + 8, text=f"{xv:.0f}", anchor="n",
                             fill=C_MUTED, font=("Segoe UI", 8))
        for i in range(6):
            cv = peak * (5 - i) / 5.0
            py = y0 + (y1 - y0) * i / 5.0
            self.create_text(x0 - 6, py, text=f"{cv:.0f}", anchor="e",
                             fill=C_MUTED, font=("Segoe UI", 8))


# ===================================================================
#  Command confirmation
# ===================================================================
class CmdWatch:
    """Did the commands actually run, or did the reply just go missing?

    The "ok" a command answers with is bare text sharing the wire with
    the binary frames, and text has no framing to protect it. So a lost
    reply and a command that never ran look identical from here, which
    is exactly the failure that lets a setting silently not apply.

    The board therefore also counts, inside the status frame, how many
    commands it has completed and how many of those failed. That frame
    is CRC protected and resent every second, so the question worth
    asking is not "did a reply come back" but "has the completed count
    moved by as many as were sent". This watches that.
    """

    # The board sends status once a second and a single 'push' blocks it
    # for 0.4 s, so an honest verdict can take two beats to arrive.
    TIMEOUT_S = 4.0

    def __init__(self):
        self.reset()

    def reset(self):
        self.done = None        # counters as the board last reported them
        self.failed = None
        self.reports = None     # does this firmware carry them at all?
        self._job = None        # (label, count, done0, failed0, sent_at)

    # -- inputs
    def expect(self, label, count):
        """Note that `count` commands were just sent under `label`.

        The baseline is whatever the last status frame carried. It
        cannot have moved between the send and this call: frames are
        drained in pump(), which runs on the same Tk thread as the
        button handler that sent them.
        """
        self._job = (label, count, self.done, self.failed, time.monotonic())

    def on_frame(self, frame):
        """Take the counters from a status frame. May return a verdict."""
        if frame.get("type") != "status":
            return None
        done = frame.get("cmd_done")
        if done is None:
            self.reports = False
        else:
            self.reports = True
            self.done = done
            self.failed = frame.get("cmd_failed", 0)
        return self.poll()

    # -- output
    def poll(self):
        """The verdict, once there is one. (ok, message) or None.

        Also call this on a timer: a batch that is never confirmed is
        the whole point, and nothing arrives to trigger it.
        """
        if self._job is None:
            return None
        label, count, done0, failed0, sent_at = self._job

        if self.reports is False:
            self._job = None
            return (None, "%s: this firmware does not report command "
                          "results - the console is the only confirmation"
                    % label)

        if done0 is not None and self.done is not None:
            # Both counters wrap at 256, so only differences mean anything.
            ran = (self.done - done0) & 0xFF
            bad = (self.failed - failed0) & 0xFF
            if ran >= count:
                self._job = None
                if bad:
                    return (False, "%s: the board ran all %d, and %d "
                                   "FAILED on the ASIC" % (label, count, bad))
                return (True, "%s: all %d confirmed by the board"
                        % (label, count))

        if time.monotonic() - sent_at > self.TIMEOUT_S:
            self._job = None
            if done0 is None:
                return (False, "%s: NOT confirmed - no status frame to "
                               "compare against. Do not trust the settings"
                        % label)
            ran = (self.done - done0) & 0xFF if self.done is not None else 0
            return (False, "%s: NOT confirmed - the board completed %d of "
                           "%d. Check the link before trusting the settings"
                    % (label, ran, count))
        return None


# ===================================================================
#  Link indicator
# ===================================================================
class HealthLight(ttk.Frame):
    """A lamp and a sentence: is the link carrying data right now?

    "Connected" used to mean the port opened, which is the one thing
    that is true even when the board is unplugged mid-run, running the
    wrong baud rate, or sitting in a reset loop. This watches the
    once-a-second heartbeat instead.
    """

    DOT = 12
    COLOURS = {
        LinkHealth.OK: (C_OK, "link ok"),
        LinkHealth.UNSTABLE: (C_WARN, "link unstable"),
        LinkHealth.DEAD: (C_BAD, "no data"),
        LinkHealth.OFFLINE: (C_MUTED, "offline"),
    }

    def __init__(self, master, detail=True, **kw):
        super().__init__(master, **kw)
        self.lamp = tk.Canvas(self, width=self.DOT + 2, height=self.DOT + 2,
                              highlightthickness=0, bg=C_BG)
        self.lamp.pack(side="left")
        self._dot = self.lamp.create_oval(1, 1, self.DOT, self.DOT,
                                          fill=C_MUTED, outline="")
        self.lbl = ttk.Label(self, text="offline", style="Bad.TLabel")
        self.lbl.pack(side="left", padx=(6, 0))
        self.detail = ttk.Label(self, text="", style="Sub.TLabel") \
            if detail else None
        if self.detail:
            self.detail.pack(side="left", padx=(10, 0))
        self._level = None

    def update_health(self, level, why):
        colour, text = self.COLOURS.get(level, (C_MUTED, level))
        if level != self._level:
            self._level = level
            self.lamp.itemconfigure(self._dot, fill=colour)
            style = {LinkHealth.OK: "Ok.TLabel",
                     LinkHealth.UNSTABLE: "Warn.TLabel"}.get(level, "Bad.TLabel")
            self.lbl.configure(text=text, style=style)
        if self.detail:
            self.detail.configure(text=why)


# ===================================================================
#  Measurement records
# ===================================================================
def ensure_store():
    os.makedirs(STORE_DIR, exist_ok=True)


def save_measurement(rec):
    ensure_store()
    name = f"meas_{rec['started'].replace(':', '').replace('-', '').replace(' ', '_')}.json"
    with open(os.path.join(STORE_DIR, name), "w", encoding="utf-8") as fh:
        json.dump(rec, fh)
    prune_measurements()


def load_measurements():
    ensure_store()
    out = []
    for fn in os.listdir(STORE_DIR):
        if not fn.endswith(".json"):
            continue
        try:
            with open(os.path.join(STORE_DIR, fn), encoding="utf-8") as fh:
                rec = json.load(fh)
            rec["_file"] = os.path.join(STORE_DIR, fn)
            out.append(rec)
        except Exception:
            continue
    out.sort(key=lambda r: r.get("started", ""), reverse=True)
    return out[:MAX_STORED]


def prune_measurements():
    """Keep only the newest MAX_STORED records on disk."""
    ensure_store()
    files = [(os.path.getmtime(os.path.join(STORE_DIR, f)), f)
             for f in os.listdir(STORE_DIR) if f.endswith(".json")]
    files.sort(reverse=True)
    for _, fn in files[MAX_STORED:]:
        try:
            os.remove(os.path.join(STORE_DIR, fn))
        except OSError:
            pass


# The order these come out in, and the words next to them. A saved run
# is read by someone who no longer remembers what patGain was for, so
# the units travel with the number.
SETTING_UNITS = {
    "isotope": "what faced the crystal",
    "oven_setpoint_c": "chamber setpoint, C",
    "hv_supply_set_v": "supply setpoint, V",
    "hv_measured_v": "at the supply output, V",
    "v_channel_pins_v": "socket, sensor unplugged, V",
    "operator_note": "",
    "inDac": "0-255, SiPM bias trim",
    "lgGain": "0-15, x0.5-8",
    "hgGain": "0-15, trigger only",
    "tauLG": "0-15 shaping steps",
    "tauHG": "0-15 shaping steps",
    "slowLG": "1 = 120 ns steps",
    "slowHG": "1 = 120 ns steps",
    "patGain": "0-63, trigger gain",
    "trimT1": "0-63, below DAC1",
    "trimT2": "0-63, below DAC2",
    "ctest": "charge injection",
    "lg_path_on": "LG readout powered",
    "discri_on": "may fire the trigger",
    "dac1": "0-1023, T1 threshold",
    "dac2": "0-1023, T2 threshold",
    "dacq": "0-1023, charge thr.",
    "hold_delay": "0-255, x0.85 ns x slope",
    "slope_trim": "0-15, x hold delay",
    "selTrig": "trigger source",
    "hold_external": "1 = HOLDEXT pin",
    "amux_lg": "0 = ADC reads nothing",
    "abuffer": "output buffer supply",
    "hold_ns": "readout wait, ns",
    "dropped": "frames lost by board",
    "pending": "frames queued",
    "rx_overruns": "cmd bytes lost",
    "format": "0 = binary, 1 = text",
}

# Which form field answers which register, for the disagreement check.
_FORM_VS_CHANNEL = ("inDac", "lgGain", "tauLG", "slowLG", "patGain",
                    "trimT1", "trimT2")
_FORM_VS_GLOBAL = ("dac1", "dac2", "dacq", "hold_delay", "slope_trim",
                   "selTrig", "hold_external", "amux_lg")


def _rows(d, keys=None):
    """(key, value, note) rows for a settings dict, units attached."""
    items = [(k, d[k]) for k in keys] if keys else list(d.items())
    out = []
    for k, v in items:
        if isinstance(v, bool):
            v = int(v)
        out.append((k, v, SETTING_UNITS.get(k, "")))
    return out


def settings_sections(rec):
    """The saved settings of one record, grouped and ready to print.

    Returns [(title, [(key, value, note), ...]), ...]. Records written
    before the settings were captured simply come back with fewer
    sections, so old files still open.
    """
    s = rec.get("settings")
    out = []

    # First, because it is the half of the record nothing else can
    # reconstruct. Blank fields are dropped rather than printed: an
    # empty box is not a fact, and a printed empty box is how a
    # question nobody answered turns into a zero.
    cond = {k: v for k, v in (rec.get("conditions") or {}).items() if v != ""}
    if cond:
        out.append(("Run conditions, as typed", _rows(cond)))

    if not isinstance(s, dict):
        # Pre-snapshot record: everything that is known is at top level.
        out.append(("Acquisition", _rows({
            "channel": rec.get("channel", ""),
            "source": rec.get("source", ""),
            "bins": rec.get("bins", ""),
            "lo": rec.get("lo", ""),
            "hi": rec.get("hi", ""),
        })))
        if s:
            out.append(("Settings", [("settings", s, "")]))
        return out

    acq = s.get("acquisition") or {}
    if acq:
        out.append(("Acquisition", _rows(acq)))

    dev = s.get("device") or {}
    form = s.get("form") or {}

    if "unavailable" in dev:
        out.append(("ASIC registers", [("read back", "no", dev["unavailable"])]))
    else:
        chs = dev.get("channel_settings") or {}
        if chs:
            title = "ASIC channel %s, read back from the board" % dev.get("channel")
            rows = _rows(chs, [k for k in ("inDac", "lgGain", "hgGain",
                                           "tauLG", "tauHG", "slowLG",
                                           "slowHG", "patGain", "trimT1",
                                           "trimT2", "ctest", "lg_path_on",
                                           "discri_on") if k in chs])
            if dev.get("channel_raw"):
                rows.append(("shadow bytes", dev["channel_raw"],
                             "subadd 0-7"))
            out.append((title, rows))
        glob = dev.get("global") or {}
        if glob:
            rows = _rows(glob, [k for k in ("dac1", "dac2", "dacq",
                                            "hold_delay", "slope_trim",
                                            "selTrig", "hold_external",
                                            "amux_lg", "abuffer")
                                if k in glob])
            if dev.get("global_raw"):
                rows.append(("shadow bytes", dev["global_raw"],
                             "dacs, delay, trig, power"))
            out.append(("ASIC global, read back from the board", rows))
        stat = dev.get("stat") or {}
        if stat:
            out.append(("Firmware status at the start of the run",
                        _rows(stat)))

    if form:
        out.append(("Settings page, as it stood", _rows(form)))
        # Where the two disagree, the form was edited and never applied.
        # That is exactly the mistake this snapshot exists to catch.
        diffs = []
        chs = dev.get("channel_settings") or {}
        glob = dev.get("global") or {}
        if str(form.get("ui_channel")) == str(dev.get("channel")):
            for k in _FORM_VS_CHANNEL:
                if k in chs and k in form and int(chs[k]) != int(form[k]):
                    diffs.append((k, "form %s, chip %s" % (form[k], chs[k]),
                                  "not applied"))
        for k in _FORM_VS_GLOBAL:
            if k in glob and k in form and int(glob[k]) != int(form[k]):
                diffs.append((k, "form %s, chip %s" % (form[k], glob[k]),
                              "not applied"))
        if diffs:
            out.append(("Form and chip disagree - the chip wins", diffs))

    link = rec.get("link") or {}
    if link:
        end = link.get("at_end") or {}
        rows = [("verdict", end.get("level", "?"), end.get("detail", "")),
                ("events received", link.get("events_in_run", ""), ""),
                ("status frames", end.get("status_frames", ""),
                 "one per second"),
                ("worst beat gap", end.get("worst_heartbeat_gap_s", ""),
                 "s by board clock"),
                ("frames failing CRC", end.get("bad_crc", ""),
                 "a corrupt wire")]
        board = end.get("board") or {}
        for k in ("triggers", "events_ok", "events_bad", "dropped"):
            if k in board:
                rows.append((k, board[k], "board counter"))
        out.append(("Link health", rows))

    return out


def link_verdict(rec):
    """How the link behaved during a saved run, in one word.

    Records written before the link was tracked say so, rather than
    claiming a clean bill of health they never had.
    """
    link = rec.get("link") or {}
    end = link.get("at_end") or {}
    return end.get("level", "-")


def settings_text(rec):
    """The settings sections as a block of text, for reading on screen."""
    lines = []
    for title, rows in settings_sections(rec):
        lines.append(title)
        lines.append("-" * len(title))
        for key, val, note in rows:
            lines.append("  %-18s %-10s %s" % (key, val, note))
        lines.append("")
    return "\n".join(lines) if lines else "no settings were saved with this run"


# The flat block at the top of an export. The typed conditions sit up
# here next to the label and not only down in their own section, so that
# six exports of a thermal sweep can be stacked and sorted on the rows
# that tell them apart.
_RUN_HEADER = ("started", "label", "isotope", "oven_setpoint_c",
               "hv_supply_set_v", "hv_measured_v", "v_channel_pins_v",
               "duration_s", "channel", "source", "bins", "lo", "hi",
               "total", "overflow", "saturated", "temp_c", "rate_cps",
               "raw_file", "note", "operator_note")


def _run_value(rec, key):
    """One header value, from the record or from its conditions block."""
    if key in rec:
        return rec[key]
    return (rec.get("conditions") or {}).get(key, "")


def export_measurement(rec, path):
    """Write one measurement to .xlsx, or CSV if openpyxl is missing."""
    counts = rec["counts"]
    lo, hi, bins = rec["lo"], rec["hi"], rec["bins"]
    width = (hi - lo) / float(bins)

    if not HAVE_XLSX or path.lower().endswith(".csv"):
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            for k in _RUN_HEADER:
                w.writerow([k, _run_value(rec, k)])
            for title, rows in settings_sections(rec):
                w.writerow([])
                w.writerow([title])
                for key, val, note in rows:
                    w.writerow([key, val, note])
            w.writerow([])
            w.writerow(["bin", "adc_low", "adc_high", "adc_centre", "counts"])
            for i, c in enumerate(counts):
                w.writerow([i, lo + i * width, lo + (i + 1) * width,
                            lo + (i + 0.5) * width, c])
        return

    wb = Workbook()
    ws = wb.active
    ws.title = "Spectrum"
    ws.append(["bin", "adc_low", "adc_high", "adc_centre", "counts"])
    for i, c in enumerate(counts):
        ws.append([i, round(lo + i * width, 2), round(lo + (i + 1) * width, 2),
                   round(lo + (i + 0.5) * width, 2), c])

    chart = LineChart()
    chart.title = f"Spectrum - ch{rec.get('channel')} OUT_AMUXLG"
    chart.y_axis.title = "counts"
    chart.x_axis.title = "ADC channel"
    chart.height, chart.width = 10, 22
    data = Reference(ws, min_col=5, min_row=1, max_row=len(counts) + 1)
    cats = Reference(ws, min_col=4, min_row=2, max_row=len(counts) + 1)
    chart.add_data(data, titles_from_data=True)
    chart.set_categories(cats)
    ws.add_chart(chart, "G2")

    meta = wb.create_sheet("Metadata")
    meta.append(["Run"])
    for k in _RUN_HEADER:
        v = _run_value(rec, k)
        # Numbers stay numbers here. The HV column of a six-point sweep
        # is the x axis of a plot, and a column of text is not.
        meta.append([k, v if isinstance(v, (int, float)) else str(v)])
    meta.append([])
    meta.append(["axis", "raw ADC counts on OUT_AMUXLG"])
    # One row per setting. A blob in a single cell is not something a
    # thermal sweep can be sorted or plotted against afterwards.
    for title, rows in settings_sections(rec):
        meta.append([])
        meta.append([title])
        for key, val, note in rows:
            meta.append([key, val if not isinstance(val, bool) else int(val),
                         note])
    meta.column_dimensions["A"].width = 24
    meta.column_dimensions["B"].width = 22
    meta.column_dimensions["C"].width = 48

    wb.save(path)


def safe_label(text):
    """A run label as a filename fragment. Shared by the raw log and the
    screenshots so one run's files sort together."""
    return "".join(c if (c.isalnum() or c in "._-") else "_"
                   for c in text.strip())


# ===================================================================
#  Screenshots
# ===================================================================
def shot_path(kind, label=""):
    """A fresh PNG path under screenshots/, named like the raw files."""
    os.makedirs(SHOT_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    lab = safe_label(label)
    return os.path.join(SHOT_DIR,
                        f"{kind}_{stamp}_{lab}.png" if lab
                        else f"{kind}_{stamp}.png")


def widget_box(widget):
    """Screen rectangle of one widget, in Tk's coordinates."""
    x, y = widget.winfo_rootx(), widget.winfo_rooty()
    return (x, y, x + widget.winfo_width(), y + widget.winfo_height())


class _RECT(ctypes.Structure):
    _fields_ = [("left", ctypes.c_long), ("top", ctypes.c_long),
                ("right", ctypes.c_long), ("bottom", ctypes.c_long)]


# DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE. The same one Pillow borrows
# before it grabs, so the numbers below come from the space its image
# is in.
_PER_MONITOR_DPI = -3


def _window_rect(root):
    """This window's rectangle, in whatever space the thread is in."""
    r = _RECT()
    ctypes.windll.user32.GetWindowRect(ctypes.c_void_p(root.winfo_id()),
                                       ctypes.byref(r))
    return (r.left, r.top, r.right, r.bottom)


def _dpi_aware(fn):
    """Run fn() with this thread temporarily per-monitor DPI aware.

    On Windows 8.1 and earlier the call does not exist, and fn() simply
    runs in the virtualised space. Nothing needs special handling: those
    Pillows grab virtualised pixels too, so the two spaces are already
    the same one and the ratio below comes out at 1.
    """
    try:
        set_ctx = ctypes.windll.user32.SetThreadDpiAwarenessContext
    except (AttributeError, OSError):
        return fn()
    set_ctx.argtypes = [ctypes.c_void_p]
    set_ctx.restype = ctypes.c_void_p
    prev = set_ctx(ctypes.c_void_p(_PER_MONITOR_DPI))
    if not prev:                            # the context was refused
        return fn()
    try:
        return fn()
    finally:
        set_ctx(ctypes.c_void_p(prev))


def screen_map(root, shot_size, whole_desktop=True):
    """A function mapping a Tk (x, y) to a pixel in a grab of the screen.

    Tk is not DPI aware, so Windows hands it virtualised coordinates,
    while Pillow grabs the framebuffer under per-monitor awareness and
    gets real ones. On a mixed-DPI desktop those two are not a uniform
    scaling of each other: a 200% laptop reports 1440x900 for a
    2880x1800 panel, while a 100% monitor beside it reports its true
    2560x1080 - so the desktop measures 5440x1080 as Tk sees it and
    5440x1800 as Pillow does. Dividing one by the other gives a factor
    of 1.0 across and 1.67 down, and a crop computed from that lands
    nowhere near the widget.

    What does work is calibrating on a rectangle that exists in both
    spaces - this window's - measured once as Tk has it and once with
    the thread borrowed into the awareness the grab happens under. The
    ratio is then exact, and it is the right one for whichever monitor
    the window is actually on.
    """
    def uniform():
        # One screen, one scale, and the grab itself reports it: a
        # Retina capture comes back at twice the size Tk claims.
        wide = root.winfo_screenwidth() or shot_size[0]
        k = shot_size[0] / float(wide)
        return lambda x, y: (round(x * k), round(y * k))

    if sys.platform != "win32" or not whole_desktop:
        return uniform()

    try:
        tk_x, tk_y = root.winfo_rootx(), root.winfo_rooty()
        tk_w = root.winfo_width()
        real, virt = _dpi_aware(
            lambda: (_window_rect(root),
                     (ctypes.windll.user32.GetSystemMetrics(76),
                      ctypes.windll.user32.GetSystemMetrics(77))))
    except Exception:
        return uniform()

    if tk_w < 1 or real[2] <= real[0]:
        return uniform()

    k = (real[2] - real[0]) / float(tk_w)
    return lambda x, y: (round((x - tk_x) * k + real[0] - virt[0]),
                         round((y - tk_y) * k + real[1] - virt[1]))


def _front(root, on):
    """Hold the window above everything else for the length of a grab.

    This is a screen grab, so it takes whatever is on the glass: a file
    manager left open over the plot ends up in the PNG instead of the
    plot. Raising first is the only reliable defence.
    """
    try:
        if on:
            root.lift()
        root.attributes("-topmost", bool(on))
        root.update()
        if on:
            time.sleep(0.15)                # let the desktop repaint
            root.update()
    except tk.TclError:                     # window manager said no
        pass


def grab_widgets(widgets, path, pad=0):
    """Save the rectangle covering `widgets` as a PNG, and return the path.

    `pad` adds breathing room around the widgets, never past the edge of
    the window - a margin of desktop wallpaper is not part of the plot.
    """
    if not HAVE_PIL:
        raise RuntimeError("Screenshots need Pillow.\n\n"
                           "    pip install pillow")
    root = widgets[0].winfo_toplevel()
    root.update_idletasks()                 # flush any pending redraw
    if not widgets[0].winfo_viewable():
        raise RuntimeError("The window is not on screen.")

    try:
        _front(root, True)
        # Measured after the raise: lifting a window can move it.
        boxes = [widget_box(w) for w in widgets]
        box = (min(b[0] for b in boxes), min(b[1] for b in boxes),
               max(b[2] for b in boxes), max(b[3] for b in boxes))
        if pad:
            win = widget_box(root)
            box = (max(box[0] - pad, win[0]), max(box[1] - pad, win[1]),
                   min(box[2] + pad, win[2]), min(box[3] + pad, win[3]))
        if box[2] - box[0] < 2 or box[3] - box[1] < 2:
            raise RuntimeError("Nothing to capture.")
        try:
            # all_screens: the whole desktop, so a window on the second
            # monitor is in the picture and not off the edge of it.
            shot = ImageGrab.grab(all_screens=True)
            to_px = screen_map(root, shot.size)
        except TypeError:
            # Pillow older than 6.2: no all_screens, and no DPI
            # awareness either, so its grab is in Tk's own coordinates
            # and the primary screen is all there is.
            shot = ImageGrab.grab()
            to_px = screen_map(root, shot.size, whole_desktop=False)
    finally:
        _front(root, False)                 # never leave it pinned on top

    # Tk coordinates -> pixels in that image. Cropping without the
    # conversion lands up and left of the widget on any scaled display.
    left, top = to_px(box[0], box[1])
    right, bottom = to_px(box[2], box[3])
    img = shot.crop((left, top, right, bottom))
    img.save(path, "PNG")
    return path


# ===================================================================
#  Pages
# ===================================================================
class Page(ttk.Frame):
    title = "Page"

    def __init__(self, master, app):
        super().__init__(master, padding=16)
        self.app = app
        self.build()

    def build(self):
        pass

    def on_show(self):
        pass


class PageMain(Page):
    title = "RADIOROC2 Spectrometer"

    def build(self):
        head = ttk.Frame(self)
        head.pack(fill="x", pady=(0, 6))
        ttk.Label(head, text="RADIOROC2 Gamma Spectrometer",
                  style="H1.TLabel").pack(anchor="w")
        ttk.Label(head, text="CsI(Tl) + SiPM + RADIOROC2 on NUCLEO-F722ZE",
                  style="Sub.TLabel").pack(anchor="w")

        ttk.Separator(self).pack(fill="x", pady=12)

        # connection bar
        conn = ttk.LabelFrame(self, text="Connection", padding=12)
        conn.pack(fill="x")
        ttk.Label(conn, text="Port").grid(row=0, column=0, sticky="w")
        self.cmb = ttk.Combobox(conn, width=22, state="readonly")
        self.cmb.grid(row=0, column=1, padx=8)
        ttk.Button(conn, text="Refresh", command=self.refresh_ports)\
            .grid(row=0, column=2, padx=4)
        ttk.Button(conn, text="Connect", style="Accent.TButton",
                   command=self.connect).grid(row=0, column=3, padx=4)
        ttk.Button(conn, text="Simulator", command=self.connect_sim)\
            .grid(row=0, column=4, padx=4)
        ttk.Button(conn, text="Disconnect", command=self.disconnect)\
            .grid(row=0, column=5, padx=4)
        self.lbl_state = ttk.Label(conn, text="offline", style="Bad.TLabel")
        self.lbl_state.grid(row=0, column=6, padx=12)

        # The row above says whether a port is open. This one says
        # whether anything is coming out of it, which is the question
        # that actually decides if a measurement is worth starting.
        row1 = ttk.Frame(conn)
        row1.grid(row=1, column=0, columnspan=7, sticky="w", pady=(10, 0))
        self.light = HealthLight(row1)
        self.light.pack(side="left")
        ttk.Button(row1, text="Test link", command=self.test_link)\
            .pack(side="left", padx=(18, 0))

        # navigation tiles
        nav = ttk.Frame(self)
        nav.pack(fill="both", expand=True, pady=16)
        for i in range(2):
            nav.columnconfigure(i, weight=1, uniform="tile")

        tiles = [
            ("Settings", "Set ASIC parameters: gains, shaping,\n"
                         "thresholds, per-channel enables", "settings"),
            ("Measure", "Run an acquisition and watch the\n"
                        "spectrum accumulate live", "measure"),
            ("History", "Saved runs, the settings they were\n"
                        "taken under, and export to Excel", "history"),
            ("Help", "How the system works and the\n"
                     "order to do things in", "help"),
        ]
        for i, (name, desc, key) in enumerate(tiles):
            self._tile(nav, name, desc, key).grid(
                row=i // 2, column=i % 2, sticky="nsew", padx=8, pady=8)
            nav.rowconfigure(i // 2, weight=1)

        self.refresh_ports()

    def _tile(self, parent, name, desc, key):
        f = ttk.Frame(parent, style="Tile.TFrame", padding=18)
        ttk.Label(f, text=name, style="H2.TLabel").pack(anchor="w")
        ttk.Label(f, text=desc, style="Sub.TLabel", justify="left")\
            .pack(anchor="w", pady=(4, 12))
        ttk.Button(f, text="Open", command=lambda: self.app.show(key))\
            .pack(anchor="w")
        return f

    def refresh_ports(self):
        ports = Link.ports()          # ST-Link first, see Link.ports()
        self.cmb["values"] = ports
        # Prefer the ST-Link even if something else is already selected -
        # its COM number moves around between USB sockets, so a stale
        # selection is more likely wrong than deliberate.
        link = find_link_port()
        if link and link in ports:
            self.cmb.set(link)
        elif ports and not self.cmb.get():
            self.cmb.set(ports[0])

    def connect(self):
        port = self.cmb.get()
        if not port:
            messagebox.showwarning("No port", "Pick a serial port first.")
            return
        try:
            self.app.link.open(port)
        except Exception as exc:
            messagebox.showerror("Connection failed", str(exc))
            return
        self.app.set_state(True, port)

    def connect_sim(self):
        self.app.link.open_sim()
        self.app.set_state(True, "simulator")

    def disconnect(self):
        self.app.link.close()
        self.app.set_state(False, "")

    def test_link(self):
        """Ask the board a question and see whether it answers.

        The heartbeat proves the board is transmitting. This proves it is
        also listening - a half-working link, where commands go nowhere
        but frames keep arriving, otherwise looks perfectly healthy right
        up until the moment a setting silently fails to apply.
        """
        if not self.app.link.connected:
            messagebox.showwarning("Offline", "Connect to the board first.")
            return
        before = self.app.state.stamp
        self.app.link.send("stat")
        self.app.toast("test: asked the board for 'stat' ...")
        self.after(700, lambda: self._test_result(before))

    def _test_result(self, before):
        if self.app.state.stamp > before:
            hold = self.app.state.stat.get("hold_ns")
            self.app.toast("test: the board replied"
                           + (f" (hold {hold} ns)" if hold else ""))
        elif self.app.link.sim:
            self.app.toast("test: the simulator does not answer commands")
        else:
            self.app.toast("test: no reply - commands are not getting "
                           "through, even if frames are")

    def on_show(self):
        self.refresh_ports()
        self.app.refresh_state_labels()
        self.app.refresh_health()


class PageSettings(Page):
    title = "Settings"

    def build(self):
        top = ttk.Frame(self)
        top.pack(fill="x")
        ttk.Button(top, text="< Main", command=lambda: self.app.show("main"))\
            .pack(side="left")
        ttk.Label(top, text="ASIC Settings", style="H1.TLabel")\
            .pack(side="left", padx=12)

        body = ttk.Frame(self)
        body.pack(fill="both", expand=True, pady=12)
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)

        # ---- per channel
        ch = ttk.LabelFrame(body, text="Per channel", padding=12)
        ch.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        self.v_ch = tk.StringVar(value="0")
        r = 0
        ttk.Label(ch, text="Channel").grid(row=r, column=0, sticky="w", pady=3)
        cmb = ttk.Combobox(ch, textvariable=self.v_ch, width=8, state="readonly",
                           values=["all"] + [str(i) for i in range(64)])
        cmb.grid(row=r, column=1, sticky="w", pady=3)

        # Mid-scale, matching what the firmware pins at boot. The sweep
        # moves the HV supply; this DAC stays where it can still trim
        # in either direction afterwards.
        self.v_indac = self._spin(ch, "inDac (0-255)", 0, 255, 128, r := r + 1)
        self.v_glg = self._spin(ch, "lgGain (0-15)", 0, 15, 4, r := r + 1)
        self.v_tlg = self._spin(ch, "tauLG (0-15)", 0, 15, 14, r := r + 1)

        r += 1
        self.v_slg = tk.IntVar(value=1)
        ttk.Checkbutton(ch, text="slow shaping LG", variable=self.v_slg)\
            .grid(row=r, column=0, columnspan=2, sticky="w", pady=3)

        self.v_pat = self._spin(ch, "patGain (0-63)", 0, 63, 32, r := r + 1)
        self.v_t1 = self._spin(ch, "trim T1 (0-63)", 0, 63, 0, r := r + 1)
        self.v_t2 = self._spin(ch, "trim T2 (0-63)", 0, 63, 0, r := r + 1)

        r += 1
        bar = ttk.Frame(ch)
        bar.grid(row=r, column=0, columnspan=2, sticky="w", pady=(10, 0))
        ttk.Button(bar, text="Apply channel", style="Accent.TButton",
                   command=self.apply_channel).pack(side="left")
        ttk.Button(bar, text="Enable", command=lambda: self.enable(True))\
            .pack(side="left", padx=4)
        ttk.Button(bar, text="Disable", command=lambda: self.enable(False))\
            .pack(side="left")
        ttk.Button(bar, text="Dump", command=self.dump).pack(side="left", padx=4)

        # ---- global
        gl = ttk.LabelFrame(body, text="Global", padding=12)
        gl.grid(row=0, column=1, sticky="nsew")

        r = 0
        self.v_d1 = self._spin(gl, "Threshold DAC1 low (0-1023)", 0, 1023, 300, r)
        self.v_d2 = self._spin(gl, "Threshold DAC2 high (0-1023)", 0, 1023, 500, r := r + 1)
        self.v_dq = self._spin(gl, "Threshold DACQ charge (0-1023)", 0, 1023, 200, r := r + 1)
        self.v_dly = self._spin(gl, "Hold delay (0-255)", 0, 255, 255, r := r + 1)
        self.v_slp = self._spin(gl, "Slope trim (0-15)", 0, 15, 15, r := r + 1)
        self.v_trig = self._spin(gl, "selTrig (0-15)", 0, 15, 4, r := r + 1)

        r += 1
        self.v_hold_ext = tk.IntVar(value=0)
        ttk.Checkbutton(gl, text="external hold (HOLDEXT pin)",
                        variable=self.v_hold_ext)\
            .grid(row=r, column=0, columnspan=2, sticky="w", pady=3)

        r += 1
        self.v_mlg = tk.IntVar(value=1)
        ttk.Checkbutton(gl, text="AMUX LG buffer (the readout path)",
                        variable=self.v_mlg)\
            .grid(row=r, column=0, columnspan=2, sticky="w", pady=3)

        r += 1
        gbar = ttk.Frame(gl)
        gbar.grid(row=r, column=0, columnspan=2, sticky="w", pady=(10, 0))
        ttk.Button(gbar, text="Apply global", style="Accent.TButton",
                   command=self.apply_global).pack(side="left")
        ttk.Button(gbar, text="Preset CsI", command=self.preset_csi)\
            .pack(side="left", padx=4)
        ttk.Button(gbar, text="Defaults", command=lambda: self.cmd("defaults"))\
            .pack(side="left")
        ttk.Button(gbar, text="Push all",
                   command=lambda: self.run("Push all", ["push"]))\
            .pack(side="left", padx=4)

        # ---- what the board said about the last batch
        # Above the console rather than in it: a scrolling log is where
        # a failed Apply goes unnoticed, and this is the one line that
        # must not.
        self.lbl_result = ttk.Label(self, text="", style="Sub.TLabel",
                                    anchor="w")
        self.lbl_result.pack(fill="x", pady=(8, 0))

        # ---- console
        con = ttk.LabelFrame(self, text="Console", padding=8)
        con.pack(fill="both", expand=True)
        self.txt = tk.Text(con, height=8, bg="#0f1620", fg="#cfe3ff",
                           insertbackground="#cfe3ff", relief="flat",
                           font=("Consolas", 9))
        self.txt.pack(fill="both", expand=True, side="left")
        sb = ttk.Scrollbar(con, command=self.txt.yview)
        sb.pack(side="right", fill="y")
        self.txt.configure(yscrollcommand=sb.set)

        entry = ttk.Frame(self)
        entry.pack(fill="x", pady=(6, 0))
        self.v_raw = tk.StringVar()
        e = ttk.Entry(entry, textvariable=self.v_raw)
        e.pack(side="left", fill="x", expand=True)
        e.bind("<Return>", lambda _ev: self.send_raw())
        ttk.Button(entry, text="Send", command=self.send_raw)\
            .pack(side="left", padx=6)

    def _spin(self, parent, label, lo, hi, init, row):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=3)
        var = tk.IntVar(value=init)
        ttk.Spinbox(parent, from_=lo, to=hi, textvariable=var, width=8)\
            .grid(row=row, column=1, sticky="w", pady=3)
        return var

    # -- actions
    def log(self, s):
        self.txt.insert("end", s + "\n")
        self.txt.see("end")

    def cmd(self, line):
        """Send one command. True if it actually went out."""
        if not self.app.link.connected:
            messagebox.showwarning("Offline", "Connect to the board first.")
            return False
        self.app.link.send(line)
        self.log("> " + line)
        return True

    def run(self, label, lines):
        """Send a batch, then wait for the board to confirm it ran.

        Every button that changes the ASIC goes through here. Waiting on
        the replies would be the obvious thing and the wrong one: they
        are unframed text and can be lost, and a lost "ok" is not the
        same as a command that failed. The confirmation comes from the
        counters in the status frame instead - see CmdWatch.
        """
        sent = 0
        for line in lines:
            if not self.cmd(line):
                break
            sent += 1
        if not sent:
            return 0
        if self.app.link.sim:
            # The simulator produces frames, not answers. Waiting four
            # seconds to say so would only look like a fault.
            self.set_result(None, "%s: sent %d - the simulator does not "
                                  "execute commands" % (label, sent))
            return sent
        self.set_result(None, "%s: sent %d, waiting for the board ..."
                        % (label, sent))
        self.app.cmdwatch.expect(label, sent)
        return sent

    def set_result(self, ok, msg):
        """ok: True confirmed, False failed or unconfirmed, None pending."""
        style = {True: "Ok.TLabel", False: "Bad.TLabel"}.get(ok, "Sub.TLabel")
        self.lbl_result.configure(text=msg, style=style)

    def send_raw(self):
        line = self.v_raw.get().strip()
        if line:
            self.cmd(line)
            self.v_raw.set("")

    def apply_channel(self):
        c = self.v_ch.get()
        self.run(f"Apply channel {c}", [
            f"ch {c} indac {self.v_indac.get()}",
            # One argument each: the firmware leaves the HG half of
            # these registers untouched when it is omitted.
            f"ch {c} gain {self.v_glg.get()}",
            f"ch {c} slow {self.v_slg.get()}",
            f"ch {c} tau {self.v_tlg.get()}",
            f"ch {c} patgain {self.v_pat.get()}",
            f"ch {c} trim {self.v_t1.get()} {self.v_t2.get()}",
        ])

    def enable(self, on):
        word = "Enable" if on else "Disable"
        c = self.v_ch.get()
        self.run(f"{word} channel {c}",
                 [f"ch {c} {'on' if on else 'off'}"])

    def dump(self):
        c = self.v_ch.get()
        if c == "all":
            messagebox.showinfo("Dump", "Pick a single channel to dump.")
            return
        self.cmd(f"ch {c} dump")

    def apply_global(self):
        self.run("Apply global", [
            f"th {self.v_d1.get()} {self.v_d2.get()} {self.v_dq.get()}",
            f"delay {self.v_dly.get()} {self.v_slp.get()}",
            f"trig {self.v_trig.get()}",
            f"hold {'ext' if self.v_hold_ext.get() else 'int'}",
            f"mux {self.v_mlg.get()}",
        ])

    def form_values(self):
        """Everything on this page, as a dict.

        What the form holds is not necessarily what the ASIC holds - the
        page can be edited without ever pressing Apply, and Defaults
        resets the chip behind its back. So this is saved next to the
        board's own shadow, never instead of it.
        """
        return {
            "ui_channel": self.v_ch.get(),
            "inDac": self.v_indac.get(),
            "lgGain": self.v_glg.get(),
            "tauLG": self.v_tlg.get(),
            "slowLG": bool(self.v_slg.get()),
            "patGain": self.v_pat.get(),
            "trimT1": self.v_t1.get(),
            "trimT2": self.v_t2.get(),
            "dac1": self.v_d1.get(),
            "dac2": self.v_d2.get(),
            "dacq": self.v_dq.get(),
            "hold_delay": self.v_dly.get(),
            "slope_trim": self.v_slp.get(),
            "selTrig": self.v_trig.get(),
            "hold_external": bool(self.v_hold_ext.get()),
            "amux_lg": bool(self.v_mlg.get()),
        }

    def preset_csi(self):
        self.run("Preset CsI", ["preset csi"])
        # Mirror what the firmware preset does, so the form stays honest.
        self.v_slg.set(1)
        self.v_tlg.set(14)
        self.v_dly.set(255); self.v_slp.set(15)

    def on_show(self):
        while not self.app.link.text.empty():
            self.log(self.app.link.text.get())


class PageMeasure(Page):
    title = "Measure"

    def build(self):
        top = ttk.Frame(self)
        top.pack(fill="x")
        ttk.Button(top, text="< Main", command=lambda: self.app.show("main"))\
            .pack(side="left")
        ttk.Label(top, text="Acquisition", style="H1.TLabel")\
            .pack(side="left", padx=12)

        ctl = ttk.LabelFrame(self, text="Setup", padding=12)
        ctl.pack(fill="x", pady=10)

        ttk.Label(ctl, text="Channel").grid(row=0, column=0, sticky="w")
        self.v_ch = tk.IntVar(value=0)
        ttk.Spinbox(ctl, from_=0, to=63, textvariable=self.v_ch, width=6)\
            .grid(row=0, column=1, padx=(6, 18))

        ttk.Label(ctl, text="Bins").grid(row=0, column=2, sticky="w")
        self.v_bins = tk.IntVar(value=512)
        ttk.Combobox(ctl, textvariable=self.v_bins, width=7, state="readonly",
                     values=[128, 256, 512, 1024, 2048])\
            .grid(row=0, column=3, padx=(6, 18))

        ttk.Label(ctl, text="Duration (s, 0 = manual)")\
            .grid(row=0, column=4, sticky="w")
        self.v_dur = tk.IntVar(value=60)
        ttk.Spinbox(ctl, from_=0, to=36000, textvariable=self.v_dur, width=8)\
            .grid(row=0, column=5, padx=6)

        # There is one signal to histogram, so there is nothing to choose.
        ttk.Label(ctl, text="source: OUT_AMUXLG (low gain)",
                  style="Sub.TLabel")\
            .grid(row=0, column=6, sticky="w", padx=(18, 0))

        # Names the raw file. Type the temperature point and the DAC
        # setting here and the run identifies itself six months later.
        ttk.Label(ctl, text="Run label").grid(row=1, column=0, sticky="w",
                                              pady=(10, 0))
        self.v_label = tk.StringVar(value="")
        ttk.Entry(ctl, textvariable=self.v_label, width=28)\
            .grid(row=1, column=1, columnspan=3, sticky="w",
                  padx=(6, 18), pady=(10, 0))
        ttk.Label(ctl, text="e.g.  T30_HV27.14", style="Sub.TLabel")\
            .grid(row=1, column=4, columnspan=3, sticky="w", pady=(10, 0))

        # Facts about the bench that no wire carries. The board knows the
        # ASIC registers and its own temperature; it cannot know what the
        # HV supply was set to, what was sitting in front of the crystal,
        # or what the chamber was told to do. Left to a run label these
        # get typed differently every time and stop being sortable, so
        # they get fields of their own and travel in the record.
        cond = ttk.LabelFrame(self, text="Run conditions (typed, not measured by the board)",
                              padding=12)
        cond.pack(fill="x", pady=(0, 10))

        def entry(parent, text, var, row, col, width=12, hint=""):
            ttk.Label(parent, text=text).grid(row=row, column=col, sticky="w",
                                              pady=3)
            ttk.Entry(parent, textvariable=var, width=width)\
                .grid(row=row, column=col + 1, sticky="w", padx=(6, 18), pady=3)
            if hint:
                ttk.Label(parent, text=hint, style="Sub.TLabel")\
                    .grid(row=row, column=col + 2, sticky="w", pady=3)

        ttk.Label(cond, text="Isotope").grid(row=0, column=0, sticky="w", pady=3)
        self.v_isotope = tk.StringVar(value="")
        # Blank is the default on purpose. "no source" is a real
        # measurement - the background run the pedestal comes from - and
        # a run that never said what was in front of it is not that.
        ttk.Combobox(cond, textvariable=self.v_isotope, width=10,
                     state="readonly",
                     values=["", "Cs-137", "Co-60", "no source"])\
            .grid(row=0, column=1, sticky="w", padx=(6, 18), pady=3)
        ttk.Label(cond, text="blank = not stated", style="Sub.TLabel")\
            .grid(row=0, column=2, sticky="w", pady=3)

        self.v_oven = tk.StringVar(value="")
        entry(cond, "Oven setpoint (C)", self.v_oven, 0, 3, 10,
              "what the chamber was told, not what it reached")

        self.v_hv_set = tk.StringVar(value="")
        entry(cond, "HV supply set (V)", self.v_hv_set, 1, 0, 12)
        self.v_hv_meas = tk.StringVar(value="")
        entry(cond, "HV measured (V)", self.v_hv_meas, 1, 3, 12,
              "at the supply output")

        self.v_v_pins = tk.StringVar(value="")
        entry(cond, "V at channel pins (V)", self.v_v_pins, 2, 0, 12)
        ttk.Label(cond, text="across the socket with the sensor unplugged - "
                             "this is the overvoltage the SiPM will see",
                  style="Sub.TLabel")\
            .grid(row=2, column=3, columnspan=3, sticky="w", pady=3)

        ttk.Label(cond, text="Note").grid(row=3, column=0, sticky="w", pady=3)
        self.v_note = tk.StringVar(value="")
        ttk.Entry(cond, textvariable=self.v_note, width=72)\
            .grid(row=3, column=1, columnspan=5, sticky="w", padx=(6, 0),
                  pady=3)

        bar = ttk.Frame(self)
        bar.pack(fill="x")
        self.btn_start = ttk.Button(bar, text="Start", style="Accent.TButton",
                                    command=self.start)
        self.btn_start.pack(side="left")
        self.btn_stop = ttk.Button(bar, text="Stop", command=self.stop,
                                   state="disabled")
        self.btn_stop.pack(side="left", padx=6)
        ttk.Button(bar, text="Clear", command=self.clear).pack(side="left")
        self.v_log = tk.IntVar(value=0)
        ttk.Checkbutton(bar, text="log scale", variable=self.v_log,
                        command=lambda: self.canvas.set_log(self.v_log.get()))\
            .pack(side="left", padx=16)
        # The histogram is one channel. The raw file is every channel in
        # the readout window, which is what makes the dark neighbours -
        # and with them the pedestal - recoverable after the fact.
        self.v_raw = tk.IntVar(value=1)
        ttk.Checkbutton(bar, text="log raw events", variable=self.v_raw)\
            .pack(side="left")
        # Read this before pressing Start. A run that quietly stops
        # receiving looks exactly like a run with no source in front of
        # it, and only the heartbeat tells the two apart.
        self.light = HealthLight(bar)
        self.light.pack(side="left", padx=(24, 0))
        ttk.Button(bar, text="Save measurement", command=self.save)\
            .pack(side="right")
        ttk.Button(bar, text="Save plot PNG", command=self.shot_plot)\
            .pack(side="right", padx=6)

        # From the bottom, and before the plot. Setup and Run conditions
        # together ask for more height than a 740-pixel window has, and
        # the packer starves whatever comes last - which would be this
        # line, the one that says whether the silence is a dead link or
        # simply no events. The canvas expands, so it absorbs the
        # shortfall instead. Same reasoning as the History button bar.
        stat = ttk.Frame(self)
        stat.pack(fill="x", side="bottom")
        self.lbl = ttk.Label(stat, text="idle", style="Sub.TLabel")
        self.lbl.pack(side="left")

        self.canvas = HistCanvas(self, height=340)
        self.canvas.pack(fill="both", expand=True, pady=12)

        self.hist = Histogram()
        self.canvas.set_hist(self.hist)
        self.running = False
        self.t_start = None
        self.last_temp = 0.0
        self._last_ui = 0.0
        self.raw_fh = None
        self.raw_writer = None
        self.raw_path = ""
        self.raw_rows = 0
        # The board's own account of its settings, frozen at the moment
        # the run started - see _capture_settings().
        self.dev_snapshot = None
        self.snap_ch = None
        self.form_snapshot = None
        self.link_at_start = None
        self.events_at_start = 0

    # -- control
    def start(self):
        if not self.app.link.connected:
            messagebox.showwarning("Offline", "Connect to the board first.")
            return
        level, why = self.app.health.summary(True)
        if level in (LinkHealth.DEAD, LinkHealth.UNSTABLE):
            if not messagebox.askyesno(
                    "Link is not healthy",
                    f"The link reports: {level} - {why}\n\n"
                    "A run started now may record nothing, or may lose "
                    "events without saying so.\n\nStart anyway?"):
                return
        self.hist.reset(self.v_bins.get(), 0, LG_CEIL_CODE)
        self.canvas.set_hist(self.hist)
        self._open_raw()
        self.running = True
        self.t_start = time.time()
        self.dev_snapshot = None
        self.form_snapshot = self.app.pages["settings"].form_values()
        self.link_at_start = self.app.health.snapshot(True)
        self.events_at_start = self.app.health.event_count
        self._ask_settings()
        self.btn_start.configure(state="disabled")
        self.btn_stop.configure(state="normal")

    def stop(self):
        self.running = False
        self._close_raw()
        self.btn_start.configure(state="normal")
        self.btn_stop.configure(state="disabled")

    # -- settings snapshot
    def _ask_settings(self):
        """Ask the board for its shadow, so the run can be saved with it.

        Both replies are plain text and arrive a few milliseconds later,
        through the same queue as everything else; _capture_settings()
        picks them up. Asking at the start and freezing the answer is
        what keeps the record honest if the Settings page is edited
        halfway through a run.
        """
        # Remember which channel was asked about: the spinbox can be
        # moved mid-run, and the answer coming back is about this one.
        self.snap_ch = self.v_ch.get()
        self.app.state.ch_raw.pop(self.snap_ch, None)
        self.app.state.glob_raw = []
        self.app.link.send("stat")
        self.app.link.send(f"ch {self.snap_ch} dump")

    def _capture_settings(self):
        """Freeze the board's answer, once, and stop watching for it."""
        if self.dev_snapshot is None and self.snap_ch is not None                 and self.app.state.has_dump(self.snap_ch):
            self.dev_snapshot = self.app.state.snapshot(self.snap_ch)

    # -- raw event log
    def _open_raw(self):
        self._close_raw()
        self.raw_rows = 0
        if not self.v_raw.get():
            self.raw_path = ""
            return
        os.makedirs(RAW_DIR, exist_ok=True)
        label = safe_label(self.v_label.get())
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        name = f"raw_{stamp}_{label}.csv" if label else f"raw_{stamp}.csv"
        self.raw_path = os.path.join(RAW_DIR, name)
        try:
            self.raw_fh = open(self.raw_path, "w", newline="", encoding="utf-8")
        except OSError as exc:
            self.raw_path = ""
            messagebox.showwarning(
                "Raw log not started",
                f"Could not open the raw file:\n{exc}\n\n"
                "The histogram still runs, but the per-event data is not "
                "being kept.")
            return
        self.raw_writer = csv.writer(self.raw_fh)
        self.raw_writer.writerow(["seq", "t_ms", "temp_c", "ch", "lg"])

    def _close_raw(self):
        if self.raw_fh:
            try:
                self.raw_fh.close()
            except OSError:
                pass
        self.raw_fh = None
        self.raw_writer = None

    def clear(self):
        self.hist.reset()
        self.canvas.redraw()
        self.lbl.configure(text="cleared")

    # -- screenshot
    def shot_plot(self):
        """PNG of the spectrum plus the line of numbers under it - the
        counts, the rate and the temperature are what make the picture
        mean something in a logbook."""
        self.canvas.clear_tip()
        self.app.screenshot([self.canvas, self.lbl],
                            label=self.v_label.get(), kind="spectrum")

    def feed(self, frame):
        """Called by the app pump for every decoded frame."""
        if frame["type"] == "status":
            self.last_temp = frame.get("temp_c", 0.0)
            return
        if frame["type"] != "event" or not self.running:
            return

        # Raw first, and for the whole window: the channel the histogram
        # skips is exactly the one carrying the baseline.
        if self.raw_writer:
            first = frame["first_ch"]
            temp = f"{frame['temp_c']:.3f}"
            for i in range(frame["count"]):
                self.raw_writer.writerow([frame["seq"], frame["t_ms"], temp,
                                          first + i, frame["lg"][i]])
            self.raw_rows += frame["count"]

        ch = self.v_ch.get()
        idx = ch - frame["first_ch"]
        if idx < 0 or idx >= frame["count"]:
            return

        self.hist.add(frame["lg"][idx])

    def tick(self):
        if not self.running:
            return
        self._capture_settings()
        elapsed = time.time() - self.t_start
        dur = self.v_dur.get()
        if dur and elapsed >= dur:
            raw = os.path.basename(self.raw_path) if self.raw_path else ""
            self.stop()
            self.lbl.configure(text=f"finished: {self.hist.total} counts "
                                    f"in {elapsed:.1f} s"
                                    + (f" | raw: {raw}" if raw else ""))
            self.canvas.redraw()
            return
        now = time.time()
        if now - self._last_ui > 0.25:          # keep redraws cheap
            self._last_ui = now
            self.canvas.redraw()
            # Four flushes a second, so a crash in hour three of a sweep
            # costs a quarter second of events, not the whole point.
            if self.raw_fh:
                try:
                    self.raw_fh.flush()
                except OSError:
                    pass
            rate = self.hist.total / elapsed if elapsed > 0 else 0
            pk = self.hist.peak_bin()
            pk_txt = f"peak ADC {self.hist.centre_of(pk):.0f}" if pk is not None else "-"
            raw_txt = f" | raw {self.raw_rows} rows" if self.raw_writer else ""
            self.lbl.configure(
                text=f"running {elapsed:6.1f} s | {self.hist.total} counts | "
                     f"{rate:6.1f} cps | {pk_txt} | T {self.last_temp:.1f} C | "
                     f"saturated {self.hist.saturated}{raw_txt}"
                     + self._silence_note(elapsed))

    def _silence_note(self, elapsed):
        """Why nothing is arriving, when nothing is arriving.

        Two very different faults look identical on an empty histogram:
        a link that has stopped, and a threshold set above every pulse.
        The heartbeat separates them, so say which one it is.
        """
        level = self.app.health.level(self.app.link.connected)
        if level in (LinkHealth.DEAD, LinkHealth.OFFLINE):
            return "   <<  LINK DOWN - these seconds are not being recorded"
        age = self.app.health.event_age()
        quiet = (age is None and elapsed > 5.0) or (age is not None and age > 5.0)
        if quiet:
            return ("   <<  link ok, no events - threshold, channel enable, "
                    "or no source")
        if level == LinkHealth.UNSTABLE:
            return "   <<  link unstable - counts may be missing"
        return ""

    def _settings_block(self):
        """Everything the run was taken under, as one saveable dict."""
        ch = self.v_ch.get()
        device = self.dev_snapshot
        if device is None:
            device = {"unavailable": (
                "the simulator does not report registers"
                if self.app.link.sim else
                "the board did not answer 'ch %s dump' at the start of "
                "the run" % self.snap_ch)}
        return {
            "device": device,
            "form": self.form_snapshot or {},
            "acquisition": {
                "channel": ch,
                "source": "OUT_AMUXLG",
                "bins": self.hist.bins,
                "lo": self.hist.lo,
                "hi": self.hist.hi,
                "duration_set_s": self.v_dur.get(),
                "raw_logging": bool(self.v_raw.get()),
            },
        }

    def _conditions_block(self):
        """The typed bench conditions, numbers parsed where they parse.

        A field that does not parse is kept as the operator wrote it
        rather than dropped: "27.1, drifting" is worth more six months
        later than a blank, even though nothing can sort it.
        """
        def num(var):
            s = var.get().strip()
            if not s:
                return ""
            try:
                return float(s)
            except ValueError:
                return s

        return {
            "isotope": self.v_isotope.get().strip(),
            "oven_setpoint_c": num(self.v_oven),
            "hv_supply_set_v": num(self.v_hv_set),
            "hv_measured_v": num(self.v_hv_meas),
            "v_channel_pins_v": num(self.v_v_pins),
            "operator_note": self.v_note.get().strip(),
        }

    def save(self):
        if self.hist.total == 0:
            messagebox.showinfo("Nothing to save", "Acquire some data first.")
            return
        self._capture_settings()        # in case the reply landed late
        elapsed = (time.time() - self.t_start) if self.t_start else 0.0
        link_now = self.app.health.snapshot(self.app.link.connected)
        rec = {
            "started": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "duration_s": round(elapsed, 1),
            "channel": self.v_ch.get(),
            "source": "OUT_AMUXLG",
            "bins": self.hist.bins,
            "lo": self.hist.lo,
            "hi": self.hist.hi,
            "counts": self.hist.counts[:],
            "total": self.hist.total,
            "overflow": self.hist.overflow,
            "saturated": self.hist.saturated,
            "temp_c": round(self.last_temp, 2),
            "rate_cps": round(self.hist.total / elapsed, 2) if elapsed else 0,
            "label": self.v_label.get().strip(),
            "raw_file": os.path.basename(self.raw_path) if self.raw_path else "",
            "note": "simulator" if self.app.link.sim else "",
            "conditions": self._conditions_block(),
            "settings": self._settings_block(),
            # Saved as evidence, not decoration: a spectrum taken across
            # a link that dropped frames is a spectrum with a hole in it,
            # and six months later nothing else will remember.
            "link": {
                "at_start": self.link_at_start or {},
                "at_end": link_now,
                "events_in_run": max(0, self.app.health.event_count
                                     - self.events_at_start),
            },
        }
        save_measurement(rec)
        note = "Measurement stored in History."
        if link_now.get("level") != LinkHealth.OK:
            note += ("\n\nNote: the link was reported as "
                     f"{link_now.get('level')} - {link_now.get('detail')}")
        if isinstance(rec["settings"]["device"], dict) \
                and "unavailable" in rec["settings"]["device"]:
            note += ("\n\nThe ASIC registers could not be read back, so "
                     "only the values from the Settings form were saved.")
        messagebox.showinfo("Saved", note)
        self.app.pages["history"].reload()


class PageHistory(Page):
    title = "History"

    def build(self):
        top = ttk.Frame(self)
        top.pack(fill="x")
        ttk.Button(top, text="< Main", command=lambda: self.app.show("main"))\
            .pack(side="left")
        ttk.Label(top, text="Saved measurements", style="H1.TLabel")\
            .pack(side="left", padx=12)
        ttk.Button(top, text="Refresh", command=self.reload).pack(side="right")

        # isotope and the chamber setpoint sit next to the label because
        # they are what tells six runs of one sweep apart at a glance -
        # temp_c is what the board felt, which is not the same question.
        cols = ("started", "label", "isotope", "oven_c", "duration_s",
                "channel", "total", "rate_cps", "temp_c", "saturated", "link")
        self.tree = ttk.Treeview(self, columns=cols, show="headings", height=12)
        widths = (140, 130, 70, 65, 75, 60, 80, 80, 65, 70, 75)
        for c, w in zip(cols, widths):
            self.tree.heading(c, text=c.replace("_", " "))
            self.tree.column(c, width=w, anchor="center")
        self.tree.pack(fill="x", pady=12)
        self.tree.bind("<<TreeviewSelect>>", lambda _e: self.preview())

        # This bar is packed before the plot, and from the bottom. The
        # page asks for more height than the window has, and the packer
        # starves whatever comes last: these buttons, Export included.
        bar = ttk.Frame(self)
        bar.pack(fill="x", side="bottom", pady=10)
        ttk.Button(bar, text="Export to Excel", style="Accent.TButton",
                   command=self.export).pack(side="left")
        ttk.Button(bar, text="Save plot PNG", command=self.shot_plot)\
            .pack(side="left", padx=6)
        ttk.Button(bar, text="Delete", command=self.delete)\
            .pack(side="left")
        self.lbl = ttk.Label(bar, text="", style="Sub.TLabel")
        self.lbl.pack(side="left", padx=16)

        # Plot on the left, the numbers it was taken under on the right.
        # Saving the settings and then having nowhere to read them would
        # be half a feature.
        split = ttk.Frame(self)
        split.pack(fill="both", expand=True)
        self.canvas = HistCanvas(split, height=280)
        self.canvas.pack(side="left", fill="both", expand=True)

        det = ttk.LabelFrame(split, text="Settings for this run", padding=6)
        det.pack(side="right", fill="both", padx=(10, 0))
        self.det = tk.Text(det, width=54, height=12, wrap="word",
                           relief="flat", bg=C_PANEL, fg=C_TEXT,
                           font=("Consolas", 9))
        self.det.pack(side="left", fill="both", expand=True)
        dsb = ttk.Scrollbar(det, command=self.det.yview)
        dsb.pack(side="right", fill="y")
        self.det.configure(yscrollcommand=dsb.set, state="disabled")
        # A wrapped note that restarts at column 0 destroys the columns
        # it was wrapping out of. Indent the continuation to where the
        # note began, so a long row still reads as one row.
        f = tkfont.Font(font=self.det.cget("font"))
        self.det.tag_configure("row", lmargin2=f.measure(" " * 31))

        self.records = []
        self.reload()

    def show_settings(self, text):
        self.det.configure(state="normal")
        self.det.delete("1.0", "end")
        self.det.insert("1.0", text, "row")
        self.det.configure(state="disabled")

    def reload(self):
        self.records = load_measurements()
        for iid in self.tree.get_children():
            self.tree.delete(iid)
        for i, r in enumerate(self.records):
            self.tree.insert("", "end", iid=str(i), values=(
                r.get("started", ""), r.get("label", ""),
                _run_value(r, "isotope"), _run_value(r, "oven_setpoint_c"),
                r.get("duration_s", ""),
                r.get("channel", ""), r.get("total", ""),
                r.get("rate_cps", ""), r.get("temp_c", ""),
                r.get("saturated", ""), link_verdict(r)))
        self.canvas.set_hist(None)
        self.show_settings("select a measurement to see the settings it "
                           "was taken under")
        self.lbl.configure(
            text="" if HAVE_XLSX else "openpyxl not installed - export falls back to CSV")

    def selected(self):
        sel = self.tree.selection()
        if not sel:
            return None
        return self.records[int(sel[0])]

    def preview(self):
        rec = self.selected()
        if not rec:
            return
        h = Histogram(rec["bins"], rec["lo"], rec["hi"])
        h.counts = rec["counts"][:]
        h.total = rec.get("total", sum(h.counts))
        self.canvas.set_hist(h)
        self.show_settings(settings_text(rec))

    def shot_plot(self):
        rec = self.selected()
        if not rec:
            messagebox.showinfo("Pick one", "Select a measurement first.")
            return
        self.canvas.clear_tip()
        self.app.screenshot(self.canvas, label=rec.get("label", ""),
                            kind="spectrum")

    def export(self):
        rec = self.selected()
        if not rec:
            messagebox.showinfo("Pick one", "Select a measurement first.")
            return
        default = "spectrum_{}.xlsx".format(
            rec["started"].replace(":", "").replace("-", "").replace(" ", "_"))
        types = [("Excel workbook", "*.xlsx"), ("CSV", "*.csv")] if HAVE_XLSX \
            else [("CSV", "*.csv")]
        path = filedialog.asksaveasfilename(
            defaultextension=".xlsx" if HAVE_XLSX else ".csv",
            initialfile=default, filetypes=types)
        if not path:
            return
        try:
            export_measurement(rec, path)
        except Exception as exc:
            messagebox.showerror("Export failed", str(exc))
            return
        messagebox.showinfo("Exported", f"Written to\n{path}")

    def delete(self):
        rec = self.selected()
        if not rec:
            return
        if not messagebox.askyesno("Delete", "Remove this measurement?"):
            return
        try:
            os.remove(rec["_file"])
        except OSError:
            pass
        self.reload()

    def on_show(self):
        self.reload()


class PageHelp(Page):
    title = "Help"

    TEXT = """\
WHAT THIS IS
    A front end for a RADIOROC2 based gamma spectrometer: a CsI(Tl)
    crystal read out by a SiPM, digitised by an STM32F722 and streamed
    to this PC over the ST-Link virtual COM port.

ORDER OF OPERATIONS
    1. Connect
       Main page, pick the port, press Connect, and check that the lamp
       under it goes green. If the board is not built yet, press
       Simulator to explore the interface with generated data.

    2. Settings
       - Disable every channel first (Channel = all, then Disable).
         Unconnected channels still feed the NOR trigger and will fire
         on noise.
       - Enable only the channel your SiPM is wired to.
       - Press Preset CsI. This is important: the ASIC defaults to
         20 ns shaping steps, while CsI(Tl) light decays over about
         1 microsecond. Without slow shaping the charge is never fully
         collected and the energy resolution collapses.
       - Set the thresholds. Too low and the system free-runs on noise,
         too high and you lose the low energy part of the spectrum.
       - Press Apply. Nothing on this page reaches the chip until you
         do, and the saved record will say so if you forget.

    3. Measure
       Choose the channel, set the bin count and a duration, then
       Start. The spectrum builds up live. Save Measurement when it
       looks reasonable.

    4. History
       Select a run to preview it and read the settings it was taken
       under, then Export to Excel.

WHAT EVERY SETTING DOES - PER CHANNEL
    These are written per channel, or to all 64 at once with
    Channel = all. Only the channel your SiPM is on matters for the
    spectrum; the rest matter because they can trigger.

    Channel
        Which of the 64 inputs the other fields on this side of the
        page are written to. It is not the channel that gets
        histogrammed - that one is chosen on the Measure page.

    inDac  (0-255, default 128)
        An 8-bit DAC per channel that trims the SiPM bias, and with it
        the SiPM gain. It is the fine trim, not the coarse knob: its
        whole span is worth roughly a couple of volts, which is about
        28 C of thermal drift compensation. Anything larger has to come
        from the HV supply.
        Changing it moves the whole spectrum along the ADC axis,
        photopeak and Compton edge together, because it changes how
        much charge each gamma produces before the ASIC ever sees it.
        It also moves the dark count rate, steeply.
        Confirm the direction once on your own board: step it by 32 and
        watch which way the peak goes. The DAC sits between the bias
        rail and the pixel, so a larger code usually means less
        overvoltage and a smaller pulse - but do not take that on
        trust.

    lgGain  (0-15, default 4)
        Gain of the low-gain charge preamp, spanning about x0.5 to x8
        across the sixteen steps. This is the pulse height knob,
        because OUT_AMUXLG is the only signal on this board that
        reaches an ADC.
        Raise it and the spectrum stretches towards higher ADC
        channels. Too far and the photopeak piles up against 1638: the
        peak stops moving, grows a hard edge, and the saturated counter
        on the Measure page starts climbing. That ceiling belongs to the
        ASIC, not to the ADC - OUT_AMUXLG cannot output more than
        1.32 V, which is 1638 of the 4095 codes the ADC could digitise,
        and nothing amplifies the line in between.
        Lower it and the spectrum compresses towards zero. Too far and
        the whole thing lives in the first few bins, where the
        resolution is limited by how coarsely it is being digitised.
        Aim to put the photopeak at roughly two thirds of the 1638
        ceiling - about 1100 codes - which leaves headroom for a higher
        energy line.

    tauLG  (0-15, default 14 under Preset CsI)
        Peaking time of the low-gain shaper, as an index. The step is
        20 ns normally and 120 ns with slow shaping on, so the index
        covers 20-300 ns fast, or 120 ns to about 1.8 us slow.
        Raise it and the shaper integrates for longer: more of the
        scintillation light is collected, the pulse is taller and the
        energy resolution improves - up to the point where it is
        collecting noise as well, or where pulses start piling up on
        each other at high rate.
        Lower it and less charge is collected, so the whole spectrum
        shrinks. For CsI(Tl), whose light decays over about a
        microsecond, a short peaking time throws most of the signal
        away.
        Changing this moves the shaper peak in time, so the hold delay
        has to be re-tuned afterwards. They are not independent.

    slow shaping LG  (default on under Preset CsI)
        Switches the tauLG step from 20 ns to 120 ns. Off suits fast
        scintillators such as LYSO or plastic. On is required for
        CsI(Tl).
        Turning it off while leaving tauLG at 14 drops the peaking time
        from about 1.8 us to 300 ns, and with it most of the pulse
        height. If the photopeak collapses for no apparent reason, this
        is the first thing to check.

    patGain  (0-63, default 32)
        Gain of the time preamp, closed loop 15 to 100. It feeds the
        discriminators only - it is not in the digitised path.
        Raise it and the trigger sees a larger pulse, so the same DAC1
        threshold now catches lower energy events and the trigger rate
        goes up. Lower it and the effective threshold rises.
        It does not move the photopeak. It changes which events are
        recorded, not how tall they are once recorded.

    trim T1 / trim T2  (0-63 each, default 0)
        Per-channel threshold trims: each step takes the channel's
        trigger point a fraction of a millivolt below the global
        threshold, 0-15 mV over the full range.
        Raise one and that channel triggers more readily. They exist to
        equalise trigger points across an array; on a single-channel
        setup leave them at 0 and use DAC1.

    Enable / Disable
        Powers a channel completely: preamps, discriminators, shapers
        and peak detectors together.
        Disable every channel you have not wired something to. An
        unconnected input floats, its discriminator fires on noise, and
        because the trigger is a NOR across all channels the whole
        system then free-runs. This is the usual cause of a count rate
        in the tens of thousands per second.

WHAT EVERY SETTING DOES - GLOBAL
    One copy of each of these, shared by all 64 channels.

    Threshold DAC1 low  (0-1023, default 300)
        The low time-trigger threshold, and with the default selTrig
        the one that actually decides what is recorded.
        Raise it and fewer events pass: noise is excluded first, then
        the low energy end of the spectrum, which starts to show as a
        spectrum that begins abruptly part way up the ADC axis instead
        of rising from zero.
        Lower it and more of the spectrum appears, until the threshold
        reaches the noise floor and the rate explodes - thousands of
        counts per second piled into the lowest bins.
        Set it a little above the point where lowering it further stops
        adding spectrum and starts adding rate.

    Threshold DAC2 high  (0-1023, default 500)
        The high time-trigger threshold. A second discriminator on the
        same signal, used for time-over-threshold style measurements.
        It only affects the acquisition if selTrig is set to route it.
        Normally leave it above DAC1 and ignore it.

    Threshold DACQ charge  (0-1023, default 200)
        Threshold of the charge discriminator, which runs off the high
        gain chain. HG is not digitised on this board, but its
        discriminator still works, so this matters if selTrig selects
        the charge trigger. Otherwise it does nothing.

    Hold delay  (0-255) and Slope trim  (0-15)
        Together they set how long after the trigger the peak detector
        is sampled: delay x 0.85 ns x slope trim. Slope trim multiplies,
        so at 0 the delay collapses to the firmware's minimum and at 15
        it is longest.
        This is the most important knob after gain, and the least
        obvious. Sample too early and the shaper has not reached its
        peak; too late and it has already decayed. Either way every
        pulse is measured too small and the photopeak sits low, broad
        and in the wrong place - which looks exactly like a gain
        problem.
        Tune it by sweeping the delay across its range and keeping the
        value that pushes the photopeak highest. Re-tune it whenever
        tauLG or slow shaping changes.
        The firmware pads the nominal figure for the spread in the
        delay cell. Type 'stat' in the console to see hold_ns: what the
        readout will really wait for. After Preset CsI it must grow,
        not stay put.

    selTrig  (0-15, default 4)
        Which signal starts an acquisition. The low two bits pick the
        source: 0 = the global trigger, 1 = T1, 2 = T2, 3 = charge.
        When they are 0, bits 3:2 pick which global trigger: 0 =
        external TRIGEXT, 1 = T1, 2 = T2, 3 = charge.
        The default, 4, means "global trigger fed by T1" - the DAC1
        discriminator of any enabled channel. Change this and you
        change which threshold has any effect at all, which is a fast
        way to make DAC1 look broken.

    external hold (HOLDEXT pin)
        Off, the internal delay cell times the hold from the trigger,
        which is what Hold delay above controls. On, an external pin
        does instead. Nothing on this board drives that pin, so turning
        it on stops acquisitions.

    AMUX LG buffer
        Powers the buffer that drives OUT_AMUXLG - the one analog pin
        the ADC is wired to. Off, the ADC reads the same flat value for
        every event no matter what the detector does.
        It should always be on. It has a control here only so that
        "every event reads the same number" has somewhere obvious to be
        checked.

WHAT EVERY SETTING DOES - MEASURE PAGE
    Channel
        Which channel's ADC code goes into the histogram. The raw CSV
        keeps every channel in the readout window regardless, which is
        what makes the dark neighbours - and with them the pedestal -
        recoverable afterwards.

    Bins  (128-2048)
        How finely the reachable 0-1638 range is divided. 512 bins is
        a little over 3 ADC codes per bin.
        More bins resolve a narrow peak better but need proportionally
        more counts before the shape stops being noise. Fewer bins give
        a smooth curve quickly, at the cost of knowing exactly where
        the peak centre is. Changing this clears the histogram.

    Duration  (s, 0 = manual)
        How long Start runs for. 0 runs until Stop is pressed. The
        counting statistics in each bin improve as the square root of
        the time: four times as long is twice as clean.

    Run label
        Names the raw CSV and the screenshots, and shows up in History.
        Put the temperature point and the HV in it. It is what tells six
        runs of a sweep apart afterwards.

    Run conditions  (isotope, oven setpoint, three HV fields, note)
        Facts about the bench that no wire carries. The board reports
        its own temperature and can be asked what the ASIC registers
        hold. It has no idea what the HV supply was set to, what was in
        front of the crystal, or what the chamber was told to do.

        Isotope is blank until you say otherwise. "no source" is a real
        measurement - the background run a pedestal comes from - and a
        run that never said what it was pointed at is not that run.

        Oven setpoint is what the chamber was told. temp_c, beside it in
        History, is what the board felt. The gap between them is the
        thermal gradient, which is the reason to keep both.

        HV supply set and HV measured are the supply's own two numbers.
        V at channel pins is the third and the most useful: measured
        across the socket with the sensor unplugged, it is HV minus
        whatever the input DAC holds the input at, and that difference
        is the overvoltage the SiPM will actually see.

        All six travel with the run: History, the settings pane, and the
        Metadata sheet of an Excel export - numbers stored as numbers,
        so a sweep can be plotted against them. They are deliberately
        not remembered between sessions. A stale HV quietly attaching
        itself to tomorrow's run is worse than an empty field.

    log scale
        Vertical axis only. Makes the Compton continuum and any small
        peaks visible next to a tall photopeak. It changes nothing
        about what is recorded.

    log raw events
        Writes every event, every channel, to a CSV under raw/.
        Analysis that needs a pedestal needs this file. It costs disk,
        not counts.

WHICH SIGNAL IS MEASURED
    Only OUT_AMUXLG is wired to an ADC, so every spectrum here is the
    low gain path: one ADC code per channel per event, and the x axis is
    raw ADC counts. There is no gain choice to make and nothing to
    cross-calibrate.

    The ASIC still has a high gain chain and the charge trigger is
    derived from it, but nothing digitises it: its output pad,
    OUT_AMUXHG, is not connected and its mux buffer is written off. So
    it has no controls on the Settings page, and applying a channel
    leaves its registers exactly as they were. If you ever do need it,
    the console still takes the second argument:

        ch 0 gain 4 6          lgGain 4, hgGain 6
        ch 0 gain 4            lgGain 4, HG untouched

IS THE LINK ACTUALLY WORKING
    A green Connect button means a serial port opened. It does not mean
    the board is alive: the ST-Link enumerates whether or not the
    firmware is running, and at the wrong baud rate every byte arrives
    as junk.

    So the firmware sends a status frame once a second, always, whether
    or not anything is being measured. The lamp on the Main page, on
    the Measure page and in the bottom bar watches that heartbeat:

        green,  link ok
            A heartbeat arrived within the last two seconds and nothing
            has gone wrong in the last ten. The line beside it says how
            long ago and how many have arrived.

        amber,  link unstable
            The link is alive but something is wrong, and the text says
            which: a late heartbeat, frames the board had to drop, a
            failed readout, frames that failed their CRC, or the ASIC
            reporting that Slow Control is not answering. Counts may be
            missing from a run taken in this state.
            It also shows for the first few seconds after connecting,
            saying "waiting for the first heartbeat" - not knowing yet
            is not the same as a verdict.

        red,  no data
            Nothing has arrived for three and a half seconds. The
            board has been unplugged, has reset, is running at a
            different baud rate, or was never running at all.

        grey,  offline
            No port is open.

    Test link, on the Main page, checks the other direction. The
    heartbeat proves the board is transmitting; Test link asks it a
    question and waits for the answer, which proves it is also
    listening. A link where frames arrive but commands go nowhere looks
    perfectly healthy until a setting silently fails to apply.

    Apply and Push all do not take the reply as proof. The "ok" a
    command answers with is bare text sharing the wire with the frames,
    and text has no CRC to survive on - a lost reply and a command that
    never ran look identical. So the board also counts, inside the
    status frame, how many commands it has completed and how many of
    those failed, and the line above the console reports what those
    counters actually did:

        green   every command in the batch completed on the board
        red     the board ran them and the ASIC refused a write, or
                nothing was confirmed within four seconds
        grey    sent, still waiting

    Green there is the only thing that means a setting applied. Nothing
    on this page is confirmed by the console filling with "ok".

    During a run the line under the plot says which kind of quiet you
    have: LINK DOWN means the seconds passing are not being recorded at
    all, while "link ok, no events" means the board is fine and nothing
    is triggering - a threshold, a disabled channel, or no source.

    Starting a run while the lamp is red or amber asks for confirmation
    first, and whatever the lamp said is saved with the measurement
    either way.

TUNING THE HOLD DELAY
    The peak detector is sampled after a programmable delay. If that
    delay does not land on the shaper peak, every pulse is measured too
    small. Sweep Hold delay across its range while watching where the
    photopeak sits, and keep the value that pushes it highest.

READING THE PLOT
    The dashed red line marks the highest bin. Hover anywhere to read
    the ADC value and the counts in that bin. Log scale makes the
    Compton continuum and low intensity peaks visible next to a tall
    photopeak.

WHAT IS SAVED WITH A MEASUREMENT
    Save measurement keeps the histogram and, with it, the state of the
    system at the moment the run started:

        - The ASIC's own registers, read back from the board. When a
          run starts, the firmware is asked for 'stat' and for a dump
          of the channel being measured, and the reply is decoded into
          named values: inDac, lgGain, tauLG, the thresholds, the hold
          delay, whether the LG path is powered. This is what the chip
          was actually set to, not what the form said.
        - The Settings page as it stood, separately. Where the two
          disagree, History says so under "Form and chip disagree" -
          which is how a value that was typed but never applied shows
          itself, months later.
        - The acquisition setup: channel, bins, range, the duration
          asked for, whether raw logging was on.
        - The link's vital signs: the verdict at the start and at the
          save, how many status frames and events arrived, the worst
          heartbeat gap, CRC failures, and the board's own trigger,
          readout and dropped-frame counters.

    All of it is in the JSON under measurements/, in the panel beside
    the plot in History, and in the Metadata sheet of an Excel export,
    one row per setting.

    Runs saved by an older version of this program simply show fewer
    sections. Nothing pretends to know what it was not told.

    In the simulator there are no registers to read back, so only the
    form values are kept, and the record says why.

SCREENSHOTS
    F12, or the Screenshot button in the bottom bar, writes a PNG of the
    whole window into the screenshots folder. It does not ask where to
    put it - it saves and names the file in the status bar, so a run
    never has to stop for a dialog.

    Save plot PNG, on Measure and on History, keeps the spectrum on its
    own. On Measure it also takes in the line of numbers under the plot,
    which is what carries the counts, the rate and the temperature the
    picture was taken at.

    Run label names these files too, exactly as it names the raw CSV.

    Screenshots need Pillow:  pip install pillow

TEMPERATURE
    Both the SiPM gain and the crystal light yield drift with
    temperature: roughly +21 mV per degree on the SiPM breakdown
    voltage, and about -0.3 percent per degree of light output. The
    temperature is logged with each measurement so the peak position
    can be corrected afterwards.

TROUBLESHOOTING
    The lamp is red
        Nothing is arriving. Check the cable, check that the port is
        the ST-Link's, and check that the firmware is running. A board
        that has reset comes back on its own within a few seconds.
    The lamp is amber and says frames were dropped
        The board is producing events faster than the link can carry
        them. Reduce the number of enabled channels or the event rate.
        Counts recorded in this state are incomplete.
    The lamp is green and Test link gets no reply
        Frames are coming back but commands are not getting through.
        Settings will appear to apply and will not. The line above the
        console is what settles it, not the console.
    Apply says NOT confirmed
        The board did not report those commands as completed within
        four seconds - either they never arrived, or the link is losing
        traffic in the direction that carries them. Do not trust the
        settings until an Apply comes back green. This is not the same
        as an Apply that comes back red saying commands FAILED: that
        one reached the board and the ASIC refused the write, which is
        a Slow Control fault, not a link fault.
    No events at all, link green
        Check that the channel is enabled, that the LG AMUX buffer is
        on, and that the threshold is not far above the pulse height.
    Enormous count rate
        The threshold is in the noise, or unused channels are still
        enabled. Disable them, then raise DAC1.
    Peak sitting at the very top bin, saturated counter climbing
        The ASIC's LG output is clipping at its 1.32 V ceiling. The ADC
        is nowhere near full scale. Lower lgGain, or raise inDac.
    Peak low and broad for no obvious reason
        The hold delay is not landing on the shaper peak. Check that
        slow shaping is still on, then sweep the delay.
    History shows "Form and chip disagree"
        Something on the Settings page was typed and never applied.
        The chip's value is the one the spectrum was taken with.

FILES
    Measurements are kept as JSON under the measurements folder next to
    this script - the histogram, plus what it was taken under.

    With "log raw events" ticked, every event is also written to a CSV
    under the raw folder: one row per channel, with the temperature the
    board reported for that event. The histogram is one channel; the raw
    file is the whole readout window, including the dark channels that
    carry the baseline. Analysis that needs a pedestal needs this file.

    Screenshots go to the screenshots folder as PNG, named the same way:
    spectrum_20260824_113000_T30_HV27.14.png.

    Put the temperature point and the DAC setting in "Run label" before
    starting. It names the raw file and shows up in History, which is
    what tells six runs apart afterwards.
"""

    TEXT_HE = """\
מה זה
    ממשק למערכת ספקטרומטריית גמא מבוססת RADIOROC2: גביש CsI(Tl)
    שנקרא על ידי SiPM, מדוגם על ידי STM32F722 ומוזרם למחשב הזה דרך
    ה-COM הווירטואלי של ה-ST-Link.

סדר העבודה
    1. Connect
       בעמוד Main, בחר פורט ולחץ Connect. ודא שהנורה מתחת נדלקת
       בירוק. אם הלוח עוד לא מוכן, לחץ Simulator כדי להכיר את
       הממשק עם נתונים מיוצרים.

    2. Settings
       - השבת קודם את כל הערוצים (Channel = all, ואז Disable).
         ערוצים לא מחוברים עדיין מזינים את טריגר ה-NOR ויירו על רעש.
       - הפעל רק את הערוץ שאליו ה-SiPM מחובר.
       - לחץ Preset CsI. זה חשוב: ברירת המחדל של ה-ASIC היא צעדי
         shaping של 20 ns, בעוד שאור CsI(Tl) דועך לאורך כמיקרושנייה.
         בלי slow shaping המטען לעולם לא נאסף במלואו, ורזולוציית
         האנרגיה קורסת.
       - קבע את הספים. נמוך מדי והמערכת רצה חופשי על רעש; גבוה מדי
         ואתה מאבד את הקצה הנמוך של הספקטרום.
       - לחץ Apply. שום דבר בעמוד הזה לא מגיע לשבב לפני כן, והרשומה
         השמורה תגיד את זה אם תשכח.

    3. Measure
       בחר ערוץ, קבע מספר תאים ומשך, ולחץ Start. הספקטרום נבנה
       בזמן אמת. לחץ Save measurement כשהוא נראה סביר.

    4. History
       בחר ריצה כדי לראות אותה ואת ההגדרות שבהן נמדדה, ואז
       Export to Excel.

מה כל ערך עושה - לפי ערוץ
    אלה נכתבים לערוץ בודד, או לכל 64 בבת אחת עם Channel = all. רק
    הערוץ שה-SiPM יושב עליו משנה לספקטרום; השאר משנים כי הם יכולים
    לירות טריגר.

    Channel
        לאיזה מ-64 הכניסות נכתבות שאר השדות בצד הזה של העמוד. זה
        אינו הערוץ שנכנס להיסטוגרמה - אותו בוחרים בעמוד Measure.

    inDac  (0-255, ברירת מחדל 128)
        DAC של 8 ביט לכל ערוץ, שמכוונן את מתח ה-SiPM ואיתו את ההגבר
        שלו. זהו הכוונון העדין, לא הכפתור הגס: כל הטווח שלו שווה
        בערך לזוג וולטים, שהם כ-28 מעלות של פיצוי דריפט תרמי. כל מה
        שגדול מזה חייב להגיע מספק ה-HV.
        שינוי שלו מזיז את כל הספקטרום לאורך ציר ה-ADC, פוטו-פיק
        וקצה Compton יחד, כי הוא משנה כמה מטען כל גמא מייצר עוד לפני
        שה-ASIC רואה אותו. הוא גם מזיז את קצב ה-dark count, בתלילות.
        ודא את הכיוון פעם אחת על הלוח שלך: הזז אותו ב-32 וראה לאן זז
        הפיק. ה-DAC יושב בין מסילת המתח לפיקסל, ולכן קוד גדול יותר
        פירושו בדרך כלל פחות מתח יתר ופולס קטן יותר - אבל אל תסמוך
        על זה בלי לבדוק.

    lgGain  (0-15, ברירת מחדל 4)
        ההגבר של מגבר המטען בערוץ ה-low gain, בטווח של בערך x0.5 עד
        x8 לאורך שישה עשר הצעדים. זהו הכפתור לגובה הפולס, כי
        OUT_AMUXLG הוא האות היחיד בלוח הזה שמגיע ל-ADC.
        העלאה שלו מותחת את הספקטרום כלפי ערוצי ADC גבוהים יותר. יותר
        מדי, והפוטו-פיק נערם על 1638: הפיק מפסיק לזוז, מקבל קצה חד,
        ומונה ה-saturated בעמוד Measure מתחיל לטפס. התקרה הזו
        שייכת ל-ASIC, לא ל-ADC: OUT_AMUXLG לא מוציא יותר מ-1.32V, שהם
        1638 קודים מתוך 4095 שה-ADC יודע לדגום, ואין מגבר ביניהם.
        הורדה שלו דוחסת את הספקטרום לכיוון האפס. יותר מדי, וכל
        הספקטרום חי בתאים הראשונים, שם הרזולוציה מוגבלת על ידי
        גסות הדיגיטציה.
        כוון לפוטו-פיק בערך בשני שלישים מהתקרה 1638 - כ-1100
        קודים - מה שמשאיר
        מקום לקו אנרגיה גבוה יותר.

    tauLG  (0-15, ברירת מחדל 14 תחת Preset CsI)
        זמן העלייה של ה-shaper בערוץ ה-low gain, כאינדקס. הצעד הוא
        20 ns רגיל ו-120 ns כאשר slow shaping דלוק, כך שהאינדקס מכסה
        20-300 ns מהיר, או 120 ns עד כ-1.8 us איטי.
        העלאה שלו מאריכה את האינטגרציה: יותר מאור הסינטילציה נאסף,
        הפולס גבוה יותר ורזולוציית האנרגיה משתפרת - עד לנקודה שבה
        נאסף גם רעש, או שפולסים מתחילים להיערם זה על זה בקצב גבוה.
        הורדה שלו אוספת פחות מטען, וכל הספקטרום מתכווץ. עבור
        CsI(Tl), שאורו דועך לאורך כמיקרושנייה, זמן עלייה קצר זורק את
        רוב האות.
        שינוי הערך הזה מזיז את שיא ה-shaper בזמן, ולכן יש לכוון מחדש
        את hold delay אחריו. הם אינם בלתי תלויים.

    slow shaping LG  (דלוק תחת Preset CsI)
        מחליף את צעד ה-tauLG מ-20 ns ל-120 ns. כבוי מתאים
        לסינטילטורים מהירים כמו LYSO או פלסטיק. דלוק נדרש ל-CsI(Tl).
        כיבוי שלו בזמן ש-tauLG נשאר על 14 מפיל את זמן העלייה מכ-1.8
        us ל-300 ns, ואיתו את רוב גובה הפולס. אם הפוטו-פיק קורס בלי
        סיבה נראית לעין, זה הדבר הראשון לבדוק.

    patGain  (0-63, ברירת מחדל 32)
        ההגבר של מגבר הזמן, לולאה סגורה 15 עד 100. הוא מזין רק את
        הדיסקרימינטורים - הוא אינו במסלול המדוגם.
        העלאה שלו גורמת לטריגר לראות פולס גדול יותר, כך שאותו סף
        DAC1 תופס עכשיו אירועים באנרגיה נמוכה יותר וקצב הטריגר עולה.
        הורדה שלו מעלה את הסף האפקטיבי.
        הוא אינו מזיז את הפוטו-פיק. הוא משנה אילו אירועים נרשמים, לא
        כמה גבוהים הם אחרי שנרשמו.

    trim T1 / trim T2  (0-63 כל אחד, ברירת מחדל 0)
        כוונון סף לכל ערוץ: כל צעד מוריד את נקודת הטריגר של הערוץ
        בשבריר מיליוולט מתחת לסף הגלובלי, 0-15 mV לאורך כל הטווח.
        העלאה של אחד מהם גורמת לערוץ לירות ביתר קלות. הם קיימים כדי
        להשוות נקודות טריגר בין ערוצי מערך; במערכת חד-ערוצית השאר
        אותם על 0 והשתמש ב-DAC1.

    Enable / Disable
        מפעיל או מכבה ערוץ שלם: מגברים, דיסקרימינטורים, shapers
        ו-peak detectors יחד.
        השבת כל ערוץ שלא חיברת אליו דבר. כניסה לא מחוברת צפה,
        הדיסקרימינטור שלה יורה על רעש, ומכיוון שהטריגר הוא NOR על פני
        כל הערוצים המערכת כולה רצה חופשי. זו הסיבה הרגילה לקצב ספירה
        של עשרות אלפים בשנייה.

מה כל ערך עושה - גלובלי
    עותק אחד מכל אחד מאלה, משותף לכל 64 הערוצים.

    Threshold DAC1 low  (0-1023, ברירת מחדל 300)
        סף הטריגר הזמני הנמוך, ועם selTrig שבברירת המחדל הוא זה
        שבאמת מחליט מה נרשם.
        העלאה שלו מעבירה פחות אירועים: קודם נחסם הרעש, ואז הקצה
        הנמוך של הספקטרום, שמתחיל להיראות כספקטרום שמתחיל בחדות
        באמצע ציר ה-ADC במקום לעלות מאפס.
        הורדה שלו חושפת יותר מהספקטרום, עד שהסף מגיע לרצפת הרעש
        והקצב מתפוצץ - אלפי ספירות בשנייה נערמות בתאים הנמוכים.
        קבע אותו מעט מעל הנקודה שבה הורדה נוספת מפסיקה להוסיף
        ספקטרום ומתחילה להוסיף קצב.

    Threshold DAC2 high  (0-1023, ברירת מחדל 500)
        סף הטריגר הזמני הגבוה. דיסקרימינטור שני על אותו אות, לשימוש
        במדידות מסוג time-over-threshold. הוא משפיע על הרכישה רק אם
        selTrig מנתב אותו. בדרך כלל השאר אותו מעל DAC1 והתעלם.

    Threshold DACQ charge  (0-1023, ברירת מחדל 200)
        הסף של דיסקרימינטור המטען, שרץ ממסלול ה-high gain. ה-HG אינו
        מדוגם בלוח הזה, אבל הדיסקרימינטור שלו עדיין עובד, ולכן זה
        משנה אם selTrig בוחר בטריגר המטען. אחרת הוא לא עושה דבר.

    Hold delay  (0-255)  ו-Slope trim  (0-15)
        יחד הם קובעים כמה זמן אחרי הטריגר נדגם ה-peak detector:
        delay x 0.85 ns x slope trim. ה-slope trim מכפיל, כך שב-0
        ההשהיה מתכווצת למינימום של הקושחה ועל 15 היא הארוכה ביותר.
        זהו הכפתור החשוב ביותר אחרי ההגבר, והפחות מובן מאליו. דגימה
        מוקדמת מדי וה-shaper עוד לא הגיע לשיאו; מאוחרת מדי והוא כבר
        דעך. כך או כך כל פולס נמדד כקטן מדי, והפוטו-פיק יושב נמוך,
        רחב ובמקום הלא נכון - מה שנראה בדיוק כמו בעיית הגבר.
        כוון אותו על ידי סריקת ההשהיה לאורך הטווח, ושמור את הערך
        שדוחף את הפוטו-פיק הכי גבוה. כוון מחדש בכל פעם ש-tauLG או
        slow shaping משתנים.
        הקושחה מוסיפה מרווח על הערך הנומינלי בגלל הפיזור בתא ההשהיה.
        הקלד stat בקונסולה כדי לראות את hold_ns: כמה הקריאה באמת
        תמתין. אחרי Preset CsI הוא חייב לגדול, לא להישאר במקומו.

    selTrig  (0-15, ברירת מחדל 4)
        איזה אות מתחיל רכישה. שני הביטים הנמוכים בוחרים את המקור:
        0 = הטריגר הגלובלי, 1 = T1, 2 = T2, 3 = מטען. כשהם 0, ביטים
        3:2 בוחרים איזה טריגר גלובלי: 0 = TRIGEXT חיצוני, 1 = T1,
        2 = T2, 3 = מטען.
        ברירת המחדל, 4, פירושה "טריגר גלובלי מוזן מ-T1" - כלומר
        דיסקרימינטור ה-DAC1 של כל ערוץ פעיל. שינוי שלו משנה איזה סף
        בכלל משפיע, וזו דרך מהירה לגרום ל-DAC1 להיראות שבור.

    external hold (HOLDEXT pin)
        כבוי, תא ההשהיה הפנימי מתזמן את ה-hold מרגע הטריגר - זה מה
        ש-Hold delay למעלה שולט בו. דלוק, פין חיצוני עושה זאת במקומו.
        שום דבר בלוח הזה לא מניע את הפין ההוא, ולכן הדלקה שלו עוצרת
        את הרכישות.

    AMUX LG buffer
        מפעיל את המאגר שמניע את OUT_AMUXLG - הפין האנלוגי היחיד
        שה-ADC מחובר אליו. כבוי, ה-ADC קורא את אותו ערך שטוח לכל
        אירוע, לא משנה מה הגלאי עושה.
        הוא צריך להיות דלוק תמיד. יש לו פקד כאן רק כדי של"כל אירוע
        קורא את אותו מספר" יהיה מקום ברור להיבדק בו.

מה כל ערך עושה - עמוד Measure
    Channel
        איזה ערוץ נכנס להיסטוגרמה. קובץ ה-raw שומר בכל מקרה את כל
        הערוצים בחלון הקריאה, וזה מה שהופך את השכנים החשוכים - ואיתם
        את הפדסטל - לניתנים לשחזור בדיעבד.

    Bins  (128-2048)
        לכמה תאים מחולק הטווח הבר-השגה 0-1638. 512 תאים הם קצת
        יותר מ-3 קודי ADC לתא.
        יותר תאים מפרידים פיק צר טוב יותר, אבל דורשים פי כמה יותר
        ספירות עד שהצורה מפסיקה להיות רעש. פחות תאים נותנים עקומה
        חלקה מהר, במחיר של דיוק במיקום מרכז הפיק. שינוי הערך מנקה את
        ההיסטוגרמה.

    Duration  (שניות, 0 = ידני)
        כמה זמן Start רץ. 0 רץ עד שלוחצים Stop. הסטטיסטיקה בכל תא
        משתפרת כשורש הזמן: פי ארבעה זמן הוא פי שניים ניקיון.

    Run label
        נותן שם לקובץ ה-raw ולצילומי המסך, ומופיע ב-History. כתוב בו
        את נקודת הטמפרטורה ואת ה-HV. זה מה שיבדיל בין שש ריצות של
        סריקה אחר כך.

    Run conditions  (איזוטופ, טמפרטורת תנור, שלושה שדות HV, הערה)
        עובדות על השולחן ששום חוט לא נושא. הלוח מדווח על הטמפרטורה
        שלו ואפשר לשאול אותו מה יש ברגיסטרים של ה-ASIC. הוא לא יודע
        למה הוגדר ספק ה-HV, מה עמד מול הגביש, או מה נאמר לתא.

        Isotope נשאר ריק עד שתגיד אחרת. "no source" היא מדידה אמיתית -
        ריצת הרקע שממנה מגיע הפדסטל - וריצה שמעולם לא אמרה מול מה
        היא עמדה איננה הריצה הזו.

        Oven setpoint הוא מה שנאמר לתא. temp_c, לידו ב-History, הוא מה
        שהלוח הרגיש. ההפרש ביניהם הוא הגרדיאנט התרמי, וזו הסיבה
        לשמור את שניהם.

        HV supply set ו-HV measured הם שני המספרים של הספק עצמו.
        V at channel pins הוא השלישי והמועיל מכולם: נמדד על השקע כשהחיישן
        מנותק, והוא HV פחות מה שה-input DAC מחזיק עליו את הכניסה -
        וההפרש הזה הוא מתח היתר שה-SiPM באמת יראה.

        כל השישה נוסעים עם הריצה: ב-History, בחלונית ההגדרות, ובגיליון
        Metadata של ייצוא Excel - מספרים נשמרים כמספרים, כך שאפשר
        לשרטט סריקה מולם. הם בכוונה לא נזכרים בין הרצות. HV ישן
        שנדבק בשקט לריצה של מחר גרוע יותר משדה ריק.

    log scale
        ציר אנכי בלבד. מאפשר לראות את רצף Compton ופיקים קטנים לצד
        פוטו-פיק גבוה. הוא אינו משנה דבר במה שנרשם.

    log raw events
        כותב כל אירוע, כל ערוץ, ל-CSV תחת raw/. כל ניתוח שצריך
        פדסטל צריך את הקובץ הזה. הוא עולה בדיסק, לא בספירות.

איזה אות נמדד
    רק OUT_AMUXLG מחובר ל-ADC, ולכן כל ספקטרום כאן הוא מסלול ה-low
    gain: קוד ADC אחד לכל ערוץ לכל אירוע, וציר ה-X הוא ספירות ADC
    גולמיות. אין מה לבחור בין מסלולי הגבר ואין מה לכייל ביניהם.

    ל-ASIC עדיין יש מסלול high gain וטריגר המטען נגזר ממנו, אבל אף
    אחד לא מדגם אותו: פד היציאה שלו, OUT_AMUXHG, אינו מחובר והמאגר
    שלו נכתב כבוי. לכן אין לו פקדים בעמוד Settings, ו-Apply channel
    משאיר את הרגיסטרים שלו בדיוק כפי שהיו. אם בכל זאת תצטרך אותו,
    הקונסולה עדיין מקבלת ארגומנט שני:

        ch 0 gain 4 6          lgGain 4, hgGain 6
        ch 0 gain 4            lgGain 4, ה-HG לא נגע

האם התקשורת באמת עובדת
    כפתור Connect ירוק אומר שפורט טורי נפתח. הוא אינו אומר שהלוח חי:
    ה-ST-Link נרשם בין אם הקושחה רצה ובין אם לא, ובקצב שידור שגוי כל
    בית מגיע כזבל.

    לכן הקושחה שולחת מסגרת סטטוס פעם בשנייה, תמיד, בין אם נמדד משהו
    ובין אם לא. הנורה בעמוד Main, בעמוד Measure ובשורה התחתונה עוקבת
    אחרי הדופק הזה:

        ירוק,  link ok
            דופק הגיע בשתי השניות האחרונות ושום דבר לא נכשל בעשר
            האחרונות. השורה לידו אומרת לפני כמה זמן וכמה הגיעו.

        כתום,  link unstable
            הקישור חי אבל משהו לא בסדר, והטקסט אומר מה: דופק מאוחר,
            מסגרות שהלוח נאלץ לזרוק, קריאה שנכשלה, מסגרות שנכשלו
            ב-CRC, או ASIC שמדווח שה-Slow Control אינו עונה. ייתכן
            שחסרות ספירות בריצה שנלקחה במצב הזה.
            הוא מופיע גם בשניות הראשונות אחרי חיבור, ואומר
            "waiting for the first heartbeat" - עוד לא לדעת זה לא
            אותו דבר כמו פסק דין.

        אדום,  no data
            שלוש וחצי שניות בלי כלום. הלוח נותק, אותחל, רץ בקצב
            שידור אחר, או שמעולם לא רץ. אם בתים כן מגיעים אבל שום
            דבר לא מפוענח, הטקסט אומר לבדוק את קצב השידור.

        אפור,  offline
            אין פורט פתוח.

    Test link, בעמוד Main, בודק את הכיוון ההפוך. הדופק מוכיח שהלוח
    משדר; Test link שואל אותו שאלה ומחכה לתשובה, מה שמוכיח שהוא גם
    מקשיב. קישור שבו מסגרות מגיעות אבל פקודות הולכות לאיבוד נראה
    בריא לגמרי - עד שהגדרה נכשלת בשקט.

    Apply ו-Push all אינם מסתמכים על התשובה כהוכחה. ה-"ok" שפקודה
    עונה הוא טקסט חשוף שחולק את הקו עם המסגרות, ולטקסט אין CRC לשרוד
    בעזרתו - תשובה שאבדה ופקודה שמעולם לא רצה נראות זהות. לכן הלוח גם
    סופר, בתוך מסגרת הסטטוס, כמה פקודות השלים וכמה מהן נכשלו, והשורה
    מעל הקונסולה מדווחת מה המונים האלה באמת עשו:

        ירוק   כל הפקודות באצווה הושלמו בלוח
        אדום   הלוח הריץ אותן וה-ASIC סירב לכתיבה, או ששום דבר לא
               אושר בתוך ארבע שניות
        אפור   נשלח, עדיין ממתין

    ירוק שם הוא הדבר היחיד שמשמעותו שההגדרה הוחלה. שום דבר בעמוד הזה
    לא מאושר על ידי קונסולה שמתמלאת ב-"ok".

    בזמן ריצה השורה מתחת לגרף אומרת איזה סוג של שקט זה: LINK DOWN
    פירושו שהשניות שעוברות אינן נרשמות בכלל, ואילו
    "link ok, no events" פירושו שהלוח בסדר ושום דבר לא מפעיל טריגר -
    סף, ערוץ מושבת, או שאין מקור.

    התחלת ריצה כשהנורה אדומה או כתומה מבקשת אישור, ומה שהנורה אמרה
    נשמר עם המדידה בכל מקרה.

כיוונון השהיית ה-hold
    ה-peak detector נדגם אחרי השהיה ניתנת לתכנות. אם ההשהיה אינה
    נוחתת על שיא ה-shaper, כל פולס נמדד כקטן מדי. סרוק את Hold delay
    לאורך הטווח שלו תוך מעקב אחרי מיקום הפוטו-פיק, ושמור את הערך
    שדוחף אותו הכי גבוה.

קריאת הגרף
    הקו האדום המקווקו מסמן את התא הגבוה ביותר. רחף עם העכבר בכל מקום
    כדי לקרוא את ערך ה-ADC ואת מספר הספירות באותו תא. log scale
    מאפשר לראות את רצף Compton ופיקים חלשים לצד פוטו-פיק גבוה.

מה נשמר עם מדידה
    Save measurement שומר את ההיסטוגרמה, ואיתה את מצב המערכת ברגע
    שהריצה התחילה:

        - הרגיסטרים של ה-ASIC עצמו, כפי שנקראו מהלוח. כשריצה
          מתחילה, הקושחה נשאלת stat ו-dump של הערוץ הנמדד, והתשובה
          מפוענחת לערכים בעלי שם: inDac, lgGain, tauLG, הספים,
          השהיית ה-hold, והאם מסלול ה-LG מוזן. זה מה שהשבב היה מכוון
          אליו בפועל, לא מה שהטופס אמר.
        - עמוד Settings כפי שהיה, בנפרד. איפה שהשניים אינם מסכימים,
          History אומר זאת תחת "Form and chip disagree" - כך ערך
          שהוקלד ומעולם לא הוחל מסגיר את עצמו, חודשים אחרי.
        - הגדרות הרכישה: ערוץ, תאים, טווח, המשך שהתבקש, והאם רישום
          raw היה פעיל.
        - סימני החיים של הקישור: הפסק בהתחלה ובשמירה, כמה מסגרות
          סטטוס ואירועים הגיעו, פער הדופק הגרוע ביותר, כשלי CRC,
          והמונים של הלוח עצמו לטריגרים, קריאות ומסגרות שנזרקו.

    הכול נמצא ב-JSON תחת measurements/, בחלונית שליד הגרף ב-History,
    ובגיליון Metadata של ייצוא Excel - שורה לכל הגדרה.

    ריצות שנשמרו בגרסה קודמת של התוכנה פשוט מציגות פחות מקטעים. שום
    דבר לא מתיימר לדעת מה שלא נאמר לו.

    במצב סימולציה אין רגיסטרים לקרוא, ולכן נשמרים רק ערכי הטופס,
    והרשומה אומרת למה.

צילומי מסך
    F12, או כפתור Screenshot בשורה התחתונה, כותב PNG של כל החלון
    לתיקיית screenshots. הוא אינו שואל לאן לשמור - הוא שומר ואומר את
    שם הקובץ בשורת הסטטוס, כך שריצה לעולם לא נעצרת בשביל דיאלוג.

    Save plot PNG, ב-Measure וב-History, שומר את הספקטרום לבדו.
    ב-Measure הוא לוקח גם את שורת המספרים שמתחת לגרף, שהיא זו שנושאת
    את הספירות, הקצב והטמפרטורה שבהם התמונה נלקחה.

    Run label נותן שם גם לקבצים האלה, בדיוק כפי שהוא נותן שם ל-CSV
    הגולמי.

    צילומי מסך דורשים את Pillow:  pip install pillow

טמפרטורה
    גם ההגבר של ה-SiPM וגם תפוקת האור של הגביש נודדים עם הטמפרטורה:
    בערך 21 mV למעלה על מתח הפריצה של ה-SiPM, וכ-0.3 אחוז למעלה
    בתפוקת האור. הטמפרטורה נרשמת עם כל מדידה כדי שאפשר יהיה לתקן את
    מיקום הפיק בדיעבד.

פתרון בעיות
    הנורה אדומה
        שום דבר לא מגיע. בדוק את הכבל, בדוק שהפורט הוא זה של
        ה-ST-Link, ובדוק שהקושחה רצה. לוח שאותחל חוזר מעצמו תוך
        כמה שניות.
    הנורה כתומה ואומרת שמסגרות נזרקו
        הלוח מייצר אירועים מהר יותר מכפי שהקישור מסוגל לשאת. צמצם את
        מספר הערוצים הפעילים או את קצב האירועים. ספירות שנרשמו במצב
        הזה אינן שלמות.
    הנורה ירוקה ו-Test link לא מקבל תשובה
        מסגרות חוזרות אבל פקודות אינן עוברות. הגדרות ייראו כאילו
        הוחלו, ולא יוחלו. השורה מעל הקונסולה היא שמכריעה, לא
        הקונסולה.
    Apply אומר NOT confirmed
        הלוח לא דיווח שהפקודות האלה הושלמו בתוך ארבע שניות - או
        שמעולם לא הגיעו, או שהקישור מאבד תעבורה בכיוון שנושא אותן. אל
        תסמוך על ההגדרות עד ש-Apply יחזור ירוק. זה לא אותו דבר כמו
        Apply שחוזר אדום ואומר שפקודות FAILED: זה הגיע ללוח וה-ASIC
        סירב לכתיבה, כלומר תקלת Slow Control ולא תקלת קישור.
    אין אירועים בכלל, נורה ירוקה
        בדוק שהערוץ מופעל, שמאגר ה-AMUX של LG דלוק, ושהסף אינו הרבה
        מעל גובה הפולס.
    קצב ספירה עצום
        הסף בתוך הרעש, או שערוצים לא בשימוש עדיין מופעלים. השבת אותם
        ואז העלה את DAC1.
    הפיק יושב בתא העליון ביותר, ומונה ה-saturated מטפס
        מוצא ה-LG של ה-ASIC נחתך על תקרת ה-1.32V שלו. ה-ADC לא
        מתקרב בכלל לסקאלה המלאה. הורד את lgGain, או העלה את inDac.
    הפיק נמוך ורחב בלי סיבה נראית לעין
        השהיית ה-hold אינה נוחתת על שיא ה-shaper. בדוק ש-slow
        shaping עדיין דלוק, ואז סרוק את ההשהיה.
    History מציג "Form and chip disagree"
        משהו בעמוד Settings הוקלד ומעולם לא הוחל. הערך של השבב הוא
        זה שהספקטרום נמדד איתו.

קבצים
    מדידות נשמרות כ-JSON בתיקיית measurements ליד הסקריפט הזה -
    ההיסטוגרמה, ועוד מה שהיא נמדדה תחתיו.

    כאשר log raw events מסומן, כל אירוע נכתב גם ל-CSV תחת תיקיית
    raw: שורה לכל ערוץ, עם הטמפרטורה שהלוח דיווח עבור אותו אירוע.
    ההיסטוגרמה היא ערוץ אחד; קובץ ה-raw הוא כל חלון הקריאה, כולל
    הערוצים החשוכים שנושאים את קו הבסיס. כל ניתוח שצריך פדסטל צריך
    את הקובץ הזה.

    צילומי מסך הולכים לתיקיית screenshots כ-PNG, עם אותה שיטת שמות:
    spectrum_20260824_113000_T30_HV27.14.png.

    כתוב את נקודת הטמפרטורה ואת ערך ה-DAC ב-Run label לפני שאתה
    מתחיל. זה נותן שם לקובץ הגולמי ומופיע ב-History, וזה מה שיבדיל
    בין שש ריצות אחר כך.
"""

    # Right-to-left embedding, and the pop that ends it. Tk gives every
    # line a left-to-right base direction, which puts the full stop of a
    # Hebrew sentence on the wrong side; these ask the platform for an
    # RTL base instead. They go on one line at a time, which is why the
    # Hebrew text is hard-wrapped and its widget never re-wraps: a line
    # broken by the widget loses the embedding on its continuation and
    # comes out scrambled.
    RLE, PDF = "\u202B", "\u202C"

    LANGS = ("en", "he")

    def _lang(self):
        """Everything that differs between the two renderings."""
        if self.v_lang.get() == "he":
            return {
                "text": self.TEXT_HE,
                "title": "\u05d4\u05d5\u05e8\u05d0\u05d5\u05ea",
                "font": ("Segoe UI", 11),
                "justify": "right",
                "wrap": "none",          # see the note on RLE above
                "nav_side": "right",     # the index mirrors with the text
                "rtl": True,
            }
        return {
            "text": self.TEXT,
            "title": "Instructions",
            "font": ("Consolas", 10),
            "justify": "left",
            "wrap": "word",
            "nav_side": "left",
            "rtl": False,
        }

    def build(self):
        top = ttk.Frame(self)
        top.pack(fill="x")
        ttk.Button(top, text="< Main", command=lambda: self.app.show("main"))\
            .pack(side="left")
        self.lbl_title = ttk.Label(top, text="Instructions", style="H1.TLabel")
        self.lbl_title.pack(side="left", padx=12)

        self.v_lang = tk.StringVar(
            value=self.app.prefs.get("help_lang", "en"))
        if self.v_lang.get() not in self.LANGS:
            self.v_lang.set("en")
        pick = ttk.Frame(top)
        pick.pack(side="right")
        ttk.Label(pick, text="language", style="Sub.TLabel").pack(side="left")
        for code, name in (("en", "English"),
                           ("he", "\u05e2\u05d1\u05e8\u05d9\u05ea")):
            ttk.Radiobutton(pick, text=name, value=code,
                            variable=self.v_lang, command=self.relayout)\
                .pack(side="left", padx=(10, 0))

        self.wrap = ttk.Frame(self)
        self.wrap.pack(fill="both", expand=True, pady=12)
        self.relayout()

    def relayout(self):
        """Draw the page in the language that is now selected.

        The widgets are rebuilt rather than reconfigured: font, wrapping,
        justification and which side the index sits on all change
        together, and there is nothing to gain by doing that in place.
        """
        lang = self._lang()
        self.app.set_pref("help_lang", self.v_lang.get())
        for child in self.wrap.winfo_children():
            child.destroy()
        self.lbl_title.configure(text=lang["title"])

        # Headings are the lines that start at column 0. Ten screens of
        # text cannot be scrolled through, so they get an index.
        lines = lang["text"].split("\n")
        self.marks = [(i + 1, ln) for i, ln in enumerate(lines)
                      if ln and not ln.startswith(" ")]

        nav = tk.Listbox(self.wrap, width=40, relief="flat", bg=C_PANEL,
                         fg=C_TEXT, highlightthickness=1,
                         highlightbackground=C_GRID, activestyle="none",
                         justify="right" if lang["rtl"] else "left",
                         font=("Segoe UI", 9))
        for _, ln in self.marks:
            nav.insert("end", self._shape(ln if lang["rtl"]
                                          else ln.capitalize(), lang))
        nav.pack(side=lang["nav_side"], fill="y",
                 padx=(12, 0) if lang["rtl"] else (0, 12))
        nav.bind("<<ListboxSelect>>", self._jump)
        self.nav = nav

        body = ttk.Frame(self.wrap)
        body.pack(side=lang["nav_side"], fill="both", expand=True)
        txt = tk.Text(body, wrap=lang["wrap"], relief="flat", bg=C_PANEL,
                      fg=C_TEXT, font=lang["font"], padx=14, pady=12)
        sb = ttk.Scrollbar(body, command=txt.yview)
        sb.pack(side="left" if lang["rtl"] else "right", fill="y")
        txt.pack(side="right" if lang["rtl"] else "left",
                 fill="both", expand=True)
        txt.configure(yscrollcommand=sb.set)

        txt.tag_configure("body", justify=lang["justify"])
        txt.tag_configure("head", font=(lang["font"][0], lang["font"][1],
                                        "bold"),
                          foreground=C_ACCENT_DK, spacing1=10,
                          justify=lang["justify"])
        # Right to left, the indent cannot be leading spaces: reordered,
        # they end up on the far side of the line and move its right edge
        # by nothing at all. Measured, four spaces and eight leave a line
        # in exactly the same place as none. The widget's own right
        # margin does move it, so the structure is expressed that way and
        # _shape() drops the spaces before the text is ever laid out.
        # One digit's width per space, to match the weight of the
        # indent on the English page, which is set in a monospace font.
        space = tkfont.Font(font=lang["font"]).measure("0")
        heads = {n for n, _ in self.marks}
        for i, line in enumerate(lines, start=1):
            tags = ["head" if i in heads else "body"]
            if lang["rtl"]:
                pad = len(line) - len(line.lstrip(" "))
                if pad:
                    tag = "ind%d" % pad
                    txt.tag_configure(tag, rmargin=pad * space)
                    tags.append(tag)
            txt.insert("end", self._shape(line, lang) + "\n", tuple(tags))
        txt.configure(state="disabled")
        self.txt = txt

    def _shape(self, line, lang):
        """One line, ready to hand to Tk in this language.

        The leading spaces come off for Hebrew - relayout() turns them
        into a right margin instead - so the bidi engine only ever sees
        words, and never has to decide which side a run of neutral
        spaces belongs on.
        """
        if not lang["rtl"] or not line.strip():
            return line
        return self.RLE + line.strip() + self.PDF

    def _jump(self, _ev=None):
        sel = self.nav.curselection()
        if sel:
            # see() puts the line on screen; yview puts it at the top,
            # which is what a table of contents is expected to do.
            self.txt.see("%d.0" % self.marks[sel[0]][0])
            self.txt.yview("%d.0" % self.marks[sel[0]][0])


# ===================================================================
#  Application shell
# ===================================================================
class App(tk.Tk):
    def __init__(self, sim=False):
        super().__init__()
        self.title("RADIOROC2 Spectrometer")
        self.geometry("1060x740")
        self.minsize(940, 640)
        self.configure(bg=C_BG)

        self.link = Link()
        self.prefs = load_prefs()
        # Two things the pages all need to agree about: what the board
        # says it is set to, and whether it is still talking.
        self.state = DeviceState()
        self.health = LinkHealth()
        self.cmdwatch = CmdWatch()
        self._health_at = 0.0
        self._style()

        # The bottom bar is packed first, before the pages. History asks
        # for 745 px of height in a 740 px window, and the packer gives
        # whatever comes after it the cavity that is left - which was
        # nothing, so the status line never appeared at all.
        self._status_text = "offline"
        self._toast_job = None
        bottom = ttk.Frame(self)
        bottom.pack(fill="x", side="bottom")
        self.status = ttk.Label(bottom, text=self._status_text,
                                style="Sub.TLabel", anchor="w",
                                padding=(14, 6))
        self.status.pack(side="left", fill="x", expand=True)
        ttk.Button(bottom, text="Screenshot (F12)",
                   command=self.screenshot).pack(side="right", padx=(6, 10))
        # Visible from every page, because the link can die on any of
        # them and the one that matters is whichever is on screen.
        self.health_bar = HealthLight(bottom, detail=False)
        self.health_bar.pack(side="right", padx=(6, 10))

        container = ttk.Frame(self)
        container.pack(fill="both", expand=True)
        container.rowconfigure(0, weight=1)
        container.columnconfigure(0, weight=1)

        self.pages = {}
        for key, cls in (("main", PageMain), ("settings", PageSettings),
                         ("measure", PageMeasure), ("history", PageHistory),
                         ("help", PageHelp)):
            pg = cls(container, self)
            pg.grid(row=0, column=0, sticky="nsew")
            self.pages[key] = pg

        self.show("main")
        self.bind("<F12>", lambda _e: self.screenshot())
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        if sim:
            self.link.open_sim()
            self.set_state(True, "simulator")

        self.after(40, self.pump)

    def _style(self):
        st = ttk.Style(self)
        try:
            st.theme_use("clam")
        except tk.TclError:
            pass
        st.configure(".", background=C_BG, foreground=C_TEXT,
                     font=("Segoe UI", 10))
        st.configure("TFrame", background=C_BG)
        st.configure("Tile.TFrame", background=C_PANEL, relief="solid",
                     borderwidth=1)
        st.configure("TLabelframe", background=C_BG, bordercolor=C_GRID)
        st.configure("TLabelframe.Label", background=C_BG, foreground=C_MUTED)
        st.configure("TLabel", background=C_BG)
        st.configure("H1.TLabel", font=("Segoe UI Semibold", 17))
        st.configure("H2.TLabel", font=("Segoe UI Semibold", 13))
        st.configure("Sub.TLabel", foreground=C_MUTED)
        st.configure("Ok.TLabel", foreground=C_OK)
        st.configure("Warn.TLabel", foreground=C_WARN)
        st.configure("Bad.TLabel", foreground=C_BAD)
        st.configure("Mono.TLabel", font=("Consolas", 9))
        st.configure("TButton", padding=(12, 6))
        st.configure("Accent.TButton", background=C_ACCENT, foreground="white")
        st.map("Accent.TButton", background=[("active", C_ACCENT_DK)])
        st.configure("Treeview", background=C_PANEL, fieldbackground=C_PANEL,
                     rowheight=24)

    # -- preferences
    def set_pref(self, key, value):
        """Remember one choice across runs."""
        if self.prefs.get(key) != value:
            self.prefs[key] = value
            save_prefs(self.prefs)

    # -- navigation
    def show(self, key):
        pg = self.pages[key]
        pg.tkraise()
        pg.on_show()

    # -- screenshots
    def screenshot(self, target=None, label="", kind="shot"):
        """PNG of the window, or of `target`, straight into screenshots/.

        No file dialog: during a sweep the point is one keypress and back
        to the run. The status bar says where it went.
        """
        if target is None:
            widgets = [self]
        elif isinstance(target, (list, tuple)):
            widgets = list(target)
        else:
            widgets = [target]
        try:
            path = grab_widgets(widgets, shot_path(kind, label),
                                pad=0 if target is None else 6)
        except Exception as exc:
            messagebox.showerror("Screenshot failed", str(exc))
            return None
        self.toast("saved  screenshots/" + os.path.basename(path))
        return path

    # -- status bar
    def toast(self, msg, ms=5000):
        """Say something in the status bar, then hand it back to the link."""
        if self._toast_job:
            self.after_cancel(self._toast_job)
        self.status.configure(text=msg)
        self._toast_job = self.after(ms, self._clear_toast)

    def _clear_toast(self):
        if self._toast_job:
            self.after_cancel(self._toast_job)
            self._toast_job = None
        self.status.configure(text=self._status_text)

    # -- connection state
    def set_state(self, connected, where):
        self._status_text = f"connected to {where}" if connected else "offline"
        self._clear_toast()          # the link state outranks a toast
        # A fresh connection starts with a clean sheet: the CRC errors
        # and missed heartbeats of the last one say nothing about it.
        self.health.reset()
        self.health.on_stats(self.link.stats())
        self.refresh_state_labels()
        self.refresh_health()

    def refresh_state_labels(self):
        main = self.pages["main"]
        if self.link.connected:
            main.lbl_state.configure(text="connected", style="Ok.TLabel")
        else:
            main.lbl_state.configure(text="offline", style="Bad.TLabel")

    # -- main pump: drain the link and refresh the live page
    def pump(self):
        drained = 0
        while drained < 4000:
            try:
                frame = self.link.frames.get_nowait()
            except queue.Empty:
                break
            drained += 1
            self.health.on_frame(frame)
            self.show_cmd_verdict(self.cmdwatch.on_frame(frame))
            self.pages["measure"].feed(frame)

        while True:
            try:
                line = self.link.text.get_nowait()
            except queue.Empty:
                break
            # Parse first, then print. The console is for reading; the
            # parsed copy is what gets saved with the measurement.
            self.state.feed(line)
            self.pages["settings"].log(line)

        now = time.monotonic()
        if now - self._health_at > 0.5:
            self._health_at = now
            self.health.on_stats(self.link.stats())
            self.refresh_health()
            # A batch that is never confirmed produces no frame to
            # notice it by, so the timeout has to be asked, not awaited.
            self.show_cmd_verdict(self.cmdwatch.poll())

        self.pages["measure"].tick()
        self.after(40, self.pump)

    # -- command confirmation
    def show_cmd_verdict(self, verdict):
        """Put one verdict everywhere it needs to be.

        The status bar because it is where the eye is after pressing a
        button, the page label because a toast expires and this must
        not, and the console because that is the record.
        """
        if verdict is None:
            return
        ok, msg = verdict
        self.toast(msg)
        self.pages["settings"].set_result(ok, msg)
        self.pages["settings"].log("  " + msg)

    # -- link health
    def refresh_health(self):
        """Push one verdict to every indicator on screen."""
        level, why = self.health.summary(self.link.connected)
        self.health_bar.update_health(level, why)
        self.pages["main"].light.update_health(level, why)
        self.pages["measure"].light.update_health(level, why)

    def on_close(self):
        self.link.close()
        self.destroy()


def main():
    ap = argparse.ArgumentParser(description="RADIOROC2 spectrometer GUI")
    ap.add_argument("--sim", action="store_true",
                    help="start in simulator mode, no hardware needed")
    args = ap.parse_args()
    App(sim=args.sim).mainloop()


if __name__ == "__main__":
    main()
