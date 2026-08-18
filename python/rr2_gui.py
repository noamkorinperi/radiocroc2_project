#!/usr/bin/env python3
"""
rr2_gui.py - desktop front end for the RADIOROC2 gamma spectrometer.

Pages
    Main       navigation hub
    Settings   push Slow Control parameters to the ASIC
    Measure    run an acquisition and watch the spectrum build up
    History    the last six measurements, with Excel export
    Help       static instructions

Requirements
    pyserial   pip install pyserial      (hardware link)
    openpyxl   pip install openpyxl      (Excel export; CSV fallback otherwise)

rr2_decode.py must sit next to this file - it owns the wire protocol so
there is only one place to change if the framing ever moves.

Run without hardware:
    python rr2_gui.py --sim
"""

import argparse
import csv
import json
import os
import queue
import random
import sys
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import filedialog, messagebox, ttk

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


ADC_MAX = 4095                      # 12-bit ADC
STORE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "measurements")
MAX_STORED = 6

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
C_BAD = "#c0392b"


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
        self._sim_state = {"rate": 180.0, "centre": 2400, "sigma": 90}

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
            # Text replies and binary frames share the pipe. The decoder
            # skips anything that is not a valid frame, so printable runs
            # are surfaced separately for the log.
            for frame in self._dec.feed(chunk):
                self.frames.put(frame)

    def _read_sim(self):
        """Poisson-ish event generator with a Gaussian photopeak."""
        seq = 0
        t0 = time.time()
        next_status = t0 + 1.0
        while not self._stop.is_set():
            time.sleep(0.02)
            now = time.time()
            n = max(0, int(random.gauss(self._sim_state["rate"] * 0.02,
                                        self._sim_state["rate"] * 0.02 * 0.4)))
            for _ in range(n):
                seq += 1
                if random.random() < 0.72:
                    hg = random.gauss(self._sim_state["centre"],
                                      self._sim_state["sigma"])
                else:
                    hg = random.expovariate(1 / 700.0)      # Compton-ish tail
                hg = int(max(0, min(ADC_MAX, hg)))
                lg = int(hg * 0.22)
                self.frames.put({
                    "type": "event", "seq": seq,
                    "t_ms": int((now - t0) * 1000),
                    "temp_c": 24.8 + 0.4 * random.random(),
                    "mask": 1, "channels": [0],
                    "first_ch": 0, "count": 1,
                    "hg": [hg], "lg": [lg],
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
#  Histogram
# ===================================================================
class Histogram:
    def __init__(self, bins=512, lo=0, hi=ADC_MAX):
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

    @property
    def width(self):
        return (self.hi - self.lo) / float(self.bins)

    def add(self, value):
        if value < self.lo or value > self.hi:
            self.overflow += 1
            return
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
#  Cross-calibration between the two gain paths
# ===================================================================
class GainCalib:
    """Reconstruct one energy scale from the HG and LG readings.

    HG and LG are two measurements of the SAME charge at different
    amplification, so adding them is meaningless. Instead the LG reading
    is rescaled into HG-equivalent units with a straight line fitted over
    the region where both paths are valid, and every event is taken from
    whichever path is still in range:

        E = HG                      while HG is below saturation
        E = a * LG + b              once HG has saturated

    That keeps HG resolution at low energy and LG headroom at high
    energy, on a single continuous axis.
    """

    def __init__(self):
        self.a = 0.0          # slope: HG counts per LG count
        self.b = 0.0          # offset
        self.sat = 3900       # HG treated as saturated at or above this
        self.lg_min = 150     # ignore LG samples sitting in the baseline
        self.valid = False
        self.n_fit = 0
        self.r2 = 0.0
        self.points = []      # (lg, hg) pairs kept for fitting
        self.max_points = 20000
        self.capped = False

    # -- data collection
    def add_pair(self, lg, hg):
        if len(self.points) < self.max_points:
            self.points.append((lg, hg))
        else:
            self.capped = True

    def clear(self):
        self.points.clear()
        self.valid = False
        self.n_fit = 0
        self.r2 = 0.0
        self.capped = False

    # -- fitting
    def fit(self):
        """Least squares HG = a*LG + b over the overlap region."""
        pts = [(lg, hg) for lg, hg in self.points
               if hg < self.sat and lg >= self.lg_min]
        n = len(pts)
        self.n_fit = n
        if n < 20:
            self.valid = False
            return False

        sx = sum(p[0] for p in pts)
        sy = sum(p[1] for p in pts)
        sxx = sum(p[0] * p[0] for p in pts)
        sxy = sum(p[0] * p[1] for p in pts)
        den = n * sxx - sx * sx
        if abs(den) < 1e-9:
            self.valid = False
            return False

        self.a = (n * sxy - sx * sy) / den
        self.b = (sy - self.a * sx) / n

        mean = sy / n
        ss_tot = sum((p[1] - mean) ** 2 for p in pts)
        ss_res = sum((p[1] - (self.a * p[0] + self.b)) ** 2 for p in pts)
        self.r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0

        # A negative or absurd slope means the overlap region was never
        # populated - refuse it rather than produce a nonsense spectrum.
        self.valid = 0.5 < self.a < 200.0
        return self.valid

    def set_manual(self, a, b):
        self.a = float(a)
        self.b = float(b)
        self.valid = self.a > 0
        self.r2 = 0.0
        self.n_fit = 0
        return self.valid

    # -- use
    def combined(self, hg, lg):
        """One value on the HG-equivalent scale."""
        if hg < self.sat:
            return float(hg)
        return self.a * lg + self.b

    def full_scale(self):
        """Top of the combined axis, in HG-equivalent units."""
        return max(ADC_MAX, int(self.a * ADC_MAX + self.b))


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

    def _hover(self, event):
        if not self.hist or not self.hist.total:
            return
        x0, y0, x1, y1 = self._plot_box()
        if not (x0 <= event.x <= x1 and y0 <= event.y <= y1):
            if self._tip:
                self.delete(self._tip)
                self._tip = None
            return
        frac = (event.x - x0) / max(1, (x1 - x0))
        idx = min(self.hist.bins - 1, int(frac * self.hist.bins))
        adc = self.hist.centre_of(idx)
        if self._tip:
            self.delete(self._tip)
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
            self._axis_labels(x0, y0, x1, y1, 0, 0, ADC_MAX)
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


def export_measurement(rec, path):
    """Write one measurement to .xlsx, or CSV if openpyxl is missing."""
    counts = rec["counts"]
    lo, hi, bins = rec["lo"], rec["hi"], rec["bins"]
    width = (hi - lo) / float(bins)

    if not HAVE_XLSX or path.lower().endswith(".csv"):
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            for k in ("started", "duration_s", "channel", "gain",
                      "total", "overflow", "temp_c", "note",
                      "calib_a", "calib_b", "calib_r2", "hg_saturation"):
                w.writerow([k, rec.get(k, "")])
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
    chart.title = f"Spectrum - ch{rec.get('channel')} {rec.get('gain')}"
    chart.y_axis.title = "counts"
    chart.x_axis.title = "ADC channel"
    chart.height, chart.width = 10, 22
    data = Reference(ws, min_col=5, min_row=1, max_row=len(counts) + 1)
    cats = Reference(ws, min_col=4, min_row=2, max_row=len(counts) + 1)
    chart.add_data(data, titles_from_data=True)
    chart.set_categories(cats)
    ws.add_chart(chart, "G2")

    meta = wb.create_sheet("Metadata")
    for k in ("started", "duration_s", "channel", "gain", "bins", "lo", "hi",
              "total", "overflow", "temp_c", "rate_cps", "note", "settings",
              "calib_a", "calib_b", "calib_r2", "calib_n", "hg_saturation"):
        meta.append([k, str(rec.get(k, ""))])
    if rec.get("gain") == "HG+LG":
        meta.append([])
        meta.append(["axis", "HG-equivalent ADC counts"])
        meta.append(["reconstruction", "E = HG below saturation, "
                                       "E = calib_a * LG + calib_b above"])

    wb.save(path)


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
            ("History", "The last six measurements,\n"
                        "with export to Excel", "history"),
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

    def on_show(self):
        self.refresh_ports()
        self.app.refresh_state_labels()


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

        self.v_indac = self._spin(ch, "inDac (0-255)", 0, 255, 128, r := r + 1)
        self.v_glg = self._spin(ch, "lgGain (0-15)", 0, 15, 4, r := r + 1)
        self.v_ghg = self._spin(ch, "hgGain (0-15)", 0, 15, 4, r := r + 1)
        self.v_tlg = self._spin(ch, "tauLG (0-15)", 0, 15, 14, r := r + 1)
        self.v_thg = self._spin(ch, "tauHG (0-15)", 0, 15, 14, r := r + 1)

        r += 1
        self.v_slg = tk.IntVar(value=1)
        self.v_shg = tk.IntVar(value=1)
        ttk.Checkbutton(ch, text="slow shaping LG", variable=self.v_slg)\
            .grid(row=r, column=0, sticky="w", pady=3)
        ttk.Checkbutton(ch, text="slow shaping HG", variable=self.v_shg)\
            .grid(row=r, column=1, sticky="w", pady=3)

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
        self.v_mhg = tk.IntVar(value=1)
        self.v_mlg = tk.IntVar(value=1)
        ttk.Checkbutton(gl, text="AMUX HG buffer", variable=self.v_mhg)\
            .grid(row=r, column=0, sticky="w", pady=3)
        ttk.Checkbutton(gl, text="AMUX LG buffer", variable=self.v_mlg)\
            .grid(row=r, column=1, sticky="w", pady=3)

        r += 1
        gbar = ttk.Frame(gl)
        gbar.grid(row=r, column=0, columnspan=2, sticky="w", pady=(10, 0))
        ttk.Button(gbar, text="Apply global", style="Accent.TButton",
                   command=self.apply_global).pack(side="left")
        ttk.Button(gbar, text="Preset CsI", command=self.preset_csi)\
            .pack(side="left", padx=4)
        ttk.Button(gbar, text="Defaults", command=lambda: self.cmd("defaults"))\
            .pack(side="left")
        ttk.Button(gbar, text="Push all", command=lambda: self.cmd("push"))\
            .pack(side="left", padx=4)

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
        if not self.app.link.connected:
            messagebox.showwarning("Offline", "Connect to the board first.")
            return
        self.app.link.send(line)
        self.log("> " + line)

    def send_raw(self):
        line = self.v_raw.get().strip()
        if line:
            self.cmd(line)
            self.v_raw.set("")

    def apply_channel(self):
        c = self.v_ch.get()
        self.cmd(f"ch {c} indac {self.v_indac.get()}")
        self.cmd(f"ch {c} gain {self.v_glg.get()} {self.v_ghg.get()}")
        self.cmd(f"ch {c} slow {self.v_slg.get()} {self.v_shg.get()}")
        self.cmd(f"ch {c} tau {self.v_tlg.get()} {self.v_thg.get()}")
        self.cmd(f"ch {c} patgain {self.v_pat.get()}")
        self.cmd(f"ch {c} trim {self.v_t1.get()} {self.v_t2.get()}")

    def enable(self, on):
        self.cmd(f"ch {self.v_ch.get()} {'on' if on else 'off'}")

    def dump(self):
        c = self.v_ch.get()
        if c == "all":
            messagebox.showinfo("Dump", "Pick a single channel to dump.")
            return
        self.cmd(f"ch {c} dump")

    def apply_global(self):
        self.cmd(f"th {self.v_d1.get()} {self.v_d2.get()} {self.v_dq.get()}")
        self.cmd(f"delay {self.v_dly.get()} {self.v_slp.get()}")
        self.cmd(f"trig {self.v_trig.get()}")
        self.cmd(f"hold {'ext' if self.v_hold_ext.get() else 'int'}")
        self.cmd(f"mux {self.v_mhg.get()} {self.v_mlg.get()}")

    def preset_csi(self):
        self.cmd("preset csi")
        # Mirror what the firmware preset does, so the form stays honest.
        self.v_slg.set(1); self.v_shg.set(1)
        self.v_tlg.set(14); self.v_thg.set(14)
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

        ttk.Label(ctl, text="Gain").grid(row=0, column=2, sticky="w")
        self.v_gain = tk.StringVar(value="HG")
        ttk.Combobox(ctl, textvariable=self.v_gain, width=9, state="readonly",
                     values=["HG", "LG", "HG+LG"])\
            .grid(row=0, column=3, padx=(6, 18))

        ttk.Label(ctl, text="Bins").grid(row=0, column=4, sticky="w")
        self.v_bins = tk.IntVar(value=512)
        ttk.Combobox(ctl, textvariable=self.v_bins, width=7, state="readonly",
                     values=[128, 256, 512, 1024, 2048])\
            .grid(row=0, column=5, padx=(6, 18))

        ttk.Label(ctl, text="Duration (s, 0 = manual)")\
            .grid(row=0, column=6, sticky="w")
        self.v_dur = tk.IntVar(value=60)
        ttk.Spinbox(ctl, from_=0, to=36000, textvariable=self.v_dur, width=8)\
            .grid(row=0, column=7, padx=6)

        # ---- HG+LG cross-calibration -------------------------------
        cal = ttk.LabelFrame(self, text="HG+LG cross-calibration", padding=12)
        cal.pack(fill="x", pady=(0, 8))

        ttk.Label(cal, text="HG-equivalent  =  a x LG + b").grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 6))

        ttk.Label(cal, text="a").grid(row=1, column=0, sticky="e")
        self.v_a = tk.StringVar(value="0.000")
        ttk.Entry(cal, textvariable=self.v_a, width=10)\
            .grid(row=1, column=1, padx=(4, 14))

        ttk.Label(cal, text="b").grid(row=1, column=2, sticky="e")
        self.v_b = tk.StringVar(value="0.0")
        ttk.Entry(cal, textvariable=self.v_b, width=10)\
            .grid(row=1, column=3, padx=(4, 14))

        ttk.Label(cal, text="HG saturates at").grid(row=1, column=4, sticky="e")
        self.v_sat = tk.IntVar(value=3900)
        ttk.Spinbox(cal, from_=1000, to=4095, textvariable=self.v_sat, width=7)\
            .grid(row=1, column=5, padx=(4, 14))

        ttk.Button(cal, text="Auto-calibrate", style="Accent.TButton",
                   command=self.auto_calibrate).grid(row=1, column=6, padx=4)
        ttk.Button(cal, text="Apply manual", command=self.apply_manual)\
            .grid(row=1, column=7, padx=4)
        ttk.Button(cal, text="Clear pairs", command=self.clear_pairs)\
            .grid(row=1, column=8, padx=4)

        self.lbl_cal = ttk.Label(cal, text="not calibrated", style="Bad.TLabel")
        self.lbl_cal.grid(row=2, column=0, columnspan=9, sticky="w", pady=(8, 0))

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
        ttk.Button(bar, text="Save measurement", command=self.save)\
            .pack(side="right")

        self.canvas = HistCanvas(self, height=340)
        self.canvas.pack(fill="both", expand=True, pady=12)

        stat = ttk.Frame(self)
        stat.pack(fill="x")
        self.lbl = ttk.Label(stat, text="idle", style="Sub.TLabel")
        self.lbl.pack(side="left")

        self.hist = Histogram()
        self.canvas.set_hist(self.hist)
        self.calib = GainCalib()
        self.running = False
        self.t_start = None
        self.last_temp = 0.0
        self._last_ui = 0.0

    # -- control
    def _axis_top(self):
        """Upper edge of the histogram axis for the selected mode."""
        if self.v_gain.get() == "HG+LG" and self.calib.valid:
            return self.calib.full_scale()
        return ADC_MAX

    def start(self):
        if not self.app.link.connected:
            messagebox.showwarning("Offline", "Connect to the board first.")
            return
        self.calib.sat = self.v_sat.get()
        self.hist.reset(self.v_bins.get(), 0, self._axis_top())
        self.canvas.set_hist(self.hist)
        self.running = True
        self.t_start = time.time()
        self.btn_start.configure(state="disabled")
        self.btn_stop.configure(state="normal")

    def stop(self):
        self.running = False
        self.btn_start.configure(state="normal")
        self.btn_stop.configure(state="disabled")

    def clear(self):
        self.hist.reset()
        self.canvas.redraw()
        self.lbl.configure(text="cleared")

    # -- cross-calibration
    def _show_cal(self):
        if self.calib.valid:
            extra = f", R2 {self.calib.r2:.4f}" if self.calib.n_fit else " (manual)"
            capped = "  [pair buffer full]" if self.calib.capped else ""
            self.lbl_cal.configure(
                style="Ok.TLabel",
                text=f"calibrated: a = {self.calib.a:.4f}, b = {self.calib.b:.1f}"
                     f"  |  {self.calib.n_fit} points{extra}"
                     f"  |  full scale {self.calib.full_scale()} HG-equivalent{capped}")
        else:
            self.lbl_cal.configure(
                style="Bad.TLabel",
                text=f"not calibrated - {len(self.calib.points)} pairs collected, "
                     f"{self.calib.n_fit} usable in the overlap region")
        self.v_a.set(f"{self.calib.a:.4f}")
        self.v_b.set(f"{self.calib.b:.1f}")

    def auto_calibrate(self):
        self.calib.sat = self.v_sat.get()
        if not self.calib.fit():
            self._show_cal()
            messagebox.showwarning(
                "Not enough overlap",
                "The fit needs at least 20 events where HG is below "
                "saturation and LG is above its baseline.\n\n"
                "Acquire in HG+LG mode with a source that produces both "
                "small and large pulses, then try again.")
            return
        self.rebuild_from_pairs()
        self._show_cal()

    def apply_manual(self):
        try:
            a = float(self.v_a.get())
            b = float(self.v_b.get())
        except ValueError:
            messagebox.showerror("Bad value", "a and b must be numbers.")
            return
        self.calib.sat = self.v_sat.get()
        if not self.calib.set_manual(a, b):
            messagebox.showerror("Bad value", "a must be positive.")
            return
        self.rebuild_from_pairs()
        self._show_cal()

    def clear_pairs(self):
        self.calib.clear()
        self._show_cal()

    def rebuild_from_pairs(self):
        """Re-fill the histogram from the stored pairs.

        Needed because the calibration usually arrives after the data:
        the spectrum has to be recomputed on the new energy scale.
        """
        if not self.calib.valid or not self.calib.points:
            return
        self.hist.reset(self.v_bins.get(), 0, self.calib.full_scale())
        for lg, hg in self.calib.points:
            self.hist.add(self.calib.combined(hg, lg))
        self.canvas.set_hist(self.hist)
        self.canvas.redraw()

    def feed(self, frame):
        """Called by the app pump for every decoded frame."""
        if frame["type"] == "status":
            self.last_temp = frame.get("temp_c", 0.0)
            return
        if frame["type"] != "event" or not self.running:
            return

        # The selection can be sparse, so look the channel up rather
        # than deriving its position from a first/count pair.
        ch = self.v_ch.get()
        try:
            idx = frame["channels"].index(ch)
        except (KeyError, ValueError):
            return

        hg = frame["hg"][idx]
        lg = frame["lg"][idx]
        mode = self.v_gain.get()

        if mode == "HG":
            self.hist.add(hg)
        elif mode == "LG":
            self.hist.add(lg)
        else:
            # Keep every pair so a calibration can be fitted from data
            # already taken, then histogram only once it is trustworthy.
            self.calib.add_pair(lg, hg)
            if self.calib.valid:
                self.hist.add(self.calib.combined(hg, lg))

    def tick(self):
        if not self.running:
            return
        elapsed = time.time() - self.t_start
        dur = self.v_dur.get()
        if dur and elapsed >= dur:
            self.stop()
            self.lbl.configure(text=f"finished: {self.hist.total} counts "
                                    f"in {elapsed:.1f} s")
            self.canvas.redraw()
            return
        now = time.time()
        if now - self._last_ui > 0.25:          # keep redraws cheap
            self._last_ui = now
            self.canvas.redraw()
            rate = self.hist.total / elapsed if elapsed > 0 else 0
            pk = self.hist.peak_bin()
            pk_txt = f"peak ADC {self.hist.centre_of(pk):.0f}" if pk is not None else "-"
            extra = ""
            if self.v_gain.get() == "HG+LG" and not self.calib.valid:
                extra = (f" | COLLECTING PAIRS ({len(self.calib.points)}) - "
                         f"press Auto-calibrate")
            self.lbl.configure(
                text=f"running {elapsed:6.1f} s | {self.hist.total} counts | "
                     f"{rate:6.1f} cps | {pk_txt} | T {self.last_temp:.1f} C | "
                     f"overflow {self.hist.overflow}{extra}")

    def save(self):
        if self.hist.total == 0:
            messagebox.showinfo("Nothing to save", "Acquire some data first.")
            return
        if self.v_gain.get() == "HG+LG" and not self.calib.valid:
            messagebox.showwarning(
                "Not calibrated",
                "A combined spectrum needs a calibration before it means "
                "anything. Press Auto-calibrate first.")
            return
        elapsed = (time.time() - self.t_start) if self.t_start else 0.0
        rec = {
            "started": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "duration_s": round(elapsed, 1),
            "channel": self.v_ch.get(),
            "gain": self.v_gain.get(),
            "bins": self.hist.bins,
            "lo": self.hist.lo,
            "hi": self.hist.hi,
            "counts": self.hist.counts[:],
            "total": self.hist.total,
            "overflow": self.hist.overflow,
            "temp_c": round(self.last_temp, 2),
            "rate_cps": round(self.hist.total / elapsed, 2) if elapsed else 0,
            "note": "simulator" if self.app.link.sim else "",
            "settings": "",
            # Without these the combined spectrum cannot be interpreted
            # later, since its x axis is in HG-equivalent units.
            "calib_a": round(self.calib.a, 6) if self.v_gain.get() == "HG+LG" else "",
            "calib_b": round(self.calib.b, 3) if self.v_gain.get() == "HG+LG" else "",
            "calib_r2": round(self.calib.r2, 6) if self.v_gain.get() == "HG+LG" else "",
            "calib_n": self.calib.n_fit if self.v_gain.get() == "HG+LG" else "",
            "hg_saturation": self.calib.sat if self.v_gain.get() == "HG+LG" else "",
        }
        save_measurement(rec)
        messagebox.showinfo("Saved", "Measurement stored in History.")
        self.app.pages["history"].reload()


class PageHistory(Page):
    title = "History"

    def build(self):
        top = ttk.Frame(self)
        top.pack(fill="x")
        ttk.Button(top, text="< Main", command=lambda: self.app.show("main"))\
            .pack(side="left")
        ttk.Label(top, text="Last six measurements", style="H1.TLabel")\
            .pack(side="left", padx=12)
        ttk.Button(top, text="Refresh", command=self.reload).pack(side="right")

        cols = ("started", "duration_s", "channel", "gain", "total",
                "rate_cps", "temp_c", "calib_a")
        self.tree = ttk.Treeview(self, columns=cols, show="headings", height=7)
        widths = (160, 80, 70, 70, 90, 90, 80, 90)
        for c, w in zip(cols, widths):
            self.tree.heading(c, text=c.replace("_", " "))
            self.tree.column(c, width=w, anchor="center")
        self.tree.pack(fill="x", pady=12)
        self.tree.bind("<<TreeviewSelect>>", lambda _e: self.preview())

        self.canvas = HistCanvas(self, height=280)
        self.canvas.pack(fill="both", expand=True)

        bar = ttk.Frame(self)
        bar.pack(fill="x", pady=10)
        ttk.Button(bar, text="Export to Excel", style="Accent.TButton",
                   command=self.export).pack(side="left")
        ttk.Button(bar, text="Delete", command=self.delete)\
            .pack(side="left", padx=6)
        self.lbl = ttk.Label(bar, text="", style="Sub.TLabel")
        self.lbl.pack(side="left", padx=16)

        self.records = []
        self.reload()

    def reload(self):
        self.records = load_measurements()
        for iid in self.tree.get_children():
            self.tree.delete(iid)
        for i, r in enumerate(self.records):
            self.tree.insert("", "end", iid=str(i), values=(
                r.get("started", ""), r.get("duration_s", ""),
                r.get("channel", ""), r.get("gain", ""),
                r.get("total", ""), r.get("rate_cps", ""), r.get("temp_c", ""),
                r.get("calib_a", "")))
        self.canvas.set_hist(None)
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
    to this PC over the USB virtual COM port.

ORDER OF OPERATIONS
    1. Connect
       Main page, pick the port, press Connect. If the board is not
       built yet, press Simulator to explore the interface with
       generated data.

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

    3. Measure
       Choose the channel and gain path, set the bin count and a
       duration, then Start. The spectrum builds up live. Save
       Measurement when it looks reasonable.

    4. History
       The six most recent measurements are kept. Select one to preview
       it, then Export to Excel.

HIGH GAIN, LOW GAIN, OR BOTH
    HG has more resolution but saturates earlier; LG covers the full
    dynamic range with coarser steps. Start with HG for low energy
    sources and switch to LG if the peak runs into the top of the ADC
    range.

    HG+LG combines the two into one continuous energy scale. Note that
    this is NOT an addition: both paths measure the same charge at
    different amplification, so adding them would be meaningless. What
    happens instead is a gain switch:

        E = HG                        while HG is below saturation
        E = a x LG + b                once HG has saturated

    The line a, b is fitted over the overlap region, where HG is still
    in range and LG is already above its baseline. The result carries HG
    resolution at low energy and LG headroom at high energy, on one axis
    expressed in HG-equivalent counts.

HOW TO CALIBRATE HG+LG
    1. Select HG+LG and run a short acquisition, ideally with a source
       that produces both small and large pulses. Nothing is histogrammed
       yet - the software is collecting (LG, HG) pairs.
    2. Press Auto-calibrate. The fit needs at least 20 events inside the
       overlap region. The spectrum is then rebuilt on the new scale.
    3. Check R2 in the status line. Above about 0.99 is healthy; a low
       value usually means the overlap region was never populated, so
       either lower the threshold or use a stronger source.
    4. Once a and b look stable you can type them in by hand on later
       runs with Apply manual, as long as the gain settings have not
       changed.

    The calibration is stored with every saved measurement, because
    without a and b the combined x axis cannot be interpreted later.

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

TEMPERATURE
    Both the SiPM gain and the crystal light yield drift with
    temperature: roughly +21 mV per degree on the SiPM breakdown
    voltage, and about -0.3 percent per degree of light output. The
    temperature is logged with each measurement so the peak position
    can be corrected afterwards.

TROUBLESHOOTING
    No events at all
        Check that the channel is enabled, that the AMUX buffers are
        on, and that the threshold is not far above the pulse height.
    Enormous count rate
        The threshold is in the noise. Raise DAC1.
    Peak sitting at the very top bin
        The ADC is saturating. Switch to LG, or lower hgGain.
    Dropped frames in the status line
        The USB link cannot keep up. Reduce the number of enabled
        channels or the event rate.

FILES
    Measurements are kept as JSON under the measurements folder next to
    this script. Only the newest six are retained.
"""

    def build(self):
        top = ttk.Frame(self)
        top.pack(fill="x")
        ttk.Button(top, text="< Main", command=lambda: self.app.show("main"))\
            .pack(side="left")
        ttk.Label(top, text="Instructions", style="H1.TLabel")\
            .pack(side="left", padx=12)

        wrap = ttk.Frame(self)
        wrap.pack(fill="both", expand=True, pady=12)
        txt = tk.Text(wrap, wrap="word", relief="flat", bg=C_PANEL,
                      fg=C_TEXT, font=("Consolas", 10), padx=14, pady=12)
        txt.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(wrap, command=txt.yview)
        sb.pack(side="right", fill="y")
        txt.configure(yscrollcommand=sb.set)
        txt.insert("1.0", self.TEXT)
        txt.configure(state="disabled")


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
        self._style()

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

        self.status = ttk.Label(self, text="offline", style="Sub.TLabel",
                                anchor="w", padding=(14, 6))
        self.status.pack(fill="x", side="bottom")

        self.show("main")
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
        st.configure("Bad.TLabel", foreground=C_BAD)
        st.configure("TButton", padding=(12, 6))
        st.configure("Accent.TButton", background=C_ACCENT, foreground="white")
        st.map("Accent.TButton", background=[("active", C_ACCENT_DK)])
        st.configure("Treeview", background=C_PANEL, fieldbackground=C_PANEL,
                     rowheight=24)

    # -- navigation
    def show(self, key):
        pg = self.pages[key]
        pg.tkraise()
        pg.on_show()

    # -- connection state
    def set_state(self, connected, where):
        self.status.configure(
            text=f"connected to {where}" if connected else "offline")
        self.refresh_state_labels()

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
            self.pages["measure"].feed(frame)

        while True:
            try:
                self.pages["settings"].log(self.link.text.get_nowait())
            except queue.Empty:
                break

        self.pages["measure"].tick()
        self.after(40, self.pump)

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
