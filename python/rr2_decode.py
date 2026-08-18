#!/usr/bin/env python3
"""
rr2_decode.py - host-side decoder for the RADIOROC2 USB CDC stream.

Usage:
    python rr2_decode.py /dev/ttyACM0            # Linux / macOS
    python rr2_decode.py COM5                    # Windows
    python rr2_decode.py --file dump.bin         # replay a capture
    python rr2_decode.py COM5 --csv out.csv      # also write CSV

Frame layout (little endian), mirroring usb_stream.h:

    off  size  field
    0    2     sync 0xA5 0x5A
    2    1     frame type: 1 = event, 2 = status
    3    2     payload length
    5    N     payload
    5+N  2     CRC16-CCITT over bytes [2 .. 4+N]

Event payload:
    0   4   sequence number
    4   4   timestamp, ms since boot
    8   4   temperature, milli-Celsius (signed)
    12  1   first channel
    13  1   channel count
    14  2*count  high-gain codes
    ..  2*count  low-gain codes
"""

import argparse
import csv
import struct
import sys

SYNC = b"\xA5\x5A"
FRAME_EVENT = 1
FRAME_STATUS = 2


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    """CRC16-CCITT, polynomial 0x1021 - must match usb_stream.c."""
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Decoder:
    """Incremental parser - feed it arbitrary byte chunks."""

    def __init__(self):
        self.buf = bytearray()
        self.bad_crc = 0
        self.resyncs = 0

    def feed(self, chunk: bytes):
        self.buf.extend(chunk)
        while True:
            frame = self._try_one()
            if frame is None:
                return
            yield frame

    def _try_one(self):
        # Find the sync word, discarding anything before it.
        idx = self.buf.find(SYNC)
        if idx < 0:
            # Keep one byte in case a sync straddles the chunk boundary.
            if len(self.buf) > 1:
                del self.buf[:-1]
            return None
        if idx > 0:
            self.resyncs += 1
            del self.buf[:idx]

        if len(self.buf) < 5:
            return None

        ftype = self.buf[2]
        plen = struct.unpack_from("<H", self.buf, 3)[0]
        total = 5 + plen + 2
        if len(self.buf) < total:
            return None

        body = bytes(self.buf[2:5 + plen])           # type + length + payload
        got = struct.unpack_from("<H", self.buf, 5 + plen)[0]
        want = crc16_ccitt(body)

        if got != want:
            self.bad_crc += 1
            # Drop the sync byte and rescan - a false sync inside data.
            del self.buf[:1]
            return None

        payload = bytes(self.buf[5:5 + plen])
        del self.buf[:total]
        return self._parse(ftype, payload)

    @staticmethod
    def _parse(ftype: int, p: bytes):
        if ftype == FRAME_EVENT:
            seq, ts, temp, first, count = struct.unpack_from("<IIiBB", p, 0)
            off = 14
            hg = list(struct.unpack_from(f"<{count}H", p, off))
            off += 2 * count
            lg = list(struct.unpack_from(f"<{count}H", p, off))
            return {
                "type": "event",
                "seq": seq,
                "t_ms": ts,
                "temp_c": temp / 1000.0,
                "first_ch": first,
                "count": count,
                "hg": hg,
                "lg": lg,
            }

        if ftype == FRAME_STATUS:
            (uptime, trig, ok, bad, drop, temp,
             flags, cfg_st, rd_st) = struct.unpack_from("<IIIIIiBBB", p, 0)
            return {
                "type": "status",
                "uptime_ms": uptime,
                "triggers": trig,
                "events_ok": ok,
                "events_bad": bad,
                "dropped": drop,
                "temp_c": temp / 1000.0,
                "rr2_online": bool(flags & 0x01),
                "temp_online": bool(flags & 0x02),
                "timing_ok": bool(flags & 0x04),
                "cfg_status": cfg_st,
                "read_status": rd_st,
            }

        return {"type": "unknown", "ftype": ftype, "raw": p}


def main():
    ap = argparse.ArgumentParser(description="Decode the RADIOROC2 USB stream.")
    ap.add_argument("port", nargs="?", help="serial port, e.g. /dev/ttyACM0 or COM5")
    ap.add_argument("--file", help="read from a binary capture instead of a port")
    ap.add_argument("--csv", help="write decoded events to this CSV file")
    ap.add_argument("--baud", type=int, default=115200,
                    help="ignored by CDC, kept for pyserial compatibility")
    args = ap.parse_args()

    if not args.port and not args.file:
        ap.error("give a serial port or --file")

    dec = Decoder()
    writer = None
    csv_fh = None

    if args.csv:
        csv_fh = open(args.csv, "w", newline="")
        writer = csv.writer(csv_fh)
        writer.writerow(["seq", "t_ms", "temp_c", "ch", "hg", "lg"])

    def handle(frame):
        if frame["type"] == "event":
            peak = max(frame["hg"]) if frame["hg"] else 0
            print(f"EVT seq={frame['seq']:<6} t={frame['t_ms']/1000:8.3f}s "
                  f"T={frame['temp_c']:6.2f}C ch={frame['first_ch']}+{frame['count']} "
                  f"maxHG={peak}")
            if writer:
                for i in range(frame["count"]):
                    writer.writerow([frame["seq"], frame["t_ms"], frame["temp_c"],
                                     frame["first_ch"] + i,
                                     frame["hg"][i], frame["lg"][i]])
        elif frame["type"] == "status":
            print(f"STA up={frame['uptime_ms']/1000:.1f}s trig={frame['triggers']} "
                  f"ok={frame['events_ok']} bad={frame['events_bad']} "
                  f"drop={frame['dropped']} T={frame['temp_c']:.2f}C "
                  f"asic={'Y' if frame['rr2_online'] else 'N'} "
                  f"tmp={'Y' if frame['temp_online'] else 'N'} "
                  f"dwt={'Y' if frame['timing_ok'] else 'N'}")

    try:
        if args.file:
            with open(args.file, "rb") as fh:
                while True:
                    chunk = fh.read(4096)
                    if not chunk:
                        break
                    for frame in dec.feed(chunk):
                        handle(frame)
        else:
            try:
                import serial  # pyserial
            except ImportError:
                sys.exit("pyserial is required for live capture:  pip install pyserial")

            with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
                print(f"listening on {args.port} - Ctrl-C to stop")
                while True:
                    chunk = ser.read(4096)
                    if chunk:
                        for frame in dec.feed(chunk):
                            handle(frame)
    except KeyboardInterrupt:
        pass
    finally:
        if csv_fh:
            csv_fh.close()
        print(f"\ndone. bad CRC: {dec.bad_crc}, resyncs: {dec.resyncs}")


if __name__ == "__main__":
    main()
