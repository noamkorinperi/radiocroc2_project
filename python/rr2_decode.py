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

The command interface shares this one UART and its replies are plain
text with no framing, so they show up as gaps between frames. Decoder
keeps those bytes instead of discarding them - see take_text().
"""

import argparse
import csv
import struct
import sys

SYNC = b"\xA5\x5A"

# The board streams over USART3, which is wired to the ST-Link's Virtual
# COM Port - one USB cable carries both the debugger and the data. Unlike
# the old USB CDC link, this is a real UART, so the baud rate is no longer
# cosmetic: get it wrong and you get garbage, not silence.
#
# Must match RR2_LINK_BAUD in Core/Inc/usart.h.
LINK_BAUD = 921600

# ST-Link composite device. 0x374B is the V2-1 fitted to a NUCLEO-F722ZE;
# the others are STLINK-V3 variants, listed so the same script works on a
# "-Q" board without edits.
STLINK_VID = 0x0483
STLINK_PIDS = (0x374B, 0x374E, 0x374F, 0x3753, 0x3754)


def _comports():
    """pyserial's port list, or None if pyserial is not installed.

    None and [] mean different things - "you need to pip install
    something" versus "plug the board in" - and the callers say so.
    """
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    return list(list_ports.comports())


def _is_stlink(p):
    return p.vid == STLINK_VID and p.pid in STLINK_PIDS


def find_link_port():
    """Device name of the ST-Link VCP, or None if it is not there.

    Saves hunting through Device Manager for the COM number, which
    changes whenever the board lands on a different USB socket.
    """
    for p in _comports() or ():
        if _is_stlink(p):
            return p.device
    return None


def describe_ports():
    """[(device, description, is_stlink)], or None if pyserial is absent."""
    ports = _comports()
    if ports is None:
        return None
    return [(p.device, p.description or "?", _is_stlink(p)) for p in ports]
FRAME_EVENT = 1
FRAME_STATUS = 2

# Longest payload the firmware can emit: a 64-channel event is 20 header
# bytes plus 4 per channel. Status frames are 35. plen is a uint16, so a
# false sync can claim up to 65535 - bounding it here is what stops the
# parser stalling on two bytes of data that happened to read A5 5A.
MAX_PAYLOAD = 20 + 4 * 64          # 276

# Cap on the recovered-text buffer. A link that is pure noise must not be
# able to grow it without bound.
MAX_SKIPPED = 8192

# _try_one() has three outcomes, and the caller has to tell them apart:
# a frame, "the buffer is short - wait", and "that candidate was junk -
# rescan now". Conflating the last two is what used to make a single bad
# CRC stop the parse for the rest of the chunk.
NEED_MORE = object()
RETRY = object()


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    """CRC16-CCITT, polynomial 0x1021 - must match usb_stream.c."""
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Decoder:
    """Incremental parser - feed it arbitrary byte chunks.

    Binary frames come back from feed(). Everything else on the wire is
    kept, not dropped: the firmware's command replies travel as bare
    text on the same UART and have no framing of their own, so the only
    way to see them is to hold on to what the frame parser rejected.
    Collect them with take_text().
    """

    def __init__(self):
        self.buf = bytearray()
        self.skipped = bytearray()     # non-frame bytes, awaiting take_text
        self.bad_crc = 0
        self.resyncs = 0

    def feed(self, chunk: bytes):
        """Yield every frame completed by this chunk.

        Non-frame bytes are moved aside for take_text(), which the caller
        should drain alongside this - otherwise replies and ERR: lines go
        unseen.
        """
        self.buf.extend(chunk)
        while True:
            frame = self._try_one()
            if frame is NEED_MORE:
                return
            if frame is not RETRY:
                yield frame

    def take_text(self):
        """Complete printable lines recovered from the non-frame bytes.

        The firmware terminates every reply with CRLF, so split on the
        newline and leave a partial tail for the next call. A rejected
        frame lands in the same buffer, so runs that are not mostly
        printable are discarded rather than shown as binary confetti.
        """
        out = []
        while True:
            nl = self.skipped.find(b"\n")
            if nl < 0:
                break
            raw = bytes(self.skipped[:nl])
            del self.skipped[:nl + 1]
            line = raw.decode("ascii", "replace").strip()
            if line and sum(c.isprintable() for c in line) >= 0.8 * len(line):
                out.append(line)
        return out

    def _skip(self, n):
        """Move n leading bytes out of the frame buffer, keeping them."""
        self.skipped.extend(self.buf[:n])
        del self.buf[:n]
        if len(self.skipped) > MAX_SKIPPED:
            del self.skipped[:-MAX_SKIPPED]

    def _try_one(self):
        # Find the sync word. Anything ahead of it is not frame data -
        # most often a text reply - so it is set aside, not discarded.
        idx = self.buf.find(SYNC)
        if idx < 0:
            # A sync can only straddle the chunk boundary if the last
            # byte is its first half, so hold that one back and release
            # the rest. Keeping the tail unconditionally would strand a
            # reply whose newline lands exactly on the boundary.
            keep = 1 if self.buf[-1:] == SYNC[:1] else 0
            if len(self.buf) > keep:
                self._skip(len(self.buf) - keep)
            return NEED_MORE
        if idx > 0:
            self.resyncs += 1
            self._skip(idx)

        if len(self.buf) < 5:
            return NEED_MORE

        ftype = self.buf[2]
        plen = struct.unpack_from("<H", self.buf, 3)[0]

        # Judge the candidate on its header before trusting its length.
        # Waiting on a bogus plen would park the parser for up to 64 kB
        # while real frames queue up behind it.
        if plen > MAX_PAYLOAD or ftype not in (FRAME_EVENT, FRAME_STATUS):
            self._skip(1)
            return RETRY

        total = 5 + plen + 2
        if len(self.buf) < total:
            return NEED_MORE

        body = bytes(self.buf[2:5 + plen])           # type + length + payload
        got = struct.unpack_from("<H", self.buf, 5 + plen)[0]
        want = crc16_ccitt(body)

        if got != want:
            self.bad_crc += 1
            # Drop the sync byte and rescan - a false sync inside data.
            self._skip(1)
            return RETRY

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
    ap = argparse.ArgumentParser(
        description="Decode the RADIOROC2 stream from the ST-Link VCP.")
    ap.add_argument("port", nargs="?",
                    help="serial port, e.g. COM3 or /dev/ttyACM0. "
                         "Omit it and the ST-Link is found automatically.")
    ap.add_argument("--file", help="read from a binary capture instead of a port")
    ap.add_argument("--csv", help="write decoded events to this CSV file")
    ap.add_argument("--baud", type=int, default=LINK_BAUD,
                    help=f"link speed, default {LINK_BAUD} "
                         f"(must match RR2_LINK_BAUD in usart.h)")
    ap.add_argument("--list", action="store_true",
                    help="list serial ports and exit")
    args = ap.parse_args()

    if args.list:
        ports = describe_ports()
        if ports is None:
            print("pyserial is not installed:  pip install pyserial")
        elif not ports:
            print("no serial ports found - is the board plugged in?")
        else:
            for dev, desc, is_link in ports:
                print(f"  {dev:<10} {desc}" + ("   <- ST-Link" if is_link else ""))
        return

    if not args.port and not args.file:
        if _comports() is None:
            ap.error("pyserial is not installed:  pip install pyserial")
        args.port = find_link_port()
        if not args.port:
            ap.error("no ST-Link found - name a port, or use --file. "
                     "Run with --list to see what is connected.")
        print(f"auto-detected ST-Link on {args.port}")

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

    def pump(chunk):
        """One chunk in, both kinds of output out.

        The command replies are not framed and used to be thrown away
        with the rest of the inter-frame bytes, which hid every ERR:
        the firmware sent.
        """
        for frame in dec.feed(chunk):
            handle(frame)
        for line in dec.take_text():
            print(f"  | {line}")

    try:
        if args.file:
            with open(args.file, "rb") as fh:
                while True:
                    chunk = fh.read(4096)
                    if not chunk:
                        break
                    pump(chunk)
        else:
            try:
                import serial  # pyserial
            except ImportError:
                sys.exit("pyserial is required for live capture:  pip install pyserial")

            with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
                print(f"listening on {args.port} at {args.baud} baud"
                      f" - Ctrl-C to stop")
                while True:
                    chunk = ser.read(4096)
                    if chunk:
                        pump(chunk)
    except KeyboardInterrupt:
        pass
    finally:
        if csv_fh:
            csv_fh.close()
        print(f"\ndone. bad CRC: {dec.bad_crc}, resyncs: {dec.resyncs}")


if __name__ == "__main__":
    main()
