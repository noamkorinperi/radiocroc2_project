#!/usr/bin/env python3
"""
rr2_i2ctest.py - run the firmware's Slow Control link test and show the result.

Sends the `i2ctest` command over the ST-Link VCP and prints what comes
back, then summarises the verdict. Nothing else: no GUI, no plotting, no
Eclipse plugin. If the board is flashed and plugged in, this works.

    py rr2_i2ctest.py            # find the ST-Link automatically
    py rr2_i2ctest.py --list     # show the serial ports and which is which
    py rr2_i2ctest.py --port COM7

Port detection and the link speed are imported from rr2_decode.py rather
than repeated here, so there is still only one place that knows how to
find the board.

A note on the mixed stream: the firmware's event and status frames are
binary and share this link with the text replies. They cannot actually
interleave with a test run - USBCmd_Task() calls straight into the test,
so the main loop is inside it and no status frame can be queued while it
runs - but anything binary that was already in flight is filtered out
here rather than being printed as mojibake.
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is missing:  py -m pip install pyserial")

try:
    from rr2_decode import find_link_port, describe_ports, LINK_BAUD
except ImportError:
    sys.exit("rr2_decode.py must sit next to this file")


# The test runs two passes at worst, each with a full 112-address scan.
# Three seconds is the measured worst case; ten is the giving-up point.
TIMEOUT_S = 10.0

VERDICT_HELP = {
    "OK":            "writes and reads both round-trip - the link is good",
    "WRITE_ONLY":    "the ASIC ACKs writes but reads do not come back",
    "NO_DEVICE":     "nothing ACKed at any address, at either speed",
    "WRONG_ID":      "something answered, but not on eight aligned addresses",
    "SCL_STUCK_LOW": "SCL is held low - the bus cannot be used",
    "SDA_STUCK_LOW": "SDA is held low - a slave is jamming the bus",
    "BUS_NOT_READY": "pads or the I2C peripheral are not configured",
}


def printable(line: str) -> bool:
    """True for a line that is plain text, not a fragment of a binary frame."""
    return bool(line) and all((32 <= ord(c) < 127) or c == "\t" for c in line)


def main():
    ap = argparse.ArgumentParser(description="run the firmware's i2ctest")
    ap.add_argument("--port", help="serial port; default is autodetect")
    ap.add_argument("--baud", type=int, default=LINK_BAUD,
                    help=f"link speed, default {LINK_BAUD}")
    ap.add_argument("--list", action="store_true",
                    help="list serial ports and exit")
    args = ap.parse_args()

    if args.list:
        print(describe_ports())
        return 0

    port = args.port or find_link_port()
    if not port:
        print("No ST-Link VCP found. Ports seen:\n")
        print(describe_ports())
        print("\nPass one explicitly with --port COMx")
        return 1

    print(f"port {port} @ {args.baud}\n")

    with serial.Serial(port, args.baud, timeout=0.2) as ser:
        # Anything already queued (a status frame, a partial line) would
        # otherwise be read as part of the reply.
        time.sleep(0.1)
        ser.reset_input_buffer()

        ser.write(b"i2ctest\r\n")
        ser.flush()

        fields = {}
        acks = []
        deadline = time.time() + TIMEOUT_S
        buf = b""

        while time.time() < deadline:
            buf += ser.read(256)

            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip("\r\x00 ")

                if not printable(line):
                    continue          # a binary frame that was in flight

                print("   ", line)

                if "=" in line and line.startswith("i2ctest."):
                    key, _, value = line[len("i2ctest."):].partition("=")
                    if key == "ack_addr":
                        acks.append(value)
                    else:
                        fields[key] = value

            if fields.get("done") == "1":
                break
        else:
            print("\nTimed out. The board did not answer.")
            print("Check that it is running the new firmware, that the baud")
            print("matches RR2_LINK_BAUD, and that nothing else holds the port.")
            return 1

    # ---- summary -------------------------------------------------------
    name = fields.get("verdict_name", "?")

    print("\n" + "-" * 56)
    print(f"  verdict   {name}   {VERDICT_HELP.get(name, '')}")

    if "pass" in fields:
        print(f"  pass      {fields['pass']}"
              f"   clk_sm={fields.get('clk_sm_hz', '?')} Hz")
    if acks:
        print(f"  answered  {len(acks)} addresses: {', '.join(acks)}")
    if "chip_id" in fields and fields["chip_id"] != "-1":
        print(f"  chip id   {fields['chip_id']}")
    if "roundtrips" in fields:
        n = fields["roundtrips"]
        print(f"  writes    {fields.get('w_ok', '?')}/{n}")
        print(f"  reads     {fields.get('r_ok', '?')}/{n}")
        print(f"  verified  {fields.get('match', '?')}/{n}")
    print("-" * 56)

    return 0 if name == "OK" else 2


if __name__ == "__main__":
    sys.exit(main())
