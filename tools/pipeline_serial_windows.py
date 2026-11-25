"""Interactive Windows COM terminal for the 115200-baud pipeline firmware."""
import argparse
from contextlib import nullcontext
import msvcrt
import sys
import time

import serial

from pipeline_serial import extract_lines, stalled


def monitor(device, log):
    pending = b""
    last_rx = last_warn = time.monotonic()
    print("O cart zero | Z encoder zero | E stats | C source | S swing | A balance | X/space stop | q quit")
    try:
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getwch()
                if key in ("\x00", "\xe0"):
                    msvcrt.getwch()  # consume the second byte of a function/arrow key
                elif key in ("q", "Q", "\x03"):
                    break
                elif key.isascii():
                    device.write(key.encode("ascii"))

            chunk = device.read(min(device.in_waiting or 1, 8192))
            now = time.monotonic()
            if chunk:
                last_rx = now
                lines, separator, tail = (pending + chunk).rpartition(b"\n")
                pending = tail[-8192:]
                if separator:
                    for line in extract_lines(lines):
                        entry = f"{time.strftime('%H:%M:%S')} {line}"
                        print(entry)
                        if log:
                            print(entry, file=log, flush=True)
            elif stalled(now, last_rx, last_warn):
                last_warn = now
                print("No ESP32 data. If USB is down, stop with the driver power switch.", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            device.write(b"X")
        except OSError:
            print("Stop command failed. Use the driver power switch.", file=sys.stderr)
            raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="ESP32 COM port, e.g. COM3")
    parser.add_argument("log", nargs="?", help="new text log")
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error("Use an interactive PowerShell or VS Code terminal.")
    try:
        with serial.Serial(args.port, 115200, timeout=0.02, write_timeout=0.5) as device:
            with open(args.log, "x", encoding="utf-8") if args.log else nullcontext() as log:
                monitor(device, log)
    except OSError as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
