#!/usr/bin/env python3
"""Minimal terminal for the pipeline / camera_pipeline firmware (115200, no extra packages).

  python tools/pipeline_serial.py <port> [log_file]

The firmware also streams binary telemetry frames for the GUI, which turn a normal serial monitor into noise.
This tool shows only the text messages ('# SH ...' shadow stats, 'OK: ...', 'ARMED', 'DISARMED', 'REFUSED', ...)
and sends every key you type to the firmware:
    Z  zero the encoder (pendulum hanging still)     E  shadow camera stats on/off
    S  swing-up      A  balance (hold the pendulum upright first)      X or space  stop
    q  stop and quit (Ctrl+C does the same)
Do not run it together with the GUI or another monitor on the same port. Opening the port can reset the ESP32,
so press Z again afterwards. There is no keep-alive in this firmware: if this tool dies while the cart is
running, the controller keeps going until it trips by itself; use X / space / the driver power switch to stop.
"""
import os
import re
import select
import sys
import termios
import time
import tty

TEXT = re.compile(rb"(?:# SH[^\r\n]*|OK:[^\r\n]*|ARMED[^\r\n]*|DISARMED[^\r\n]*|CATCH[^\r\n]*|[A-Z ]*REFUSED[^\r\n]*|===[^\r\n]*)")


def extract_lines(chunk):
    """Text messages inside a chunk that may also hold binary telemetry frames."""
    out = []
    for m in TEXT.finditer(chunk):
        s = m.group(0)
        # a binary frame can follow the message directly: keep the printable ASCII part only
        end = len(s)
        for i, b in enumerate(s):
            if b < 0x20 or b > 0x7E:
                end = i
                break
        out.append(s[:end].decode("ascii"))
    return out


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    port = sys.argv[1]
    log = open(sys.argv[2], "w") if len(sys.argv) > 2 else None
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = attrs[1] = attrs[3] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[4] = attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print("keys: Z zero | E shadow stats | S swing-up | A balance | X/space stop | q quit\r")
    buf = b""
    try:
        while True:
            r, _, _ = select.select([fd, sys.stdin], [], [], 0.05)
            if fd in r:
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    chunk = b""
                buf = (buf + chunk)[-4096:]
                # report each message once: cut the buffer after the last newline
                cut = buf.rfind(b"\n")
                if cut >= 0:
                    for line in extract_lines(buf[:cut + 1]):
                        stamp = time.strftime("%H:%M:%S")
                        print(f"{stamp} {line}\r")
                        if log:
                            log.write(f"{stamp} {line}\n")
                            log.flush()
                    buf = buf[cut + 1:]
            if sys.stdin in r:
                ch = sys.stdin.read(1)
                if ch in ("q", "Q", "\x03"):
                    break
                os.write(fd, ch.encode())
    except KeyboardInterrupt:
        pass
    finally:
        try:
            os.write(fd, b"X")      # stop the controller before leaving
        except OSError:
            pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        os.close(fd)
        print("\nstopped (sent X)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
