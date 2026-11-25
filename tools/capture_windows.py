"""Common COM capture routines for the 921600-baud ESP32 measurement firmware."""
from contextlib import contextmanager
import math
import re
import time

import serial


def duration(value):
    seconds = float(value)
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError("seconds must be finite and positive")
    return seconds


class Capture:
    def __init__(self, device, log):
        self.device = device
        self.log = log
        self.pending = b""

    def pump(self, seconds, keepalive=False, until=None):
        deadline = time.monotonic() + seconds
        next_keepalive = 0.0
        while time.monotonic() < deadline:
            if until:
                match = re.search(until, self.pending)
                if match:
                    self.pending = self.pending[match.end():]
                    return match
            now = time.monotonic()
            if keepalive and now >= next_keepalive:
                self.device.write(b"k")
                next_keepalive = now + 0.2
            chunk = self.device.read(min(self.device.in_waiting or 1, 65536))
            if chunk:
                self.log.write(chunk)
                self.pending = self.pending[-64:] + chunk if until else b""
        return None


@contextmanager
def recording(port, path, zero=False):
    with open(path, "xb") as log, serial.Serial(port, 921600, timeout=0.02, write_timeout=0.5) as device:
        cap = Capture(device, log)
        cap.pump(2.5)  # Opening the USB port can reset the ESP32.
        if zero:
            device.write(b"z")
            cap.pump(1.0)
        device.write(b"l")
        status = cap.pump(1, until=rb"# logging (ON|OFF)")
        if status and status[1] == b"OFF":
            device.write(b"l")
            status = cap.pump(1, until=rb"# logging (ON|OFF)")
        if not status or status[1] != b"ON":
            raise RuntimeError("Missing logging ON acknowledgement. Check the COM port and firmware.")
        try:
            yield cap
        finally:
            device.write(b"l")
            cap.pump(0.3)


def capture_timed(port, seconds, path, zero=False):
    if zero:
        input("Hang the pendulum still with markers visible. Press Enter and hold until prompted. ")
    with recording(port, path, zero) as cap:
        print(f"Recording for {seconds:g} seconds. Move the pendulum/cart by hand now.")
        cap.pump(seconds)
