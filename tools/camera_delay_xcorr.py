#!/usr/bin/env python3
"""Camera latency / sign / offset from the camera_delay_log firmware.

  capture:  python tools/camera_delay_xcorr.py capture <port> <seconds> <out.log>
            (sends 'l' to start/stop logging; hang the pendulum still and press 'z' first, then swing by hand)
  analyze:  python tools/camera_delay_xcorr.py analyze <out.log>

Compares the encoder angle with the camera angle on the ESP32 clock. The score is circular
(1 - mean(1 - cos(camera*s - encoder(t - d) - offset)) maximised over the offset), so it is
insensitive to angle wrapping and to a constant zero offset. Only needs numpy.
"""
import os
import sys
import time

COUNTS_PER_REV = 2400


def capture(port, seconds, out_path):
    import termios
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = attrs[1] = attrs[3] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[4] = attrs[5] = termios.B921600
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)

    os.write(fd, b"l")
    data = bytearray()
    t_end = time.time() + seconds
    while time.time() < t_end:
        try:
            chunk = os.read(fd, 65536)
        except BlockingIOError:
            chunk = b""
        if chunk:
            data += chunk
        else:
            time.sleep(0.002)
    os.write(fd, b"l")
    time.sleep(0.3)
    try:
        data += os.read(fd, 65536)
    except BlockingIOError:
        pass
    os.close(fd)
    with open(out_path, "wb") as f:
        f.write(data)
    lines = data.count(b"\n")
    print(f"saved {len(data)} bytes, {lines} lines -> {out_path}")


def parse(path):
    import numpy as np
    enc_t, enc_c, cam_t, cam_a, cam_flags, cam_age = [], [], [], [], [], []
    with open(path, "rb") as f:
        for raw in f:
            try:
                s = raw.decode("ascii").strip()
            except UnicodeDecodeError:
                continue
            if s.startswith("E,"):
                p = s.split(",")
                if len(p) == 3:
                    try:
                        enc_t.append(int(p[1])); enc_c.append(int(p[2]))
                    except ValueError:
                        pass
            elif s.startswith("C,"):
                p = s.split(",")
                if len(p) == 6:
                    try:
                        cam_t.append(int(p[1])); cam_a.append(int(p[3]))
                        cam_flags.append(int(p[4])); cam_age.append(int(p[5]))
                    except ValueError:
                        pass

    # shared time base: unwrap both series with the same origin
    all_t = np.array(enc_t + cam_t, dtype=np.int64)
    origin = int(all_t.min())
    enc_t = ((np.asarray(enc_t, dtype=np.int64) - origin) % (1 << 32)).astype(float)
    cam_t = ((np.asarray(cam_t, dtype=np.int64) - origin) % (1 << 32)).astype(float)
    return (enc_t * 1e-6, np.asarray(enc_c, dtype=float), cam_t * 1e-6,
            np.asarray(cam_a, dtype=float) * 0.01, np.asarray(cam_age, dtype=float) * 0.1)


def analyze(path):
    import numpy as np
    enc_t, enc_c, cam_t, cam_deg, cam_age_ms = parse(path)
    if len(enc_t) < 200 or len(cam_t) < 50:
        print(f"not enough data: encoder {len(enc_t)}, camera {len(cam_t)} samples")
        return 1

    # encoder: hanging = 0 count, upright = half a revolution away; unwrap for interpolation
    enc_rad = np.unwrap(enc_c * 2 * np.pi / COUNTS_PER_REV - np.pi)
    cam_rad = np.deg2rad(cam_deg)

    # use only the window where both are present
    t0, t1 = max(enc_t[0], cam_t[0]), min(enc_t[-1], cam_t[-1])
    m = (cam_t >= t0 + 0.5) & (cam_t <= t1)
    cam_t, cam_rad = cam_t[m], cam_rad[m]
    dur = cam_t[-1] - cam_t[0]
    swing = np.ptp(enc_rad)
    print(f"samples: encoder {len(enc_t)}, camera {len(cam_t)} ({len(cam_t) / dur:.1f}/s) over {dur:.1f} s;"
          f" encoder range {np.rad2deg(swing):.0f} deg")
    if swing < np.deg2rad(20):
        print("WARNING: the pendulum barely moved; swing it more for a meaningful result")

    delays = np.arange(-0.050, 0.300, 0.001)
    best = None
    curves = {}
    for sign in (+1, -1):
        scores = []
        for d in delays:
            e = np.interp(cam_t - d, enc_t, enc_rad)
            z = np.mean(np.exp(1j * (sign * cam_rad - e)))
            scores.append(abs(z))
        scores = np.array(scores)
        curves[sign] = scores
        i = int(np.argmax(scores))
        if best is None or scores[i] > best[0]:
            best = (scores[i], sign, delays[i], i)
    R, sign, d, i = best
    e = np.interp(cam_t - d, enc_t, enc_rad)
    z = np.mean(np.exp(1j * (sign * cam_rad - e)))
    offset = np.angle(z)
    other = curves[-sign].max()

    # scale check: regress camera on shifted encoder around the circular offset
    diff = np.angle(np.exp(1j * (sign * cam_rad - e - offset)))
    resid_deg = np.rad2deg(diff)
    print(f"best sign: {'+' if sign > 0 else '-'}  (opposite sign score {other:.3f})")
    print(f"total latency (exposure -> ESP32 reception): {d * 1000:.0f} ms   score R={R:.3f}")
    print(f"zero offset (camera*sign - encoder): {np.rad2deg(offset):+.2f} deg")
    print(f"residual after alignment: rms {np.sqrt(np.mean(resid_deg ** 2)):.2f} deg,"
          f" max {np.max(np.abs(resid_deg)):.1f} deg")
    print(f"packet 'age' field (driver frame -> transmit): mean {cam_age_ms.mean():.1f} ms"
          f" (min {cam_age_ms.min():.1f}, max {cam_age_ms.max():.1f})")
    peak = curves[sign]
    width = (peak > R - 0.02).sum()
    print(f"peak width (score within 0.02 of best): {width} ms")
    if R < 0.9:
        print("WARNING: weak match (R < 0.9): check sign/zero, marker dropouts, or swing amplitude")
    if abs(d) < 0.001 or i == len(delays) - 1:
        print("WARNING: best delay is at the edge of the search range")
    return 0


if __name__ == "__main__":
    if len(sys.argv) >= 5 and sys.argv[1] == "capture":
        capture(sys.argv[2], float(sys.argv[3]), sys.argv[4])
    elif len(sys.argv) >= 3 and sys.argv[1] == "analyze":
        sys.exit(analyze(sys.argv[2]))
    else:
        print(__doc__)
        sys.exit(1)
