#!/usr/bin/env python3
"""Camera angle calibration: a table c(z0) that corrects the camera angle against the encoder.

  capture: python tools/camera_angle_calib.py capture <port> <seconds> <out.log>
  fit:     python tools/camera_angle_calib.py fit <log> [<log> ...] [--write-header src/tests/camera_angle_calib.h]

capture talks to the camera_estimator_test firmware (921600): hang the pendulum still (it is zeroed there),
then swing it by hand through ALL angles, slowly past upright as often as you can, without covering the markers.
It records the X (encoder counts) and C (camera packets) lines.

fit compares every camera packet with the encoder angle at the time the camera frame was taken
(reception time - (age + 6.3 ms)), keeps the slow samples (|rate| < 4 rad/s, so that a small delay error
does not matter), and builds a periodic 36-entry table over z0 = -camera angle (10 deg steps, triangular
smoothing). It reports the residual before (constant -0.96 deg) and after, measured on held-out blocks of the
data (fit on the even 2 s blocks, test on the odd ones), and writes the table as a firmware header.
Needs numpy only; the port is read with termios.
"""
import os
import sys
import time

import numpy as np

import camera_kalman_sim as ks

COUNTS_PER_REV = 2400
N = 36
STEP = 2 * np.pi / N
RATE_MAX = 4.0
MIN_WEIGHT = 25.0
OLD_OFFSET = np.deg2rad(-0.96)
PRE_DRIVER_S = 6.3e-3
DEFAULT_LAG_S = 14e-3


def capture(port, seconds, out_path):
    import termios

    print("Hang the pendulum STILL now (the encoder is zeroed there). Keep the markers uncovered.")
    print(f"Then swing it by hand through all angles for {seconds:.0f} s: slowly past upright as often as you can.")
    input("Press Enter when the pendulum hangs still... ")
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = attrs[1] = attrs[3] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[4] = attrs[5] = termios.B921600
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    data = bytearray()

    def pump(s):
        end = time.time() + s
        while time.time() < end:
            try:
                chunk = os.read(fd, 65536)
            except BlockingIOError:
                chunk = b""
            if chunk:
                data.extend(chunk)
            else:
                time.sleep(0.002)

    try:
        pump(2.5)                       # opening the port may reset the board: let it boot, pendulum still
        os.write(fd, b"z")
        pump(1.0)
        os.write(fd, b"l")
        for n in range(int(seconds), 0, -1):
            if n % 5 == 0 or n <= 3:
                print(f"  {n} s left ...")
            pump(1.0)
    except KeyboardInterrupt:
        print("\ninterrupted: saving what was recorded")
    finally:
        try:
            os.write(fd, b"l")
        except OSError:
            pass
        pump(0.3)
        os.close(fd)
        with open(out_path, "wb") as f:
            f.write(data)
        n_lines = data.count(b"\n")
        print(f"saved {len(data)} bytes, {n_lines} lines -> {out_path}")
    return 0


def parse(path):
    """Encoder counts (E or X lines) and camera packets (C lines) of one log, on the ESP32 clock."""
    enc_t, enc_c, cam = [], [], []
    with open(path, "rb") as f:
        for raw in f:
            p = raw.decode("ascii", "ignore").strip().split(",")
            try:
                if p[0] in ("E", "X") and len(p) >= 3:
                    enc_t.append(int(p[1])); enc_c.append(int(p[2]))
                elif p[0] == "C" and len(p) >= 6:
                    cam.append((int(p[1]), int(p[3]), int(p[4]), int(p[5])))
            except ValueError:
                continue
    return np.array(enc_t, dtype=np.int64), np.array(enc_c, dtype=float), np.array(cam, dtype=np.int64)


def samples_from(path):
    enc_t, enc_c, cam = parse(path)
    if len(enc_t) < 500 or len(cam) < 50:
        print(f"  {path}: not enough data (encoder {len(enc_t)}, camera {len(cam)})")
        return None
    origin = int(min(enc_t[0], cam[0, 0]))
    te = ((enc_t - origin) % (1 << 32)).astype(float) * 1e-6
    tc = ((cam[:, 0] - origin) % (1 << 32)).astype(float) * 1e-6
    dt = float(np.median(np.diff(te)))
    phi = np.unwrap(enc_c * 2 * np.pi / COUNTS_PER_REV - np.pi)
    rate = ks.sg(phi, 41, 3, 1, dt)
    valid = ((cam[:, 2] & 0x01) != 0) & (cam[:, 1] != -32768)
    ang, flags, age = cam[valid, 1] * 0.01, cam[valid, 2], cam[valid, 3]
    tc = tc[valid]
    lag = np.where((flags & 0x20) != 0, age * 1e-4 + PRE_DRIVER_S, DEFAULT_LAG_S)
    tm = tc - lag
    ok = (tm > te[0] + 0.3) & (tm < te[-1] - 0.3)
    tm, ang = tm[ok], ang[ok]
    phi_m = ks.wrap(np.interp(tm, te, phi))
    rate_m = np.interp(tm, te, rate)
    z0 = ks.wrap(-np.deg2rad(ang))
    return dict(z0=z0, phi=phi_m, rate=rate_m, t=tm, target=ks.wrap(phi_m - z0), name=path)


def fit_table(z0, target):
    centers = -np.pi + STEP * np.arange(N)
    c = np.full(N, np.nan)
    wsum = np.zeros(N)
    for i, ctr in enumerate(centers):
        d = np.abs(ks.wrap(z0 - ctr))
        w = np.clip(1.0 - d / STEP, 0.0, None)
        wsum[i] = w.sum()
        if wsum[i] >= MIN_WEIGHT:
            c[i] = np.sum(w * target) / wsum[i]
    good = ~np.isnan(c)
    if good.sum() == 0:
        c[:] = float(np.mean(target))
    elif (~good).any():           # fill the poorly covered entries from their neighbours (periodic)
        idx = np.arange(N)
        gi = idx[good]
        ext_i = np.concatenate((gi - N, gi, gi + N))
        ext_v = np.concatenate((c[good], c[good], c[good]))
        c[~good] = np.interp(idx[~good], ext_i, ext_v)
    return c, wsum, ~good


def apply_table(c, z0):
    x = (z0 + np.pi) / STEP
    i = np.floor(x).astype(int)
    f = x - i
    a, b = c[i % N], c[(i + 1) % N]
    return ks.wrap(z0 + a + (b - a) * f)


def residual_stats(phi, z):
    r = np.rad2deg(ks.wrap(phi - z))
    out = {}
    for name, m in (("all", np.ones(len(r), bool)), ("|phi|<30", np.abs(np.rad2deg(phi)) < 30), ("|phi|<5", np.abs(np.rad2deg(phi)) < 5)):
        out[name] = (int(m.sum()), float(r[m].mean()) if m.any() else float("nan"),
                     float(np.sqrt(np.mean(r[m] ** 2))) if m.any() else float("nan"))
    return out


def write_header(path, c, info):
    vals = ", ".join(f"{v:.6f}f" for v in c)
    text = f'''// Camera angle calibration table (periodic, {N} entries, -180..+170 deg in 10 deg steps).
// Generated by tools/camera_angle_calib.py fit: {info}
// camCalibrate(z0): z0 = -camera angle (sign already applied), wrapped to (-pi, pi]; returns the corrected angle
// z0 + c(z0) in the encoder's convention (0 = upright). c is linearly interpolated between the entries and
// absorbs the zero offset. Entry i belongs to z0 = -180 deg + 10 deg * i.
#pragma once
#include <math.h>

constexpr int CAM_CAL_N = {N};
constexpr float CAM_CAL_STEP_RAD = 2.0f * 3.14159265358979f / CAM_CAL_N;
constexpr float CAM_CAL_RAD[CAM_CAL_N] = {{{vals}}};

static inline float camCalWrap(float a) {{
    while (a > 3.14159265358979f) a -= 2.0f * 3.14159265358979f;
    while (a <= -3.14159265358979f) a += 2.0f * 3.14159265358979f;
    return a;
}}

static inline float camCalibrate(float z0) {{
    const float x = (z0 + 3.14159265358979f) / CAM_CAL_STEP_RAD;
    int i = (int)x;
    if (i < 0) i = 0;
    const float f = x - (float)i;
    const float a = CAM_CAL_RAD[i % CAM_CAL_N];
    const float b = CAM_CAL_RAD[(i + 1) % CAM_CAL_N];
    return camCalWrap(z0 + a + (b - a) * f);
}}
'''
    with open(path, "w") as f:
        f.write(text)


def fit(paths, header):
    parts = [s for s in (samples_from(p) for p in paths) if s]
    if not parts:
        return 1
    z0 = np.concatenate([p["z0"] for p in parts]); phi = np.concatenate([p["phi"] for p in parts])
    rate = np.concatenate([p["rate"] for p in parts]); target = np.concatenate([p["target"] for p in parts])
    block = np.concatenate([np.floor(p["t"] / 2.0).astype(int) % 2 + 10 * k for k, p in enumerate(parts)]) % 2
    slow = np.abs(rate) < RATE_MAX
    print(f"{len(z0)} camera samples from {len(parts)} log(s), {int(slow.sum())} slow ones (|rate| < {RATE_MAX} rad/s) used for the fit")

    train = slow & (block == 0)
    test = slow & (block == 1)
    c_tr, _, _ = fit_table(z0[train], target[train])
    print("\nhold-out check (table fitted on the even 2 s blocks, residual measured on the odd ones, slow samples):")
    for label, z in (("constant -0.96 deg ", ks.wrap(z0[test] + OLD_OFFSET)), ("fitted table        ", apply_table(c_tr, z0[test]))):
        st = residual_stats(phi[test], z)
        print("  " + label + " | " + " | ".join(f"{k}: n={v[0]} mean {v[1]:+.2f} rms {v[2]:.2f} deg" for k, v in st.items()))

    c, wsum, filled = fit_table(z0[slow], target[slow])
    print("\nfinal table (entry angle z0 = -camera angle; correction in deg; weight = samples in the +-10 deg window):")
    for i in range(N):
        ang = -180 + 10 * i
        note = "   (filled from neighbours: too few samples)" if filled[i] else ""
        print(f"  {ang:+4d} deg  c = {np.rad2deg(c[i]):+6.2f}   weight {wsum[i]:6.0f}{note}")
    st = residual_stats(phi[slow], apply_table(c, z0[slow]))
    print("in-sample residual of the final table: " + " | ".join(f"{k}: mean {v[1]:+.2f} rms {v[2]:.2f}" for k, v in st.items()))
    near = [i for i in range(N) if abs(-180 + 10 * i) <= 20]
    if filled[near].any():
        print("WARNING: entries within 20 deg of upright were filled from neighbours: collect more slow passes near upright")
    if header:
        write_header(header, c, f"{len(z0)} samples, {int(slow.sum())} used, logs: {', '.join(os.path.basename(p['name']) for p in parts)}")
        print(f"\nwrote {header}")
    return 0


if __name__ == "__main__":
    args = sys.argv[1:]
    if len(args) >= 4 and args[0] == "capture":
        sys.exit(capture(args[1], float(args[2]), args[3]))
    elif len(args) >= 2 and args[0] == "fit":
        header = None
        files = []
        i = 1
        while i < len(args):
            if args[i] == "--write-header":
                header = args[i + 1]; i += 2
            else:
                files.append(args[i]); i += 1
        sys.exit(fit(files, header))
    else:
        print(__doc__)
        sys.exit(1)
