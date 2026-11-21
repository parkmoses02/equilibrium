#!/usr/bin/env python3
"""Cart-acceleration calibration for the cart_accel_log firmware.

  capture:  python tools/cart_accel_fit.py capture <port> <out.log>
  analyze:  python tools/cart_accel_fit.py analyze <out.log>

capture hangs the pendulum still (you do that), zeroes the encoder, logs, runs the cart chirp and
sends a keep-alive byte every 0.2 s; Ctrl+C / a crash of this tool stops the cart within 1 s
(the firmware also stops on any other key, on position/angle limits and when the run ends).

analyze fits, on the encoder angle phi (0 = up, hanging = +-pi):
    phi'' = K sin(phi) - C phi' + B cos(phi) a
where a is the cart acceleration in the firmware's sign convention (positive = increasing STEP count),
and compares the camera cart x (px) with the counted cart position to get the px/m scale and delay.
Only numpy is needed; the port is read with termios (no pyserial).
"""
import os
import sys
import time

import numpy as np

import camera_kalman_sim as ks

COUNTS_PER_REV = 2400
STEPS_PER_METRE = (200.0 * 16.0) / 0.040


def capture(port, out_path):
    import termios

    print("Safety check: pendulum hanging STILL, cart at the CENTRE of the rail (+-8 cm free each way),")
    print("hands and cables clear of the rail and the pendulum swing, motor driver power ON.")
    print("The cart sweeps 0.5-2 Hz for 30 s. Ctrl+C stops it. Driver power switch is the hard stop.")
    input("Press Enter to start... ")

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

    def pump(seconds, keepalive=False):
        end = time.time() + seconds
        last_k = 0.0
        while time.time() < end:
            if keepalive and time.time() - last_k >= 0.2:
                os.write(fd, b"k")
                last_k = time.time()
            try:
                chunk = os.read(fd, 65536)
            except BlockingIOError:
                chunk = b""
            if chunk:
                data.extend(chunk)
            else:
                time.sleep(0.002)

    started = False
    try:
        for n in (3, 2, 1):
            print(f"starting in {n} ...")
            pump(1.0)
        os.write(fd, b"z")
        pump(1.0)
        os.write(fd, b"l")
        pump(1.5)                      # rest data before the motion
        os.write(fd, b"g")
        t_wait = time.time() + 2.0
        while time.time() < t_wait and b"# STARTED" not in data:
            pump(0.1, keepalive=True)
        if b"# STARTED" not in data:
            refused = [l for l in data.split(b"\n") if b"REFUSED" in l]
            print("the cart did NOT start:", refused[-1].decode(errors="replace") if refused else "no reply")
            return 1
        started = True
        print("running the chirp (30 s) ...")
        t_end = time.time() + 40.0
        while time.time() < t_end:
            pump(0.2, keepalive=True)
            tail = bytes(data[-4000:])
            if b"# STOPPED" in tail:
                break
        stop_line = [l for l in data.split(b"\n") if l.startswith(b"# STOPPED")]
        print("firmware:", stop_line[-1].decode(errors="replace").strip() if stop_line else "(no STOPPED line)")
        pump(3.0)                       # ring-down after the cart stops: free decay data
        os.write(fd, b"l")
        pump(0.3)
    except KeyboardInterrupt:
        print("\ninterrupted: stopping the cart")
    finally:
        try:
            if started:
                os.write(fd, b"x")      # any non-keepalive key stops the cart
        except OSError:
            pass
        time.sleep(0.2)
        try:
            data.extend(os.read(fd, 65536))
        except BlockingIOError:
            pass
        os.close(fd)
        with open(out_path, "wb") as f:
            f.write(data)
        n_lines = data.count(b"\n")
        print(f"saved {len(data)} bytes, {n_lines} lines -> {out_path}")
    return 0


def parse(path):
    S, C = [], []
    with open(path, "rb") as f:
        for raw in f:
            try:
                s = raw.decode("ascii").strip()
            except UnicodeDecodeError:
                continue
            p = s.split(",")
            try:
                if p[0] == "S" and len(p) == 5:
                    S.append((int(p[1]), int(p[2]), int(p[3]), int(p[4])))
                elif p[0] == "C" and len(p) == 7:
                    C.append((int(p[1]), int(p[3]), int(p[4]), int(p[6])))
            except ValueError:
                continue
    S = np.array(S, dtype=np.int64)
    C = np.array(C, dtype=np.int64)
    return S, C


def rel_time(us, origin):
    return ((us - origin) % (1 << 32)).astype(float) * 1e-6


def analyze(path):
    S, C = parse(path)
    if len(S) < 1000:
        print(f"not enough data: {len(S)} S lines")
        return 1
    origin = int(min(S[0, 0], C[0, 0] if len(C) else S[0, 0]))
    t = rel_time(S[:, 0], origin)
    count, vel_mm, steps = S[:, 1].astype(float), S[:, 2].astype(float), S[:, 3].astype(float)
    dt = float(np.median(np.diff(t)))

    moving = np.abs(vel_mm) > 1.0
    if moving.sum() < 500:
        print("the cart barely moved in this log; nothing to fit")
        return 1
    t_move0, t_move1 = t[moving][0], t[moving][-1]
    print(f"log: {len(S)} S lines ({1 / dt:.0f} Hz), cart moved for {t_move1 - t_move0:.1f} s,"
          f" peak speed {np.abs(vel_mm).max() / 1000:.2f} m/s, peak position {np.abs(steps).max() / STEPS_PER_METRE * 100:.1f} cm")

    phi = np.unwrap(count * 2 * np.pi / COUNTS_PER_REV - np.pi)
    w = ks.sg(phi, 41, 3, 1, dt)
    alpha = ks.sg(phi, 61, 3, 2, dt)
    v = vel_mm * 1e-3
    a = ks.sg(v, 61, 3, 1, dt)

    sl = slice(100, -100)
    A = np.column_stack([np.sin(phi[sl]), -w[sl], np.cos(phi[sl]) * a[sl]])
    y = alpha[sl]
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    K, Cd, B = coef
    res = y - A @ coef
    r2 = 1 - np.sum(res ** 2) / np.sum((y - y.mean()) ** 2)
    print(f"\npendulum fit  phi'' = K sin(phi) - C phi' + B cos(phi) a   (a in the firmware's sign convention)")
    print(f"  K = {K:7.2f} 1/s^2   (sqrt = {np.sqrt(abs(K)):.2f} rad/s, effective length g/K = {9.81 / K * 100:.1f} cm)")
    print(f"  C = {Cd:7.3f} 1/s")
    print(f"  B = {B:7.3f} 1/m     (effective length 1/|B| = {100 / abs(B):.1f} cm, sign {'+' if B > 0 else '-'})")
    print(f"  R^2 = {r2:.3f}, residual rms {np.sqrt(np.mean(res ** 2)):.1f} of {np.sqrt(np.mean(y ** 2)):.1f} rad/s^2")
    # how much of the response each term explains
    for name, col in (("gravity", A[:, 0] * K), ("damping", A[:, 1] * Cd), ("cart accel", A[:, 2] * B)):
        print(f"  {name:10s} term rms {np.sqrt(np.mean(col ** 2)):6.1f} rad/s^2")
    if abs(B) < 0.2 * abs(K) ** 0.5:
        print("  note: the cart term is small; the pendulum was barely excited, B is poorly determined")

    # open-loop step counter vs integral of the commanded velocity
    integ = np.concatenate(([0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(t))))
    pos = steps / STEPS_PER_METRE
    off = pos[np.argmax(moving)] - integ[np.argmax(moving)]
    print(f"\ncounted steps vs integrated command: max difference {np.abs(pos - integ - off).max() * 1000:.1f} mm")

    # camera: cart x (px) against the counted position
    if len(C):
        ct = rel_time(C[:, 0], origin)
        flags = C[:, 2]
        x_px = C[:, 3] * 0.1
        ok = (C[:, 3] != 65535) & ((flags & 0x08) != 0)
        ct, x_px = ct[ok], x_px[ok]
        m = (ct > t_move0 - 0.5) & (ct < t_move1 + 0.5)
        ct, x_px = ct[m], x_px[m]
        if len(ct) > 100:
            best = None
            for d in np.arange(-0.02, 0.10, 0.001):
                p = np.interp(ct - d, t, pos)
                if np.ptp(p) < 1e-4:
                    continue
                c = np.corrcoef(p, x_px)[0, 1]
                if best is None or abs(c) > abs(best[0]):
                    best = (c, d)
            if best:
                c, d = best
                p = np.interp(ct - d, t, pos)
                s, x0 = np.polyfit(p, x_px, 1)
                r = x_px - (s * p + x0)
                print(f"camera cart x vs counted position: delay {d * 1000:.0f} ms, correlation {c:.4f},"
                      f" scale {s:.0f} px/m ({s / 1000:.2f} px/mm, sign {'+' if s > 0 else '-'}), residual rms {np.sqrt(np.mean(r ** 2)):.2f} px")
        else:
            print("camera cart x: too few valid samples while the cart moved")
    return 0


if __name__ == "__main__":
    if len(sys.argv) >= 4 and sys.argv[1] == "capture":
        sys.exit(capture(sys.argv[2], sys.argv[3]))
    elif len(sys.argv) >= 3 and sys.argv[1] == "analyze":
        sys.exit(analyze(sys.argv[2]))
    else:
        print(__doc__)
        sys.exit(1)
