#!/usr/bin/env python3
"""Checks the camera angle estimator while the cart moves (camera_estimator_cart_test firmware).

  python tools/camera_est_cart_check.py <log> [--no-sweep]

<log> holds X and C lines (a real capture from tools/cart_accel_fit.py capture, or the host harness output).
  1. error against the encoder of: raw camera, firmware EKF with the cart term (B), firmware EKF without it (0),
     over the whole log and over the time the cart moved
  2. the Python EKF (float64) replays the firmware's ticks, camera packets and logged cart acceleration with B = -9.4:
     the firmware must match it (porting check, now including the acceleration input)
  3. B sweep: the replay is repeated for other B values; the minimum of the angle error shows the best B and sign
"""
import sys

import numpy as np

import camera_kalman_sim as ks

COUNTS_PER_REV = 2400
CAM_SIGN, CAM_OFFSET_RAD = -1.0, np.deg2rad(-0.96)
PRE_DRIVER_MS = 6.3
K, C, B_FW, Q, R_DEG = 64.6, 0.05, -9.4, 200.0, 1.0   # must match camera_estimator_cart_test.cpp


def parse(path):
    X, Cp = [], []
    with open(path, "rb") as f:
        for raw in f:
            s = raw.decode("ascii", "ignore").strip()
            p = s.split(",")
            try:
                if p[0] == "X" and len(p) == 10:
                    X.append([int(v) for v in p[1:]])
                elif p[0] == "C" and len(p) >= 6:
                    Cp.append((int(p[1]), int(p[3]), int(p[4]), int(p[5])))
            except ValueError:
                continue
    return np.array(X, dtype=np.int64), np.array(Cp, dtype=np.int64)


def replay(tx, z_at, acc, B):
    f = ks.Ekf(K, C, B, Q, R_DEG, 7, history=64)
    out = np.zeros((len(tx), 2))
    for k in range(len(tx)):
        if k in z_at:
            zz, lg = z_at[k]
            f.lag = lg
            f.tick(acc[k], zz)
        else:
            f.tick(acc[k], None)
        out[k] = f.x
    return out


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    sweep = "--no-sweep" not in sys.argv
    X, Cp = parse(args[0])
    if len(X) < 1000:
        print(f"not enough X lines: {len(X)}")
        return 1
    tx = (X[:, 0] - X[0, 0]) * 1e-6
    tc = (Cp[:, 0] - X[0, 0]) * 1e-6
    count = X[:, 1].astype(float)
    fwB = np.deg2rad(X[:, 2] * 0.01)
    wB = X[:, 3] * 1e-3
    fw0 = np.deg2rad(X[:, 4] * 0.01)
    w0 = X[:, 5] * 1e-3
    vel = X[:, 6] * 1e-3
    acc = X[:, 7] * 1e-3
    dt_tick = float(np.median(np.diff(tx)))
    enc_phi = ks.wrap(count * 2 * np.pi / COUNTS_PER_REV - np.pi)
    w_enc = ks.sg(np.unwrap(enc_phi), 41, 3, 1, dt_tick)

    valid = ((Cp[:, 2] & 0x01) != 0) & (Cp[:, 1] != -32768)
    tcv, ang, flags, age = tc[valid], Cp[valid, 1] * 0.01, Cp[valid, 2], Cp[valid, 3]
    z = ks.wrap(CAM_SIGN * np.deg2rad(ang) + CAM_OFFSET_RAD)
    lag = np.where((flags & 0x20) != 0, np.rint((age * 0.1 + PRE_DRIVER_MS) / 2.0), 7).astype(int)
    tick_of = np.searchsorted(tx, tcv, side="left")
    z_at = {int(k): (z[i], int(lag[i])) for i, k in enumerate(tick_of) if k < len(tx)}
    idx = np.clip(np.searchsorted(tcv, tx, side="right") - 1, 0, len(tcv) - 1)
    raw = z[idx]

    moving = np.abs(vel) > 0.001
    if moving.sum() < 500:
        print("the cart barely moved in this log")
        return 1
    t_m0, t_m1 = tx[moving][0], tx[moving][-1]
    sk = 250
    keep_all = np.zeros(len(tx), bool); keep_all[sk:-sk] = True
    keep_mov = keep_all & (tx > t_m0 + 1.0) & (tx < t_m1 - 1.0)
    print(f"{len(tx)} ticks ({1 / dt_tick:.0f} Hz), {len(tcv)} valid camera packets; cart moved {t_m0:.1f}-{t_m1:.1f} s,"
          f" peak accel {np.abs(acc).max():.2f} m/s^2 (counted)")

    def ang_stats(est, mask):
        e = np.rad2deg(ks.wrap(est - enc_phi))[mask]
        return np.sqrt(np.mean(e ** 2)), np.abs(e).max()

    def rate_rms(w, mask):
        return np.sqrt(np.mean((w - w_enc)[mask] ** 2))

    for name, mask in (("whole log", keep_all), ("while the cart moved", keep_mov)):
        print(f"\n{name}: error against the encoder at the current time")
        r, rm = ang_stats(raw, mask)
        print(f"  raw camera              angle rms {r:6.2f} deg (max {rm:5.1f})")
        for label, ph, w in (("firmware EKF, cart term B", fwB, wB), ("firmware EKF, a = 0     ", fw0, w0)):
            a_rms, a_max = ang_stats(ph, mask)
            print(f"  {label}  angle rms {a_rms:6.2f} deg (max {a_max:5.1f})   rate rms {rate_rms(w, mask):.3f} rad/s")

    rep = replay(tx, z_at, acc, B_FW)
    d_phi = np.rad2deg(ks.wrap(fwB - rep[:, 0]))[keep_all]
    d_w = (wB - rep[:, 1])[keep_all]
    print(f"\nfirmware minus python replay (B = {B_FW}): angle rms {np.sqrt(np.mean(d_phi ** 2)):.4f} deg"
          f" (max {np.abs(d_phi).max():.3f}), rate rms {np.sqrt(np.mean(d_w ** 2)):.4f} rad/s (max {np.abs(d_w).max():.3f})")
    print("(logging rounds angles to 0.01 deg and rates to 0.001 rad/s)")

    if sweep:
        print("\nB sweep (Python replay on the logged cart acceleration; error over the time the cart moved):")
        cache = {}
        def score(B):
            if B not in cache:
                o = replay(tx, z_at, acc, B)
                cache[B] = (ang_stats(o[:, 0], keep_mov)[0], rate_rms(o[:, 1], keep_mov))
            return cache[B]
        for B in range(-16, 5, 2):
            score(float(B))
        # B changes the predicted RATE; the angle is pinned by the 90 Hz camera and barely moves with B,
        # so the best B is chosen on the rate error
        best = min(cache, key=lambda b: cache[b][1])
        for B in np.arange(best - 1.5, best + 1.75, 0.5):
            score(round(float(B), 2))
        best = min(cache, key=lambda b: cache[b][1])
        for B in sorted(cache):
            a, w = cache[B]
            mark = "   <- best (rate)" if B == best else ""
            print(f"  B = {B:6.2f}   angle rms {a:6.3f} deg   rate rms {w:.3f} rad/s{mark}")
        b0 = cache.get(0.0, (np.nan, np.nan))
        print(f"firmware uses B = {B_FW}; B = 0 (cart term off) gives {b0[0]:.3f} deg / {b0[1]:.3f} rad/s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
