#!/usr/bin/env python3
"""Checks the firmware estimator (camera_estimator_test) against the Python EKF and the encoder.

  python tools/camera_ekf_check.py <log>

<log> holds the firmware's X and C lines (a real capture, or the host harness output). The Python EKF
(tools/camera_kalman_sim.py, float64) is run on exactly the firmware's tick times and camera packets
(same sign/offset correction, same age-based lag), so any difference to the firmware output is a
porting error or float32 rounding. The encoder (count in the X lines) is the ground truth.
"""
import sys

import numpy as np

import camera_kalman_sim as ks

COUNTS_PER_REV = 2400
CAM_SIGN, CAM_OFFSET_RAD = -1.0, np.deg2rad(-0.96)
PRE_DRIVER_MS = 6.3
K, C, B, Q, R_DEG = 64.6, 0.05, -9.4, 200.0, 1.0   # must match camera_estimator_test.cpp


def parse(path):
    X, Cp = [], []
    with open(path, "rb") as f:
        for raw in f:
            s = raw.decode("ascii", "ignore").strip()
            p = s.split(",")
            try:
                if p[0] == "X" and len(p) == 5:
                    X.append((int(p[1]), int(p[2]), int(p[3]), int(p[4])))
                elif p[0] == "C" and len(p) == 6:
                    Cp.append((int(p[1]), int(p[3]), int(p[4]), int(p[5])))
            except ValueError:
                continue
    return np.array(X, dtype=np.int64), np.array(Cp, dtype=np.int64)


def main():
    X, Cp = parse(sys.argv[1])
    if len(X) < 1000:
        print(f"not enough X lines: {len(X)}")
        return 1
    tx = (X[:, 0] - X[0, 0]) * 1e-6
    tc = (Cp[:, 0] - X[0, 0]) * 1e-6
    count = X[:, 1].astype(float)
    fw_phi = np.deg2rad(X[:, 2] * 0.01)
    fw_w = X[:, 3] * 1e-3
    dt_tick = float(np.median(np.diff(tx)))
    enc_phi = ks.wrap(count * 2 * np.pi / COUNTS_PER_REV - np.pi)
    enc_unw = np.unwrap(enc_phi)
    w_enc = ks.sg(enc_unw, 41, 3, 1, dt_tick)

    # firmware semantics: a camera angle is consumed by the first tick at or after its reception
    valid = ((Cp[:, 2] & 0x01) != 0) & (Cp[:, 1] != -32768)
    tc, ang, flags, age = tc[valid], Cp[valid, 1] * 0.01, Cp[valid, 2], Cp[valid, 3]
    z = ks.wrap(CAM_SIGN * np.deg2rad(ang) + CAM_OFFSET_RAD)
    lag = np.where((flags & 0x20) != 0, np.rint((age * 0.1 + PRE_DRIVER_MS) / 2.0), 7).astype(int)
    tick_of = np.searchsorted(tx, tc, side="left")
    z_at = {}
    for i, k in enumerate(tick_of):
        if k < len(tx):
            z_at[int(k)] = (z[i], int(lag[i]))

    f = ks.Ekf(K, C, B, Q, R_DEG, 7, history=64)
    py = np.zeros((len(tx), 2))
    for k in range(len(tx)):
        if k in z_at:
            zz, lg = z_at[k]
            f.lag = lg
            f.tick(0.0, zz)
        else:
            f.tick(0.0, None)
        py[k] = f.x

    # raw camera angle, held at the latest sample
    idx = np.clip(np.searchsorted(tc, tx, side="right") - 1, 0, len(tc) - 1)
    raw = z[idx]
    sk = 250
    def deg_rms(a, b): return np.rad2deg(np.sqrt(np.mean(ks.wrap(a - b)[sk:-sk] ** 2)))
    def deg_max(a, b): return np.rad2deg(np.abs(ks.wrap(a - b)[sk:-sk]).max())
    print(f"{len(tx)} ticks ({1 / dt_tick:.0f} Hz), {len(tc)} valid camera packets, lag ticks used: {sorted(set(lag.tolist()))}")
    print("angle error against the encoder at the current time:")
    print(f"  raw camera         rms {deg_rms(raw, enc_phi):6.2f} deg   max {deg_max(raw, enc_phi):6.1f} deg")
    print(f"  firmware EKF       rms {deg_rms(fw_phi, enc_phi):6.2f} deg   max {deg_max(fw_phi, enc_phi):6.1f} deg")
    print(f"  python EKF replica rms {deg_rms(py[:, 0], enc_phi):6.2f} deg   max {deg_max(py[:, 0], enc_phi):6.1f} deg")
    ew_fw, ew_py = (fw_w - w_enc)[sk:-sk], (py[:, 1] - w_enc)[sk:-sk]
    print(f"rate error: firmware rms {np.sqrt(np.mean(ew_fw ** 2)):.2f} rad/s (max {np.abs(ew_fw).max():.1f}),"
          f" python rms {np.sqrt(np.mean(ew_py ** 2)):.2f} rad/s")
    d_phi = np.rad2deg(ks.wrap(fw_phi - py[:, 0]))[sk:-sk]
    d_w = (fw_w - py[:, 1])[sk:-sk]
    print(f"firmware minus python: angle rms {np.sqrt(np.mean(d_phi ** 2)):.4f} deg (max {np.abs(d_phi).max():.3f}),"
          f" rate rms {np.sqrt(np.mean(d_w ** 2)):.4f} rad/s (max {np.abs(d_w).max():.3f})")
    print("(the firmware logs angles in 0.01 deg and rates in 0.001 rad/s, so ~0.005 deg / 0.0005 rad/s is rounding)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
