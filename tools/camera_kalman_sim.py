#!/usr/bin/env python3
"""Offline test of the camera angle estimator on a camera_delay_log capture.

  python tools/camera_kalman_sim.py <log> [--delay-ms 13.9] [--drop 0.0]

The encoder is the ground truth (no delay). The estimator only sees the camera angle
(sign/offset corrected) arriving with a latency, runs at 500 Hz and must output the
CURRENT angle and rate. Compared:
  raw      camera angle as received (no filter)
  diff     finite-difference rate of the raw camera angle
  ekf      EKF, measurement applied as if it were current (no delay compensation)
  ekf+lag  EKF with fixed-lag retrodiction (update the state D ticks ago, re-propagate)

Model (phi = angle from upright, cart acceleration a):
  phi' = w,  w' = K sin(phi) - C w + B cos(phi) a        (K, C fitted from the encoder; a = 0 in the log)
The firmware port (not done yet) keeps the same structure.
"""
import argparse
import sys

import numpy as np

import camera_delay_xcorr as m

DT = 0.002                 # 500 Hz control tick
SIGN = -1.0                # camera angle sign relative to the encoder (measured)
OFFSET_DEG = -0.96         # enc = SIGN * cam + OFFSET (measured, +-1 deg)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def sg_coeffs(window, order, deriv, dt):
    half = window // 2
    x = np.arange(-half, half + 1, dtype=float)
    V = np.vander(x, order + 1, increasing=True)
    fact = np.prod(np.arange(1, deriv + 1)) if deriv > 0 else 1.0
    return np.linalg.pinv(V)[deriv] * fact / dt ** deriv


def sg(y, window, order, deriv, dt):
    c = sg_coeffs(window, order, deriv, dt)
    return np.convolve(y, c[::-1], mode="same")


def fit_model(t, phi_unwrapped):
    """K, C from phi'' = K sin(phi) - C phi' using smoothed encoder derivatives."""
    dt = np.median(np.diff(t))
    w = sg(phi_unwrapped, 41, 3, 1, dt)
    a = sg(phi_unwrapped, 61, 3, 2, dt)
    sl = slice(80, -80)
    A = np.column_stack([np.sin(phi_unwrapped[sl]), -w[sl]])
    (K, C), *_ = np.linalg.lstsq(A, a[sl], rcond=None)
    if C < 0.0:   # negative damping is a fit artefact (energy would grow in the predictor)
        C = 0.0
        K = float(np.linalg.lstsq(A[:, :1], a[sl], rcond=None)[0][0])
    res = a[sl] - A @ np.array([K, C])
    return K, C, w, np.sqrt(np.mean(res ** 2)), np.sqrt(np.mean(a[sl] ** 2))


class Ekf:
    def __init__(self, K, C, B, q_w, r_deg, lag_ticks, history=64):
        self.K, self.C, self.B = K, C, B
        self.q = q_w
        self.R = np.deg2rad(r_deg) ** 2
        self.lag = lag_ticks
        self.x = np.zeros(2)
        self.P = np.diag([1.0, 100.0])
        self.hist = []          # per tick: (x, P, a) AFTER the predict of that tick
        self.history = history
        self.init = False

    def _predict(self, x, P, a):
        phi, w = x
        s, c = np.sin(phi), np.cos(phi)
        xn = np.array([phi + DT * w, w + DT * (self.K * s - self.C * w + self.B * c * a)])
        F = np.array([[1.0, DT], [DT * (self.K * c - self.B * s * a), 1.0 - self.C * DT]])
        Q = np.diag([0.0, self.q * DT])
        return xn, F @ P @ F.T + Q

    def _update(self, x, P, z):
        y = wrap(z - x[0])
        S = P[0, 0] + self.R
        Kg = P[:, 0] / S
        xn = x + Kg * y
        xn[0] = wrap(xn[0])
        Pn = P - np.outer(Kg, P[0, :])
        return xn, 0.5 * (Pn + Pn.T)

    def tick(self, a, z=None):
        """One 500 Hz tick. z: camera angle that arrived this tick (or None)."""
        if z is not None and not self.init:
            self.x = np.array([z, 0.0]); self.init = True
            self.hist = [(self.x.copy(), self.P.copy(), 0.0)] * (self.lag + 1)
        self.x, self.P = self._predict(self.x, self.P, a)
        self.hist.append((self.x.copy(), self.P.copy(), a))
        if len(self.hist) > self.history:
            self.hist.pop(0)
        if z is None or not self.init:
            return
        if self.lag == 0:
            self.x, self.P = self._update(self.x, self.P, z)
            self.hist[-1] = (self.x.copy(), self.P.copy(), a)
            return
        # measurement belongs to the state 'lag' ticks ago: update it, then re-propagate with stored inputs
        k = len(self.hist) - 1 - self.lag
        xk, Pk, ak = self.hist[k]
        xk, Pk = self._update(xk, Pk, z)
        self.hist[k] = (xk, Pk, ak)
        for j in range(k + 1, len(self.hist)):
            xk, Pk = self._predict(xk, Pk, self.hist[j][2])
            self.hist[j] = (xk.copy(), Pk.copy(), self.hist[j][2])
        self.x, self.P = xk, Pk


def run(ct, cam_phi, et, enc_phi, K, C, q_w, r_deg, lag_ticks, drop_mask=None):
    t0 = max(ct[0], et[0]) + 0.2
    t1 = min(ct[-1], et[-1]) - 0.2
    n = int((t1 - t0) / DT)
    arrive = np.floor((ct - t0) / DT).astype(int)
    z_at = {}
    for i, k in enumerate(arrive):
        if 0 <= k < n and not (drop_mask is not None and drop_mask[i]):
            z_at[k] = cam_phi[i]
    f = Ekf(K, C, 0.0, q_w, r_deg, lag_ticks)
    out = np.zeros((n, 2))
    for k in range(n):
        f.tick(0.0, z_at.get(k))
        out[k] = f.x
    t = t0 + np.arange(n) * DT
    return t, out


def stats(name, phi_hat, w_hat, phi_true, w_true, skip=0):
    e = np.rad2deg(wrap(phi_hat - phi_true))[skip:]
    ew = (w_hat - w_true)[skip:]
    print(f"  {name:10s} angle rms {np.sqrt(np.mean(e ** 2)):6.2f} deg  max {np.abs(e).max():6.1f} deg"
          f" | rate rms {np.sqrt(np.mean(ew ** 2)):6.2f} rad/s  max {np.abs(ew).max():6.1f} rad/s")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--delay-ms", type=float, default=13.9)
    ap.add_argument("--drop", type=float, default=0.0, help="fraction of camera packets dropped in bursts")
    args = ap.parse_args()

    et, ec, ct, cd, age = m.parse(args.log)
    enc_phi_wrapped = wrap(ec * 2 * np.pi / m.COUNTS_PER_REV - np.pi)
    enc_phi_unw = np.unwrap(enc_phi_wrapped)
    K, C, w_enc, rms_res, rms_acc = fit_model(et, enc_phi_unw)
    print(f"model fitted on the encoder: K={K:.1f} 1/s^2 (sqrt={np.sqrt(abs(K)):.2f} rad/s), C={C:.3f} 1/s; "
          f"phi'' residual {rms_res:.1f} of {rms_acc:.1f} rad/s^2 rms")
    cam_phi = wrap(np.deg2rad(SIGN * cd + OFFSET_DEG))

    drop = None
    if args.drop > 0:
        rng = np.random.default_rng(1)
        drop = np.zeros(len(ct), dtype=bool)
        while drop.mean() < args.drop:
            s = rng.integers(0, len(ct)); drop[s:s + int(rng.integers(3, 12))] = True
        print(f"dropping {drop.mean() * 100:.0f}% of camera packets in bursts of 3-11 frames")

    lag_ticks = int(round(args.delay_ms * 1e-3 / DT))
    grid = [(q, r) for q in (50.0, 200.0, 1000.0, 5000.0) for r in (0.3, 1.0)]

    results = []
    for q, r in grid:
        t, o = run(ct, cam_phi, et, enc_phi_wrapped, K, C, q, r, lag_ticks, drop)
        phi_true = np.interp(t, et, enc_phi_unw)
        w_true = np.interp(t, et, w_enc)
        e = wrap(o[:, 0] - phi_true)
        results.append((np.sqrt(np.mean(e[250:] ** 2)), q, r))
    best = min(results)
    print("EKF+lag tuning (angle rms deg over the log):",
          ", ".join(f"q={q:g},r={r:g}:{np.rad2deg(s):.2f}" for s, q, r in results))
    _, q, r = best
    print(f"chosen q={q:g} (rate noise density), r={r:g} deg, lag {lag_ticks} ticks = {lag_ticks * DT * 1000:.0f} ms\n")

    t, o_lag = run(ct, cam_phi, et, enc_phi_wrapped, K, C, q, r, lag_ticks, drop)
    _, o_nolag = run(ct, cam_phi, et, enc_phi_wrapped, K, C, q, r, 0, drop)
    phi_true = np.interp(t, et, enc_phi_unw)
    w_true = np.interp(t, et, w_enc)

    # raw camera angle held at the latest sample, and finite-difference rate
    idx = np.clip(np.searchsorted(ct, t, side="right") - 1, 0, len(ct) - 1)
    raw_phi = cam_phi[idx]
    cu = np.unwrap(cam_phi)
    w_cam = np.gradient(cu, ct)
    skip = 250
    print("error against the encoder at the current time:")
    stats("raw", raw_phi, w_cam[idx], wrap(phi_true), w_true, skip)
    stats("ekf", o_nolag[:, 0], o_nolag[:, 1], wrap(phi_true), w_true, skip)
    stats("ekf+lag", o_lag[:, 0], o_lag[:, 1], wrap(phi_true), w_true, skip)

    # where it matters for balancing: near upright
    near = np.abs(wrap(phi_true)) < np.deg2rad(30)
    near[:skip] = False
    if near.sum() > 100:
        print(f"within +-30 deg of upright ({near.sum() * DT:.1f} s of the log):")
        stats("raw", raw_phi[near], w_cam[idx][near], wrap(phi_true)[near], w_true[near])
        stats("ekf", o_nolag[near, 0], o_nolag[near, 1], wrap(phi_true)[near], w_true[near])
        stats("ekf+lag", o_lag[near, 0], o_lag[near, 1], wrap(phi_true)[near], w_true[near])
    else:
        print("(the pendulum spent too little time within +-30 deg of upright in this log to compare)")


if __name__ == "__main__":
    sys.exit(main())
