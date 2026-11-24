"""Fit the double-pendulum model to SYSID_CSV logs from the double_sysid firmware.

python simulation/sysid/fit.py logs/*.log             # fit and report
python simulation/sysid/fit.py logs/*.log --write     # also rewrite src/double_pendulum/model_params.h
python simulation/sysid/fit.py --selftest             # synthetic round trip, no hardware

Requires numpy and scipy. Model (absolute angles from up, u = cart acceleration):
  J1 a1'' + C[cos(a1-a2) a2'' + sin(a1-a2) w2^2] = d1 (g sin a1 - u cos a1)
        + b2 (w2-w1) + fc2 sgn(w2-w1) - b1 w1 - fc1 sgn(w1)
  J2 a2'' + C[cos(a1-a2) a1'' - sin(a1-a2) w1^2] = d2 (g sin a2 - u cos a2)
        - b2 (w2-w1) - fc2 sgn(w2-w1)
The equations are linear in (J1, J2, C, d1, d2, b1, b2, fc1, fc2) and the motion
is unchanged when all of them are scaled together, so d2 is held at its
nominal value. To avoid differentiating quantised angles twice (noise in the
regressors biases every inertia low), they are used in integral form: the
change of generalised momentum p = H(q) q' over each short interval equals
the integral of the generalised forces. That needs rates only. Bounded least
squares gives the estimate; --refine then minimises the simulated angle error
over short windows (output error, insensitive to measurement noise).
Cart acceleration comes from the ramp velocity (commanded, not measured
travel); a pure input delay is estimated on chirp runs.
"""
import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares, lsq_linear
from scipy.signal import savgol_filter

ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "src/double_pendulum/model_params.h"
G = 9.80665
NAMES = ("J1", "J2", "C", "d1", "d2", "b1", "b2", "fc1", "fc2")
COULOMB_WIDTH = .05
INTERVAL_S = .1   # momentum-balance interval
BAD_FLAGS = 1 | 2


def nominal():
    B, S, length, T2, Tl = .002475, .0128, .16, .792172, .930958
    C = length * B
    J2 = B * G * (T2 / (2 * np.pi)) ** 2
    J1 = S * G * (Tl / (2 * np.pi)) ** 2 - J2 - 2 * C
    return dict(J1=J1, J2=J2, C=C, d1=S - B, d2=B, b1=0., b2=0., fc1=0., fc2=0.)


def deriv(p, s, u):
    """Vectorised over leading axes: s[..., 4] = (a1, a2, w1, w2)."""
    a, b, w, z = np.moveaxis(s, -1, 0)
    h, sn = p["C"] * np.cos(a - b), p["C"] * np.sin(a - b)
    det = p["J1"] * p["J2"] - h * h
    rel = z - w
    elbow = p["b2"] * rel + p["fc2"] * np.tanh(rel / COULOMB_WIDTH)
    f = p["d1"] * (G * np.sin(a) - u * np.cos(a)) - sn * z * z + elbow - p["b1"] * w \
        - p["fc1"] * np.tanh(w / COULOMB_WIDTH)
    k = p["d2"] * (G * np.sin(b) - u * np.cos(b)) + sn * w * w - elbow
    return np.stack([w, z, (p["J2"] * f - h * k) / det, (p["J1"] * k - h * f) / det], -1)


def rk4(p, s, u, dt):
    k1 = deriv(p, s, u)
    k2 = deriv(p, s + dt / 2 * k1, u)
    k3 = deriv(p, s + dt / 2 * k2, u)
    k4 = deriv(p, s + dt * k3, u)
    return s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


# ---------------------------------------------------------------- parsing ---
@dataclass
class Run:
    name: str
    kind: str
    dt: float
    t: np.ndarray
    a: np.ndarray     # unwrapped base angle
    b: np.ndarray     # unwrapped tip angle
    v: np.ndarray     # ramp velocity
    vcmd: np.ndarray
    flags: np.ndarray


BLOCK = re.compile(r"SYSID_CSV_BEGIN([^\n]*)\n[^\n]*\n(.*?)\nSYSID_CSV_END", re.S)


def parse(path):
    text = Path(path).read_text(encoding="utf-8", errors="replace").replace("\r", "")
    runs = []
    for n, (meta, body) in enumerate(BLOCK.findall(text)):
        info = dict(kv.split("=", 1) for kv in meta.split() if "=" in kv)
        rows = [line.split(",") for line in body.splitlines() if line.count(",") == 6]
        data = np.array(rows, dtype=float)
        if len(data) < 100:
            print(f"skip {path}#{n}: only {len(data)} rows")
            continue
        t = data[:, 0] * 1e-6
        if not np.all(np.diff(t) > 0):
            raise ValueError(f"{path}#{n}: timestamps not increasing")
        dt = float(np.median(np.diff(t)))
        grid = np.arange(t[0], t[-1], dt)
        interp = lambda y: np.interp(grid, t, y)
        flags = np.interp(grid, t, data[:, 6], left=0, right=0).round().astype(int)
        runs.append(Run(f"{Path(path).name}#{n}", info.get("kind", "?"), dt, grid,
                        interp(np.unwrap(data[:, 4])), interp(np.unwrap(data[:, 5])),
                        interp(data[:, 2]), interp(data[:, 3]), flags))
    return runs


# ------------------------------------------------------------- regression ---
@dataclass
class Prepared:
    run: Run
    s: np.ndarray      # (N, 4) smoothed state
    u: np.ndarray      # (N,) cart acceleration
    mask: np.ndarray   # usable samples


def prepare(run, window, skip):
    n = window if window % 2 else window + 1
    sg = lambda y, d: savgol_filter(y, n, 3, deriv=d, delta=run.dt)
    s = np.column_stack([sg(run.a, 0), sg(run.b, 0), sg(run.a, 1), sg(run.b, 1)])
    u = sg(run.v, 1)
    mask = np.ones(len(run.t), bool)
    mask[: n] = mask[-n:] = False
    mask &= run.t - run.t[0] >= skip
    bad = np.flatnonzero(run.flags & BAD_FLAGS)
    for i in bad:   # invalid encoder edges corrupt the derivatives nearby
        mask[max(0, i - n): i + n + 1] = False
    return Prepared(run, s, u, mask)


def shift(u, delay, dt):
    """u delayed by `delay` seconds (fractional samples by linear interpolation)."""
    if delay <= 0:
        return u
    t = np.arange(len(u)) * dt
    return np.interp(t - delay, t, u, left=u[0])


def rows(prep, delay, d2):
    """Momentum balance over [i, i+L] for every fully usable interval.

    p1 = J1 w1 + C cos(a-b) w2,  dp1/dt = -C sin(a-b) w1 w2 + Q1
    p2 = J2 w2 + C cos(a-b) w1,  dp2/dt = +C sin(a-b) w1 w2 + Q2
    """
    a, b, w, z = prep.s.T
    dt = prep.run.dt
    u = shift(prep.u, delay, dt)
    c, sn = np.cos(a - b), np.sin(a - b)
    rel = z - w
    sw, sr = np.tanh(w / COULOMB_WIDTH), np.tanh(rel / COULOMB_WIDTH)
    G1, G2 = G * np.sin(a) - u * np.cos(a), G * np.sin(b) - u * np.cos(b)
    integ = lambda y: np.r_[0, np.cumsum((y[1:] + y[:-1]) * dt / 2)]
    I = {k: integ(v) for k, v in dict(sww=sn * w * z, G1=G1, G2=G2, w=w, rel=rel, sw=sw, sr=sr).items()}
    L = max(2, int(round(INTERVAL_S / dt)))
    bad = np.r_[0, np.cumsum(~prep.mask)]
    i = np.arange(0, len(a) - L, max(1, L // 4))
    i = i[bad[i + L + 1] - bad[i] == 0]
    j = i + L
    D = lambda y: y[j] - y[i]
    zero = np.zeros(len(i))
    # columns: J1, J2, C, d1, b1, b2, fc1, fc2   (d2 fixed -> right-hand side)
    r1 = np.column_stack([D(w), zero, D(c * z) + D(I["sww"]), -D(I["G1"]), D(I["w"]), -D(I["rel"]),
                          D(I["sw"]), -D(I["sr"])])
    r2 = np.column_stack([zero, D(z), D(c * w) - D(I["sww"]), zero, zero, D(I["rel"]), zero, D(I["sr"])])
    return np.vstack([r1, r2]), np.r_[zero, d2 * D(I["G2"])]


def regress(preps, delay, d2, coulomb):
    blocks = [rows(p, delay if p.run.kind == "c" else 0, d2) for p in preps]
    A = np.vstack([b[0] for b in blocks]); y = np.concatenate([b[1] for b in blocks])
    lo = np.array([1e-6, 1e-6, 0, 1e-5, 0, 0, 0, 0])
    hi = np.array([1, 1, 1, 1, 1, 1, 1, 1.])
    if not coulomb:
        hi[6:] = 1e-12
    scale = np.linalg.norm(A, axis=0) + 1e-12
    sol = lsq_linear(A / scale, y, bounds=(lo * scale, hi * scale))
    x = sol.x / scale
    p = dict(zip(("J1", "J2", "C", "d1", "b1", "b2", "fc1", "fc2"), x))
    p["d2"] = d2
    rms = np.sqrt(np.mean((A @ x - y) ** 2))
    return p, rms


# ------------------------------------------------------------- simulation ---
def windows(preps, length, stride):
    """Start indices of fully usable windows for multi-step validation."""
    out = []
    for p in preps:
        n = int(round(length / p.run.dt))
        for i in range(0, len(p.run.t) - n, int(round(stride / p.run.dt))):
            if p.mask[i: i + n].all():
                out.append((p, i, n))
    return out


def simulate_error(params, wins, delay):
    """Angle error (rad) of open-loop simulations from smoothed initial states."""
    errs = []
    for p, i, n in wins:
        u = shift(p.u, delay if p.run.kind == "c" else 0, p.run.dt)
        s = p.s[i].copy()
        sim = np.empty((n, 2))
        for k in range(n):
            sim[k] = s[:2]
            s = rk4(params, s, u[i + k], p.run.dt)
        errs.append(sim - np.column_stack([p.run.a[i: i + n], p.run.b[i: i + n]]))
    return np.concatenate(errs) if errs else np.zeros((0, 2))


def refine(params, wins, delay, coulomb):
    free = ["J1", "J2", "C", "d1", "b1", "b2"] + (["fc1", "fc2"] if coulomb else [])
    x0 = np.array([max(params[k], 1e-7) for k in free])

    def residual(logx):
        q = dict(params, **dict(zip(free, np.exp(logx))))
        return simulate_error(q, wins, delay).ravel()

    sol = least_squares(residual, np.log(x0), x_scale=1.0, max_nfev=60)
    return dict(params, **dict(zip(free, np.exp(sol.x))))


def rms_deg(params, wins, delay):
    e = simulate_error(params, wins, delay)
    return np.degrees(np.sqrt(np.mean(e ** 2, axis=0))) if len(e) else np.array([np.nan, np.nan])


# ---------------------------------------------------------------- reports ---
def check_model_terms(p):
    """Express the fit in check_model.py / model.js terms (B, S, length, T2, Tlocked)."""
    B, S = p["d2"], p["d1"] + p["d2"]
    return dict(B=B, S=S, length=p["C"] / B,
                T2=2 * np.pi * np.sqrt(p["J2"] / (B * G)),
                Tlocked=2 * np.pi * np.sqrt((p["J1"] + p["J2"] + 2 * p["C"]) / (S * G)))


def write_header(p, source):
    lit = lambda v: f"{float(v):.9e}f"   # always a valid float literal (no bare "0f")
    vals = ", ".join(lit(p[k]) for k in ("J1", "J2", "C", "d1", "d2", "b1", "b2", "fc1", "fc2"))
    HEADER.write_text(f"""// Pendulum model used by the state estimator (state_estimator.h).
// Generated by simulation/sysid/fit.py from: {source}
// Re-run the fit to update; the nominal (period-only) values live in check_model.py.
#pragma once

// Absolute link angles from up. H(q) q'' = gravity/cart terms - Coriolis - friction:
//   d1 = first moment of link 1 incl. link 2 mass at the elbow   [kg m]
//   d2 = first moment of link 2 about the elbow                   [kg m]
//   J1, J2 = inertias about the pivots, C = l1 * d2 coupling      [kg m^2]
//   b1 = pivot viscous, b2 = elbow viscous                        [N m s]
//   fc1, fc2 = Coulomb torques, smoothed by tanh(w / coulombWidth) [N m]
struct ModelParams {{ float g, J1, J2, C, d1, d2, b1, b2, fc1, fc2, coulombWidth; }};

constexpr ModelParams kModel{{{lit(G)}, {vals}, {lit(COULOMB_WIDTH)}}};
""", encoding="utf8")
    print(f"wrote {HEADER.relative_to(ROOT)}")


def fit(runs, args):
    nom = nominal()
    preps = [prepare(r, args.window, args.skip if r.kind == "f" else 0.) for r in runs]
    used = sum(int(p.mask.sum()) for p in preps)
    print(f"{len(runs)} runs, {used} usable samples "
          f"({', '.join(f'{p.run.name}:{p.run.kind}' for p in preps)})")
    if used < 400:
        raise SystemExit("Not enough usable samples; record longer or check flags.")
    has_chirp = any(p.run.kind == "c" for p in preps)
    delays = np.arange(0, .0305, .001) if has_chirp else [0.]
    fits = [(regress(preps, d, nom["d2"], not args.no_coulomb), d) for d in delays]
    (params, rms), delay = min(fits, key=lambda f: f[0][1])
    if has_chirp:
        print("input delay scan (ms: regression RMS):",
              ", ".join(f"{d * 1e3:.0f}:{f[1]:.2e}" for f, d in fits[::5]))
    wins = windows(preps, args.horizon, args.horizon / 2)
    if args.refine and wins:
        params = refine(params, wins[:: max(1, len(wins) // 60)], delay, not args.no_coulomb)

    print(f"\ninput delay: {delay * 1e3:.0f} ms (ramp velocity sample -> link response; "
          f"includes the half-frame hold)")
    print(f"{'param':>5} {'nominal':>12} {'fitted':>12}  ratio")
    for k in NAMES:
        ratio = f"{params[k] / nom[k]:.3f}" if nom[k] else "-"
        print(f"{k:>5} {nom[k]:12.6g} {params[k]:12.6g}  {ratio}")
    if wins:
        print(f"\nopen-loop angle RMS over {len(wins)} windows of {args.horizon:.2f} s (base, tip deg):")
        print(f"  nominal : {np.array2string(rms_deg(nom, wins, 0), precision=2)}")
        print(f"  fitted  : {np.array2string(rms_deg(params, wins, delay), precision=2)}")
    cm = check_model_terms(params)
    print("\nFor check_model.py / dd-du/model.js / dd-uu/design.py:")
    print("  " + ", ".join(f"{k}={v:.6g}" for k, v in cm.items()))
    print(f"  drag b1={params['b1']:.3g}, b2={params['b2']:.3g} N m s  (model.js deriv(..., drag))")
    print("Then rerun check_model.py, regenerate the gains/trajectories, and rebuild.")
    if args.write:
        write_header(params, ", ".join(Path(f).name for f in args.logs) or "selftest")
    return params, delay


# ---------------------------------------------------------------- selftest ---
def synth_log(params, kind, seconds, amp=1.2, delay=.008, seed=0):
    """Produce a firmware-format log from a known model, x4-quantised at 4000 cpr."""
    rng = np.random.default_rng(seed)
    dt, sub = .005, 5
    n = int(seconds / dt)
    s = np.array([np.pi, np.pi, 0, 0.]) + (np.array([.6, -.5, 0, 0]) if kind == "f" else 0)
    v = np.zeros(n); u = np.zeros(n)
    if kind == "c":
        t = np.arange(n) * dt
        T, f0, f1 = seconds - 4, .3, 5.
        ratio = f1 / f0
        ph = 2 * np.pi * f0 * T / np.log(ratio) * (ratio ** (np.minimum(t, T) / T) - 1)
        f = f0 * ratio ** (np.minimum(t, T) / T)
        fade = np.clip(np.minimum(t, T - t) / 1.5, 0, 1)
        X = fade * np.minimum(.08, amp / (2 * np.pi * f) ** 2)
        x = X * np.sin(ph)
        v = np.gradient(x, dt)
        u = np.gradient(v, dt)
    lag = int(round(delay / dt))
    rows = []
    for i in range(n):
        ui = u[i - lag] if kind == "c" and i >= lag else 0.
        cnt = lambda ang: np.floor(ang * 4000 / (2 * np.pi)) * 2 * np.pi / 4000
        base = cnt(s[0] - np.pi) + np.pi
        tip = base + cnt(s[1] - s[0])
        wrap = lambda q: (q + np.pi) % (2 * np.pi) - np.pi
        flag = 1 if rng.random() < .001 else 0
        rows.append(f"{i * 5000},{0:.6f},{v[i]:.6f},{v[i]:.6f},{wrap(base):.6f},{wrap(tip):.6f},{flag}")
        for _ in range(sub):
            s = rk4(params, s, ui, dt / sub)
    return (f"SYSID_CSV_BEGIN kind={kind} amp={amp:.2f} frame_us=5000 base_cpr=4000 rel_cpr=4000 "
            f"base_channel=0 reason=COMPLETE\nt_us,x_m,v_m_s,vcmd_m_s,base_rad,tip_rad,flags\n"
            + "\n".join(rows) + "\nSYSID_CSV_END\n")


def selftest(args):
    truth = nominal()
    truth.update(J1=truth["J1"] * 1.08, J2=truth["J2"] * .94, C=truth["C"] * 1.05, d1=truth["d1"] * .97,
                 b1=4e-5, b2=1.5e-5)
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        paths = []
        for kind, sec, seed in (("f", 20, 1), ("c", 20, 2)):
            path = Path(tmp) / f"synthetic_{kind}.log"
            path.write_text("boot noise\n" + synth_log(truth, kind, sec, seed=seed), encoding="utf8")
            paths.append(path)
        args.logs = [str(p) for p in paths]
        args.write = False
        params, delay = fit([r for p in paths for r in parse(p)], args)
    print("\nselftest truth vs fit:")
    ok = True
    for k in ("J1", "J2", "C", "d1"):
        err = params[k] / truth[k] - 1
        ok &= abs(err) < .03
        print(f"  {k}: {err * 100:+.2f}%")
    for k in ("b1", "b2"):
        print(f"  {k}: truth {truth[k]:.2e}, fit {params[k]:.2e}")
        ok &= abs(params[k] - truth[k]) < .5 * truth[k]
    # 8 ms -> 2 whole frames of lag, plus half a frame from holding u over each frame.
    print(f"  delay: truth 12.5 ms effective, fit {delay * 1e3:.0f} ms")
    ok &= abs(delay - .0125) <= .003
    print("SELFTEST", "PASS" if ok else "FAIL")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="*", help="serial logs containing SYSID_CSV blocks")
    ap.add_argument("--write", action="store_true", help="rewrite src/double_pendulum/model_params.h")
    ap.add_argument("--refine", action="store_true", help="refine by simulation error (slower)")
    ap.add_argument("--no-coulomb", action="store_true", help="viscous friction only")
    ap.add_argument("--window", type=int, default=15, help="Savitzky-Golay window, samples (default 15 = 75 ms)")
    ap.add_argument("--skip", type=float, default=.5, help="seconds dropped at the start of free-decay runs")
    ap.add_argument("--horizon", type=float, default=.6, help="validation window, s")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        sys.exit(0 if selftest(args) else 1)
    if not args.logs:
        ap.error("give log files or --selftest")
    runs = [r for path in args.logs for r in parse(path)]
    if not runs:
        raise SystemExit("No SYSID_CSV blocks found.")
    fit(runs, args)


if __name__ == "__main__":
    main()
