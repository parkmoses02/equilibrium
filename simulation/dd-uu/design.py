"""Design DD -> UU or DD -> UD using constrained multiple shooting (CasADi/IPOPT, NumPy, SciPy).

python simulation/dd-uu/design.py
python simulation/dd-uu/design.py ud
python simulation/dd-uu/design.py uu --fitted --x-limit .16 --out <file>
python simulation/dd-uu/design.py ud-du --fitted        # UD hold -> DU (ud_du_frames.h)
python simulation/dd-uu/design.py du-ud --fitted        # DU hold -> UD (du_ud_frames.h)
python simulation/dd-uu/design.py uu-ud --fitted        # UU hold -> UD (uu_ud_frames.h)
python simulation/dd-uu/design.py uu-du --fitted        # UU hold -> DU (uu_du_frames.h)
python simulation/dd-uu/design.py du-uu --fitted        # DU hold -> UU (du_uu_frames.h)
python simulation/dd-uu/design.py ud-uu --fitted --x-limit .15   # UD hold -> UU (ud_uu_frames.h)
python simulation/dd-uu/design.py uu-dd --fitted --x-limit .15   # soft stop (x): UU -> DD
python simulation/dd-uu/design.py ud-dd --fitted --x-limit .18   # soft stop (x): UD -> DD
python simulation/dd-uu/design.py du-dd --fitted                 # soft stop (x): DU -> DD
Only needed to redesign the table, not to build the firmware.

--fitted uses the sysid model (src/double_pendulum/model_params.h) including
pivot/elbow friction, in the same form as state_estimator.h, instead of the
frictionless nominal model from check_model.py.
"""
import argparse
import re
import sys
from pathlib import Path

import casadi as ca
import numpy as np
from scipy.linalg import solve_discrete_are

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from src.double_pendulum.check_model import B, C, I1, I2, S, UU_UD_Q, g

DT = .005
STRIDE = 8
KNOTS = 90


NOMINAL = dict(J1=I1, J2=I2, C=C, d1=S-B, d2=B, b1=0., b2=0., fc1=0., fc2=0., width=.05)


def fitted_params():
    text = (ROOT/"src/double_pendulum/model_params.h").read_text()
    values = [float(v.strip().rstrip("f")) for v in re.search(r"kModel\{([^}]*)\}", text)[1].split(",")]
    return dict(zip(("g", "J1", "J2", "C", "d1", "d2", "b1", "b2", "fc1", "fc2", "width"), values))


def plant(p=NOMINAL):
    state = ca.SX.sym("state", 6)
    accel = ca.SX.sym("accel")
    _, base, tip, v, wb, wt = ca.vertsplit(state)
    h, s = p["C"]*ca.cos(base-tip), p["C"]*ca.sin(base-tip)
    rel = wt - wb
    # Elbow friction acts equally and oppositely on the two links (as state_estimator.h).
    elbow = p["b2"]*rel + p["fc2"]*ca.tanh(rel/p["width"])
    f = (p["d1"]*(g*ca.sin(base)-accel*ca.cos(base)) - s*wt**2 + elbow
         - p["b1"]*wb - p["fc1"]*ca.tanh(wb/p["width"]))
    k = p["d2"]*(g*ca.sin(tip)-accel*ca.cos(tip)) + s*wb**2 - elbow
    I1, I2 = p["J1"], p["J2"]
    det = I1*I2-h*h
    rhs = ca.Function("rhs", [state, accel], [
        ca.vertcat(v, wb, wt, accel, (I2*f-h*k)/det, (I1*k-h*f)/det)])
    a = rhs(state, accel)
    b = rhs(state + DT/2*a, accel)
    c = rhs(state + DT/2*b, accel)
    d = rhs(state + DT*c, accel)
    end = state + DT/6*(a+2*b+2*c+d)
    step = ca.Function("step", [state, accel], [end])
    jac = ca.Function("step_jacobian", [state, accel],
                      [ca.jacobian(end, state), ca.jacobian(end, accel)])
    return step, jac


DD = np.array([0, np.pi, np.pi, 0, 0, 0])


def optimize(step, target, x_limit=.24, warm_uu=False, start=DD, guess=None, rate_weight=0.):
    opt = ca.Opti()
    states = opt.variable(6, KNOTS+1)
    controls = opt.variable(1, KNOTS)
    opt.subject_to(states[:, 0] == start)
    opt.subject_to(states[:, -1] == target)
    opt.subject_to(opt.bounded(-x_limit, states[0, :], x_limit))
    opt.subject_to(opt.bounded(-.72, states[3, :], .72))
    opt.subject_to(opt.bounded(-13, controls, 13))
    chunk = step.mapaccum(STRIDE)
    for i in range(KNOTS):
        path = chunk(states[:, i], ca.repmat(controls[i], 1, STRIDE))
        opt.subject_to(states[:, i+1] == path[:, -1])
    opt.minimize(ca.sumsqr(controls) + .3*ca.sumsqr(ca.diff(controls.T))
                 + 100*ca.sumsqr(states[0, :]) + 2*ca.sumsqr(states[3, :])
                 + rate_weight*ca.sumsqr(states[4:, :]))
    t = np.linspace(0, 1, KNOTS+1)
    if guess is not None:
        for j in (1, 2):
            opt.set_initial(states[j, :], guess(j, t))
    elif warm_uu:
        # Start from the existing (frictionless) UU table and let the optimizer
        # re-shape it for the fitted model.
        text = (ROOT/"src/double_pendulum/uu_frames.h").read_text()
        ref = np.array([[float(s.rstrip("f")) for s in row.split(",")[:5]]
                        for row in re.findall(r"^\s*\{([^{}]+)\},", text, re.M)])
        seed = ref[:STRIDE*KNOTS+1:STRIDE].copy()
        opt.set_initial(states[:4, :], seed[:, :4].T)
        opt.set_initial(states[4:, :], np.gradient(np.unwrap(seed[:, 1:3], axis=0), STRIDE*DT, axis=0).T)
        opt.set_initial(controls, seed[:-1, 4])
    elif target[2] == np.pi:
        # The UU path already raises the base. Warm-start its swing and let
        # the optimizer replan the terminal tip motion to finish hanging.
        text = (ROOT/"src/double_pendulum/uu_frames.h").read_text()
        ref = np.array([[float(s.rstrip("f")) for s in row.split(",")[:5]]
                        for row in re.findall(r"^\s*\{([^{}]+)\},", text, re.M)])
        seed = ref[:STRIDE*KNOTS+1:STRIDE].copy()
        seed[:, 2] += np.pi*np.clip((t-.68)/.32, 0, 1)
        opt.set_initial(states[:4, :], seed[:, :4].T)
        opt.set_initial(states[4:, :], np.gradient(seed[:, 1:3], STRIDE*DT, axis=0).T)
        opt.set_initial(controls, seed[:-1, 4])
    else:
        for j in (1, 2):
            opt.set_initial(states[j, :], np.pi*(1-t) + (1 if j==1 else -1)*np.sin(4*np.pi*t))
    opt.solver("ipopt", {"expand": False, "print_time": False},
               {"print_level": 0, "max_iter": 1800, "tol": 1e-8})
    sol = opt.solve()
    return sol.value(states).T, sol.value(controls)


def frames(step, jac, knots, controls, target):
    reference = []
    for state, u in zip(knots, controls):
        for _ in range(STRIDE):
            reference.append((state, u))
            state = np.asarray(step(state, u)).ravel()
    # A short dwell at the target gives the angular-rate filter time to settle.
    reference.extend([(target, 0.)]*24)
    Q, R = DT*np.diag(UU_UD_Q), DT
    A, b = map(np.asarray, jac(target, 0.))
    P = solve_discrete_are(A, b, Q, [[R]])
    rows = []
    for state, u in reversed(reference):
        A, b = map(np.asarray, jac(state, u))
        gain = np.linalg.solve([[R+(b.T@P@b).item()]], b.T@P@A)
        P = Q + A.T@P@(A-b@gain)
        P = (P+P.T)/2
        rows.append([*state[:4], u, *gain[0]])
    return rows[::-1]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pose", choices=("uu", "ud", "ud-du", "du-ud", "uu-ud", "uu-du", "du-uu", "ud-uu", "uu-dd", "ud-dd", "du-dd"), default="uu", nargs="?")
    parser.add_argument("--fitted", action="store_true", help="sysid model with friction (model_params.h)")
    parser.add_argument("--x-limit", type=float, default=.24, help="cart travel bound for the knots, m")
    parser.add_argument("--out", type=Path, help="output header (default: src/double_pendulum/<pose>_frames.h)")
    args = parser.parse_args()
    pose = args.pose
    target = np.zeros(6)
    start, guess = DD, None
    if pose == "ud":
        target[2] = np.pi
    if pose == "ud-du":
        # UD (base up, tip down) -> DU. Cheapest solution found: the folded pair
        # turns 180 deg as one body (base 0 -> +pi, tip pi -> 2pi), ~9 cm of cart travel.
        start = np.array([0, 0, np.pi, 0, 0, 0])
        target = np.array([0, np.pi, 2*np.pi, 0, 0, 0])
        guess = lambda j, t: start[j] + (target[j]-start[j])*t + (-.8 if j == 1 else .8)*np.sin(2*np.pi*t)
    if pose == "du-ud":
        # DU -> UD, the same rigid turn of the folded pair (base pi -> 2pi, tip 0 -> pi):
        # ~9 cm of cart travel. A cheaper-looking option spins the elbow a full turn; avoided.
        start = np.array([0, np.pi, 0, 0, 0, 0])
        target = np.array([0, 2*np.pi, np.pi, 0, 0, 0])
        guess = lambda j, t: start[j] + (target[j]-start[j])*t + (-.8 if j == 1 else .8)*np.sin(2*np.pi*t)
    if pose == "uu-ud":
        # UU -> UD: the base stays up (swings < 20 deg) while the tip flips down (0 -> pi).
        start = np.array([0, 0, 0, 0, 0, 0])
        target = np.array([0, 0, np.pi, 0, 0, 0])
        guess = lambda j, t: start[j] + (target[j]-start[j])*t - (-.8 if j == 1 else .8)*np.sin(2*np.pi*t)
    if pose == "uu-du":
        # UU -> DU: base 0 -> pi while the tip turns once (0 -> 2pi) the same way, so the
        # elbow only turns pi. Slowest elbow of the options found (about 10 rad/s vs 29-42).
        start = np.array([0, 0, 0, 0, 0, 0])
        target = np.array([0, np.pi, 2*np.pi, 0, 0, 0])
        guess = lambda j, t: start[j] + (target[j]-start[j])*t
    if pose == "du-uu":
        # DU -> UU: base pi -> 2pi, tip 0 -> 2pi the same way. Of the solutions found this one
        # (elbow up to ~21 rad/s) tolerated +-2% inertia; the slower-elbow one did not
        # (simulation/sysid/sim-transition.cpp).
        start = np.array([0, np.pi, 0, 0, 0, 0])
        target = np.array([0, 2*np.pi, 2*np.pi, 0, 0, 0])
        guess = lambda j, t: start[j] + (target[j]-start[j])*t + (-.8 if j == 1 else .8)*np.sin(2*np.pi*t)
    if pose == "ud-uu":
        # UD -> UU, reverse of uu-ud: the base stays up while the tip swings up (pi -> 0).
        start = np.array([0, 0, np.pi, 0, 0, 0])
        target = np.array([0, 0, 0, 0, 0, 0])
        guess = lambda j, t: start[j] + (target[j]-start[j])*t + (-.8 if j == 1 else .8)*np.sin(2*np.pi*t)
    rate_weight = 0.
    if pose.endswith("-dd"):
        # Soft stop: bring a held pose down to DD with the least link motion the rail allows
        # (link rates are penalised). Chosen in simulation/sysid/sim-stop.cpp.
        b0, t0, tb, tt, sign = {"uu-dd": (0, 0, -np.pi, -np.pi, -1), "ud-dd": (0, np.pi, np.pi, np.pi, 0),
                                "du-dd": (np.pi, 0, np.pi, np.pi, 0)}[pose]
        start = np.array([0, b0, t0, 0, 0, 0])
        target = np.array([0, tb, tt, 0, 0, 0])
        guess = lambda j, t: start[j] + (target[j]-start[j])*t + sign*(-.8 if j == 1 else .8)*np.sin(2*np.pi*t)
        rate_weight = 2.
    params = fitted_params() if args.fitted else NOMINAL
    step, jac = plant(params)
    knots, controls = optimize(step, target, args.x_limit, warm_uu=args.fitted and pose == "uu",
                               start=start, guess=guess, rate_weight=rate_weight)
    # Gains: linearising Coulomb friction (tanh(w/width)) near w=0 gives a huge
    # fictitious damping and blows up the terminal LQR, so design them without it.
    _, gain_jac = plant({**params, "fc1": 0., "fc2": 0.})
    rows = frames(step, gain_jac, knots, controls, target)
    name = {"uu": "uu", "ud": "ud", "ud-du": "udDu", "du-ud": "duUd", "uu-ud": "uuUd", "uu-du": "uuDu", "du-uu": "duUu", "ud-uu": "udUu", "uu-dd": "uuDd", "ud-dd": "udDd", "du-dd": "duDd"}[pose]
    output = args.out or ROOT/f"src/double_pendulum/{pose.replace('-', '_')}_frames.h"
    with output.open("w", encoding="ascii", newline="\n") as f:
        model = "sysid-fitted model with friction" if args.fitted else "nominal frictionless model"
        f.write(f"// Generated by simulation/dd-uu/design.py ({model}, |x| <= {args.x_limit} m); SI units, 5 ms frames.\n"
                "#pragma once\n#include \"du_frames.h\"\n"
                f"static constexpr DuFrame {name}Frames[] = {{\n")
        for row in rows:
            f.write("    {" + ",".join(f"{v:.8f}f" for v in row) + "},\n")
        f.write("};\n")
    print(f"{len(rows)} frames, {len(rows)*DT:.3f} s; peak knot speed {max(abs(knots[:,3])):.3f} m/s")
