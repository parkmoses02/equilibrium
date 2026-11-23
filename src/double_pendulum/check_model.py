"""Offline commissioning check: python src/double_pendulum/check_model.py (numpy/scipy)."""
from pathlib import Path
import re
import numpy as np
from scipy.linalg import solve_continuous_are

g, B, S, length, T2, Tlocked = 9.80665, .002475, .0128, .16, .792172, .930958
C = length * B
I2 = B * g * (T2 / (2*np.pi))**2
I1 = S * g * (Tlocked / (2*np.pi))**2 - I2 - 2*C
d = np.array([S-B, B])
# Both angles absolute, upright=0, positive tilt toward positive cart travel.
# H theta_ddot = g diag(d) theta - d x_ddot. No damping measurement yet.
UU_UD_Q = (12, 60, 60, 2, 2, 2)
DU_Q = (220, 500, 900, 60, 8, 4)
DEFAULT_Q_BY_POSE = {'q': UU_UD_Q, 'w': UU_UD_Q, 'e': DU_Q}
DEFAULT_R = 1
POSE_SIGNS = {'q': (1, 1), 'w': (1, -1), 'e': (-1, 1)}


def linear_model(key):
    s1, s2 = POSE_SIGNS[key]
    A = np.zeros((6, 6))
    A[:3, 3:] = np.eye(3)
    # cos(theta1-theta2) signs the coupling; each cos(theta_i) signs
    # both gravity and cart-acceleration forcing in absolute coordinates.
    c = C*s1*s2
    H = np.array([[I1,c],[c,I2]])
    ds = d*[s1,s2]
    A[4:,1:3] = np.linalg.solve(H,g*np.diag(ds))
    b = np.r_[0,0,0,1,np.linalg.solve(H,-ds)][:,None]
    assert np.linalg.eigvalsh(H).min() > 0
    return A, b


def feedback_gain(key, q_diagonal=None, r=DEFAULT_R):
    if q_diagonal is None:
        q_diagonal = DEFAULT_Q_BY_POSE[key]
    q = np.asarray(q_diagonal, dtype=float)
    if q.shape != (6,) or not np.all(np.isfinite(q)) or np.any(q < 0):
        raise ValueError("Q의 대각 성분 6개에 각각 0 이상의 유한한 수를 입력하세요.")
    if not np.isfinite(r) or r <= 0:
        raise ValueError("R에 0보다 큰 유한한 수를 입력하세요.")
    if q[0] == 0:
        raise ValueError("카트 위치를 안정화하려면 위치 가중치를 0보다 크게 입력하세요.")
    A, b = linear_model(key)
    P = solve_continuous_are(A, b, np.diag(q), [[r]])
    gain = (b.T @ P) / r
    if not np.isfinite(gain).all() or np.max(np.linalg.eigvals(A - b @ gain).real) >= 0:
        raise np.linalg.LinAlgError("No stabilizing gain")
    return gain[0]


def main():
    code = Path(__file__).with_name("upright.h").read_text(encoding="utf8")
    for key in POSE_SIGNS:
        gain = feedback_gain(key)
        print(key, ','.join(f'{v:.6f}f' for v in gain))
        match = re.search(rf"K_{key}\[\]=\{{([^}}]+)",code)
        if match is None:
            raise AssertionError(f"Missing firmware gain for {key}")
        stored = [float(v.rstrip('f')) for v in match[1].split(',')]
        np.testing.assert_allclose(stored, gain, rtol=0, atol=1e-5)
        A, b = linear_model(key)
        assert max(np.linalg.eigvals(A - b @ np.array([stored])).real) < 0
    print("PASS: all three equilibrium gains and nominal poles")
    print(f"T2={T2:.6f} s, Tlocked={Tlocked:.6f} s, B={B:g} kg m, S={S:g} kg m")
    print(f"I1={I1:.8g}, I2={I2:.8g}, C={C:.8g} kg m^2")
    print("Nominal linear model only: excludes friction, actuator saturation and delay.")


if __name__ == "__main__":
    main()
