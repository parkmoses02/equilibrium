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
A = np.zeros((6, 6))
A[:3, 3:] = np.eye(3)
Q = np.diag([12, 60, 60, 2, 2, 2])
code = Path(__file__).with_name("upright.h").read_text(encoding="utf8")
for key, s1, s2 in [('q',1,1), ('w',1,-1), ('e',-1,1)]:
    # cos(theta1-theta2) signs the coupling; each cos(theta_i) signs
    # both gravity and cart-acceleration forcing in absolute coordinates.
    c = C*s1*s2
    H = np.array([[I1,c],[c,I2]])
    ds = d*[s1,s2]
    A[4:,1:3] = np.linalg.solve(H,g*np.diag(ds))
    b = np.r_[0,0,0,1,np.linalg.solve(H,-ds)][:,None]
    gain = (b.T @ solve_continuous_are(A,b,Q,[[1]]))[0]
    print(key, ','.join(f'{v:.6f}f' for v in gain))
    match = re.search(rf"K_{key}\[\]=\{{([^}}]+)",code)
    if match is None:
        raise AssertionError(f"Missing firmware gain for {key}")
    stored = [float(v.rstrip('f')) for v in match[1].split(',')]
    np.testing.assert_allclose(stored,gain,rtol=0,atol=1e-5)
    assert np.linalg.eigvalsh(H).min()>0 and max(np.linalg.eigvals(A-b@np.array([stored])).real)<0
print("PASS: all three equilibrium gains and nominal poles")
print(f"T2={T2:.6f} s, Tlocked={Tlocked:.6f} s, B={B:g} kg m, S={S:g} kg m")
print(f"I1={I1:.8g}, I2={I2:.8g}, C={C:.8g} kg m^2")
print("Nominal linear model only: excludes friction, actuator saturation and delay.")
