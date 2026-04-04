"""Double inverted pendulum dynamics template (Python + numpy).

Usage:
    .venv\\bin\\python.exe dynamics_template.py

The template computes:
    - M(q)
    - C(q, qdot)
    - G(q)
    - qddot
    - linearized A, B matrices

Replace the example coefficients with your identified physical parameters.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Params:
    m_total: float
    A: float
    B: float
    Cc: float
    I_link1: float
    I_link2: float
    k1: float
    k2: float


def mass_matrix(q: np.ndarray, p: Params) -> np.ndarray:
    """Return M(q) for q = [x, theta1, theta2]."""
    _, th1, th2 = q
    return np.array(
        [
            [p.m_total, p.A * np.cos(th1), p.B * np.cos(th2)],
            [p.A * np.cos(th1), p.I_link1, p.Cc * np.cos(th1 - th2)],
            [p.B * np.cos(th2), p.Cc * np.cos(th1 - th2), p.I_link2],
        ],
        dtype=float,
    )


def dM_dq(q: np.ndarray, p: Params) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return partial derivatives (dM/dx, dM/dtheta1, dM/dtheta2)."""
    _, th1, th2 = q

    dM_dx = np.zeros((3, 3), dtype=float)
    dM_dth1 = np.array(
        [
            [0.0, -p.A * np.sin(th1), 0.0],
            [-p.A * np.sin(th1), 0.0, -p.Cc * np.sin(th1 - th2)],
            [0.0, -p.Cc * np.sin(th1 - th2), 0.0],
        ],
        dtype=float,
    )
    dM_dth2 = np.array(
        [
            [0.0, 0.0, -p.B * np.sin(th2)],
            [0.0, 0.0, p.Cc * np.sin(th1 - th2)],
            [-p.B * np.sin(th2), p.Cc * np.sin(th1 - th2), 0.0],
        ],
        dtype=float,
    )
    return dM_dx, dM_dth1, dM_dth2


def coriolis_matrix(q: np.ndarray, qdot: np.ndarray, p: Params) -> np.ndarray:
    """Return C(q, qdot) so that C(q, qdot) @ qdot is the Coriolis/centrifugal term."""
    dM_dx, dM_dth1, dM_dth2 = dM_dq(q, p)
    dM = [dM_dx, dM_dth1, dM_dth2]
    Cmat = np.zeros((3, 3), dtype=float)

    for i in range(3):
        for j in range(3):
            c_ij = 0.0
            for k in range(3):
                c_ijk = 0.5 * (dM[k][i, j] + dM[j][i, k] - dM[i][j, k])
                c_ij += c_ijk * qdot[k]
            Cmat[i, j] = c_ij

    return Cmat


def gravity_vector(q: np.ndarray, p: Params) -> np.ndarray:
    """Return G(q)."""
    _, th1, th2 = q
    return np.array([0.0, -p.k1 * np.sin(th1), -p.k2 * np.sin(th2)], dtype=float)


def qddot(q: np.ndarray, qdot: np.ndarray, tau: np.ndarray, p: Params) -> np.ndarray:
    """Solve M(q) qddot = tau - C(q, qdot) qdot - G(q)."""
    M = mass_matrix(q, p)
    Cmat = coriolis_matrix(q, qdot, p)
    G = gravity_vector(q, p)
    rhs = tau - Cmat @ qdot - G
    return np.linalg.solve(M, rhs)


def state_derivative(x_state: np.ndarray, tau: np.ndarray, p: Params) -> np.ndarray:
    """Return xdot for state x = [q; qdot]."""
    q = x_state[:3]
    qdot_vec = x_state[3:]
    qddot_vec = qddot(q, qdot_vec, tau, p)
    return np.hstack([qdot_vec, qddot_vec])


def linearized_matrices(p: Params) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return the linearized matrices at theta1 = theta2 = 0."""
    M0 = np.array(
        [
            [p.m_total, p.A, p.B],
            [p.A, p.I_link1, p.Cc],
            [p.B, p.Cc, p.I_link2],
        ],
        dtype=float,
    )
    K = np.diag([0.0, p.k1, p.k2])
    Btau = np.eye(3, dtype=float)
    return M0, K, Btau


def state_space_linearized(p: Params) -> tuple[np.ndarray, np.ndarray]:
    """Return linearized A, B for Xdot = A X + B tau."""
    M0, K, Btau = linearized_matrices(p)
    zeros = np.zeros((3, 3), dtype=float)
    identity = np.eye(3, dtype=float)
    M0_inv = np.linalg.inv(M0)

    A = np.block(
        [
            [zeros, identity],
            [-M0_inv @ K, zeros],
        ]
    )
    B = np.vstack([zeros, M0_inv @ Btau])
    return A, B


if __name__ == "__main__":
    # Example values only. Replace with identified physical parameters.
    p = Params(
        m_total=2.0,
        A=0.25,
        B=0.10,
        Cc=0.05,
        I_link1=0.30,
        I_link2=0.12,
        k1=4.2,
        k2=1.8,
    )

    q = np.array([0.0, 0.05, -0.03], dtype=float)
    qdot = np.array([0.1, 0.0, 0.0], dtype=float)
    tau = np.array([0.0, 0.0, 0.0], dtype=float)

    M = mass_matrix(q, p)
    Cmat = coriolis_matrix(q, qdot, p)
    G = gravity_vector(q, p)
    qdd = qddot(q, qdot, tau, p)
    A_lin, B_lin = state_space_linearized(p)

    np.set_printoptions(precision=5, suppress=True)
    print("M(q)=\n", M)
    print("C(q,qdot)=\n", Cmat)
    print("G(q)=\n", G)
    print("qddot=\n", qdd)
    print("A_linear=\n", A_lin)
    print("B_linear=\n", B_lin)