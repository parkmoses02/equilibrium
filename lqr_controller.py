"""LQR Controller for Double Inverted Pendulum."""

from __future__ import annotations

import numpy as np
import scipy.linalg as la


class LQRController:
    """LQR Controller that computes optimal gain and control command."""

    def __init__(
        self,
        A: np.ndarray,
        B: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray,
        u_max: float,
        u_quant: float,
    ) -> None:
        """Initialize LQR Controller by computing gain K.

        Args:
            A: System matrix (6x6).
            B: Input matrix (6x1).
            Q: State weight matrix (6x6).
            R: Input weight matrix (1x1).
            u_max: Maximum control input magnitude.
            u_quant: Quantization resolution for control input.
        """
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.u_max = u_max
        self.u_quant = u_quant

        # Solve continuous algebraic Riccati equation
        self.P = la.solve_continuous_are(self.A, self.B, self.Q, self.R)
        # Compute optimal feedback gain K (1x6)
        self.K = la.inv(self.R) @ self.B.T @ self.P

    def compute_input(self, x: np.ndarray) -> float:
        """Compute the control input u = -K * x with saturation and quantization.

        Args:
            x: Current state vector (6,).

        Returns:
            u: Constrained control command.
        """
        # self.K has shape (1, 6), x has shape (6,). The product is shape (1,).
        u_cmd = -float((self.K @ x)[0])
        # Saturation
        u_cmd = np.clip(u_cmd, -self.u_max, self.u_max)
        # Quantization
        if self.u_quant > 0:
            u_cmd = self.u_quant * np.round(u_cmd / self.u_quant)
        return float(u_cmd)
