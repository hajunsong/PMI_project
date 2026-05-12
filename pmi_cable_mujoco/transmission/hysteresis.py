"""Bouc–Wen hysteresis for cable-driven joints (jnt2–jnt4)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class BoucWenState:
    """Scalar Bouc–Wen hysteresis driven by cable velocity :math:`\\dot x`.

    Differential equation:

        \\dot z = A \\dot x - \\beta |\\dot x| |z|^{n-1} z - \\gamma \\dot x |z|^n

        F_{hyst} = \\alpha z

    Integration uses semi-implicit Euler; ``z`` and output force are clipped.
    """

    alpha: float
    A: float
    beta: float
    gamma: float
    n: float
    z_clip: float

    def __post_init__(self) -> None:
        self.z: float = 0.0

    def reset(self, z0: float = 0.0) -> None:
        self.z = float(np.clip(z0, -self.z_clip, self.z_clip))

    def step(self, xdot: float, dt: float, max_force: float) -> float:
        """Advance internal state and return hysteresis force [N]."""
        z = self.z
        n = max(self.n, 1e-6)
        abs_z = abs(z)
        zn = abs_z**n

        if abs_z < 1e-12:
            zn1_z = 0.0
        else:
            zn1_z = (abs_z ** (n - 1.0)) * z

        z_dot = (
            self.A * xdot
            - self.beta * abs(xdot) * zn1_z
            - self.gamma * xdot * zn
        )
        z_next = z + dt * z_dot
        z_next = float(np.clip(z_next, -self.z_clip, self.z_clip))
        self.z = z_next

        fh = self.alpha * self.z
        return float(np.clip(fh, -max_force, max_force))
