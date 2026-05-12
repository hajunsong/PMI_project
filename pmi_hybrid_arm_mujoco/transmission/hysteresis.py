"""Bouc–Wen scalar hysteresis (auxiliary internal state z)."""


from dataclasses import dataclass

import numpy as np


@dataclass
class BoucWenParams:
    alpha: float
    A: float
    beta: float
    gamma: float
    n: float
    z_clip: float


class BoucWenHysteresis:
    """
    Auxiliary state dynamics:
      z_dot = A*xdot - beta*|xdot|*|z|^(n-1)*z - gamma*xdot*|z|^n
    Hysteresis contribution to force: fh = alpha * z
    """

    def __init__(self, p: BoucWenParams):
        self.p = p
        self.z = 0.0

    def reset(self) -> None:
        self.z = 0.0

    def force(self, xdot: float, dt: float) -> tuple[float, float]:
        """
        Advances z via semi-implicit Euler; returns (hysteresis_force, z_used).
        """
        p = self.p
        n = float(p.n)
        z = float(self.z)
        xd = float(xdot)

        zn = abs(z) + 1e-12
        z_pow_n = zn**n
        z_pow_nm1 = zn ** max(n - 1.0, 0.0)

        zdot = p.A * xd - p.beta * abs(xd) * z_pow_nm1 * z - p.gamma * xd * z_pow_n
        z_next = np.clip(z + zdot * float(dt), -abs(p.z_clip), abs(p.z_clip))
        self.z = float(z_next)
        return float(p.alpha) * float(self.z), float(self.z)
