from __future__ import annotations

import numpy as np


def _g(s: float) -> float:
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def _gd(s: float) -> float:
    return 30 * s**2 - 60 * s**3 + 30 * s**4


def _gdd(s: float) -> float:
    return 60 * s - 180 * s**2 + 120 * s**3


class JointQuinticPath:
    def __init__(self, t0: float, q0: np.ndarray, t1: float, q1: np.ndarray, t2: float, q2: np.ndarray):
        self.t0, self.t1, self.t2 = float(t0), float(t1), float(t2)
        self.q0 = np.asarray(q0, dtype=float).copy()
        self.q1 = np.asarray(q1, dtype=float).copy()
        self.q2 = np.asarray(q2, dtype=float).copy()

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        t = float(t)
        if t <= self.t0:
            return self.q0.copy(), np.zeros_like(self.q0), np.zeros_like(self.q0)
        if t >= self.t2:
            return self.q2.copy(), np.zeros_like(self.q2), np.zeros_like(self.q2)
        if t <= self.t1:
            ta, tb, qa, qb = self.t0, self.t1, self.q0, self.q1
        else:
            ta, tb, qa, qb = self.t1, self.t2, self.q1, self.q2
        T = max(tb - ta, 1e-9)
        s = min(1.0, max(0.0, (t - ta) / T))
        dq = qb - qa
        q = qa + dq * _g(s)
        qd = dq * (_gd(s) / T)
        qdd = dq * (_gdd(s) / (T * T))
        return q, qd, qdd


def scaled_joint_quintic(q0: np.ndarray, q1: np.ndarray, q2: np.ndarray, duration: float) -> JointQuinticPath:
    T = float(duration)
    return JointQuinticPath(0.0, q0, T * 0.5, q1, T, q2)
