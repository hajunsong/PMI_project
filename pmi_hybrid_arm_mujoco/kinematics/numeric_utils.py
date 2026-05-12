"""Numerical differentiation for joint-angle trajectories."""

from __future__ import annotations

import numpy as np


def central_difference_joint_series(q_series: np.ndarray, dt: float) -> np.ndarray:
    """``q_series``: (N, 4); return qdot estimates same shape."""
    q = np.asarray(q_series, dtype=np.float64)
    n = q.shape[0]
    if n == 1:
        return np.zeros_like(q)
    dt = float(max(dt, 1e-12))
    qd = np.zeros_like(q)
    qd[0] = (q[1] - q[0]) / dt
    qd[-1] = (q[-1] - q[-2]) / dt
    for k in range(1, n - 1):
        qd[k] = (q[k + 1] - q[k - 1]) / (2.0 * dt)
    return qd


def moving_average_columns(x: np.ndarray, iterations: int) -> np.ndarray:
    if iterations <= 0:
        return x.copy()
    y = x.astype(np.float64).copy()
    kernel = np.array([1.0, 1.0, 1.0], dtype=np.float64) / 3.0
    for _ in range(int(iterations)):
        padded = np.vstack([y[0:1], y, y[-1:]])
        new_y = np.zeros_like(y)
        for i in range(y.shape[0]):
            patch = padded[i : i + 3]
            new_y[i] = (kernel.reshape(-1, 1) * patch).sum(axis=0)
        y = new_y
    return y
