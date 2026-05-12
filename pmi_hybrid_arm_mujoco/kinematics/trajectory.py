"""Quintic (5th-order) blending between Cartesian waypoints."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


def _quintic_pos(s: float) -> float:
    s = float(s)
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def _quintic_vel(s: float) -> float:
    s = float(s)
    return 30 * s**2 - 60 * s**3 + 30 * s**4


def _quintic_acc(s: float) -> float:
    s = float(s)
    return 60 * s - 180 * s**2 + 120 * s**3


@dataclass(frozen=True)
class WaypointXYZ:
    t: float
    x: float
    y: float
    z: float


class CartesianQuinticPath:
    """Zero endpoint velocity/acceleration quintic blend between waypoints (spec formula)."""

    def __init__(self, waypoints: list[WaypointXYZ]) -> None:
        if len(waypoints) < 2:
            raise ValueError("need at least two waypoints")
        self.wp = sorted(waypoints, key=lambda w: w.t)

    @classmethod
    def from_yaml(cls, rows: list[dict[str, Any]]) -> CartesianQuinticPath:
        pts = [WaypointXYZ(float(r["t"]), float(r["x"]), float(r["y"]), float(r["z"])) for r in rows]
        return cls(pts)

    def segment_index(self, t: float) -> int:
        if t <= self.wp[0].t:
            return 0
        for i in range(len(self.wp) - 1):
            if self.wp[i].t <= t <= self.wp[i + 1].t:
                return i
        return len(self.wp) - 2

    def sample(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Vectors (3,) ``p``, ``pdot``, ``pddot`` in world Cartesian."""
        wp = self.wp
        if t <= wp[0].t:
            p = np.array([wp[0].x, wp[0].y, wp[0].z], dtype=np.float64)
            return p, np.zeros(3), np.zeros(3)
        if t >= wp[-1].t:
            p = np.array([wp[-1].x, wp[-1].y, wp[-1].z], dtype=np.float64)
            return p, np.zeros(3), np.zeros(3)

        i = self.segment_index(t)
        a = wp[i]
        b = wp[i + 1]
        t0, t1 = float(a.t), float(b.t)
        T = max(t1 - t0, 1e-9)
        s = (float(t) - t0) / T
        s = min(1.0, max(0.0, s))
        g = _quintic_pos(s)
        dg_ds = _quintic_vel(s)
        d2g_ds2 = _quintic_acc(s)

        p0 = np.array([a.x, a.y, a.z], dtype=np.float64)
        p1 = np.array([b.x, b.y, b.z], dtype=np.float64)
        dp = p1 - p0

        pos = p0 + dp * g
        # dp/dt = dp * dg/dt = dp * (dg_ds / T)
        vel = dp * (dg_ds / T)
        acc = dp * (d2g_ds2 / (T**2))

        return pos, vel.astype(np.float64), acc.astype(np.float64)
