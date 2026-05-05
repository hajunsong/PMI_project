"""``main.run_vsd`` 와 동일한 웨이포인트·5차 경로 생성 (``analysis/python/path_generation`` 재사용)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
_AP = _ROOT / "analysis/python"
if str(_AP) not in sys.path:
    sys.path.insert(0, str(_AP))

from path_generation import path_generation  # noqa: E402


def path_build_full_quintic(
    wp_t: np.ndarray,
    wp_vals: np.ndarray,
    h: float,
) -> np.ndarray:
    """``main._path_build_full_quintic`` 와 동일 → ``(N, 3)`` = pos, vel, acc."""
    wp_t = np.asarray(wp_t, dtype=float)
    wp_vals = np.asarray(wp_vals, dtype=float)
    wp_n = len(wp_t)
    parts = []
    for i in range(1, wp_n):
        seg = path_generation(
            float(wp_vals[i - 1]),
            float(wp_vals[i]),
            float(wp_t[i] - wp_t[i - 1]),
            0.0,
            h,
            full_quintic=True,
        )
        if i < wp_n - 1 and len(seg) > 0:
            seg = seg[:-1]
        parts.append(seg)
    return np.vstack(parts)


def default_run_vsd_waypoints():
    """``run_vsd`` 에 하드코딩된 웨이포인트."""
    wp_t = np.array([0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0], dtype=float)
    wp_x = np.array([-0.35, -0.25, 0.25, 0.35, 0.18, -0.18, -0.35], dtype=float)
    wp_y = np.array([0.15, -0.28, -0.28, 0.15, 0.37, 0.37, 0.15], dtype=float)
    wp_z = np.array([-0.2, -0.2, -0.2, -0.2, 0.13, 0.13, -0.2], dtype=float)
    return wp_t, wp_x, wp_y, wp_z


def build_run_vsd_trajectories(h: float = 0.001):
    """전체 3 s 궤적 스택 ( ``run_vsd`` 와 동일 조건)."""
    wp_t, wp_x, wp_y, wp_z = default_run_vsd_waypoints()
    sx = path_build_full_quintic(wp_t, wp_x, h)
    sy = path_build_full_quintic(wp_t, wp_y, h)
    sz = path_build_full_quintic(wp_t, wp_z, h)
    return sx, sy, sz
