"""
Task-space path planner — Python port of ``PMI_Server/src/path_planner.cpp`` / ``PathPlanner``.

- Quintic segments in :math:`x,y,z` vs absolute time ``t`` on waypoints
- Pre-pass IK (chain ``q`` through samples, snap path xyz to reachable FK)
- Orientation target fixed: roll = ``-π/2``, pitch = ``0`` (same 5D error as server)
- Default joint limits: jnt1 ``±π``, jnt2–4 ``±π/2`` (``path_planner.h``)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

from kinematics.pmi_chain import (
    JOINT_LIMIT_RAD_MAX,
    JOINT_LIMIT_RAD_MIN,
    fk_ee_pose_joint_rad,
    jacobian_5x4_joint_rad,
    wrap_to_pi,
)

_PI = np.pi
_ROLL_PITCH_TARGET = np.array([-_PI / 2.0, 0.0], dtype=np.float64)


@dataclass
class Waypoint:
    t: float
    x: float
    y: float
    z: float


@dataclass
class PathSample:
    x: float
    y: float
    z: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0


def _norm5(v: np.ndarray) -> float:
    return float(np.linalg.norm(v))


def quintic_path(p0: float, pf: float, tf: float, h: float) -> List[Tuple[float, float, float]]:
    """Returns list of (pos, vel, acc) — same polynomial as ``PathPlanner::quinticPath``."""
    if tf <= 0.0 or h <= 0.0:
        return []
    n = max(1, int(round(tf / h)))
    a0, a1, a2 = p0, 0.0, 0.0
    a3 = 10.0 * (pf - p0) / (tf**3)
    a4 = -15.0 * (pf - p0) / (tf**4)
    a5 = 6.0 * (pf - p0) / (tf**5)
    out: List[Tuple[float, float, float]] = []
    for i in range(n + 1):
        t = tf * float(i) / float(n)
        t2, t3, t4, t5 = t * t, t**3, t**4, t**5
        pos = a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5
        vel = a1 + 2.0 * a2 * t + 3.0 * a3 * t2 + 4.0 * a4 * t3 + 5.0 * a5 * t4
        acc = 2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t2 + 20.0 * a5 * t3
        out.append((pos, vel, acc))
    return out


def ik_solve_to(
    q: np.ndarray,
    sample: PathSample,
    q_min: np.ndarray,
    q_max: np.ndarray,
    *,
    max_iter: int = 200,
    err_tol: float = 1e-3,
    damping: float = 1e-7,
    alpha: float = 0.6,
    bound_eps: float = 1e-9,
) -> bool:
    """Damped least-squares IK; mutates ``q`` in place (same structure as ``PathPlanner::ikSolveTo``)."""
    qv = np.asarray(q, dtype=np.float64).reshape(4).copy()
    clamped = np.zeros(4, dtype=bool)

    for k in range(4):
        if qv[k] < q_min[k]:
            qv[k] = q_min[k]
        if qv[k] > q_max[k]:
            qv[k] = q_max[k]

    for _ in range(max_iter):
        ee, rpy = fk_ee_pose_joint_rad(qv)
        err = np.array(
            [
                sample.x - ee[0],
                sample.y - ee[1],
                sample.z - ee[2],
                wrap_to_pi(float(_ROLL_PITCH_TARGET[0] - rpy[0])),
                wrap_to_pi(float(_ROLL_PITCH_TARGET[1] - rpy[1])),
            ],
            dtype=np.float64,
        )
        if _norm5(err) < err_tol:
            q[:] = qv
            return True

        J = jacobian_5x4_joint_rad(qv)
        for k in range(4):
            if clamped[k]:
                J[:, k] = 0.0

        JJT = J @ J.T
        JJT[np.diag_indices(5)] += damping**2
        try:
            y = np.linalg.solve(JJT, err)
        except np.linalg.LinAlgError:
            y = np.linalg.lstsq(JJT, err, rcond=None)[0]
        dq = alpha * (J.T @ y)
        for k in range(4):
            if clamped[k]:
                dq[k] = 0.0

        new_clamp = False
        for k in range(4):
            if clamped[k]:
                continue
            qk = qv[k] + dq[k]
            qmin, qmax = float(q_min[k]), float(q_max[k])
            if qk > qmax + bound_eps:
                qv[k] = qmax
                clamped[k] = True
                new_clamp = True
            elif qk < qmin - bound_eps:
                qv[k] = qmin
                clamped[k] = True
                new_clamp = True
            else:
                qv[k] = qk
        if new_clamp:
            continue

    q[:] = qv
    ee, rpy = fk_ee_pose_joint_rad(qv)
    err = np.array(
        [
            sample.x - ee[0],
            sample.y - ee[1],
            sample.z - ee[2],
            wrap_to_pi(float(_ROLL_PITCH_TARGET[0] - rpy[0])),
            wrap_to_pi(float(_ROLL_PITCH_TARGET[1] - rpy[1])),
        ],
        dtype=np.float64,
    )
    return _norm5(err) < err_tol


class TaskPathPlanner:
    """Mirror of ``PathPlanner``: ``set_waypoints`` → ``plan(dt)`` → ``joint_playback_sequence``."""

    def __init__(self) -> None:
        self._wps: List[Waypoint] = []
        self._path: List[PathSample] = []
        self._plan_dt = 0.002
        self._q_min = JOINT_LIMIT_RAD_MIN.copy()
        self._q_max = JOINT_LIMIT_RAD_MAX.copy()
        self._last_plan_clipped = False
        self._last_plan_first_clipped_idx = 0
        self._last_plan_unconverged_count = 0

    def set_joint_limits(self, q_min: Sequence[float], q_max: Sequence[float]) -> None:
        self._q_min = np.asarray(q_min, dtype=np.float64).reshape(4)
        self._q_max = np.asarray(q_max, dtype=np.float64).reshape(4)

    def set_waypoints(self, waypoints: List[Waypoint]) -> None:
        self._wps = list(waypoints)
        self._path = []

    def plan(self, dt: float, q_start: np.ndarray) -> bool:
        """Build workspace path + pre-IK (same as ``PathPlanner::plan``). ``q_start`` is 4 joint rad."""
        self._path = []
        self._last_plan_clipped = False
        self._last_plan_first_clipped_idx = 0
        self._last_plan_unconverged_count = 0
        if len(self._wps) < 2 or dt <= 0.0:
            return False

        for i in range(1, len(self._wps)):
            a, b = self._wps[i - 1], self._wps[i]
            seg_t = b.t - a.t
            if seg_t <= 0.0:
                return False
            px = quintic_path(a.x, b.x, seg_t, dt)
            py = quintic_path(a.y, b.y, seg_t, dt)
            pz = quintic_path(a.z, b.z, seg_t, dt)
            n = min(len(px), len(py), len(pz))
            if n == 0:
                return False
            for k in range(n):
                if i < len(self._wps) - 1 and k == n - 1:
                    continue
                self._path.append(PathSample(px[k][0], py[k][0], pz[k][0], 0.0, 0.0, 0.0))

        if not self._path:
            return False

        q_work = np.asarray(q_start, dtype=np.float64).reshape(4).copy()
        for k, s in enumerate(self._path):
            desired_xyz = np.array([s.x, s.y, s.z], dtype=np.float64)
            converged = ik_solve_to(q_work, s, self._q_min, self._q_max)
            if not converged:
                self._last_plan_unconverged_count += 1
            ee, _ = fk_ee_pose_joint_rad(q_work)
            s.x, s.y, s.z = float(ee[0]), float(ee[1]), float(ee[2])
            if float(np.linalg.norm(ee - desired_xyz)) > 1e-3:
                if not self._last_plan_clipped:
                    self._last_plan_first_clipped_idx = k
                self._last_plan_clipped = True

        dt_sec = dt if len(self._path) > 1 else 0.0
        if dt_sec > 0.0:
            for k in range(len(self._path)):
                kp = k + 1 if k + 1 < len(self._path) else k
                km = k - 1 if k > 0 else k
                denom = dt_sec if (kp == k or km == k) else 2.0 * dt_sec
                self._path[k].vx = (self._path[kp].x - self._path[km].x) / denom
                self._path[k].vy = (self._path[kp].y - self._path[km].y) / denom
                self._path[k].vz = (self._path[kp].z - self._path[km].z) / denom

        self._plan_dt = float(dt)
        return True

    @property
    def path_samples(self) -> List[PathSample]:
        return self._path

    @property
    def last_plan_clipped(self) -> bool:
        return self._last_plan_clipped

    @property
    def last_plan_first_clipped_idx(self) -> int:
        return self._last_plan_first_clipped_idx

    @property
    def last_plan_unconverged_count(self) -> int:
        return self._last_plan_unconverged_count

    def joint_playback_sequence(self, q_start: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Same chaining as ``PathPlanner::step``: for each path sample, IK from current ``q``.
        Returns ``(q_traj, qdot_traj)`` both shape ``(N, 4)`` with central-diff velocity on joints.
        """
        qv = np.asarray(q_start, dtype=np.float64).reshape(4).copy()
        qs: List[np.ndarray] = []
        for s in self._path:
            ik_solve_to(qv, s, self._q_min, self._q_max)
            qs.append(qv.copy())
        Q = np.stack(qs, axis=0)
        N = Q.shape[0]
        if N == 0:
            return Q, Q
        dt = float(self._plan_dt)
        Qd = np.zeros_like(Q)
        if N == 1:
            return Q, Qd
        for k in range(N):
            kp = k + 1 if k + 1 < N else k
            km = k - 1 if k > 0 else k
            denom = dt if (kp == k or km == k) else 2.0 * dt
            Qd[k] = (Q[kp] - Q[km]) / denom
        return Q, Qd
