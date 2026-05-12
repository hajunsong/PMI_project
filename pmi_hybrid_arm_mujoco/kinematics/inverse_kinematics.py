"""Numerical IK: weighted Cartesian position + roll/pitch; yaw unconstrained."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Literal

import mujoco as mj
import numpy as np
from scipy.optimize import least_squares

from kinematics.forward_kinematics import fk_ee_rp
from kinematics.orientation_utils import angle_error


@dataclass
class IKConfig:
    weights_pos: tuple[float, float, float]
    weight_roll: float
    weight_pitch: float
    damping: float
    regularization: float
    max_iterations: int
    tolerance_ftol: float
    joint_side_order: tuple[str, ...]


IKTaskFeasMode = Literal["xyz", "xyz_pitch", "xyz_roll_pitch"]


def _least_squares_ik_single(
    model: mj.MjModel,
    scratch: mj.MjData,
    p_des: np.ndarray,
    *,
    roll_des: float,
    pitch_des: float,
    ik: IKConfig,
    q_seed_x0: np.ndarray,
    bounds_lo: np.ndarray,
    bounds_hi: np.ndarray,
) -> tuple[np.ndarray, dict[str, Any]]:
    def fun(x: np.ndarray) -> np.ndarray:
        return residual_vector(
            x,
            p_des=p_des,
            roll_des=roll_des,
            pitch_des=pitch_des,
            model=model,
            scratch=scratch,
            ik=ik,
            q_seed=q_seed_x0.copy(),
        )

    res = least_squares(
        fun,
        x0=np.asarray(q_seed_x0, dtype=np.float64),
        bounds=(bounds_lo.astype(np.float64), bounds_hi.astype(np.float64)),
        loss="huber",
        ftol=max(ik.tolerance_ftol, 1e-12),
        xtol=max(ik.tolerance_ftol, 1e-12),
        gtol=max(ik.tolerance_ftol, 1e-12),
        max_nfev=max(500, int(ik.max_iterations) * 35),
        verbose=0,
    )
    geo = residual_vector(
        res.x,
        p_des=p_des,
        roll_des=roll_des,
        pitch_des=pitch_des,
        model=model,
        scratch=scratch,
        ik=ik,
        q_seed=q_seed_x0.copy(),
    )[:5]
    diag = {
        "ik_cost": float(np.dot(geo, geo)),
        "ik_norm_geom": float(np.linalg.norm(geo)),
        "message": getattr(res, "message", ""),
        "nfev": int(res.nfev),
        "success": bool(res.success),
    }
    return res.x.astype(np.float64).copy(), diag


def residual_vector(
    q: np.ndarray,
    *,
    p_des: np.ndarray,
    roll_des: float,
    pitch_des: float,
    model: mj.MjModel,
    scratch: mj.MjData,
    ik: IKConfig,
    q_seed: np.ndarray | None,
) -> np.ndarray:
    p_ee, roll_ee, pitch_ee, _yaw_unused = fk_ee_rp(
        model, scratch, q, list(ik.joint_side_order)
    )
    e_pos = p_des.flatten()[:3] - p_ee
    wp = ik.weights_pos
    roll_e = angle_error(float(roll_des), float(roll_ee))
    pitch_e = angle_error(float(pitch_des), float(pitch_ee))
    geo = np.array(
        [
            wp[0] * float(e_pos[0]),
            wp[1] * float(e_pos[1]),
            wp[2] * float(e_pos[2]),
            ik.weight_roll * roll_e,
            ik.weight_pitch * pitch_e,
        ],
        dtype=np.float64,
    )
    dq = np.asarray(q - (q_seed if q_seed is not None else np.zeros_like(q)), dtype=np.float64)
    scale = math.sqrt(max(ik.damping + ik.regularization, 1e-18))
    pen = scale * dq
    return np.concatenate([geo, pen])


def residual_vector_task_mode(
    q: np.ndarray,
    *,
    p_des: np.ndarray,
    roll_des: float,
    pitch_des: float,
    task_feas_mode: IKTaskFeasMode,
    model: mj.MjModel,
    scratch: mj.MjData,
    ik: IKConfig,
    q_seed: np.ndarray | None,
) -> np.ndarray:
    """IK 기하 잔차: ``xyz`` / ``xyz_pitch`` / ``xyz_roll_pitch``."""
    p_ee, roll_ee, pitch_ee, _yaw = fk_ee_rp(
        model, scratch, q, list(ik.joint_side_order)
    )
    e_pos = p_des.flatten()[:3] - p_ee
    wp = ik.weights_pos
    pos_w = np.array(
        [wp[0] * float(e_pos[0]), wp[1] * float(e_pos[1]), wp[2] * float(e_pos[2])],
        dtype=np.float64,
    )
    roll_e = angle_error(float(roll_des), float(roll_ee))
    pitch_e = angle_error(float(pitch_des), float(pitch_ee))

    if task_feas_mode == "xyz":
        geo = pos_w
    elif task_feas_mode == "xyz_pitch":
        geo = np.concatenate([pos_w, np.array([ik.weight_pitch * pitch_e], dtype=np.float64)])
    elif task_feas_mode == "xyz_roll_pitch":
        geo = np.concatenate(
            [
                pos_w,
                np.array([ik.weight_roll * roll_e, ik.weight_pitch * pitch_e], dtype=np.float64),
            ]
        )
    else:
        raise ValueError(task_feas_mode)

    dq = np.asarray(q - (q_seed if q_seed is not None else np.zeros_like(q)), dtype=np.float64)
    scale = math.sqrt(max(ik.damping + ik.regularization, 1e-18))
    pen = scale * dq
    return np.concatenate([geo, pen])


def _least_squares_ik_task_single(
    model: mj.MjModel,
    scratch: mj.MjData,
    p_des: np.ndarray,
    *,
    roll_des: float,
    pitch_des: float,
    task_feas_mode: IKTaskFeasMode,
    ik: IKConfig,
    q_seed_x0: np.ndarray,
    bounds_lo: np.ndarray,
    bounds_hi: np.ndarray,
) -> tuple[np.ndarray, dict[str, Any]]:
    def fun(x: np.ndarray) -> np.ndarray:
        return residual_vector_task_mode(
            x,
            p_des=p_des,
            roll_des=roll_des,
            pitch_des=pitch_des,
            task_feas_mode=task_feas_mode,
            model=model,
            scratch=scratch,
            ik=ik,
            q_seed=q_seed_x0.copy(),
        )

    res = least_squares(
        fun,
        x0=np.asarray(q_seed_x0, dtype=np.float64),
        bounds=(bounds_lo.astype(np.float64), bounds_hi.astype(np.float64)),
        loss="huber",
        ftol=max(ik.tolerance_ftol, 1e-12),
        xtol=max(ik.tolerance_ftol, 1e-12),
        gtol=max(ik.tolerance_ftol, 1e-12),
        max_nfev=max(500, int(ik.max_iterations) * 35),
        verbose=0,
    )
    tail = residual_vector_task_mode(
        res.x,
        p_des=p_des,
        roll_des=roll_des,
        pitch_des=pitch_des,
        task_feas_mode=task_feas_mode,
        model=model,
        scratch=scratch,
        ik=ik,
        q_seed=q_seed_x0.copy(),
    )
    gdim = {"xyz": 3, "xyz_pitch": 4, "xyz_roll_pitch": 5}[task_feas_mode]
    geo = tail[:gdim]
    diag = {
        "ik_cost": float(np.dot(geo, geo)),
        "ik_norm_geom": float(np.linalg.norm(geo)),
        "message": getattr(res, "message", ""),
        "nfev": int(res.nfev),
        "success": bool(res.success),
    }
    return res.x.astype(np.float64).copy(), diag


def solve_ik_task_mode(
    model: mj.MjModel,
    scratch: mj.MjData,
    p_des: np.ndarray,
    *,
    roll_des: float,
    pitch_des: float,
    task_feas_mode: IKTaskFeasMode,
    ik: IKConfig,
    q_seed: np.ndarray,
    bounds_lo: np.ndarray,
    bounds_hi: np.ndarray,
    multistart_random_trials: int = 8,
    multistart_if_geom_above: float = 0.05,
    rng: np.random.Generator | None = None,
) -> tuple[np.ndarray, dict[str, Any]]:
    q0, diag0 = _least_squares_ik_task_single(
        model,
        scratch,
        p_des,
        roll_des=roll_des,
        pitch_des=pitch_des,
        task_feas_mode=task_feas_mode,
        ik=ik,
        q_seed_x0=q_seed,
        bounds_lo=bounds_lo,
        bounds_hi=bounds_hi,
    )
    best_q, best_d = q0, diag0

    need_ms = multistart_random_trials > 0 and float(diag0["ik_norm_geom"]) > float(multistart_if_geom_above)
    if need_ms:
        gen = rng if rng is not None else np.random.default_rng(0)
        lo = bounds_lo.astype(np.float64)
        hi = bounds_hi.astype(np.float64)
        for _ in range(int(multistart_random_trials)):
            x0 = gen.uniform(lo, hi)
            q_try, d_try = _least_squares_ik_task_single(
                model,
                scratch,
                p_des,
                roll_des=roll_des,
                pitch_des=pitch_des,
                task_feas_mode=task_feas_mode,
                ik=ik,
                q_seed_x0=x0,
                bounds_lo=bounds_lo,
                bounds_hi=bounds_hi,
            )
            if float(d_try["ik_cost"]) < float(best_d["ik_cost"]) - 1e-14:
                best_q, best_d = q_try, d_try

    return best_q, best_d


def solve_ik(
    model: mj.MjModel,
    scratch: mj.MjData,
    p_des: np.ndarray,
    *,
    roll_des: float,
    pitch_des: float,
    ik: IKConfig,
    q_seed: np.ndarray,
    bounds_lo: np.ndarray,
    bounds_hi: np.ndarray,
    multistart_random_trials: int = 10,
    multistart_if_geom_above: float = 0.05,
    rng: np.random.Generator | None = None,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Return (q_opt, diagnostics). Uses ``scipy.optimize.least_squares``.

    If the primary residual norm exceeds ``multistart_if_geom_above``, up to
    ``multistart_random_trials`` additional optimizations are launched from random
    initial poses inside ``bounds_*`` so the chained warm-start IK does not get
    trapped in Euler-related local minima on this 4R arm.

    Passing ``multistart_random_trials=0`` restores a single solve from ``q_seed``.
    """

    q0, diag0 = _least_squares_ik_single(
        model,
        scratch,
        p_des,
        roll_des=roll_des,
        pitch_des=pitch_des,
        ik=ik,
        q_seed_x0=q_seed,
        bounds_lo=bounds_lo,
        bounds_hi=bounds_hi,
    )
    best_q, best_d = q0, diag0

    need_ms = multistart_random_trials > 0 and float(diag0["ik_norm_geom"]) > float(multistart_if_geom_above)
    if need_ms:
        gen = rng if rng is not None else np.random.default_rng(0)
        lo = bounds_lo.astype(np.float64)
        hi = bounds_hi.astype(np.float64)
        for _ in range(int(multistart_random_trials)):
            x0 = gen.uniform(lo, hi)
            q_try, d_try = _least_squares_ik_single(
                model,
                scratch,
                p_des,
                roll_des=roll_des,
                pitch_des=pitch_des,
                ik=ik,
                q_seed_x0=x0,
                bounds_lo=bounds_lo,
                bounds_hi=bounds_hi,
            )
            if (
                float(d_try["ik_cost"]) < float(best_d["ik_cost"]) - 1e-14
                or (
                    np.isclose(d_try["ik_cost"], best_d["ik_cost"])
                    and float(d_try["ik_norm_geom"]) < float(best_d["ik_norm_geom"])
                )
            ):
                best_q, best_d = q_try, d_try

    return best_q, best_d
