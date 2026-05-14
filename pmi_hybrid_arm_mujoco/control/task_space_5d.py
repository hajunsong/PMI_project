"""5D EE task (xyz + roll/pitch) helpers: pose, weighted error, wrench → torque with optional DLS.

Uses :func:`compute_task_jacobian_mode` with ``xyz_roll_pitch`` (MuJoCo hybrid): roll/pitch rows match FK ZYX Euler.
"""

from __future__ import annotations

from typing import Literal

import mujoco as mj
import numpy as np

from kinematics.forward_kinematics import fk_ee_rp
from kinematics.orientation_utils import angle_error
from kinematics.task_jacobian import compute_task_jacobian_mode

WrenchMapMode = Literal[
    "wrench_jacobian_transpose",
    "dls_task_correction",
    "dls_weighted_wrench",
]


def get_ee_pose_5d(
    model: mj.MjModel,
    data: mj.MjData,
    q_arm_jnt: np.ndarray,
    *,
    joint_names: list[str],
    site_name: str = "end_effector",
) -> tuple[np.ndarray, np.ndarray, float]:
    """Return ``xyz (3,), roll_pitch (2,), yaw`` yaw for logging."""
    q = np.asarray(q_arm_jnt, dtype=np.float64).reshape(-1)
    xyz, rr, pp, yaw = fk_ee_rp(model, data, q, joint_names, site_name=site_name)
    return (
        np.asarray(xyz, dtype=np.float64).reshape(3).copy(),
        np.array([float(rr), float(pp)], dtype=np.float64),
        float(yaw),
    )


def compute_5d_task_error(
    x_now: np.ndarray,
    xyz_des: np.ndarray,
    roll_now: float,
    pitch_now: float,
    *,
    roll_des: float,
    pitch_des: float,
    w_roll_soft: float,
    w_pitch_soft: float,
) -> np.ndarray:
    ex = np.asarray(xyz_des, dtype=np.float64).reshape(3) - np.asarray(x_now, dtype=np.float64).reshape(3)
    er = float(angle_error(float(roll_des), float(roll_now)))
    ep = float(angle_error(float(pitch_des), float(pitch_now)))
    return np.array(
        [float(ex[0]), float(ex[1]), float(ex[2]), float(w_roll_soft) * er, float(w_pitch_soft) * ep],
        dtype=np.float64,
    )


def analytic_jacobian_5dof(
    model: mj.MjModel,
    jac_workspace: mj.MjData,
    *,
    joint_names: list[str],
    site_name: str = "end_effector",
    epsilon: float = 1e-6,
) -> np.ndarray:
    return compute_task_jacobian_mode(
        model,
        jac_workspace,
        joint_names=joint_names,
        task_mode="xyz_roll_pitch",
        ee_site_name=site_name,
        mode="mujoco_analytic",
        epsilon=float(epsilon),
    )


def slice_joint_columns(J: np.ndarray, *, cable_only: bool) -> np.ndarray:
    J = np.asarray(J, dtype=np.float64).reshape(J.shape[0], -1)
    if cable_only:
        return J[:, 1:4].copy()
    return J.copy()


def scatter_to_four(tau_partial: np.ndarray, *, cable_only: bool) -> np.ndarray:
    tau_partial = np.asarray(tau_partial, dtype=np.float64).reshape(-1)
    out = np.zeros(4, dtype=np.float64)
    if cable_only:
        if tau_partial.shape[0] != 3:
            raise ValueError("cable_only expects 3 joint torques (jnt2–jnt4)")
        out[1:4] = tau_partial
        return out
    if tau_partial.shape[0] != 4:
        raise ValueError("all joints expects torque length 4")
    out[:] = tau_partial
    return out


def dls_inverse_right_square(
    J_cols: np.ndarray,
    *,
    task_weight_row: np.ndarray,
    damping_sq: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return ``Jp (nj×5), Jw`` with ``Jw[i,:]=w[i]*J[i,:]``.

    Pseudoinverse: ``Jp = Jw.T @ (Jw Jw.T + λ²I)^{-1}``.
    """

    J = np.asarray(J_cols, dtype=np.float64)
    nrow = int(J.shape[0])
    wt = np.asarray(task_weight_row, dtype=np.float64).reshape(-1)
    if wt.shape[0] != nrow:
        raise ValueError("task_weight_row length vs J rows mismatch")

    Jw = wt.reshape(-1, 1) * J
    lamb2 = float(max(float(damping_sq), 1e-18))
    a = Jw @ Jw.T + lamb2 * np.eye(nrow, dtype=np.float64)
    a_inv = np.linalg.inv(a)
    j_nt = Jw.T @ a_inv
    return j_nt, Jw


def residual_torque_from_wrench_5d(
    J_cols: np.ndarray,
    wrench_cmd_5d: np.ndarray,
    *,
    wrench_map_mode: WrenchMapMode,
    task_weight_diag: np.ndarray,
    cable_only: bool,
    lambda_dls: float,
    K_dls: float,
    D_dls: float,
    qd_joint_cols: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Map task wrench command ``W∈ℝ⁵`` → joint torque (4-vector). Second return packs aux [dqdot.., alpha]."""

    w_cmd = np.asarray(wrench_cmd_5d, dtype=np.float64).reshape(5)
    tw = np.asarray(task_weight_diag, dtype=np.float64).reshape(5)
    qd_cols = np.asarray(qd_joint_cols, dtype=np.float64).reshape(-1)
    if qd_cols.shape[0] != J_cols.shape[1]:
        raise ValueError("qd_joint_cols mismatches Jacobian column count")

    lam2 = float(max(float(lambda_dls), 1e-9)) ** 2
    aux = np.zeros(8, dtype=np.float64)

    if wrench_map_mode == "wrench_jacobian_transpose":
        w_eff = tw * w_cmd
        tau_p = np.asarray(J_cols, dtype=np.float64).T @ w_eff
        return scatter_to_four(tau_p, cable_only=cable_only), aux

    if wrench_map_mode == "dls_task_correction":
        jp, _ = dls_inverse_right_square(J_cols, task_weight_row=tw, damping_sq=lam2)
        dq = jp @ w_cmd
        tau_p = float(K_dls) * dq - float(D_dls) * qd_cols
        aux[: min(len(dq), aux.shape[0])] = dq[: aux.shape[0]]
        return scatter_to_four(tau_p, cable_only=cable_only), aux

    if wrench_map_mode == "dls_weighted_wrench":
        w_eff = tw * w_cmd
        tau_raw = np.asarray(J_cols, dtype=np.float64).T @ w_eff
        _, Jw = dls_inverse_right_square(J_cols, task_weight_row=tw, damping_sq=lam2)
        sig = np.linalg.svd(Jw, compute_uv=False)
        sigma_min_sq = float(sig[-1] * sig[-1]) if sig.size > 0 else 0.0
        alpha = sigma_min_sq / (sigma_min_sq + lam2)
        tau_p = float(alpha) * tau_raw
        aux[7] = float(alpha)
        return scatter_to_four(tau_p, cable_only=cable_only), aux

    raise ValueError(f"unknown wrench_map_mode: {wrench_map_mode}")


def heuristic_wrench_rp_zeros_force(
    J_full_54: np.ndarray,
    *,
    roll_des: float,
    pitch_des: float,
    q_j_arm: np.ndarray,
    qd_j_arm: np.ndarray,
    q_des_arm: np.ndarray,
    qd_des_arm: np.ndarray,
    model: mj.MjModel,
    fk_data: mj.MjData,
    joint_names: list[str],
    site_name: str,
    Kp_roll: float,
    Dp_roll: float,
    Kp_pitch: float,
    Dp_pitch: float,
) -> np.ndarray:
    """Return ``[0,0,0,M_roll,M_pitch]`` stabilizing wrench (orientation rows of ``J``, FK convention)."""

    q = np.asarray(q_j_arm, dtype=np.float64).reshape(4)
    qv = np.asarray(qd_j_arm, dtype=np.float64).reshape(4)
    qvd = np.asarray(qd_des_arm, dtype=np.float64).reshape(4)

    jr = np.asarray(J_full_54[3, :], dtype=np.float64).reshape(4)
    jp = np.asarray(J_full_54[4, :], dtype=np.float64).reshape(4)

    _, r_act, p_act, _ = fk_ee_rp(model, fk_data, q, list(joint_names), site_name=site_name)

    er = float(angle_error(float(roll_des), float(r_act)))
    ep = float(angle_error(float(pitch_des), float(p_act)))

    rd_m = float(jr @ qv)
    pd_m = float(jp @ qv)
    rd_d = float(jr @ qvd)
    pd_d = float(jp @ qvd)

    m_roll = float(Kp_roll) * er + float(Dp_roll) * (rd_d - rd_m)
    m_pitch = float(Kp_pitch) * ep + float(Dp_pitch) * (pd_d - pd_m)

    return np.array([0.0, 0.0, 0.0, m_roll, m_pitch], dtype=np.float64)
