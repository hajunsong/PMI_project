"""시뮬 전 계약 검증: Jacobian 줄·역-Jacobian 가상단위율 검사."""

from __future__ import annotations

import mujoco as mj
import numpy as np

from kinematics.task_jacobian import compute_task_jacobian
def jac_xyz_numeric_vs_analytic_report(
    model: mj.MjModel,
    data: mj.MjData,
    jac_workspace: mj.MjData,
    joint_names: list[str],
    *,
    ee_site_name: str = "end_effector",
    epsilon: float = 1e-6,
) -> str:
    """위치Jacobian 3×4 줄의 수치/하이브리드 MuJoCo 위치 줄 최대 차이."""
    jac_workspace.qpos[:] = data.qpos
    jac_workspace.qvel[:] = data.qvel
    mj.mj_forward(model, jac_workspace)
    Jn = compute_task_jacobian(
        model,
        jac_workspace,
        joint_names=joint_names,
        ee_site_name=ee_site_name,
        mode="numerical",
        epsilon=float(epsilon),
    )
    jac_workspace.qpos[:] = data.qpos
    jac_workspace.qvel[:] = data.qvel
    mj.mj_forward(model, jac_workspace)
    Ja = compute_task_jacobian(
        model,
        jac_workspace,
        joint_names=joint_names,
        ee_site_name=ee_site_name,
        mode="mujoco_analytic",
        epsilon=float(epsilon),
    )
    md = float(np.max(np.abs(Jn[0:3, :] - Ja[0:3, :])))
    return f"max|J_xyz_num - J_xyz_analytic| = {md:.3e} (epsilon={epsilon})"


def virtual_work_slack_abs(
    F_task: np.ndarray,
    ydot_actual: np.ndarray,
    tau_joint: np.ndarray,
    qdot_joint: np.ndarray,
) -> float:
    """작은 창에서 | F^T ydot_actual - tau^T qdot | (미터·뉴턴·라디언 스케일 혼합)."""
    lf = np.asarray(F_task, dtype=np.float64).reshape(-1)
    yt = np.asarray(ydot_actual, dtype=np.float64).reshape(-1)
    tq = np.asarray(tau_joint, dtype=np.float64).reshape(-1)
    dq = np.asarray(qdot_joint, dtype=np.float64).reshape(-1)
    pw = float(lf.dot(yt) - float(tq.dot(dq)))
    return abs(pw)


def torque_scaling_report(tau_joint: np.ndarray, tau_act: np.ndarray, ratios: np.ndarray) -> str:
    r_row = np.asarray(ratios, dtype=np.float64).reshape(1, -1)
    tj = np.atleast_2d(np.asarray(tau_joint, dtype=np.float64))
    ta = np.atleast_2d(np.asarray(tau_act, dtype=np.float64))
    if ta.shape != tj.shape:
        raise ValueError(f"tau_act shape {ta.shape} vs tau_joint {tj.shape}")
    chk = ta - tj * r_row
    mx = float(np.max(np.abs(chk)))
    return f"max|tau_act - ratio*tau_joint| = {mx:.3e}"
