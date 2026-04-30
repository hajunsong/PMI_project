"""
analysis/python ``run_vsd`` 와 동일한 목표(위치·자세 PD + 중력 보상)를 MuJoCo PMI URDF에 적용.

- 태스크: EE 위치 (x,y,z) + roll/pitch (``mat2rpy(Ae)``, ``Ae = R_link4 @ Ce``).
- 토크: ``tau = J^T (Ks e + Kd ev) + qfrc_bias`` (중력은 MuJoCo ``qfrc_bias``).
- ``J`` 는 태스크(5)를 **메인 체인 관절** ``q*_jnt`` (4)에 대한 수치 미분으로 계산한다.
"""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

from pmi_mujoco_rl.model import actuated_joints_for_model

_REPO_ROOT = Path(__file__).resolve().parents[2]
_ANALYSIS_PY = _REPO_ROOT / "analysis" / "python"
if str(_ANALYSIS_PY) not in sys.path:
    sys.path.insert(0, str(_ANALYSIS_PY))

from path_generation import path_generation  # noqa: E402
from utils import euler_zxz, mat2rpy, wrap_to_pi  # noqa: E402

# controlmain.cpp / main.py run_vsd
KS_DEFAULT = np.array([15000.0, 15000.0, 15000.0, 1500.0, 1500.0], dtype=float)
KD_DEFAULT = np.array([1000.0, 1000.0, 1000.0, 10.0, 10.0], dtype=float)
WP_T = np.array([0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0], dtype=float)
WP_X = np.array([-0.35, -0.25, 0.25, 0.35, 0.18, -0.18, -0.35], dtype=float)
WP_Y = np.array([0.15, -0.28, -0.28, 0.15, 0.37, 0.37, 0.15], dtype=float)
WP_Z = np.array([-0.2, -0.2, -0.2, -0.2, 0.13, 0.13, -0.2], dtype=float)
# analysis ControlMain body[3].sep / Ce
EE_OFFSET_BODY = np.array([0.18, 0.0, 0.0], dtype=float)
CE_FIX = euler_zxz(-np.pi / 2.0, np.pi / 2.0, 0.0)


def path_build_full_quintic(wp_t: np.ndarray, wp_vals: np.ndarray, h: float) -> np.ndarray:
    """C++ ``path_build`` / analysis ``_path_build_full_quintic`` 와 동일."""
    wp_t = np.asarray(wp_t, dtype=float)
    wp_vals = np.asarray(wp_vals, dtype=float)
    wp_n = len(wp_t)
    parts: list[np.ndarray] = []
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


def _jnt_qpos_idx(model: mujoco.MjModel, joint_name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if jid < 0:
        raise ValueError(f"joint not found: {joint_name}")
    return int(model.jnt_qposadr[jid])


def _jnt_dof_idx(model: mujoco.MjModel, joint_name: str) -> int:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if jid < 0:
        raise ValueError(f"joint not found: {joint_name}")
    return int(model.jnt_dofadr[jid])


def ee_world_pos(model: mujoco.MjModel, data: mujoco.MjData, body_id: int) -> np.ndarray:
    R = data.xmat[body_id].reshape(3, 3)
    return np.asarray(data.xpos[body_id], dtype=float) + R @ EE_OFFSET_BODY


def task_desired_orientation_matrix(model: mujoco.MjModel, data: mujoco.MjData, body_id: int) -> np.ndarray:
    R = data.xmat[body_id].reshape(3, 3)
    return R @ CE_FIX


def task_vector(model: mujoco.MjModel, data: mujoco.MjData, body_id: int) -> np.ndarray:
    """[x,y,z, roll, pitch] — analysis ``mat2rpy(Ae)`` 의 roll/pitch."""
    p = ee_world_pos(model, data, body_id)
    Ae = task_desired_orientation_matrix(model, data, body_id)
    rpy = mat2rpy(Ae)
    return np.array([p[0], p[1], p[2], float(rpy[0]), float(rpy[1])], dtype=float)


def numerical_task_jacobian(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_id: int,
    eps: float = 1e-5,
) -> np.ndarray:
    """∂(태스크 5) / ∂(q_jnt1..4)."""
    J = np.zeros((5, 4), dtype=float)
    mujoco.mj_forward(model, data)
    joint_names = actuated_joints_for_model(model)
    for j, jname in enumerate(joint_names):
        iq = _jnt_qpos_idx(model, jname)
        data.qpos[iq] += eps
        mujoco.mj_forward(model, data)
        fp = task_vector(model, data, body_id)
        data.qpos[iq] -= 2.0 * eps
        mujoco.mj_forward(model, data)
        fm = task_vector(model, data, body_id)
        data.qpos[iq] += eps
        mujoco.mj_forward(model, data)
        J[:, j] = (fp - fm) / (2.0 * eps)
    return J


def velocity_task_error(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_id: int,
    des_vel: np.ndarray,
    p_ee: np.ndarray,
) -> np.ndarray:
    """analysis/C++: ``ev[0:3]`` = ``des - dre``, ``ev[3:5]`` = ``0 - wi[0:2]``."""
    jacp = np.zeros((3, model.nv), dtype=float)
    jacr = np.zeros((3, model.nv), dtype=float)
    mujoco.mj_jac(model, data, jacp, jacr, p_ee, body_id)
    v_ee = jacp @ np.asarray(data.qvel, dtype=float)
    omega = np.asarray(data.cvel[body_id, 3:6], dtype=float)
    ev = np.zeros(5, dtype=float)
    ev[0:3] = des_vel[0:3] - v_ee
    ev[3] = float(des_vel[3]) - float(omega[0])
    ev[4] = float(des_vel[4]) - float(omega[1])
    return ev


def operational_pd_torque(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_id: int,
    des_pos: np.ndarray,
    des_roll: float,
    des_pitch: float,
    des_vel: np.ndarray,
    ks: np.ndarray,
    kd: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    반환: (tau 4D — 모델의 메인 체인 모터 축, err 5, ev 5).
    tau = J^T (Ks*e + Kd*ev) + qfrc_bias[해당 DOF].
    """
    mujoco.mj_forward(model, data)

    J = numerical_task_jacobian(model, data, body_id)
    mujoco.mj_forward(model, data)

    p_ee = ee_world_pos(model, data, body_id)
    x = task_vector(model, data, body_id)

    err = np.zeros(5, dtype=float)
    err[0:3] = np.asarray(des_pos, dtype=float) - x[0:3]
    err[3] = wrap_to_pi(des_roll - x[3])
    err[4] = wrap_to_pi(des_pitch - x[4])

    ev = velocity_task_error(model, data, body_id, des_vel, p_ee)

    w = ks * err + kd * ev
    tau_pd = J.T @ w

    dof_i = np.array([_jnt_dof_idx(model, name) for name in actuated_joints_for_model(model)], dtype=np.int32)
    tau_bias = np.asarray(data.qfrc_bias[dof_i], dtype=float)
    tau = tau_pd + tau_bias
    return tau, err, ev


def apply_initial_from_recurdyn_row(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    row: np.ndarray,
) -> None:
    """
    ``rec_data_path.csv`` 한 행(0열 제거)에서 **메인 체인** 관절각 초기화.
    analysis ``rec_data(0, 31+i)`` 는 ``q*_jnt`` 와 동일 가정.
    """
    for i, jname in enumerate(actuated_joints_for_model(model)):
        q_j = float(row[31 + i])
        iq = _jnt_qpos_idx(model, jname)
        data.qpos[iq] = q_j
