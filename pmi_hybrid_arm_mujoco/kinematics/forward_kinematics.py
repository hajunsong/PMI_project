"""Forward kinematics for arm joints ``jnt1..jnt4`` (MuJoCo)."""

from __future__ import annotations

import mujoco as mj
import numpy as np

from kinematics.orientation_utils import euler_xyz_from_rotation_matrix
from utils.mujoco_helpers import joint_id


def set_arm_joint_positions(
    model: mj.MjModel,
    data: mj.MjData,
    q_joint: np.ndarray,
    joint_side_order: list[str],
    *,
    clear_velocities: bool = False,
) -> None:
    """Write ``q_joint`` to listed hinge joints only; preserves other poses."""
    q = np.asarray(q_joint, dtype=np.float64).flatten()
    if q.size != len(joint_side_order):
        raise ValueError("q_joint length mismatch")
    for i, name in enumerate(joint_side_order):
        jid = joint_id(model, name)
        adr = int(model.jnt_qposadr[jid])
        data.qpos[adr] = float(q[i])
    if clear_velocities:
        data.qvel[:] = 0.0


def fk_ee_pose(
    model: mj.MjModel,
    data: mj.MjData,
    q_joint: np.ndarray,
    joint_side_order: list[str],
    site_name: str = "end_effector",
) -> tuple[np.ndarray, np.ndarray]:
    """Return (p_ee_world, R_wb_flat9) given arm joints only."""
    set_arm_joint_positions(model, data, q_joint, joint_side_order)
    mj.mj_forward(model, data)
    sid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, site_name)
    if sid < 0:
        raise ValueError(f"missing site: {site_name}")
    pos = np.array(data.site_xpos[sid], dtype=np.float64).copy()
    rmat = np.array(data.site_xmat[sid], dtype=np.float64).reshape(3, 3).copy()
    return pos, rmat


def fk_ee_rp(
    model: mj.MjModel,
    data: mj.MjData,
    q_joint: np.ndarray,
    joint_side_order: list[str],
    site_name: str = "end_effector",
) -> tuple[np.ndarray, float, float, float]:
    pos, R = fk_ee_pose(model, data, q_joint, joint_side_order, site_name=site_name)
    roll, pitch, yaw = euler_xyz_from_rotation_matrix(R)
    return pos, roll, pitch, yaw
