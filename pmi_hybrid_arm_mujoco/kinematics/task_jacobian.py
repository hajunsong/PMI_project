"""작업 공간 [x,y,z,roll,pitch] Jacobian (관절 4-DOF, yaw 미제어).

Euler 규약은 ``orientation_utils`` / FK 와 동일: SciPy **extrinsic XYZ**
``Rotation.as_euler('xyz')`` ⇒ ``(roll, pitch, yaw)``.

모드
----
* ``numerical``: 전체 ``5×4`` 중앙차분. 회전량은 각도 차이를 단기 차분으로 처리
  (:func:`orientation_utils.angle_error` 와 같은 래핑).
* ``mujoco_analytic``: EE site의 MuJoCo 위치 Jacobian ``3×nv`` 에서 네 관절 열만
  발췌. roll/pitch 2행은 비선형이라 동일 차분 근사(하이브리드).

수치Jacobian을 기본으로 두는 것을 권장한다.
"""

from __future__ import annotations

from typing import Literal

import mujoco as mj
import numpy as np

from kinematics.forward_kinematics import fk_ee_rp
from kinematics.orientation_utils import angle_error
from utils.mujoco_helpers import joint_id

JacobianMode = Literal["numerical", "mujoco_analytic"]
TaskJacobianMode = Literal["xyz", "xyz_pitch", "xyz_roll_pitch"]


def fk_task_pose5(
    model: mj.MjModel,
    data: mj.MjData,
    q_joint_arm: np.ndarray,
    joint_names: list[str],
    *,
    site_name: str = "end_effector",
) -> np.ndarray:
    """``y=[x,y,z,roll,pitch]`` yaw 제외."""
    p, r, pith, _yaw = fk_ee_rp(model, data, q_joint_arm, joint_names, site_name=site_name)
    return np.array([float(p[0]), float(p[1]), float(p[2]), float(r), float(pith)], dtype=np.float64)


def fk_task_y(
    model: mj.MjModel,
    data: mj.MjData,
    q_joint_arm: np.ndarray,
    joint_names: list[str],
    task_mode: TaskJacobianMode,
    *,
    site_name: str = "end_effector",
) -> np.ndarray:
    """과제 모드에 따른 ``y`` (``xyz``:3, ``xyz_pitch``:4, ``xyz_roll_pitch``:5)."""
    p, r, pith, _yaw = fk_ee_rp(model, data, q_joint_arm, joint_names, site_name=site_name)
    if task_mode == "xyz":
        return np.array([float(p[0]), float(p[1]), float(p[2])], dtype=np.float64)
    if task_mode == "xyz_pitch":
        return np.array([float(p[0]), float(p[1]), float(p[2]), float(pith)], dtype=np.float64)
    if task_mode == "xyz_roll_pitch":
        return np.array([float(p[0]), float(p[1]), float(p[2]), float(r), float(pith)], dtype=np.float64)
    raise ValueError(f"unknown task_mode: {task_mode}")


def task_dim(task_mode: TaskJacobianMode) -> int:
    return {"xyz": 3, "xyz_pitch": 4, "xyz_roll_pitch": 5}[task_mode]


def _numeric_jacobian(
    model: mj.MjModel,
    work: mj.MjData,
    joint_names: list[str],
    adr_q: np.ndarray,
    site_name: str,
    epsilon: float,
) -> np.ndarray:
    J = np.zeros((5, 4), dtype=np.float64)
    q0 = np.array([float(work.qpos[int(adr_q[k])]) for k in range(4)], dtype=np.float64)
    eps = float(epsilon)

    for col in range(4):
        a = int(adr_q[col])
        work.qpos[a] = q0[col] + eps
        mj.mj_forward(model, work)
        qp = np.array([float(work.qpos[int(adr_q[i])]) for i in range(4)])
        y_p = fk_task_pose5(model, work, qp, joint_names, site_name=site_name)

        work.qpos[a] = q0[col] - eps
        mj.mj_forward(model, work)
        qm = np.array([float(work.qpos[int(adr_q[i])]) for i in range(4)])
        y_m = fk_task_pose5(model, work, qm, joint_names, site_name=site_name)

        work.qpos[a] = q0[col]

        denom = 2.0 * eps
        J[0:3, col] = (y_p[0:3] - y_m[0:3]) / denom
        J[3, col] = angle_error(float(y_p[3]), float(y_m[3])) / denom
        J[4, col] = angle_error(float(y_p[4]), float(y_m[4])) / denom

    mj.mj_forward(model, work)
    return J


def _numeric_jacobian_task_mode(
    model: mj.MjModel,
    work: mj.MjData,
    joint_names: list[str],
    adr_q: np.ndarray,
    site_name: str,
    task_mode: TaskJacobianMode,
    epsilon: float,
) -> np.ndarray:
    m = task_dim(task_mode)
    J = np.zeros((m, 4), dtype=np.float64)
    q0 = np.array([float(work.qpos[int(adr_q[k])]) for k in range(4)], dtype=np.float64)
    eps = float(epsilon)

    def y_of() -> np.ndarray:
        qp = np.array([float(work.qpos[int(adr_q[i])]) for i in range(4)])
        return fk_task_y(model, work, qp, joint_names, task_mode, site_name=site_name)

    for col in range(4):
        a = int(adr_q[col])
        work.qpos[a] = q0[col] + eps
        mj.mj_forward(model, work)
        y_p = y_of()

        work.qpos[a] = q0[col] - eps
        mj.mj_forward(model, work)
        y_m = y_of()

        work.qpos[a] = q0[col]

        denom = 2.0 * eps
        if task_mode == "xyz":
            J[:, col] = (y_p - y_m) / denom
        elif task_mode == "xyz_pitch":
            J[0:3, col] = (y_p[0:3] - y_m[0:3]) / denom
            J[3, col] = angle_error(float(y_p[3]), float(y_m[3])) / denom
        else:
            J[0:3, col] = (y_p[0:3] - y_m[0:3]) / denom
            J[3, col] = angle_error(float(y_p[3]), float(y_m[3])) / denom
            J[4, col] = angle_error(float(y_p[4]), float(y_m[4])) / denom

    mj.mj_forward(model, work)
    return J


def compute_task_jacobian(
    model: mj.MjModel,
    jac_workspace: mj.MjData,
    *,
    joint_names: list[str],
    ee_site_name: str = "end_effector",
    mode: JacobianMode = "numerical",
    epsilon: float = 1e-6,
) -> np.ndarray:
    """
    ``J_task`` (``5×4``).

    주의: ``jac_workspace`` 는 **시뮬레이션 ``data``와 별개**여야 한다. 매 호출 전에 예::

        jac_workspace.qpos[:] = sim_data.qpos
        jac_workspace.qvel[:] = sim_data.qvel
        mujoco.mj_forward(model, jac_workspace)

    소작동 수치Jacobian이 ``jac_workspace.qpos`` 를 흔들며 복귀한다.
    """
    if len(joint_names) != 4:
        raise ValueError("expected arm joint_names length 4")

    adr_q = np.array([int(model.jnt_qposadr[joint_id(model, n)]) for n in joint_names], dtype=np.int32)
    dof_ix = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in joint_names], dtype=np.int32)
    nv = model.nv
    work = jac_workspace

    if mode == "numerical":
        return _numeric_jacobian(model, work, joint_names, adr_q, ee_site_name, epsilon)

    if mode == "mujoco_analytic":
        mj.mj_forward(model, work)
        sid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, ee_site_name)
        if sid < 0:
            raise ValueError(f"site missing: {ee_site_name}")
        jacp = np.zeros((3, nv), dtype=np.float64)
        jacr = np.zeros((3, nv), dtype=np.float64)
        mj.mj_jacSite(model, work, jacp, jacr, sid)

        J = np.zeros((5, 4), dtype=np.float64)
        J[0:3, :] = jacp[:, dof_ix]
        Jrp = _numeric_jacobian(model, work, joint_names, adr_q, ee_site_name, epsilon)
        J[3:5, :] = Jrp[3:5, :]
        mj.mj_forward(model, work)
        return J

    raise ValueError(f"unknown jacobian_mode: {mode}")


def compute_task_jacobian_mode(
    model: mj.MjModel,
    jac_workspace: mj.MjData,
    *,
    joint_names: list[str],
    task_mode: TaskJacobianMode,
    ee_site_name: str = "end_effector",
    mode: JacobianMode = "numerical",
    epsilon: float = 1e-6,
) -> np.ndarray:
    """``J_task`` 크기 ``(task_dim, 4)`` — ``task_mode`` 별 위치·자세 부분집합."""
    if len(joint_names) != 4:
        raise ValueError("expected arm joint_names length 4")

    adr_q = np.array([int(model.jnt_qposadr[joint_id(model, n)]) for n in joint_names], dtype=np.int32)
    dof_ix = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in joint_names], dtype=np.int32)
    nv = model.nv
    work = jac_workspace

    if mode == "numerical":
        return _numeric_jacobian_task_mode(model, work, joint_names, adr_q, ee_site_name, task_mode, epsilon)

    if mode == "mujoco_analytic":
        mj.mj_forward(model, work)
        sid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, ee_site_name)
        if sid < 0:
            raise ValueError(f"site missing: {ee_site_name}")
        jacp = np.zeros((3, nv), dtype=np.float64)
        jacr = np.zeros((3, nv), dtype=np.float64)
        mj.mj_jacSite(model, work, jacp, jacr, sid)

        if task_mode == "xyz":
            J = np.zeros((3, 4), dtype=np.float64)
            J[:, :] = jacp[:, dof_ix]
            mj.mj_forward(model, work)
            return J

        # 회전 행은 수치 하이브리드
        Jfull = _numeric_jacobian_task_mode(model, work, joint_names, adr_q, ee_site_name, task_mode, epsilon)
        Jfull[0:3, :] = jacp[:, dof_ix]
        mj.mj_forward(model, work)
        return Jfull

    raise ValueError(f"unknown jacobian_mode: {mode}")
