"""작업공간 VSD 관절 토크 — ``ControlMain::run_vsd`` (``analysis/cpp/src/controlmain.cpp``) 와 동일한 5D 오차·속도 오차· :math:`\\tau = J^{\\mathsf T}(K_s\\odot e + K_d\\odot e_v)` 구조."""

from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

import numpy as np

from kinematics.pmi_chain import fk_ee_pose_joint_rad, jacobian_5x4_joint_rad, wrap_to_pi
from planning.task_path_planner import _ROLL_PITCH_TARGET


def default_desired_roll_pitch_rad() -> Tuple[float, float]:
    """플래너·서버와 동일: roll = ``-π/2``, pitch = ``0``."""
    return float(_ROLL_PITCH_TARGET[0]), float(_ROLL_PITCH_TARGET[1])


def load_task_space_vsd_from_cfg(cfg: Dict[str, Any]) -> Tuple[np.ndarray, np.ndarray, bool, float]:
    """YAML ``task_space_vsd`` 에서 ``Ks``, ``Kd``, ``gravity_bias_feedforward``, ``gravity_compensation_gain``."""
    block = cfg.get("task_space_vsd") or {}
    Ks = np.asarray(block.get("Ks", [140.0, 140.0, 140.0, 0.0, 0.0]), dtype=np.float64)
    Kd = np.asarray(block.get("Kd", [22.0, 22.0, 22.0, 0.0, 0.0]), dtype=np.float64)
    gff = bool(block.get("gravity_bias_feedforward", False))
    gcg = float(block.get("gravity_compensation_gain", 0.0))
    if Ks.size != 5 or Kd.size != 5:
        raise ValueError("task_space_vsd.Ks / Kd 는 길이 5 여야 합니다 [x,y,z,roll,pitch]")
    return Ks, Kd, gff, gcg


def compute_run_vsd_joint_torques(
    q_joint_rad: np.ndarray,
    qdot_joint: np.ndarray,
    des_pos_xyz: np.ndarray,
    des_vel_task5: np.ndarray,
    Ks: np.ndarray,
    Kd: np.ndarray,
    *,
    des_roll: Optional[float] = None,
    des_pitch: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    C++ ``run_vsd`` 와 동일하게

    - ``err[:3] = des_pos - ee``, ``err[3:5] = wrap(des_rpy - rpy[:2])``
    - ``J = jacobian_5x4(q)``, ``task_vel = J @ qdot``, ``ev = des_vel - task_vel``
    - ``tau_joint = J.T @ (Ks*err + Kd*ev)``

    Parameters
    ----------
    des_vel_task5
        ``[vx, vy, vz, droll, dpitch]`` 목표 속도. C++ 경로는 마지막 두 성분이 0.
    des_roll, des_pitch
        생략 시 ``default_desired_roll_pitch_rad()`` 사용.

    Returns
    -------
    tau_joint
        shape (4,) **관절** 일반화력 [N·m]. 구동축 모터를 쓰면 ``actuator_torque_from_joint_torque`` 로 ``ctrl`` 에 넣는다.
    err, ev
        각각 (5,) 작업공간 오차·속도 오차 (로깅용).
    """
    q = np.asarray(q_joint_rad, dtype=np.float64).reshape(4)
    qd = np.asarray(qdot_joint, dtype=np.float64).reshape(4)
    des_pos = np.asarray(des_pos_xyz, dtype=np.float64).reshape(3)
    des_vel = np.asarray(des_vel_task5, dtype=np.float64).reshape(5)
    Ks_ = np.asarray(Ks, dtype=np.float64).reshape(5)
    Kd_ = np.asarray(Kd, dtype=np.float64).reshape(5)
    dr = float(_ROLL_PITCH_TARGET[0]) if des_roll is None else float(des_roll)
    dp = float(_ROLL_PITCH_TARGET[1]) if des_pitch is None else float(des_pitch)

    ee, rpy = fk_ee_pose_joint_rad(q)
    err = np.zeros(5, dtype=np.float64)
    err[:3] = des_pos - ee
    err[3] = wrap_to_pi(dr - rpy[0])
    err[4] = wrap_to_pi(dp - rpy[1])

    J = jacobian_5x4_joint_rad(q)
    task_vel = J @ qd
    ev = des_vel - task_vel

    w = Ks_ * err + Kd_ * ev
    tau = (J.T @ w).reshape(4)
    return tau, err, ev


def joint_reference_to_task_desires(
    q_des_joint: np.ndarray,
    qdot_des_joint: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    관절 기준 ``(q_des, qdot_des)`` 를 작업공간 목표로 변환: EE 위치 = ``FK(q_des)``,
    선속도 = ``J[:3,:](q_des) @ qdot_des``, roll/pitch 목표 속도 = 0 (``run_vsd`` 의 ``des_vel`` 후반과 동일).
    """
    qd = np.asarray(q_des_joint, dtype=np.float64).reshape(4)
    q = np.asarray(q_des_joint, dtype=np.float64).reshape(4)
    ee_d, _ = fk_ee_pose_joint_rad(q)
    Jd = jacobian_5x4_joint_rad(q)
    v_lin = (Jd[:3, :] @ qd).reshape(3)
    des_vel = np.concatenate([v_lin, np.zeros(2, dtype=np.float64)])
    return ee_d.copy(), des_vel


def compose_ctrl_torque_pd_then_bias(
    tau_vsd: np.ndarray,
    tau_extra: np.ndarray,
    qfrc_bias_joint: np.ndarray,
    *,
    gravity_bias_ff: bool,
    tau_gravity_joint: Optional[np.ndarray] = None,
    gravity_compensation_gain: float = 0.0,
) -> np.ndarray:
    """
    제어 토크 조합 (MuJoCo ``motor`` ``ctrl`` 용).

    ``τ = τ_vsd + τ_extra + (qfrc_bias_joint if gravity_bias_ff) + λ_g * τ_gravity``.

    ``τ_gravity`` 는 ``qvel=0`` 기준 ``qfrc_bias`` 관절 성분(중력 위주). ``gravity_bias_ff`` 가 켜져
    있으면 중력 항이 겹치므로 ``gravity_compensation_gain`` 은 보통 **0** 또는 **작은 값**만 권장.

    소프트웨어 ``np.clip`` 은 적용하지 않는다. 반환값은 **관절** 토크이며, 구동축 ``motor`` 에는
    ``kinematics.pmi_chain.actuator_torque_from_joint_torque`` 로 변환한 뒤 ``ctrl`` 에 넣는다.
    """
    tv = np.asarray(tau_vsd, dtype=np.float64).reshape(4)
    te = np.asarray(tau_extra, dtype=np.float64).reshape(4)
    u = tv + te
    if gravity_bias_ff:
        b = np.asarray(qfrc_bias_joint, dtype=np.float64).reshape(4)
        u = u + b
    lg = float(gravity_compensation_gain)
    if tau_gravity_joint is not None and lg > 1e-12:
        tg = np.asarray(tau_gravity_joint, dtype=np.float64).reshape(4)
        u = u + lg * tg
    return u
