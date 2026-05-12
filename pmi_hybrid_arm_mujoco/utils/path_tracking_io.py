"""Load path_tracking.yaml and derive ordered ratio vectors."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from kinematics.task_jacobian import TaskJacobianMode, task_dim
from utils.mujoco_helpers import load_yaml


def load_path_tracking_yaml(path: Path | str | None = None) -> dict[str, Any]:
    from utils.mujoco_helpers import PKG_ROOT

    p = Path(path) if path is not None else PKG_ROOT / "configs" / "path_tracking.yaml"
    return load_yaml(p)


def load_task_space_vsd_yaml(path: Path | str | None = None) -> dict[str, Any]:
    """``configs/task_space_vsd.yaml`` — 작업공간 VSD Phase B 설정."""
    from utils.mujoco_helpers import PKG_ROOT

    p = Path(path) if path is not None else PKG_ROOT / "configs" / "task_space_vsd.yaml"
    return load_yaml(p)


def load_task_space_vsd_debug_yaml(path: Path | str | None = None) -> dict[str, Any]:
    """``configs/task_space_vsd_debug.yaml``."""
    from utils.mujoco_helpers import PKG_ROOT

    p = Path(path) if path is not None else PKG_ROOT / "configs" / "task_space_vsd_debug.yaml"
    return load_yaml(p)


def ordered_transmission_arrays(cfg: dict[str, Any]) -> tuple[list[str], list[str], np.ndarray]:
    ts = cfg["transmission"]
    # Derive order from jnt1..jnt4 keys (no extra ordering keys required).
    j_order = [f"jnt{i}" for i in range(1, 5)]
    missing = [jn for jn in j_order if jn not in ts]
    if missing:
        raise KeyError(f"path_tracking.transmission missing: {missing}")

    a_order: list[str] = []
    ratios: list[float] = []
    for jn in j_order:
        block = ts[jn]
        if not isinstance(block, dict):
            raise TypeError(f"transmission[{jn}] must be a mapping")
        a_order.append(str(block["actuator"]))
        ratios.append(float(block["ratio"]))
    return j_order, a_order, np.array(ratios, dtype=np.float64)


def joint_actuator_bounds(model_mj: Any, j_order: list[str], a_order: list[str]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return (qj_lo,qj_hi,qact_lo,qact_hi) from MJCF joint ranges."""
    from utils.mujoco_helpers import joint_id

    lj = []
    hj = []
    for n in j_order:
        jid = joint_id(model_mj, n)
        lj.append(float(model_mj.jnt_range[jid, 0]))
        hj.append(float(model_mj.jnt_range[jid, 1]))
    la = []
    ha = []
    for n in a_order:
        jid = joint_id(model_mj, n)
        la.append(float(model_mj.jnt_range[jid, 0]))
        ha.append(float(model_mj.jnt_range[jid, 1]))
    return (
        np.array(lj),
        np.array(hj),
        np.array(la),
        np.array(ha),
    )


def actuator_from_joint_positions(q_joint: np.ndarray, ratios: np.ndarray) -> np.ndarray:
    return np.asarray(q_joint, dtype=np.float64) / np.asarray(ratios, dtype=np.float64)


def joint_from_actuator_positions(q_act: np.ndarray, ratios: np.ndarray) -> np.ndarray:
    return np.asarray(q_act, dtype=np.float64) * np.asarray(ratios, dtype=np.float64)


def gains_limits_task_space_vsd(
    tsv: dict[str, Any],
) -> tuple[TaskJacobianMode, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """YAML ``task_space_vsd`` 블록에서 과제 차원별 ``K, D, F, tau_jnt, tau_act``."""
    raw_mode = str(tsv.get("task_mode", "xyz")).strip()
    mode: TaskJacobianMode
    if raw_mode in ("xyz", "xyz_pitch", "xyz_roll_pitch"):
        mode = raw_mode  # type: ignore[assignment]
    else:
        raise ValueError(f"unknown task_mode: {raw_mode!r}")

    g = tsv["gains"]
    lim = tsv["limits"]

    if isinstance(g.get("K_task"), dict):
        keys_k = ["x", "y", "z", "roll", "pitch"]
        keys_d = keys_k
        K5 = np.array([float(g["K_task"][k]) for k in keys_k], dtype=np.float64)
        D5 = np.array([float(g["D_task"][k]) for k in keys_d], dtype=np.float64)
        F5 = np.array([float(lim["F_task"][k]) for k in keys_k], dtype=np.float64)
        if mode != "xyz_roll_pitch":
            raise ValueError("legacy YAML (K_task dict) implies task_mode xyz_roll_pitch only")
        return mode, K5, D5, F5, np.asarray(lim["tau_jnt"], dtype=np.float64), np.asarray(
            lim["tau_act"], dtype=np.float64
        )

    kx = np.asarray(g["xyz"]["K"], dtype=np.float64).reshape(3)
    dx = np.asarray(g["xyz"]["D"], dtype=np.float64).reshape(3)
    kr = float(g["roll"]["K"])
    dr = float(g["roll"]["D"])
    kp = float(g["pitch"]["K"])
    dp = float(g["pitch"]["D"])

    fz = np.asarray(lim["F_xyz"], dtype=np.float64).reshape(3)
    m_rp = np.asarray(lim["M_roll_pitch"], dtype=np.float64).reshape(2)

    if mode == "xyz":
        K, D, F = kx, dx, fz
    elif mode == "xyz_pitch":
        K = np.concatenate([kx, [kp]])
        D = np.concatenate([dx, [dp]])
        Fp = float(lim["F_pitch"]) if lim.get("F_pitch") is not None else float(m_rp[1])
        F = np.concatenate([fz, [Fp]])
    else:
        K = np.concatenate([kx, [kr, kp]])
        D = np.concatenate([dx, [dr, dp]])
        F = np.concatenate([fz, m_rp])

    m_expect = task_dim(mode)
    if K.size != m_expect:
        raise ValueError(f"gains size mismatch task_mode={mode}")

    tau_j = np.asarray(lim["tau_jnt"], dtype=np.float64).reshape(4)
    tau_a = np.asarray(lim["tau_act"], dtype=np.float64).reshape(4)

    return mode, K.astype(np.float64), D.astype(np.float64), F.astype(np.float64), tau_j, tau_a


def roll_pitch_des_from_orientation_config(
    ori: dict[str, Any],
    model: Any,
    scratch: Any,
    q_init_joints: np.ndarray,
    joint_order: list[str],
) -> tuple[float, float]:
    """IK에 쓸 roll/pitch 목표.

    - ``roll_pitch_reference`` (기본값 ``initial_pose``): ``q_init_joints`` FK의 roll/pitch (초기 자세 유지).
    - ``fixed``: ``orientation.roll`` / ``orientation.pitch`` 수치 사용.
    """
    ref = str(ori.get("roll_pitch_reference", "initial_pose")).strip().lower()
    if ref in ("initial_pose", "initial", "from_q_init", "maintain_initial", "q_init"):
        from kinematics.forward_kinematics import fk_ee_rp

        _p, rol, pit, _yaw = fk_ee_rp(model, scratch, q_init_joints, joint_order)
        return float(rol), float(pit)
    if ref in ("fixed", "yaml", "explicit"):
        return float(ori["roll"]), float(ori["pitch"])
    raise ValueError(
        "orientation.roll_pitch_reference must be initial_pose or fixed "
        f"(got {ori.get('roll_pitch_reference')!r})"
    )
