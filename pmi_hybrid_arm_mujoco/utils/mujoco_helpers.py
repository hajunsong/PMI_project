"""MuJoCo model loading, kinematics coupling, transmission helpers."""

from __future__ import annotations

import math
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Literal, Sequence

import numpy as np
import yaml
import mujoco as mj

PKG_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL_PATH = PKG_ROOT / "models" / "pmi_hybrid_arm.xml"
VSD_DEBUG_MODEL_PATH = PKG_ROOT / "models" / "pmi_hybrid_arm_vsd_debug.xml"
DEFAULT_CONFIG_DIR = PKG_ROOT / "configs"


def load_yaml(path: Path | str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


@contextmanager
def passive_mujoco_viewer(model: mj.MjModel, data: mj.MjData, enabled: bool):
    """실시간 IK/추종 확인용 GLFW MuJoCo 뷰어(``launch_passive``). ``enabled`` False면 무-오퍼레이션."""
    if enabled:
        import mujoco.viewer as mj_view

        with mj_view.launch_passive(model, data) as viewer:
            yield viewer
    else:
        yield None


def populate_ik_path_overlay_geoms(
    user_scn: mj.MjvScene,
    path_points_xyz: np.ndarray,
    pos_step_des: Sequence[float] | np.ndarray,
    pos_step_ee: Sequence[float] | np.ndarray,
    *,
    rgba_path: tuple[float, float, float, float] = (1.0, 1.0, 0.0, 1.0),
    rgba_step_des: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 1.0),
    rgba_step_ee: tuple[float, float, float, float] = (0.0, 1.0, 0.0, 1.0),
    path_line_width_px: float = 4.0,
    sphere_radius_des: float = 0.012,
    sphere_radius_ee: float = 0.011,
) -> None:
    """
    Passive viewer ``user_scn`` 카트 좌표 시각화: 경로 라인(strip) + 현재 목표점 + 현재 EE.

    - 경로(strip 연결선): 노란색 기본 ``rgba_path``
    - 현재 스텝 목표 ``pos_step_des``: 빨간 구
    - 현재 FK EE ``pos_step_ee``: 녹색 구
    """
    path = np.asarray(path_points_xyz, dtype=np.float64)
    if path.ndim != 2 or path.shape[1] != 3:
        raise ValueError("path_points_xyz must be shape (N, 3)")
    p_des = np.asarray(pos_step_des, dtype=np.float64).reshape(3)
    p_ee = np.asarray(pos_step_ee, dtype=np.float64).reshape(3)

    c_path = np.array(rgba_path, dtype=np.float32, copy=True)
    c_des = np.array(rgba_step_des, dtype=np.float32, copy=True)
    c_ee = np.array(rgba_step_ee, dtype=np.float32, copy=True)
    mat = np.eye(3, dtype=np.float64).flatten()
    sz_line = np.zeros((3, 1), dtype=np.float64)
    origin = np.zeros((3, 1), dtype=np.float64)

    n_line = max(0, int(path.shape[0]) - 1)
    n_geom = n_line + 2
    if n_geom > user_scn.maxgeom:
        raise ValueError(f"overlay needs {n_geom} geoms but user_scn.maxgeom={user_scn.maxgeom}")

    user_scn.ngeom = 0
    k = 0
    if path.shape[0] >= 2:
        for j in range(path.shape[0] - 1):
            mj.mjv_initGeom(
                user_scn.geoms[k],
                int(mj.mjtGeom.mjGEOM_LINE),
                sz_line,
                origin,
                mat,
                c_path,
            )
            f = np.array(path[j], dtype=np.float64).reshape(3, 1)
            t = np.array(path[j + 1], dtype=np.float64).reshape(3, 1)
            mj.mjv_connector(user_scn.geoms[k], int(mj.mjtGeom.mjGEOM_LINE), float(path_line_width_px), f, t)
            k += 1

    for rad, p, rgba in (
        (float(sphere_radius_des), p_des, c_des),
        (float(sphere_radius_ee), p_ee, c_ee),
    ):
        sz_s = np.array([[rad], [0.0], [0.0]], dtype=np.float64)
        pos_col = np.array(p, dtype=np.float64).reshape(3, 1)
        mj.mjv_initGeom(user_scn.geoms[k], int(mj.mjtGeom.mjGEOM_SPHERE), sz_s, pos_col, mat, rgba)
        k += 1

    user_scn.ngeom = k


@dataclass
class TransmissionRow:
    joint: str  # robot joint side
    actuator: str
    kind: str  # "belt" | "cable"
    ratio: float  # q_joint = ratio * q_actuator
    r_motor: float
    r_joint: float
    x0: float


def transmission_from_config(cfg: dict[str, Any]) -> list[TransmissionRow]:
    joints = cfg["joints"]
    rows = []
    for jname in cfg["joint_side_order"]:
        block = joints[jname]
        rows.append(
            TransmissionRow(
                joint=jname,
                actuator=str(block["actuator"]),
                kind=str(block["type"]),
                ratio=float(block["ratio"]),
                r_motor=float(block["r_motor"]),
                r_joint=float(block["r_joint"]),
                x0=float(block.get("x0", 0.0)),
            )
        )
    return rows


def joint_id(model: mj.MjModel, name: str) -> int:
    jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
    if jid < 0:
        raise ValueError(f"joint not found: {name}")
    return jid


def compile_model_for_simulation(
    xml_path: Path | str,
    rows: Sequence[TransmissionRow],
    *,
    equality_mode: Literal["none", "ideal_all", "belt_only"] = "none",
    strip_position_actuators: bool = False,
) -> mj.MjModel:
    """
    equality_mode:
      none      — independent q_act / jnt (Phase 1 kinematic python sync)
      ideal_all — perfect bilinear coupling jnt = ratio * q_act for all four pairs
      belt_only — only jnt1/q1_act coupled; other pairs remain independent (Phase 3 cable path)
    """
    spec = mj.MjSpec.from_file(str(xml_path))
    if strip_position_actuators:
        for a in list(spec.actuators)[::-1]:
            spec.delete(a)

    if equality_mode == "ideal_all":
        for row in rows:
            _add_joint_equality(spec, row.joint, row.actuator, row.ratio)
    elif equality_mode == "belt_only":
        if rows and rows[0].kind != "belt":
            raise ValueError("first transmission row expected belt coupling for Phase 3")
        _add_joint_equality(spec, rows[0].joint, rows[0].actuator, rows[0].ratio)

    return spec.compile()


def _add_joint_equality(spec: mj.MjSpec, joint_dependent: str, joint_reference: str, ratio: float) -> None:
    eq = spec.add_equality(
        name=f"eq_{joint_dependent}_{joint_reference}",
        type=mj.mjtEq.mjEQ_JOINT,
    )
    eq.name1 = joint_dependent
    eq.name2 = joint_reference
    d = list(eq.data)
    d[0] = 0.0
    d[1] = float(ratio)
    eq.data = d


def load_mjmodel(
    xml_path: Path | str,
    *,
    strip_position_actuators: bool = False,
) -> mj.MjModel:
    """Thin wrapper retained for callers that only toggle actuators (no coupling)."""
    spec = mj.MjSpec.from_file(str(xml_path))
    if strip_position_actuators:
        for a in list(spec.actuators)[::-1]:
            spec.delete(a)
    return spec.compile()


def expected_joint_positions(q_act: np.ndarray, ratios: Sequence[float]) -> np.ndarray:
    """q_jnt[i] = ratio[i] * q_act[i] (pairwise for this robot)."""
    return np.asarray(ratios, dtype=np.float64) * np.asarray(q_act, dtype=np.float64)


def ideal_actuator_torque_to_joint_torque(tau_act: np.ndarray, ratios: Sequence[float]) -> np.ndarray:
    """
    Power-consistent mapping when q_joint = ratio * q_actuator:
        tau_joint = tau_act / ratio
    """
    r = np.asarray(ratios, dtype=np.float64)
    return np.asarray(tau_act, dtype=np.float64) / r


def apply_kinematic_transmission(
    model: mj.MjModel,
    data: mj.MjData,
    rows: Iterable[TransmissionRow],
) -> None:
    """
    Phase 1 ideal kinematic coupling (NOT physics-constrained equality):
    After integrating actuators, enforce joint positions/velocities from motor side.

    q_joint = ratio * q_actuator
    qdot_joint = ratio * qdot_actuator
    """
    for row in rows:
        j_id = joint_id(model, row.joint)
        a_id = joint_id(model, row.actuator)
        qj = model.jnt_qposadr[j_id]
        qa = model.jnt_qposadr[a_id]
        vj = model.jnt_dofadr[j_id]
        va = model.jnt_dofadr[a_id]
        data.qpos[qj] = row.ratio * data.qpos[qa]
        data.qvel[vj] = row.ratio * data.qvel[va]
    mj.mj_forward(model, data)


def apply_ideal_qjnt_equals_ratio_qact(
    model: mj.MjModel,
    data: mj.MjData,
    joint_names: Sequence[str],
    actuator_names: Sequence[str],
    ratios: Sequence[float],
) -> None:
    """q_joint_i = ratio_i * q_act_i 로 동기화 (path-tracking Phase A)."""
    for jn, an, rr in zip(joint_names, actuator_names, ratios, strict=True):
        jid = joint_id(model, str(jn))
        aid = joint_id(model, str(an))
        qj = model.jnt_qposadr[jid]
        qa = model.jnt_qposadr[aid]
        vj = model.jnt_dofadr[jid]
        va = model.jnt_dofadr[aid]
        r = float(rr)
        data.qpos[qj] = r * data.qpos[qa]
        data.qvel[vj] = r * data.qvel[va]
    mj.mj_forward(model, data)


def apply_ideal_qact_from_joint(
    model: mj.MjModel,
    data: mj.MjData,
    joint_names: Sequence[str],
    actuator_names: Sequence[str],
    ratios: Sequence[float],
) -> None:
    """q_act_i = q_joint_i / ratio_i — joint_direct 모드에서 로깅/정렬용."""
    for jn, an, rr in zip(joint_names, actuator_names, ratios, strict=True):
        jid = joint_id(model, str(jn))
        aid = joint_id(model, str(an))
        qj = model.jnt_qposadr[jid]
        qa = model.jnt_qposadr[aid]
        vj = model.jnt_dofadr[jid]
        va = model.jnt_dofadr[aid]
        r = float(rr)
        data.qpos[qa] = data.qpos[qj] / r
        data.qvel[va] = data.qvel[vj] / r
    mj.mj_forward(model, data)


def compile_model_with_custom_position_actuators(
    xml_path: Path | str,
    actuator_kp: Sequence[float],
    actuator_kv: Sequence[float],
) -> mj.MjModel:
    """기존 spec 의 position actuator KP/KV 교체 후 컴파일."""
    spec = mj.MjSpec.from_file(str(xml_path))
    acts = list(spec.actuators)
    if len(acts) != len(actuator_kp) or len(acts) != len(actuator_kv):
        raise ValueError("actuator count must match KP/KV list length")
    for a, kp, kd in zip(acts, actuator_kp, actuator_kv, strict=True):
        a.set_to_position(kp=float(kp), kv=float(kd), dampratio=-1)
    return spec.compile()


def infer_actuator_pos_from_joint(q_joint: float, ratio: float) -> float:
    """Inverse of mimic: q_act = q_joint / ratio (for logging in Phase 2)."""
    return q_joint / ratio


def site_pose_world(model: mj.MjModel, data: mj.MjData, site_name: str) -> tuple[np.ndarray, np.ndarray]:
    sid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, site_name)
    if sid < 0:
        raise ValueError(f"site not found: {site_name}")
    pos = data.site_xpos[sid].copy()
    mat = data.site_xmat[sid].reshape(3, 3).copy()
    return pos, mat


def quat_wxyz_from_mat(mat: np.ndarray) -> np.ndarray:
    """Rotation matrix (3x3) -> quaternion w,x,y,z (MuJoCo convention)."""
    m = mat.reshape(9)
    trace = m[0] + m[4] + m[8]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[7] - m[5]) * s
        y = (m[2] - m[6]) * s
        z = (m[3] - m[1]) * s
    elif m[0] > m[4] and m[0] > m[8]:
        s = 2.0 * math.sqrt(1.0 + m[0] - m[4] - m[8])
        w = (m[7] - m[5]) / s
        x = 0.25 * s
        y = (m[1] + m[3]) / s
        z = (m[2] + m[6]) / s
    elif m[4] > m[8]:
        s = 2.0 * math.sqrt(1.0 + m[4] - m[0] - m[8])
        w = (m[2] - m[6]) / s
        x = (m[1] + m[3]) / s
        y = 0.25 * s
        z = (m[5] + m[7]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[8] - m[0] - m[4])
        w = (m[3] - m[1]) / s
        x = (m[2] + m[6]) / s
        y = (m[5] + m[7]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    return q / (np.linalg.norm(q) + 1e-12)


def q_act_des_trajectory_scalar(
    index: int,
    t_s: float,
    traj: dict[str, Any],
    switch_time_sec: float,
) -> tuple[float, float]:
    """Return desired (position, velocity) scalar for actuator index."""
    traj_type = str(traj["type"]).lower()
    ofs = traj["offset"][index]
    if traj_type == "sinusoidal":
        amp = traj["amplitude"][index]
        f = traj["frequency_hz"][index]
        ph = traj["phase_rad"][index]
        w = 2.0 * math.pi * float(f)
        q = ofs + amp * math.sin(w * t_s + ph)
        qd = amp * w * math.cos(w * t_s + ph)
        return q, qd
    if traj_type == "constant":
        v = traj["constant_pose_rad"][index]
        return float(v), 0.0
    if traj_type == "step":
        tgt = traj["step_target_rad"][index]
        ts = switch_time_sec
        if float(t_s) < float(ts):
            return float(ofs), 0.0
        return float(tgt), 0.0
    raise ValueError(f"unknown trajectory type: {traj_type}")


def q_act_trajectory_vectors(
    t_s: float,
    traj_cfg: dict[str, Any],
    actuator_count: int = 4,
    switch_override: float | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Stack per-actuator sinusoidal/step/constant trajectories."""
    switch = switch_override if switch_override is not None else traj_cfg.get("step_switch_sec", 0.5)
    q = np.zeros(actuator_count, dtype=np.float64)
    qd = np.zeros(actuator_count, dtype=np.float64)
    traj = traj_cfg
    for i in range(actuator_count):
        qi, qdi = q_act_des_trajectory_scalar(i, float(t_s), traj, switch)
        q[i] = qi
        qd[i] = qdi
    return q, qd


def sensor_adr(model: mj.MjModel, name: str) -> int:
    sid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SENSOR, name)
    if sid < 0:
        raise ValueError(f"sensor not found: {name}")
    return model.sensor_adr[sid]
