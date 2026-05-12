#!/usr/bin/env python3
"""MuJoCo 토크 모터 + 작업공간 VSD로 관절 목표 경로 추종(경로 재생) 시각화.

``models/pmi_cable_arm.xml`` 은 ``motor`` 를 **구동축** ``q1_act``…``q4_act`` 에 걸고, ``data.ctrl`` 은 구동축 토크 [N·m]이다. 관절 공간 VSD 출력은 ``τ_act = τ_joint ⊙ gear`` 로 변환된다 (``kinematics.pmi_chain.actuator_torque_from_joint_torque``).

경로 생성은 ``demo_position_path_follow.py`` 와 동일:
- 관절 목표 ``q_des(t), qdot_des(t)`` 생성 (YAML ``trajectory``)
- 이를 FK로 작업공간 목표 ``des_pos_xyz`` 로 변환
- 작업공간 VSD 컨트롤러 (``controllers/task_space_vsd.py``) 로 토크 계산: ``τ = J^T(Ks⊙e + Kd⊙ev)``
- **기본**: ``transmission/hybrid_joint_torque.py`` 로 ``pmi_cable_arm_env`` 와 동일하게 벨트(jnt1)·케이블(jnt2–4) 잔차 토크를 관절 명령에 합산한 뒤 구동축 ``ctrl`` 로 변환. 끄려면 ``--no-transmission``.
- MuJoCo 토크 모터에 입력

기본 설정은 ``configs/control_params_vsd_task.yaml`` 에서 로드됩니다:
- ``task_space_vsd.Ks``, ``Kd``: 작업공간 비례/미분 게인 [x,y,z,roll,pitch]
- ``simulation.frame_skip``: 제어 스텝 간격
- ``trajectory``: 관절 궤적 (``sine``, ``step_hold``, ``waypoints_xyz``)

**오류 진단**:
- **시작 튐**: ``--settle-steps`` 로 경로 전 안정화 (VSD 기반 현재 자세 유지). settle 후에는 **현재 관절로 웨이포인트 궤적을 재플랜**한다.
- **추종 불안정**: ``--ks`` / ``--kd`` 로 게인 조정. 웨이포인트 xyz 추종만 할 때는 보통 **roll/pitch 게인 0** 이 안전하다 (오리엔테이션 항이 xyz 토크를 잠식함).
- ``--waypoint-task-from path_xyz``: 플래너 작업공간 샘플을 직접 목표 위치로 사용 (``demo_vsd_torque_sim.py`` 와 동일 옵션).
- **토크**: 스크립트에서 ``np.clip`` 하지 않는다. 한도는 MJCF ``motor`` 의 ``forcerange`` 등에 따른다.

``waypoints_xyz`` 모드에서는 관절 궤적을 생성 후, 실제 EE가 마지막 웨이포인트에
``--ee-arrival-tol`` 이내로 ``--ee-arrival-hold`` 스텝 유지되면 종료합니다.

뷰어 오버레이 (``--no-overlay`` 로 끔):
- **녹색 선**: 웨이포인트 모드에서 플래너 작업공간 경로 (xyz 폴리라인).
- **노란 구**: YAML에 적은 사용자 웨이포인트 위치.
- **적색 선**: 목표 작업공간(FK q_des)과 현재 EE 위치를 잇는 선.

실행 예::

    cd pmi_cable_mujoco
    python scripts/demo_vsd_torque_path_follow.py
    python scripts/demo_vsd_torque_path_follow.py --config configs/control_params_vsd_task.yaml
    python scripts/demo_vsd_torque_path_follow.py --headless --steps 3000 --settle-steps 120
    python scripts/demo_vsd_torque_path_follow.py --ks 150,150,150,0,0 --kd 25,25,25,0,0
    python scripts/demo_vsd_torque_path_follow.py --settle-steps 80
    python scripts/demo_vsd_torque_path_follow.py --no-transmission   # 강체 전달만

시뮬 결과를 위치제어와 같은 형식의 그래프로 저장하려면 ``plot_vsd_torque_path_desired_vs_ee.py`` 를 사용한다.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple, Union

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import mujoco
import mujoco.viewer

from transmission.antagonistic_cable_joint import AntagonisticCableStack
from transmission.belt_model import BeltTransmissionModel
from transmission.hybrid_joint_torque import (
    apply_transmission_joint_torque,
    build_belt_cable_models,
)

from controllers.mujoco_joint_feedforward import joint_gravity_reference_torques
from controllers.task_space_vsd import (
    compose_ctrl_torque_pd_then_bias,
    compute_run_vsd_joint_torques,
    load_task_space_vsd_from_cfg as _default_task_vsd_gains,
)
from controllers.vsd_controller import ReferenceTrajectory
from controllers.waypoint_trajectory import WaypointTrajectory
from kinematics.pmi_chain import (
    JOINT_LIMIT_RAD_MAX,
    JOINT_LIMIT_RAD_MIN,
    actuator_torque_from_joint_torque,
    fk_ee_pose_joint_rad,
    jacobian_5x4_joint_rad,
)

PROJECT_ROOT = Path(ROOT)
DEFAULT_MODEL = PROJECT_ROOT / "models" / "pmi_cable_arm.xml"
DEFAULT_CONFIG = PROJECT_ROOT / "configs" / "control_params_vsd_task.yaml"
JOINT_NAMES = ("jnt1", "jnt2", "jnt3", "jnt4")


def _load_yaml(path: Path) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _joint_adr(model: mujoco.MjModel) -> Tuple[list[int], list[int]]:
    qpos_adr, dof_adr = [], []
    for jn in JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid < 0:
            raise RuntimeError(f"Joint not found: {jn}")
        qpos_adr.append(int(model.jnt_qposadr[jid]))
        dof_adr.append(int(model.jnt_dofadr[jid]))
    return qpos_adr, dof_adr


def _clip_q(q: np.ndarray) -> np.ndarray:
    return np.clip(
        np.asarray(q, dtype=np.float64).reshape(4),
        JOINT_LIMIT_RAD_MIN,
        JOINT_LIMIT_RAD_MAX,
    )


def _parse_vec5(s: str) -> np.ndarray:
    """Parse comma-separated 5D vector for Ks or Kd."""
    parts = [float(x.strip()) for x in s.split(",")]
    if len(parts) != 5:
        raise argparse.ArgumentTypeError(
            "5개 실수를 콤마로 구분해 주세요 (예: 140,140,140,0,0)"
        )
    return np.asarray(parts, dtype=np.float64)


def _parse_vec4(s: str) -> np.ndarray:
    """Parse comma-separated 4D vector for tau_extra."""
    parts = [float(x.strip()) for x in s.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(
            "4개 실수를 콤마로 구분해 주세요 (예: 0,0,0,0)"
        )
    return np.asarray(parts, dtype=np.float64)


def _settle_vsd_on_current_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    qpos_adr: list[int],
    dof_adr: list[int],
    frame_skip: int,
    settle_steps: int,
    Ks: np.ndarray,
    Kd: np.ndarray,
    gravity_bias_ff: bool,
    gravity_compensation_gain: float,
    hybrid_dt: float,
    belt: Optional[BeltTransmissionModel] = None,
    cable: Optional[AntagonisticCableStack] = None,
) -> None:
    """Stabilize on current pose using VSD (hold target = current EE position)."""
    tau_ex = np.zeros(4, dtype=np.float64)
    qd_zero = np.zeros(4, dtype=np.float64)
    for _ in range(max(0, int(settle_steps))):
        mujoco.mj_forward(model, data)
        q = np.array([float(data.qpos[a]) for a in qpos_adr], dtype=np.float64)
        qd = np.array([float(data.qvel[a]) for a in dof_adr], dtype=np.float64)
        ee, _ = fk_ee_pose_joint_rad(q)
        des_vel = np.zeros(5, dtype=np.float64)
        tau_vsd, _, _ = compute_run_vsd_joint_torques(q, qd, ee, des_vel, Ks, Kd)
        bias = np.array([float(data.qfrc_bias[dof_adr[i]]) for i in range(4)], dtype=np.float64)
        tau_g = (
            joint_gravity_reference_torques(model, data, dof_adr)
            if float(gravity_compensation_gain) > 1e-12
            else None
        )
        tau_j = compose_ctrl_torque_pd_then_bias(
            tau_vsd,
            tau_ex,
            bias,
            gravity_bias_ff=gravity_bias_ff,
            tau_gravity_joint=tau_g,
            gravity_compensation_gain=float(gravity_compensation_gain),
        )
        tau_j, _ = apply_transmission_joint_torque(
            tau_j, q, qd, q, qd_zero, float(hybrid_dt), belt, cable
        )
        data.ctrl[: model.nu] = actuator_torque_from_joint_torque(tau_j)
        for _ in range(frame_skip):
            mujoco.mj_step(model, data)


def vsd_torque_path_control_step(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    traj: Union[ReferenceTrajectory, WaypointTrajectory],
    qpos_adr: list[int],
    dof_adr: list[int],
    Ks: np.ndarray,
    Kd: np.ndarray,
    tau_extra: np.ndarray,
    gravity_bias_ff: bool,
    gravity_compensation_gain: float,
    frame_skip: int,
    hybrid_dt: float,
    belt: Optional[BeltTransmissionModel] = None,
    cable: Optional[AntagonisticCableStack] = None,
    waypoint_task_from: str = "fk_q_des",
) -> Tuple[np.ndarray, np.ndarray, float, np.ndarray, Dict[str, Any]]:
    """
    한 제어 주기: VSD 토크 적용 후 ``frame_skip`` 번 ``mj_step``.

    Returns
    -------
    q, q_des
        측정·참조 관절각 (4,).
    pos_err
        작업공간 위치 오차 ``||err[:3]||`` [m].
    err
        5D 작업공간 오차 벡터 (로깅·RMSE용).
    meta
        ``tau_vsd``, ``tau_joint_cmd``, ``tau_joint_delivered``, ``belt_diag``, ``cable_transmission``.
    """
    mujoco.mj_forward(model, data)
    q = np.array([float(data.qpos[a]) for a in qpos_adr], dtype=np.float64)
    qd = np.array([float(data.qvel[a]) for a in dof_adr], dtype=np.float64)

    q_des, qdot_des = traj.step()
    q_des = _clip_q(q_des)

    if (
        str(waypoint_task_from) == "path_xyz"
        and isinstance(traj, WaypointTrajectory)
    ):
        p_xyz, v_xyz = traj.workspace_desired_pos_vel_at_step()
        if p_xyz is not None:
            ee_des = p_xyz.copy()
            if v_xyz is None:
                v_xyz = np.zeros(3, dtype=np.float64)
            des_vel5 = np.concatenate(
                [np.asarray(v_xyz, dtype=np.float64).reshape(3), np.zeros(2, dtype=np.float64)]
            )
        else:
            ee_des, _ = fk_ee_pose_joint_rad(q_des)
            J_des = jacobian_5x4_joint_rad(q_des)
            des_vel5 = np.concatenate(
                [J_des[:3, :] @ qdot_des, np.zeros(2, dtype=np.float64)]
            )
    else:
        ee_des, _ = fk_ee_pose_joint_rad(q_des)
        J_des = jacobian_5x4_joint_rad(q_des)
        des_vel5 = np.concatenate(
            [J_des[:3, :] @ qdot_des, np.zeros(2, dtype=np.float64)]
        )

    tau_vsd, err, _ev = compute_run_vsd_joint_torques(q, qd, ee_des, des_vel5, Ks, Kd)
    bias = np.array([float(data.qfrc_bias[dof_adr[i]]) for i in range(4)], dtype=np.float64)
    tau_g = (
        joint_gravity_reference_torques(model, data, dof_adr)
        if float(gravity_compensation_gain) > 1e-12
        else None
    )
    tau_j = compose_ctrl_torque_pd_then_bias(
        tau_vsd,
        tau_extra,
        bias,
        gravity_bias_ff=gravity_bias_ff,
        tau_gravity_joint=tau_g,
        gravity_compensation_gain=float(gravity_compensation_gain),
    )
    tau_joint_cmd = tau_j.copy()
    tau_j, trans_diag = apply_transmission_joint_torque(
        tau_j, q, qd, q_des, qdot_des, float(hybrid_dt), belt, cable
    )
    data.ctrl[: model.nu] = actuator_torque_from_joint_torque(tau_j)

    for _ in range(frame_skip):
        mujoco.mj_step(model, data)

    pos_err = float(np.linalg.norm(err[:3]))
    meta: Dict[str, Any] = {
        "tau_vsd": tau_vsd,
        "tau_joint_cmd": tau_joint_cmd,
        "tau_joint_delivered": tau_j,
        "belt_diag": trans_diag.get("belt_diag"),
        "cable_transmission": trans_diag.get("cable_transmission", {}),
    }
    return q, q_des, pos_err, err, meta


def _viewer_fill_tracking_overlay(
    viewer: mujoco.viewer.Handle,
    *,
    traj: Union[ReferenceTrajectory, WaypointTrajectory],
    is_wp: bool,
    q: np.ndarray,
    q_des: np.ndarray,
    draw_overlay: bool,
    max_path_segments: int = 420,
    path_line_px: float = 4.0,
    err_line_px: float = 7.0,
) -> None:
    """Fill viewer overlay: path (green), waypoints (yellow), EE error (red)."""
    scn = viewer.user_scn
    if scn is None or not draw_overlay:
        return

    mat_flat = np.eye(3, dtype=np.float64).flatten()
    gidx = 0
    ncap = len(scn.geoms)
    rgba_green = np.array([0.15, 0.92, 0.28, 0.9], dtype=np.float32)
    rgba_yellow = np.array([0.98, 0.92, 0.15, 0.95], dtype=np.float32)
    rgba_red = np.array([0.95, 0.12, 0.12, 0.95], dtype=np.float32)

    def add_line(p0: np.ndarray, p1: np.ndarray, rgba: np.ndarray, wpx: float) -> None:
        nonlocal gidx
        if gidx >= ncap:
            return
        geom = scn.geoms[gidx]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_LINE,
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            mat_flat,
            rgba,
        )
        mujoco.mjv_connector(
            geom,
            mujoco.mjtGeom.mjGEOM_LINE,
            float(wpx),
            np.asarray(p0, dtype=np.float64),
            np.asarray(p1, dtype=np.float64),
        )
        gidx += 1

    def add_sphere(center: np.ndarray, rgba: np.ndarray, radius: float) -> None:
        nonlocal gidx
        if gidx >= ncap:
            return
        geom = scn.geoms[gidx]
        size = np.array([radius, 0.0, 0.0], dtype=np.float64)
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            size,
            np.asarray(center, dtype=np.float64),
            mat_flat,
            rgba,
        )
        gidx += 1

    if is_wp and isinstance(traj, WaypointTrajectory):
        path = traj.cartesian_path_xyz()
        if path.shape[0] >= 2:
            n = path.shape[0]
            if n > max_path_segments:
                idx = np.linspace(0, n - 1, max_path_segments).astype(int)
                pts = path[idx]
            else:
                pts = path
            for i in range(len(pts) - 1):
                add_line(pts[i], pts[i + 1], rgba_green, path_line_px)
        wps = traj.user_waypoints_xyz()
        for i in range(wps.shape[0]):
            add_sphere(wps[i], rgba_yellow, 0.015)

    ee_des, _ = fk_ee_pose_joint_rad(q_des)
    ee_act, _ = fk_ee_pose_joint_rad(q)
    add_line(ee_des, ee_act, rgba_red, err_line_px)

    scn.ngeom = gidx


def _build_trajectory(
    control_cfg: Dict[str, Any], ctrl_dt: float
) -> Tuple[Union[ReferenceTrajectory, WaypointTrajectory], bool]:
    traj_cfg = control_cfg.get("trajectory", {})
    mode = str(traj_cfg.get("mode", "sine"))
    if mode == "waypoints_xyz":
        return WaypointTrajectory(traj_cfg, ctrl_dt), True
    return ReferenceTrajectory.from_config(traj_cfg, ctrl_dt), False


def _safe_viewer_shutdown(handle: mujoco.viewer.Handle, delay_s: float) -> None:
    """Safely shutdown viewer with optional delay."""
    if delay_s > 0:
        time.sleep(float(delay_s))
    try:
        with handle.lock():
            handle.close()
    except mujoco.UnexpectedError:
        pass


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL,
        help="토크 모터 MJCF (기본: pmi_cable_arm.xml)",
    )
    ap.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help="control_params YAML (기본: control_params_vsd_task.yaml)",
    )
    ap.add_argument("--headless", action="store_true", help="뷰어 없이 시뮬만")
    ap.add_argument("--steps", type=int, default=0, help="headless 시 최대 제어 스텝 (0=무한)")
    ap.add_argument(
        "--print-every",
        type=int,
        default=250,
        help="상태 출력 간격 [제어 스텝]",
    )
    ap.add_argument(
        "--ks",
        type=_parse_vec5,
        default=None,
        help="작업공간 VSD 비례 게인 (5D, 콤마 구분). 미지정 시 YAML 값 사용",
    )
    ap.add_argument(
        "--kd",
        type=_parse_vec5,
        default=None,
        help="작업공간 VSD 미분 게인 (5D, 콤마 구분). 미지정 시 YAML 값 사용",
    )
    ap.add_argument(
        "--no-overlay",
        action="store_true",
        help="뷰어의 경로/웨이포인트/EE 오차 선 그리지 않음",
    )
    ap.add_argument(
        "--settle-steps",
        type=int,
        default=0,
        help="경로 재생 전 현재 자세 유지 VSD 안정화 스텝 수",
    )
    ap.add_argument(
        "--viewer-shutdown-delay",
        type=float,
        default=0.08,
        help="뷰어 종료 대기 [s] (GLXBadDrawable 완화용)",
    )
    ap.add_argument(
        "--tau-extra",
        type=_parse_vec4,
        default=np.zeros(4),
        help="매 스텝 가산 관절 토크 (4D, 콤마 구분)",
    )
    ap.add_argument(
        "--gravity-bias-ff",
        action="store_true",
        help="중력 항 피드포워드 활성화 (--no-gravity-bias-ff로 끔)",
    )
    ap.add_argument(
        "--no-gravity-bias-ff",
        action="store_true",
        help="YAML 설정 무시하고 중력 항 피드포워드 비활성화",
    )
    ap.add_argument(
        "--gravity-compensation-gain",
        type=float,
        default=None,
        help="YAML ``gravity_compensation_gain`` 덮어쓰기 (qvel=0 기준 중력 참조 토크 배율)",
    )
    ap.add_argument(
        "--ee-arrival-tol",
        type=float,
        default=0.03,
        help="waypoints_xyz: 마지막 웨이포인트까지 EE 거리 [m]",
    )
    ap.add_argument(
        "--ee-arrival-hold",
        type=int,
        default=30,
        help="waypoints_xyz: 목표 도달 판정 연속 스텝 수",
    )
    ap.add_argument(
        "--ee-arrival-max-steps",
        type=int,
        default=120_000,
        help="waypoints_xyz: 안전 상한 제어 스텝",
    )
    ap.add_argument(
        "--no-transmission",
        action="store_true",
        help="벨트·케이블 Python 전달 모델 끔 (MJCF 등식만인 강체 VSD)",
    )
    ap.add_argument(
        "--belt-config",
        type=Path,
        default=PROJECT_ROOT / "configs" / "belt_params.yaml",
        help="벨트 파라미터 YAML",
    )
    ap.add_argument(
        "--cable-config",
        type=Path,
        default=PROJECT_ROOT / "configs" / "cable_params.yaml",
        help="케이블 파라미터 YAML",
    )
    ap.add_argument(
        "--transmission-randomize",
        action="store_true",
        help="belt/cable YAML 의 randomization 유지 (기본: 비활성, 재현성)",
    )
    ap.add_argument(
        "--waypoint-task-from",
        choices=("fk_q_des", "path_xyz"),
        default="fk_q_des",
        help="waypoints_xyz: 목표 EE 위치를 FK(q_des)(기본) 또는 플래너 path_xyz 샘플에서 취함",
    )
    args = ap.parse_args()

    control_cfg = _load_yaml(Path(args.config))
    sim_cfg = control_cfg.get("simulation", {})
    frame_skip = int(sim_cfg.get("frame_skip", 1))

    Ks, Kd, gff_yaml, gcomp_yaml = _default_task_vsd_gains(control_cfg)
    if args.ks is not None:
        Ks = args.ks
    if args.kd is not None:
        Kd = args.kd

    gravity_bias_ff = bool(gff_yaml)
    if args.gravity_bias_ff:
        gravity_bias_ff = True
    if args.no_gravity_bias_ff:
        gravity_bias_ff = False
    gcomp = (
        float(gcomp_yaml)
        if args.gravity_compensation_gain is None
        else float(args.gravity_compensation_gain)
    )

    model = mujoco.MjModel.from_xml_path(os.fspath(args.model))
    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)
    ctrl_dt = dt * frame_skip
    hybrid_dt = dt

    belt: Optional[BeltTransmissionModel] = None
    cable: Optional[AntagonisticCableStack] = None
    if not args.no_transmission:
        belt, cable = build_belt_cable_models(
            Path(args.belt_config),
            Path(args.cable_config),
            randomize=bool(args.transmission_randomize),
        )

    if model.nu != 4:
        print(f"warning: nu={model.nu}, 기대 4 (q1_act..q4_act motor)", flush=True)

    traj, is_wp = _build_trajectory(control_cfg, ctrl_dt)
    qpos_adr, dof_adr = _joint_adr(model)
    tau_extra = np.asarray(args.tau_extra, dtype=np.float64).reshape(4)

    mujoco.mj_resetData(model, data)
    if is_wp:
        q_seed = np.array([float(data.qpos[a]) for a in qpos_adr], dtype=np.float64)
        traj.on_reset(
            q_seed,
            mujoco_data=data,
            joint_qpos_adr=qpos_adr,
        )
    else:
        traj.reset()

    mujoco.mj_forward(model, data)
    if belt is not None:
        belt.reset()
    if cable is not None:
        cable.reset()

    settle_n = max(0, int(args.settle_steps))
    if settle_n > 0:
        _settle_vsd_on_current_pose(
            model,
            data,
            qpos_adr=qpos_adr,
            dof_adr=dof_adr,
            frame_skip=frame_skip,
            settle_steps=settle_n,
            Ks=Ks,
            Kd=Kd,
            gravity_bias_ff=gravity_bias_ff,
            gravity_compensation_gain=gcomp,
            hybrid_dt=hybrid_dt,
            belt=belt,
            cable=cable,
        )
        # settle 로 관절이 움직였으므로, YAML 초기 자세 기준으로 빌드된 q_des 궤적을 현재 q 에서 재플랜
        if is_wp and isinstance(traj, WaypointTrajectory):
            q_now = np.array([float(data.qpos[a]) for a in qpos_adr], dtype=np.float64)
            traj.on_reset(
                q_now,
                mujoco_data=data,
                joint_qpos_adr=qpos_adr,
                apply_yaml_initial_pose=False,
            )
            mujoco.mj_forward(model, data)

    goal_xyz_wp: np.ndarray | None = None
    wp_playback_len = 0
    if is_wp and isinstance(traj, WaypointTrajectory):
        goal_xyz_wp = traj.final_waypoint_xyz_world()
        wp_playback_len = traj.playback_num_steps()

    err_acc = 0.0
    n_err = 0

    def one_control_step() -> Tuple[np.ndarray, np.ndarray, float]:
        nonlocal err_acc, n_err
        q, q_des, pos_err, err, _meta = vsd_torque_path_control_step(
            model,
            data,
            traj=traj,
            qpos_adr=qpos_adr,
            dof_adr=dof_adr,
            Ks=Ks,
            Kd=Kd,
            tau_extra=tau_extra,
            gravity_bias_ff=gravity_bias_ff,
            gravity_compensation_gain=gcomp,
            frame_skip=frame_skip,
            hybrid_dt=hybrid_dt,
            belt=belt,
            cable=cable,
            waypoint_task_from=str(args.waypoint_task_from),
        )
        err_acc += float(np.dot(err[:3], err[:3]))
        n_err += 1
        return q, q_des, pos_err

    if args.headless:
        max_steps = int(args.steps) if args.steps > 0 else int(args.ee_arrival_max_steps)
        max_steps = min(max_steps, int(args.ee_arrival_max_steps))
        ee_hold_h = 0
        for k in range(max_steps):
            q, q_des, pe = one_control_step()
            pe_interval = int(args.print_every)
            if pe_interval > 0 and (k + 1) % pe_interval == 0:
                ee, _ = fk_ee_pose_joint_rad(q)
                extra = ""
                if (
                    goal_xyz_wp is not None
                    and wp_playback_len > 0
                    and (k + 1) >= wp_playback_len
                ):
                    d = float(np.linalg.norm(ee - goal_xyz_wp))
                    extra = f"  ee_to_final_wp={d:.4f}m"
                print(
                    f"step {k+1}  task_pos_err={pe:.5f}m  ee_xyz=[{ee[0]:.3f} {ee[1]:.3f} {ee[2]:.3f}]{extra}",
                    flush=True,
                )

            if (
                goal_xyz_wp is not None
                and wp_playback_len > 0
                and (k + 1) >= wp_playback_len
            ):
                ee, _ = fk_ee_pose_joint_rad(q)
                if float(np.linalg.norm(ee - goal_xyz_wp)) < float(args.ee_arrival_tol):
                    ee_hold_h += 1
                else:
                    ee_hold_h = 0
                if ee_hold_h >= int(args.ee_arrival_hold):
                    break

            if goal_xyz_wp is not None and (k + 1) >= int(args.ee_arrival_max_steps):
                print(
                    "warning: ee_arrival_max_steps reached without EE arrival",
                    flush=True,
                )
                break

        if n_err > 0:
            rmse = float(np.sqrt(err_acc / n_err))
            print(f"mean task_pos RMSE [m]: {rmse:.5f}", flush=True)
        return

    # Viewer mode
    viewer_handle = mujoco.viewer.launch_passive(model, data)
    viewer_closed = False
    try:
        step_i = 0
        ee_hold_v = 0
        wall_prev = time.time()
        draw_overlay = not args.no_overlay
        while viewer_handle.is_running():
            q, q_des, pe = one_control_step()
            step_i += 1

            pe_interval = int(args.print_every)
            if pe_interval > 0 and step_i % pe_interval == 0:
                ee, _ = fk_ee_pose_joint_rad(q)
                extra = ""
                if (
                    goal_xyz_wp is not None
                    and wp_playback_len > 0
                    and step_i >= wp_playback_len
                ):
                    d = float(np.linalg.norm(ee - goal_xyz_wp))
                    extra = f"  ee_to_final_wp={d:.4f}m"
                print(
                    f"step {step_i}  task_pos_err={pe:.5f}m  ee_xyz=[{ee[0]:.3f} {ee[1]:.3f} {ee[2]:.3f}]{extra}",
                    flush=True,
                )

            _viewer_fill_tracking_overlay(
                viewer_handle,
                traj=traj,
                is_wp=is_wp,
                q=q,
                q_des=q_des,
                draw_overlay=draw_overlay,
            )
            viewer_handle.sync()

            # Approximate realtime (simulate at sim dt * frame_skip per second)
            target_period = dt * frame_skip
            now = time.time()
            spare = wall_prev + target_period - now
            if spare > 0:
                time.sleep(spare)
            wall_prev = time.time()

            if (
                goal_xyz_wp is not None
                and wp_playback_len > 0
                and step_i >= wp_playback_len
            ):
                ee_v, _ = fk_ee_pose_joint_rad(q)
                if float(np.linalg.norm(ee_v - goal_xyz_wp)) < float(
                    args.ee_arrival_tol
                ):
                    ee_hold_v += 1
                else:
                    ee_hold_v = 0
                if ee_hold_v >= int(args.ee_arrival_hold):
                    _safe_viewer_shutdown(
                        viewer_handle, float(args.viewer_shutdown_delay)
                    )
                    viewer_closed = True
                    break

            if (
                goal_xyz_wp is not None
                and step_i >= int(args.ee_arrival_max_steps)
            ):
                print(
                    "warning: ee_arrival_max_steps reached without EE arrival",
                    flush=True,
                )
                _safe_viewer_shutdown(
                    viewer_handle, float(args.viewer_shutdown_delay)
                )
                viewer_closed = True
                break
    finally:
        if not viewer_closed:
            _safe_viewer_shutdown(
                viewer_handle, float(args.viewer_shutdown_delay)
            )


if __name__ == "__main__":
    main()
