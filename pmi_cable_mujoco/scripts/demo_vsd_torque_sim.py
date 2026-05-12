#!/usr/bin/env python3
"""MuJoCo 토크 모터 + 작업공간 VSD (``ControlMain::run_vsd`` 와 동일 수식).

``models/pmi_cable_arm.xml`` 의 ``motor`` 는 **구동축** ``q*_act`` 에 걸리며 ``data.ctrl`` 은 구동축 토크 [N·m]이다. VSD 출력(관절 토크)은 ``actuator_torque_from_joint_torque`` 로 변환한다.

- 목표: ``des_pos`` (경로 xyz 또는 관절 기준 FK), ``des_roll=-π/2``, ``des_pitch=0``
- ``τ = Jᵀ (K_s ⊙ e + K_d ⊙ e_v)`` (``controllers/task_space_vsd.py``)
- **기본**: ``transmission/hybrid_joint_torque.py`` 로 벨트(jnt1)·케이블(jnt2–4) 잔차를 ``pmi_cable_arm_env`` 와 동일하게 합산. ``--no-transmission`` 으로 끔.
- 중력·바이어스: YAML ``task_space_vsd.gravity_bias_feedforward`` / ``--gravity-bias-ff`` (``qfrc_bias`` 가산). 토크는 스크립트에서 ``clip`` 하지 않으며 MJCF 모터 ``forcerange`` 만 적용된다.

초기 자세: YAML ``trajectory.initial_actuator_rad`` (길이 4, **구동축** ``q_act`` [rad], ``q_jnt = gear * q_act``). 웨이포인트는 ``WaypointTrajectory.on_reset`` 이 ``jnt1..4`` ``qpos`` 에 반영한다. ``sine``/``step_hold`` 는 리셋 직후 같은 필드로 관절을 맞춘다. ``--initial-actuator-rad a1,a2,a3,a4`` 로 YAML 값을 덮어쓸 수 있다.

추가 토크: ``--tau-extra`` 는 **관절** 기준 [N·m] 가산 후 구동축으로 변환된다.

실행 예::

    cd pmi_cable_mujoco
    python scripts/demo_vsd_torque_sim.py
    python scripts/demo_vsd_torque_sim.py --config configs/control_params_vsd_task.yaml
    python scripts/demo_vsd_torque_sim.py --headless --steps 3000 --settle-steps 120
    python scripts/demo_vsd_torque_sim.py --tau-extra 0,0,0.5,0 --gravity-bias-ff
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
    joint_reference_to_task_desires,
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
    joint_rad_from_actuator_rad,
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


def _parse_vec4(s: str) -> np.ndarray:
    parts = [float(x.strip()) for x in s.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("4개 실수를 콤마로 구분해 주세요 (예: 0,0,0.2,0)")
    return np.asarray(parts, dtype=np.float64)


def _apply_initial_actuator_to_joints(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    qpos_adr: list[int],
    actuator_rad: np.ndarray,
) -> None:
    """``initial_actuator_rad`` → 관절 ``qpos`` (``joint_rad_from_actuator_rad``) 후 ``mj_forward``."""
    qj = _clip_q(joint_rad_from_actuator_rad(np.asarray(actuator_rad, dtype=np.float64)))
    for i, adr in enumerate(qpos_adr):
        data.qpos[adr] = float(qj[i])
    mujoco.mj_forward(model, data)


def _safe_viewer_shutdown(handle: mujoco.viewer.Handle, delay_s: float) -> None:
    if delay_s > 0:
        time.sleep(float(delay_s))
    try:
        with handle.lock():
            handle.close()
    except mujoco.UnexpectedError:
        pass


def _viewer_fill_err_overlay(
    viewer: mujoco.viewer.Handle,
    *,
    q: np.ndarray,
    q_des: np.ndarray,
    draw: bool,
) -> None:
    if not draw:
        return
    scn = viewer.user_scn
    if scn is None:
        return
    mat_flat = np.eye(3, dtype=np.float64).flatten()
    rgba_red = np.array([0.95, 0.12, 0.12, 0.95], dtype=np.float32)
    ee_des, _ = fk_ee_pose_joint_rad(q_des)
    ee_act, _ = fk_ee_pose_joint_rad(q)
    geom = scn.geoms[0]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_LINE,
        np.zeros(3, dtype=np.float64),
        np.zeros(3, dtype=np.float64),
        mat_flat,
        rgba_red,
    )
    mujoco.mjv_connector(
        geom,
        mujoco.mjtGeom.mjGEOM_LINE,
        7.0,
        np.asarray(ee_des, dtype=np.float64),
        np.asarray(ee_act, dtype=np.float64),
    )
    scn.ngeom = 1


def _build_trajectory(
    control_cfg: Dict[str, Any], ctrl_dt: float
) -> Tuple[Union[ReferenceTrajectory, WaypointTrajectory], bool]:
    traj_cfg = control_cfg.get("trajectory", {})
    mode = str(traj_cfg.get("mode", "sine"))
    if mode == "waypoints_xyz":
        return WaypointTrajectory(traj_cfg, ctrl_dt), True
    return ReferenceTrajectory.from_config(traj_cfg, ctrl_dt), False


def _task_desires_after_step(
    traj: Union[ReferenceTrajectory, WaypointTrajectory],
    is_wp: bool,
    q_des: np.ndarray,
    qdot_des: np.ndarray,
    *,
    waypoint_task_from: str,
) -> Tuple[np.ndarray, np.ndarray]:
    qd = np.asarray(qdot_des, dtype=np.float64).reshape(4)
    qcl = _clip_q(q_des)
    Jd = jacobian_5x4_joint_rad(qcl)
    v_lin = (Jd[:3, :] @ qd).reshape(3)
    des_vel5 = np.concatenate([v_lin, np.zeros(2, dtype=np.float64)])

    if is_wp and isinstance(traj, WaypointTrajectory) and waypoint_task_from == "path_xyz":
        p = traj.workspace_target_xyz_at_step()
        if p is not None:
            return p.copy(), des_vel5
    return joint_reference_to_task_desires(qcl, qdot_des)


def _settle_vsd_hold(
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
    tau_extra: np.ndarray,
    hybrid_dt: float,
    belt: Optional[BeltTransmissionModel] = None,
    cable: Optional[AntagonisticCableStack] = None,
) -> None:
    """현재 FK 자세를 목표로 한 작업공간 VSD로 짧게 안정화 (포지션 서보 없음)."""
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
            tau_extra,
            bias,
            gravity_bias_ff=gravity_bias_ff,
            tau_gravity_joint=tau_g,
            gravity_compensation_gain=float(gravity_compensation_gain),
        )
        tau_j, _ = apply_transmission_joint_torque(
            tau_j, q, qd, q, qd_zero, float(hybrid_dt), belt, cable
        )
        data.ctrl[: model.nu] = actuator_torque_from_joint_torque(tau_j)
        for _fs in range(frame_skip):
            mujoco.mj_step(model, data)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="토크 motor MJCF")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--steps", type=int, default=0, help="headless 최대 제어 스텝 (0=무한)")
    ap.add_argument("--print-every", type=int, default=200)
    ap.add_argument("--settle-steps", type=int, default=0, help="경로 전 현재 자세 유지 VSD 스텝")
    ap.add_argument(
        "--tau-extra",
        type=_parse_vec4,
        default=np.zeros(4),
        help="매 스텝 가산 관절 토크 4개, 콤마 구분",
    )
    ap.add_argument(
        "--gravity-bias-ff",
        action="store_true",
        help="YAML 설정보다 우선해 qfrc_bias 관절 성분을 토크에 가산",
    )
    ap.add_argument("--no-gravity-bias-ff", action="store_true", help="YAML 의 bias 플래그 끔")
    ap.add_argument(
        "--gravity-compensation-gain",
        type=float,
        default=None,
        help="YAML ``gravity_compensation_gain`` 덮어쓰기 (정적 중력 참조 토크 배율)",
    )
    ap.add_argument("--viewer-shutdown-delay", type=float, default=0.08)
    ap.add_argument("--overlay", action="store_true", help="뷰어에 EE 오차 선(적색) 표시")
    ap.add_argument(
        "--ee-arrival-tol",
        type=float,
        default=0.03,
        help="waypoints: 마지막 웨이포인트까지 EE 거리 [m]",
    )
    ap.add_argument("--ee-arrival-hold", type=int, default=30)
    ap.add_argument("--ee-arrival-max-steps", type=int, default=120_000)
    ap.add_argument(
        "--waypoint-task-from",
        choices=("fk_q_des", "path_xyz"),
        default="fk_q_des",
        help="waypoints_xyz: 목표 EE 위치를 FK(q_des)(기본) 또는 path_xyz 샘플(C++ 경로 xyz)에서 취함",
    )
    ap.add_argument(
        "--initial-actuator-rad",
        type=_parse_vec4,
        default=None,
        help="구동축 초기각 [rad] 4개(콤마). 지정 시 trajectory.initial_actuator_rad 를 덮어씀",
    )
    ap.add_argument(
        "--no-transmission",
        action="store_true",
        help="벨트·케이블 Python 전달 모델 끔",
    )
    ap.add_argument(
        "--belt-config",
        type=Path,
        default=PROJECT_ROOT / "configs" / "belt_params.yaml",
    )
    ap.add_argument(
        "--cable-config",
        type=Path,
        default=PROJECT_ROOT / "configs" / "cable_params.yaml",
    )
    ap.add_argument(
        "--transmission-randomize",
        action="store_true",
        help="belt/cable YAML randomization 유지",
    )
    args = ap.parse_args()

    control_cfg = _load_yaml(Path(args.config))
    if args.initial_actuator_rad is not None:
        control_cfg.setdefault("trajectory", {})["initial_actuator_rad"] = [
            float(x) for x in np.asarray(args.initial_actuator_rad, dtype=np.float64).reshape(4)
        ]
    sim_cfg = control_cfg.get("simulation", {})
    frame_skip = int(sim_cfg.get("frame_skip", 1))

    Ks, Kd, gff_yaml, gcomp_yaml = _default_task_vsd_gains(control_cfg)
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
        print(f"warning: nu={model.nu}, 기대 4 (jnt1..4 motor)", flush=True)

    traj, is_wp = _build_trajectory(control_cfg, ctrl_dt)
    qpos_adr, dof_adr = _joint_adr(model)
    tau_extra = np.asarray(args.tau_extra, dtype=np.float64).reshape(4)

    mujoco.mj_resetData(model, data)
    traj_cfg = control_cfg.get("trajectory", {})
    if is_wp:
        q_seed = np.array([float(data.qpos[a]) for a in qpos_adr], dtype=np.float64)
        traj.on_reset(
            q_seed,
            mujoco_data=data,
            joint_qpos_adr=qpos_adr,
        )
    else:
        act0 = traj_cfg.get("initial_actuator_rad")
        if act0 is not None:
            _apply_initial_actuator_to_joints(model, data, qpos_adr, np.asarray(act0, dtype=float))
        traj.reset()

    mujoco.mj_forward(model, data)
    if belt is not None:
        belt.reset()
    if cable is not None:
        cable.reset()

    _settle_vsd_hold(
        model,
        data,
        qpos_adr=qpos_adr,
        dof_adr=dof_adr,
        frame_skip=frame_skip,
        settle_steps=int(args.settle_steps),
        Ks=Ks,
        Kd=Kd,
        gravity_bias_ff=gravity_bias_ff,
        gravity_compensation_gain=gcomp,
        tau_extra=tau_extra,
        hybrid_dt=hybrid_dt,
        belt=belt,
        cable=cable,
    )
    if int(args.settle_steps) > 0 and is_wp and isinstance(traj, WaypointTrajectory):
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

    def one_control_step() -> Tuple[np.ndarray, np.ndarray, float]:
        mujoco.mj_forward(model, data)
        q = np.array([float(data.qpos[a]) for a in qpos_adr], dtype=np.float64)
        qd = np.array([float(data.qvel[a]) for a in dof_adr], dtype=np.float64)
        q_des, qdot_des = traj.step()
        q_des = _clip_q(q_des)
        des_pos, des_vel5 = _task_desires_after_step(
            traj,
            is_wp,
            q_des,
            qdot_des,
            waypoint_task_from=str(args.waypoint_task_from),
        )
        tau_vsd, err, _ev = compute_run_vsd_joint_torques(q, qd, des_pos, des_vel5, Ks, Kd)
        bias = np.array([float(data.qfrc_bias[dof_adr[i]]) for i in range(4)], dtype=np.float64)
        tau_g = (
            joint_gravity_reference_torques(model, data, dof_adr)
            if float(gcomp) > 1e-12
            else None
        )
        tau_j = compose_ctrl_torque_pd_then_bias(
            tau_vsd,
            tau_extra,
            bias,
            gravity_bias_ff=gravity_bias_ff,
            tau_gravity_joint=tau_g,
            gravity_compensation_gain=float(gcomp),
        )
        tau_j, _ = apply_transmission_joint_torque(
            tau_j, q, qd, q_des, qdot_des, float(hybrid_dt), belt, cable
        )
        data.ctrl[: model.nu] = actuator_torque_from_joint_torque(tau_j)
        for _ in range(frame_skip):
            mujoco.mj_step(model, data)
        pos_err = float(np.linalg.norm(err[:3]))
        return q, q_des, pos_err

    if args.headless:
        max_steps = int(args.steps) if args.steps > 0 else int(args.ee_arrival_max_steps)
        max_steps = min(max_steps, int(args.ee_arrival_max_steps))
        ee_hold = 0
        for k in range(max_steps):
            q, q_des, pe = one_control_step()
            pev = int(args.print_every)
            if pev > 0 and (k + 1) % pev == 0:
                ee, _ = fk_ee_pose_joint_rad(q)
                extra = ""
                if goal_xyz_wp is not None and wp_playback_len > 0 and (k + 1) >= wp_playback_len:
                    d = float(np.linalg.norm(ee - goal_xyz_wp))
                    extra = f"  ee_to_final_wp={d:.4f}m"
                print(
                    f"step {k+1}  task_pos_err={pe:.5f}m  ee_xyz=[{ee[0]:.3f} {ee[1]:.3f} {ee[2]:.3f}]{extra}",
                    flush=True,
                )
            if goal_xyz_wp is not None and wp_playback_len > 0 and (k + 1) >= wp_playback_len:
                ee, _ = fk_ee_pose_joint_rad(q)
                if float(np.linalg.norm(ee - goal_xyz_wp)) < float(args.ee_arrival_tol):
                    ee_hold += 1
                else:
                    ee_hold = 0
                if ee_hold >= int(args.ee_arrival_hold):
                    break
            if goal_xyz_wp is not None and (k + 1) >= int(args.ee_arrival_max_steps):
                print("warning: ee_arrival_max_steps reached", flush=True)
                break
        return

    viewer_handle = mujoco.viewer.launch_passive(model, data)
    viewer_closed = False
    draw_overlay = bool(args.overlay)
    try:
        step_i = 0
        ee_hold_v = 0
        wall_prev = time.time()
        while viewer_handle.is_running():
            q, q_des, pe = one_control_step()
            step_i += 1
            pev = int(args.print_every)
            if pev > 0 and step_i % pev == 0:
                ee, _ = fk_ee_pose_joint_rad(q)
                print(
                    f"step {step_i}  task_pos_err={pe:.5f}m  ee_xyz=[{ee[0]:.3f} {ee[1]:.3f} {ee[2]:.3f}]",
                    flush=True,
                )
            _viewer_fill_err_overlay(
                viewer_handle, q=q, q_des=q_des, draw=draw_overlay
            )
            viewer_handle.sync()
            target_period = dt * frame_skip
            now = time.time()
            spare = wall_prev + target_period - now
            if spare > 0:
                time.sleep(spare)
            wall_prev = time.time()

            if goal_xyz_wp is not None and wp_playback_len > 0 and step_i >= wp_playback_len:
                ee_v, _ = fk_ee_pose_joint_rad(q)
                if float(np.linalg.norm(ee_v - goal_xyz_wp)) < float(args.ee_arrival_tol):
                    ee_hold_v += 1
                else:
                    ee_hold_v = 0
                if ee_hold_v >= int(args.ee_arrival_hold):
                    _safe_viewer_shutdown(
                        viewer_handle, float(args.viewer_shutdown_delay)
                    )
                    viewer_closed = True
                    break
            if goal_xyz_wp is not None and step_i >= int(args.ee_arrival_max_steps):
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
