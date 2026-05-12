#!/usr/bin/env python3
"""MuJoCo ``position`` 액추에이터로 관절 목표 추종(경로 재생) 시각화.

``models/pmi_cable_arm_position.xml`` 은 ``position`` 서보를 ``q1_act``…``q4_act`` 에 걸고,
스크립트에서 ``ctrl = clip( q_jnt_des / gear )`` 로 채운다 (등식 ``q_jnt = gear * q_act`` 와 동일).
데모 MJCF는 캡슐 자기간섭을 피하려 ``contact`` 를 끈다.

기준 경로는 ``configs/control_params*.yaml`` 의 ``trajectory`` (``waypoints_xyz`` 또는 ``sine``/``step_hold``)와
``envs/pmi_cable_arm_env`` 와 동일한 초기화 흐름을 따릅니다.

시작 직후 팔이 크게 흔들리거나 **발산하는 것처럼** 보일 때는 보통 아래가 겹친 현상입니다.

1. **목표 급변**: 플래너 첫 샘플용 IK 해가 초기 관절각과 다르면 첫 ``ctrl`` 에서 관절 목표가 한 번에 많이 바뀝니다.
2. **서보 게인**: ``position`` 의 ``kp`` 가 크면 작은 오차에도 토크가 세게 걸려 첫 구간에서 과슈팅·출렁임이 큽니다.
3. **중력**: 리셋 직후 한두 스텝은 관성·중력과 등식이 맞물리며 움직이다가 서보가 따라잡습니다.
4. **오버레이**: 적색 EE 오차 선 길이가 순간 크게 보이면 실제 관성보다 더 심하게 “튀는” 느낌을 줄 수 있습니다.

완화: ``--settle-steps`` 로 경로 재생 **전에** 측정 관절각을 목표로 짧게 안정화합니다. 필요하면 ``--servo-kp`` 를 낮추거나 ``simulation.frame_skip`` 을 키우세요.

``waypoints_xyz`` 모드에서는 관절 궤적을 끝까지 낸 뒤(마지막 샘플 유지), **실제 EE**(측정 관절 ``q`` FK)가 **마지막 웨이포인트** (``t`` 최대)에 ``--ee-arrival-tol`` 이내로 ``--ee-arrival-hold`` 스텝 유지되면 종료합니다. 관절 샘플 개수만으로 끊지 않습니다.

원격 X11·일부 드라이버에서 뷰어 자동 종료 직후 ``GLXBadDrawable`` 이 찍히면 ``--viewer-shutdown-delay 0.15`` 처럼 늘리거나, ``MUJOCO_GL=egl`` 을 시도해 보세요.

뷰어 오버레이 ( ``--no-overlay`` 로 끔):

- **녹색 선**: 웨이포인트 모드에서 플래너 작업공간 경로 (xyz 폴리라인).
- **노란 구**: YAML에 적은 사용자 웨이포인트 위치.
- **적색 선**: 목표 관절각에 대한 FK EE 위치와 현재 FK EE 위치를 잇는 선(카테시안 추종 오차 방향).

실행 예::

    cd pmi_cable_mujoco
    python scripts/demo_position_path_follow.py
    python scripts/demo_position_path_follow.py --config configs/control_params.yaml
    python scripts/demo_position_path_follow.py --headless --steps 2000
    python scripts/demo_position_path_follow.py --settle-steps 80

**추종 오차 줄이기 (요약)**

1. **MJCF 서보** — ``pmi_cable_arm_position.xml`` 의 ``position`` 액추에이터 ``kp``/``kv`` 를 올리면 강성↑ (너무 크면 진동).
   ``forcerange`` 가 부족하면 포화로 정상상태 오차↑ → 한계 상향.
2. **CLI 오버라이드** — 아래 ``--servo-kp`` / ``--servo-kv`` / ``--forcerange`` 로 로드 직후 게인·토크 한계 조정 (4축 동일).
3. **YAML ``simulation.frame_skip``** — 값을 키우면 같은 궤적을 **시뮬 시간상 더 천천히** 재생해 스텝당 목표 변화가 작아져 추종이 쉬워짐 (예: 2~5).
4. **``trajectory.planner_dt``** — 웨이포인트 모드에서 플래너 샘플 간격을 ``ctrl_dt``(= timestep×frame_skip) 와 맞추거나 더 촘촘히 하면 보간·IK 단계 오차 감소.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, Tuple, Union

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import mujoco
import mujoco.viewer

from controllers.vsd_controller import ReferenceTrajectory
from controllers.waypoint_trajectory import WaypointTrajectory
from kinematics.pmi_chain import (
    ACTUATOR_LIMIT_RAD_MAX,
    ACTUATOR_LIMIT_RAD_MIN,
    JOINT_LIMIT_RAD_MAX,
    JOINT_LIMIT_RAD_MIN,
    actuator_rad_from_joint_rad,
    fk_ee_pose_joint_rad,
)

PROJECT_ROOT = Path(ROOT)
DEFAULT_MODEL = PROJECT_ROOT / "models" / "pmi_cable_arm_position.xml"
DEFAULT_CONFIG = PROJECT_ROOT / "configs" / "control_params_waypoints.yaml"
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


def _clip_q_act(q_act: np.ndarray) -> np.ndarray:
    return np.clip(
        np.asarray(q_act, dtype=np.float64).reshape(4),
        ACTUATOR_LIMIT_RAD_MIN,
        ACTUATOR_LIMIT_RAD_MAX,
    )


def _apply_position_servo_gains(
    model: mujoco.MjModel,
    *,
    kp: float | None,
    kv: float | None,
    forcerange: float | None,
) -> None:
    """MuJoCo ``position`` 액추에이터: gainprm[:,0]=kp, biasprm[:,1]=-kp, biasprm[:,2]=-kv."""
    nu = model.nu
    if kp is not None:
        kpf = float(kp)
        for i in range(nu):
            model.actuator_gainprm[i, 0] = kpf
            model.actuator_biasprm[i, 1] = -kpf
    if kv is not None:
        kvf = float(kv)
        for i in range(nu):
            model.actuator_biasprm[i, 2] = -kvf
    if forcerange is not None:
        lim = abs(float(forcerange))
        model.actuator_forcerange[:, 0] = -lim
        model.actuator_forcerange[:, 1] = lim


def _apply_joint_target_steps(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    q_joint_target: np.ndarray,
    qpos_adr: list[int],
    frame_skip: int,
) -> None:
    q_act_cmd = _clip_q_act(
        actuator_rad_from_joint_rad(_clip_q(np.asarray(q_joint_target)))
    )
    data.ctrl[:] = q_act_cmd
    for _ in range(frame_skip):
        mujoco.mj_step(model, data)


def _settle_on_measured_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    qpos_adr: list[int],
    frame_skip: int,
    settle_steps: int,
) -> None:
    """재생 전에 ``ctrl`` 을 매번 현재 관절각에 맞춰 추종 → 중력·등식 완화 후 시작."""
    for _ in range(settle_steps):
        q = np.array([float(data.qpos[a]) for a in qpos_adr], dtype=np.float64)
        _apply_joint_target_steps(model, data, q, qpos_adr, frame_skip)


def _safe_viewer_shutdown(handle: mujoco.viewer.Handle, delay_s: float) -> None:
    """렌더 스레드와 경합하지 않도록 잠깐 쉰 뒤 ``lock`` 안에서 ``close``."""
    if delay_s > 0:
        time.sleep(float(delay_s))
    try:
        with handle.lock():
            handle.close()
    except mujoco.UnexpectedError:
        pass


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
    """``launch_passive`` 의 ``user_scn`` 에 경로·웨이포인트·EE 오차 선을 그린다."""
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


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL,
        help="MJCF (기본: position 액추에이터 모델)",
    )
    ap.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help="control_params YAML",
    )
    ap.add_argument("--headless", action="store_true", help="뷰어 없이 시뮬만")
    ap.add_argument("--steps", type=int, default=0, help="headless 시 최대 mj_step 수 (0=무한)")
    ap.add_argument(
        "--print-every",
        type=int,
        default=250,
        help="관절 오차 RMS를 출력할 간격 [sim steps]",
    )
    ap.add_argument(
        "--servo-kp",
        type=float,
        default=None,
        help="position 액추에이터 Kp (4축 동일). 미지정 시 MJCF 값 유지",
    )
    ap.add_argument(
        "--servo-kv",
        type=float,
        default=None,
        help="position 액추에이터 Kd/Kv (4축 동일). 미지정 시 MJCF 값 유지",
    )
    ap.add_argument(
        "--forcerange",
        type=float,
        default=None,
        help="액추에이터 토크 한계 ±값 [N·m]. 미지정 시 MJCF 값 유지",
    )
    ap.add_argument(
        "--no-overlay",
        action="store_true",
        help="뷰어의 녹색 경로/노란 웨이포인트/적색 EE 오차 선 그리지 않음",
    )
    ap.add_argument(
        "--settle-steps",
        type=int,
        default=0,
        help="경로 재생 전 관측 관절을 목표로 한 안정화 제어 반복 횟수 (큰 값일수록 시작 튐 완화)",
    )
    ap.add_argument(
        "--viewer-shutdown-delay",
        type=float,
        default=0.08,
        help="뷰어 종료 직전 대기 [s]. GLXBadDrawable 완화(원격 X 등)용. 0 이면 생략",
    )
    ap.add_argument(
        "--ee-arrival-tol",
        type=float,
        default=0.025,
        help="waypoints_xyz: 마지막 웨이포인트까지 실제 EE 거리 [m] 이 값 이하",
    )
    ap.add_argument(
        "--ee-arrival-hold",
        type=int,
        default=25,
        help="waypoints_xyz: 위 조건을 연속 이 만큼 제어 스텝 만족하면 종료",
    )
    ap.add_argument(
        "--ee-arrival-max-steps",
        type=int,
        default=120_000,
        help="waypoints_xyz: EE가 끝내 안 들어올 때 상한 제어 스텝(안전)",
    )
    args = ap.parse_args()

    control_cfg = _load_yaml(Path(args.config))
    sim_cfg = control_cfg.get("simulation", {})
    frame_skip = int(sim_cfg.get("frame_skip", 1))

    model = mujoco.MjModel.from_xml_path(os.fspath(args.model))
    _apply_position_servo_gains(
        model,
        kp=args.servo_kp,
        kv=args.servo_kv,
        forcerange=args.forcerange,
    )
    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)
    ctrl_dt = dt * frame_skip

    traj, is_wp = _build_trajectory(control_cfg, ctrl_dt)
    qpos_adr, _ = _joint_adr(model)

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

    settle_n = max(0, int(args.settle_steps))
    if settle_n > 0:
        _settle_on_measured_pose(model, data, qpos_adr, frame_skip, settle_n)
        if is_wp and isinstance(traj, WaypointTrajectory):
            q_now = np.array([float(data.qpos[a]) for a in qpos_adr], dtype=np.float64)
            traj.on_reset(
                q_now,
                mujoco_data=data,
                joint_qpos_adr=qpos_adr,
                apply_yaml_initial_pose=False,
            )
            mujoco.mj_forward(model, data)

    err_acc = 0.0
    n_err = 0

    def one_control_step() -> Tuple[np.ndarray, np.ndarray]:
        nonlocal err_acc, n_err
        q_des, _ = traj.step()
        q_des = _clip_q(q_des)
        _apply_joint_target_steps(model, data, q_des, qpos_adr, frame_skip)
        q = np.array([float(data.qpos[a]) for a in qpos_adr], dtype=np.float64)
        err = q_des - q
        err_acc += float(np.dot(err, err))
        n_err += 1
        return q, q_des

    goal_xyz_wp: np.ndarray | None = None
    wp_playback_len = 0
    if is_wp and isinstance(traj, WaypointTrajectory):
        goal_xyz_wp = traj.final_waypoint_xyz_world()
        wp_playback_len = traj.playback_num_steps()

    if args.headless:
        if is_wp and isinstance(traj, WaypointTrajectory):
            max_steps = (
                int(args.steps)
                if args.steps > 0
                else int(args.ee_arrival_max_steps)
            )
            max_steps = min(max_steps, int(args.ee_arrival_max_steps))
        else:
            max_steps = int(args.steps) if args.steps > 0 else 10_000
        ee_hold_h = 0
        for k in range(max_steps):
            q, q_des = one_control_step()
            pe = args.print_every
            if pe > 0 and (k + 1) % pe == 0:
                rms = np.sqrt(np.mean((q_des - q) ** 2))
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
                    f"step {k+1}  joint_rms_err_rad={rms:.5f}  ee_xyz={ee}{extra}"
                )
            if goal_xyz_wp is not None and wp_playback_len > 0 and (k + 1) >= wp_playback_len:
                ee, _ = fk_ee_pose_joint_rad(q)
                if float(np.linalg.norm(ee - goal_xyz_wp)) < float(args.ee_arrival_tol):
                    ee_hold_h += 1
                else:
                    ee_hold_h = 0
                if ee_hold_h >= int(args.ee_arrival_hold):
                    break
            if goal_xyz_wp is not None and (k + 1) >= int(args.ee_arrival_max_steps):
                print(
                    "warning: ee_arrival_max_steps reached without EE inside tol",
                    flush=True,
                )
                break

        if n_err > 0:
            print(
                "mean joint RMSE [rad]",
                float(np.sqrt(err_acc / (4 * n_err))),
            )
        return

    # ``with viewer:`` 는 블록 종료 시 ``close()`` 를 또 부를 수 있어 이중 종료 위험이 있다.
    # 종료 시 렌더 스레드의 GLX swap 과 맞물리면 ``GLXBadDrawable`` 이 나올 수 있어
    # ``--viewer-shutdown-delay`` + ``lock()`` 안에서 한 번만 닫는다.
    viewer_handle = mujoco.viewer.launch_passive(model, data)
    viewer_closed = False
    try:
        step_i = 0
        ee_hold_v = 0
        wall_prev = time.time()
        draw_overlay = not args.no_overlay
        while viewer_handle.is_running():
            q, q_des = one_control_step()
            step_i += 1

            pe = args.print_every
            if pe > 0 and step_i % pe == 0:
                rms = np.sqrt(np.mean((q_des - q) ** 2))
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
                    f"step {step_i}  joint_rms_err_rad={rms:.5f}  ee_xyz=[{ee[0]:.3f} {ee[1]:.3f} {ee[2]:.3f}]{extra}"
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

            # 대략 실시간에 가깝게 (시뮬 dt * frame_skip 초당 한 번 제어)
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
