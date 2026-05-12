#!/usr/bin/env python3
"""Desired vs actual EE **위치(xyz)** 및 **자세(rpy)** 시계열을 기록해 그래프로 저장·표시.

``demo_position_path_follow.py`` 와 동일한 MJCF·YAML·궤적·종료 조건으로 시뮬한 뒤,
각 스텝에서 ``fk_ee_pose_joint_rad(q_des)``, ``fk_ee_pose_joint_rad(q)`` 의 위치·RPY를 쌓는다.
자세 오차는 각 축을 ``[-π, π]`` 로 감싼 뒤 표시한다.

실행 (``pmi_cable_mujoco`` 디렉터리에서)::

    python scripts/plot_desired_vs_ee_pose.py
    python scripts/plot_desired_vs_ee_pose.py --show
    python scripts/plot_desired_vs_ee_pose.py --output ./figures/out.png --show
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import mujoco  # noqa: E402

import demo_position_path_follow as dpf  # noqa: E402
from controllers.waypoint_trajectory import WaypointTrajectory  # noqa: E402
from kinematics.pmi_chain import fk_ee_pose_joint_rad  # noqa: E402


def _wrap_angle_vec(diff: np.ndarray) -> np.ndarray:
    """요소별 각도 차를 (-π, π] 구간으로."""
    return np.arctan2(np.sin(diff), np.cos(diff))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--model", type=Path, default=dpf.DEFAULT_MODEL, help="MJCF")
    ap.add_argument("--config", type=Path, default=dpf.DEFAULT_CONFIG, help="YAML")
    ap.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "figures" / "ee_pose_desired_vs_actual.png",
        help="저장 PNG 경로",
    )
    ap.add_argument(
        "--show",
        action="store_true",
        help="저장 후 matplotlib 창으로 바로 표시 (닫으면 스크립트 종료)",
    )
    ap.add_argument("--settle-steps", type=int, default=0)
    ap.add_argument("--servo-kp", type=float, default=None)
    ap.add_argument("--servo-kv", type=float, default=None)
    ap.add_argument("--forcerange", type=float, default=None)
    ap.add_argument(
        "--steps",
        type=int,
        default=0,
        help="비웨이포인트 최대 스텝 (0이면 8000)",
    )
    ap.add_argument("--ee-arrival-tol", type=float, default=0.025)
    ap.add_argument("--ee-arrival-hold", type=int, default=25)
    ap.add_argument("--ee-arrival-max-steps", type=int, default=120_000)
    args = ap.parse_args()

    control_cfg = dpf._load_yaml(Path(args.config))
    sim_cfg = control_cfg.get("simulation", {})
    frame_skip = int(sim_cfg.get("frame_skip", 1))

    model = mujoco.MjModel.from_xml_path(os.fspath(args.model))
    dpf._apply_position_servo_gains(
        model,
        kp=args.servo_kp,
        kv=args.servo_kv,
        forcerange=args.forcerange,
    )
    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)
    ctrl_dt = dt * frame_skip

    traj, is_wp = dpf._build_trajectory(control_cfg, ctrl_dt)
    qpos_adr, _ = dpf._joint_adr(model)

    mujoco.mj_resetData(model, data)
    if is_wp:
        q_seed = np.array([float(data.qpos[a]) for a in qpos_adr], dtype=np.float64)
        assert isinstance(traj, WaypointTrajectory)
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
        dpf._settle_on_measured_pose(model, data, qpos_adr, frame_skip, settle_n)
        if is_wp:
            assert isinstance(traj, WaypointTrajectory)
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
    if is_wp:
        assert isinstance(traj, WaypointTrajectory)
        goal_xyz_wp = traj.final_waypoint_xyz_world()
        wp_playback_len = traj.playback_num_steps()

    times: list[float] = []
    ee_des_hist: list[np.ndarray] = []
    ee_act_hist: list[np.ndarray] = []
    rpy_des_hist: list[np.ndarray] = []
    rpy_act_hist: list[np.ndarray] = []

    err_acc = 0.0
    n_err = 0

    def one_step() -> tuple[np.ndarray, np.ndarray]:
        nonlocal err_acc, n_err
        q_des, _ = traj.step()
        q_des = dpf._clip_q(q_des)
        dpf._apply_joint_target_steps(model, data, q_des, qpos_adr, frame_skip)
        q = np.array([float(data.qpos[a]) for a in qpos_adr], dtype=np.float64)
        err = q_des - q
        err_acc += float(np.dot(err, err))
        n_err += 1
        return q, q_des

    if is_wp:
        max_steps = (
            int(args.steps)
            if args.steps > 0
            else int(args.ee_arrival_max_steps)
        )
        max_steps = min(max_steps, int(args.ee_arrival_max_steps))
    else:
        max_steps = int(args.steps) if args.steps > 0 else 8000

    ee_hold = 0
    for k in range(max_steps):
        q, q_des = one_step()
        t = float((k + 1) * ctrl_dt)
        ee_des, rpy_des = fk_ee_pose_joint_rad(q_des)
        ee_act, rpy_act = fk_ee_pose_joint_rad(q)
        times.append(t)
        ee_des_hist.append(ee_des.copy())
        ee_act_hist.append(ee_act.copy())
        rpy_des_hist.append(rpy_des.copy())
        rpy_act_hist.append(rpy_act.copy())

        if goal_xyz_wp is not None and wp_playback_len > 0 and (k + 1) >= wp_playback_len:
            if float(np.linalg.norm(ee_act - goal_xyz_wp)) < float(args.ee_arrival_tol):
                ee_hold += 1
            else:
                ee_hold = 0
            if ee_hold >= int(args.ee_arrival_hold):
                break
        if goal_xyz_wp is not None and (k + 1) >= int(args.ee_arrival_max_steps):
            print(
                "warning: ee_arrival_max_steps reached without EE inside tol",
                flush=True,
            )
            break

    t_arr = np.asarray(times, dtype=np.float64)
    Ed = np.stack(ee_des_hist, axis=0)
    Ea = np.stack(ee_act_hist, axis=0)
    Rd = np.stack(rpy_des_hist, axis=0)
    Ra = np.stack(rpy_act_hist, axis=0)
    err_xyz = Ed - Ea
    err_pos_norm = np.linalg.norm(err_xyz, axis=1)
    err_rpy = _wrap_angle_vec(Rd - Ra)
    err_rpy_norm = np.linalg.norm(err_rpy, axis=1)

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)

    rpy_names = ("roll", "pitch", "yaw")
    pos_names = ("x", "y", "z")

    fig = plt.figure(figsize=(12.0, 9.5))
    fig.suptitle("EE position (m) and orientation (rad): desired vs actual (FK)")
    gs = fig.add_gridspec(4, 2, width_ratios=[1, 1], hspace=0.32, wspace=0.28)

    for i, name in enumerate(pos_names):
        ax = fig.add_subplot(gs[i, 0])
        ax.plot(t_arr, Ed[:, i], label="desired", color="C0", linewidth=1.1)
        ax.plot(t_arr, Ea[:, i], label="actual", color="C1", linewidth=1.0, alpha=0.9)
        ax.set_ylabel(f"{name} [m]")
        ax.legend(loc="upper right", fontsize=7)
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.set_title("Position")

    ax_pe = fig.add_subplot(gs[3, 0])
    ax_pe.plot(t_arr, err_pos_norm, color="C3", lw=1.1, label=r"$\|e_{xyz}\|_2$")
    ax_pe.plot(t_arr, err_xyz[:, 0], "--", alpha=0.45, label=r"$e_x$")
    ax_pe.plot(t_arr, err_xyz[:, 1], "--", alpha=0.45, label=r"$e_y$")
    ax_pe.plot(t_arr, err_xyz[:, 2], "--", alpha=0.45, label=r"$e_z$")
    ax_pe.set_xlabel("time [s]")
    ax_pe.set_ylabel("position err [m]")
    ax_pe.legend(loc="upper right", fontsize=7, ncol=2)
    ax_pe.grid(True, alpha=0.3)

    for i, name in enumerate(rpy_names):
        ax = fig.add_subplot(gs[i, 1])
        ax.plot(t_arr, Rd[:, i], label="desired", color="C0", linewidth=1.1)
        ax.plot(t_arr, Ra[:, i], label="actual", color="C1", linewidth=1.0, alpha=0.9)
        ax.set_ylabel(f"{name} [rad]")
        ax.legend(loc="upper right", fontsize=7)
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.set_title("Orientation (RPY)")

    ax_re = fig.add_subplot(gs[3, 1])
    ax_re.plot(t_arr, err_rpy_norm, color="C3", lw=1.1, label=r"$\|e_{rpy}\|_2$")
    ax_re.plot(t_arr, err_rpy[:, 0], "--", alpha=0.45, label=r"$e_{roll}$")
    ax_re.plot(t_arr, err_rpy[:, 1], "--", alpha=0.45, label=r"$e_{pitch}$")
    ax_re.plot(t_arr, err_rpy[:, 2], "--", alpha=0.45, label=r"$e_{yaw}$")
    ax_re.set_xlabel("time [s]")
    ax_re.set_ylabel("orientation err [rad]")
    ax_re.legend(loc="upper right", fontsize=7, ncol=2)
    ax_re.grid(True, alpha=0.3)

    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(
        f"saved {out}  samples={len(t_arr)}  "
        f"mean ||e_xyz||={float(err_pos_norm.mean()):.5f} m  "
        f"mean ||e_rpy||={float(err_rpy_norm.mean()):.5f} rad",
    )

    if args.show:
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    main()
