#!/usr/bin/env python3
"""토크 VSD 경로 추종(``demo_vsd_torque_path_follow.py``)과 동일 시뮬 후, 위치제어 플롯과 같은 형식으로 EE 비교.

``--settle-steps 100`` 등 데모와 **같은** MJCF·YAML·VSD·**벨트/케이블 전달(기본 on)**·종료 조건으로 headless 시뮬한 뒤,
각 제어 스텝에서 ``fk_ee_pose_joint_rad(q_des)``, ``fk_ee_pose_joint_rad(q)`` 의 xyz·RPY를 쌓아 PNG로 저장한다.

실행 (``pmi_cable_mujoco`` 에서)::

    python scripts/plot_vsd_torque_path_desired_vs_ee.py --settle-steps 100
    python scripts/plot_vsd_torque_path_desired_vs_ee.py --config configs/control_params_vsd_task.yaml --settle-steps 100 --show
    python scripts/plot_vsd_torque_path_desired_vs_ee.py --settle-steps 100 --steps 4000 --output figures/vsd_ee_compare.png
    python scripts/plot_vsd_torque_path_desired_vs_ee.py --no-transmission --settle-steps 80

전달 모델이 켜져 있으면 추가로 ``<output stem>_cable.png`` 에 jnt2–4 토크·장력·신장·z·플래그 시계열을 저장한다. 경로는 ``--output-cable`` 로 덮어쓸 수 있다.
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

import demo_vsd_torque_path_follow as vsd  # noqa: E402
from controllers.waypoint_trajectory import WaypointTrajectory  # noqa: E402
from kinematics.pmi_chain import fk_ee_pose_joint_rad  # noqa: E402


def _wrap_angle_vec(diff: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(diff), np.cos(diff))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--model", type=Path, default=vsd.DEFAULT_MODEL)
    ap.add_argument("--config", type=Path, default=vsd.DEFAULT_CONFIG)
    ap.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "figures" / "vsd_path_ee_desired_vs_actual.png",
        help="저장 PNG 경로",
    )
    ap.add_argument(
        "--output-cable",
        type=Path,
        default=None,
        help="케이블 전달 진단 PNG (기본: <output stem>_cable.png, --no-transmission 이면 생략)",
    )
    ap.add_argument("--show", action="store_true", help="저장 후 matplotlib 창 표시")
    ap.add_argument("--settle-steps", type=int, default=0)
    ap.add_argument(
        "--steps",
        type=int,
        default=0,
        help="최대 제어 스텝 (0이면 waypoints: ee_arrival_max_steps, 그 외 8000)",
    )
    ap.add_argument("--ks", type=vsd._parse_vec5, default=None)
    ap.add_argument("--kd", type=vsd._parse_vec5, default=None)
    ap.add_argument("--tau-extra", type=vsd._parse_vec4, default=np.zeros(4))
    ap.add_argument("--gravity-bias-ff", action="store_true")
    ap.add_argument("--no-gravity-bias-ff", action="store_true")
    ap.add_argument(
        "--gravity-compensation-gain",
        type=float,
        default=None,
        help="YAML ``gravity_compensation_gain`` 덮어쓰기",
    )
    ap.add_argument("--ee-arrival-tol", type=float, default=0.03)
    ap.add_argument("--ee-arrival-hold", type=int, default=30)
    ap.add_argument("--ee-arrival-max-steps", type=int, default=120_000)
    ap.add_argument(
        "--no-transmission",
        action="store_true",
        help="데모와 동일하게 벨트·케이블 Python 모델 끔",
    )
    ap.add_argument(
        "--belt-config",
        type=Path,
        default=REPO_ROOT / "configs" / "belt_params.yaml",
    )
    ap.add_argument(
        "--cable-config",
        type=Path,
        default=REPO_ROOT / "configs" / "cable_params.yaml",
    )
    ap.add_argument(
        "--transmission-randomize",
        action="store_true",
        help="belt/cable YAML randomization 유지",
    )
    ap.add_argument(
        "--waypoint-task-from",
        choices=("fk_q_des", "path_xyz"),
        default="fk_q_des",
        help="waypoints_xyz: VSD 목표 EE 위치 소스 (데모와 동일)",
    )
    args = ap.parse_args()

    control_cfg = vsd._load_yaml(Path(args.config))
    sim_cfg = control_cfg.get("simulation", {})
    frame_skip = int(sim_cfg.get("frame_skip", 1))

    Ks, Kd, gff_yaml, gcomp_yaml = vsd._default_task_vsd_gains(control_cfg)
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

    belt = cable = None
    if not args.no_transmission:
        belt, cable = vsd.build_belt_cable_models(
            Path(args.belt_config),
            Path(args.cable_config),
            randomize=bool(args.transmission_randomize),
        )

    traj, is_wp = vsd._build_trajectory(control_cfg, ctrl_dt)
    qpos_adr, dof_adr = vsd._joint_adr(model)
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
        vsd._settle_vsd_on_current_pose(
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

    if is_wp:
        max_steps = (
            int(args.steps)
            if args.steps > 0
            else int(args.ee_arrival_max_steps)
        )
        max_steps = min(max_steps, int(args.ee_arrival_max_steps))
    else:
        max_steps = int(args.steps) if args.steps > 0 else 8000

    times: list[float] = []
    ee_des_hist: list[np.ndarray] = []
    ee_act_hist: list[np.ndarray] = []
    rpy_des_hist: list[np.ndarray] = []
    rpy_act_hist: list[np.ndarray] = []

    tau_vsds: list[np.ndarray] = []
    tau_cmds: list[np.ndarray] = []
    tau_dels: list[np.ndarray] = []
    loss234: list[np.ndarray] = []
    tpc234: list[np.ndarray] = []
    tmc234: list[np.ndarray] = []
    tpo234: list[np.ndarray] = []
    tmo234: list[np.ndarray] = []
    xp234: list[np.ndarray] = []
    xpe234: list[np.ndarray] = []
    xdp234: list[np.ndarray] = []
    zp234: list[np.ndarray] = []
    dzp234: list[np.ndarray] = []
    bzp234: list[np.ndarray] = []

    ee_hold = 0
    for k in range(max_steps):
        q, q_des, _pe, _err, meta = vsd.vsd_torque_path_control_step(
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
        t = float((k + 1) * ctrl_dt)
        ee_des, rpy_des = fk_ee_pose_joint_rad(q_des)
        if (
            str(args.waypoint_task_from) == "path_xyz"
            and isinstance(traj, WaypointTrajectory)
        ):
            pwp = traj.workspace_target_xyz_at_step()
            if pwp is not None:
                ee_des = pwp.copy()
        ee_act, rpy_act = fk_ee_pose_joint_rad(q)
        times.append(t)
        ee_des_hist.append(ee_des.copy())
        ee_act_hist.append(ee_act.copy())
        rpy_des_hist.append(rpy_des.copy())
        rpy_act_hist.append(rpy_act.copy())

        tau_vsds.append(np.asarray(meta["tau_vsd"], dtype=np.float64).reshape(4))
        tau_cmds.append(np.asarray(meta["tau_joint_cmd"], dtype=np.float64).reshape(4))
        tau_dels.append(np.asarray(meta["tau_joint_delivered"], dtype=np.float64).reshape(4))
        pj = meta.get("cable_transmission", {}).get("per_joint", [])
        row_l = np.full(3, np.nan)
        row_tpc = np.full(3, np.nan)
        row_tmc = np.full(3, np.nan)
        row_tpo = np.full(3, np.nan)
        row_tmo = np.full(3, np.nan)
        row_xp = np.full(3, np.nan)
        row_xpe = np.full(3, np.nan)
        row_xdp = np.full(3, np.nan)
        row_zp = np.full(3, np.nan)
        row_dz = np.full(3, np.nan)
        row_bz = np.full(3, np.nan)
        for ji in range(min(3, len(pj))):
            dj = pj[ji]
            dp = dj.get("plus") or {}
            row_l[ji] = float(dj.get("torque_loss_joint", np.nan))
            row_tpc[ji] = float(dj.get("T_plus_cmd", np.nan))
            row_tmc[ji] = float(dj.get("T_minus_cmd", np.nan))
            row_tpo[ji] = float(dj.get("T_plus_out", np.nan))
            row_tmo[ji] = float(dj.get("T_minus_out", np.nan))
            row_xp[ji] = float(dp.get("x", np.nan))
            row_xpe[ji] = float(dp.get("x_elastic", np.nan))
            row_xdp[ji] = float(dp.get("xdot", np.nan))
            row_zp[ji] = float(dp.get("z", np.nan))
            row_dz[ji] = float(bool(dp.get("deadzone_active", False)))
            row_bz[ji] = float(bool(dp.get("backlash_active", False)))
        loss234.append(row_l)
        tpc234.append(row_tpc)
        tmc234.append(row_tmc)
        tpo234.append(row_tpo)
        tmo234.append(row_tmo)
        xp234.append(row_xp)
        xpe234.append(row_xpe)
        xdp234.append(row_xdp)
        zp234.append(row_zp)
        dzp234.append(row_dz)
        bzp234.append(row_bz)

        if goal_xyz_wp is not None and wp_playback_len > 0 and (k + 1) >= wp_playback_len:
            if float(np.linalg.norm(ee_act - goal_xyz_wp)) < float(args.ee_arrival_tol):
                ee_hold += 1
            else:
                ee_hold = 0
            if ee_hold >= int(args.ee_arrival_hold):
                break
        if goal_xyz_wp is not None and (k + 1) >= int(args.ee_arrival_max_steps):
            print(
                "warning: ee_arrival_max_steps reached without EE arrival",
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
    hy = "hybrid belt+cable" if not args.no_transmission else "rigid (no transmission model)"
    fig.suptitle(
        "VSD torque path follow — EE desired (FK q_des) vs actual (FK q), "
        f"settle_steps={settle_n}, {hy}"
    )
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
        flush=True,
    )

    fig_c = None
    if not args.no_transmission and len(t_arr) > 0:
        cable_out = args.output_cable
        if cable_out is None:
            cable_out = out.parent / f"{out.stem}_cable{out.suffix}"
        Tv = np.stack(tau_vsds, axis=0)
        Tcmd = np.stack(tau_cmds, axis=0)
        Tdel = np.stack(tau_dels, axis=0)
        Lm = np.stack(loss234, axis=0)
        Tpc = np.stack(tpc234, axis=0)
        Tmc = np.stack(tmc234, axis=0)
        Tpo = np.stack(tpo234, axis=0)
        Tmo = np.stack(tmo234, axis=0)
        Xp = np.stack(xp234, axis=0)
        Xpe = np.stack(xpe234, axis=0)
        Xdp = np.stack(xdp234, axis=0)
        Zp = np.stack(zp234, axis=0)
        Dz = np.stack(dzp234, axis=0)
        Bz = np.stack(bzp234, axis=0)

        fig_c = plt.figure(figsize=(13.5, 10.0))
        fig_c.suptitle(
            "Cable transmission (jnt2–4): cmd vs delivered, tensions, stretch, z, flags"
        )
        gsc = fig_c.add_gridspec(4, 3, hspace=0.35, wspace=0.28)
        jnames = ("jnt2", "jnt3", "jnt4")
        for j in range(3):
            ax0 = fig_c.add_subplot(gsc[0, j])
            ax0.plot(t_arr, Tcmd[:, 1 + j], label="tau_joint_cmd", lw=1.0)
            ax0.plot(t_arr, Tdel[:, 1 + j], label="tau_joint_del", lw=1.0)
            ax0.plot(t_arr, Lm[:, j], ":", label="torque_loss", lw=1.0)
            ax0.plot(t_arr, Tv[:, 1 + j], "--", alpha=0.45, label="tau_vsd", lw=0.9)
            ax0.set_title(jnames[j])
            ax0.set_ylabel("[N·m]")
            ax0.legend(loc="upper right", fontsize=6)
            ax0.grid(True, alpha=0.3)

            ax1 = fig_c.add_subplot(gsc[1, j])
            ax1.plot(t_arr, Tpc[:, j], label="T_plus_cmd", lw=0.9)
            ax1.plot(t_arr, Tmc[:, j], label="T_minus_cmd", lw=0.9)
            ax1.plot(t_arr, Tpo[:, j], "--", label="T_plus_out", lw=0.9)
            ax1.plot(t_arr, Tmo[:, j], "--", label="T_minus_out", lw=0.9)
            ax1.set_ylabel("[N]")
            ax1.legend(loc="upper right", fontsize=6)
            ax1.grid(True, alpha=0.3)

            ax2 = fig_c.add_subplot(gsc[2, j])
            ax2.plot(t_arr, Xp[:, j], label="x_plus", lw=0.9)
            ax2.plot(t_arr, Xpe[:, j], "--", label="x_elastic", lw=0.9)
            ax2.plot(t_arr, Xdp[:, j], ":", label="xdot_plus", lw=0.9)
            ax2.set_ylabel("[m] / [m/s]")
            ax2.legend(loc="upper right", fontsize=6)
            ax2.grid(True, alpha=0.3)

            ax3 = fig_c.add_subplot(gsc[3, j])
            ax3.plot(t_arr, Zp[:, j], label="z_plus", color="C0", lw=0.9)
            ax3.plot(t_arr, Dz[:, j], label="deadzone+", color="C1", lw=0.8)
            ax3.plot(t_arr, Bz[:, j], label="backlash+", color="C2", lw=0.8)
            ax3.set_xlabel("time [s]")
            ax3.set_ylabel("z / flag")
            zmax = float(np.nanmax(Zp[:, j]))
            zhi = 1.05 if not np.isfinite(zmax) else max(1.05, zmax * 1.05 + 0.01)
            ax3.set_ylim(-0.05, zhi)
            ax3.legend(loc="upper right", fontsize=6)
            ax3.grid(True, alpha=0.3)

        cable_out = Path(cable_out)
        cable_out.parent.mkdir(parents=True, exist_ok=True)
        fig_c.savefig(cable_out, dpi=150, bbox_inches="tight")
        print(f"saved cable diagnostics {cable_out}", flush=True)

    if args.show:
        plt.show()
    else:
        plt.close(fig)
        if fig_c is not None:
            plt.close(fig_c)


if __name__ == "__main__":
    main()
