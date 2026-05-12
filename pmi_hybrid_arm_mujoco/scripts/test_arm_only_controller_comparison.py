#!/usr/bin/env python3
from __future__ import annotations

import copy
import csv
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np

from kinematics.forward_kinematics import fk_ee_rp
from kinematics.inverse_kinematics import IKConfig, solve_ik_task_mode
from kinematics.task_jacobian import compute_task_jacobian_mode
from kinematics.trajectory import CartesianQuinticPath, WaypointXYZ
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel
from utils.path_tracking_io import load_task_space_vsd_debug_yaml

MODEL_PATH = PKG_ROOT / "models" / "pmi_arm_only_torque_debug.xml"
OUT = PKG_ROOT / "debug_outputs" / "arm_only_debug"
WPS = [
    {"t": 0.0, "x": 0.25, "y": -0.20, "z": -0.10},
    {"t": 0.5, "x": 0.00, "y": -0.35, "z": -0.15},
    {"t": 1.0, "x": -0.25, "y": -0.20, "z": -0.10},
]
RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=float)
Q_ACT0 = np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=float)
QJ_INIT = RATIOS * Q_ACT0
J_ORD = ["jnt1", "jnt2", "jnt3", "jnt4"]


def _save(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(path, dpi=140)
    plt.close()


def run_controller(kind: str, tau_lim: float = 20.0) -> tuple[dict, dict]:
    cfg = load_task_space_vsd_debug_yaml(None)
    dt = float(cfg["simulation"]["dt"])
    dur = 1.0
    n = int(round(dur / dt)) + 1
    ts = np.array([min(i * dt, dur) for i in range(n)], dtype=float)

    model = load_mjmodel(MODEL_PATH, strip_position_actuators=True)
    model.opt.timestep = dt
    data = mj.MjData(model)
    scratch = mj.MjData(model)
    jac_s = mj.MjData(model)

    # initial state
    for i, jn in enumerate(J_ORD):
        data.qpos[int(model.jnt_qposadr[joint_id(model, jn)])] = QJ_INIT[i]
    data.qvel[:] = 0.0
    mj.mj_forward(model, data)

    dof = np.array([int(model.jnt_dofadr[joint_id(model, n0)]) for n0 in J_ORD], dtype=int)
    q_lo = np.array([float(model.jnt_range[joint_id(model, n0), 0]) for n0 in J_ORD])
    q_hi = np.array([float(model.jnt_range[joint_id(model, n0), 1]) for n0 in J_ORD])

    spline = CartesianQuinticPath([WaypointXYZ(float(w["t"]), float(w["x"]), float(w["y"]), float(w["z"])) for w in WPS])

    # IK precompute for IK controller
    q_des = np.zeros((n, 4), dtype=float)
    qdot_des = np.zeros((n, 4), dtype=float)
    if kind == "ik_joint_space_vsd":
        ik = IKConfig((1, 1, 1), 1.0, 1.0, 1e-3, 1e-4, 80, 1e-9, tuple(J_ORD))
        q_seed = QJ_INIT.copy()
        for k, t in enumerate(ts):
            p, _, _ = spline.sample(float(t))
            q_sol, _ = solve_ik_task_mode(
                model, scratch, np.asarray(p), roll_des=-np.pi/2, pitch_des=0.0,
                task_feas_mode="xyz", ik=ik, q_seed=q_seed, bounds_lo=q_lo, bounds_hi=q_hi,
            )
            q_des[k] = q_sol
            q_seed = q_sol.copy()
        qdot_des[1:-1] = (q_des[2:] - q_des[:-2]) / (2.0 * dt)
        qdot_des[0] = (q_des[1] - q_des[0]) / dt
        qdot_des[-1] = (q_des[-1] - q_des[-2]) / dt

    # logs
    p_des_log = np.zeros((n, 3))
    p_act_log = np.zeros((n, 3))
    q_log = np.zeros((n, 4))
    tau_log = np.zeros((n, 4))
    err = np.zeros(n)
    sat_steps = 0
    jl_steps = 0
    max_tau_unc = 0.0

    q_cmd = QJ_INIT.copy()
    Kx = np.array([3.0, 3.0, 3.0])
    Dq_dls = np.array([8.0, 8.0, 8.0, 4.0])
    lam = 0.08

    for k, t in enumerate(ts):
        q = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])]) for n0 in J_ORD])
        qdot = np.array([float(data.qvel[int(model.jnt_dofadr[joint_id(model, n0)])]) for n0 in J_ORD])
        p_des, pdot_des, _ = spline.sample(float(t))
        p_des = np.asarray(p_des)
        pdot_des = np.asarray(pdot_des)
        p_act, *_ = fk_ee_rp(model, scratch, q, J_ORD)
        p_act = np.asarray(p_act)

        mj.mj_forward(model, data)
        tau_bias = np.array([float(data.qfrc_bias[d]) for d in dof])

        if kind == "pure_jtf_task_vsd":
            jac_s.qpos[:] = data.qpos
            jac_s.qvel[:] = data.qvel
            mj.mj_forward(model, jac_s)
            J = compute_task_jacobian_mode(model, jac_s, joint_names=J_ORD, task_mode="xyz", ee_site_name="end_effector", mode="numerical", epsilon=1e-6)
            e = p_des - p_act
            ydot = J @ qdot
            F = np.array([300.0, 300.0, 300.0]) * e + np.array([8.0, 8.0, 8.0]) * (pdot_des - ydot)
            tau_unc = tau_bias + J.T @ F
        elif kind == "ik_joint_space_vsd":
            e = q_des[k] - q
            ed = qdot_des[k] - qdot
            tau_unc = tau_bias + np.array([300.0, 300.0, 300.0, 40.0]) * e + np.array([8.0, 8.0, 8.0, 4.0]) * ed
        else:
            jac_s.qpos[:] = data.qpos
            jac_s.qvel[:] = data.qvel
            mj.mj_forward(model, jac_s)
            J = compute_task_jacobian_mode(model, jac_s, joint_names=J_ORD, task_mode="xyz", ee_site_name="end_effector", mode="numerical", epsilon=1e-6)
            xdot_cmd = pdot_des + Kx * (p_des - p_act)
            J_dls = J.T @ np.linalg.inv(J @ J.T + (lam ** 2) * np.eye(3))
            qdot_cmd = J_dls @ xdot_cmd
            q_cmd = np.clip(q_cmd + qdot_cmd * dt, q_lo, q_hi)
            tau_unc = tau_bias + Dq_dls * (qdot_cmd - qdot) + np.array([20.0, 20.0, 20.0, 5.0]) * (q_cmd - q)

        tau = np.clip(tau_unc, -tau_lim, tau_lim)
        sat_steps += int(np.any(np.abs(tau_unc - tau) > 1e-9))
        max_tau_unc = max(max_tau_unc, float(np.max(np.abs(tau_unc))))

        data.qfrc_applied[:] = 0.0
        for i in range(4):
            data.qfrc_applied[dof[i]] = tau[i]
        mj.mj_step(model, data)

        q_after = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])]) for n0 in J_ORD])
        p_after, *_ = fk_ee_rp(model, scratch, q_after, J_ORD)
        p_after = np.asarray(p_after)

        p_des_log[k] = p_des
        p_act_log[k] = p_after
        q_log[k] = q_after
        tau_log[k] = tau
        err[k] = float(np.linalg.norm(p_des - p_after))

        jm = np.minimum(q_hi - q_after, q_after - q_lo)
        jl_steps += int(np.min(jm) < 0.0)

    summary = {
        "controller_name": kind,
        "rms_pos": float(np.sqrt(np.mean(err ** 2))),
        "max_pos": float(np.max(err)),
        "final_pos_err": float(err[-1]),
        "max_tau_before_clip": max_tau_unc,
        "max_tau_after_clip": float(np.max(np.abs(tau_log))),
        "torque_saturation_steps": int(sat_steps),
        "joint_limit_steps": int(jl_steps),
        "stable": bool(np.isfinite(np.max(err))),
    }
    logs = {"t": ts, "p_des": p_des_log, "p_act": p_act_log, "q": q_log, "tau": tau_log}
    return summary, logs


def plot_controller(name: str, logs: dict) -> None:
    t = logs["t"]
    Pd = logs["p_des"]
    Pa = logs["p_act"]

    fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    for i, lb in enumerate(["x", "y", "z"]):
        ax[i].plot(t, Pd[:, i], label="des")
        ax[i].plot(t, Pa[:, i], "--", label="act")
        ax[i].set_ylabel(lb)
        ax[i].legend()
        ax[i].grid(True, alpha=0.3)
    _save(OUT / f"{name}_ee_xyz.png")

    fig = plt.figure(figsize=(5.5, 5))
    try:
        ax3 = fig.add_subplot(111, projection="3d")
        ax3.plot(Pa[:, 0], Pa[:, 1], Pa[:, 2], label="act")
        ax3.plot(Pd[:, 0], Pd[:, 1], Pd[:, 2], "--", label="des")
        ax3.legend()
    except Exception:
        plt.close(fig)
        fig, ax2 = plt.subplots(figsize=(5.5, 5))
        ax2.plot(Pa[:, 0], Pa[:, 1], label="act")
        ax2.plot(Pd[:, 0], Pd[:, 1], "--", label="des")
        ax2.legend()
        ax2.grid(True, alpha=0.3)
    _save(OUT / f"{name}_path3d.png")

    Q = logs["q"]
    fig, axs = plt.subplots(4, 1, figsize=(8, 7), sharex=True)
    for i in range(4):
        axs[i].plot(t, Q[:, i])
        axs[i].grid(True, alpha=0.3)
    _save(OUT / f"{name}_q_jnt.png")

    T = logs["tau"]
    fig, axs = plt.subplots(4, 1, figsize=(8, 7), sharex=True)
    for i in range(4):
        axs[i].plot(t, T[:, i])
        axs[i].grid(True, alpha=0.3)
    _save(OUT / f"{name}_tau_jnt.png")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    for name in ["pure_jtf_task_vsd", "ik_joint_space_vsd", "dls_resolved_rate_joint_torque"]:
        s, logs = run_controller(name, tau_lim=20.0)
        rows.append(s)
        plot_controller(name, logs)

    with open(OUT / "controller_comparison.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    best = min(rows, key=lambda r: r["rms_pos"])
    rep = ["# Arm-only Controller Comparison", ""]
    for r in rows:
        rep.append("- {controller_name}: rms={rms_pos:.6f}, max={max_pos:.6f}, final={final_pos_err:.6f}, sat={torque_saturation_steps}".format(**r))
    rep += ["", f"best by rms: {best['controller_name']}"]
    (OUT / "controller_comparison_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
