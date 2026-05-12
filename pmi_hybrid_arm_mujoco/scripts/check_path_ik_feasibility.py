#!/usr/bin/env python3
from __future__ import annotations

import argparse
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
from kinematics.trajectory import CartesianQuinticPath, WaypointXYZ
from utils.mujoco_helpers import PKG_ROOT, VSD_DEBUG_MODEL_PATH, apply_ideal_qjnt_equals_ratio_qact, joint_id, load_mjmodel
from utils.path_tracking_io import joint_actuator_bounds, load_task_space_vsd_debug_yaml, ordered_transmission_arrays

OUT = PKG_ROOT / "debug_outputs" / "task_space_vsd_next"
QL = np.array([-3.1416, -1.5709, -1.5709, -1.5709], dtype=float)
QH = np.array([3.1416, 1.5709, 1.5709, 1.5709], dtype=float)
CANONICAL_WPS = [
    {"t": 0.0, "x": 0.25, "y": -0.20, "z": -0.10},
    {"t": 0.5, "x": 0.00, "y": -0.35, "z": -0.15},
    {"t": 1.0, "x": -0.25, "y": -0.20, "z": -0.10},
]


def margins(q, lo, hi):
    return np.minimum(hi - q, q - lo)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=None)
    args = ap.parse_args()

    cfg = load_task_space_vsd_debug_yaml(args.config)
    OUT.mkdir(parents=True, exist_ok=True)

    model = load_mjmodel(VSD_DEBUG_MODEL_PATH, strip_position_actuators=True)
    data = mj.MjData(model)
    scratch = mj.MjData(model)

    j_ord, a_ord, ratios = ordered_transmission_arrays(cfg)
    ratios = np.asarray(ratios, dtype=float)
    qa0 = np.asarray(cfg["path"]["initial_actuator_rad"], dtype=float)
    for i, n in enumerate(a_ord):
        data.qpos[int(model.jnt_qposadr[joint_id(model, n)])] = qa0[i]
    apply_ideal_qjnt_equals_ratio_qact(model, data, j_ord, a_ord, ratios)
    mj.mj_forward(model, data)
    q_seed = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in j_ord])

    qlj, qhj, qla, qha = joint_actuator_bounds(model, list(j_ord), list(a_ord))

    cfg["simulation"]["duration"] = 1.0
    cfg["path"]["waypoints"] = CANONICAL_WPS
    wps = [WaypointXYZ(float(w["t"]), float(w["x"]), float(w["y"]), float(w["z"])) for w in cfg["path"]["waypoints"]]
    spl = CartesianQuinticPath(wps)
    T = float(cfg["simulation"]["duration"])
    ts = np.linspace(0.0, T, 101)

    ik = IKConfig(
        weights_pos=(1.0, 1.0, 1.0),
        weight_roll=1.0,
        weight_pitch=1.0,
        damping=1e-3,
        regularization=1e-4,
        max_iterations=80,
        tolerance_ftol=1e-9,
        joint_side_order=tuple(j_ord),
    )

    rows = []
    mode_stats = {}
    for mode in ["xyz", "xyz_pitch", "xyz_roll_pitch"]:
        q_now = q_seed.copy()
        max_res = (-1.0, -1)
        first_limited = None
        for i, t in enumerate(ts):
            pd, _, _ = spl.sample(float(t))
            p_des = np.asarray(pd, dtype=float)
            q_sol, diag = solve_ik_task_mode(
                model,
                scratch,
                p_des,
                roll_des=-np.pi / 2,
                pitch_des=0.0,
                task_feas_mode=mode,
                ik=ik,
                q_seed=q_now,
                bounds_lo=QL,
                bounds_hi=QH,
            )
            q_now = q_sol.copy()
            qa = q_sol / ratios
            p_sol, *_ = fk_ee_rp(model, scratch, q_sol, list(j_ord))
            p_sol = np.asarray(p_sol)
            rnorm = float(diag["ik_norm_geom"])
            if rnorm > max_res[0]:
                max_res = (rnorm, i)

            jm = margins(q_sol, QL, QH)
            am = margins(qa, qla, qha)
            limj = int(np.argmin(jm))
            lima = int(np.argmin(am))
            active_j = j_ord[limj]
            active_a = a_ord[lima]
            if first_limited is None and (float(np.min(jm)) < 0.05 or float(np.min(am)) < 0.05):
                first_limited = (i, active_j, active_a)

            success = bool(np.isfinite(rnorm) and rnorm <= 0.02)
            row = {
                "t": float(t),
                "task_mode": mode,
                "des_x": float(p_des[0]),
                "des_y": float(p_des[1]),
                "des_z": float(p_des[2]),
                "sol_x": float(p_sol[0]),
                "sol_y": float(p_sol[1]),
                "sol_z": float(p_sol[2]),
                "residual_norm": rnorm,
                "q_jnt_1": float(q_sol[0]),
                "q_jnt_2": float(q_sol[1]),
                "q_jnt_3": float(q_sol[2]),
                "q_jnt_4": float(q_sol[3]),
                "q_act_1": float(qa[0]),
                "q_act_2": float(qa[1]),
                "q_act_3": float(qa[2]),
                "q_act_4": float(qa[3]),
                "joint_margin_1": float(jm[0]),
                "joint_margin_2": float(jm[1]),
                "joint_margin_3": float(jm[2]),
                "joint_margin_4": float(jm[3]),
                "act_margin_1": float(am[0]),
                "act_margin_2": float(am[1]),
                "act_margin_3": float(am[2]),
                "act_margin_4": float(am[3]),
                "joint_margin_min": float(np.min(jm)),
                "act_margin_min": float(np.min(am)),
                "active_limit_joint": active_j,
                "active_limit_actuator": active_a,
                "success": success,
            }
            rows.append(row)

        mode_stats[mode] = {"max_res": max_res, "first_limit": first_limited}

    csv_path = OUT / "ik_feasibility.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # plots
    for mode in ["xyz", "xyz_pitch", "xyz_roll_pitch"]:
        rr = [r for r in rows if r["task_mode"] == mode]
        t = np.array([r["t"] for r in rr])
        res = np.array([r["residual_norm"] for r in rr])
        Q = np.array([[r[f"q_jnt_{i+1}"] for i in range(4)] for r in rr])
        QA = np.array([[r[f"q_act_{i+1}"] for i in range(4)] for r in rr])
        JM = np.array([[r[f"joint_margin_{i+1}"] for i in range(4)] for r in rr])
        AM = np.array([[r[f"act_margin_{i+1}"] for i in range(4)] for r in rr])
        Pd = np.array([[r["des_x"], r["des_y"], r["des_z"]] for r in rr])
        Ps = np.array([[r["sol_x"], r["sol_y"], r["sol_z"]] for r in rr])

        plt.figure(figsize=(8, 3.5)); plt.plot(t, res); plt.grid(True, alpha=0.3); plt.xlabel("t"); plt.ylabel("res")
        plt.tight_layout(); plt.savefig(OUT / f"ik_{mode}_residual.png", dpi=140); plt.close()

        plt.figure(figsize=(8, 5))
        for i in range(4): plt.plot(t, Q[:, i], label=f"j{i+1}")
        plt.legend(); plt.grid(True, alpha=0.3); plt.tight_layout(); plt.savefig(OUT / f"ik_{mode}_qjnt.png", dpi=140); plt.close()

        plt.figure(figsize=(8, 5))
        for i in range(4): plt.plot(t, QA[:, i], label=f"a{i+1}")
        plt.legend(); plt.grid(True, alpha=0.3); plt.tight_layout(); plt.savefig(OUT / f"ik_{mode}_qact.png", dpi=140); plt.close()

        plt.figure(figsize=(8, 5))
        for i in range(4): plt.plot(t, JM[:, i], label=f"j{i+1}")
        plt.legend(); plt.grid(True, alpha=0.3); plt.tight_layout(); plt.savefig(OUT / f"ik_{mode}_joint_margin.png", dpi=140); plt.close()

        plt.figure(figsize=(8, 5))
        for i in range(4): plt.plot(t, AM[:, i], label=f"a{i+1}")
        plt.legend(); plt.grid(True, alpha=0.3); plt.tight_layout(); plt.savefig(OUT / f"ik_{mode}_act_margin.png", dpi=140); plt.close()

        fig, ax = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
        for i, n in enumerate(["x", "y", "z"]):
            ax[i].plot(t, Pd[:, i], label="des"); ax[i].plot(t, Ps[:, i], "--", label="ik"); ax[i].set_ylabel(n); ax[i].grid(True, alpha=0.3); ax[i].legend()
        ax[-1].set_xlabel("t")
        plt.tight_layout(); plt.savefig(OUT / f"ik_{mode}_xyz_compare.png", dpi=140); plt.close()

    # report
    mid_idx = 50
    rep = ["# IK Feasibility Report", ""]
    for mode in ["xyz", "xyz_pitch", "xyz_roll_pitch"]:
        rr = [r for r in rows if r["task_mode"] == mode]
        worst = max(rr, key=lambda x: x["residual_norm"])
        rep.append(f"## {mode}")
        rep.append(f"- middle waypoint feasible? {rr[mid_idx]['success']} (residual={rr[mid_idx]['residual_norm']:.6f})")
        rep.append(f"- largest residual sample: t={worst['t']:.6f}, residual={worst['residual_norm']:.6f}")
        rep.append(f"- first active tight joint/actuator: {mode_stats[mode]['first_limit']}")
        rep.append(f"- feasible for mode? {all(r['success'] for r in rr)}")
        rep.append("")

    (OUT / "ik_feasibility_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
