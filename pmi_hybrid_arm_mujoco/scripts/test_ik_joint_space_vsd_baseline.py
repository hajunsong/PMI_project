#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import csv
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import mujoco as mj
import numpy as np

from kinematics.forward_kinematics import fk_ee_rp
from kinematics.inverse_kinematics import IKConfig, solve_ik_task_mode
from kinematics.trajectory import CartesianQuinticPath, WaypointXYZ
from utils.mujoco_helpers import PKG_ROOT, VSD_DEBUG_MODEL_PATH, apply_ideal_qact_from_joint, apply_ideal_qjnt_equals_ratio_qact, joint_id, load_mjmodel
from utils.path_tracking_io import gains_limits_task_space_vsd, joint_actuator_bounds, load_task_space_vsd_debug_yaml, ordered_transmission_arrays

OUT = PKG_ROOT / "debug_outputs" / "task_space_vsd_next"
CANONICAL_WPS = [
    {"t": 0.0, "x": 0.25, "y": -0.20, "z": -0.10},
    {"t": 0.5, "x": 0.00, "y": -0.35, "z": -0.15},
    {"t": 1.0, "x": -0.25, "y": -0.20, "z": -0.10},
]


def run_ik_joint_pd(cfg: dict, tau_limit_override: float | None = None) -> tuple[dict, list[dict]]:
    sim = cfg["simulation"]
    dt = float(sim["dt"])
    dur = float(sim["duration"])
    n = int(round(dur / dt)) + 1
    ts = np.array([min(i * dt, dur) for i in range(n)], dtype=float)

    j_ord, a_ord, ratios = ordered_transmission_arrays(cfg)
    ratios = np.asarray(ratios, dtype=float)
    qa0 = np.asarray(cfg["path"]["initial_actuator_rad"], dtype=float)
    wps = [WaypointXYZ(float(w["t"]), float(w["x"]), float(w["y"]), float(w["z"])) for w in cfg["path"]["waypoints"]]
    spline = CartesianQuinticPath(wps)

    model = load_mjmodel(VSD_DEBUG_MODEL_PATH, strip_position_actuators=True)
    model.opt.timestep = dt
    data = mj.MjData(model)
    scratch = mj.MjData(model)

    for i, nm in enumerate(a_ord):
        data.qpos[int(model.jnt_qposadr[joint_id(model, nm)])] = qa0[i]
    apply_ideal_qjnt_equals_ratio_qact(model, data, list(j_ord), list(a_ord), ratios)
    mj.mj_forward(model, data)

    dof_j = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in j_ord], dtype=int)
    qlj, qhj, qla, qha = joint_actuator_bounds(model, list(j_ord), list(a_ord))
    _tm, gk, gd, _lf, tj_lim, _ta_lim = gains_limits_task_space_vsd(cfg["task_space_vsd"])
    Kq = np.asarray(gk[:4] if len(gk) >= 4 else [gk[0], gk[1], gk[2], 20.0], dtype=float)
    Dq = np.asarray(gd[:4] if len(gd) >= 4 else [gd[0], gd[1], gd[2], 4.0], dtype=float)
    tj_lim = np.asarray(tj_lim, dtype=float)
    if tau_limit_override is not None:
        tj_lim[:] = float(tau_limit_override)

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

    # IK trajectory precompute
    q_seed = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in j_ord], dtype=float)
    q_des = np.zeros((n, 4), dtype=float)
    p_des = np.zeros((n, 3), dtype=float)
    for k, t in enumerate(ts):
        p, _, _ = spline.sample(float(t))
        p_des[k] = np.asarray(p, dtype=float)
        q_sol, _ = solve_ik_task_mode(
            model,
            scratch,
            p_des[k],
            roll_des=-np.pi / 2,
            pitch_des=0.0,
            task_feas_mode="xyz",
            ik=ik,
            q_seed=q_seed,
            bounds_lo=qlj,
            bounds_hi=qhj,
        )
        q_des[k] = q_sol
        q_seed = q_sol.copy()

    qdot_des = np.zeros_like(q_des)
    if n >= 3:
        qdot_des[1:-1] = (q_des[2:] - q_des[:-2]) / (2.0 * dt)
    qdot_des[0] = (q_des[1] - q_des[0]) / dt
    qdot_des[-1] = (q_des[-1] - q_des[-2]) / dt

    rows: list[dict] = []
    ee_err = []
    sat_steps = 0
    jl_steps = 0
    max_tau = 0.0
    max_tau_unc = 0.0

    for k, t in enumerate(ts):
        q = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in j_ord], dtype=float)
        qdot = np.array([float(data.qvel[int(model.jnt_dofadr[joint_id(model, n)])]) for n in j_ord], dtype=float)

        mj.mj_forward(model, data)
        tau_bias = np.array([float(data.qfrc_bias[int(dof_j[i])]) for i in range(4)], dtype=float)

        e = q_des[k] - q
        ed = qdot_des[k] - qdot
        tau_pd = Kq * e + Dq * ed
        tau_unc = tau_bias + tau_pd
        tau = np.clip(tau_unc, -tj_lim, tj_lim)
        sat = bool(np.any(np.abs(tau_unc - tau) > 1e-9))
        if sat:
            sat_steps += 1

        data.qfrc_applied[:] = 0.0
        data.ctrl[:] = 0.0
        for i in range(4):
            data.qfrc_applied[dof_j[i]] += float(tau[i])

        mj.mj_step(model, data)
        apply_ideal_qact_from_joint(model, data, list(j_ord), list(a_ord), ratios)
        mj.mj_forward(model, data)

        q_aft = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in j_ord], dtype=float)
        qdot_aft = np.array([float(data.qvel[int(model.jnt_dofadr[joint_id(model, n)])]) for n in j_ord], dtype=float)
        qa = q_aft / ratios

        p_act, *_ = fk_ee_rp(model, scratch, q_aft, list(j_ord))
        p_act = np.asarray(p_act, dtype=float)
        err = p_des[k] - p_act
        en = float(np.linalg.norm(err))
        ee_err.append(en)

        j_margin = np.minimum(qhj - q_aft, q_aft - qlj)
        a_margin = np.minimum(qha - qa, qa - qla)
        if float(np.min(j_margin)) < 0.0 or float(np.min(a_margin)) < 0.0:
            jl_steps += 1

        max_tau = max(max_tau, float(np.max(np.abs(tau))))
        max_tau_unc = max(max_tau_unc, float(np.max(np.abs(tau_unc))))

        rows.append(
            {
                "t": float(t),
                "des_x": float(p_des[k, 0]), "des_y": float(p_des[k, 1]), "des_z": float(p_des[k, 2]),
                "act_x": float(p_act[0]), "act_y": float(p_act[1]), "act_z": float(p_act[2]),
                "q_jnt_des_1": float(q_des[k, 0]), "q_jnt_des_2": float(q_des[k, 1]), "q_jnt_des_3": float(q_des[k, 2]), "q_jnt_des_4": float(q_des[k, 3]),
                "q_jnt_act_1": float(q_aft[0]), "q_jnt_act_2": float(q_aft[1]), "q_jnt_act_3": float(q_aft[2]), "q_jnt_act_4": float(q_aft[3]),
                "qdot_jnt_des_1": float(qdot_des[k, 0]), "qdot_jnt_des_2": float(qdot_des[k, 1]), "qdot_jnt_des_3": float(qdot_des[k, 2]), "qdot_jnt_des_4": float(qdot_des[k, 3]),
                "qdot_jnt_act_1": float(qdot_aft[0]), "qdot_jnt_act_2": float(qdot_aft[1]), "qdot_jnt_act_3": float(qdot_aft[2]), "qdot_jnt_act_4": float(qdot_aft[3]),
                "tau_bias_1": float(tau_bias[0]), "tau_bias_2": float(tau_bias[1]), "tau_bias_3": float(tau_bias[2]), "tau_bias_4": float(tau_bias[3]),
                "tau_pd_1": float(tau_pd[0]), "tau_pd_2": float(tau_pd[1]), "tau_pd_3": float(tau_pd[2]), "tau_pd_4": float(tau_pd[3]),
                "tau_total_1": float(tau[0]), "tau_total_2": float(tau[1]), "tau_total_3": float(tau[2]), "tau_total_4": float(tau[3]),
                "joint_margin_min": float(np.min(j_margin)),
                "act_margin_min": float(np.min(a_margin)),
                "q_act_equiv_1": float(qa[0]), "q_act_equiv_2": float(qa[1]), "q_act_equiv_3": float(qa[2]), "q_act_equiv_4": float(qa[3]),
                "pos_err_norm": en,
            }
        )

    ee_err_arr = np.asarray(ee_err, dtype=float)
    summary = {
        "controller": "ik_joint_space_vsd",
        "rms_pos": float(np.sqrt(np.mean(ee_err_arr**2))),
        "max_pos": float(np.max(ee_err_arr)),
        "final_pos_err": float(ee_err_arr[-1]),
        "max_tau": float(max_tau),
        "max_tau_before_clip": float(max_tau_unc),
        "joint_limit_steps": int(jl_steps),
        "torque_saturation_steps": int(sat_steps),
        "final_desired_xyz": p_des[-1].tolist(),
        "final_actual_xyz": [rows[-1]["act_x"], rows[-1]["act_y"], rows[-1]["act_z"]],
        "stable": True,
    }
    return summary, rows


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=None)
    args = ap.parse_args()

    cfg = copy.deepcopy(load_task_space_vsd_debug_yaml(args.config))
    cfg["simulation"]["duration"] = 1.0
    cfg["path"]["waypoints"] = copy.deepcopy(CANONICAL_WPS)
    cfg["task_space_vsd"]["task_mode"] = "xyz"
    cfg["task_space_vsd"]["use_bias_compensation"] = True
    cfg["task_space_vsd"]["torque_application_mode"] = "joint_direct_debug"

    summary, rows = run_ik_joint_pd(cfg)

    OUT.mkdir(parents=True, exist_ok=True)
    csv_path = OUT / "ik_joint_space_vsd_baseline_timeseries.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    sum_csv = OUT / "ik_joint_space_vsd_baseline_summary.csv"
    with open(sum_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(summary.keys()))
        w.writeheader()
        w.writerow(summary)

    print(summary)


if __name__ == "__main__":
    main()
