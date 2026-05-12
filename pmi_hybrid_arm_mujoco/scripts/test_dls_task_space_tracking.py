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
from kinematics.task_jacobian import compute_task_jacobian_mode
from kinematics.trajectory import CartesianQuinticPath, WaypointXYZ
from test_ik_joint_space_vsd_baseline import CANONICAL_WPS, run_ik_joint_pd
from utils.mujoco_helpers import PKG_ROOT, VSD_DEBUG_MODEL_PATH, apply_ideal_qact_from_joint, apply_ideal_qjnt_equals_ratio_qact, joint_id, load_mjmodel
from utils.path_tracking_io import gains_limits_task_space_vsd, joint_actuator_bounds, load_task_space_vsd_debug_yaml, ordered_transmission_arrays
from utils.task_space_vsd_rollout import rollout_task_space_vsd

OUT = PKG_ROOT / "debug_outputs" / "task_space_vsd_next"


def run_dls(cfg: dict, tau_limit_override: float | None = None) -> tuple[dict, list[dict]]:
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
    jac_s = mj.MjData(model)

    for i, nm in enumerate(a_ord):
        data.qpos[int(model.jnt_qposadr[joint_id(model, nm)])] = qa0[i]
    apply_ideal_qjnt_equals_ratio_qact(model, data, list(j_ord), list(a_ord), ratios)
    mj.mj_forward(model, data)

    dof_j = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in j_ord], dtype=int)
    qlj, qhj, qla, qha = joint_actuator_bounds(model, list(j_ord), list(a_ord))
    _tm, gk, gd, _lf, tj_lim, _ta_lim = gains_limits_task_space_vsd(cfg["task_space_vsd"])
    Dq = np.asarray([gd[0], gd[1], gd[2], 4.0], dtype=float)
    Kq_int = np.asarray([20.0, 20.0, 20.0, 5.0], dtype=float)
    tj_lim = np.asarray(tj_lim, dtype=float)
    if tau_limit_override is not None:
        tj_lim[:] = float(tau_limit_override)

    Kx = np.asarray([3.0, 3.0, 3.0], dtype=float)
    lam = 0.08

    q_cmd = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in j_ord], dtype=float)

    rows = []
    ee_err = []
    sat_steps = 0
    jl_steps = 0
    max_tau = 0.0
    max_tau_unc = 0.0
    final_des = np.zeros(3, dtype=float)
    final_act = np.zeros(3, dtype=float)

    for k, t in enumerate(ts):
        q = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in j_ord], dtype=float)
        qdot = np.array([float(data.qvel[int(model.jnt_dofadr[joint_id(model, n)])]) for n in j_ord], dtype=float)

        p_des, pdot_des, _ = spline.sample(float(t))
        p_des = np.asarray(p_des, dtype=float)
        pdot_des = np.asarray(pdot_des, dtype=float)

        p_act, *_ = fk_ee_rp(model, scratch, q, list(j_ord))
        p_act = np.asarray(p_act, dtype=float)
        e = p_des - p_act
        xdot_cmd = pdot_des + Kx * e

        jac_s.qpos[:] = data.qpos
        jac_s.qvel[:] = data.qvel
        mj.mj_forward(model, jac_s)
        J = compute_task_jacobian_mode(
            model,
            jac_s,
            joint_names=list(j_ord),
            task_mode="xyz",
            ee_site_name="end_effector",
            mode="numerical",
            epsilon=1e-6,
        )
        JJt = J @ J.T
        J_dls = J.T @ np.linalg.inv(JJt + (lam**2) * np.eye(3))
        qdot_cmd = J_dls @ xdot_cmd

        # joint limit avoidance by velocity clipping near bounds
        margin = np.minimum(qhj - q, q - qlj)
        for i in range(4):
            if margin[i] < 0.05:
                if q[i] > (qhj[i] - 0.05):
                    qdot_cmd[i] = min(qdot_cmd[i], 0.0)
                if q[i] < (qlj[i] + 0.05):
                    qdot_cmd[i] = max(qdot_cmd[i], 0.0)

        q_cmd = q_cmd + qdot_cmd * dt
        q_cmd = np.clip(q_cmd, qlj, qhj)

        mj.mj_forward(model, data)
        tau_bias = np.array([float(data.qfrc_bias[int(dof_j[i])]) for i in range(4)], dtype=float)
        tau_unc = tau_bias + Kq_int * (q_cmd - q) + Dq * (qdot_cmd - qdot)
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
        qa = q_aft / ratios
        p_aft, *_ = fk_ee_rp(model, scratch, q_aft, list(j_ord))
        p_aft = np.asarray(p_aft, dtype=float)
        err = p_des - p_aft
        final_des = p_des.copy()
        final_act = p_aft.copy()
        en = float(np.linalg.norm(err))
        ee_err.append(en)

        j_margin = np.minimum(qhj - q_aft, q_aft - qlj)
        a_margin = np.minimum(qha - qa, qa - qla)
        if float(np.min(j_margin)) < 0.0 or float(np.min(a_margin)) < 0.0:
            jl_steps += 1

        sv = np.linalg.svd(J, compute_uv=False)
        max_tau = max(max_tau, float(np.max(np.abs(tau))))
        max_tau_unc = max(max_tau_unc, float(np.max(np.abs(tau_unc))))
        rows.append(
            {
                "t": float(t),
                "pos_err_norm": en,
                "sv1": float(sv[0]), "sv2": float(sv[1]), "sv3": float(sv[2]),
                "lambda": lam,
                "xdot_cmd_x": float(xdot_cmd[0]), "xdot_cmd_y": float(xdot_cmd[1]), "xdot_cmd_z": float(xdot_cmd[2]),
                "qdot_cmd_1": float(qdot_cmd[0]), "qdot_cmd_2": float(qdot_cmd[1]), "qdot_cmd_3": float(qdot_cmd[2]), "qdot_cmd_4": float(qdot_cmd[3]),
                "tau_1": float(tau[0]), "tau_2": float(tau[1]), "tau_3": float(tau[2]), "tau_4": float(tau[3]),
                "joint_margin_min": float(np.min(j_margin)), "act_margin_min": float(np.min(a_margin)),
            }
        )

    ee = np.asarray(ee_err, dtype=float)
    summary = {
        "controller": "dls_resolved_rate_joint_torque",
        "rms_pos": float(np.sqrt(np.mean(ee**2))),
        "max_pos": float(np.max(ee)),
        "final_pos_err": float(ee[-1]),
        "max_tau": float(max_tau),
        "max_tau_before_clip": float(max_tau_unc),
        "joint_limit_steps": int(jl_steps),
        "torque_saturation_steps": int(sat_steps),
        "final_desired_xyz": final_des.tolist(),
        "final_actual_xyz": final_act.tolist(),
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

    # A: pure J^T F
    rA = rollout_task_space_vsd(cfg)
    sumA = {
        "controller": "pure_task_space_jtf_vsd",
        "rms_pos": float(rA["rms_pos"]),
        "max_pos": float(rA["max_pos"]),
        "final_pos_err": float(rA["final_pos_err"]),
        "max_tau": float(rA["max_tau"]),
        "joint_limit_steps": int(rA["jl_activation_steps"]),
        "torque_saturation_steps": int(rA["torque_sat_steps"]),
    }

    # B: IK + joint PD
    sumB, rowsB = run_ik_joint_pd(cfg)

    # C: DLS resolved-rate + torque servo
    sumC, rowsC = run_dls(cfg)

    OUT.mkdir(parents=True, exist_ok=True)
    cmp_csv = OUT / "controller_comparison.csv"
    rows_cmp = [sumA, sumB, sumC]
    with open(cmp_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows_cmp[0].keys()))
        w.writeheader()
        w.writerows(rows_cmp)

    with open(OUT / "dls_timeseries.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rowsC[0].keys()))
        w.writeheader()
        w.writerows(rowsC)

    success = []
    for row in rows_cmp:
        ok = (
            row["rms_pos"] < 0.05
            and row["max_pos"] < 0.10
            and row["final_pos_err"] < 0.02
            and row["joint_limit_steps"] == 0
            and row["torque_saturation_steps"] <= 5
        )
        success.append((row["controller"], ok))

    reason = "controller structure issue"
    if all(r["max_tau"] >= 9.99 for r in rows_cmp):
        reason = "torque limit issue"
    if any(r["joint_limit_steps"] > 0 for r in rows_cmp):
        reason = "model/transmission issue or limit-handling issue"

    rep = [
        "# Controller Comparison Report",
        "",
        "Controllers:",
        "- A. pure JTF task-space VSD",
        "- B. IK + joint-space VSD",
        "- C. DLS resolved-rate + joint torque servo",
        "",
    ]
    for r in rows_cmp:
        rep.append(
            "- {controller}: rms={rms_pos:.6f}, max={max_pos:.6f}, final={final_pos_err:.6f}, max_tau={max_tau:.4f}, jl={joint_limit_steps}, sat={torque_saturation_steps}".format(**r)
        )
    rep += ["", "Success criteria pass/fail:"]
    for name, ok in success:
        rep.append(f"- {name}: {ok}")
    if not any(ok for _, ok in success):
        rep += ["", f"No controller reached target criteria. Likely dominant limit: {reason}."]

    (OUT / "controller_comparison_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")
    print(rows_cmp)


if __name__ == "__main__":
    main()
