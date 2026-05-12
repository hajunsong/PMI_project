#!/usr/bin/env python3
from __future__ import annotations

import copy
import csv
import sys
from pathlib import Path

import mujoco as mj
import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from kinematics.inverse_kinematics import IKConfig, solve_ik_task_mode
from test_ik_joint_space_vsd_baseline import CANONICAL_WPS
from utils.mujoco_helpers import PKG_ROOT, VSD_DEBUG_MODEL_PATH, apply_ideal_qact_from_joint, apply_ideal_qjnt_equals_ratio_qact, joint_id, load_mjmodel
from utils.path_tracking_io import joint_actuator_bounds, load_task_space_vsd_debug_yaml, ordered_transmission_arrays

OUT = PKG_ROOT / "debug_outputs" / "torque_diagnostics"


def main() -> None:
    cfg = load_task_space_vsd_debug_yaml(None)
    cfg["path"]["waypoints"] = copy.deepcopy(CANONICAL_WPS)

    model = load_mjmodel(VSD_DEBUG_MODEL_PATH, strip_position_actuators=True)
    data = mj.MjData(model)
    scratch = mj.MjData(model)

    j_ord, a_ord, ratios = ordered_transmission_arrays(cfg)
    ratios = np.asarray(ratios, dtype=float)
    qa0 = np.asarray(cfg["path"]["initial_actuator_rad"], dtype=float)
    for i, n in enumerate(a_ord):
        data.qpos[int(model.jnt_qposadr[joint_id(model, n)])] = qa0[i]
    apply_ideal_qjnt_equals_ratio_qact(model, data, list(j_ord), list(a_ord), ratios)
    mj.mj_forward(model, data)

    q_seed = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in j_ord], dtype=float)
    qlj, qhj, qla, qha = joint_actuator_bounds(model, list(j_ord), list(a_ord))
    dof_j = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in j_ord], dtype=int)

    ik = IKConfig(
        weights_pos=(1.0, 1.0, 1.0), weight_roll=1.0, weight_pitch=1.0,
        damping=1e-3, regularization=1e-4, max_iterations=80, tolerance_ftol=1e-9,
        joint_side_order=tuple(j_ord),
    )

    rows = []
    for i, w in enumerate(CANONICAL_WPS):
        p = np.array([w["x"], w["y"], w["z"]], dtype=float)
        q_sol, _ = solve_ik_task_mode(
            model, scratch, p,
            roll_des=-np.pi / 2, pitch_des=0.0, task_feas_mode="xyz",
            ik=ik, q_seed=q_seed, bounds_lo=qlj, bounds_hi=qhj,
        )
        q_seed = q_sol.copy()

        for j, n in enumerate(j_ord):
            data.qpos[int(model.jnt_qposadr[joint_id(model, n)])] = q_sol[j]
        data.qvel[:] = 0.0
        apply_ideal_qact_from_joint(model, data, list(j_ord), list(a_ord), ratios)
        mj.mj_forward(model, data)

        tau_j = np.array([float(data.qfrc_bias[d]) for d in dof_j], dtype=float)
        tau_a = ratios * tau_j
        q_act = q_sol / ratios
        jm = np.minimum(qhj - q_sol, q_sol - qlj)
        am = np.minimum(qha - q_act, q_act - qla)

        rows.append(
            {
                "waypoint_index": i,
                "desired_xyz": p.tolist(),
                "ik_q_jnt": q_sol.tolist(),
                "equivalent_q_act": q_act.tolist(),
                "tau_bias_jnt": tau_j.tolist(),
                "tau_bias_act": tau_a.tolist(),
                "max_abs_tau_bias_jnt": float(np.max(np.abs(tau_j))),
                "max_abs_tau_bias_act": float(np.max(np.abs(tau_a))),
                "joint_limit_margin": float(np.min(jm)),
                "actuator_limit_margin": float(np.min(am)),
            }
        )

    OUT.mkdir(parents=True, exist_ok=True)
    csv_path = OUT / "static_bias_torque_waypoints.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    max_row = max(rows, key=lambda r: r["max_abs_tau_bias_jnt"])
    any_over_10 = any(r["max_abs_tau_bias_jnt"] > 10.0 for r in rows)
    rep = [
        "# Static Bias Torque Report",
        "",
        f"1. Any waypoint statically requiring >10Nm joint torque? {any_over_10}",
        f"2. Largest gravity/bias torque waypoint: idx={max_row['waypoint_index']} value={max_row['max_abs_tau_bias_jnt']:.6f}",
        "3. Actuator-side implied torque is tau_act = ratio * tau_jnt (see CSV columns).",
        "4. Waypoint limit proximity is in joint_limit_margin / actuator_limit_margin columns.",
    ]
    (OUT / "static_bias_torque_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
