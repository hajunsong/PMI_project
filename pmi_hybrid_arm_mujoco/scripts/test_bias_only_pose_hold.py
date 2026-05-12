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

from kinematics.forward_kinematics import fk_ee_rp
from kinematics.inverse_kinematics import IKConfig, solve_ik_task_mode
from test_ik_joint_space_vsd_baseline import CANONICAL_WPS
from utils.mujoco_helpers import PKG_ROOT, VSD_DEBUG_MODEL_PATH, apply_ideal_qact_from_joint, apply_ideal_qjnt_equals_ratio_qact, joint_id, load_mjmodel
from utils.path_tracking_io import load_task_space_vsd_debug_yaml, ordered_transmission_arrays

OUT = PKG_ROOT / "debug_outputs" / "torque_diagnostics"


def main() -> None:
    cfg = load_task_space_vsd_debug_yaml(None)
    cfg["path"]["waypoints"] = copy.deepcopy(CANONICAL_WPS)

    model = load_mjmodel(VSD_DEBUG_MODEL_PATH, strip_position_actuators=True)
    dt = float(cfg["simulation"]["dt"])
    n = int(round(2.0 / dt))
    data = mj.MjData(model)
    scratch = mj.MjData(model)

    j_ord, a_ord, ratios = ordered_transmission_arrays(cfg)
    ratios = np.asarray(ratios, dtype=float)
    dof_j = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in j_ord], dtype=int)

    # initial from actuator seed
    qa0 = np.asarray(cfg["path"]["initial_actuator_rad"], dtype=float)
    for i, n0 in enumerate(a_ord):
        data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])] = qa0[i]
    apply_ideal_qjnt_equals_ratio_qact(model, data, list(j_ord), list(a_ord), ratios)
    mj.mj_forward(model, data)

    q_seed = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])]) for n0 in j_ord], dtype=float)
    qlj = np.array([-3.1416, -1.5709, -1.5709, -1.5709], dtype=float)
    qhj = np.array([3.1416, 1.5709, 1.5709, 1.5709], dtype=float)

    ik = IKConfig((1, 1, 1), 1.0, 1.0, 1e-3, 1e-4, 80, 1e-9, tuple(j_ord))

    rows = []
    for i, w in enumerate(CANONICAL_WPS):
        p = np.array([w["x"], w["y"], w["z"]], dtype=float)
        q_sol, _ = solve_ik_task_mode(
            model, scratch, p,
            roll_des=-np.pi / 2, pitch_des=0.0, task_feas_mode="xyz",
            ik=ik, q_seed=q_seed, bounds_lo=qlj, bounds_hi=qhj,
        )
        q_seed = q_sol.copy()

        for j, n0 in enumerate(j_ord):
            data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])] = q_sol[j]
        data.qvel[:] = 0.0
        apply_ideal_qact_from_joint(model, data, list(j_ord), list(a_ord), ratios)
        mj.mj_forward(model, data)

        q_init = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])]) for n0 in j_ord], dtype=float)
        p_init, *_ = fk_ee_rp(model, scratch, q_init, list(j_ord))
        p_init = np.asarray(p_init, dtype=float)

        max_qvel = 0.0
        max_tb = 0.0
        for _k in range(n):
            mj.mj_forward(model, data)
            tau_bias = np.array([float(data.qfrc_bias[d]) for d in dof_j], dtype=float)
            max_tb = max(max_tb, float(np.max(np.abs(tau_bias))))
            data.qfrc_applied[:] = 0.0
            for j in range(4):
                data.qfrc_applied[dof_j[j]] = tau_bias[j]
            mj.mj_step(model, data)
            apply_ideal_qact_from_joint(model, data, list(j_ord), list(a_ord), ratios)
            qvel_now = np.array([float(data.qvel[int(model.jnt_dofadr[joint_id(model, n0)])]) for n0 in j_ord], dtype=float)
            max_qvel = max(max_qvel, float(np.max(np.abs(qvel_now))))

        q_fin = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])]) for n0 in j_ord], dtype=float)
        p_fin, *_ = fk_ee_rp(model, scratch, q_fin, list(j_ord))
        p_fin = np.asarray(p_fin, dtype=float)

        rows.append(
            {
                "waypoint_index": i,
                "initial_EE_xyz": p_init.tolist(),
                "final_EE_xyz": p_fin.tolist(),
                "drift_distance": float(np.linalg.norm(p_fin - p_init)),
                "q_drift_norm": float(np.linalg.norm(q_fin - q_init)),
                "max_qvel": max_qvel,
                "max_tau_bias": max_tb,
            }
        )

    OUT.mkdir(parents=True, exist_ok=True)
    with open(OUT / "bias_only_pose_hold.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    max_drift = max(r["drift_distance"] for r in rows)
    rep = [
        "# Bias-only Pose Hold Report",
        "",
        f"max drift distance across waypoints: {max_drift:.6f}",
        "If drift is large, bias compensation sign/indexing/qfrc path may be wrong.",
    ]
    (OUT / "bias_only_pose_hold_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
