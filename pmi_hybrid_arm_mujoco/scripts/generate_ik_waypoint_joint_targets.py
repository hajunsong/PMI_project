#!/usr/bin/env python3
from __future__ import annotations

import csv
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np

from kinematics.forward_kinematics import fk_ee_rp
from kinematics.inverse_kinematics import IKConfig, solve_ik_task_mode
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

MODEL = PKG_ROOT / "models" / "pmi_arm_only_torque_debug.xml"
OUT = PKG_ROOT / "debug_outputs" / "ik_joint_tracking"
J = ["jnt1", "jnt2", "jnt3", "jnt4"]
R = np.array([0.5333, 0.15, 0.3, 0.3], dtype=float)
Q0 = R * np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=float)
WPS = [np.array([0.25, -0.20, -0.10]), np.array([0.00, -0.35, -0.15]), np.array([-0.25, -0.20, -0.10])]


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    model = load_mjmodel(MODEL, strip_position_actuators=True)
    scratch = __import__("mujoco").MjData(model)

    q_lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in J])
    q_hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in J])
    ik = IKConfig((1, 1, 1), 1.0, 1.0, 1e-3, 1e-4, 80, 1e-9, tuple(J))

    rows = []
    q_prev = Q0.copy()
    qs = []
    for i, wp in enumerate(WPS):
        q, d = solve_ik_task_mode(model, scratch, wp, roll_des=-np.pi/2, pitch_des=0.0, task_feas_mode="xyz", ik=ik, q_seed=q_prev, bounds_lo=q_lo, bounds_hi=q_hi)
        p_sol, *_ = fk_ee_rp(model, scratch, q, J)
        p_sol = np.asarray(p_sol)
        jm = np.minimum(q_hi - q, q - q_lo)
        qa = q / R
        rows.append({
            "waypoint": i,
            "desired_xyz": wp.tolist(),
            "q_jnt": q.tolist(),
            "q_act_equiv": qa.tolist(),
            "residual_norm": float(d["ik_norm_geom"]),
            "joint_margin_min": float(np.min(jm)),
            "delta_q_from_prev": (q - q_prev).tolist(),
            "ee_solved_xyz": p_sol.tolist(),
        })
        qs.append(q.copy())
        q_prev = q.copy()

    with open(OUT / "ik_waypoint_targets.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    rep = ["# IK Waypoint Targets", ""]
    rep.append(f"- q0={qs[0].tolist()}")
    rep.append(f"- q1={qs[1].tolist()}")
    rep.append(f"- q2={qs[2].tolist()}")
    rep.append(f"- residuals={[r['residual_norm'] for r in rows]}")
    (OUT / "ik_waypoint_targets_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
