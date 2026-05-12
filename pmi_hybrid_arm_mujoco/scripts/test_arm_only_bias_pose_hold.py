#!/usr/bin/env python3
from __future__ import annotations

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
from kinematics.trajectory import WaypointXYZ
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

MODEL = PKG_ROOT / "models" / "pmi_arm_only_torque_debug.xml"
OUT = PKG_ROOT / "debug_outputs" / "arm_only_debug"
WPS = [
    WaypointXYZ(0.0, 0.25, -0.20, -0.10),
    WaypointXYZ(0.5, 0.00, -0.35, -0.15),
    WaypointXYZ(1.0, -0.25, -0.20, -0.10),
]
J = ["jnt1", "jnt2", "jnt3", "jnt4"]
RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=float)
QJ0 = RATIOS * np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=float)


def solve_wp_ik(model, scratch):
    q_lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in J])
    q_hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in J])
    ik = IKConfig((1, 1, 1), 1.0, 1.0, 1e-3, 1e-4, 80, 1e-9, tuple(J))
    q_seed = QJ0.copy()
    qs = []
    for w in WPS:
        p = np.array([w.x, w.y, w.z], dtype=float)
        q, _ = solve_ik_task_mode(model, scratch, p, roll_des=-np.pi/2, pitch_des=0.0, task_feas_mode="xyz", ik=ik, q_seed=q_seed, bounds_lo=q_lo, bounds_hi=q_hi)
        qs.append(q)
        q_seed = q.copy()
    return qs


def main() -> None:
    model = load_mjmodel(MODEL, strip_position_actuators=True)
    data = mj.MjData(model)
    scratch = mj.MjData(model)
    dof = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in J], dtype=int)

    qs = solve_wp_ik(model, scratch)
    dt = float(model.opt.timestep)
    n = int(round(2.0 / dt))

    rows = []
    for i, q in enumerate(qs):
        for j, n0 in enumerate(J):
            data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])] = q[j]
        data.qvel[:] = 0.0
        mj.mj_forward(model, data)

        q0 = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])]) for n0 in J])
        p0, *_ = fk_ee_rp(model, scratch, q0, J)
        p0 = np.asarray(p0)
        max_qvel = 0.0

        for _ in range(n):
            mj.mj_forward(model, data)
            tau_bias = np.array([float(data.qfrc_bias[d]) for d in dof])
            data.qfrc_applied[:] = 0.0
            for k in range(4):
                data.qfrc_applied[dof[k]] = tau_bias[k]
            mj.mj_step(model, data)
            qvel = np.array([float(data.qvel[int(model.jnt_dofadr[joint_id(model, n0)])]) for n0 in J])
            max_qvel = max(max_qvel, float(np.max(np.abs(qvel))))

        q1 = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])]) for n0 in J])
        p1, *_ = fk_ee_rp(model, scratch, q1, J)
        p1 = np.asarray(p1)

        rows.append({
            "waypoint_index": i,
            "initial_ee_xyz": p0.tolist(),
            "final_ee_xyz": p1.tolist(),
            "drift_distance": float(np.linalg.norm(p1 - p0)),
            "q_drift": float(np.linalg.norm(q1 - q0)),
            "max_qvel": max_qvel,
        })

    OUT.mkdir(parents=True, exist_ok=True)
    with open(OUT / "bias_pose_hold.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    rep = ["# Arm-only Bias Pose Hold", ""]
    rep.append(f"- max drift: {max(r['drift_distance'] for r in rows):.6f}")
    (OUT / "bias_pose_hold_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
