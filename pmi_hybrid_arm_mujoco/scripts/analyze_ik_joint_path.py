#!/usr/bin/env python3
from __future__ import annotations

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
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

MODEL = PKG_ROOT / "models" / "pmi_arm_only_torque_debug.xml"
OUT = PKG_ROOT / "debug_outputs" / "kinematic_sign"
J = ["jnt1", "jnt2", "jnt3", "jnt4"]
RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=float)
Q_ACT0 = np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=float)
Q0 = RATIOS * Q_ACT0
WPS = [WaypointXYZ(0.0, 0.25, -0.20, -0.10), WaypointXYZ(0.5, 0.00, -0.35, -0.15), WaypointXYZ(1.0, -0.25, -0.20, -0.10)]


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    model = load_mjmodel(MODEL, strip_position_actuators=True)
    data = mj.MjData(model)
    scratch = mj.MjData(model)

    q_lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in J])
    q_hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in J])

    for i, n in enumerate(J):
        data.qpos[int(model.jnt_qposadr[joint_id(model, n)])] = Q0[i]
    data.qvel[:] = 0.0
    mj.mj_forward(model, data)
    p_init, *_ = fk_ee_rp(model, scratch, Q0, J)
    p_init = np.asarray(p_init)

    spline = CartesianQuinticPath(WPS)
    ts = np.linspace(0.0, 1.0, 101)
    ik = IKConfig((1, 1, 1), 1.0, 1.0, 1e-3, 1e-4, 80, 1e-9, tuple(J))
    q_seed = Q0.copy()

    rows = []
    q_hist = []
    for i, t in enumerate(ts):
        p_des, _, _ = spline.sample(float(t))
        p_des = np.asarray(p_des)
        q, diag = solve_ik_task_mode(model, scratch, p_des, roll_des=-np.pi/2, pitch_des=0.0, task_feas_mode="xyz", ik=ik, q_seed=q_seed, bounds_lo=q_lo, bounds_hi=q_hi)
        q_seed = q.copy()
        q_hist.append(q.copy())
        q_act = q / RATIOS
        p_sol, *_ = fk_ee_rp(model, scratch, q, J)
        p_sol = np.asarray(p_sol)
        jm = np.minimum(q_hi - q, q - q_lo)
        am = np.abs(q_act)
        rows.append(
            {
                "t": float(t),
                "q_jnt": q.tolist(),
                "q_act_equiv": q_act.tolist(),
                "ee_solved_xyz": p_sol.tolist(),
                "residual": float(diag["ik_norm_geom"]),
                "joint_limit_margin": float(np.min(jm)),
                "actuator_limit_margin_proxy": float(np.min(20.0 - am)),
            }
        )

    with open(OUT / "ik_joint_path.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    q_hist = np.asarray(q_hist)
    q0_ik = q_hist[0]
    qmid = q_hist[50]
    qend = q_hist[-1]
    first_wp = np.array([WPS[0].x, WPS[0].y, WPS[0].z])
    init_err = float(np.linalg.norm(p_init - first_wp))
    dq0 = q0_ik - Q0
    max_joint_change = np.max(np.abs(q_hist - q_hist[0]), axis=0)

    rep = [
        "# IK Joint Path Report",
        "",
        f"- initial EE xyz: {p_init.tolist()}",
        f"- first waypoint xyz: {first_wp.tolist()}",
        f"- initial EE to first waypoint error: {init_err:.6f}",
        f"- IK q at first waypoint: {q0_ik.tolist()}",
        f"- delta(initial q -> IK first): {dq0.tolist()}",
        f"- IK q at t=0.5: {qmid.tolist()}",
        f"- IK q at t=1.0: {qend.tolist()}",
        f"- max joint changes along IK path: {max_joint_change.tolist()}",
        "- singular crossing hint should be checked with jacobian_columns singular values.",
    ]
    (OUT / "ik_joint_path_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
