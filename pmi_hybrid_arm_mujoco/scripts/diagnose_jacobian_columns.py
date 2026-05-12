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

from kinematics.inverse_kinematics import IKConfig, solve_ik_task_mode
from kinematics.task_jacobian import compute_task_jacobian_mode
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

MODEL = PKG_ROOT / "models" / "pmi_arm_only_torque_debug.xml"
OUT = PKG_ROOT / "debug_outputs" / "kinematic_sign"
J = ["jnt1", "jnt2", "jnt3", "jnt4"]
WPS = [np.array([0.25, -0.20, -0.10]), np.array([0.00, -0.35, -0.15]), np.array([-0.25, -0.20, -0.10])]
RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=float)
Q0 = RATIOS * np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=float)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    model = load_mjmodel(MODEL, strip_position_actuators=True)
    data = mj.MjData(model)
    jac_s = mj.MjData(model)
    scratch = mj.MjData(model)

    q_lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in J])
    q_hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in J])

    ik = IKConfig((1, 1, 1), 1.0, 1.0, 1e-3, 1e-4, 80, 1e-9, tuple(J))
    q_seed = Q0.copy()
    qs = {"A_initial": Q0.copy()}
    for name, p in zip(["B_wp0", "C_wp_mid", "D_wp_final"], WPS):
        q_sol, _ = solve_ik_task_mode(model, scratch, p, roll_des=-np.pi/2, pitch_des=0.0, task_feas_mode="xyz", ik=ik, q_seed=q_seed, bounds_lo=q_lo, bounds_hi=q_hi)
        qs[name] = q_sol
        q_seed = q_sol.copy()

    rows = []
    report = ["# Jacobian Columns Report", ""]
    for cfg_name, q in qs.items():
        for i, n in enumerate(J):
            data.qpos[int(model.jnt_qposadr[joint_id(model, n)])] = q[i]
        data.qvel[:] = 0.0
        mj.mj_forward(model, data)

        jac_s.qpos[:] = data.qpos
        jac_s.qvel[:] = data.qvel
        mj.mj_forward(model, jac_s)
        J_num = compute_task_jacobian_mode(model, jac_s, joint_names=J, task_mode="xyz", ee_site_name="end_effector", mode="numerical", epsilon=1e-6)

        sid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "end_effector")
        jacp = np.zeros((3, model.nv), dtype=float)
        jacr = np.zeros((3, model.nv), dtype=float)
        mj.mj_jacSite(model, data, jacp, jacr, sid)
        dof = [int(model.jnt_dofadr[joint_id(model, n)]) for n in J]
        J_ana = jacp[:, dof]

        s = np.linalg.svd(J_num, compute_uv=False)
        report.append(f"## {cfg_name}")
        report.append(f"- J_pos numerical: {J_num.tolist()}")
        report.append(f"- singular values: {s.tolist()}")

        for j, n in enumerate(J):
            col = J_num[:, j]
            col_a = J_ana[:, j]
            rows.append(
                {
                    "config": cfg_name,
                    "joint": n,
                    "dx_dq": float(col[0]),
                    "dy_dq": float(col[1]),
                    "dz_dq": float(col[2]),
                    "col_norm": float(np.linalg.norm(col)),
                    "sign_dx": int(np.sign(col[0])),
                    "sign_dy": int(np.sign(col[1])),
                    "sign_dz": int(np.sign(col[2])),
                    "dx_analytic": float(col_a[0]),
                    "dy_analytic": float(col_a[1]),
                    "dz_analytic": float(col_a[2]),
                    "num_analytic_diff_norm": float(np.linalg.norm(col - col_a)),
                }
            )
        report.append("")

    with open(OUT / "jacobian_columns.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    init_rows = [r for r in rows if r["config"] == "A_initial"]
    neg_x = [r["joint"] for r in init_rows if r["dx_dq"] < 0]
    final_rows = [r for r in rows if r["config"] == "D_wp_final"]
    report += [
        "## Answers",
        f"- At initial q, joints contributing negative x motion: {neg_x}",
        f"- At final waypoint IK q, near singular hint from column norms: {[r['col_norm'] for r in final_rows]}",
        "- jnt1 sign consistency can be checked by comparing this report with joint sweep slopes.",
        "- numerical vs analytic mismatch is in num_analytic_diff_norm columns.",
    ]
    (OUT / "jacobian_columns_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
