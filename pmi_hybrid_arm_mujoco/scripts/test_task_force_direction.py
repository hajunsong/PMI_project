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
from kinematics.task_jacobian import compute_task_jacobian_mode
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

MODEL = PKG_ROOT / "models" / "pmi_arm_only_torque_debug.xml"
OUT = PKG_ROOT / "debug_outputs" / "kinematic_sign"
J = ["jnt1", "jnt2", "jnt3", "jnt4"]
RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=float)
Q0 = RATIOS * np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=float)


def reset(model, data):
    for i, n in enumerate(J):
        data.qpos[int(model.jnt_qposadr[joint_id(model, n)])] = Q0[i]
    data.qvel[:] = 0.0
    mj.mj_forward(model, data)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    model = load_mjmodel(MODEL, strip_position_actuators=True)
    data = mj.MjData(model)
    scratch = mj.MjData(model)
    jac_s = mj.MjData(model)

    dof = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in J], dtype=int)
    tests = [np.array([10.0, 0.0, 0.0]), np.array([-10.0, 0.0, 0.0]), np.array([0.0, 10.0, 0.0]), np.array([0.0, -10.0, 0.0]), np.array([0.0, 0.0, 10.0]), np.array([0.0, 0.0, -10.0])]

    rows = []
    dt = float(model.opt.timestep)
    nstep = int(round(0.2 / dt))

    for F in tests:
        reset(model, data)
        q0 = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in J])
        p0, *_ = fk_ee_rp(model, scratch, q0, J)
        p0 = np.asarray(p0)

        jac_s.qpos[:] = data.qpos
        jac_s.qvel[:] = data.qvel
        mj.mj_forward(model, jac_s)
        Jp = compute_task_jacobian_mode(model, jac_s, joint_names=J, task_mode="xyz", ee_site_name="end_effector", mode="numerical", epsilon=1e-6)
        tau_task = Jp.T @ F

        for _ in range(nstep):
            mj.mj_forward(model, data)
            tb = np.array([float(data.qfrc_bias[d]) for d in dof])
            tau = tb + tau_task
            data.qfrc_applied[:] = 0.0
            for k in range(4):
                data.qfrc_applied[dof[k]] = tau[k]
            mj.mj_step(model, data)

        q1 = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in J])
        p1, *_ = fk_ee_rp(model, scratch, q1, J)
        p1 = np.asarray(p1)
        dp = p1 - p0

        expected_axis = int(np.argmax(np.abs(F)))
        expected_sign = int(np.sign(F[expected_axis]))
        actual_sign = int(np.sign(dp[expected_axis]))

        rows.append(
            {
                "F_xyz": F.tolist(),
                "tau_jnt": tau_task.tolist(),
                "delta_ee_xyz": dp.tolist(),
                "delta_q": (q1 - q0).tolist(),
                "expected_axis": expected_axis,
                "expected_sign": expected_sign,
                "actual_sign": actual_sign,
                "direction_match": bool(expected_sign == actual_sign or actual_sign == 0),
            }
        )

    with open(OUT / "task_force_direction.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    rep = ["# Task Force Direction Report", ""]
    for r in rows:
        rep.append(f"- F={r['F_xyz']}, dEE={r['delta_ee_xyz']}, match={r['direction_match']}")
    rep += [
        "",
        "- +Fx/-Fx, +Fy/-Fy, +Fz/-Fz direction checks are listed above.",
        "- If mismatch exists, inspect Jacobian sign/frame and qfrc sign conventions.",
    ]
    (OUT / "task_force_direction_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
