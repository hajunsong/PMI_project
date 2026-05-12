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
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

MODEL = PKG_ROOT / "models" / "pmi_arm_only_torque_debug.xml"
OUT = PKG_ROOT / "debug_outputs" / "arm_only_debug"
J = ["jnt1", "jnt2", "jnt3", "jnt4"]
RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=float)
QJ0 = RATIOS * np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=float)


def main() -> None:
    model = load_mjmodel(MODEL, strip_position_actuators=True)
    data = mj.MjData(model)
    scratch = mj.MjData(model)
    dof = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in J], dtype=int)

    rows = []
    for sign in [+1.0, -1.0]:
        for i, n in enumerate(J):
            data.qpos[int(model.jnt_qposadr[joint_id(model, n)])] = QJ0[i]
        data.qvel[:] = 0.0
        mj.mj_forward(model, data)

        q0 = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])]) for n0 in J])
        p0, *_ = fk_ee_rp(model, scratch, q0, J)
        p0 = np.asarray(p0)

        dt = float(model.opt.timestep)
        nstep = int(round(2.0 / dt))
        for _ in range(nstep):
            mj.mj_forward(model, data)
            tb = np.array([float(data.qfrc_bias[d]) for d in dof])
            data.qfrc_applied[:] = 0.0
            for k in range(4):
                data.qfrc_applied[dof[k]] = sign * tb[k]
            mj.mj_step(model, data)

        q1 = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])]) for n0 in J])
        p1, *_ = fk_ee_rp(model, scratch, q1, J)
        p1 = np.asarray(p1)

        rows.append({
            "bias_sign": int(sign),
            "drift_distance": float(np.linalg.norm(p1 - p0)),
            "q_drift": float(np.linalg.norm(q1 - q0)),
        })

    OUT.mkdir(parents=True, exist_ok=True)
    with open(OUT / "bias_sign_check.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    best = min(rows, key=lambda r: r["drift_distance"])
    rep = ["# Bias Sign Check", "", f"best sign by drift: {best['bias_sign']} (drift={best['drift_distance']:.6f})"]
    (OUT / "bias_sign_check_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
