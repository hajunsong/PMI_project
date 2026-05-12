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

from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

MODEL = PKG_ROOT / "models" / "pmi_arm_only_torque_debug.xml"
OUT = PKG_ROOT / "debug_outputs" / "joint_servo_debug"
J = ["jnt1", "jnt2", "jnt3", "jnt4"]
R = np.array([0.5333, 0.15, 0.3, 0.3], dtype=float)
Q0 = R * np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=float)
KP = np.array([30.0, 30.0, 30.0, 30.0])
KD = np.array([5.0, 5.0, 5.0, 5.0])


def run_case(target_joint: int, step: float, mode: str):
    model = load_mjmodel(MODEL, strip_position_actuators=True)
    data = mj.MjData(model)
    dof = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in J], dtype=int)
    for i, n in enumerate(J):
        data.qpos[int(model.jnt_qposadr[joint_id(model, n)])] = Q0[i]
    data.qvel[:] = 0.0
    mj.mj_forward(model, data)

    q_des = Q0.copy(); q_des[target_joint] += step
    qdot_des = np.zeros(4)
    dt = float(model.opt.timestep)
    n = int(round(2.0 / dt))

    for _ in range(n):
        q = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])]) for n0 in J])
        qdot = np.array([float(data.qvel[int(model.jnt_dofadr[joint_id(model, n0)])]) for n0 in J])
        mj.mj_forward(model, data)
        tb = np.array([float(data.qfrc_bias[d]) for d in dof])
        tpd = KP * (q_des - q) + KD * (qdot_des - qdot)
        tau = tb + tpd

        if mode == "flipped_for_target_joint":
            tau[target_joint] *= -1.0
        elif mode == "flipped_jnt1_only":
            tau[0] *= -1.0

        tau = np.clip(tau, -20.0, 20.0)
        data.qfrc_applied[:] = 0.0
        for i, d in enumerate(dof):
            data.qfrc_applied[d] = tau[i]
        mj.mj_step(model, data)

    qf = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])]) for n0 in J])
    return float(q_des[target_joint] - qf[target_joint])


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    for j in range(4):
        for step in [0.2, -0.2, 0.5, -0.5]:
            e_n = run_case(j, step, "normal")
            e_f = run_case(j, step, "flipped_for_target_joint")
            e_f1 = run_case(j, step, "flipped_jnt1_only")
            best = min([("normal", abs(e_n)), ("flipped_for_target_joint", abs(e_f)), ("flipped_jnt1_only", abs(e_f1))], key=lambda x: x[1])[0]
            rows.append({
                "target_joint": J[j], "step_size": step,
                "final_q_error_normal": e_n,
                "final_q_error_flipped": e_f,
                "final_q_error_flipped_jnt1_only": e_f1,
                "best_sign_mode": best,
            })

    with open(OUT / "torque_sign_flip.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)

    best_j1 = [r for r in rows if r["target_joint"] == "jnt1"]
    j1_flip_better = sum(abs(r["final_q_error_flipped"]) < abs(r["final_q_error_normal"]) for r in best_j1)
    rep = ["# Torque Sign Flip Report", "", f"- jnt1 flipped_for_target better cases: {j1_flip_better}/{len(best_j1)}"]
    rep.append("- per-joint best sign mode is in CSV")

    # suggested torque_sign
    torque_sign = [1, 1, 1, 1]
    if j1_flip_better >= 3:
        torque_sign[0] = -1
    rep.append(f"- suggested torque_sign vector: {torque_sign}")
    (OUT / "torque_sign_flip_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
