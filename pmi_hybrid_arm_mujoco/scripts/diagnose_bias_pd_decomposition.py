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


def run(target_joint: int, step: float, variant: str) -> float:
    model = load_mjmodel(MODEL, strip_position_actuators=True)
    data = mj.MjData(model)
    dof = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in J], dtype=int)
    for i, n in enumerate(J):
        data.qpos[int(model.jnt_qposadr[joint_id(model, n)])] = Q0[i]
    data.qvel[:] = 0.0
    mj.mj_forward(model, data)

    q_des = Q0.copy(); q_des[target_joint] += step
    dt = float(model.opt.timestep)
    n = int(round(2.0 / dt))
    rows = []
    for k in range(n):
        q = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])]) for n0 in J])
        qdot = np.array([float(data.qvel[int(model.jnt_dofadr[joint_id(model, n0)])]) for n0 in J])
        mj.mj_forward(model, data)
        tb = np.array([float(data.qfrc_bias[d]) for d in dof])
        tpd = KP * (q_des - q) + KD * (0.0 - qdot)
        if variant == "bias_only":
            tau = tb
        elif variant == "pd_only":
            tau = tpd
        elif variant == "bias_plus_pd":
            tau = tb + tpd
        else:
            tau = tb - tpd
        tau = np.clip(tau, -20.0, 20.0)

        data.qfrc_applied[:] = 0.0
        for i, d in enumerate(dof):
            data.qfrc_applied[d] = tau[i]
        mj.mj_step(model, data)

        rows.append({
            "target_joint": J[target_joint],
            "step_size": step,
            "variant": variant,
            "time": k * dt,
            "tau_bias": float(tb[target_joint]),
            "tau_pd": float(tpd[target_joint]),
            "tau_total": float(tau[target_joint]),
        })

    qf = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])]) for n0 in J])
    err = float(q_des[target_joint] - qf[target_joint])
    return err, rows


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    all_rows = []
    summary = []
    variants = ["bias_only", "pd_only", "bias_plus_pd", "bias_minus_pd"]

    for j in range(4):
        for step in [0.2, -0.2]:
            for v in variants:
                e, rows = run(j, step, v)
                all_rows.extend(rows)
                summary.append({"joint": J[j], "step": step, "variant": v, "final_error": e})

    with open(OUT / "bias_pd_decomposition.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
        w.writeheader(); w.writerows(all_rows)

    rep = ["# Bias PD Decomposition Report", ""]
    for s in summary:
        rep.append(f"- {s['joint']} step {s['step']:+.2f} {s['variant']}: final_error={s['final_error']:.5f}")
    rep += [
        "",
        "- Compare bias+PD vs bias-PD signs above.",
        "- If bias-PD is consistently better for a joint, PD sign for that joint is likely inverted.",
    ]
    (OUT / "bias_pd_decomposition_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
