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


def reset(model, data):
    for i, n in enumerate(J):
        data.qpos[int(model.jnt_qposadr[joint_id(model, n)])] = Q0[i]
    data.qvel[:] = 0.0
    mj.mj_forward(model, data)


def run_case(target_joint: int, step: float, duration: float = 2.0, tau_limit: float = 20.0):
    model = load_mjmodel(MODEL, strip_position_actuators=True)
    data = mj.MjData(model)
    dof = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in J], dtype=int)
    reset(model, data)

    dt = float(model.opt.timestep)
    n = int(round(duration / dt))

    q_des = Q0.copy()
    q_des[target_joint] = Q0[target_joint] + step
    qdot_des = np.zeros(4, dtype=float)

    rows = []
    qs = []
    sat_count = 0
    for k in range(n):
        t = k * dt
        q = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])]) for n0 in J])
        qdot = np.array([float(data.qvel[int(model.jnt_dofadr[joint_id(model, n0)])]) for n0 in J])
        mj.mj_forward(model, data)
        tau_bias = np.array([float(data.qfrc_bias[d]) for d in dof])
        tau_pd = KP * (q_des - q) + KD * (qdot_des - qdot)
        tau_unc = tau_bias + tau_pd
        tau = np.clip(tau_unc, -tau_limit, tau_limit)
        sat = bool(np.any(np.abs(tau_unc - tau) > 1e-9))
        sat_count += int(sat)

        data.qfrc_applied[:] = 0.0
        for i, d in enumerate(dof):
            data.qfrc_applied[d] = tau[i]
        mj.mj_step(model, data)

        qn = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])]) for n0 in J])
        qs.append(float(qn[target_joint]))
        rows.append({
            "target_joint": J[target_joint],
            "step_size": step,
            "time": t,
            "q_des": float(q_des[target_joint]),
            "q_actual": float(qn[target_joint]),
            "q_error": float(q_des[target_joint] - qn[target_joint]),
            "qdot": float(qdot[target_joint]),
            "tau_bias": float(tau_bias[target_joint]),
            "tau_pd": float(tau_pd[target_joint]),
            "tau_total_before_clip": float(tau_unc[target_joint]),
            "tau_total_after_clip": float(tau[target_joint]),
            "saturation_flag": sat,
        })

    q_series = np.array(qs)
    final_err = float(q_des[target_joint] - q_series[-1])
    overshoot = float(np.max(np.abs(q_series - q_des[target_joint])))
    steady = float(np.mean(q_des[target_joint] - q_series[-max(1, int(0.2 / dt)) :]))
    return rows, {
        "target_joint": J[target_joint],
        "step_size": step,
        "final_q_error": final_err,
        "overshoot": overshoot,
        "steady_state_error": steady,
        "saturation_steps": sat_count,
        "moved_wrong_direction": bool(np.sign(step) != np.sign(q_series[-1] - Q0[target_joint]) and abs(q_series[-1] - Q0[target_joint]) > 1e-6),
    }


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    all_rows = []
    summary = []
    for j in range(4):
        for step in [0.2, -0.2, 0.5, -0.5]:
            rows, summ = run_case(j, step)
            all_rows.extend(rows)
            summary.append(summ)

    with open(OUT / "single_joint_tracking.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
        w.writeheader(); w.writerows(all_rows)

    by_joint = {jn: [s for s in summary if s["target_joint"] == jn] for jn in J}
    worst = max(summary, key=lambda s: abs(s["steady_state_error"]))
    bad_dir = [s for s in summary if s["moved_wrong_direction"]]
    rep = ["# Single Joint Tracking Report", ""]
    rep.append("- Can each joint track +/-0.2 and +/-0.5? see per-case errors below.")
    for s in summary:
        rep.append(f"- {s['target_joint']} step {s['step_size']:+.2f}: final={s['final_q_error']:.5f}, steady={s['steady_state_error']:.5f}, sat={s['saturation_steps']}, wrong_dir={s['moved_wrong_direction']}")
    rep += [
        "",
        f"- largest steady-state error: {worst['target_joint']} step {worst['step_size']} (|e_ss|={abs(worst['steady_state_error']):.5f})",
        f"- wrong direction cases: {len(bad_dir)}",
        f"- jnt1 failing while others work? {'yes' if any(s['target_joint']=='jnt1' and s['moved_wrong_direction'] for s in summary) else 'no'}",
    ]
    (OUT / "single_joint_tracking_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
