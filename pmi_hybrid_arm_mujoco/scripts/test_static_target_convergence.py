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
OUT = PKG_ROOT / "debug_outputs" / "static_target"
J = ["jnt1", "jnt2", "jnt3", "jnt4"]
RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=float)
Q0 = RATIOS * np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=float)
TARGETS = {
    "A_first": np.array([0.25, -0.20, -0.10]),
    "B_middle": np.array([0.00, -0.35, -0.15]),
    "C_final": np.array([-0.25, -0.20, -0.10]),
}


def run_target(target: np.ndarray) -> dict:
    model = load_mjmodel(MODEL, strip_position_actuators=True)
    data = mj.MjData(model)
    scratch = mj.MjData(model)
    jac_s = mj.MjData(model)

    for i, n in enumerate(J):
        data.qpos[int(model.jnt_qposadr[joint_id(model, n)])] = Q0[i]
    data.qvel[:] = 0.0
    mj.mj_forward(model, data)

    dof = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in J], dtype=int)
    q_lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in J])
    q_hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in J])

    dt = float(model.opt.timestep)
    dur = 3.0
    nstep = int(round(dur / dt))

    errs = []
    max_tau = 0.0
    min_margin = 1e9
    sv_min_hist = []

    for _ in range(nstep):
        q = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in J])
        qdot = np.array([float(data.qvel[int(model.jnt_dofadr[joint_id(model, n)])]) for n in J])
        p, *_ = fk_ee_rp(model, scratch, q, J)
        p = np.asarray(p)

        jac_s.qpos[:] = data.qpos
        jac_s.qvel[:] = data.qvel
        mj.mj_forward(model, jac_s)
        Jp = compute_task_jacobian_mode(model, jac_s, joint_names=J, task_mode="xyz", ee_site_name="end_effector", mode="numerical", epsilon=1e-6)

        ydot = Jp @ qdot
        e = target - p
        F = np.array([300.0, 300.0, 300.0]) * e + np.array([8.0, 8.0, 8.0]) * (0.0 - ydot)
        tau_bias = np.array([float(data.qfrc_bias[d]) for d in dof])
        tau_unc = tau_bias + Jp.T @ F
        tau = np.clip(tau_unc, -20.0, 20.0)

        data.qfrc_applied[:] = 0.0
        for k in range(4):
            data.qfrc_applied[dof[k]] = tau[k]
        mj.mj_step(model, data)

        err = float(np.linalg.norm(target - p))
        errs.append(err)
        max_tau = max(max_tau, float(np.max(np.abs(tau))))
        q_now = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in J])
        min_margin = min(min_margin, float(np.min(np.minimum(q_hi - q_now, q_now - q_lo))))
        sv = np.linalg.svd(Jp, compute_uv=False)
        sv_min_hist.append(float(np.min(sv)))

    errs = np.asarray(errs)
    qf = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in J])
    pf, *_ = fk_ee_rp(model, scratch, qf, J)
    pf = np.asarray(pf)

    thr = 0.02
    idx = np.where(errs < thr)[0]
    tconv = float(idx[0] * dt) if len(idx) > 0 else -1.0

    return {
        "rms_error": float(np.sqrt(np.mean(errs**2))),
        "final_error": float(errs[-1]),
        "convergence_time": tconv,
        "final_q": qf.tolist(),
        "final_ee_xyz": pf.tolist(),
        "max_tau": max_tau,
        "joint_limit_margin_min": float(min_margin),
        "sv_min_over_time": float(np.min(np.asarray(sv_min_hist))),
    }


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    for name, t in TARGETS.items():
        r = run_target(t)
        row = {"target": name, "target_xyz": t.tolist(), **r}
        rows.append(row)

    with open(OUT / "static_target_convergence.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    rep = ["# Static Target Convergence Report", ""]
    for r in rows:
        rep.append(
            "- {target}: final={final_error:.6f}, rms={rms_error:.6f}, t_conv={convergence_time:.3f}, max_tau={max_tau:.3f}".format(**r)
        )
    rep += [
        "",
        "- Can JTF VSD converge to first waypoint? see row A_first.",
        "- Can JTF VSD converge to middle waypoint? see row B_middle.",
        "- Can JTF VSD converge to final waypoint? see row C_final.",
    ]
    (OUT / "static_target_convergence_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
