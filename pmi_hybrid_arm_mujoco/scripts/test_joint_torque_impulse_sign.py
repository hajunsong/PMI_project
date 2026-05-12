#!/usr/bin/env python3
from __future__ import annotations

import csv
import sys
from pathlib import Path

import mujoco as mj
import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from kinematics.forward_kinematics import fk_ee_rp
from utils.mujoco_helpers import PKG_ROOT, VSD_DEBUG_MODEL_PATH, apply_ideal_qjnt_equals_ratio_qact, joint_id, load_mjmodel
from utils.path_tracking_io import load_task_space_vsd_debug_yaml, ordered_transmission_arrays

OUT = PKG_ROOT / "debug_outputs" / "torque_diagnostics"


def reset_initial(model, data, a_ord, j_ord, ratios, qa0):
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    for i, n in enumerate(a_ord):
        data.qpos[int(model.jnt_qposadr[joint_id(model, n)])] = qa0[i]
    apply_ideal_qjnt_equals_ratio_qact(model, data, list(j_ord), list(a_ord), ratios)
    mj.mj_forward(model, data)


def main() -> None:
    cfg = load_task_space_vsd_debug_yaml(None)
    model = load_mjmodel(VSD_DEBUG_MODEL_PATH, strip_position_actuators=True)
    data = mj.MjData(model)
    scratch = mj.MjData(model)

    dt = float(cfg["simulation"]["dt"])
    steps = int(round(0.05 / dt))
    j_ord, a_ord, ratios = ordered_transmission_arrays(cfg)
    ratios = np.asarray(ratios, dtype=float)
    qa0 = np.asarray(cfg["path"]["initial_actuator_rad"], dtype=float)
    dof_j = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in j_ord], dtype=int)

    rows = []
    for ji, jn in enumerate(j_ord):
        for sign in [+1.0, -1.0]:
            reset_initial(model, data, a_ord, j_ord, ratios, qa0)
            q0 = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in j_ord], dtype=float)
            p0, *_ = fk_ee_rp(model, scratch, q0, list(j_ord))
            p0 = np.asarray(p0, dtype=float)

            for _ in range(steps):
                data.qfrc_applied[:] = 0.0
                data.qfrc_applied[dof_j[ji]] = sign * 1.0
                mj.mj_step(model, data)

            q1 = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in j_ord], dtype=float)
            p1, *_ = fk_ee_rp(model, scratch, q1, list(j_ord))
            p1 = np.asarray(p1, dtype=float)
            dq = q1 - q0
            dp = p1 - p0
            rows.append(
                {
                    "joint_name": jn,
                    "torque_sign": sign,
                    "delta_q": dq.tolist(),
                    "delta_EE_xyz": dp.tolist(),
                    "delta_q_target_joint": float(dq[ji]),
                    "sign_matches_expected": bool(np.sign(dq[ji]) == np.sign(sign) or abs(dq[ji]) < 1e-12),
                }
            )

    OUT.mkdir(parents=True, exist_ok=True)
    with open(OUT / "joint_torque_impulse_sign.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    bad = [r for r in rows if not r["sign_matches_expected"]]
    rep = [
        "# Joint Torque Impulse Sign Report",
        "",
        f"mismatched sign cases: {len(bad)}",
        "See CSV for per-joint +/-1Nm delta_q and delta_EE_xyz.",
    ]
    (OUT / "joint_torque_impulse_sign_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
