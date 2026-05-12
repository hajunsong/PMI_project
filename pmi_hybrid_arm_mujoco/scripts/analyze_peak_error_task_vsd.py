#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import copy
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np

from utils.mujoco_helpers import PKG_ROOT
from utils.path_tracking_io import load_task_space_vsd_debug_yaml
from utils.task_space_vsd_rollout import rollout_task_space_vsd

OUT = PKG_ROOT / "debug_outputs" / "task_space_vsd_next"
CANONICAL_WPS = [
    {"t": 0.0, "x": 0.25, "y": -0.20, "z": -0.10},
    {"t": 0.5, "x": 0.00, "y": -0.35, "z": -0.15},
    {"t": 1.0, "x": -0.25, "y": -0.20, "z": -0.10},
]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=None)
    args = ap.parse_args()

    cfg = copy.deepcopy(load_task_space_vsd_debug_yaml(args.config))
    cfg["simulation"]["duration"] = 1.0
    cfg["path"]["waypoints"] = copy.deepcopy(CANONICAL_WPS)
    cfg["task_space_vsd"]["task_mode"] = "xyz"
    cfg["task_space_vsd"]["use_bias_compensation"] = True
    cfg["task_space_vsd"]["torque_application_mode"] = "joint_direct_debug"

    r = rollout_task_space_vsd(cfg)
    if r.get("error"):
        raise RuntimeError(str(r))

    k = int(np.argmax(np.asarray(r["ee_err_norm"])))
    t_peak = float(r["ts"][k])
    p_des = np.asarray(r["p_des_xyz"][k])
    p_act = np.asarray(r["p_act"][k])
    e = p_des - p_act
    J = np.asarray(r["J_task_log"][k])
    sv = np.asarray(r["sv_singular_values"][k])

    row = {
        "time_peak": t_peak,
        "desired_xyz": p_des.tolist(),
        "actual_xyz": p_act.tolist(),
        "position_error_vector": e.tolist(),
        "position_error_norm": float(np.linalg.norm(e)),
        "q_jnt": np.asarray(r["q_joint"][k]).tolist(),
        "qdot_jnt": np.asarray(r["qdot_joint"][k]).tolist(),
        "q_act": np.asarray(r["q_act"][k]).tolist(),
        "tau_bias_jnt": np.asarray(r["tau_bias"][k]).tolist(),
        "tau_task_jnt": np.asarray(r["tau_task"][k]).tolist(),
        "tau_total_jnt": np.asarray(r["tau_j"][k]).tolist(),
        "F_task": np.asarray(r["F_task_pad"][k]).tolist(),
        "J_task": J.tolist(),
        "J_task_singular_values": sv.tolist(),
        "joint_limit_margins_min": float(r["joint_margin_min"][k]),
        "actuator_limit_margins_min": float(r["act_margin_min"][k]),
        "desired_velocity": np.asarray(r["ydot_des_pad"][k]).tolist(),
        "actual_task_velocity": np.asarray(r["ydot_actual_pad"][k]).tolist(),
        "edot_task": np.asarray(r["edot_task_pad"][k]).tolist(),
    }

    OUT.mkdir(parents=True, exist_ok=True)
    csv_path = OUT / "peak_error_analysis.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["key", "value"])
        for k0, v0 in row.items():
            w.writerow([k0, v0])

    cond = float(sv[0] / sv[-1]) if sv[-1] > 1e-12 else float("inf")
    v_des = np.asarray(r["ydot_des_pad"][k])[:3]
    v_act = np.asarray(r["ydot_actual_pad"][k])[:3]
    denom = float(np.linalg.norm(v_des) * np.linalg.norm(v_act))
    cosang = float(np.dot(v_des, v_act) / denom) if denom > 1e-12 else float("nan")

    rep = [
        "# Peak Error Report",
        "",
        f"- Does max error occur near middle waypoint? {abs(t_peak - 0.5) <= 0.15} (t_peak={t_peak:.6f})",
        f"- Near joint/actuator limit? joint_min_margin={row['joint_limit_margins_min']:.6f}, actuator_min_margin={row['actuator_limit_margins_min']:.6f}",
        f"- Jacobian poorly conditioned? condition={cond:.4f}, singular_values={sv.tolist()}",
        f"- Enough force/torque? max|F_task|={float(np.max(np.abs(np.asarray(row['F_task'])))):.5f}, max|tau_total|={float(np.max(np.abs(np.asarray(row['tau_total_jnt'])))):.5f}",
        f"- Is actual velocity opposite desired? cosine(des,act)={cosang:.5f}",
    ]
    (OUT / "peak_error_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
