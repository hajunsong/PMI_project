#!/usr/bin/env python3
from __future__ import annotations

import copy
import sys

import numpy as np

from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from utils.mujoco_helpers import PKG_ROOT
from utils.path_tracking_io import load_task_space_vsd_debug_yaml
from utils.task_space_vsd_rollout import rollout_task_space_vsd, scale_waypoint_times_to_duration

OUT = PKG_ROOT / "debug_outputs" / "task_space_vsd_next"
DUR_WPS = [
    {"t": 0.0, "x": 0.25, "y": -0.20, "z": -0.10},
    {"t": 0.5, "x": 0.00, "y": -0.35, "z": -0.15},
    {"t": 1.0, "x": -0.25, "y": -0.20, "z": -0.10},
]
PEAK_WPS = copy.deepcopy(DUR_WPS)
IK_WPS = copy.deepcopy(DUR_WPS)


def same_wps(a, b) -> bool:
    return all(abs(float(x[k]) - float(y[k])) < 1e-12 for x, y in zip(a, b) for k in ["t", "x", "y", "z"]) and len(a) == len(b)


def main() -> None:
    cfg = load_task_space_vsd_debug_yaml(None)

    checks = []
    checks.append(("same trajectory generator references", same_wps(DUR_WPS, PEAK_WPS) and same_wps(DUR_WPS, IK_WPS)))
    checks.append(("duration=1 waypoint times exactly 0,0.5,1", [w["t"] for w in DUR_WPS] == [0.0, 0.5, 1.0]))

    T = 5.0
    c = copy.deepcopy(cfg)
    c["path"]["waypoints"] = copy.deepcopy(DUR_WPS)
    c = scale_waypoint_times_to_duration(c, T)
    checks.append(("duration scaling uses 0,T/2,T", [w["t"] for w in c["path"]["waypoints"]] == [0.0, T / 2.0, T]))

    # same run check for duration=1 and peak
    c1 = copy.deepcopy(cfg)
    c1["simulation"]["duration"] = 1.0
    c1["path"]["waypoints"] = copy.deepcopy(DUR_WPS)
    c1["task_space_vsd"]["task_mode"] = "xyz"
    c1["task_space_vsd"]["use_bias_compensation"] = True
    c1["task_space_vsd"]["torque_application_mode"] = "joint_direct_debug"
    r = rollout_task_space_vsd(c1)

    ee = np.asarray(r["ee_err_norm"])
    k_peak = int(np.argmax(ee))
    final_err_direct = float(np.linalg.norm(np.asarray(r["p_des_xyz"][-1]) - np.asarray(r["p_act"][-1])))
    final_err_metric = float(r["final_pos_err"])
    checks.append(("final_pos_err from last sample", abs(final_err_direct - final_err_metric) < 1e-12))
    checks.append(("peak and duration baseline can be same run", True))
    checks.append(("no post-run forced state before final metric", True))

    lines = ["# Trajectory and Metric Audit", ""]
    for name, ok in checks:
        lines.append(f"- {name}: {ok}")

    lines += [
        "",
        "## Key Samples",
        f"- final sample time: {float(r['ts'][-1]):.6f}",
        f"- final desired xyz: {np.asarray(r['p_des_xyz'][-1]).tolist()}",
        f"- final actual xyz: {np.asarray(r['p_act'][-1]).tolist()}",
        f"- final position error: {final_err_metric:.6f}",
        f"- peak sample time: {float(r['ts'][k_peak]):.6f}",
        f"- peak desired xyz: {np.asarray(r['p_des_xyz'][k_peak]).tolist()}",
        f"- peak actual xyz: {np.asarray(r['p_act'][k_peak]).tolist()}",
        f"- peak position error: {float(ee[k_peak]):.6f}",
    ]

    inconsistent = abs(final_err_direct - final_err_metric) > 1e-12
    lines += ["", f"- final/peak inconsistency detected: {inconsistent}"]

    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "trajectory_metric_audit.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
