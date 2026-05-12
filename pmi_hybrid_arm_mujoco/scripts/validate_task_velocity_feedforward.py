#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import csv
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from kinematics.trajectory import CartesianQuinticPath, WaypointXYZ
from utils.mujoco_helpers import PKG_ROOT
from utils.path_tracking_io import load_task_space_vsd_debug_yaml
from utils.task_space_vsd_rollout import rollout_task_space_vsd

OUT = PKG_ROOT / "debug_outputs" / "task_space_vsd_next"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=None)
    args = ap.parse_args()

    cfg = load_task_space_vsd_debug_yaml(args.config)
    OUT.mkdir(parents=True, exist_ok=True)

    # quintic trajectory validation
    wps = [WaypointXYZ(float(w["t"]), float(w["x"]), float(w["y"]), float(w["z"])) for w in cfg["path"]["waypoints"]]
    spl = CartesianQuinticPath(wps)
    T = float(cfg["simulation"]["duration"])
    ts = np.linspace(0.0, T, 301)
    p = np.zeros((len(ts), 3))
    pd = np.zeros((len(ts), 3))
    pdd = np.zeros((len(ts), 3))
    for i, t in enumerate(ts):
        pp, vv, aa = spl.sample(float(t))
        p[i], pd[i], pdd[i] = np.asarray(pp), np.asarray(vv), np.asarray(aa)

    fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for k, nm in enumerate(["x", "y", "z"]):
        ax[k].plot(ts, p[:, k], label="p_des")
        ax[k].plot(ts, pd[:, k], label="pdot_des")
        ax[k].plot(ts, pdd[:, k], label="pddot_des")
        ax[k].set_ylabel(nm)
        ax[k].grid(True, alpha=0.3)
        ax[k].legend()
    ax[-1].set_xlabel("t [s]")
    plt.tight_layout()
    plt.savefig(OUT / "quintic_desired_kinematics.png", dpi=140)
    plt.close()

    rows = []
    for name, ud in [("A_no_ff", False), ("B_with_ff", True)]:
        c = copy.deepcopy(cfg)
        c["task_space_vsd"]["task_mode"] = "xyz"
        c["task_space_vsd"]["use_bias_compensation"] = True
        c["task_space_vsd"]["torque_application_mode"] = "joint_direct_debug"
        c["task_space_vsd"]["use_desired_velocity"] = ud
        r = rollout_task_space_vsd(c)
        row = {
            "case": name,
            "use_desired_velocity": ud,
            "edot_definition": "-ydot_actual" if not ud else "ydot_des-ydot_actual",
            "rms_pos": float(r["rms_pos"]),
            "max_pos": float(r["max_pos"]),
            "final_pos_err": float(r["final_pos_err"]),
            "max_tau": float(r["max_tau"]),
            "max_position_error_norm": float(np.max(np.asarray(r["ee_err_norm"]))),
        }
        rows.append(row)

    csv_path = OUT / "velocity_feedforward_compare.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    rep = ["# Desired Velocity Feedforward Report", ""]
    for r in rows:
        rep.append(
            "- {case}: rms={rms_pos:.6f}, max={max_pos:.6f}, final={final_pos_err:.6f}, max_tau={max_tau:.4f}, max_err={max_position_error_norm:.6f}".format(**r)
        )
    better = "B_with_ff" if rows[1]["rms_pos"] < rows[0]["rms_pos"] else "A_no_ff"
    rep += ["", f"- Better RMS case: {better}"]
    (OUT / "velocity_feedforward_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
