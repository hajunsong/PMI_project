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

import numpy as np

from utils.mujoco_helpers import PKG_ROOT
from utils.path_tracking_io import load_task_space_vsd_debug_yaml
from utils.task_space_vsd_rollout import rollout_task_space_vsd

OUT = PKG_ROOT / "debug_outputs" / "task_space_vsd_next"


def _stable(r: dict) -> bool:
    return not r.get("error") and not r.get("nan_abort") and np.isfinite(float(r.get("max_pos", np.inf))) and float(r.get("max_pos", np.inf)) < 5.0


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=None)
    args = ap.parse_args()

    base = load_task_space_vsd_debug_yaml(args.config)
    K_vals = [10, 20, 30, 50, 80, 120]
    D_vals = [2, 4, 8, 12, 16, 24]

    rows = []
    for K in K_vals:
        for D in D_vals:
            cfg = copy.deepcopy(base)
            cfg["task_space_vsd"]["task_mode"] = "xyz"
            cfg["task_space_vsd"]["use_bias_compensation"] = True
            cfg["task_space_vsd"]["torque_application_mode"] = "joint_direct_debug"
            cfg["task_space_vsd"]["gains"]["xyz"]["K"] = [float(K)] * 3
            cfg["task_space_vsd"]["gains"]["xyz"]["D"] = [float(D)] * 3

            r = rollout_task_space_vsd(cfg)
            row = {
                "K_xyz": K,
                "D_xyz": D,
                "rms_pos": float(r.get("rms_pos", np.nan)),
                "max_pos": float(r.get("max_pos", np.nan)),
                "final_pos_err": float(r.get("final_pos_err", np.nan)),
                "max_tau": float(r.get("max_tau", np.nan)),
                "joint_limit_steps": int(r.get("jl_activation_steps", -1)),
                "torque_saturation_steps": int(r.get("torque_sat_steps", -1)),
                "stable": _stable(r),
            }
            rows.append(row)
            print(row)

    OUT.mkdir(parents=True, exist_ok=True)
    csv_path = OUT / "xyz_bias_gain_sweep.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    stable_rows = [r for r in rows if r["stable"]]
    best_rms = min(stable_rows, key=lambda x: x["rms_pos"]) if stable_rows else None
    best_final = min(stable_rows, key=lambda x: x["final_pos_err"]) if stable_rows else None
    unstable = [r for r in rows if not r["stable"]]

    rep = [
        "# XYZ + Bias Gain Sweep Report",
        "",
        f"- best gain by rms_pos: {best_rms}" if best_rms else "- best gain by rms_pos: none",
        f"- best gain by final_pos_err: {best_final}" if best_final else "- best gain by final_pos_err: none",
        f"- unstable gains: {len(unstable)} cases",
    ]

    # crude trend checks
    if stable_rows:
        byK = {}
        for r in stable_rows:
            byK.setdefault(r["K_xyz"], []).append(r["rms_pos"])
        k_sorted = sorted(byK)
        k_mean = [float(np.mean(byK[k])) for k in k_sorted]
        helps = k_mean[-1] < k_mean[0]
        rep.append(f"- whether increasing K helps: {helps} (mean rms at K={k_sorted[0]}: {k_mean[0]:.5f}, K={k_sorted[-1]}: {k_mean[-1]:.5f})")

        byD = {}
        for r in stable_rows:
            byD.setdefault(r["D_xyz"], []).append(r["rms_pos"])
        d_sorted = sorted(byD)
        d_mean = [float(np.mean(byD[d])) for d in d_sorted]
        rep.append(
            f"- damping too low/high hint: min mean-rms at D={d_sorted[int(np.argmin(d_mean))]} (edge minima may indicate too low/high damping regime)."
        )

    (OUT / "xyz_bias_gain_sweep_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
