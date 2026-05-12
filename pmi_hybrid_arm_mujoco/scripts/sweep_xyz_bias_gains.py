#!/usr/bin/env python3
"""xyz + 바이어스 on 등방 K/D 그리드 (간단 스캔)."""

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


def _stable(r: dict) -> bool:
    if r.get("error"):
        return False
    if r.get("nan_abort"):
        return False
    if float(r["max_pos"]) > 5.0:
        return False
    if not np.all(np.isfinite(r["p_act"])):
        return False
    return True


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=None)
    args = ap.parse_args()

    base = load_task_space_vsd_debug_yaml(args.config)
    K_vals = [10.0, 20.0, 30.0, 50.0, 80.0, 120.0]
    D_vals = [2.0, 4.0, 8.0, 12.0, 16.0]

    csv_path = PKG_ROOT / "debug_outputs" / "task_space_vsd" / "xyz_bias_gain_sweep.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    rows: list[dict] = []

    for K in K_vals:
        for D in D_vals:
            cfg = copy.deepcopy(base)
            cfg["task_space_vsd"]["task_mode"] = "xyz"
            cfg["task_space_vsd"]["use_bias_compensation"] = True
            cfg["task_space_vsd"]["torque_application_mode"] = "joint_direct_debug"
            cfg["task_space_vsd"]["gains"]["xyz"]["K"] = [K, K, K]
            cfg["task_space_vsd"]["gains"]["xyz"]["D"] = [D, D, D]

            r = rollout_task_space_vsd(cfg)
            st = _stable(r)
            row = {
                "K_xyz_iso": K,
                "D_xyz_iso": D,
                "rms_pos": r.get("rms_pos", float("nan")),
                "max_pos": r.get("max_pos", float("nan")),
                "final_pos_err": r.get("final_pos_err", float("nan")),
                "max_tau": r.get("max_tau", float("nan")),
                "jl_activation_steps": r.get("jl_activation_steps", -1),
                "torque_sat_steps": r.get("torque_sat_steps", -1),
                "stable": st,
                "nan_abort": r.get("nan_abort", True),
            }
            rows.append(row)
            tag = "ok" if st else "BAD"
            print(f"K={K:g} D={D:g}  rms_pos={row['rms_pos']:.6f}  max_pos={row['max_pos']:.6f}  {tag}")

    if rows:
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
    print("CSV:", csv_path.resolve())


if __name__ == "__main__":
    main()
