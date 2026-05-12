#!/usr/bin/env python3
"""use_desired_velocity true vs false 비교 (xyz + bias 기본 설정)."""

from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np

from utils.path_tracking_io import load_task_space_vsd_debug_yaml
from utils.task_space_vsd_rollout import rollout_task_space_vsd


def _ydot_stat(r: dict, key: str) -> float:
    m = int(r["m_dim"])
    Y = np.asarray(r[key])[:, :m]
    norms = np.linalg.norm(Y, axis=1)
    return float(np.max(norms))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=None)
    args = ap.parse_args()

    base = load_task_space_vsd_debug_yaml(args.config)
    for tag, ud in [("desired_vel OFF", False), ("desired_vel ON ", True)]:
        cfg = copy.deepcopy(base)
        cfg["task_space_vsd"]["task_mode"] = "xyz"
        cfg["task_space_vsd"]["use_bias_compensation"] = True
        cfg["task_space_vsd"]["torque_application_mode"] = "joint_direct_debug"
        cfg["task_space_vsd"]["use_desired_velocity"] = ud

        r = rollout_task_space_vsd(cfg)
        if r.get("error"):
            print(tag, r)
            continue

        ymax_d = _ydot_stat(r, "ydot_des_pad")
        ymax_a = _ydot_stat(r, "ydot_actual_pad")
        emax = float(np.max(r["edot_task_norm"]))
        print(
            f"[{tag}] rms_pos={r['rms_pos']:.6f} max_pos={r['max_pos']:.6f} "
            f"max ‖yd_des‖₂={ymax_d:.6g} max ‖yd_actual‖₂={ymax_a:.6g} max ‖ė‖₂={emax:.6g}"
        )


if __name__ == "__main__":
    main()
