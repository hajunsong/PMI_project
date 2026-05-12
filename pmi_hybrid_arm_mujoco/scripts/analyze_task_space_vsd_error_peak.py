#!/usr/bin/env python3
"""최대 EE 위치 오차 시점 상태를 표·CSV로 기록."""

from __future__ import annotations

import argparse
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


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=None)
    args = ap.parse_args()

    cfg = load_task_space_vsd_debug_yaml(args.config)
    cfg["task_space_vsd"]["task_mode"] = "xyz"
    cfg["task_space_vsd"]["use_bias_compensation"] = True
    cfg["task_space_vsd"]["torque_application_mode"] = "joint_direct_debug"

    r = rollout_task_space_vsd(cfg)
    out_csv = PKG_ROOT / "debug_outputs" / "task_space_vsd" / "error_peak_analysis.csv"
    out_csv.parent.mkdir(parents=True, exist_ok=True)

    if r.get("error"):
        print("rollout 실패:", r)
        return

    ee = np.asarray(r["ee_err_norm"])
    k = int(np.argmax(ee))
    m = int(r["m_dim"])
    sv = np.asarray(r["sv_singular_values"][k])
    tau = np.asarray(r["tau_j"][k])
    F = np.asarray(r["F_task_pad"][k])[:m]
    jm = np.asarray(r["joint_margin_min"][k])
    am = np.asarray(r["act_margin_min"][k])
    Fp = np.zeros(5, dtype=np.float64)
    Fflat = np.asarray(F).reshape(-1)
    Fp[: len(Fflat)] = Fflat
    jd = dict(zip(r["j_ord"], np.asarray(r["q_joint"][k])))
    ad = dict(zip(r["a_ord"], np.asarray(r["q_act"][k])))

    row = {
        "t_peak_err": float(r["ts"][k]),
        "ee_pos_err_norm": float(ee[k]),
        "des_x": float(r["p_des_xyz"][k, 0]),
        "des_y": float(r["p_des_xyz"][k, 1]),
        "des_z": float(r["p_des_xyz"][k, 2]),
        "act_x": float(r["p_act"][k, 0]),
        "act_y": float(r["p_act"][k, 1]),
        "act_z": float(r["p_act"][k, 2]),
        **{f"q_jnt_{n}": float(jd[n]) for n in r["j_ord"]},
        **{f"q_act_{n}": float(ad[n]) for n in r["a_ord"]},
        **{f"tau_jnt{i+1}": float(tau[i]) for i in range(4)},
        **{f"F_task_{i}": float(Fp[i]) for i in range(5)},
        **{f"sv_{i+1}": float(sv[i]) if i < len(sv) else float("nan") for i in range(4)},
        "joint_margin_min_rad": float(jm),
        "act_margin_min_rad": float(am),
        "peak_index": k,
        "use_desired_velocity": r.get("use_desired_velocity", True),
    }

    print("=== 최대 EE 위치 오차 시점 분석 ===")
    print("| 항목 | 값 |")
    print("|---|---|")
    for label, key in [
        ("t (peak)", "t_peak_err"),
        ("‖p_des−p_act‖₂", "ee_pos_err_norm"),
        ("p_des (x,y,z)", None),
        ("p_act (x,y,z)", None),
    ]:
        if key:
            print(f"| {label} | {row[key]} |")
    print(f"| p_des_xyz | {[row['des_x'], row['des_y'], row['des_z']]} |")
    print(f"| p_act_xyz | {[row['act_x'], row['act_y'], row['act_z']]} |")

    print(
        "| q_jnt | "
        + ", ".join("{0}={1:.5f}".format(n, row[f"q_jnt_{n}"]) for n in r["j_ord"])
        + " |",
    )
    print("| τ_jnt | " + ",".join(f"{tau[i]:.5g}" for i in range(4)) + " |")
    print("| F_task[:m] | " + ",".join(f"{float(F[i]):.5g}" for i in range(m)) + " |")
    print("| J singular values σ₁…σ₄ | " + ",".join(f"{sv[i]:.5g}" for i in range(4)) + " |")
    print("| min joint margin [rad] | %.5f |" % row["joint_margin_min_rad"])
    print("| min actuator margin [rad] | %.5f |" % row["act_margin_min_rad"])

    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        w.writeheader()
        w.writerow(row)
    print("CSV 저장:", out_csv.resolve())


if __name__ == "__main__":
    main()
