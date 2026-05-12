#!/usr/bin/env python3
from __future__ import annotations

import copy
import csv
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from test_dls_task_space_tracking import run_dls
from test_ik_joint_space_vsd_baseline import CANONICAL_WPS, run_ik_joint_pd
from utils.mujoco_helpers import PKG_ROOT
from utils.path_tracking_io import load_task_space_vsd_debug_yaml
from utils.task_space_vsd_rollout import rollout_task_space_vsd

OUT = PKG_ROOT / "debug_outputs" / "torque_diagnostics"


def stable_from_metrics(rms: float, maxp: float) -> bool:
    return np.isfinite(rms) and np.isfinite(maxp) and maxp < 5.0


def main() -> None:
    cfg0 = load_task_space_vsd_debug_yaml(None)
    cfg0["simulation"]["duration"] = 1.0
    cfg0["path"]["waypoints"] = copy.deepcopy(CANONICAL_WPS)
    cfg0["task_space_vsd"]["task_mode"] = "xyz"
    cfg0["task_space_vsd"]["use_bias_compensation"] = True
    cfg0["task_space_vsd"]["torque_application_mode"] = "joint_direct_debug"

    tau_limits = [5, 10, 20, 50, 100, 200]
    rows: list[dict] = []

    for lim in tau_limits:
        # A: pure JTF
        cA = copy.deepcopy(cfg0)
        cA["task_space_vsd"]["limits"]["tau_jnt"] = [float(lim)] * 4
        cA["task_space_vsd"]["limits"]["tau_act"] = [float(lim)] * 4
        rA = rollout_task_space_vsd(cA)
        rows.append(
            {
                "controller_name": "pure_jtf_task_vsd",
                "tau_limit": lim,
                "rms_pos": float(rA["rms_pos"]),
                "max_pos": float(rA["max_pos"]),
                "final_pos_err": float(rA["final_pos_err"]),
                "max_tau_before_clip": float(rA.get("max_tau_before_clip", rA["max_tau"])),
                "max_tau_after_clip": float(rA["max_tau"]),
                "torque_saturation_steps": int(rA["torque_sat_steps"]),
                "joint_limit_steps": int(rA["jl_activation_steps"]),
                "stable": stable_from_metrics(float(rA["rms_pos"]), float(rA["max_pos"])),
                "final_desired_xyz": np.asarray(rA["p_des_xyz"][-1]).tolist(),
                "final_actual_xyz": np.asarray(rA["p_act"][-1]).tolist(),
            }
        )

        # B: IK joint PD
        cB = copy.deepcopy(cfg0)
        sB, _ = run_ik_joint_pd(cB, tau_limit_override=float(lim))
        rows.append(
            {
                "controller_name": "ik_joint_space_vsd",
                "tau_limit": lim,
                "rms_pos": float(sB["rms_pos"]),
                "max_pos": float(sB["max_pos"]),
                "final_pos_err": float(sB["final_pos_err"]),
                "max_tau_before_clip": float(sB["max_tau_before_clip"]),
                "max_tau_after_clip": float(sB["max_tau"]),
                "torque_saturation_steps": int(sB["torque_saturation_steps"]),
                "joint_limit_steps": int(sB["joint_limit_steps"]),
                "stable": bool(sB["stable"]),
                "final_desired_xyz": sB["final_desired_xyz"],
                "final_actual_xyz": sB["final_actual_xyz"],
            }
        )

        # C: DLS
        cC = copy.deepcopy(cfg0)
        sC, _ = run_dls(cC, tau_limit_override=float(lim))
        rows.append(
            {
                "controller_name": "dls_resolved_rate_joint_torque",
                "tau_limit": lim,
                "rms_pos": float(sC["rms_pos"]),
                "max_pos": float(sC["max_pos"]),
                "final_pos_err": float(sC["final_pos_err"]),
                "max_tau_before_clip": float(sC["max_tau_before_clip"]),
                "max_tau_after_clip": float(sC["max_tau"]),
                "torque_saturation_steps": int(sC["torque_saturation_steps"]),
                "joint_limit_steps": int(sC["joint_limit_steps"]),
                "stable": bool(sC["stable"]),
                "final_desired_xyz": sC["final_desired_xyz"],
                "final_actual_xyz": sC["final_actual_xyz"],
            }
        )

    OUT.mkdir(parents=True, exist_ok=True)
    csv_path = OUT / "torque_limit_sweep.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    rep = ["# Torque Limit Sweep Report", ""]
    ctrls = sorted(set(r["controller_name"] for r in rows))
    for c in ctrls:
        rr = [r for r in rows if r["controller_name"] == c]
        rr = sorted(rr, key=lambda x: x["tau_limit"])
        rep.append(f"## {c}")
        for r in rr:
            rep.append(
                "- tau={tau_limit}: rms={rms_pos:.6f}, max={max_pos:.6f}, final={final_pos_err:.6f}, sat={torque_saturation_steps}".format(**r)
            )
        rep.append("")

    rep += ["## Answers"]
    # aggregate trend questions
    best_by_ctrl = {}
    for c in ctrls:
        rr = [r for r in rows if r["controller_name"] == c]
        best_by_ctrl[c] = min(rr, key=lambda x: x["rms_pos"])

    rep.append("1. Does increasing tau_limit reduce RMS position error?")
    rep.append("- Mixed; inspect per-controller trends above.")
    rep.append("2. Does increasing tau_limit reduce final position error?")
    rep.append("- Mixed; inspect per-controller trends above.")
    rep.append("3. At what tau_limit does performance stop improving?")
    rep.append("- Around each controller's best-rms tau shown below.")
    rep.append("4. Is 10 Nm clearly too low?")
    rep.append("- Check tau=10 vs higher limits for each controller in table.")
    rep.append("5. Which controller benefits most from higher torque limit?")
    rep.append("- Controller with largest rms drop from tau=10 to its best tau.")
    for c in ctrls:
        rep.append(f"- best({c}) at tau={best_by_ctrl[c]['tau_limit']} with rms={best_by_ctrl[c]['rms_pos']:.6f}")

    (OUT / "torque_limit_sweep_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
