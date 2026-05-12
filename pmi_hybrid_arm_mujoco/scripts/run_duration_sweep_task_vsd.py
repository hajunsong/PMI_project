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

from utils.mujoco_helpers import PKG_ROOT
from utils.path_tracking_io import load_task_space_vsd_debug_yaml
from utils.task_space_vsd_rollout import rollout_task_space_vsd, scale_waypoint_times_to_duration

OUT = PKG_ROOT / "debug_outputs" / "task_space_vsd_next"
CANONICAL_WPS = [
    {"t": 0.0, "x": 0.25, "y": -0.20, "z": -0.10},
    {"t": 0.5, "x": 0.00, "y": -0.35, "z": -0.15},
    {"t": 1.0, "x": -0.25, "y": -0.20, "z": -0.10},
]


def _stable(r: dict) -> bool:
    if r.get("error") or r.get("nan_abort"):
        return False
    if not np.all(np.isfinite(np.asarray(r.get("p_act", [])))):
        return False
    if float(r.get("max_pos", 1e9)) > 5.0:
        return False
    return True


def _save(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(path, dpi=140)
    plt.close()


def _plot(tag: str, out_dir: Path, r: dict) -> None:
    t = np.asarray(r["ts"])
    Pd = np.asarray(r["p_des_xyz"])
    Pa = np.asarray(r["p_act"])

    fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    for i, lb in enumerate(["x", "y", "z"]):
        ax[i].plot(t, Pd[:, i], label="des")
        ax[i].plot(t, Pa[:, i], "--", label="act")
        ax[i].set_ylabel(lb)
        ax[i].grid(True, alpha=0.3)
        ax[i].legend()
    ax[-1].set_xlabel("t [s]")
    _save(out_dir / f"{tag}_ee_xyz.png")

    fig = plt.figure(figsize=(5.5, 5))
    try:
        ax3 = fig.add_subplot(111, projection="3d")
        ax3.plot(Pa[:, 0], Pa[:, 1], Pa[:, 2], label="act")
        ax3.plot(Pd[:, 0], Pd[:, 1], Pd[:, 2], "--", label="des")
        ax3.set_xlabel("x")
        ax3.set_ylabel("y")
        ax3.set_zlabel("z")
        ax3.legend()
    except Exception:
        plt.close(fig)
        fig, ax2 = plt.subplots(figsize=(5.5, 5))
        ax2.plot(Pa[:, 0], Pa[:, 1], label="act")
        ax2.plot(Pd[:, 0], Pd[:, 1], "--", label="des")
        ax2.set_xlabel("x")
        ax2.set_ylabel("y")
        ax2.grid(True, alpha=0.3)
        ax2.legend()
    _save(out_dir / f"{tag}_path3d.png")

    plt.figure(figsize=(8, 3.5))
    plt.plot(t, np.asarray(r["ee_err_norm"]))
    plt.grid(True, alpha=0.3)
    plt.xlabel("t [s]")
    plt.ylabel("||e_pos||")
    _save(out_dir / f"{tag}_pos_err.png")

    Tj = np.asarray(r["tau_j"])
    fig, axs = plt.subplots(4, 1, figsize=(8, 7), sharex=True)
    for i in range(4):
        axs[i].plot(t, Tj[:, i])
        axs[i].set_ylabel(f"tau{i+1}")
        axs[i].grid(True, alpha=0.3)
    axs[-1].set_xlabel("t [s]")
    _save(out_dir / f"{tag}_tau_jnt.png")

    m = int(r["m_dim"])
    F = np.asarray(r["F_task_pad"])[:, :m]
    fig, axs = plt.subplots(m, 1, figsize=(8, 2 + 2 * m), sharex=True)
    axs = np.atleast_1d(axs)
    for i in range(m):
        axs[i].plot(t, F[:, i])
        axs[i].set_ylabel(f"F{i}")
        axs[i].grid(True, alpha=0.3)
    axs[-1].set_xlabel("t [s]")
    _save(out_dir / f"{tag}_F_task.png")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=None)
    args = ap.parse_args()

    base = load_task_space_vsd_debug_yaml(args.config)
    durations = [1.0, 2.0, 3.0, 5.0]
    OUT.mkdir(parents=True, exist_ok=True)

    rows = []
    for T in durations:
        cfg = copy.deepcopy(base)
        cfg["path"]["waypoints"] = copy.deepcopy(CANONICAL_WPS)
        cfg = scale_waypoint_times_to_duration(cfg, T)
        cfg["task_space_vsd"]["task_mode"] = "xyz"
        cfg["task_space_vsd"]["use_bias_compensation"] = True
        cfg["task_space_vsd"]["torque_application_mode"] = "joint_direct_debug"

        r = rollout_task_space_vsd(cfg)
        if r.get("error"):
            continue

        m = int(r["m_dim"])
        max_ee_vel = float(np.max(np.linalg.norm(np.asarray(r["ydot_actual_pad"])[:, :m], axis=1)))
        max_des_vel = float(np.max(np.linalg.norm(np.asarray(r["ydot_des_pad"])[:, :m], axis=1)))

        row = {
            "duration_s": T,
            "rms_pos": float(r["rms_pos"]),
            "max_pos": float(r["max_pos"]),
            "final_pos_err": float(r["final_pos_err"]),
            "max_tau": float(r["max_tau"]),
            "joint_limit_steps": int(r["jl_activation_steps"]),
            "torque_saturation_steps": int(r["torque_sat_steps"]),
            "max_task_force_norm": float(r["max_task_force_norm"]),
            "max_ee_velocity_norm": max_ee_vel,
            "max_desired_velocity_norm": max_des_vel,
            "stable": _stable(r),
        }
        rows.append(row)

        tag = f"dur_{int(T)}s"
        _plot(tag, OUT, r)
        print(tag, row)

    csv_path = OUT / "duration_sweep.csv"
    if rows:
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

    lines = ["# Duration Sweep Report", "", "## Results"]
    for r in rows:
        lines.append(
            "- T={duration_s}s: rms={rms_pos:.6f}, max={max_pos:.6f}, final={final_pos_err:.6f}, "
            "max_tau={max_tau:.4f}, jl={joint_limit_steps}, sat={torque_saturation_steps}, stable={stable}".format(**r)
        )

    if len(rows) >= 2:
        rms_vals = np.array([x["rms_pos"] for x in rows], dtype=float)
        trend = "improves" if rms_vals[-1] < rms_vals[0] else "does_not_improve"
        lines += [
            "",
            "## Conclusion",
            f"- Does RMS position error improve when duration increases? {trend}.",
            "- If yes, trajectory speed/dynamic tracking is likely the main problem."
            if trend == "improves"
            else "- If no, path feasibility or controller structure is likely the main problem.",
        ]

    (OUT / "duration_sweep_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
