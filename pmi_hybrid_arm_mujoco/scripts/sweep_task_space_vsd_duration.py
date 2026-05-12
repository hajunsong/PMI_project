#!/usr/bin/env python3
"""Case2와 동등: xyz + bias on, 시간 스케일만 변경 (웨이포인트 ``t`` 선형 확대)."""

from __future__ import annotations

import argparse
import copy
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


def _save(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(path, dpi=140)
    plt.close()


def plots_for_rollout(tag: str, out_dir: Path, r: dict) -> None:
    t = np.asarray(r["ts"])
    if len(t) < 2:
        return
    Pd, Pa = np.asarray(r["p_des_xyz"]), np.asarray(r["p_act"])
    fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    for i, lb in enumerate(["x", "y", "z"]):
        ax[i].plot(t, Pd[:, i], label="des")
        ax[i].plot(t, Pa[:, i], "--", label="act")
        ax[i].set_ylabel(lb)
        ax[i].grid(True, alpha=0.3)
        ax[i].legend()
    ax[-1].set_xlabel("t")
    plt.suptitle(tag)
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
        fig, ax2 = plt.subplots(figsize=(5, 4))
        ax2.plot(Pa[:, 0], Pa[:, 1], label="act")
        ax2.plot(Pd[:, 0], Pd[:, 1], "--", label="des")
        ax2.legend()
        ax2.grid(True, alpha=0.3)
    plt.suptitle(f"{tag} path")
    _save(out_dir / f"{tag}_path3d.png")

    plt.figure(figsize=(8, 3))
    plt.plot(t, r["ee_err_norm"], label="||p_des-p_act||")
    plt.xlabel("t")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.title(f"{tag} position error norm")
    _save(out_dir / f"{tag}_pos_err.png")

    Tj = np.asarray(r["tau_j"])
    fig, axs = plt.subplots(4, 1, figsize=(8, 7), sharex=True)
    for i in range(4):
        axs[i].plot(t, Tj[:, i])
        axs[i].set_ylabel(f"τ{i+1}")
        axs[i].grid(True, alpha=0.3)
    axs[-1].set_xlabel("t")
    plt.suptitle(f"{tag} τ_joint")
    _save(out_dir / f"{tag}_tau_jnt.png")

    m = int(r["m_dim"])
    Fm = np.asarray(r["F_task_pad"])[:, :m]
    fig, axs = plt.subplots(m, 1, figsize=(8, min(12, 2 + 2 * m)), sharex=True)
    if m == 1:
        axs = np.array([axs])
    for i in range(m):
        axs[i].plot(t, Fm[:, i])
        axs[i].set_ylabel(f"F_{i}")
        axs[i].grid(True, alpha=0.3)
    axs[-1].set_xlabel("t")
    plt.suptitle(f"{tag} F_task")
    _save(out_dir / f"{tag}_F_task.png")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=None)
    args = ap.parse_args()

    base = load_task_space_vsd_debug_yaml(args.config)
    durations = [1.0, 2.0, 3.0, 5.0]
    out_dir = PKG_ROOT / "figures" / "task_space_vsd_duration_sweep"
    out_dir.mkdir(parents=True, exist_ok=True)

    print(
        "tag | dur | rms_pos | max_pos | final_pos_err | max_tau | jl_steps | "
        "tau_sat | max_vel | max_Fnorm"
    )
    for dur in durations:
        cfg = copy.deepcopy(base)
        cfg = scale_waypoint_times_to_duration(cfg, dur)
        cfg["task_space_vsd"]["task_mode"] = "xyz"
        cfg["task_space_vsd"]["use_bias_compensation"] = True
        cfg["task_space_vsd"]["torque_application_mode"] = "joint_direct_debug"

        r = rollout_task_space_vsd(cfg)
        if r.get("error"):
            print(f"d{dur}s FAILED", r)
            continue
        tag = f"dur_{dur:g}s"
        print(
            f"{tag} | {dur} | {r['rms_pos']:.6f} | {r['max_pos']:.6f} | "
            f"{r['final_pos_err']:.6f} | {r['max_tau']:.5g} | {r['jl_activation_steps']} | "
            f"{r['torque_sat_steps']} | {r['max_velocity_norm']:.6f} | {r['max_task_force_norm']:.6g}"
        )
        plots_for_rollout(tag, out_dir, r)

    print("plots →", out_dir.resolve())


if __name__ == "__main__":
    main()
