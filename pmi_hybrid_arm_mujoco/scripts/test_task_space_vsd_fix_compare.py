#!/usr/bin/env python3
"""
작업 공간 VSD 수정 사항 비교: 바이어스·과제 모드에 따른 지표 및 플롯.

Case 1: xyz, bias off
Case 2: xyz, bias on
Case 3: xyz_pitch, bias on
Case 4: xyz_roll_pitch, bias on
"""

from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path
from typing import Any

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from utils.mujoco_helpers import PKG_ROOT
from utils.path_tracking_io import load_task_space_vsd_debug_yaml
from utils.task_space_vsd_rollout import rollout_task_space_vsd


def _tight_save(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(path, dpi=140)
    plt.close()


def save_case_plots(out_dir: Path, tag: str, r: dict[str, Any]) -> None:
    t = np.asarray(r["ts"])
    Pd = np.asarray(r["p_des_xyz"])
    Pa = np.asarray(r["p_act"])
    if len(t) < 2:
        return

    fig, ax = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for i, lb in enumerate(["x", "y", "z"]):
        ax[i].plot(t, Pd[:, i], label="des")
        ax[i].plot(t, Pa[:, i], "--", label="act")
        ax[i].set_ylabel(lb)
        ax[i].grid(True, alpha=0.3)
        ax[i].legend()
    ax[-1].set_xlabel("t [s]")
    plt.suptitle(f"{tag}: EE XYZ")
    _tight_save(out_dir / f"{tag}_ee_xyz.png")

    fig = plt.figure(figsize=(6, 5))
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
        ax2.set_aspect("equal", adjustable="box")
        ax2.legend()
        ax2.grid(True, alpha=0.3)
    plt.suptitle(f"{tag}: EE path")
    _tight_save(out_dir / f"{tag}_path3d.png")

    plt.figure(figsize=(10, 3.8))
    plt.plot(t, r["e_task_norm"], label="||e_task||")
    plt.plot(t, r["ee_err_norm"], label="||p_des - p_act||")
    plt.xlabel("t [s]")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.title(f"{tag}: task error")
    _tight_save(out_dir / f"{tag}_task_error.png")

    Tb = np.asarray(r["tau_bias"])
    Tt = np.asarray(r["tau_task"])
    Ttot = np.asarray(r["tau_j"])
    fig, axs = plt.subplots(4, 1, figsize=(9, 7), sharex=True)
    for i in range(4):
        axs[i].plot(t, Tb[:, i], label="bias")
        axs[i].set_ylabel(f"j{i+1}")
        axs[i].grid(True, alpha=0.3)
        axs[i].legend()
    axs[-1].set_xlabel("t [s]")
    plt.suptitle(f"{tag}: tau_bias joint")
    _tight_save(out_dir / f"{tag}_tau_bias.png")

    fig, axs = plt.subplots(4, 1, figsize=(9, 7), sharex=True)
    for i in range(4):
        axs[i].plot(t, Tt[:, i], color="tab:orange", label="task JᵀF")
        axs[i].grid(True, alpha=0.3)
        axs[i].legend()
    axs[-1].set_xlabel("t [s]")
    plt.suptitle(f"{tag}: tau_task joint")
    _tight_save(out_dir / f"{tag}_tau_task.png")

    fig, axs = plt.subplots(4, 1, figsize=(9, 7), sharex=True)
    tlim = np.asarray(r.get("tj_lim_arr", np.full(4, np.nan))).reshape(4)
    for i in range(4):
        axs[i].plot(t, Ttot[:, i], color="tab:green", label="total (applied)")
        if np.all(np.isfinite(tlim)):
            axs[i].axhline(tlim[i], color="gray", ls=":")
            axs[i].axhline(-tlim[i], color="gray", ls=":")
        axs[i].grid(True, alpha=0.3)
        axs[i].legend()
    axs[-1].set_xlabel("t [s]")
    plt.suptitle(f"{tag}: tau_total joint (after limits)")
    _tight_save(out_dir / f"{tag}_tau_total.png")

    Q = np.asarray(r["q_joint"])
    fig, axs = plt.subplots(4, 1, figsize=(9, 7), sharex=True)
    for i in range(4):
        axs[i].plot(t, Q[:, i], label=f"jnt{i+1}")
        axs[i].grid(True, alpha=0.3)
    axs[-1].set_xlabel("t [s]")
    plt.suptitle(f"{tag}: joint positions")
    _tight_save(out_dir / f"{tag}_joint_q.png")

    JL = np.asarray(r["jl_active"], dtype=float)
    Jc = np.asarray(r["jl_clip"])
    fig, axs = plt.subplots(4, 1, figsize=(9, 8), sharex=True)
    for i in range(4):
        axs[i].step(t, JL[:, i], where="post", label="limit active")
        axs[i].plot(t, Jc[:, i], "--", linewidth=1.0, label="|clipped| gate Δτ")
        axs[i].set_ylim(-0.1, max(1.1, float(np.max(JL)) + 0.1) if JL.size else 1.1)
        axs[i].grid(True, alpha=0.3)
        axs[i].legend()
    axs[-1].set_xlabel("t [s]")
    plt.suptitle(f"{tag}: joint limit flags / clipped torque magnitude")
    _tight_save(out_dir / f"{tag}_joint_limit.png")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=None)
    ap.add_argument(
        "--out",
        type=Path,
        default=None,
        help="플롯·요약 디렉터리 (기본 figures/task_space_vsd_fix_compare)",
    )
    args = ap.parse_args()

    cfg_base = load_task_space_vsd_debug_yaml(args.config)
    cfg_base["task_space_vsd"]["torque_application_mode"] = "joint_direct_debug"

    cases: list[tuple[str, str, bool]] = [
        ("case1_xyz_bias_off", "xyz", False),
        ("case2_xyz_bias_on", "xyz", True),
        ("case3_xyz_pitch_bias_on", "xyz_pitch", True),
        ("case4_xyz_rp_bias_on", "xyz_roll_pitch", True),
    ]

    out_dir = args.out or (PKG_ROOT / "figures" / "task_space_vsd_fix_compare")
    out_dir.mkdir(parents=True, exist_ok=True)

    summary_lines: list[str] = []

    cfg_path_disp = Path(args.config) if args.config is not None else PKG_ROOT / "configs" / "task_space_vsd_debug.yaml"
    print("=== task-space VSD fix comparison ===")
    print(f"config: {cfg_path_disp.resolve()}")
    print(f"plots → {out_dir.resolve()}\n")

    header = (
        "case | task_mode | bias | rms_pos | max_pos | rms_roll | rms_pitch | max_tau | "
        "jl_steps | tau_sat_steps | final_pos_err"
    )
    print(header)

    for tag, tm, use_bias in cases:
        cfg = copy.deepcopy(cfg_base)
        cfg["task_space_vsd"]["task_mode"] = tm  # type: ignore[assignment]
        cfg["task_space_vsd"]["use_bias_compensation"] = use_bias

        r = rollout_task_space_vsd(cfg)
        if r.get("error"):
            print(f"{tag}: FAILED ({r})")
            continue

        row = (
            f"{tag} | {r['task_mode']} | {r['use_bias']} | "
            f"{r['rms_pos']:.5g} | {r['max_pos']:.5g} | {r['rms_roll']:.5g} | {r['rms_pitch']:.5g} | "
            f"{r['max_tau']:.5g} | {r['jl_activation_steps']} | {r['torque_sat_steps']} | {r['final_pos_err']:.5g}"
        )
        print(row)
        summary_lines.append(row)
        save_case_plots(out_dir, tag, r)

    report = out_dir / "compare_summary.txt"
    report.write_text(header + "\n" + "\n".join(summary_lines), encoding="utf-8")


if __name__ == "__main__":
    main()