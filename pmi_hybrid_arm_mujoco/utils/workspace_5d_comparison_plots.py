"""Shared plotting for workspace 5D VSD vs SAC residual pairwise rollouts."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

from utils.workspace_5d_rl_metrics import workspace_smooth_score


def trim_pair_timeseries(z: dict[str, Any], s: dict[str, Any]) -> tuple[np.ndarray, dict[str, Any], dict[str, Any]]:
    n = int(
        np.min(
            [
                z["time"].shape[0],
                z["e_norm"].shape[0],
                z["x_des"].shape[0],
                s["time"].shape[0],
                s["e_norm"].shape[0],
                s["x_act"].shape[0],
            ]
        )
    )
    keys = (
        "time",
        "ee_err_xyz",
        "e_norm",
        "e_roll",
        "e_pitch",
        "W_residual_used",
        "tau_residual",
        "tau_vsd",
        "tau_jnt_cmd",
        "x_des",
        "x_act",
        "tau_ideal_q234",
        "tau_out_q234",
        "hys_z_q234",
        "roll_des",
        "roll_act",
        "pitch_des",
        "pitch_act",
        "yaw_act",
        "ee_hf_norm",
    )
    z2 = dict(z)
    s2 = dict(s)
    for d in (z2, s2):
        for k in keys:
            v = d.get(k)
            if isinstance(v, np.ndarray) and v.shape[0] >= n:
                d[k] = v[:n]
    t = z2["time"]
    return t, z2, s2


def smooth_dashboard(ax: Any, z: dict[str, Any], s: dict[str, Any]) -> None:
    names = ["zero", "sac"]
    scores = [workspace_smooth_score(z), workspace_smooth_score(s)]
    ax.bar(names, scores, color=["C0", "C1"])
    ax.set_ylabel("smooth score (lower better)")
    ax.set_title("Smooth score dashboard")


def save_comparison_plot_set(
    z: dict[str, Any],
    s: dict[str, Any],
    plots_dir: Path,
    *,
    file_prefix: str,
) -> None:
    """Write the standard comparison PNG set (same semantics as compare_workspace_5d_vsd_vs_sac)."""
    plots_dir = Path(plots_dir)
    plots_dir.mkdir(parents=True, exist_ok=True)
    t, z, s = trim_pair_timeseries(z, s)

    try:
        plt.style.use("seaborn-v0_8-whitegrid")
    except OSError:
        pass

    labs = ("x", "y", "z")

    fig, axes = plt.subplots(3, 1, figsize=(9, 6.5), sharex=True)
    for ax, k in zip(axes, range(3)):
        ax.plot(t, z["x_des"][:, k], label="des")
        ax.plot(t, z["x_act"][:, k], "--", alpha=0.85, label="zero")
        ax.plot(t, s["x_act"][:, k], ":", lw=1.2, label="sac")
        ax.set_ylabel(f"{labs[k]} [m]")
    axes[0].legend(fontsize=7, ncol=3)
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Desired vs actual XYZ")
    fig.savefig(plots_dir / f"{file_prefix}_xyz_desired_actual.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(9, 7.5), sharex=True)
    for i in range(3):
        axes[i].plot(t, z["ee_err_xyz"][:, i], label="zero")
        axes[i].plot(t, s["ee_err_xyz"][:, i], "--", label="sac")
        axes[i].set_ylabel(f"e_{labs[i]}")
    axes[3].plot(t, z["e_norm"], label="zero ||e||")
    axes[3].plot(t, s["e_norm"], "--", label="sac ||e||")
    axes[3].set_ylabel("||e||")
    axes[0].legend(fontsize=7)
    axes[-1].set_xlabel("time [s]")
    fig.savefig(plots_dir / f"{file_prefix}_ee_error.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    axes[0].plot(t, z["roll_des"], label="des")
    axes[0].plot(t, z["roll_act"], "--", label="zero")
    axes[0].plot(t, s["roll_act"], ":", label="sac")
    axes[0].set_ylabel("roll [rad]")
    axes[0].legend(fontsize=7)
    axes[1].plot(t, z["pitch_des"], label="des")
    axes[1].plot(t, z["pitch_act"], "--", label="zero")
    axes[1].plot(t, s["pitch_act"], ":", label="sac")
    axes[1].set_ylabel("pitch [rad]")
    axes[-1].set_xlabel("time [s]")
    fig.savefig(plots_dir / f"{file_prefix}_roll_pitch.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 2.8))
    ax.plot(t, z["e_roll"], label="e_roll zero")
    ax.plot(t, s["e_roll"], "--", label="e_roll sac")
    ax.plot(t, z["e_pitch"], label="e_pitch zero")
    ax.plot(t, s["e_pitch"], "--", label="e_pitch sac")
    ax.legend(fontsize=7)
    ax.set_xlabel("time [s]")
    fig.savefig(plots_dir / f"{file_prefix}_roll_pitch_error.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 2.8))
    ax.plot(t, z["yaw_act"], label="yaw zero")
    ax.plot(t, s["yaw_act"], "--", label="yaw sac")
    ax.set_title("yaw (free)")
    ax.legend(fontsize=7)
    fig.savefig(plots_dir / f"{file_prefix}_yaw_free.png", dpi=140)
    plt.close(fig)

    n = len(t)
    fig = plt.figure(figsize=(7, 6))
    try:
        ax3 = fig.add_subplot(111, projection="3d")
        ax3.plot(z["x_des"][:, 0], z["x_des"][:, 1], z["x_des"][:, 2], label="des", lw=2)
        ax3.plot(z["x_act"][:, 0], z["x_act"][:, 1], z["x_act"][:, 2], label="zero", lw=1.2)
        ax3.plot(s["x_act"][:, 0], s["x_act"][:, 1], s["x_act"][:, 2], label="sac", lw=1.2)
        ax3.set_xlabel("x")
        ax3.set_ylabel("y")
        ax3.set_zlabel("z")
        ax3.legend()
    except Exception:
        ax3 = fig.add_subplot(111)
        ax3.plot(z["x_des"][:, 0], z["x_des"][:, 1], label="des")
        ax3.plot(z["x_act"][:, 0], z["x_act"][:, 1], label="zero")
        ax3.plot(s["x_act"][:, 0], s["x_act"][:, 1], label="sac")
    fig.tight_layout()
    fig.savefig(plots_dir / f"{file_prefix}_3d_path.png", dpi=140)
    plt.close(fig)

    wrz = z["W_residual_used"]
    wrs = s["W_residual_used"]
    fig, axes = plt.subplots(5, 1, figsize=(9, 9), sharex=True)
    labels = ("Fx", "Fy", "Fz", "Mroll", "Mpitch")
    for i in range(5):
        axes[i].plot(t, wrz[:, i], label="zero")
        axes[i].plot(t, wrs[:, i], "--", label="sac")
        axes[i].set_ylabel(labels[i])
    axes[0].legend(fontsize=7)
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Residual wrench (used)")
    fig.savefig(plots_dir / f"{file_prefix}_residual_wrench.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(9, 8), sharex=True)
    for i in range(4):
        axes[i].plot(t, z["tau_residual"][:, i], label="zero")
        axes[i].plot(t, s["tau_residual"][:, i], "--", label="sac")
        axes[i].set_ylabel(f"tau_res {i+1}")
    axes[0].legend(fontsize=7)
    axes[-1].set_xlabel("time [s]")
    fig.savefig(plots_dir / f"{file_prefix}_residual_torque.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(9, 8), sharex=True)
    for i in range(4):
        axes[i].plot(t, z["tau_vsd"][:, i], label="vsd")
        axes[i].plot(t, s["tau_residual"][:, i], "--", label="tau_res sac")
        axes[i].plot(t, s["tau_jnt_cmd"][:, i], ":", lw=1.2, label="total sac")
        axes[i].set_ylabel(f"jnt {i+1}")
    axes[0].legend(fontsize=6, ncol=3)
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("tau_vsd vs tau_res vs tau_total (SAC run)")
    fig.savefig(plots_dir / f"{file_prefix}_tau_vsd_res_total.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(9, 6), sharex=True)
    for i in range(3):
        axes[i].plot(t, z["tau_ideal_q234"][:, i], label="ideal q234 zero")
        axes[i].plot(t, z["tau_out_q234"][:, i], "--", label="out q234 zero")
        axes[i].plot(t, s["tau_ideal_q234"][:, i], ":", label="ideal sac")
        axes[i].plot(t, s["tau_out_q234"][:, i], "-.", lw=1, label="out sac")
        axes[i].set_ylabel(f"q{i+2} act torque")
    axes[0].legend(fontsize=6, ncol=4)
    axes[-1].set_xlabel("time [s]")
    fig.savefig(plots_dir / f"{file_prefix}_cable_states.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 3))
    ax.plot(t, z.get("ee_hf_norm", np.zeros(n)), label="||ee_hf|| zero")
    ax.plot(t, s.get("ee_hf_norm", np.zeros(n)), "--", label="||ee_hf|| sac")
    ax.set_ylabel("HF EE norm (env LP)")
    ax.set_xlabel("time [s]")
    ax.legend(fontsize=7)
    fig.savefig(plots_dir / f"{file_prefix}_highfreq.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(5, 3))
    smooth_dashboard(ax, z, s)
    fig.tight_layout()
    fig.savefig(plots_dir / f"{file_prefix}_smooth_dashboard.png", dpi=140)
    plt.close(fig)
