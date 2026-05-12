"""Matplotlib plotting helpers for phase scripts."""

from pathlib import Path
from typing import Sequence

import matplotlib.pyplot as plt
import numpy as np


def savefig(path: Path | str) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def plot_joint_motion(
    t: np.ndarray,
    q_act: np.ndarray,
    q_jnt: np.ndarray,
    q_jnt_exp: np.ndarray,
    ee_xyz: np.ndarray,
    out_path: Path | str,
) -> None:
    fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axs[0].plot(t, q_act[:, 0], label="q1_act")
    axs[0].plot(t, q_act[:, 1], label="q2_act")
    axs[0].plot(t, q_act[:, 2], label="q3_act")
    axs[0].plot(t, q_act[:, 3], label="q4_act")
    axs[0].set_ylabel("q_act [rad]")
    axs[0].legend(ncol=4, fontsize=8)
    axs[0].grid(True, alpha=0.3)

    axs[1].plot(t, q_jnt[:, 0], label="jnt1 act")
    axs[1].plot(t, q_jnt[:, 1], label="jnt2 act")
    axs[1].plot(t, q_jnt[:, 2], label="jnt3 act")
    axs[1].plot(t, q_jnt[:, 3], label="jnt4 act")
    axs[1].plot(t, q_jnt_exp[:, 0], "--", label="jnt1 exp")
    axs[1].plot(t, q_jnt_exp[:, 1], "--", label="jnt2 exp")
    axs[1].plot(t, q_jnt_exp[:, 2], "--", label="jnt3 exp")
    axs[1].plot(t, q_jnt_exp[:, 3], "--", label="jnt4 exp")
    axs[1].set_ylabel("joint [rad]")
    axs[1].legend(ncol=4, fontsize=7)
    axs[1].grid(True, alpha=0.3)

    axs[2].plot(t, ee_xyz[:, 0], label="x")
    axs[2].plot(t, ee_xyz[:, 1], label="y")
    axs[2].plot(t, ee_xyz[:, 2], label="z")
    axs[2].set_ylabel("EE pos [m]")
    axs[2].set_xlabel("time [s]")
    axs[2].legend()
    axs[2].grid(True, alpha=0.3)
    savefig(out_path)


def plot_tracking_error(
    t: np.ndarray,
    err: np.ndarray,
    out_path: Path | str,
) -> None:
    plt.figure(figsize=(10, 4))
    for i in range(err.shape[1]):
        plt.plot(t, err[:, i], label=f"jnt{i+1}")
    plt.ylabel("jnt_err [rad]")
    plt.xlabel("time [s]")
    plt.legend()
    plt.grid(True, alpha=0.3)
    savefig(out_path)


def plot_phase2_bundle(
    t: np.ndarray,
    q_act_des: np.ndarray,
    q_act_est: np.ndarray,
    q_jnt_des: np.ndarray,
    q_jnt: np.ndarray,
    tau_act: np.ndarray,
    tau_jnt: np.ndarray,
    ee_xyz: np.ndarray,
    out_path: Path | str,
) -> None:
    fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    axs[0].plot(t, q_act_des[:, 0], label="q1 des")
    axs[0].plot(t, q_act_des[:, 1], label="q2 des")
    axs[0].plot(t, q_act_des[:, 2], label="q3 des")
    axs[0].plot(t, q_act_des[:, 3], label="q4 des")
    axs[0].plot(t, q_act_est[:, 0], "--", label="q1 est")
    axs[0].plot(t, q_act_est[:, 1], "--", label="q2 est")
    axs[0].plot(t, q_act_est[:, 2], "--", label="q3 est")
    axs[0].plot(t, q_act_est[:, 3], "--", label="q4 est")
    axs[0].set_ylabel("q_act [rad]")
    axs[0].grid(True, alpha=0.3)
    axs[0].legend(ncol=4, fontsize=7)

    axs[1].plot(t, q_jnt_des[:, 0], label="j1 des")
    axs[1].plot(t, q_jnt_des[:, 1], label="j2 des")
    axs[1].plot(t, q_jnt_des[:, 2], label="j3 des")
    axs[1].plot(t, q_jnt_des[:, 3], label="j4 des")
    axs[1].plot(t, q_jnt[:, 0], "--", label="j1 act")
    axs[1].plot(t, q_jnt[:, 1], "--", label="j2 act")
    axs[1].plot(t, q_jnt[:, 2], "--", label="j3 act")
    axs[1].plot(t, q_jnt[:, 3], "--", label="j4 act")
    axs[1].set_ylabel("joint [rad]")
    axs[1].grid(True, alpha=0.3)
    axs[1].legend(ncol=4, fontsize=7)

    axs[2].plot(t, tau_act[:, 0], label="tau1 act")
    axs[2].plot(t, tau_act[:, 1], label="tau2 act")
    axs[2].plot(t, tau_act[:, 2], label="tau3 act")
    axs[2].plot(t, tau_act[:, 3], label="tau4 act")
    axs[2].plot(t, tau_jnt[:, 0], "--", label="tau1 jnt")
    axs[2].plot(t, tau_jnt[:, 1], "--", label="tau2 jnt")
    axs[2].plot(t, tau_jnt[:, 2], "--", label="tau3 jnt")
    axs[2].plot(t, tau_jnt[:, 3], "--", label="tau4 jnt")
    axs[2].set_ylabel("torque [Nm]")
    axs[2].grid(True, alpha=0.3)
    axs[2].legend(ncol=4, fontsize=7)

    axs[3].plot(t, ee_xyz[:, 0], label="x")
    axs[3].plot(t, ee_xyz[:, 1], label="y")
    axs[3].plot(t, ee_xyz[:, 2], label="z")
    axs[3].set_ylabel("EE [m]")
    axs[3].set_xlabel("time [s]")
    axs[3].grid(True, alpha=0.3)
    axs[3].legend()
    savefig(out_path)


def plot_phase_a_path_tracking(
    t: np.ndarray,
    p_des: np.ndarray,
    p_act: np.ndarray,
    roll_des: np.ndarray,
    pitch_des: np.ndarray,
    yaw_act: np.ndarray,
    roll_act: np.ndarray,
    pitch_act: np.ndarray,
    q_j_des: np.ndarray,
    q_j_act: np.ndarray,
    q_a_des: np.ndarray,
    q_a_act: np.ndarray,
    ik_norm: np.ndarray,
    ee_err_norm: np.ndarray,
    base_path: Path | str,
) -> None:
    base = Path(base_path)
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i, lab in enumerate(["x", "y", "z"]):
        axs[i].plot(t, p_des[:, i], label=f"{lab} des")
        axs[i].plot(t, p_act[:, i], "--", label=f"{lab} act")
        axs[i].set_ylabel(lab + " [m]")
        axs[i].legend()
        axs[i].grid(True, alpha=0.3)
    axs[-1].set_xlabel("t [s]")
    savefig(base.with_name(base.stem + "_ee_xyz.png"))

    fig, ax_list = plt.subplots(2, 1, figsize=(10, 5), sharex=True)
    ax_list[0].plot(t, roll_des, label="roll des")
    ax_list[0].plot(t, roll_act, "--", label="roll act")
    ax_list[1].plot(t, pitch_des, label="pitch des")
    ax_list[1].plot(t, pitch_act, "--", label="pitch act")
    for a in ax_list:
        a.legend()
        a.grid(True, alpha=0.3)
    ax_list[1].set_xlabel("t [s]")
    savefig(base.with_name(base.stem + "_roll_pitch.png"))

    plt.figure(figsize=(10, 3))
    plt.plot(t, yaw_act, label="yaw actual (무제약)")
    plt.xlabel("t [s]")
    plt.ylabel("rad")
    plt.legend()
    plt.grid(True, alpha=0.3)
    savefig(base.with_name(base.stem + "_yaw_actual.png"))

    fig, axs2 = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for i in range(4):
        axs2[0].plot(t, q_j_des[:, i], label=f"j{i+1} des")
        axs2[0].plot(t, q_j_act[:, i], "--", linewidth=1.0)
    axs2[0].set_ylabel("q_joint [rad]")
    axs2[0].legend(ncol=4, fontsize=7)
    axs2[0].grid(True, alpha=0.3)
    for i in range(4):
        axs2[1].plot(t, q_a_des[:, i], label=f"q{i}_act des")
        axs2[1].plot(t, q_a_act[:, i], "--", linewidth=1.0)
    axs2[1].set_ylabel("q_act [rad]")
    axs2[1].set_xlabel("t [s]")
    axs2[1].legend(ncol=4, fontsize=7)
    axs2[1].grid(True, alpha=0.3)
    savefig(base.with_name(base.stem + "_joints_actuators.png"))

    plt.figure(figsize=(10, 3))
    plt.plot(t, ik_norm, label="IK ||e_geom||")
    plt.ylabel("geom residual norm")
    plt.xlabel("t [s]")
    plt.legend()
    plt.grid(True, alpha=0.3)
    savefig(base.with_name(base.stem + "_ik_residual.png"))

    plt.figure(figsize=(10, 3))
    plt.plot(t, ee_err_norm)
    plt.ylabel("||p_des-p_ee|| [m]")
    plt.xlabel("t [s]")
    plt.grid(True, alpha=0.3)
    savefig(base.with_name(base.stem + "_cart_err_norm.png"))

    fig = plt.figure(figsize=(6, 5))
    try:
        ax3 = fig.add_subplot(111, projection="3d")
        ax3.plot(p_act[:, 0], p_act[:, 1], p_act[:, 2], label="actual", linewidth=1.2)
        ax3.plot(p_des[:, 0], p_des[:, 1], p_des[:, 2], "--", label="desired", linewidth=1.0)
        ax3.set_xlabel("x")
        ax3.set_ylabel("y")
        ax3.set_zlabel("z")
        ax3.legend()
    except Exception:
        plt.close(fig)
        fig2, ax2 = plt.subplots(1, 1, figsize=(6, 5))
        ax2.plot(p_act[:, 0], p_act[:, 1], label="actual xy")
        ax2.plot(p_des[:, 0], p_des[:, 1], "--", label="desired xy")
        ax2.set_aspect("equal", adjustable="box")
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        fig = fig2
    savefig(base.with_name(base.stem + "_path3d.png"))


def plot_phase_b_vsd_tracking(
    t: np.ndarray,
    *,
    p_des: np.ndarray,
    p_act: np.ndarray,
    q_j_des: np.ndarray,
    q_j_act: np.ndarray,
    qdj_des: np.ndarray,
    qdj_act: np.ndarray,
    roll_des: np.ndarray,
    roll_act: np.ndarray,
    pitch_des: np.ndarray,
    pitch_act: np.ndarray,
    yaw_act: np.ndarray,
    tau_j: np.ndarray,
    tau_a: np.ndarray,
    saturated: np.ndarray,
    ee_err_norm: np.ndarray,
    base_path: Path | str,
) -> None:
    base = Path(base_path)
    fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for i, lab in enumerate(["x", "y", "z"]):
        axs[i].plot(t, p_des[:, i], label=f"{lab} des")
        axs[i].plot(t, p_act[:, i], "--", label=f"{lab} act")
        axs[i].set_ylabel(lab)
        axs[i].legend()
        axs[i].grid(True, alpha=0.3)
    axs[-1].set_xlabel("t [s]")
    savefig(base.with_name(base.stem + "_ee.png"))

    fig, axl = plt.subplots(2, 1, figsize=(10, 5), sharex=True)
    axl[0].plot(t, roll_act - roll_des, label="roll err")
    axl[1].plot(t, pitch_act - pitch_des, label="pitch err")
    for a in axl:
        a.legend()
        a.grid(True, alpha=0.3)
    axl[1].set_xlabel("t [s]")
    savefig(base.with_name(base.stem + "_rp_error.png"))

    plt.figure(figsize=(10, 2.5))
    plt.plot(t, yaw_act, label="yaw actual")
    plt.grid(True, alpha=0.3)
    plt.legend()
    savefig(base.with_name(base.stem + "_yaw.png"))

    fig, axqb = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for i in range(4):
        axqb[0].plot(t, q_j_des[:, i] - q_j_act[:, i], label=f"j{i+1}")
    axqb[0].set_ylabel("joint pos err")
    axqb[0].legend(ncol=4, fontsize=7)
    axqb[0].grid(True, alpha=0.3)
    for i in range(4):
        axqb[1].plot(t, qdj_des[:, i] - qdj_act[:, i])
    axqb[1].set_ylabel("joint vel err")
    axqb[1].set_xlabel("t [s]")
    axqb[1].grid(True, alpha=0.3)
    savefig(base.with_name(base.stem + "_joint_track.png"))

    fig, txt = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for i in range(4):
        txt[0].plot(t, tau_j[:, i], label=f"tau_j{i}")
    txt[0].set_ylabel("tau_joint")
    txt[0].legend()
    txt[0].grid(True, alpha=0.3)
    for i in range(4):
        txt[1].plot(t, tau_a[:, i], label=f"tau_a{i}")
    txt[1].set_ylabel("tau_act (= ratio*tau_joint)")
    txt[1].set_xlabel("t [s]")
    txt[1].legend()
    txt[1].grid(True, alpha=0.3)
    savefig(base.with_name(base.stem + "_torques.png"))

    plt.figure(figsize=(10, 3))
    for i in range(4):
        plt.plot(t, saturated[:, i].astype(float), label=f"sat{i+1}")
    plt.ylabel("saturated")
    plt.xlabel("t [s]")
    plt.legend(ncol=4)
    plt.grid(True, alpha=0.3)
    savefig(base.with_name(base.stem + "_saturation.png"))

    plt.figure(figsize=(10, 3))
    plt.plot(t, ee_err_norm)
    plt.ylabel("EE pos error norm")
    plt.xlabel("t [s]")
    plt.grid(True, alpha=0.3)
    savefig(base.with_name(base.stem + "_ee_err_norm.png"))

    fig = plt.figure(figsize=(6, 5))
    try:
        ax3b = fig.add_subplot(111, projection="3d")
        ax3b.plot(p_act[:, 0], p_act[:, 1], p_act[:, 2], label="act")
        ax3b.plot(p_des[:, 0], p_des[:, 1], p_des[:, 2], "--", label="des")
        ax3b.set_xlabel("x")
        ax3b.set_ylabel("y")
        ax3b.set_zlabel("z")
        ax3b.legend()
    except Exception:
        plt.close(fig)
        figb, axb = plt.subplots(figsize=(6, 5))
        axb.plot(p_act[:, 0], p_act[:, 1])
        axb.plot(p_des[:, 0], p_des[:, 1], "--")
        axb.grid(True)
        fig = figb
    savefig(base.with_name(base.stem + "_path3d.png"))


def plot_phase_b_task_space_vsd(
    t: np.ndarray,
    *,
    p_des: np.ndarray,
    p_act: np.ndarray,
    roll_des: np.ndarray,
    pitch_des: np.ndarray,
    roll_act: np.ndarray,
    pitch_act: np.ndarray,
    yaw_act: np.ndarray,
    F_task: np.ndarray,
    e_task_norm: np.ndarray,
    ed_task_norm: np.ndarray,
    tau_j: np.ndarray,
    tau_a: np.ndarray,
    q_joint: np.ndarray,
    q_actuator: np.ndarray,
    ee_pos_err_norm: np.ndarray,
    F_saturated_axes: np.ndarray,
    tau_j_saturated: np.ndarray,
    tau_a_saturated: np.ndarray,
    base_path: Path | str,
) -> None:
    """작업 공간 VSD Phase B 결과 번들 플롯."""
    base = Path(base_path)
    fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for i, lab in enumerate(["x", "y", "z"]):
        axs[i].plot(t, p_des[:, i], label=f"{lab} des")
        axs[i].plot(t, p_act[:, i], "--", label=f"{lab} act")
        axs[i].set_ylabel(lab + " [m]")
        axs[i].legend()
        axs[i].grid(True, alpha=0.3)
    axs[-1].set_xlabel("t [s]")
    savefig(base.with_name(base.stem + "_ee_xyz.png"))

    fig, axl = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    axl[0].plot(t, roll_act, label="roll act")
    axl[0].plot(t, roll_des, "--", label="roll des (-pi/2)")
    axl[1].plot(t, pitch_act, label="pitch act")
    axl[1].plot(t, pitch_des, "--", label="pitch des")
    axl[2].plot(t, yaw_act, color="gray", label="yaw actual (무제약)")
    for a in axl:
        a.legend()
        a.grid(True, alpha=0.3)
    axl[2].set_xlabel("t [s]")
    savefig(base.with_name(base.stem + "_orientation.png"))

    fig = plt.figure(figsize=(6, 5))
    try:
        ax3 = fig.add_subplot(111, projection="3d")
        ax3.plot(p_act[:, 0], p_act[:, 1], p_act[:, 2], label="actual", linewidth=1.2)
        ax3.plot(p_des[:, 0], p_des[:, 1], p_des[:, 2], "--", label="desired", linewidth=1.0)
        ax3.set_xlabel("x")
        ax3.set_ylabel("y")
        ax3.set_zlabel("z")
        ax3.legend()
    except Exception:
        plt.close(fig)
        fig3, ax2 = plt.subplots(1, 1, figsize=(6, 5))
        ax2.plot(p_act[:, 0], p_act[:, 1], label="actual xy")
        ax2.plot(p_des[:, 0], p_des[:, 1], "--", label="des xy")
        ax2.set_aspect("equal", adjustable="box")
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        fig = fig3
    savefig(base.with_name(base.stem + "_path3d.png"))

    lbls_F = ["Fx", "Fy", "Fz", "Mr", "Mp"]
    fig, af = plt.subplots(5, 1, figsize=(10, 10), sharex=True)
    for i in range(5):
        af[i].plot(t, F_task[:, i], label=lbls_F[i])
        af[i].legend()
        af[i].grid(True, alpha=0.3)
    af[-1].set_xlabel("t [s]")
    fig.suptitle("Task-space wrench")
    savefig(base.with_name(base.stem + "_F_task.png"))

    plt.figure(figsize=(10, 3))
    plt.plot(t, e_task_norm, label="||e_task||₂")
    plt.plot(t, ed_task_norm, label="||edot_task||₂")
    plt.plot(t, ee_pos_err_norm, label="||p_des-p||₂")
    plt.xlabel("t [s]")
    plt.legend()
    plt.grid(True, alpha=0.3)
    savefig(base.with_name(base.stem + "_task_err_norm.png"))

    fig, tr = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for i in range(4):
        tr[0].plot(t, tau_j[:, i], label=f"tau_j{i+1}")
    tr[0].set_ylabel("tau joint [Nm]")
    tr[0].legend(ncol=4, fontsize=7)
    tr[0].grid(True, alpha=0.3)
    for i in range(4):
        tr[1].plot(t, tau_a[:, i], label=f"tau_a{i+1}")
    tr[1].set_ylabel("tau act (= ratio*tau_joint) [Nm]")
    tr[1].set_xlabel("t [s]")
    tr[1].legend(ncol=4, fontsize=7)
    tr[1].grid(True, alpha=0.3)
    savefig(base.with_name(base.stem + "_torques.png"))

    fig, qv = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for i in range(4):
        qv[0].plot(t, q_joint[:, i], label=f"jnt{i+1}")
    qv[0].set_ylabel("q_joint [rad]")
    qv[0].legend(ncol=4, fontsize=7)
    qv[0].grid(True, alpha=0.3)
    for i in range(4):
        qv[1].plot(t, q_actuator[:, i], label=f"q{i+1}_act")
    qv[1].set_ylabel("q_act [rad]")
    qv[1].set_xlabel("t [s]")
    qv[1].legend(ncol=4, fontsize=7)
    qv[1].grid(True, alpha=0.3)
    savefig(base.with_name(base.stem + "_joints_actuators.png"))

    plt.figure(figsize=(10, 3))
    plt.plot(t, np.sum(F_saturated_axes.astype(np.float64), axis=1), label="axes F saturate")
    plt.plot(t, np.sum(tau_j_saturated.astype(np.float64), axis=1), label="tau_j saturate axes")
    plt.plot(t, np.sum(tau_a_saturated.astype(np.float64), axis=1), label="tau_a saturate axes")
    plt.ylabel("saturation count/step")
    plt.xlabel("t [s]")
    plt.legend(ncol=3, fontsize=8)
    plt.grid(True, alpha=0.3)
    savefig(base.with_name(base.stem + "_saturation.png"))


def plot_phase3_tension_loop(
    x_series: Sequence[np.ndarray],
    T_series: Sequence[np.ndarray],
    labels: Sequence[str],
    out_path: Path | str,
) -> None:
    plt.figure(figsize=(6, 5))
    for x, T, lb in zip(x_series, T_series, labels, strict=False):
        plt.plot(x, T, label=lb, linewidth=1.0)
    plt.xlabel("cable stretch x [m or rad-eq]")
    plt.ylabel("tension T [N or N-like]")
    plt.grid(True, alpha=0.3)
    plt.legend()
    savefig(out_path)
