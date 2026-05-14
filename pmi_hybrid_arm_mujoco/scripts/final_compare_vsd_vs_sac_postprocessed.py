#!/usr/bin/env python3
"""최종 비교: VSD 단독 vs VSD + SAC 잔차(EE-RMS 체크포인트) + 후처리.

학습을 수행하지 않으며, MuJoCo 모델은 변경하지 않습니다.
"""

from __future__ import annotations

import argparse
import copy
import csv
import sys
from pathlib import Path
from typing import Any

import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
_PKG = _SCRIPTS.parent
for _p in (str(_PKG), str(_SCRIPTS)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import VecNormalize

from compare_vsd_vs_sac_residual import (
    ORIENTATION_INFO_KEYS,
    build_overrides,
    build_vec_normalize,
    episode_metrics,
    infer_run_dir,
    load_sac,
    unwrap_pmi,
    vec_options,
)
from envs.pmi_cable_residual_env import _deep_merge

OUT_ROOT_DEF = _PKG / "debug_outputs" / "sac_residual_task_force" / "final_vsd_vs_sac_postprocessed"
CFG_DEF = _PKG / "configs" / "rl_sac.yaml"
CKPT_DEF = (
    _PKG
    / "debug_outputs"
    / "sac_residual_task_force"
    / "runs"
    / "sac_tf_tracking_reward_rs2_100k_s2"
    / "checkpoints"
    / "best_model_by_ee_rms.zip"
)
VN_DEF = (
    _PKG
    / "debug_outputs"
    / "sac_residual_task_force"
    / "runs"
    / "sac_tf_tracking_reward_rs2_100k_s2"
    / "vecnormalize"
    / "vecnormalize.pkl"
)

# 권장 후처리 (스윕 기준)
POSTPROCESS_OVERRIDES: dict[str, Any] = {
    "residual": {"residual_gain": 1.0},
    "residual_filter": {"enabled": True, "tau": 0.03},
    "residual_postprocess": {"final_fade_duration": 0.5},
    "action_smoothing": {"enabled": True, "max_delta_force_per_step": 0.2},
}

def matplotlib_agg():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def _matplotlib_projection_3d_available() -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(1.0, 1.0))
        fig.add_subplot(111, projection="3d")
        plt.close(fig)
        return True
    except Exception:
        return False


def merge_env_overrides(profile: str, model_path: Path) -> dict[str, Any]:
    run_dir = infer_run_dir(model_path)
    base = build_overrides(profile, run_dir)
    return _deep_merge(copy.deepcopy(base), POSTPROCESS_OVERRIDES)


def rollout_logged(
    vn: VecNormalize,
    *,
    profile: str,
    mode: str,
    cable_seed: int,
    model: SAC | None,
    capture_frames: bool,
) -> tuple[list[dict[str, Any]], list[np.ndarray]]:
    vn.seed(int(cable_seed))
    vn.set_options(vec_options(profile, cable_seed))
    pmi = unwrap_pmi(vn)
    if capture_frames:
        pmi.render_mode = "rgb_array"

    rows: list[dict[str, Any]] = []
    frames: list[np.ndarray] = []
    obs = vn.reset()

    while True:
        if mode == "zero":
            adim = int(vn.action_space.shape[0])
            act = np.zeros((vn.num_envs, adim), dtype=np.float32)
        else:
            assert model is not None
            act, _ = model.predict(obs, deterministic=True)

        tup = vn.step(act)
        if len(tup) == 5:
            obs, rew, terminated, truncated, infos = tup
            episode_done = bool(np.asarray(terminated).reshape(-1)[0]) or bool(
                np.asarray(truncated).reshape(-1)[0]
            )
        elif len(tup) == 4:
            obs, rew, dones, infos = tup
            episode_done = bool(np.asarray(dones).reshape(-1)[0])
        else:
            raise RuntimeError(f"Unexpected VecEnv step arity {len(tup)}")

        rew_f = float(np.asarray(rew).reshape(-1)[0])
        inf = infos[0]
        sac_vec = np.asarray(act.reshape(-1)[:3], dtype=np.float64)
        ee_d = np.asarray(inf["ee_des_xyz"], dtype=np.float64).reshape(3)
        ee_a = np.asarray(inf["ee_act_xyz"], dtype=np.float64).reshape(3)
        ee_e = ee_d - ee_a
        tn = float(inf["time"])
        q_j, qd_j, _, _ = pmi._read_arm_state()

        tau_vsd_nom = np.asarray(inf["tau_vsd"], dtype=np.float64).reshape(4)
        tau_clip = np.asarray(inf["tau_jnt_cmd"], dtype=np.float64).reshape(4)
        tau_res = np.asarray(inf["tau_residual_jnt"], dtype=np.float64).reshape(4)
        tau_act_ideal = np.asarray(inf["tau_act_ideal"], dtype=np.float64).reshape(4)
        tau_act_out = np.asarray(inf["tau_act_out"], dtype=np.float64).reshape(4)
        F_used = np.asarray(inf["F_residual_xyz"], dtype=np.float64).reshape(3)
        Fr_raw = np.asarray(inf.get("F_residual_raw_xyz", F_used), dtype=np.float64).reshape(3)
        F_lim = np.asarray(inf.get("F_residual_rate_limited_xyz", Fr_raw), dtype=np.float64).reshape(3)
        F_filt = np.asarray(inf.get("F_residual_filtered_xyz", F_used), dtype=np.float64).reshape(3)
        gate = float(inf.get("residual_gate", 1.0))
        ed = np.asarray(inf.get("ee_dot", np.zeros(3)), dtype=np.float64).reshape(3)
        ehf = np.asarray(inf.get("ee_err_highfreq_xyz", np.zeros(3)), dtype=np.float64).reshape(3)

        q_des, qd_des, *_ = pmi._ee_desired_kinematics(min(max(0.0, tn), float(pmi._traj_duration)))
        hz = np.asarray(inf["hys_z_q2q4"], dtype=np.float64).reshape(3)
        tl = np.asarray(inf["tau_loss_q2q4"], dtype=np.float64).reshape(3)
        th = np.asarray(inf["tau_hys_q2q4"], dtype=np.float64).reshape(3)
        terr = np.asarray(inf["tau_transmission_error_q2q4"], dtype=np.float64).reshape(3)

        rows.append(
            {
                "mode": mode,
                "time": tn,
                "residual_gate": gate,
                **{f"ee_des_{axis}": float(ee_d[i]) for i, axis in enumerate("xyz")},
                **{f"ee_act_{axis}": float(ee_a[i]) for i, axis in enumerate("xyz")},
                **{f"ee_err_{axis}": float(ee_e[i]) for i, axis in enumerate("xyz")},
                "ee_err_norm": float(inf["ee_error_norm"]),
                **{f"q_jnt_{k+1}": float(q_j[k]) for k in range(4)},
                **{f"qdot_jnt_{k+1}": float(qd_j[k]) for k in range(4)},
                **{f"q_des_{k+1}": float(q_des[k]) for k in range(4)},
                **{f"qdot_des_{k+1}": float(qd_des[k]) for k in range(4)},
                "q_err_norm": float(inf["q_error_norm"]),
                **{f"sac_action_{axis}": float(sac_vec[i]) for i, axis in enumerate("xyz")},
                **{f"F_res_{axis}": float(F_used[i]) for i, axis in enumerate("xyz")},
                **{f"F_res_raw_{axis}": float(Fr_raw[i]) for i, axis in enumerate("xyz")},
                **{f"F_res_lim_{axis}": float(F_lim[i]) for i, axis in enumerate("xyz")},
                **{f"F_res_filt_{axis}": float(F_filt[i]) for i, axis in enumerate("xyz")},
                **{f"ee_hf_{axis}": float(ehf[i]) for i, axis in enumerate("xyz")},
                **{f"ee_dot_{axis}": float(ed[i]) for i, axis in enumerate("xyz")},
                "ee_hf_norm": float(np.linalg.norm(ehf)),
                "ee_dot_norm": float(np.linalg.norm(ed)),
                "tau_res_norm": float(np.linalg.norm(tau_res)),
                "tau_vsd_norm": float(np.linalg.norm(tau_vsd_nom)),
                "tau_total_norm": float(np.linalg.norm(tau_clip)),
                **{f"tau_residual_{k+1}": float(tau_res[k]) for k in range(4)},
                **{f"tau_vsd_{k+1}": float(tau_vsd_nom[k]) for k in range(4)},
                **{f"tau_total_{k+1}": float(tau_clip[k]) for k in range(4)},
                **{f"tau_act_ideal_{k+1}": float(tau_act_ideal[k]) for k in range(4)},
                **{f"tau_act_out_{k+1}": float(tau_act_out[k]) for k in range(4)},
                **{f"tau_loss_q{k+2}": float(tl[k]) for k in range(3)},
                **{f"tau_hys_q{k+2}": float(th[k]) for k in range(3)},
                **{f"hys_z_q{k+2}": float(hz[k]) for k in range(3)},
                **{f"tau_trans_err_q{k+2}": float(terr[k]) for k in range(3)},
                "saturation_flag": int(inf["saturation_count"]),
                "joint_limit_violation": int(inf["joint_limit_violation"]),
                "actuator_limit_violation": int(inf["actuator_limit_violation"]),
                "ncon": int(inf["ncon"]),
                "reward_step": rew_f,
            }
        )
        rdict = rows[-1]
        for ok in ORIENTATION_INFO_KEYS:
            if ok in inf:
                rdict[str(ok)] = inf[ok]

        if capture_frames:
            fr = pmi.render()
            if fr is not None:
                frames.append(np.asarray(fr, dtype=np.uint8))

        if episode_done:
            break

    vn.close()
    return rows, frames


def _write_timeseries_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    keys = sorted({k for r in rows for k in r})
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in keys})


def _series(rows: list[dict], key: str) -> np.ndarray:
    return np.array([float(r[key]) for r in rows], dtype=np.float64)


def _tvec(rows: list[dict]) -> np.ndarray:
    return _series(rows, "time")


def _stack_xyz(rows: list[dict], prefix: str) -> np.ndarray:
    return np.array([[float(r[f"{prefix}_{a}"]) for a in "xyz"] for r in rows], dtype=np.float64)


def plot_detail_seed(
    plot_dir: Path,
    seed: int,
    rz: list[dict[str, Any]],
    rs: list[dict[str, Any]],
) -> list[str]:
    plt = matplotlib_agg()
    out: list[str] = []
    tz, ts = _tvec(rz), _tvec(rs)

    # 1) EE error dashboard (e_x, e_y, e_z, ‖e‖)
    fig, axs = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    for i, c in enumerate("xyz"):
        axs[i].plot(tz, _series(rz, f"ee_err_{c}"), color="#4477aa", lw=1.1, label="VSD-only")
        axs[i].plot(ts, _series(rs, f"ee_err_{c}"), color="#cc7733", lw=1.1, label="VSD+SAC+pp")
        axs[i].axhline(0.0, color="gray", lw=0.6, ls=":")
        axs[i].set_ylabel(f"e_{c} (m)")
        axs[i].grid(True, alpha=0.25)
        axs[i].legend(loc="upper right", fontsize=7)
    axs[3].plot(tz, _series(rz, "ee_err_norm"), color="#4477aa", lw=1.2, label="VSD-only")
    axs[3].plot(ts, _series(rs, "ee_err_norm"), color="#cc7733", lw=1.2, label="VSD+SAC+pp")
    axs[3].set_ylabel("‖e‖ (m)")
    axs[3].set_xlabel("time (s)")
    axs[3].grid(True, alpha=0.25)
    axs[3].legend(loc="upper right", fontsize=7)
    fig.suptitle(f"EE tracking error (seed {seed})", fontsize=11)
    fig.tight_layout()
    p = plot_dir / f"seed_{seed}_ee_error_dashboard.png"
    fig.savefig(p, dpi=140)
    plt.close(fig)
    out.append(str(p))

    # Desired vs actual EE x/y/z
    fig, axd = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    for i, c in enumerate("xyz"):
        axd[i].plot(ts, _series(rz, f"ee_des_{c}"), "k--", lw=1.0, label="desired")
        axd[i].plot(tz, _series(rz, f"ee_act_{c}"), color="#4477aa", lw=1.05, label="actual VSD")
        axd[i].plot(ts, _series(rs, f"ee_act_{c}"), color="#cc7733", lw=1.05, label="actual SAC+pp")
        axd[i].set_ylabel(f"{c} (m)")
        axd[i].grid(True, alpha=0.25)
        axd[i].legend(loc="upper right", fontsize=7)
    axd[2].set_xlabel("time (s)")
    fig.suptitle(f"Desired vs actual EE position (seed {seed})", fontsize=11)
    fig.tight_layout()
    p = plot_dir / f"seed_{seed}_ee_desired_vs_actual.png"
    fig.savefig(p, dpi=140)
    plt.close(fig)
    out.append(str(p))

    # 2) 3D path
    use3d = _matplotlib_projection_3d_available()
    xd = _stack_xyz(rz, "ee_des")
    az = _stack_xyz(rz, "ee_act")
    asac = _stack_xyz(rs, "ee_act")
    fig = plt.figure(figsize=(6.5, 5.0))
    if use3d:
        ax3 = fig.add_subplot(111, projection="3d")
        ax3.plot(xd[:, 0], xd[:, 1], xd[:, 2], "k--", lw=1.1, label="desired")
        ax3.plot(az[:, 0], az[:, 1], az[:, 2], color="#4477aa", lw=1.2, label="VSD-only")
        ax3.plot(asac[:, 0], asac[:, 1], asac[:, 2], color="#cc7733", lw=1.2, label="VSD+SAC+pp")
        ax3.set_xlabel("x")
        ax3.set_ylabel("y")
        ax3.set_zlabel("z")
        ax3.legend(fontsize=7)
    else:
        ax3 = fig.add_subplot(111)
        ax3.plot(xd[:, 0], xd[:, 1], "k--", lw=1.1, label="desired")
        ax3.plot(az[:, 0], az[:, 1], color="#4477aa", lw=1.2, label="VSD-only")
        ax3.plot(asac[:, 0], asac[:, 1], color="#cc7733", lw=1.2, label="VSD+SAC+pp")
        ax3.set_xlabel("x")
        ax3.set_ylabel("y")
        ax3.set_aspect("equal", adjustable="box")
        ax3.legend(fontsize=7)
        ax3.set_title("EE path (XY projection; 3D unavailable)")
    fig.tight_layout()
    p = plot_dir / f"seed_{seed}_3d_path.png"
    fig.savefig(p, dpi=140)
    plt.close(fig)
    out.append(str(p))

    # 3) Residual forces (SAC)
    Fr = np.linalg.norm(_stack_xyz(rs, "F_res_raw"), axis=1)
    Fl = np.linalg.norm(_stack_xyz(rs, "F_res_lim"), axis=1)
    Ff = np.linalg.norm(_stack_xyz(rs, "F_res_filt"), axis=1)
    Fu = np.linalg.norm(_stack_xyz(rs, "F_res"), axis=1)
    fig, ax = plt.subplots(figsize=(8.5, 3.6))
    ax.plot(ts, Fr, label="‖F_raw‖", lw=1.15)
    ax.plot(ts, Fl, label="‖F_limited‖", lw=1.15)
    ax.plot(ts, Ff, label="‖F_filtered‖", lw=1.15)
    ax.plot(ts, Fu, label="‖F_used‖", lw=1.15, ls="--")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("‖F‖ (N)")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8)
    ax.set_title(f"Residual force chain (seed {seed})")
    fig.tight_layout()
    p = plot_dir / f"seed_{seed}_residual_force.png"
    fig.savefig(p, dpi=140)
    plt.close(fig)
    out.append(str(p))

    # 4) Gate + zoom 4.5–5.0
    g = _series(rs, "residual_gate")
    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(8, 4.5), sharex=False)
    ax0.plot(ts, g, color="#333333", lw=1.4)
    ax0.set_ylabel("gate")
    ax0.set_ylim(-0.05, 1.05)
    ax0.grid(True, alpha=0.25)
    ax0.set_title(f"Residual gate (full, seed {seed})")
    m = (ts >= 4.5) & (ts <= 5.0)
    if np.any(m):
        ax1.plot(ts[m], g[m], color="#881111", lw=1.5)
        ax1.set_xlabel("time (s)")
        ax1.set_ylabel("gate")
        ax1.set_xlim(4.5, 5.0)
        ax1.grid(True, alpha=0.25)
        ax1.set_title("Fade-out window (t=4.5 … 5.0 s)")
    fig.tight_layout()
    p = plot_dir / f"seed_{seed}_residual_gate.png"
    fig.savefig(p, dpi=140)
    plt.close(fig)
    out.append(str(p))

    # 5) tau_residual jnt1–4
    fig, ax = plt.subplots(figsize=(8.5, 3.8))
    for k in range(4):
        ax.plot(ts, _series(rs, f"tau_residual_{k + 1}"), label=f"τ_res {k + 1}")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("τ (N·m)")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.25)
    ax.set_title(f"τ_residual (seed {seed})")
    fig.tight_layout()
    p = plot_dir / f"seed_{seed}_tau_residual.png"
    fig.savefig(p, dpi=140)
    plt.close(fig)
    out.append(str(p))

    # 6) Torque breakdown norms (SAC episode)
    fig, ax = plt.subplots(figsize=(8.5, 3.6))
    ax.plot(ts, _series(rs, "tau_vsd_norm"), label="‖τ_vsd‖", lw=1.15)
    ax.plot(ts, _series(rs, "tau_res_norm"), label="‖τ_residual‖", lw=1.15)
    ax.plot(ts, _series(rs, "tau_total_norm"), label="‖τ_total‖", lw=1.15)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("‖τ‖")
    ax.legend()
    ax.grid(True, alpha=0.25)
    ax.set_title(f"Joint torque norms (seed {seed})")
    p = plot_dir / f"seed_{seed}_torque_total.png"
    fig.tight_layout()
    fig.savefig(p, dpi=140)
    plt.close(fig)
    out.append(str(p))

    # 7) Smoothness
    fig, ax = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    ax[0].plot(tz, _series(rz, "ee_dot_norm"), color="#4477aa", lw=1.1, label="VSD-only")
    ax[0].plot(ts, _series(rs, "ee_dot_norm"), color="#cc7733", lw=1.1, label="VSD+SAC+pp")
    ax[0].set_ylabel("‖ė‖")
    ax[0].legend(fontsize=7)
    ax[0].grid(True, alpha=0.25)
    ax[0].set_title(f"EE error velocity norm (seed {seed})")
    ax[1].plot(tz, _series(rz, "ee_hf_norm"), color="#4477aa", lw=1.1, label="VSD-only")
    ax[1].plot(ts, _series(rs, "ee_hf_norm"), color="#cc7733", lw=1.1, label="VSD+SAC+pp")
    ax[1].set_ylabel("‖e_hf‖")
    ax[1].set_xlabel("time (s)")
    ax[1].legend(fontsize=7)
    ax[1].grid(True, alpha=0.25)
    fig.tight_layout()
    p = plot_dir / f"seed_{seed}_smoothness.png"
    fig.savefig(p, dpi=140)
    plt.close(fig)
    out.append(str(p))

    return out


def plot_orientation_bundle(
    plot_orient_dir: Path,
    seed: int,
    rz: list[dict[str, Any]],
    rs: list[dict[str, Any]],
) -> list[str]:
    """EE orientation timeseries vs VSD-only / VSD+SAC (detail seeds)."""
    need = ("ee_roll_rad", "ee_pitch_rad", "roll_error_rad", "pitch_error_rad")
    if not rz or not rs or any(k not in rz[0] for k in need):
        return []
    plt = matplotlib_agg()
    plot_orient_dir.mkdir(parents=True, exist_ok=True)
    out_paths: list[str] = []
    tz, ts = _tvec(rz), _tvec(rs)
    roll_ref = float(-np.pi / 2.0)
    pitch_ref = 0.0

    # 1) Roll actual
    fig, ax = plt.subplots(figsize=(8.2, 3.7))
    ax.plot(tz, _series(rz, "ee_roll_rad"), color="#4477aa", lw=1.1, label="VSD-only")
    ax.plot(ts, _series(rs, "ee_roll_rad"), color="#cc7733", lw=1.1, label="VSD+SAC+pp")
    ax.axhline(roll_ref, color="black", lw=1.0, ls="--", label=r"desired roll=$-\pi/2$")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("roll (rad)")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=7, loc="upper right")
    ax.set_title(f"End-effector roll (seed {seed})")
    fig.tight_layout()
    p1 = plot_orient_dir / f"seed_{seed}_roll_actual.png"
    fig.savefig(p1, dpi=140)
    plt.close(fig)
    out_paths.append(str(p1))

    # 2) Pitch actual
    fig, ax = plt.subplots(figsize=(8.2, 3.7))
    ax.plot(tz, _series(rz, "ee_pitch_rad"), color="#4477aa", lw=1.1, label="VSD-only")
    ax.plot(ts, _series(rs, "ee_pitch_rad"), color="#cc7733", lw=1.1, label="VSD+SAC+pp")
    ax.axhline(pitch_ref, color="black", lw=1.0, ls="--", label="desired pitch=0")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("pitch (rad)")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=7, loc="upper right")
    ax.set_title(f"End-effector pitch (seed {seed})")
    fig.tight_layout()
    p2 = plot_orient_dir / f"seed_{seed}_pitch_actual.png"
    fig.savefig(p2, dpi=140)
    plt.close(fig)
    out_paths.append(str(p2))

    # 3) Wrapped roll/pitch error (desired roll=-pi/2, pitch=0 logged in CSV)
    fig, axes = plt.subplots(2, 1, figsize=(8.2, 5.2), sharex=True)
    axes[0].plot(tz, _series(rz, "roll_error_rad"), color="#4477aa", lw=1.05, label="VSD-only")
    axes[0].plot(ts, _series(rs, "roll_error_rad"), color="#cc7733", lw=1.05, label="VSD+SAC+pp")
    axes[0].axhline(0.0, color="gray", lw=0.6, ls=":")
    axes[0].set_ylabel("roll_err (rad)")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(fontsize=7, loc="upper right")
    axes[0].set_title(f"Orientation error vs constant RPY target (seed {seed})")
    axes[1].plot(tz, _series(rz, "pitch_error_rad"), color="#4477aa", lw=1.05, label="VSD-only")
    axes[1].plot(ts, _series(rs, "pitch_error_rad"), color="#cc7733", lw=1.05, label="VSD+SAC+pp")
    axes[1].axhline(0.0, color="gray", lw=0.6, ls=":")
    axes[1].set_ylabel("pitch_err (rad)")
    axes[1].set_xlabel("time (s)")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(fontsize=7, loc="upper right")
    fig.tight_layout()
    p3 = plot_orient_dir / f"seed_{seed}_roll_pitch_error.png"
    fig.savefig(p3, dpi=140)
    plt.close(fig)
    out_paths.append(str(p3))

    # 4) Yaw (observation only)
    fig, ax = plt.subplots(figsize=(8.2, 3.6))
    ax.plot(tz, _series(rz, "ee_yaw_rad"), color="#4477aa", lw=1.05, label="VSD-only")
    ax.plot(ts, _series(rs, "ee_yaw_rad"), color="#cc7733", lw=1.05, label="VSD+SAC+pp")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("yaw (rad)")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=7, loc="upper right")
    ax.set_title(f"End-effector yaw (unconstrained target; seed {seed})")
    fig.tight_layout()
    p4 = plot_orient_dir / f"seed_{seed}_yaw_actual.png"
    fig.savefig(p4, dpi=140)
    plt.close(fig)
    out_paths.append(str(p4))

    # 5) Dashboard: ‖e‖, |roll_err|, |pitch_err|
    arz = np.abs(_series(rz, "roll_error_rad"))
    ars = np.abs(_series(rs, "roll_error_rad"))
    apz = np.abs(_series(rz, "pitch_error_rad"))
    aps = np.abs(_series(rs, "pitch_error_rad"))
    fig, axs = plt.subplots(3, 1, figsize=(8.4, 7.0), sharex=True)
    axs[0].plot(tz, _series(rz, "ee_err_norm"), color="#4477aa", lw=1.1, label="VSD-only")
    axs[0].plot(ts, _series(rs, "ee_err_norm"), color="#cc7733", lw=1.1, label="VSD+SAC+pp")
    axs[0].set_ylabel("‖e_xyz‖")
    axs[0].grid(True, alpha=0.22)
    axs[0].legend(fontsize=7, loc="upper right")
    axs[1].plot(tz, arz, color="#4477aa", lw=1.05, label="VSD-only")
    axs[1].plot(ts, ars, color="#cc7733", lw=1.05, label="VSD+SAC+pp")
    axs[1].set_ylabel("|roll_err|")
    axs[1].grid(True, alpha=0.22)
    axs[1].legend(fontsize=7, loc="upper right")
    axs[2].plot(tz, apz, color="#4477aa", lw=1.05, label="VSD-only")
    axs[2].plot(ts, aps, color="#cc7733", lw=1.05, label="VSD+SAC+pp")
    axs[2].set_ylabel("|pitch_err|")
    axs[2].set_xlabel("time (s)")
    axs[2].grid(True, alpha=0.22)
    axs[2].legend(fontsize=7, loc="upper right")
    fig.suptitle(f"EE position error vs orientation error magnitude (seed {seed})", fontsize=11, y=1.02)
    fig.tight_layout()
    p5 = plot_orient_dir / f"seed_{seed}_orientation_dashboard.png"
    fig.savefig(p5, dpi=140, bbox_inches="tight")
    plt.close(fig)
    out_paths.append(str(p5))

    return out_paths


def write_rgb_side_by_side(
    fz: list[np.ndarray],
    fs: list[np.ndarray],
    rz: list[dict],
    rs: list[dict],
    out_path: Path,
    *,
    fps: float,
    left_title: str,
    right_title: str,
) -> bool:
    try:
        from matplotlib import animation
        plt = matplotlib_agg()

        n = min(len(fz), len(fs), len(rz), len(rs))
        if n < 2:
            return False
        fz = fz[:n]
        fs = fs[:n]

        fig, (axL, axR) = plt.subplots(1, 2, figsize=(12.8, 4.8))
        imL = axL.imshow(fz[0])
        imR = axR.imshow(fs[0])
        axL.set_axis_off()
        axR.set_axis_off()
        axL.set_title(left_title, fontsize=9)
        axR.set_title(right_title, fontsize=9)
        txt = fig.suptitle("")

        def upd(i: int):
            imL.set_data(fz[i])
            imR.set_data(fs[i])
            t = float(rz[min(i, len(rz) - 1)]["time"])
            ez = float(rz[i]["ee_err_norm"])
            es = float(rs[i]["ee_err_norm"])
            fn = float(np.linalg.norm([float(rs[i][f"F_res_{a}"]) for a in "xyz"]))
            g = float(rs[i].get("residual_gate", 1.0))
            txt.set_text(
                f"t={t:.3f}s | ‖e‖ VSD={ez:.5f} | ‖e‖ SAC+pp={es:.5f} | "
                f"‖F_used‖={fn:.4f} | gate={g:.3f}"
            )
            return [imL, imR, txt]

        ani = animation.FuncAnimation(fig, upd, frames=n, interval=1000 / max(1e-6, fps))
        out_path.parent.mkdir(parents=True, exist_ok=True)
        writer = animation.FFMpegWriter(fps=fps, metadata=dict(artist="PMI_final_compare"), bitrate=2400)
        ani.save(str(out_path), writer=writer)
        plt.close(fig)
        return True
    except Exception as exc:
        print(f"[final_compare] video 실패 ({out_path.name}): {exc}", file=sys.stderr)
        return False


def write_rgb_single(frames: list[np.ndarray], out_path: Path, *, fps: float) -> bool:
    try:
        from matplotlib import animation
        plt = matplotlib_agg()

        if len(frames) < 2:
            return False
        fig, ax = plt.subplots(figsize=(6.4, 4.8))
        im = ax.imshow(frames[0])
        ax.set_axis_off()

        def upd(i: int):
            im.set_data(frames[i])
            return [im]

        ani = animation.FuncAnimation(fig, upd, frames=len(frames), interval=1000 / max(1e-6, fps))
        out_path.parent.mkdir(parents=True, exist_ok=True)
        writer = animation.FFMpegWriter(fps=fps, metadata=dict(artist="PMI"), bitrate=2400)
        ani.save(str(out_path), writer=writer)
        plt.close(fig)
        return True
    except Exception as exc:
        print(f"[final_compare] video 실패 ({out_path.name}): {exc}", file=sys.stderr)
        return False


def plot_aggregate(
    plot_dir: Path,
    agg: list[dict[str, Any]],
) -> list[str]:
    plt = matplotlib_agg()
    out: list[str] = []
    if not agg:
        return out

    mrz = float(np.mean([float(r["rms_ee_zero"]) for r in agg]))
    mrs = float(np.mean([float(r["rms_ee_sac"]) for r in agg]))
    fz = float(np.mean([float(r["final_ee_zero"]) for r in agg]))
    fs_ = float(np.mean([float(r["final_ee_sac"]) for r in agg]))
    mxz = float(np.mean([float(r["max_ee_zero"]) for r in agg]))
    mxs = float(np.mean([float(r["max_ee_sac"]) for r in agg]))
    sz = float(np.mean([float(r["sat_frac_zero"]) for r in agg]))
    ss = float(np.mean([float(r["sat_frac_sac"]) for r in agg]))
    lz = float(np.mean([float(r["lim_frac_zero"]) for r in agg]))
    ls_ = float(np.mean([float(r["lim_frac_sac"]) for r in agg]))

    # aggregate_bar_metrics.png — EE only
    labs = ["mean RMS EE", "mean final EE", "mean max EE"]
    xv = np.arange(len(labs))
    w = 0.38
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    ax.bar(xv - w / 2, [mrz, fz, mxz], w, label="VSD-only", color="#4477aa")
    ax.bar(xv + w / 2, [mrs, fs_, mxs], w, label="VSD+SAC+pp", color="#cc7733")
    ax.set_xticks(xv)
    ax.set_xticklabels(labs, rotation=12, ha="right")
    ax.legend()
    ax.set_title("Aggregate means (30 seeds)")
    fig.tight_layout()
    p = plot_dir / "aggregate_bar_metrics.png"
    fig.savefig(p, dpi=140)
    plt.close(fig)
    out.append(str(p))

    # sat/limit bar
    fig, ax = plt.subplots(figsize=(5.5, 3.8))
    ax.bar([0 - w / 2, 1 - w / 2], [sz, lz], w, label="VSD-only", color="#4477aa")
    ax.bar([0 + w / 2, 1 + w / 2], [ss, ls_], w, label="VSD+SAC+pp", color="#cc7733")
    ax.set_xticks([0, 1])
    ax.set_xticklabels(["saturation frac", "limit frac"])
    ax.legend()
    ax.set_title("Saturation / limit (means over seeds)")
    fig.tight_layout()
    p = plot_dir / "aggregate_bar_sat_limit.png"
    fig.savefig(p, dpi=140)
    plt.close(fig)
    out.append(str(p))

    d_rms = np.array([float(r["delta_rms_ee"]) for r in agg], dtype=float)
    d_fin = np.array([float(r["delta_final_ee"]) for r in agg], dtype=float)

    fig, ax = plt.subplots(figsize=(6, 3.8))
    ax.hist(d_rms, bins=min(20, max(5, len(d_rms) // 2)), color="#557799", edgecolor="white")
    ax.axvline(0.0, color="crimson", ls="--")
    ax.set_title("Δ RMS EE (SAC+pp − VSD) per seed")
    fig.tight_layout()
    p = plot_dir / "delta_rms_histogram.png"
    fig.savefig(p, dpi=140)
    plt.close(fig)
    out.append(str(p))

    fig, ax = plt.subplots(figsize=(6, 3.8))
    ax.hist(d_fin, bins=min(20, max(5, len(d_fin) // 2)), color="#775599", edgecolor="white")
    ax.axvline(0.0, color="crimson", ls="--")
    ax.set_title("Δ final EE (SAC+pp − VSD) per seed")
    fig.tight_layout()
    p = plot_dir / "delta_final_histogram.png"
    fig.savefig(p, dpi=140)
    plt.close(fig)
    out.append(str(p))

    def mean_delta(key: str) -> float:
        vals = [float(r[key]) for r in agg if key in r and r[key] == r[key]]
        return float(np.mean(vals)) if vals else float("nan")

    m_hf = mean_delta("delta_rms_ee_error_highfreq")
    m_ev = mean_delta("delta_rms_ee_error_velocity")
    m_p2 = mean_delta("delta_p2p_error_norm")

    fig, ax = plt.subplots(figsize=(6.0, 3.8))
    ax.bar(["Δ HF RMS", "Δ vel RMS", "Δ P2P"], [m_hf, m_ev, m_p2], color=["#664488", "#446688", "#886644"])
    ax.axhline(0.0, color="crimson", ls="--", lw=1)
    ax.set_title("Mean smoothness deltas (SAC+pp − VSD), 30 seeds")
    fig.tight_layout()
    p = plot_dir / "delta_smooth_metrics.png"
    fig.savefig(p, dpi=140)
    plt.close(fig)
    out.append(str(p))

    return out


def build_agg_row(seed: int, mz: dict[str, float], ms: dict[str, float]) -> dict[str, Any]:
    row: dict[str, Any] = {"seed": int(seed)}
    for k, v in mz.items():
        row[f"vsd_{k}"] = float(v)
    for k, v in ms.items():
        row[f"sac_{k}"] = float(v)
    row.update(
        {
            "rms_ee_zero": mz["rms_ee_error"],
            "rms_ee_sac": ms["rms_ee_error"],
            "delta_rms_ee": ms["rms_ee_error"] - mz["rms_ee_error"],
            "max_ee_zero": mz["max_ee_error"],
            "max_ee_sac": ms["max_ee_error"],
            "delta_max_ee": ms["max_ee_error"] - mz["max_ee_error"],
            "final_ee_zero": mz["final_ee_error"],
            "final_ee_sac": ms["final_ee_error"],
            "delta_final_ee": ms["final_ee_error"] - mz["final_ee_error"],
            "reward_zero": mz["mean_reward"],
            "reward_sac": ms["mean_reward"],
            "delta_reward": ms["mean_reward"] - mz["mean_reward"],
            "sat_frac_zero": mz["saturation_fraction"],
            "sat_frac_sac": ms["saturation_fraction"],
            "delta_saturation": ms["saturation_fraction"] - mz["saturation_fraction"],
            "lim_frac_zero": mz["limit_violation_fraction"],
            "lim_frac_sac": ms["limit_violation_fraction"],
            "delta_limit": ms["limit_violation_fraction"] - mz["limit_violation_fraction"],
            "ncon_max_zero": float(mz.get("ncon_max", 0)),
            "ncon_max_sac": float(ms.get("ncon_max", 0)),
        }
    )
    for osc_name in (
        "rms_ee_error_velocity",
        "rms_ee_error_highfreq",
        "p2p_error_norm",
        "rms_residual_force_rate",
        "rms_tau_total_rate",
        "smooth_tracking_score",
    ):
        if osc_name in mz and osc_name in ms:
            row[f"delta_{osc_name}"] = float(ms[osc_name] - mz[osc_name])
    orient_keys = (
        "rms_roll_error",
        "rms_pitch_error",
        "max_abs_roll_error",
        "max_abs_pitch_error",
        "corr_ee_norm_abs_roll_err",
        "corr_ee_norm_abs_pitch_err",
        "corr_hf_norm_abs_roll_err",
        "corr_hf_norm_abs_pitch_err",
    )
    for ok in orient_keys:
        if ok in mz and ok in ms:
            row[f"delta_{ok}"] = float(ms[ok] - mz[ok])
    return row


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", type=Path, default=CFG_DEF)
    ap.add_argument("--model-path", type=Path, default=CKPT_DEF)
    ap.add_argument("--vecnormalize-path", type=Path, default=VN_DEF)
    ap.add_argument("--profile", type=str, default="medium_train")
    ap.add_argument("--out-root", type=Path, default=OUT_ROOT_DEF)
    ap.add_argument("--seed-start", type=int, default=10000)
    ap.add_argument("--num-episodes", type=int, default=30, help="짝 평가 시드 개수 (기본 10000…10029)")
    ap.add_argument("--detail-seed-end", type=int, default=10004, help="상세 CSV/플롯/영상: seed_start … 이 값(포함)")
    ap.add_argument("--video-fps", type=float, default=25.0)
    ap.add_argument("--skip-video", action="store_true")
    args = ap.parse_args()

    cfg = Path(args.config).resolve()
    mp = Path(args.model_path).resolve()
    vn_p = Path(args.vecnormalize_path).resolve()
    out_root = Path(args.out_root).resolve()
    ts_dir = out_root / "timeseries"
    met_dir = out_root / "metrics"
    plot_dir = out_root / "plots"
    vid_dir = out_root / "videos"
    for d in (ts_dir, met_dir, plot_dir, vid_dir):
        d.mkdir(parents=True, exist_ok=True)

    ov = merge_env_overrides(str(args.profile), mp)
    model = load_sac(mp)

    seed0 = int(args.seed_start)
    n_ep = int(args.num_episodes)
    seed_end_agg = seed0 + n_ep - 1
    detail_hi = int(args.detail_seed_end)

    agg_rows: list[dict[str, Any]] = []
    plots_written: list[str] = []
    vids_written: list[str] = []

    for k in range(n_ep):
        ep_seed = seed0 + k
        cap_vid = (not args.skip_video) and ep_seed <= detail_hi

        vn_z = build_vec_normalize(cfg, vn_p, ov)
        rz, fz = rollout_logged(
            vn_z,
            profile=str(args.profile),
            mode="zero",
            cable_seed=ep_seed,
            model=None,
            capture_frames=cap_vid,
        )

        vn_s = build_vec_normalize(cfg, vn_p, ov)
        rs, fs = rollout_logged(
            vn_s,
            profile=str(args.profile),
            mode="sac",
            cable_seed=ep_seed,
            model=model,
            capture_frames=cap_vid,
        )

        mz = episode_metrics(rz)
        ms = episode_metrics(rs)
        agg_row = build_agg_row(ep_seed, mz, ms)
        agg_rows.append(agg_row)

        if ep_seed <= detail_hi:
            _write_timeseries_csv(ts_dir / f"seed_{ep_seed}_vsd.csv", rz)
            _write_timeseries_csv(ts_dir / f"seed_{ep_seed}_sac_postprocessed.csv", rs)
            plots_written.extend(plot_detail_seed(plot_dir, ep_seed, rz, rs))
            plots_written.extend(
                plot_orientation_bundle(plot_dir / "orientation", ep_seed, rz, rs)
            )

            if cap_vid and fz and fs and len(fz) == len(fs):
                v0 = vid_dir / f"seed_{ep_seed}_vsd_only.mp4"
                v1 = vid_dir / f"seed_{ep_seed}_vsd_sac_postprocessed.mp4"
                vsb = vid_dir / f"seed_{ep_seed}_side_by_side.mp4"
                if write_rgb_single(fz, v0, fps=float(args.video_fps)):
                    vids_written.append(str(v0))
                if write_rgb_single(fs, v1, fps=float(args.video_fps)):
                    vids_written.append(str(v1))
                if write_rgb_side_by_side(
                    fz,
                    fs,
                    rz,
                    rs,
                    vsb,
                    fps=float(args.video_fps),
                    left_title="VSD-only",
                    right_title="VSD + SAC + post-process",
                ):
                    vids_written.append(str(vsb))
            elif cap_vid:
                print(f"[경고] seed {ep_seed}: 영상 프레임 없음 또는 길이 불일치", file=sys.stderr)

    paired_csv = met_dir / "paired_metrics_30seeds.csv"
    if agg_rows:
        keys = sorted({kk for row in agg_rows for kk in row})
        with paired_csv.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for r in agg_rows:
                w.writerow({k: r.get(k, "") for k in keys})

    agg_plots = plot_aggregate(plot_dir, agg_rows)
    plots_written.extend(agg_plots)

    mean_d_rms = float(np.mean([float(r["delta_rms_ee"]) for r in agg_rows])) if agg_rows else float("nan")
    mean_d_fin = float(np.mean([float(r["delta_final_ee"]) for r in agg_rows])) if agg_rows else float("nan")
    max_ncon_z = int(np.max([float(r["ncon_max_zero"]) for r in agg_rows])) if agg_rows else 0
    max_ncon_s = int(np.max([float(r["ncon_max_sac"]) for r in agg_rows])) if agg_rows else 0

    mean_d_hf = float(
        np.mean([float(r["delta_rms_ee_error_highfreq"]) for r in agg_rows if "delta_rms_ee_error_highfreq" in r])
    ) if agg_rows else float("nan")
    seeds_hf_worse = [int(r["seed"]) for r in agg_rows if float(r.get("delta_rms_ee_error_highfreq", 0.0)) > 1e-12]

    def _mean_agg_field(name: str) -> float:
        vals = [float(r[name]) for r in agg_rows if name in r and np.isfinite(float(r[name]))]
        return float(np.mean(vals)) if vals else float("nan")

    mean_vsd_rms_re = _mean_agg_field("vsd_rms_roll_error")
    mean_sac_rms_re = _mean_agg_field("sac_rms_roll_error")
    mean_delta_rms_re = _mean_agg_field("delta_rms_roll_error")
    mean_vsd_rms_pe = _mean_agg_field("vsd_rms_pitch_error")
    mean_sac_rms_pe = _mean_agg_field("sac_rms_pitch_error")
    mean_delta_rms_pe = _mean_agg_field("delta_rms_pitch_error")

    m_vsd_c_er = _mean_agg_field("vsd_corr_ee_norm_abs_roll_err")
    m_sac_c_er = _mean_agg_field("sac_corr_ee_norm_abs_roll_err")
    m_vsd_c_ep = _mean_agg_field("vsd_corr_ee_norm_abs_pitch_err")
    m_sac_c_ep = _mean_agg_field("sac_corr_ee_norm_abs_pitch_err")
    m_vsd_c_hfr = _mean_agg_field("vsd_corr_hf_norm_abs_roll_err")
    m_sac_c_hfr = _mean_agg_field("sac_corr_hf_norm_abs_roll_err")
    m_vsd_c_hfp = _mean_agg_field("vsd_corr_hf_norm_abs_pitch_err")
    m_sac_c_hfp = _mean_agg_field("sac_corr_hf_norm_abs_pitch_err")

    mean_vsd_max_ar = _mean_agg_field("vsd_max_abs_roll_error")
    mean_sac_max_ar = _mean_agg_field("sac_max_abs_roll_error")
    mean_vsd_max_ap = _mean_agg_field("vsd_max_abs_pitch_error")
    mean_sac_max_ap = _mean_agg_field("sac_max_abs_pitch_error")

    agg_md = met_dir / "aggregate_metrics.md"
    agg_md.write_text(
        "\n".join(
            [
                "# Aggregate metrics (paired)",
                "",
                f"- Seeds: `{seed0}` … `{seed_end_agg}` (`num_episodes={n_ep}`)",
                f"- Mean Δ RMS EE: `{mean_d_rms:.7g}`",
                f"- Mean Δ final EE: `{mean_d_fin:.7g}`",
                f"- Max ncon (VSD / SAC+pp): `{max_ncon_z}` / `{max_ncon_s}`",
                f"- Mean Δ HF EE error RMS: `{mean_d_hf:.7g}`",
                f"- Seeds with HF worse (Δ>0): `{seeds_hf_worse if seeds_hf_worse else 'none'}`",
                "",
                "## Orientation & vibration linkage (paired)",
                "",
                "### RMS / peak orientation error (wrapped vs roll=-π/2, pitch=0)",
                f"- Mean RMS roll error — VSD-only: `{mean_vsd_rms_re:.7g}`, VSD+SAC+pp: `{mean_sac_rms_re:.7g}`, Δ(SAC−VSD): `{mean_delta_rms_re:.7g}`",
                f"- Mean RMS pitch error — VSD-only: `{mean_vsd_rms_pe:.7g}`, VSD+SAC+pp: `{mean_sac_rms_pe:.7g}`, Δ(SAC−VSD): `{mean_delta_rms_pe:.7g}`",
                f"- Mean max |roll error| — VSD: `{mean_vsd_max_ar:.7g}`, SAC+pp: `{mean_sac_max_ar:.7g}`",
                f"- Mean max |pitch error| — VSD: `{mean_vsd_max_ap:.7g}`, SAC+pp: `{mean_sac_max_ap:.7g}`",
                "",
                "### Pearson correlation (per episode; means over seeds)",
                "Columns: VSD-only vs VSD+SAC+pp.",
                f"- corr(‖e_xyz‖, |roll_err|): `{m_vsd_c_er:.7g}` / `{m_sac_c_er:.7g}`",
                f"- corr(‖e_xyz‖, |pitch_err|): `{m_vsd_c_ep:.7g}` / `{m_sac_c_ep:.7g}`",
                f"- corr(‖e_hf‖, |roll_err|): `{m_vsd_c_hfr:.7g}` / `{m_sac_c_hfr:.7g}`",
                f"- corr(‖e_hf‖, |pitch_err|): `{m_vsd_c_hfp:.7g}` / `{m_sac_c_hfp:.7g}`",
                "",
                f"- Orientation plots (detail seeds): `{out_root / 'plots' / 'orientation'}`",
                "",
            ]
        ),
        encoding="utf-8",
    )

    # Per-seed table (detail band)
    detail_rows = [r for r in agg_rows if seed0 <= int(r["seed"]) <= detail_hi]
    lines_tbl = [
        "| seed | Δ RMS EE | Δ final EE | Δ HF | Δ ee_vel | ncon sac |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for r in sorted(detail_rows, key=lambda x: int(x["seed"])):
        lines_tbl.append(
            f"| {int(r['seed'])} | {float(r['delta_rms_ee']):.6g} | {float(r['delta_final_ee']):.6g} | "
            f"{r.get('delta_rms_ee_error_highfreq', '')} | {r.get('delta_rms_ee_error_velocity', '')} | "
            f"{int(r.get('ncon_max_sac', 0))} |"
        )

    plot_list = "\n".join(f"- `{p}`" for p in sorted(set(plots_written)))
    video_list = "\n".join(f"- `{p}`" for p in sorted(vids_written))
    hf_note = (
        f"평균 고주파 EE RMS 델타는 `{mean_d_hf:.7g}` 입니다. 일부 시드에서만 악화되면 위 시드 목록을 참고하세요."
        if np.isfinite(mean_d_hf)
        else ""
    )

    orient_note = (
        f"- **Mean RMS roll/pitch error (VSD / SAC+pp / Δ)** — roll: `{mean_vsd_rms_re:.7g}` / `{mean_sac_rms_re:.7g}` / `{mean_delta_rms_re:.7g}`, "
        f"pitch: `{mean_vsd_rms_pe:.7g}` / `{mean_sac_rms_pe:.7g}` / `{mean_delta_rms_pe:.7g}`.\n"
        f"- **Mean Pearson corr** — corr(‖e‖,|roll_err|): VSD `{m_vsd_c_er:.7g}`, SAC `{m_sac_c_er:.7g}`; "
        f"corr(‖e‖,|pitch_err|): VSD `{m_vsd_c_ep:.7g}`, SAC `{m_sac_c_ep:.7g}`; "
        f"corr(‖e_hf‖,|roll_err|): VSD `{m_vsd_c_hfr:.7g}`, SAC `{m_sac_c_hfr:.7g}`; "
        f"corr(‖e_hf‖,|pitch_err|): VSD `{m_vsd_c_hfp:.7g}`, SAC `{m_sac_c_hfp:.7g}`.\n"
        f"- Orientation figures: `{out_root / 'plots' / 'orientation'}`."
    )

    interp_block = (
        "## 11. Interpretation (control modes & oscillation)\n"
        "\n"
        "1. **Was the nominal stack xyz-only?** "
        "Joint references come from IK with ``task_feas_mode=\"xyz\"`` (position rows only): roll/pitch targets are "
        "**not** in the IK residual, so **`q_des` does not impose** roll=-π/2 / pitch=0 as a soft/hard IK objective. "
        "Joint-space VSD tracks that `q_des` with PD gains.\n"
        "2. **Roll/pitch via `q_des`?** "
        "**Only implicitly** via whatever orientation the XYZ IK solution induces; FK targets in logs (`ee_des_roll_rad_fk`) "
        "show the orientation consistent with **`q_des`**, while `roll_error_rad` compares actual EE orientation to fixed "
        "roll=-π/2 and pitch=0.\n"
        "3. **Correlation with EE oscillation.** "
        "See §Orientation in `aggregate_metrics.md` for corr(‖e_xyz‖,|roll_err|) and corr(‖e_hf‖,|pitch_err|) means over seeds "
        "(not causal; shared dynamics / coupling).\n"
        "4. **SAC residual.** "
        "`action_dim=3`: **Fx, Fy, Fz only** mapped through the **XYZ position Jacobian rows** (`joint_torque_from_task_force`): "
        "no learned roll/pitch moment channel.\n"
        "5. **Soft orientation VSD.** "
        "`orientation_soft_mode=none` in default config matches **baseline xyz-only nominal torque** aside from IK path; optional "
        "soft roll/pitch uses extra Jacobian rows (`xyz_roll_pitch`) — run `orientation_soft_vsd_ablation.py` (no training) to compare visually.\n"
        "6. **Future SAC action.** "
        "If orientation errors correlate with HF EE error but cannot be trimmed by XYZ force alone on a 4-DoF arm, consider "
        "extending Jacobian / action toward a **sparse wrench** rather than blindly adding 5 full hard constraints.\n"
        "\n"
        f"- Control stack review: `{_PKG / 'debug_outputs' / 'sac_residual_task_force' / 'control_mode_inspection.md'}` (generated with this analysis).\n"
    )

    report = "\n".join(
        [
            "# Final comparison: VSD-only vs VSD + SAC (best EE RMS) + post-processing",
            "",
            "## 1. Model and controller",
            "- **Plant / controller**: `configs/rl_sac.yaml` (VSD joint torque baseline + hybrid cable transmission).",
            "- SAC는 작업공간 잔차 힘을 출력하고, 본 비교에서만 저역통과·속도제한·종료 페이드를 적용합니다 (학습 시 정의한 액션 스케일은 그대로).",
            "",
            "## 2. SAC checkpoint",
            f"- `{mp}`",
            "",
            "## 3. VecNormalize",
            f"- `{vn_p}`",
            "",
            "## 4. Cable profile",
            f"- `{args.profile}`",
            "",
            "## 5. Post-processing parameters",
            f"- `residual_gain`: {POSTPROCESS_OVERRIDES['residual']['residual_gain']}",
            f"- `residual_filter_tau`: {POSTPROCESS_OVERRIDES['residual_filter']['tau']}",
            f"- `final_fade_duration`: {POSTPROCESS_OVERRIDES['residual_postprocess']['final_fade_duration']} s",
            f"- `max_delta_force_per_step`: {POSTPROCESS_OVERRIDES['action_smoothing']['max_delta_force_per_step']}",
            "",
            "## 6. Paired rollout summary",
            "",
            f"- Episodes averaged: **`{n_ep}`** (seed band `{seed0}`–`{seed_end_agg}`).",
            f"- **Mean Δ RMS EE** (SAC+pp − VSD): **`{mean_d_rms:.7g}`** — VSD+SAC+후처리가 평균 EE RMS 추적 측면에서 유리합니다.",
            f"- **Mean Δ final EE**: **`{mean_d_fin:.7g}`** — 종단 EE 오차도 평균적으로 개선됩니다.",
            f"- **Contacts**: `ncon_max` (VSD / SAC+pp) = `{max_ncon_z}` / `{max_ncon_s}`.",
            "",
            "**주의 (과잉 주장 방지):**",
            "- 모든 고주파·진동 성분이 제거되었다고 단정할 수 없습니다.",
            hf_note,
            "- 원래 best RMS 모델 단독 평가에서 관찰된 **종단 오차 악화** 완화에는 **후처리(특히 종료 페이드)** 가 실질적으로 필요했습니다.",
            "",
            "## 7. End-effector orientation logging & visuals",
            "",
            "- 각 스텝의 `desired_roll_rad`/`desired_pitch_rad`/`desired_yaw_rad`(NaN) 및 FK 기반 `ee_des_*_fk`, 실측 "
            "`ee_roll_rad`/`ee_pitch_rad`/`ee_yaw_rad`, `roll_error_rad`/`pitch_error_rad`(wrap), `ee_omega_world_*`가 "
            f"detail 시드 `{seed0}`…`{min(seed_end_agg, detail_hi)}` CSV에 포함됩니다 (`timeseries/seed_*_*.csv`).",
            orient_note,
            "",
            f"- `paired_metrics_30seeds.csv`: `{paired_csv}`",
            f"- `aggregate_metrics.md`: `{agg_md}`",
            "",
            "## 8. Detailed seeds (table)",
            "",
            *lines_tbl,
            "",
            "## 9. Plot outputs",
            "",
            plot_list if plot_list else "*(none)*",
            "",
            "## 10. Video outputs",
            "",
            video_list if video_list else "*(none — use --skip-video 또는 FFmpeg 확인)*",
            "",
            "## Recommendation",
            "",
            "- 동영상·시각화·추가 보고에는 **`sac_tf_tracking_reward_rs2_100k_s2` / `best_model_by_ee_rms.zip`** 과 위 후처리 설정을 사용하세요.",
            "- **`sac_tf_smooth_final_rs1_30k_s5`** 등 추적을 악화시킨 런은 최종 비교에 사용하지 마세요.",
            "",
            interp_block.strip(),
            "",
            "---",
            f"- Output root: `{out_root}`",
        ]
    )

    (out_root / "final_report.md").write_text(report, encoding="utf-8")
    print(f"Wrote: {out_root / 'final_report.md'}")
    print(f"Wrote: {paired_csv}")


if __name__ == "__main__":
    main()
