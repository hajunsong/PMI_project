#!/usr/bin/env python3
"""Plot comparison: zero residual vs SAC (workspace 5D VSD + cable)."""

from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path
from typing import Any

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from envs.pmi_workspace_5d_residual_env import PMIWorkspace5DResidualEnv
from utils.mujoco_helpers import load_yaml
from utils.workspace_5d_rl_metrics import rollout_one_episode_metrics, workspace_smooth_score


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=_ROOT / "configs" / "rl_workspace_5d_sac.yaml")
    ap.add_argument("--model-path", type=Path, required=True)
    ap.add_argument("--vecnormalize-path", type=Path, default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument(
        "--curriculum-stage",
        type=str,
        default=None,
        choices=("deterministic", "mild", "medium_v2", "medium_train", "stress"),
        help="Overrides curriculum.stage (default: YAML). Match evaluation/training stage.",
    )
    ap.add_argument("--learning-rate", type=float, default=None, help="Override sac.learning_rate (snapshot parity)")
    ap.add_argument("--residual-force-scale", type=float, default=None, help="Override residual.residual_force_scale")
    ap.add_argument("--residual-moment-scale", type=float, default=None, help="Override residual.residual_moment_scale")
    ap.add_argument(
        "--out-dir",
        type=Path,
        default=_ROOT / "debug_outputs" / "workspace_5d_residual_rl" / "comparison",
    )
    return ap.parse_args()


def _effective_cable_randomize(cfg_eval: dict[str, Any]) -> tuple[bool, str]:
    env_cfg = cfg_eval.get("env") or {}
    randomize = bool(env_cfg.get("randomize_cable", False))
    profile = str(env_cfg.get("randomization_profile", "medium_train"))
    cur = cfg_eval.get("curriculum") or {}
    stage = str(cur.get("stage", "deterministic"))
    stages = cur.get("stages") or {}
    if stage in stages and isinstance(stages[stage], dict):
        st = stages[stage]
        if "randomize_cable" in st:
            randomize = bool(st["randomize_cable"])
        if "randomization_profile" in st:
            profile = str(st["randomization_profile"])
    return randomize, profile


def smooth_dashboard(ax: Any, z: dict[str, Any], s: dict[str, Any]) -> None:
    names = ["zero", "sac"]
    scores = [workspace_smooth_score(z), workspace_smooth_score(s)]
    ax.bar(names, scores, color=["C0", "C1"])
    ax.set_ylabel("smooth score (lower better)")
    ax.set_title("Smooth score dashboard")


def main() -> None:
    args = parse_args()
    cfg = load_yaml(args.config)
    cfg_merged = copy.deepcopy(cfg)
    if args.curriculum_stage is not None:
        cfg_merged.setdefault("curriculum", {})["stage"] = str(args.curriculum_stage)
    if args.learning_rate is not None:
        cfg_merged.setdefault("sac", {})["learning_rate"] = float(args.learning_rate)
    if args.residual_force_scale is not None:
        cfg_merged.setdefault("residual", {})["residual_force_scale"] = float(args.residual_force_scale)
    if args.residual_moment_scale is not None:
        cfg_merged.setdefault("residual", {})["residual_moment_scale"] = float(args.residual_moment_scale)

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    plots = out / "plots"
    plots.mkdir(parents=True, exist_ok=True)

    model = SAC.load(str(args.model_path), device="auto")
    vec: VecNormalize | None = None
    if args.vecnormalize_path is not None and Path(args.vecnormalize_path).is_file():

        def _make() -> Any:
            return PMIWorkspace5DResidualEnv(config=cfg_merged)

        dv = DummyVecEnv([_make])
        vec = VecNormalize.load(str(args.vecnormalize_path), dv)
        vec.training = False
        vec.norm_reward = False

    do_rnd, rnd_profile = _effective_cable_randomize(cfg_merged)
    opts: dict[str, Any] = {}
    if do_rnd:
        opts["randomize_cable"] = True
        opts["cable_seed"] = int(args.seed)
        opts["randomization_profile"] = rnd_profile

    def pol_zero(obs: np.ndarray, _i: int) -> np.ndarray:
        return np.zeros(5, dtype=np.float32)

    def pol_sac(obs: np.ndarray, _i: int) -> np.ndarray:
        o = np.asarray(obs, dtype=np.float32).reshape(1, -1)
        if vec is not None:
            o = vec.normalize_obs(o)
        a, _ = model.predict(o, deterministic=True)
        return np.asarray(a, dtype=np.float32).reshape(-1)

    z = rollout_one_episode_metrics(policy_fn=pol_zero, config=cfg_merged, seed=int(args.seed), options=opts)
    s = rollout_one_episode_metrics(policy_fn=pol_sac, config=cfg_merged, seed=int(args.seed), options=opts)

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
    _tk = (
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
    for d in (z, s):
        for k in _tk:
            v = d.get(k)
            if isinstance(v, np.ndarray) and v.shape[0] >= n:
                d[k] = v[:n]
    t = z["time"]

    try:
        plt.style.use("seaborn-v0_8-whitegrid")
    except OSError:
        pass

    # 1–2: XYZ des vs act
    fig, axes = plt.subplots(3, 1, figsize=(9, 6.5), sharex=True)
    labs = ("x", "y", "z")
    for ax, k in zip(axes, range(3)):
        ax.plot(t, z["x_des"][:, k], label="des")
        ax.plot(t, z["x_act"][:, k], "--", alpha=0.85, label="zero")
        ax.plot(t, s["x_act"][:, k], ":", lw=1.2, label="sac")
        ax.set_ylabel(f"{labs[k]} [m]")
    axes[0].legend(fontsize=7, ncol=3)
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Desired vs actual XYZ")
    fig.savefig(plots / "01_xyz_des_act.png", dpi=140)
    plt.close(fig)

    # 3: EE error
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
    fig.savefig(plots / "02_ee_errors.png", dpi=140)
    plt.close(fig)

    # 4–5: roll pitch
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
    fig.savefig(plots / "03_roll_pitch.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 2.8))
    ax.plot(t, z["e_roll"], label="e_roll zero")
    ax.plot(t, s["e_roll"], "--", label="e_roll sac")
    ax.plot(t, z["e_pitch"], label="e_pitch zero")
    ax.plot(t, s["e_pitch"], "--", label="e_pitch sac")
    ax.legend(fontsize=7)
    ax.set_xlabel("time [s]")
    fig.savefig(plots / "04_roll_pitch_error.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 2.8))
    ax.plot(t, z["yaw_act"], label="yaw zero")
    ax.plot(t, s["yaw_act"], "--", label="yaw sac")
    ax.set_title("yaw (free)")
    ax.legend(fontsize=7)
    fig.savefig(plots / "05_yaw.png", dpi=140)
    plt.close(fig)

    fig = plt.figure(figsize=(7, 6))
    try:
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(z["x_des"][:, 0], z["x_des"][:, 1], z["x_des"][:, 2], label="des", lw=2)
        ax.plot(z["x_act"][:, 0], z["x_act"][:, 1], z["x_act"][:, 2], label="zero", lw=1.2)
        ax.plot(s["x_act"][:, 0], s["x_act"][:, 1], s["x_act"][:, 2], label="sac", lw=1.2)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.legend()
    except Exception:
        ax = fig.add_subplot(111)
        ax.plot(z["x_des"][:, 0], z["x_des"][:, 1], label="des")
        ax.plot(z["x_act"][:, 0], z["x_act"][:, 1], label="zero")
        ax.plot(s["x_act"][:, 0], s["x_act"][:, 1], label="sac")
    fig.tight_layout()
    fig.savefig(plots / "06_path_3d.png", dpi=140)
    plt.close(fig)

    wrz = z["W_residual_used"]
    wrs = s["W_residual_used"]
    fig, axes = plt.subplots(5, 1, figsize=(9, 9), sharex=True)
    labels = ("Fx", "Fy", "Fz", "Mroll", "Mpitch")
    for i in range(5):
        axes[i].plot(t, wrz[:, i] if wrz.shape[0] == t.shape[0] else wrz[:, i][: t.shape[0]], label="zero")
        axes[i].plot(t, wrs[:, i] if wrs.shape[0] == t.shape[0] else wrs[:, i][: t.shape[0]], "--", label="sac")
        axes[i].set_ylabel(labels[i])
    axes[0].legend(fontsize=7)
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Residual wrench (used)")
    fig.savefig(plots / "07_residual_wrench.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(9, 8), sharex=True)
    for i in range(4):
        axes[i].plot(t, z["tau_residual"][:, i], label="zero")
        axes[i].plot(t, s["tau_residual"][:, i], "--", label="sac")
        axes[i].set_ylabel(f"tau_res {i+1}")
    axes[0].legend(fontsize=7)
    axes[-1].set_xlabel("time [s]")
    fig.savefig(plots / "08_residual_torque.png", dpi=140)
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
    fig.savefig(plots / "09_tau_breakdown_sac.png", dpi=140)
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
    fig.savefig(plots / "10_cable_torques.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 3))
    ax.plot(t, z.get("ee_hf_norm", np.zeros(n)), label="||ee_hf|| zero")
    ax.plot(t, s.get("ee_hf_norm", np.zeros(n)), "--", label="||ee_hf|| sac")
    ax.set_ylabel("HF EE norm (env LP)")
    ax.set_xlabel("time [s]")
    ax.legend(fontsize=7)
    fig.savefig(plots / "11_highfreq.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(5, 3))
    smooth_dashboard(ax, z, s)
    fig.tight_layout()
    fig.savefig(plots / "12_smooth_dashboard.png", dpi=140)
    plt.close(fig)

    np.savez_compressed(
        out / "comparison_timeseries.npz",
        time=t,
        z_x_des=z["x_des"],
        s_x_act=s["x_act"],
        z_e_norm=z["e_norm"],
        s_e_norm=s["e_norm"],
    )

    print(f"Wrote plots to {plots}")


if __name__ == "__main__":
    main()
