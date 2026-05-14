#!/usr/bin/env python3
"""Publication-quality paired compare: VSD-only vs VSD+SAC under full_noise_light (eval only)."""

from __future__ import annotations

import argparse
import copy
import csv
import subprocess
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

_ROOT = Path(__file__).resolve().parents[1]
import sys

if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from utils.mujoco_helpers import load_yaml
from utils.workspace_5d_rl_metrics import rollout_one_episode_metrics, workspace_smooth_score


def _deep_merge(base: dict[str, Any], over: dict[str, Any]) -> dict[str, Any]:
    out = copy.deepcopy(base)
    for k, v in over.items():
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


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


FULL_NOISE_LIGHT: dict[str, Any] = {
    "noise": {
        "enabled": True,
        "sensor": {"enabled": True},
        "measurement_delay": {"enabled": True, "delay_steps": 1},
        "measurement_filter": {"enabled": True, "cutoff_hz": 20.0},
        "actuator": {"enabled": True, "torque_std_nm": 0.02, "gain_std": 0.01},
        "random_walk_bias": {"enabled": False},
    }
}


def _pct(z: float, s: float) -> float:
    if not np.isfinite(z) or abs(z) < 1e-18:
        return float("nan")
    return float((z - s) / z * 100.0)


def write_mp4(frames: list[np.ndarray], path: Path, fps: float) -> bool:
    if not frames:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    arr = np.stack(frames, axis=0)
    h_, w_, _ = arr.shape[1:]
    cmd = [
        "ffmpeg",
        "-y",
        "-f",
        "rawvideo",
        "-vcodec",
        "rawvideo",
        "-s",
        f"{w_}x{h_}",
        "-pix_fmt",
        "rgb24",
        "-r",
        str(fps),
        "-i",
        "-",
        "-an",
        "-vcodec",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(path),
    ]
    try:
        p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
        assert p.stdin is not None
        p.stdin.write(arr.astype(np.uint8).tobytes())
        p.stdin.close()
        p.wait(timeout=600)
        return p.returncode == 0 and path.is_file()
    except (FileNotFoundError, OSError):
        return False


def _aligned_n(mz: dict[str, Any], ms: dict[str, Any]) -> int:
    return int(
        min(
            len(mz["time"]),
            len(ms["time"]),
            mz["x_des"].shape[0],
            ms["x_des"].shape[0],
        )
    )


def save_paired_timeseries(m: dict[str, Any], path: Path, label: str) -> None:
    t = np.asarray(m["time"], dtype=np.float64)
    xd = np.asarray(m["x_des"], dtype=np.float64)
    xa = np.asarray(m["x_act"], dtype=np.float64)
    en = np.asarray(m["e_norm"], dtype=np.float64)
    enm = np.linalg.norm(np.asarray(m["ee_err_xyz_measured"], dtype=np.float64), axis=1)
    hf = np.asarray(m["ee_hf_norm"], dtype=np.float64)
    r_d = np.asarray(m["roll_des"], dtype=np.float64)
    r_a = np.asarray(m["roll_act"], dtype=np.float64)
    p_d = np.asarray(m["pitch_des"], dtype=np.float64)
    p_a = np.asarray(m["pitch_act"], dtype=np.float64)
    tau = np.asarray(m["tau_jnt_cmd"], dtype=np.float64)
    sat = np.asarray(m["sat_per_step"], dtype=np.int32)
    jl = np.asarray(m["jl_per_step"], dtype=np.int32)
    al = np.asarray(m["al_per_step"], dtype=np.int32)
    ncon = np.asarray(m["ncon_per_step"], dtype=np.int32)
    W = np.asarray(m["W_residual_used"], dtype=np.float64)
    n = min(len(t), xd.shape[0], len(en), tau.shape[0], W.shape[0], len(sat))
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "step",
                "time",
                "policy",
                "x_des",
                "y_des",
                "z_des",
                "x_act",
                "y_act",
                "z_act",
                "ee_err_norm_true",
                "ee_err_norm_measured",
                "roll_des",
                "roll_act",
                "pitch_des",
                "pitch_act",
                "ee_hf_norm",
                "saturation_step",
                "joint_limit_flag",
                "actuator_limit_flag",
                "ncon",
                "tau_j1",
                "tau_j2",
                "tau_j3",
                "tau_j4",
                "W_Fx",
                "W_Fy",
                "W_Fz",
                "W_Mroll",
                "W_Mpitch",
            ]
        )
        for i in range(n):
            w.writerow(
                [
                    i,
                    float(t[i]) if i < len(t) else "",
                    label,
                    float(xd[i, 0]),
                    float(xd[i, 1]),
                    float(xd[i, 2]),
                    float(xa[i, 0]),
                    float(xa[i, 1]),
                    float(xa[i, 2]),
                    float(en[i]),
                    float(enm[i]) if i < len(enm) else "",
                    float(r_d[i]),
                    float(r_a[i]),
                    float(p_d[i]),
                    float(p_a[i]),
                    float(hf[i]) if i < len(hf) else "",
                    int(sat[i]),
                    int(jl[i]),
                    int(al[i]),
                    int(ncon[i]),
                    float(tau[i, 0]),
                    float(tau[i, 1]),
                    float(tau[i, 2]),
                    float(tau[i, 3]),
                    float(W[i, 0]),
                    float(W[i, 1]),
                    float(W[i, 2]),
                    float(W[i, 3]),
                    float(W[i, 4]),
                ]
            )


def _moving_rms(x: np.ndarray, win: int) -> np.ndarray:
    if x.size == 0:
        return x
    win = max(3, int(win))
    out = np.zeros_like(x, dtype=np.float64)
    c = np.cumsum(np.insert(x.astype(np.float64) ** 2, 0, 0.0))
    for i in range(len(x)):
        i0 = max(0, i - win + 1)
        w = i - i0 + 1
        out[i] = float(np.sqrt((c[i + 1] - c[i0]) / w))
    return out


def _figure_to_rgb(fig: plt.Figure) -> np.ndarray:
    fig.canvas.draw()
    rgba = np.asarray(fig.canvas.buffer_rgba())
    return np.ascontiguousarray(rgba[:, :, :3])


def _try_cv2_overlay(img_rgb: np.ndarray, lines: list[str]) -> np.ndarray:
    try:
        import cv2
    except ImportError:
        return img_rgb
    im = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    y = 22
    for line in lines[:8]:
        cv2.putText(im, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(im, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        y += 22
    return cv2.cvtColor(im, cv2.COLOR_BGR2RGB)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=_ROOT / "configs" / "rl_workspace_5d_sac.yaml")
    ap.add_argument(
        "--model-path",
        type=Path,
        default=_ROOT
        / "debug_outputs"
        / "workspace_5d_residual_rl"
        / "runs"
        / "ws5d_residual_medium_train_rs05_30k_s4"
        / "checkpoints"
        / "best_model_by_smooth_score.zip",
    )
    ap.add_argument(
        "--vecnormalize-path",
        type=Path,
        default=_ROOT
        / "debug_outputs"
        / "workspace_5d_residual_rl"
        / "runs"
        / "ws5d_residual_medium_train_rs05_30k_s4"
        / "vecnormalize"
        / "vecnormalize.pkl",
    )
    ap.add_argument("--curriculum-stage", type=str, default="medium_train")
    ap.add_argument("--noise-preset", type=str, default="full_noise_light")
    ap.add_argument("--seed", type=int, default=30000)
    ap.add_argument(
        "--auto-select-seed",
        choices=["", "median_improvement"],
        default="",
        help="median_improvement: scan seeds 30000..30009 for median RMS EE (SAC−zero) gap.",
    )
    ap.add_argument(
        "--out-dir",
        type=Path,
        default=_ROOT / "debug_outputs" / "workspace_5d_residual_rl" / "full_noise_light_compare",
    )
    ap.add_argument("--duration-override", type=float, default=None)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--video-stride", type=int, default=0, help="If 0, derive from fps and control_dt.")
    return ap.parse_args()


def build_cfg(args: argparse.Namespace) -> dict[str, Any]:
    cfg = load_yaml(args.config)
    cfg.setdefault("curriculum", {})["stage"] = str(args.curriculum_stage)
    if args.noise_preset != "full_noise_light":
        raise SystemExit("This script is wired for full_noise_light; use evaluate_workspace_5d_noise_robustness for others.")
    cfg = _deep_merge(cfg, FULL_NOISE_LIGHT)
    if args.duration_override is not None:
        cfg.setdefault("env", {})["episode_duration"] = float(args.duration_override)
    return cfg


def main() -> None:
    args = parse_args()
    cfg = build_cfg(args)
    out = Path(args.out_dir)
    plots = out / "plots"
    videos = out / "videos"
    out.mkdir(parents=True, exist_ok=True)
    plots.mkdir(parents=True, exist_ok=True)
    videos.mkdir(parents=True, exist_ok=True)

    model = SAC.load(str(args.model_path), device="auto")
    vec: VecNormalize | None = None
    if args.vecnormalize_path.is_file():

        def _mk() -> Any:
            from envs.pmi_workspace_5d_residual_env import PMIWorkspace5DResidualEnv

            return PMIWorkspace5DResidualEnv(config=copy.deepcopy(cfg))

        dv = DummyVecEnv([_mk])
        vec = VecNormalize.load(str(args.vecnormalize_path), dv)
        vec.training = False
        vec.norm_reward = False

    def pol_zero(obs: np.ndarray, _i: int) -> np.ndarray:
        return np.zeros(5, dtype=np.float32)

    def pol_sac(obs: np.ndarray, _i: int) -> np.ndarray:
        o = np.asarray(obs, dtype=np.float32).reshape(1, -1)
        if vec is not None:
            o = vec.normalize_obs(o)
        a, _ = model.predict(o, deterministic=True)
        return np.asarray(a, dtype=np.float32).reshape(-1)

    do_rnd, rnd_pro = _effective_cable_randomize(cfg)

    scan_rows: list[dict[str, Any]] = []
    selected_seed = int(args.seed)
    if args.auto_select_seed == "median_improvement":
        deltas: list[tuple[int, float]] = []
        for ep in range(10):
            s0 = 30000 + ep
            opts: dict[str, Any] = {}
            if do_rnd:
                opts["randomize_cable"] = True
                opts["cable_seed"] = s0
                opts["randomization_profile"] = rnd_pro
            z = rollout_one_episode_metrics(policy_fn=pol_zero, config=cfg, seed=s0, options=opts)
            s = rollout_one_episode_metrics(policy_fn=pol_sac, config=cfg, seed=s0, options=opts)
            dr = float(s["rms_ee_error"]) - float(z["rms_ee_error"])
            deltas.append((s0, dr))
            scan_rows.append(
                {
                    "seed": s0,
                    "zero_rms_ee": float(z["rms_ee_error"]),
                    "sac_rms_ee": float(s["rms_ee_error"]),
                    "delta_rms_ee": dr,
                }
            )
        med = float(np.median([d[1] for d in deltas]))
        selected_seed = int(min(deltas, key=lambda t: abs(t[1] - med))[0])

    from envs.pmi_workspace_5d_residual_env import PMIWorkspace5DResidualEnv

    _env_dt = PMIWorkspace5DResidualEnv(config=copy.deepcopy(cfg))
    dt = float(_env_dt.control_dt)
    _env_dt.close()
    stride = int(args.video_stride) if args.video_stride > 0 else max(1, int(round(1.0 / (float(args.fps) * dt + 1e-9))))

    opts: dict[str, Any] = {}
    if do_rnd:
        opts["randomize_cable"] = True
        opts["cable_seed"] = selected_seed
        opts["randomization_profile"] = rnd_pro

    mz = rollout_one_episode_metrics(
        policy_fn=pol_zero,
        config=cfg,
        seed=selected_seed,
        options=opts,
        collect_rgb_frames=True,
        rgb_frame_stride=stride,
    )
    ms = rollout_one_episode_metrics(
        policy_fn=pol_sac,
        config=cfg,
        seed=selected_seed,
        options=opts,
        collect_rgb_frames=True,
        rgb_frame_stride=stride,
    )
    mz["smooth_score"] = float(workspace_smooth_score(mz))
    ms["smooth_score"] = float(workspace_smooth_score(ms))

    save_paired_timeseries(mz, out / "paired_timeseries_zero.csv", "zero")
    save_paired_timeseries(ms, out / "paired_timeseries_sac.csv", "sac")

    nz = _aligned_n(mz, ms)
    tz = np.asarray(mz["time"], dtype=np.float64)[:nz]
    xz_d = np.asarray(mz["x_des"], dtype=np.float64)[:nz]
    xz_a = np.asarray(mz["x_act"], dtype=np.float64)[:nz]
    xs_d = np.asarray(ms["x_des"], dtype=np.float64)[:nz]
    xs_a = np.asarray(ms["x_act"], dtype=np.float64)[:nz]
    en_z = np.asarray(mz["e_norm"], dtype=np.float64)[:nz]
    en_s = np.asarray(ms["e_norm"], dtype=np.float64)[:nz]
    enm_z = np.linalg.norm(np.asarray(mz["ee_err_xyz_measured"], dtype=np.float64)[:nz], axis=1)
    enm_s = np.linalg.norm(np.asarray(ms["ee_err_xyz_measured"], dtype=np.float64)[:nz], axis=1)
    err_z = np.asarray(mz["ee_err_xyz"], dtype=np.float64)[:nz]
    err_s = np.asarray(ms["ee_err_xyz"], dtype=np.float64)[:nz]
    hf_z = np.asarray(mz["ee_hf_norm"], dtype=np.float64)[:nz]
    hf_s = np.asarray(ms["ee_hf_norm"], dtype=np.float64)[:nz]
    er_z = np.asarray(mz["e_roll"], dtype=np.float64)[:nz]
    er_s = np.asarray(ms["e_roll"], dtype=np.float64)[:nz]
    ep_z = np.asarray(mz["e_pitch"], dtype=np.float64)[:nz]
    ep_s = np.asarray(ms["e_pitch"], dtype=np.float64)[:nz]
    r_d = np.asarray(mz["roll_des"], dtype=np.float64)[:nz]
    r_az = np.asarray(mz["roll_act"], dtype=np.float64)[:nz]
    r_as = np.asarray(ms["roll_act"], dtype=np.float64)[:nz]
    p_d = np.asarray(mz["pitch_des"], dtype=np.float64)[:nz]
    p_az = np.asarray(mz["pitch_act"], dtype=np.float64)[:nz]
    p_as = np.asarray(ms["pitch_act"], dtype=np.float64)[:nz]
    Ws = np.asarray(ms["W_residual_used"], dtype=np.float64)[:nz]
    tau_z = np.asarray(mz["tau_jnt_cmd"], dtype=np.float64)[:nz]
    tau_s = np.asarray(ms["tau_jnt_cmd"], dtype=np.float64)[:nz]
    sat_z = np.asarray(mz["sat_per_step"], dtype=np.float64)[:nz]
    sat_s = np.asarray(ms["sat_per_step"], dtype=np.float64)[:nz]
    jl_z = np.asarray(mz["jl_per_step"], dtype=np.float64)[:nz]
    jl_s = np.asarray(ms["jl_per_step"], dtype=np.float64)[:nz]
    al_z = np.asarray(mz["al_per_step"], dtype=np.float64)[:nz]
    al_s = np.asarray(ms["al_per_step"], dtype=np.float64)[:nz]

    try:
        plt.style.use("seaborn-v0_8-whitegrid")
    except OSError:
        pass
    title_note = "full_noise_light · workspace 5D VSD · yaw free · collision off"

    def fig_xyz(m_d: np.ndarray, m_a: np.ndarray, fname: str, subtitle: str) -> None:
        fig, axs = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
        labs = ("x", "y", "z")
        for i in range(3):
            axs[i].plot(tz, m_d[:, i], "k--", lw=1.2, label="desired")
            axs[i].plot(tz, m_a[:, i], lw=1.0, label="actual (sim true)")
            axs[i].set_ylabel(f"{labs[i]} [m]")
        axs[0].legend(loc="upper right", fontsize=8)
        axs[-1].set_xlabel("time [s]")
        fig.suptitle(f"{subtitle}\n{title_note}")
        fig.tight_layout()
        fig.savefig(plots / fname, dpi=180)
        plt.close(fig)

    fig_xyz(xz_d, xz_a, "ee_xyz_desired_vs_actual_zero.png", "VSD only — EE position")
    fig_xyz(xs_d, xs_a, "ee_xyz_desired_vs_actual_sac.png", "VSD + SAC residual — EE position")

    # Paper-style stacked EE error: e_x, e_y, e_z, ||e|| — zero vs sac (blue solid / orange dashed)
    _c_zero, _c_sac = "#1f77b4", "#ff7f0e"
    fig, axs = plt.subplots(4, 1, figsize=(7.5, 6.8), sharex=True)
    y_labs = ("e_x [m]", "e_y [m]", "e_z [m]")
    for k in range(3):
        axs[k].plot(
            tz,
            err_z[:, k],
            color=_c_zero,
            ls="-",
            lw=1.25,
            label="zero" if k == 0 else "_nolegend_",
        )
        axs[k].plot(
            tz,
            err_s[:, k],
            color=_c_sac,
            ls="--",
            lw=1.25,
            label="sac" if k == 0 else "_nolegend_",
        )
        axs[k].set_ylabel(y_labs[k])
    axs[0].legend(loc="upper left", fontsize=9, framealpha=0.92)
    axs[3].plot(tz, en_z, color=_c_zero, ls="-", lw=1.25, label="_nolegend_")
    axs[3].plot(tz, en_s, color=_c_sac, ls="--", lw=1.25, label="_nolegend_")
    axs[3].set_ylabel("||e|| [m]")
    axs[3].set_xlabel("time [s]")
    fig.suptitle(
        f"EE tracking error (true) — seed {selected_seed}\n{title_note}",
        fontsize=10,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(plots / "ee_error_components_compare.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(tz, xz_d[:, 0], "k--", lw=1.2, label="des x")
    ax.plot(tz, xz_d[:, 1], "k:", lw=1.0, label="des y")
    ax.plot(tz, xz_d[:, 2], "k-.", lw=1.0, label="des z")
    ax.plot(tz, xz_a[:, 0], lw=1.0, alpha=0.85, label="actual x (VSD only)")
    ax.plot(tz, xz_a[:, 1], lw=1.0, alpha=0.85, label="actual y (VSD only)")
    ax.plot(tz, xz_a[:, 2], lw=1.0, alpha=0.85, label="actual z (VSD only)")
    ax.plot(tz, xs_a[:, 0], "--", lw=1.0, label="actual x (SAC)")
    ax.plot(tz, xs_a[:, 1], "--", lw=1.0, label="actual y (SAC)")
    ax.plot(tz, xs_a[:, 2], "--", lw=1.0, label="actual z (SAC)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("position [m]")
    ax.legend(ncol=2, fontsize=7)
    fig.suptitle(f"Desired vs actual XYZ — overlay — seed {selected_seed}\n{title_note}")
    fig.tight_layout()
    fig.savefig(plots / "ee_xyz_overlay_compare.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(tz, en_z, label="||e_xyz|| VSD only (true)")
    ax.plot(tz, en_s, label="||e_xyz|| VSD+SAC (true)")
    ax.axhline(float(en_z[-1]), color="C0", ls=":", alpha=0.4)
    ax.axhline(float(en_s[-1]), color="C1", ls=":", alpha=0.4)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("||EE pos error|| [m]")
    ax.legend()
    fig.suptitle(f"EE error norm (true) — seed {selected_seed}\n{title_note}")
    fig.tight_layout()
    fig.savefig(plots / "ee_error_norm_compare.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    ax[0].plot(tz, er_z, label="roll err VSD only")
    ax[0].plot(tz, er_s, label="roll err SAC")
    ax[0].set_ylabel("roll error [rad]")
    ax[0].legend(fontsize=8)
    ax[1].plot(tz, ep_z, label="pitch err VSD only")
    ax[1].plot(tz, ep_s, label="pitch err SAC")
    ax[1].set_ylabel("pitch error [rad]")
    ax[1].set_xlabel("time [s]")
    ax[1].legend(fontsize=8)
    fig.suptitle(f"Roll/pitch error — seed {selected_seed}\n{title_note}")
    fig.tight_layout()
    fig.savefig(plots / "roll_pitch_error_compare.png", dpi=180)
    plt.close(fig)

    win = max(5, int(0.25 / dt))
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(tz, _moving_rms(en_z, win), label=f"windowed RMS ||e|| VSD ({win} steps)")
    ax.plot(tz, _moving_rms(en_s, win), label=f"windowed RMS ||e|| SAC ({win} steps)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("moving RMS [m]")
    ax.legend()
    fig.suptitle(f"Windowed RMS position error — seed {selected_seed}\n{title_note}")
    fig.tight_layout()
    fig.savefig(plots / "ee_rms_windowed_compare.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(tz, hf_z, label="HF error norm VSD only")
    ax.plot(tz, hf_s, label="HF error norm SAC")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("high-freq EE error norm")
    ax.legend()
    fig.suptitle(f"High-frequency EE error — seed {selected_seed}\n{title_note}")
    fig.tight_layout()
    fig.savefig(plots / "hf_error_compare.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(5, 1, figsize=(8, 8), sharex=True)
    names = ("Fx", "Fy", "Fz", "Mroll", "Mpitch")
    for k in range(5):
        ax[k].plot(tz, Ws[:, k], color="C3")
        ax[k].set_ylabel(names[k])
    ax[-1].set_xlabel("time [s]")
    fig.suptitle(f"SAC residual wrench — seed {selected_seed}\n{title_note}")
    fig.tight_layout()
    fig.savefig(plots / "residual_wrench_components_sac.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(4, 1, figsize=(8, 7), sharex=True)
    for j in range(4):
        ax[j].plot(tz, tau_z[:, j], label="VSD only")
        ax[j].plot(tz, tau_s[:, j], "--", label="SAC", alpha=0.9)
        ax[j].set_ylabel(f"τ_joint_{j+1}")
        if j == 0:
            ax[j].legend(fontsize=7)
    ax[-1].set_xlabel("time [s]")
    fig.suptitle(f"Total joint torque command — seed {selected_seed}\n{title_note}")
    fig.tight_layout()
    fig.savefig(plots / "tau_total_compare.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(2, 1, figsize=(8, 4), sharex=True)
    ax[0].plot(tz, sat_z, drawstyle="steps-post", label="sat VSD only")
    ax[0].plot(tz, sat_s, drawstyle="steps-post", label="sat SAC", alpha=0.85)
    ax[0].set_ylabel("saturation / step")
    ax[0].legend(fontsize=8)
    lim_z = (jl_z + al_z).clip(0, 1)
    lim_s = (jl_s + al_s).clip(0, 1)
    ax[1].plot(tz, lim_z, drawstyle="steps-post", label="limit VSD")
    ax[1].plot(tz, lim_s, drawstyle="steps-post", label="limit SAC")
    ax[1].set_ylabel("limit breach")
    ax[1].set_xlabel("time [s]")
    ax[1].legend(fontsize=8)
    fig.suptitle(f"Saturation & limit flags — seed {selected_seed}\n{title_note}")
    fig.tight_layout()
    fig.savefig(plots / "saturation_limit_compare.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(tz, en_z, label="true ||e|| VSD")
    ax.plot(tz, enm_z, "--", label="measured ||e|| VSD")
    ax.plot(tz, en_s, label="true ||e|| SAC")
    ax.plot(tz, enm_s, "--", label="measured ||e|| SAC")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("||EE error|| [m]")
    ax.legend()
    fig.suptitle(f"True vs measured EE error (noise pipeline) — seed {selected_seed}\n{title_note}")
    fig.tight_layout()
    fig.savefig(plots / "measured_vs_true_error_compare.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(5.5, 5))
    ax.plot(xz_d[:, 0], xz_d[:, 1], "k--", lw=1.2, label="desired (xy)")
    ax.plot(xz_a[:, 0], xz_a[:, 1], lw=1.0, label="actual VSD only")
    ax.plot(xs_a[:, 0], xs_a[:, 1], "--", lw=1.0, label="actual SAC")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend()
    fig.suptitle(f"EE path XY — seed {selected_seed}\n{title_note}")
    fig.tight_layout()
    fig.savefig(plots / "ee_xy_path_compare.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.plot(xz_d[:, 0], xz_d[:, 2], "k--", lw=1.2, label="desired (xz)")
    ax.plot(xz_a[:, 0], xz_a[:, 2], lw=1.0, label="actual VSD only")
    ax.plot(xs_a[:, 0], xs_a[:, 2], "--", lw=1.0, label="actual SAC")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.legend()
    fig.suptitle(f"EE path XZ — seed {selected_seed}\n{title_note}")
    fig.tight_layout()
    fig.savefig(plots / "ee_xz_path_compare.png", dpi=180)
    plt.close(fig)

    path_3d_ok = False
    fig_3d: plt.Figure | None = None
    try:
        fig_3d = plt.figure(figsize=(6, 5))
        ax3 = fig_3d.add_subplot(111, projection="3d")
        ax3.plot(xz_d[:, 0], xz_d[:, 1], xz_d[:, 2], "k--", lw=1.0, label="desired")
        ax3.plot(xz_a[:, 0], xz_a[:, 1], xz_a[:, 2], lw=0.9, label="VSD only")
        ax3.plot(xs_a[:, 0], xs_a[:, 1], xs_a[:, 2], lw=0.9, label="SAC")
        ax3.set_xlabel("x")
        ax3.set_ylabel("y")
        ax3.set_zlabel("z")
        ax3.legend(fontsize=7)
        fig_3d.suptitle(f"EE 3D path — seed {selected_seed}\n{title_note}")
        fig_3d.tight_layout()
        fig_3d.savefig(plots / "ee_3d_path_compare.png", dpi=180)
        path_3d_ok = True
    except Exception:
        path_3d_ok = False
    finally:
        if fig_3d is not None:
            plt.close(fig_3d)

    fig, ax = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    ax[0].plot(tz, r_d, "k--", label="roll ref")
    ax[0].plot(tz, r_az, label="roll VSD")
    ax[0].plot(tz, r_as, label="roll SAC")
    ax[0].set_ylabel("roll [rad]")
    ax[0].legend(fontsize=7)
    ax[1].plot(tz, p_d, "k--", label="pitch ref")
    ax[1].plot(tz, p_az, label="pitch VSD")
    ax[1].plot(tz, p_as, label="pitch SAC")
    ax[1].set_ylabel("pitch [rad]")
    ax[1].set_xlabel("time [s]")
    ax[1].legend(fontsize=7)
    fig.suptitle(f"Orientation tracking — seed {selected_seed}\n{title_note}")
    fig.tight_layout()
    fig.savefig(plots / "orientation_tracking_compare.png", dpi=180)
    plt.close(fig)

    zr = float(mz["rms_ee_error"])
    sr = float(ms["rms_ee_error"])
    zf = float(mz["final_ee_error"])
    sf = float(ms["final_ee_error"])
    zh = float(mz["rms_highfreq"])
    sh = float(ms["rms_highfreq"])
    zsm = float(mz["smooth_score"])
    ssm = float(ms["smooth_score"])
    zsat = float(mz["saturation_fraction"])
    ssat = float(ms["saturation_fraction"])
    zlim = float(mz["limit_fraction"])
    slim = float(ms["limit_fraction"])
    zn = float(mz["ncon_max"])
    sn = float(ms["ncon_max"])

    summary_row = {
        "seed": selected_seed,
        "noise_preset": "full_noise_light",
        "rms_ee_zero": zr,
        "rms_ee_sac": sr,
        "final_ee_zero": zf,
        "final_ee_sac": sf,
        "hf_zero": zh,
        "hf_sac": sh,
        "smooth_zero": zsm,
        "smooth_sac": ssm,
        "sat_zero": zsat,
        "sat_sac": ssat,
        "lim_zero": zlim,
        "lim_sac": slim,
        "ncon_zero": zn,
        "ncon_sac": sn,
        "delta_rms_ee": sr - zr,
        "delta_final_ee": sf - zf,
        "delta_hf": sh - zh,
        "pct_imp_rms_ee": _pct(zr, sr),
        "pct_imp_final_ee": _pct(zf, sf),
        "pct_imp_hf": _pct(zh, sh),
        "control_dt": dt,
        "fps_video": float(args.fps),
        "rgb_stride": stride,
        "camera": "MuJoCo Renderer default (follows env render; 640×480)",
        "ee_3d_path_plot_ok": path_3d_ok,
    }
    with open(out / "comparison_summary.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(summary_row.keys()))
        w.writeheader()
        w.writerow(summary_row)

    rep = [
        "# Paired comparison — full_noise_light\n\n",
        "## A. Setup\n\n",
        f"- **Checkpoint:** `{args.model_path}`\n",
        f"- **VecNormalize:** `{args.vecnormalize_path}`\n",
        f"- **Seed (selected):** {selected_seed}\n",
        f"- **Noise preset:** `full_noise_light` (sensor + delay=1 + LPF 20 Hz + actuator torque/gain noise)\n",
        f"- **Controller:** workspace 5D VSD nominal; task $x,y,z$, roll, pitch; **yaw free**; roll/pitch ref = initial EE orientation.\n",
        f"- **Cable:** transmission on; **collision off.**\n",
        f"- **control_dt:** {dt:.4f} s · **video fps:** {args.fps} · **RGB stride:** {stride}\n",
        f"- **Camera:** MuJoCo `Renderer` rgb_array (640×480).\n",
        f"- **3D path plot:** {'saved `ee_3d_path_compare.png`' if path_3d_ok else 'skipped (see XY/XZ fallbacks)'}\n\n",
    ]
    if scan_rows:
        rep.append("### Auto seed scan (30000–30009)\n\n")
        rep.append("| seed | zero RMS EE | SAC RMS EE | Δ |\n| --- | --- | --- | --- |\n")
        for r in scan_rows:
            rep.append(
                f"| {r['seed']} | {r['zero_rms_ee']:.6f} | {r['sac_rms_ee']:.6f} | {r['delta_rms_ee']:.6f} |\n"
            )
        rep.append(
            f"\n**Chosen seed (closest to median Δ, Δ = SAC RMS EE − zero RMS EE):** {selected_seed}\n\n"
        )

    rep.extend(
        [
            "## B. Scalar metrics\n\n",
            "| Metric | VSD only | VSD + SAC |\n| --- | --- | --- |\n",
            f"| RMS EE (true) | {zr:.6f} | {sr:.6f} |\n",
            f"| Final EE (true) | {zf:.6f} | {sf:.6f} |\n",
            f"| RMS HF | {zh:.6f} | {sh:.6f} |\n",
            f"| Smooth score | {zsm:.6f} | {ssm:.6f} |\n",
            f"| Saturation frac. | {zsat:.6f} | {ssat:.6f} |\n",
            f"| Limit frac. | {zlim:.6f} | {slim:.6f} |\n",
            f"| ncon max | {zn:.0f} | {sn:.0f} |\n\n",
            "## C. Improvement (SAC vs zero)\n\n",
            "| | Δ (SAC−zero) | % improvement vs zero |\n| --- | --- | --- |\n",
            f"| RMS EE | {sr - zr:.6f} | {_pct(zr, sr):.2f}% |\n",
            f"| Final EE | {sf - zf:.6f} | {_pct(zf, sf):.2f}% |\n",
            f"| RMS HF | {sh - zh:.6f} | {_pct(zh, sh):.2f}% |\n\n",
            "## D. Interpretation\n\n",
            f"- SAC changes **RMS EE (true)** by {_pct(zr, sr):.2f}% vs zero residual (negative Δ means SAC lower).\n",
            f"- **Final EE** improvement: {_pct(zf, sf):.2f}%.\n",
            f"- **HF** metric: {_pct(zh, sh):.2f}%.\n",
            f"- **Contacts:** max ncon zero={zn:.0f}, SAC={sn:.0f}.\n",
            f"- **Saturation:** SAC − zero = {ssat - zsat:.4f}; **limits:** {slim - zlim:.4f}.\n\n",
            "## Outputs\n\n",
            "- Plots: `plots/` (includes `ee_error_components_compare.png`: e_x, e_y, e_z, ||e|| vs time)\n",
            "- Videos: `videos/full_noise_light_side_by_side.mp4`, `full_noise_light_overlay_path.mp4`, `full_noise_light_zoomed_terminal_compare.mp4`\n",
            "- Time series: `paired_timeseries_zero.csv`, `paired_timeseries_sac.csv`\n",
        ]
    )
    (out / "comparison_report.md").write_text("".join(rep), encoding="utf-8")

    # --- Videos ---
    fz = mz.get("rgb_frames") or []
    fs = ms.get("rgb_frames") or []
    nf = min(len(fz), len(fs))
    combined: list[np.ndarray] = []
    t_vid = tz[::stride][:nf] if len(tz) >= nf * stride else np.linspace(0, float(tz[-1]) if len(tz) else 0, nf)
    if len(t_vid) < nf:
        t_vid = np.linspace(0, float(tz[-1]) if len(tz) else 5.0, nf)
    for i in range(nf):
        a = np.asarray(fz[i], dtype=np.uint8)
        b = np.asarray(fs[i], dtype=np.uint8)
        h = min(a.shape[0], b.shape[0])
        w_ = min(a.shape[1], b.shape[1])
        a = a[:h, :w_]
        b = b[:h, :w_]
        bar = np.zeros((36, w_ * 2, 3), dtype=np.uint8) + 40
        c = np.concatenate([a, b], axis=1)
        # step index for HUD
        si = min(i * stride, nz - 1)
        ee_z = float(en_z[si]) if si < len(en_z) else 0.0
        ee_s = float(en_s[si]) if si < len(en_s) else 0.0
        rz = float(er_z[si]) if si < len(er_z) else 0.0
        rs = float(er_s[si]) if si < len(er_s) else 0.0
        pz = float(ep_z[si]) if si < len(ep_z) else 0.0
        ps = float(ep_s[si]) if si < len(ep_s) else 0.0
        lines = [
            "full_noise_light",
            f"seed {selected_seed}  t={t_vid[i]:.2f}s",
            f"|e_xyz| VSD {ee_z:.4f}  SAC {ee_s:.4f}",
            f"roll err VSD {rz:.4f}  SAC {rs:.4f}",
            f"pitch err VSD {pz:.4f}  SAC {ps:.4f}",
            "LEFT: VSD only   RIGHT: VSD+SAC residual",
        ]
        bar = _try_cv2_overlay(bar, lines)
        frame = np.concatenate([bar, c], axis=0)
        combined.append(frame)
    write_mp4(combined, videos / "full_noise_light_side_by_side.mp4", float(args.fps))

    path_frames: list[np.ndarray] = []
    zoom_frames: list[np.ndarray] = []
    hxy = 0.12
    cxy = xz_d[-1, :2].astype(np.float64)
    for i in range(nf):
        si = min(i * stride, nz - 1)
        fig = plt.figure(figsize=(6.4, 4.8))
        ax = fig.add_subplot(111)
        ax.plot(xz_d[:, 0], xz_d[:, 1], "k--", lw=1.0, label="desired xy")
        ax.plot(xz_a[: si + 1, 0], xz_a[: si + 1, 1], color="C0", lw=1.2, label="VSD only")
        ax.plot(xs_a[: si + 1, 0], xs_a[: si + 1, 1], color="C1", lw=1.2, label="SAC")
        ax.scatter([xz_a[si, 0]], [xz_a[si, 1]], c="C0", s=28, zorder=5)
        ax.scatter([xs_a[si, 0]], [xs_a[si, 1]], c="C1", s=28, zorder=5)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.legend(loc="upper right", fontsize=7)
        ax.set_title(
            f"EE XY trail — t={tz[si]:.2f}s — seed {selected_seed}\n"
            f"full_noise_light · |e| VSD {en_z[si]:.4f} SAC {en_s[si]:.4f}"
        )
        fig.text(
            0.02,
            0.02,
            f"roll err: VSD {er_z[si]:.4f}  SAC {er_s[si]:.4f}\n"
            f"pitch err: VSD {ep_z[si]:.4f}  SAC {ep_s[si]:.4f}",
            fontsize=7,
            va="bottom",
            family="monospace",
        )
        fig.tight_layout()
        path_frames.append(_figure_to_rgb(fig))
        plt.close(fig)

        figz = plt.figure(figsize=(6.4, 4.8))
        axz = figz.add_subplot(111)
        axz.plot(xz_d[:, 0], xz_d[:, 1], "k--", lw=1.0, label="desired xy")
        axz.plot(xz_a[: si + 1, 0], xz_a[: si + 1, 1], color="C0", lw=1.2, label="VSD only")
        axz.plot(xs_a[: si + 1, 0], xs_a[: si + 1, 1], color="C1", lw=1.2, label="SAC")
        axz.scatter([xz_a[si, 0]], [xz_a[si, 1]], c="C0", s=28, zorder=5)
        axz.scatter([xs_a[si, 0]], [xs_a[si, 1]], c="C1", s=28, zorder=5)
        axz.set_aspect("equal", adjustable="box")
        axz.set_xlim(float(cxy[0] - hxy), float(cxy[0] + hxy))
        axz.set_ylim(float(cxy[1] - hxy), float(cxy[1] + hxy))
        axz.set_xlabel("x [m]")
        axz.set_ylabel("y [m]")
        axz.legend(loc="upper right", fontsize=7)
        axz.set_title(
            f"Terminal region (zoom) — t={tz[si]:.2f}s — seed {selected_seed}\nfull_noise_light"
        )
        figz.tight_layout()
        zoom_frames.append(_figure_to_rgb(figz))
        plt.close(figz)

    write_mp4(path_frames, videos / "full_noise_light_overlay_path.mp4", float(args.fps))
    write_mp4(zoom_frames, videos / "full_noise_light_zoomed_terminal_compare.mp4", float(args.fps))

    print(f"Done. Outputs in {out}")


if __name__ == "__main__":
    main()
