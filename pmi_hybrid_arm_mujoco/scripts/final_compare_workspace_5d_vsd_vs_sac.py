#!/usr/bin/env python3
"""Final paired comparison: workspace 5D VSD only vs VSD + SAC residual (medium_train checkpoint)."""

from __future__ import annotations

import argparse
import copy
import csv
import subprocess
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import matplotlib

matplotlib.use("Agg")

from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from utils.mujoco_helpers import load_yaml
from utils.workspace_5d_comparison_plots import save_comparison_plot_set
from utils.workspace_5d_rl_metrics import rollout_one_episode_metrics, workspace_smooth_score


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=_ROOT / "configs" / "rl_workspace_5d_sac.yaml")
    ap.add_argument(
        "--model-path",
        type=Path,
        default=_ROOT / "debug_outputs" / "workspace_5d_residual_rl" / "runs" / "ws5d_residual_medium_train_rs05_30k_s4" / "checkpoints" / "best_model_by_smooth_score.zip",
    )
    ap.add_argument(
        "--vecnormalize-path",
        type=Path,
        default=_ROOT / "debug_outputs" / "workspace_5d_residual_rl" / "runs" / "ws5d_residual_medium_train_rs05_30k_s4" / "vecnormalize" / "vecnormalize.pkl",
    )
    ap.add_argument("--curriculum-stage", type=str, default="medium_train")
    ap.add_argument("--seed-start", type=int, default=10000)
    ap.add_argument("--num-episodes", type=int, default=50)
    ap.add_argument("--video-seeds", type=int, nargs="*", default=[10000, 10001, 10002, 10003, 10004])
    ap.add_argument(
        "--save-video",
        action="store_true",
        help="Record mp4 rollouts (on by default; this flag is optional and only documents intent).",
    )
    ap.add_argument("--no-videos", action="store_true", help="Skip RGB rollouts and mp4 export (faster).")
    ap.add_argument("--plot-detail-seed", type=int, default=10000, help="Seed for full PNG set (single seed).")
    ap.add_argument("--video-fps", type=float, default=30.0)
    ap.add_argument("--video-stride", type=int, default=3)
    ap.add_argument(
        "--out-dir",
        type=Path,
        default=_ROOT / "debug_outputs" / "workspace_5d_residual_rl" / "final_comparison_medium_train",
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


def main() -> None:
    args = parse_args()
    cfg = load_yaml(args.config)
    cfg_merged = copy.deepcopy(cfg)
    cfg_merged.setdefault("curriculum", {})["stage"] = str(args.curriculum_stage)

    out = Path(args.out_dir)
    metrics_dir = out / "metrics"
    plots_dir = out / "plots"
    videos_dir = out / "videos"
    metrics_dir.mkdir(parents=True, exist_ok=True)
    plots_dir.mkdir(parents=True, exist_ok=True)
    videos_dir.mkdir(parents=True, exist_ok=True)

    model = SAC.load(str(args.model_path), device="auto")
    vec: VecNormalize | None = None
    if args.vecnormalize_path is not None and Path(args.vecnormalize_path).is_file():

        def _make() -> Any:
            from envs.pmi_workspace_5d_residual_env import PMIWorkspace5DResidualEnv

            return PMIWorkspace5DResidualEnv(config=cfg_merged)

        dv = DummyVecEnv([_make])
        vec = VecNormalize.load(str(args.vecnormalize_path), dv)
        vec.training = False
        vec.norm_reward = False

    do_rnd, rnd_profile = _effective_cable_randomize(cfg_merged)

    def pol_zero(obs: np.ndarray, _i: int) -> np.ndarray:
        return np.zeros(5, dtype=np.float32)

    def pol_sac(obs: np.ndarray, _i: int) -> np.ndarray:
        o = np.asarray(obs, dtype=np.float32).reshape(1, -1)
        if vec is not None:
            o = vec.normalize_obs(o)
        a, _ = model.predict(o, deterministic=True)
        return np.asarray(a, dtype=np.float32).reshape(-1)

    rows: list[dict[str, Any]] = []
    n_ep = int(args.num_episodes)
    for ep in range(n_ep):
        seed = int(args.seed_start + ep)
        opts: dict[str, Any] = {}
        if do_rnd:
            opts["randomize_cable"] = True
            opts["cable_seed"] = seed
            opts["randomization_profile"] = rnd_profile

        z = rollout_one_episode_metrics(policy_fn=pol_zero, config=cfg_merged, seed=seed, options=opts)
        s = rollout_one_episode_metrics(policy_fn=pol_sac, config=cfg_merged, seed=seed, options=opts)
        z["smooth_score"] = float(workspace_smooth_score(z))  # type: ignore[arg-type]
        s["smooth_score"] = float(workspace_smooth_score(s))  # type: ignore[arg-type]

        def fmag(d: dict[str, Any], k: str) -> float:
            v = d.get(k, 0.0)
            return float(v) if np.isscalar(v) else float("nan")

        row: dict[str, Any] = {
            "episode": ep,
            "seed": seed,
            "zero_rms_ee": fmag(z, "rms_ee_error"),
            "sac_rms_ee": fmag(s, "rms_ee_error"),
            "delta_rms_ee": fmag(s, "rms_ee_error") - fmag(z, "rms_ee_error"),
            "zero_final_ee": fmag(z, "final_ee_error"),
            "sac_final_ee": fmag(s, "final_ee_error"),
            "delta_final_ee": fmag(s, "final_ee_error") - fmag(z, "final_ee_error"),
            "zero_rms_hf": fmag(z, "rms_highfreq"),
            "sac_rms_hf": fmag(s, "rms_highfreq"),
            "delta_rms_hf": fmag(s, "rms_highfreq") - fmag(z, "rms_highfreq"),
            "zero_smooth": float(z["smooth_score"]),
            "sac_smooth": float(s["smooth_score"]),
            "delta_smooth": float(s["smooth_score"]) - float(z["smooth_score"]),
            "zero_saturation": fmag(z, "saturation_fraction"),
            "sac_saturation": fmag(s, "saturation_fraction"),
            "zero_limit": fmag(z, "limit_fraction"),
            "sac_limit": fmag(s, "limit_fraction"),
            "zero_ncon": fmag(z, "ncon_max"),
            "sac_ncon": fmag(s, "ncon_max"),
        }
        rows.append(row)

    csv_path = metrics_dir / "paired_metrics_50episodes.csv"
    keys = list(rows[0].keys()) if rows else []
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(r)

    def col_mean(k: str) -> float:
        return float(np.mean([float(r[k]) for r in rows]))

    mr = col_mean("zero_rms_ee")
    ms = col_mean("sac_rms_ee")
    zf = col_mean("zero_final_ee")
    sf = col_mean("sac_final_ee")
    zh = col_mean("zero_rms_hf")
    sh = col_mean("sac_rms_hf")

    agg_md = metrics_dir / "aggregate_metrics.md"
    agg_md.write_text(
        "# Aggregate metrics (paired, mean over episodes)\n\n"
        f"- Episodes: {n_ep}, seed_start={args.seed_start}\n"
        f"- Curriculum stage: `{args.curriculum_stage}`\n\n"
        f"| Metric | Zero (VSD) | SAC residual | Δ |\n"
        f"| --- | --- | --- | --- |\n"
        f"| RMS EE | {mr:.6f} | {ms:.6f} | {col_mean('delta_rms_ee'):.6f} |\n"
        f"| Final EE | {zf:.6f} | {sf:.6f} | {col_mean('delta_final_ee'):.6f} |\n"
        f"| RMS HF | {zh:.6f} | {sh:.6f} | {col_mean('delta_rms_hf'):.6f} |\n"
        f"| Smooth | {col_mean('zero_smooth'):.6f} | {col_mean('sac_smooth'):.6f} | {col_mean('delta_smooth'):.6f} |\n"
        f"| Saturation | {col_mean('zero_saturation'):.6f} | {col_mean('sac_saturation'):.6f} | "
        f"{col_mean('sac_saturation') - col_mean('zero_saturation'):.6f} |\n"
        f"| Limit frac. | {col_mean('zero_limit'):.6f} | {col_mean('sac_limit'):.6f} | "
        f"{col_mean('sac_limit') - col_mean('zero_limit'):.6f} |\n\n"
        f"- Percent RMS improvement: {_pct(mr, ms):.4f} %\n"
        f"- Percent final EE improvement: {_pct(zf, sf):.4f} %\n"
        f"- Percent RMS HF improvement: {_pct(zh, sh):.4f} %\n",
        encoding="utf-8",
    )

    try:
        plt.style.use("seaborn-v0_8-whitegrid")
    except OSError:
        pass

    stages_lbl = [args.curriculum_stage]
    x = np.array([0.0])
    w = 0.35
    fig, ax = plt.subplots(figsize=(4, 4))
    ax.bar(x - w / 2, [mr], w, label="zero")
    ax.bar(x + w / 2, [ms], w, label="SAC")
    ax.set_xticks(x)
    ax.set_xticklabels(stages_lbl)
    ax.set_ylabel("RMS EE [m]")
    ax.legend()
    fig.tight_layout()
    fig.savefig(plots_dir / "aggregate_rms_ee_bar.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(4, 4))
    ax.bar(x - w / 2, [zf], w, label="zero")
    ax.bar(x + w / 2, [sf], w, label="SAC")
    ax.set_xticks(x)
    ax.set_xticklabels(stages_lbl)
    ax.set_ylabel("Final EE [m]")
    ax.legend()
    fig.tight_layout()
    fig.savefig(plots_dir / "aggregate_final_ee_bar.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(4, 4))
    ax.bar(x - w / 2, [zh], w, label="zero")
    ax.bar(x + w / 2, [sh], w, label="SAC")
    ax.set_xticks(x)
    ax.set_xticklabels(stages_lbl)
    ax.set_ylabel("RMS HF")
    ax.legend()
    fig.tight_layout()
    fig.savefig(plots_dir / "aggregate_rms_hf_bar.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(4, 4))
    ax.bar(x - w / 2, [col_mean("zero_smooth")], w, label="zero")
    ax.bar(x + w / 2, [col_mean("sac_smooth")], w, label="SAC")
    ax.set_xticks(x)
    ax.set_xticklabels(stages_lbl)
    ax.set_ylabel("Smooth score")
    ax.legend()
    fig.tight_layout()
    fig.savefig(plots_dir / "aggregate_smooth_score_bar.png", dpi=140)
    plt.close(fig)

    for key, fname in (
        ("delta_rms_ee", "delta_rms_histogram.png"),
        ("delta_final_ee", "delta_final_histogram.png"),
        ("delta_rms_hf", "delta_hf_histogram.png"),
    ):
        fig, ax = plt.subplots(figsize=(5, 3))
        ax.hist([float(r[key]) for r in rows], bins=min(20, max(5, n_ep // 5)), color="C0", alpha=0.85)
        ax.axvline(0.0, color="k", lw=0.8)
        ax.set_title(key)
        fig.tight_layout()
        fig.savefig(plots_dir / fname, dpi=140)
        plt.close(fig)

    pd_seed = int(args.plot_detail_seed)
    opts_d: dict[str, Any] = {}
    if do_rnd:
        opts_d["randomize_cable"] = True
        opts_d["cable_seed"] = pd_seed
        opts_d["randomization_profile"] = rnd_profile
    zd = rollout_one_episode_metrics(policy_fn=pol_zero, config=cfg_merged, seed=pd_seed, options=opts_d)
    sd = rollout_one_episode_metrics(policy_fn=pol_sac, config=cfg_merged, seed=pd_seed, options=opts_d)
    pref = f"seed_{pd_seed}"
    save_comparison_plot_set(zd, sd, plots_dir, file_prefix=pref)

    video_stride = max(1, int(args.video_stride))
    if not bool(args.no_videos):
        for vs in args.video_seeds:
            seed_v = int(vs)
            opts_v: dict[str, Any] = {}
            if do_rnd:
                opts_v["randomize_cable"] = True
                opts_v["cable_seed"] = seed_v
                opts_v["randomization_profile"] = rnd_profile
            rz = rollout_one_episode_metrics(
                policy_fn=pol_zero,
                config=cfg_merged,
                seed=seed_v,
                options=opts_v,
                collect_rgb_frames=True,
                rgb_frame_stride=video_stride,
            )
            rs = rollout_one_episode_metrics(
                policy_fn=pol_sac,
                config=cfg_merged,
                seed=seed_v,
                options=opts_v,
                collect_rgb_frames=True,
                rgb_frame_stride=video_stride,
            )
            fz = list(rz.get("rgb_frames") or [])
            fs = list(rs.get("rgb_frames") or [])
            tag = f"seed_{seed_v}"
            write_mp4(fz, videos_dir / f"{tag}_vsd_only.mp4", float(args.video_fps))
            write_mp4(fs, videos_dir / f"{tag}_sac_residual.mp4", float(args.video_fps))
            m = min(len(fz), len(fs))
            if m > 0:
                merged = [np.concatenate([fz[i], fs[i]], axis=1) for i in range(m)]
                write_mp4(merged, videos_dir / f"{tag}_side_by_side.mp4", float(args.video_fps))

    report = out / "final_report.md"
    report.write_text(
        "# Final report: workspace 5D VSD + SAC residual (medium_train)\n\n"
        "## 1. Model\n\n"
        "- `models/pmi_hybrid_no_collision.xml` — **collision disabled** in this pipeline.\n\n"
        "## 2. Controller\n\n"
        "- Nominal **workspace 5D VSD** (not replaced).\n"
        "- Task: **x, y, z, roll, pitch**; **yaw free**.\n"
        "- Desired roll/pitch track initial attitude at episode start.\n\n"
        "## 3. Transmission\n\n"
        "- q1 belt/gear; q2–q4 cable (`HybridTransmission`).\n"
        "- Torque applied **actuator-side** via `qfrc_applied` (not `data.ctrl`).\n\n"
        "## 4. SAC residual\n\n"
        "- **5D task-space wrench** on top of VSD: action = [Fx, Fy, Fz, Mroll, Mpitch].\n"
        "- VSD nominal torques are **not** removed; residual is added.\n\n"
        "## 5. Curriculum\n\n"
        "`deterministic` → `mild` → `medium_v2` → `medium_train`.\n\n"
        "## 6. Final medium_train checkpoint (user-reported reference)\n\n"
        "- RMS EE reduction ≈ **44.54 %**\n"
        "- Final EE reduction ≈ **47.35 %**\n"
        "- RMS HF reduction ≈ **14.91 %**\n"
        "- Saturation / limit: **0 → 0** in reference eval; **ncon max 0**.\n\n"
        f"## 7. This run aggregate (mean over {n_ep} paired episodes)\n\n"
        f"- Percent RMS improvement: {_pct(mr, ms):.4f} %\n"
        f"- Percent final EE improvement: {_pct(zf, sf):.4f} %\n"
        f"- Percent RMS HF improvement: {_pct(zh, sh):.4f} %\n\n"
        "## 8. Outputs\n\n"
        f"- Metrics: `{csv_path}`, `{agg_md}`\n"
        f"- Plots: `{plots_dir}` (including `{pref}_*.png` for detail seed {pd_seed})\n"
        f"- Videos: `{videos_dir}` (`seed_*_vsd_only.mp4`, `_sac_residual.mp4`, `_side_by_side.mp4`)\n\n"
        "## 9. Limitations\n\n"
        "- Simulation-only; real hardware may differ.\n"
        "- **Stress** randomization profile is **evaluation-only** (no training on stress in this project phase).\n"
        "- Cable parameters follow configured **randomization_profiles** in `cable_layer.yaml`.\n"
        "- Yaw is intentionally **uncontrolled**.\n\n"
        "## 10. Next optional step\n\n"
        "- Run **stress evaluation only**, e.g. `evaluate_workspace_5d_residual_sac.py --curriculum-stage stress ...`.\n"
        "- **Do not** train SAC on the stress profile unless explicitly planned.\n",
        encoding="utf-8",
    )

    print(f"Wrote {out} (metrics, plots, videos, final_report.md)")


if __name__ == "__main__":
    main()
