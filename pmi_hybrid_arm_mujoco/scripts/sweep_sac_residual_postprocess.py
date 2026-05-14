#!/usr/bin/env python3
"""잔차 힘 후처리(게인·저역통과·속도제한·종료 페이드) 그리드 짝 평가. 학습은 수행하지 않습니다."""

from __future__ import annotations

import argparse
import copy
import csv
import itertools
import sys
from pathlib import Path
from typing import Any

import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
_PKG = _SCRIPTS.parent
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from stable_baselines3 import SAC

import evaluate_sac_residual as eval_sac
from callbacks.sac_diagnostics_callback import rollout_one_episode_vec
from envs.pmi_cable_residual_env import _deep_merge

CONFIG_DEF = _PKG / "configs" / "rl_sac.yaml"
OUT_DIR_DEF = _PKG / "debug_outputs" / "sac_residual_task_force" / "postprocess_sweep"
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


def gv(d: dict[str, Any], key: str) -> float:
    try:
        return float(d[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def sweep_overrides(
    *,
    residual_gain: float,
    residual_filter_tau: float,
    final_fade_duration: float,
    max_delta_force_per_step: float | None,
) -> dict[str, Any]:
    o: dict[str, Any] = {
        "residual": {"residual_gain": float(residual_gain)},
        "residual_filter": {"enabled": True, "tau": float(residual_filter_tau)},
        "residual_postprocess": {"final_fade_duration": float(final_fade_duration)},
    }
    if max_delta_force_per_step is None:
        o["action_smoothing"] = {"enabled": False}
    else:
        o["action_smoothing"] = {
            "enabled": True,
            "max_delta_force_per_step": float(max_delta_force_per_step),
        }
    return o


def compute_selection_score(
    *,
    mean_delta_ee_rms: float,
    mean_delta_final_ee: float,
    mean_delta_rms_highfreq: float,
    mean_delta_p2p_error_norm: float,
    mean_delta_saturation_frac: float,
    mean_delta_limit_frac: float,
) -> float:
    return (
        float(mean_delta_ee_rms)
        + 1.0 * max(0.0, float(mean_delta_final_ee))
        + 0.5 * max(0.0, float(mean_delta_rms_highfreq))
        + 0.2 * max(0.0, float(mean_delta_p2p_error_norm))
        + 0.1 * max(0.0, float(mean_delta_saturation_frac))
        + 0.1 * max(0.0, float(mean_delta_limit_frac))
    )


def paired_means_for_config(
    *,
    model: SAC,
    sac_policy,
    config_path: Path,
    profile: str,
    vec_normalize_path: Path | None,
    env_overrides: dict[str, Any],
    seeds: list[int],
) -> dict[str, Any]:
    rows_ep: list[dict[str, float]] = []
    for seed in seeds:
        z = rollout_one_episode_vec(
            obs_policy=eval_sac._zero_obs_policy,
            config_path=config_path,
            profile=profile,
            seed=int(seed),
            vec_normalize_path=vec_normalize_path,
            config_overrides=env_overrides,
        )
        s = rollout_one_episode_vec(
            obs_policy=sac_policy,
            config_path=config_path,
            profile=profile,
            seed=int(seed),
            vec_normalize_path=vec_normalize_path,
            config_overrides=env_overrides,
        )
        d_ee = float(s["rms_ee"]) - float(z["rms_ee"])
        d_fe = float(s["final_ee_error"]) - float(z["final_ee_error"])
        d_mx = float(s["max_ee_error"]) - float(z["max_ee_error"])
        d_ev = gv(s, "rms_ee_error_velocity") - gv(z, "rms_ee_error_velocity")
        d_hf = gv(s, "rms_ee_error_highfreq") - gv(z, "rms_ee_error_highfreq")
        d_p2p = gv(s, "p2p_error_norm") - gv(z, "p2p_error_norm")
        d_sat = float(s["sat_frac"]) - float(z["sat_frac"])
        d_lim = float(s["lim_frac"]) - float(z["lim_frac"])
        d_rw = float(s["episode_return"]) - float(z["episode_return"])
        rows_ep.append(
            {
                "delta_ee_rms": d_ee,
                "delta_final_ee": d_fe,
                "delta_max_ee": d_mx,
                "delta_rms_ee_error_velocity": d_ev,
                "delta_rms_highfreq": d_hf,
                "delta_p2p_error_norm": d_p2p,
                "delta_saturation_frac": d_sat,
                "delta_limit_frac": d_lim,
                "delta_reward": d_rw,
                "ncon_sac": float(s["ncon_max"]),
            }
        )

    n = max(1, len(rows_ep))
    d_ee_a = np.asarray([r["delta_ee_rms"] for r in rows_ep], dtype=np.float64)
    improved = int(np.sum(d_ee_a < 0))
    def mean_key(k: str) -> float:
        return float(np.nanmean(np.asarray([float(r[k]) for r in rows_ep], dtype=np.float64)))

    return {
        "mean_delta_ee_rms": mean_key("delta_ee_rms"),
        "mean_delta_final_ee": mean_key("delta_final_ee"),
        "mean_delta_max_ee": mean_key("delta_max_ee"),
        "mean_delta_rms_ee_error_velocity": mean_key("delta_rms_ee_error_velocity"),
        "mean_delta_rms_highfreq": mean_key("delta_rms_highfreq"),
        "mean_delta_p2p_error_norm": mean_key("delta_p2p_error_norm"),
        "mean_delta_saturation_frac": mean_key("delta_saturation_frac"),
        "mean_delta_limit_frac": mean_key("delta_limit_frac"),
        "mean_delta_reward": mean_key("delta_reward"),
        "improvement_ratio": improved / float(n),
        "ncon_max": int(np.nanmax(np.asarray([r["ncon_sac"] for r in rows_ep], dtype=np.float64))),
    }


def _md_table(headers: list[str], raws: list[dict[str, Any]], keys: list[str]) -> str:
    lines = ["| " + " | ".join(headers) + " |", "| " + " | ".join(["---"] * len(headers)) + " |"]
    for r in raws:
        cells = []
        for k in keys:
            v = r.get(k, "")
            if isinstance(v, float):
                cells.append(f"{v:.6g}")
            else:
                cells.append(str(v))
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def plot_best_setting_episode(
    *,
    plot_dir: Path,
    seed: int,
    model: SAC,
    sac_policy,
    config_path: Path,
    profile: str,
    vec_normalize_path: Path | None,
    env_overrides: dict[str, Any],
) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[sweep_sac_residual_postprocess] matplotlib 미설치로 플롯 생략.", file=sys.stderr)
        return

    plot_dir.mkdir(parents=True, exist_ok=True)
    z = rollout_one_episode_vec(
        obs_policy=eval_sac._zero_obs_policy,
        config_path=config_path,
        profile=profile,
        seed=int(seed),
        vec_normalize_path=vec_normalize_path,
        config_overrides=env_overrides,
    )
    s = rollout_one_episode_vec(
        obs_policy=sac_policy,
        config_path=config_path,
        profile=profile,
        seed=int(seed),
        vec_normalize_path=vec_normalize_path,
        config_overrides=env_overrides,
    )

    t_z = np.asarray(z["t_control"], dtype=np.float64)
    t_s = np.asarray(s["t_control"], dtype=np.float64)
    ee_z = np.asarray(z["ee_err_xyz_series"], dtype=np.float64)
    ee_s = np.asarray(s["ee_err_xyz_series"], dtype=np.float64)
    n_z = np.linalg.norm(ee_z, axis=1)
    n_s = np.linalg.norm(ee_s, axis=1)

    # --- EE error components + norm ---
    fig, axes = plt.subplots(2, 2, figsize=(9, 6), sharex=True)
    labels_xyz = ["x", "y", "z"]
    for i in range(3):
        ax = axes.flat[i]
        ax.plot(t_z, ee_z[:, i], label=f"VSD only ({labels_xyz[i]})", color="#4477aa", lw=1.2)
        ax.plot(t_s, ee_s[:, i], label=f"SAC+pp ({labels_xyz[i]})", color="#cc8844", lw=1.2)
        ax.set_ylabel(f"e_{labels_xyz[i]} (m)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)
    axn = axes.flat[3]
    axn.plot(t_z, n_z, label="||e|| VSD", color="#4477aa", lw=1.2)
    axn.plot(t_s, n_s, label="||e|| SAC+pp", color="#cc8844", lw=1.2)
    axn.set_ylabel("||e|| (m)")
    axn.set_xlabel("time (s)")
    axn.grid(True, alpha=0.3)
    axn.legend(fontsize=7)
    fig.suptitle(f"EE error (seed={seed})")
    fig.tight_layout()
    fig.savefig(plot_dir / f"ee_err_xyz_norm_seed{seed}.png", dpi=150)
    plt.close(fig)

    # --- Residual forces ---
    Fr = np.asarray(s["F_residual_raw_series"], dtype=np.float64)
    Ff = np.asarray(s["F_residual_filtered_series"], dtype=np.float64)
    Fu = np.asarray(s["F_residual_used_series"], dtype=np.float64)
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t_s, np.linalg.norm(Fr, axis=1), label="||F_raw||", lw=1.1)
    ax.plot(t_s, np.linalg.norm(Ff, axis=1), label="||F_filtered||", lw=1.1)
    ax.plot(t_s, np.linalg.norm(Fu, axis=1), label="||F_used||", lw=1.1)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("force norm (N)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title(f"Residual force norms (seed={seed})")
    fig.tight_layout()
    fig.savefig(plot_dir / f"residual_force_norms_seed{seed}.png", dpi=150)
    plt.close(fig)

    # --- Gate ---
    g = np.asarray(s["residual_gate_series"], dtype=np.float64)
    fig, ax = plt.subplots(figsize=(8, 2.5))
    ax.plot(t_s, g, color="#333333", lw=1.5)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("residual_gate")
    ax.set_ylim(-0.05, 1.05)
    ax.grid(True, alpha=0.3)
    ax.set_title(f"Terminal fade gate (seed={seed})")
    fig.tight_layout()
    fig.savefig(plot_dir / f"residual_gate_seed{seed}.png", dpi=150)
    plt.close(fig)

    # --- High-frequency EE error norm ---
    ehf_z = np.asarray(z["ee_err_highfreq_xyz_series"], dtype=np.float64)
    ehf_s = np.asarray(s["ee_err_highfreq_xyz_series"], dtype=np.float64)
    fig, ax = plt.subplots(figsize=(8, 3))
    ax.plot(t_z, np.linalg.norm(ehf_z, axis=1), label="||e_hf|| VSD", color="#4477aa", lw=1.1)
    ax.plot(t_s, np.linalg.norm(ehf_s, axis=1), label="||e_hf|| SAC+pp", color="#cc8844", lw=1.1)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("||e_hf|| (m)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title(f"High-frequency EE error (seed={seed})")
    fig.tight_layout()
    fig.savefig(plot_dir / f"ee_err_highfreq_norm_seed{seed}.png", dpi=150)
    plt.close(fig)

    # --- EE error velocity norm ---
    ed_z = np.asarray(z["ee_dot_xyz_series"], dtype=np.float64)
    ed_s = np.asarray(s["ee_dot_xyz_series"], dtype=np.float64)
    fig, ax = plt.subplots(figsize=(8, 3))
    ax.plot(t_z, np.linalg.norm(ed_z, axis=1), label="||ė|| VSD", color="#4477aa", lw=1.1)
    ax.plot(t_s, np.linalg.norm(ed_s, axis=1), label="||ė|| SAC+pp", color="#cc8844", lw=1.1)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("||ė||")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title(f"EE error velocity norm (seed={seed})")
    fig.tight_layout()
    fig.savefig(plot_dir / f"ee_err_velocity_norm_seed{seed}.png", dpi=150)
    plt.close(fig)

    # --- Tau residual / total ---
    tr = np.asarray(s["tau_residual_series"], dtype=np.float64)
    tt = np.asarray(s["tau_total_series"], dtype=np.float64)
    fig, ax = plt.subplots(figsize=(8, 3))
    ax.plot(t_s, np.linalg.norm(tr, axis=1), label="||τ_residual||", color="#226622", lw=1.1)
    ax.plot(t_s, np.linalg.norm(tt, axis=1), label="||τ_total||", color="#662222", lw=1.1, alpha=0.85)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("torque norm (N·m)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title(f"Residual vs total joint torque (seed={seed})")
    fig.tight_layout()
    fig.savefig(plot_dir / f"tau_residual_vs_total_seed{seed}.png", dpi=150)
    plt.close(fig)

    # --- 3D path ---
    des_z = np.asarray(z["ee_des_xyz_series"], dtype=np.float64)
    act_z = np.asarray(z["ee_act_xyz_series"], dtype=np.float64)
    act_s = np.asarray(s["ee_act_xyz_series"], dtype=np.float64)
    fig = plt.figure(figsize=(6, 5))
    ax3 = fig.add_subplot(111, projection="3d")
    ax3.plot(des_z[:, 0], des_z[:, 1], des_z[:, 2], "k--", lw=1.0, label="desired")
    ax3.plot(act_z[:, 0], act_z[:, 1], act_z[:, 2], color="#4477aa", lw=1.2, label="actual VSD")
    ax3.plot(act_s[:, 0], act_s[:, 1], act_s[:, 2], color="#cc8844", lw=1.2, label="actual SAC+pp")
    ax3.set_xlabel("x")
    ax3.set_ylabel("y")
    ax3.set_zlabel("z")
    ax3.legend(fontsize=7)
    ax3.set_title(f"EE path 3D (seed={seed})")
    fig.tight_layout()
    fig.savefig(plot_dir / f"path3d_seed{seed}.png", dpi=150)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description="SAC residual post-process sweep (eval only).")
    ap.add_argument("--config", type=Path, default=CONFIG_DEF)
    ap.add_argument("--profile", type=str, default="medium_train")
    ap.add_argument("--model-path", type=Path, default=CKPT_DEF)
    ap.add_argument("--vecnormalize-path", type=Path, default=VN_DEF)
    ap.add_argument("--run-dir", type=Path, default=None)
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR_DEF)
    ap.add_argument("--seed-start", type=int, default=10000)
    ap.add_argument("--num-episodes", type=int, default=30)
    ap.add_argument("--plot-seed-end", type=int, default=10004, help="inclusive high seed for best-setting plots")
    args = ap.parse_args()

    model_path = args.model_path.expanduser().resolve()
    vn_path = args.vecnormalize_path.expanduser().resolve()
    cfg_path = args.config.expanduser().resolve()
    out_dir = args.out_dir.expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "postprocess_sweep_results.csv"
    md_path = out_dir / "postprocess_sweep_report.md"

    model = eval_sac.load_sac_maybe(model_path)
    if model is None:
        raise SystemExit(f"체크포인트를 불러올 수 없습니다: {model_path}")

    run_dir = eval_sac.infer_run_dir(model_path, args.run_dir.expanduser().resolve() if args.run_dir else None)
    base_o = eval_sac.build_paired_overrides(str(args.profile), run_dir)

    if not vn_path.is_file():
        print(f"[경고] vecnormalize.pkl 없음: {vn_path}", file=sys.stderr)

    def sac_policy(obs: np.ndarray) -> np.ndarray:
        a, _ = model.predict(obs, deterministic=True)
        return np.asarray(a, dtype=np.float32)

    gains = [0.5, 0.75, 1.0]
    taus = [0.03, 0.05, 0.08, 0.12]
    fades = [0.0, 0.3, 0.5, 0.8]
    slew_opts: list[float | None] = [None, 0.2, 0.3]

    seeds = [int(args.seed_start) + k for k in range(int(args.num_episodes))]

    grid = list(itertools.product(gains, taus, fades, slew_opts))
    all_rows: list[dict[str, Any]] = []

    for gi, (g, tau, fade, slew) in enumerate(grid):
        swo = sweep_overrides(
            residual_gain=g,
            residual_filter_tau=tau,
            final_fade_duration=fade,
            max_delta_force_per_step=slew,
        )
        merged = _deep_merge(copy.deepcopy(base_o), swo)
        print(f"[{gi + 1}/{len(grid)}] gain={g} tau={tau} fade={fade} slew={slew!s}", flush=True)
        stats = paired_means_for_config(
            model=model,
            sac_policy=sac_policy,
            config_path=cfg_path,
            profile=str(args.profile),
            vec_normalize_path=vn_path if vn_path.is_file() else None,
            env_overrides=merged,
            seeds=seeds,
        )
        slew_str = "none" if slew is None else str(float(slew))
        row: dict[str, Any] = {
            "residual_gain": float(g),
            "residual_filter_tau": float(tau),
            "final_fade_duration": float(fade),
            "max_delta_force_per_step": slew_str,
            **stats,
            "selection_score": compute_selection_score(
                mean_delta_ee_rms=stats["mean_delta_ee_rms"],
                mean_delta_final_ee=stats["mean_delta_final_ee"],
                mean_delta_rms_highfreq=stats["mean_delta_rms_highfreq"],
                mean_delta_p2p_error_norm=stats["mean_delta_p2p_error_norm"],
                mean_delta_saturation_frac=stats["mean_delta_saturation_frac"],
                mean_delta_limit_frac=stats["mean_delta_limit_frac"],
            ),
        }
        all_rows.append(row)

    fieldnames = [
        "residual_gain",
        "residual_filter_tau",
        "final_fade_duration",
        "max_delta_force_per_step",
        "mean_delta_ee_rms",
        "mean_delta_final_ee",
        "mean_delta_max_ee",
        "mean_delta_rms_ee_error_velocity",
        "mean_delta_rms_highfreq",
        "mean_delta_p2p_error_norm",
        "mean_delta_saturation_frac",
        "mean_delta_limit_frac",
        "mean_delta_reward",
        "improvement_ratio",
        "ncon_max",
        "selection_score",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as fcsv:
        w = csv.DictWriter(fcsv, fieldnames=fieldnames)
        w.writeheader()
        for r in all_rows:
            w.writerow({k: r.get(k, "") for k in fieldnames})

    # 선택: ncon_max==0 인 설정 중 최소 점수, 없으면 전체 최소.
    feasible = [r for r in all_rows if int(r["ncon_max"]) == 0]
    pool = feasible if feasible else list(all_rows)
    best = min(pool, key=lambda r: float(r["selection_score"]))

    top_score = sorted(all_rows, key=lambda r: float(r["selection_score"]))[:10]
    top_rms = sorted(all_rows, key=lambda r: float(r["mean_delta_ee_rms"]))[:10]
    top_fe = sorted(all_rows, key=lambda r: float(r["mean_delta_final_ee"]))[:10]

    tab_hdr = ["gain", "τ_filt", "fade", "slew", "sel_score", "ΔRMS", "ΔfinalEE", "ncon"]

    def pack_tab(rlist: list[dict[str, Any]]) -> list[dict[str, Any]]:
        out = []
        for r in rlist:
            out.append(
                {
                    "gain": r["residual_gain"],
                    "tau": r["residual_filter_tau"],
                    "fade": r["final_fade_duration"],
                    "slew": r["max_delta_force_per_step"],
                    "score": float(r["selection_score"]),
                    "d_rms": float(r["mean_delta_ee_rms"]),
                    "d_fe": float(r["mean_delta_final_ee"]),
                    "ncon": int(r["ncon_max"]),
                }
            )
        return out

    def mean_d_rms(pred) -> float:
        sub = [r for r in all_rows if pred(r)]
        if not sub:
            return float("nan")
        return float(np.mean([float(x["mean_delta_ee_rms"]) for x in sub]))

    fade_pos = [r for r in all_rows if float(r["final_fade_duration"]) > 0]
    fade_zero = [r for r in all_rows if float(r["final_fade_duration"]) <= 0]

    mean_final_fade = float(np.mean([float(r["mean_delta_final_ee"]) for r in fade_pos])) if fade_pos else float("nan")
    mean_final_nofade = float(np.mean([float(r["mean_delta_final_ee"]) for r in fade_zero])) if fade_zero else float("nan")
    mean_hf_filt = float(np.mean([float(r["mean_delta_rms_highfreq"]) for r in all_rows]))
    mean_hf_lowtau = float(
        np.mean([float(r["mean_delta_rms_highfreq"]) for r in all_rows if float(r["residual_filter_tau"]) <= 0.05])
    )
    mean_hf_hightau = float(
        np.mean([float(r["mean_delta_rms_highfreq"]) for r in all_rows if float(r["residual_filter_tau"]) >= 0.08])
    )

    # --- Report ---
    lines = [
        "# SAC 잔차 후처리 스윕 결과",
        "",
        "## 1. 평가 체크포인트",
        "",
        f"- `{model_path}`",
        "",
        "## 2. 프로파일",
        "",
        f"- `{args.profile}`",
        "",
        "## 3. 시드 범위",
        "",
        f"- `seed_start`: {int(args.seed_start)}",
        f"- `num_episodes`: {int(args.num_episodes)} (시드 {seeds[0]} … {seeds[-1]})",
        "",
        "## 4. 스윕 그리드",
        "",
        f"- `residual_gain`: {gains}",
        f"- `residual_filter_tau`: {taus} (필터 항상 활성)",
        f"- `final_fade_duration`: {fades}",
        f"- `max_delta_force_per_step`: none, 0.2, 0.3",
        f"- 총 조합 수: {len(grid)}",
        "",
        "## 5. selection_score 기준 상위 10",
        "",
        "낮을수록 좋음 (음수가 이상적).",
        "",
        _md_table(tab_hdr, pack_tab(top_score), ["gain", "tau", "fade", "slew", "score", "d_rms", "d_fe", "ncon"]),
        "",
        "## 6. mean_delta_ee_rms 기준 상위 10 (가장 음수에 가까움)",
        "",
        _md_table(tab_hdr, pack_tab(top_rms), ["gain", "tau", "fade", "slew", "score", "d_rms", "d_fe", "ncon"]),
        "",
        "## 7. mean_delta_final_ee 기준 상위 10 (가장 음수에 가까움)",
        "",
        _md_table(tab_hdr, pack_tab(top_fe), ["gain", "tau", "fade", "slew", "score", "d_rms", "d_fe", "ncon"]),
        "",
        "## 8. 권장 설정",
        "",
        f"- `residual_gain`: **{best['residual_gain']}**",
        f"- `residual_filter_tau`: **{best['residual_filter_tau']}**",
        f"- `final_fade_duration`: **{best['final_fade_duration']}**",
        f"- `max_delta_force_per_step`: **{best['max_delta_force_per_step']}**",
        "",
        f"- 집계: `mean_delta_ee_rms`={float(best['mean_delta_ee_rms']):.6g}, "
        f"`mean_delta_final_ee`={float(best['mean_delta_final_ee']):.6g}, "
        f"`selection_score`={float(best['selection_score']):.6g}, `ncon_max`={int(best['ncon_max'])}.",
        "",
        "**스윕 결과에 대한 요약 질문**",
        "",
        "### 1) 후처리가 RMS 개선을 유지하는가?",
        f"- 그리드 전체에 대해 각 조합의 `mean_delta_ee_rms`를 다시 평균한 값: `{mean_d_rms(lambda r: True):.6g}` (후보 간 요약 참고치).",
        f"- **권장 조합**의 짝평균 RMS 델타: `mean_delta_ee_rms={float(best['mean_delta_ee_rms']):.6g}` (Δ<0이면 VSD 대비 SAC+후처리가 RMS 우위).",
        "**해석**: 권장 조합에서 `mean_delta_ee_rms`<0 이면 후처리를 적용해도 RMS 개선이 유지되는 것입니다.",
        "",
        "### 2) 종료 페이드가 final EE 오차를 줄이는가?",
        f"- 페이드>0 평균 Δfinal EE: `{mean_final_fade:.6g}` (n={len(fade_pos)}), 페이드=0: `{mean_final_nofade:.6g}` (n={len(fade_zero)}). ",
        "**해석**: 전자가 더 작을수록(더 음수) 페이드가 종료 오차 저감에 유리합니다.",
        "",
        "### 3) 필터링이 고주파(진동성) EE 오차를 줄이는가?",
        f"- τ≤0.05 조합 평균 Δ고주파: `{mean_hf_lowtau:.6g}`, τ≥0.08: `{mean_hf_hightau:.6g}`. ",
        "**해석**: 더 음수 쪽이 고주파 오차 저감에 유리할 수 있습니다(전체 평균 `{mean_hf_filt:.6g}`).",
        "",
        "### 4) 힘 변화율 제한(rate limit)이 도움이 되는가?",
        f"- 속도 제한 ON 조합 평균 ΔRMS: `{mean_d_rms(lambda r: r['max_delta_force_per_step']!='none'):.6g}`, "
        f"OFF: `{mean_d_rms(lambda r: r['max_delta_force_per_step']=='none'):.6g}`. ",
        "**해석**: ΔRMS가 더 음수인 쪽이 추적 RMS에 유리할 수 있습니다.",
        "",
        "### 5) 동영상·향후 학습에 어떤 설정을 쓸까?",
        f"- 보고된 **권장 조합**(위 표) 또는 `selection_score` 최소 조합으로 통일하고, 필요 시 RMS·종료 EE를 가중 균형하기 위해 근처 τ/페이드/게인 후보 몇 개를 검토하세요.",
        "",
        "---",
        f"- CSV: `{csv_path}`",
        f"- 권장 설정 플롯: `{out_dir / 'plots' / 'best_setting'}`",
    ]

    md_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote CSV: {csv_path}")
    print(f"Wrote report: {md_path}")

    # Best-setting plots seeds 10000..10004
    swo_b = sweep_overrides(
        residual_gain=float(best["residual_gain"]),
        residual_filter_tau=float(best["residual_filter_tau"]),
        final_fade_duration=float(best["final_fade_duration"]),
        max_delta_force_per_step=(None if best["max_delta_force_per_step"] == "none" else float(best["max_delta_force_per_step"])),
    )
    merged_b = _deep_merge(copy.deepcopy(base_o), swo_b)
    plot_dir = out_dir / "plots" / "best_setting"
    for sd in range(int(args.seed_start), int(args.plot_seed_end) + 1):
        plot_best_setting_episode(
            plot_dir=plot_dir,
            seed=sd,
            model=model,
            sac_policy=sac_policy,
            config_path=cfg_path,
            profile=str(args.profile),
            vec_normalize_path=vn_path if vn_path.is_file() else None,
            env_overrides=merged_b,
        )
    print(f"Plots saved under: {plot_dir}")


if __name__ == "__main__":
    main()
