#!/usr/bin/env python3
"""평가: 잔차 0 베이스라인 대비 학습된 SAC 작업공간 잔차 힘 정책.

짝 평가 예시 (학습 런 로그 옆에 CSV·보고서·플롯 저장; ``--paired-out`` 이 CSV 파일이면 같은 디렉터리에 보고서 생성)::

    python scripts/evaluate_sac_residual.py \\
      --paired-seeds \\
      --model-path debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_30k_s2/checkpoints/best_model_by_ee_rms.zip \\
      --vecnormalize-path debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_30k_s2/vecnormalize/vecnormalize.pkl \\
      --config configs/rl_sac.yaml \\
      --profile medium_train \\
      --seed-start 10000 \\
      --num-episodes 30 \\
      --paired-out debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_30k_s2/logs/paired_evaluation.csv
"""

from __future__ import annotations

import argparse
import copy
import csv
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

import numpy as np
import yaml

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from stable_baselines3 import SAC

from callbacks.sac_diagnostics_callback import rollout_one_episode_vec
from envs.pmi_cable_residual_env import PMICableResidualEnv, _deep_merge

OUT_DEF = _ROOT / "debug_outputs" / "sac_residual_task_force"

PAIRED_EPILOG = r"""
Paired evaluation example:
  python scripts/evaluate_sac_residual.py \
    --paired-seeds \
    --model-path debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_30k_s2/checkpoints/best_model_by_ee_rms.zip \
    --vecnormalize-path debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_30k_s2/vecnormalize/vecnormalize.pkl \
    --config configs/rl_sac.yaml \
    --profile medium_train \
    --seed-start 10000 \
    --num-episodes 30 \
    --paired-out debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_30k_s2/logs/paired_evaluation.csv
"""


def load_sac_maybe(p: Path | None) -> SAC | None:
    if p is None:
        return None
    rf = Path(p)
    zp = rf if rf.suffix == ".zip" else rf.with_suffix(".zip")
    if not zp.is_file():
        return None
    return SAC.load(str(zp))


def infer_run_dir(model_path: Path, explicit: Path | None) -> Path | None:
    if explicit is not None:
        return Path(explicit).expanduser().resolve()
    p = Path(model_path).resolve()
    if p.parent.name == "checkpoints":
        return p.parent.parent
    return None


def resolve_paired_paths(
    paired_out: Path | None,
    *,
    run_dir: Path | None,
    model_path: Path,
) -> tuple[Path, Path, Path]:
    """Return ``csv_path``, ``md_path``, ``plots_dir`` (under ``csv_path.parent / paired_plots``)."""
    if paired_out is not None:
        po = Path(paired_out).expanduser().resolve()
        if po.suffix.lower() == ".csv":
            po.parent.mkdir(parents=True, exist_ok=True)
            csv_path = po
            md_path = po.parent / "paired_evaluation_report.md"
        else:
            po.mkdir(parents=True, exist_ok=True)
            csv_path = po / "paired_evaluation.csv"
            md_path = po / "paired_evaluation_report.md"
    else:
        rd = run_dir
        if rd is None:
            raise SystemExit(
                "[evaluate_sac_residual] 짝 평가에서 ``--paired-out`` 이 없으면 ``--model-path`` 가 "
                ".../runs/<name>/checkpoints/*.zip 형태이거나 ``--run-dir`` 로 런 디렉터리를 지정해야 합니다."
            )
        logs = rd / "logs"
        logs.mkdir(parents=True, exist_ok=True)
        csv_path = logs / "paired_evaluation.csv"
        md_path = logs / "paired_evaluation_report.md"

    plots_dir = csv_path.parent / "paired_plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    return csv_path, md_path, plots_dir


def build_paired_overrides(profile: str, run_dir: Path | None) -> dict[str, Any]:
    """Merge ``logs/training_args.yaml`` ``rl_overrides`` when present (tau_jnt_limit, residual scale, reward, …)."""
    base: dict[str, Any] = {"env": {"randomization_profile": str(profile), "randomize_cable": True}}
    if run_dir is None:
        return base
    tap = Path(run_dir) / "logs" / "training_args.yaml"
    if not tap.is_file():
        return base
    try:
        raw = yaml.safe_load(tap.read_text(encoding="utf-8"))
    except Exception:
        return base
    ro = raw.get("rl_overrides") if isinstance(raw, dict) else None
    if not isinstance(ro, dict):
        return base
    return _deep_merge(copy.deepcopy(base), ro)


def rollout_once(
    env: PMICableResidualEnv,
    policy: Callable[[np.ndarray], np.ndarray],
    *,
    seed: int,
    options: dict[str, Any],
) -> dict[str, float]:
    obs, _ = env.reset(seed=seed, options=options)
    finite = np.isfinite(obs).all()

    ee: list[float] = []
    fn: list[float] = []
    tn: list[float] = []
    fr: list[float] = []
    f_prev = np.zeros(3, dtype=float)

    qe: list[float] = []
    reward_sum = 0.0
    sat_tot = jl_steps = al_steps = lim_step = ncm = 0
    n_steps = 0

    truncated = terminated = False
    while True:
        act = policy(obs)
        obs, r, terminated, truncated, inf = env.step(act)
        n_steps += 1
        reward_sum += float(r)
        finite = finite and np.isfinite(obs).all() and np.isfinite(float(r))

        F = np.asarray(inf["F_residual_xyz"], dtype=float).reshape(3)

        tau = np.asarray(inf["tau_residual_jnt"], dtype=float).reshape(4)

        fr.append(float(np.linalg.norm(F - f_prev)))

        f_prev = F.copy()
        fn.append(float(np.linalg.norm(F)))
        tn.append(float(np.linalg.norm(tau)))
        ee.append(float(inf.get("ee_error_norm", np.nan)))
        qe.append(float(inf.get("q_error_norm", np.nan)))

        sat_tot += int(inf.get("saturation_count") or 0)
        jv = int(inf.get("joint_limit_violation") or 0)
        av = int(inf.get("actuator_limit_violation") or 0)
        jl_steps += jv

        al_steps += av

        if jv or av:
            lim_step += 1

        ncm = max(ncm, int(inf.get("ncon") or 0))

        if terminated or truncated:
            break

    eea = np.asarray(ee, dtype=float)
    qea = np.asarray(qe, dtype=float)
    ns = max(1, n_steps)
    return {
        "n_steps": float(n_steps),
        "rms_ee": float(np.sqrt(np.mean(eea**2))) if eea.size else float("nan"),
        "final_ee": float(eea[-1]) if eea.size else float("nan"),
        "max_ee": float(np.max(eea)) if eea.size else float("nan"),
        "rms_q_error": float(np.sqrt(np.mean(qea**2))) if qea.size else float("nan"),

        "total_reward": reward_sum,

        "sat_steps": float(sat_tot),
        "lim_steps": float(lim_step),
        "sat_frac": float(sat_tot / ns),
        "lim_frac": float(lim_step / ns),
        "ncon_max": float(ncm),

        "mean_residual_force_norm": float(np.mean(fn)) if fn else float("nan"),
        "mean_residual_force_rate": float(np.mean(fr)) if fr else float("nan"),
        "mean_residual_torque_norm": float(np.mean(tn)) if tn else float("nan"),

        "finite": float(finite),

        "truncated": float(truncated),
        "terminated": float(terminated),
    }


def _zero_obs_policy(obs: np.ndarray) -> np.ndarray:
    return np.zeros((obs.shape[0], 3), dtype=np.float32)


def _classify_paired(mean_delta_ee: float, improvement_ratio: float, ee_zero_mean: float) -> str:
    tol_abs = 5e-6
    tol_rel = abs(0.005 * ee_zero_mean) if ee_zero_mean == ee_zero_mean else 0.0
    tol = max(tol_abs, tol_rel)
    if np.isfinite(mean_delta_ee) and abs(mean_delta_ee) < tol:
        return "no_clear_difference"
    if mean_delta_ee < 0 and improvement_ratio > 0.6:
        return "tracking_improved"
    if mean_delta_ee < 0 and improvement_ratio > 0.5:
        return "weak_tracking_improvement"
    return "tracking_not_improved"


def _final_error_pair_classification(mean_delta_ee: float, mean_delta_final: float, mean_fe_zero: float) -> str | None:
    """EE RMS 평균 개선(SAC 우위)인데 종료 시점 EE만 악화된 경우."""
    ftol = max(5e-7, 0.002 * abs(mean_fe_zero)) if mean_fe_zero == mean_fe_zero else 5e-7
    if (
        np.isfinite(mean_delta_ee)
        and np.isfinite(mean_delta_final)
        and mean_delta_ee < 0
        and mean_delta_final > ftol
    ):
        return "tracking_improved_but_final_error_worse"
    return None


def _osc_tol(mean_abs: float) -> float:
    return max(1e-9, abs(1e-3 * mean_abs)) if mean_abs == mean_abs else 1e-9

def _paired_multilabel_tags(
    *,
    mean_delta_ee: float,
    improvement_ratio: float,
    mean_delta_final: float,
    mean_delta_hf: float,
    mean_delta_ev: float,
    mean_fe_zero: float,
    ee_zero_mean: float,
) -> list[str]:
    tol_ee = max(5e-6, abs(0.005 * ee_zero_mean)) if ee_zero_mean == ee_zero_mean else 5e-6
    ftol = max(5e-7, 0.002 * abs(mean_fe_zero)) if mean_fe_zero == mean_fe_zero else 5e-7
    tol_os = max(_osc_tol(mean_delta_hf), _osc_tol(mean_delta_ev))
    tags: list[str] = []
    if np.isfinite(mean_delta_ee) and mean_delta_ee < -tol_ee and improvement_ratio > 0.6:
        tags.append("tracking_improved")
    if np.isfinite(mean_delta_ee) and mean_delta_ee < -tol_ee and np.isfinite(mean_delta_final) and mean_delta_final > ftol:
        tags.append("tracking_improved_but_final_error_worse")
    if np.isfinite(mean_delta_ee) and mean_delta_ee < -tol_ee and (
        (np.isfinite(mean_delta_hf) and mean_delta_hf > tol_os) or (np.isfinite(mean_delta_ev) and mean_delta_ev > tol_os)
    ):
        tags.append("tracking_improved_but_more_oscillatory")
    if (
        np.isfinite(mean_delta_ee)
        and mean_delta_ee < -tol_ee
        and np.isfinite(mean_delta_final)
        and mean_delta_final <= ftol
        and np.isfinite(mean_delta_hf)
        and mean_delta_hf <= tol_os
        and np.isfinite(mean_delta_ev)
        and mean_delta_ev <= tol_os
    ):
        tags.append("smooth_tracking_improved")
    return tags


def _paired_joint_smooth_classification(
    *,
    mean_delta_ee: float,
    mean_delta_final: float,
    mean_fe_zero: float,
    ee_zero_mean: float,
    mean_delta_ev: float,
    mean_delta_hf: float,
    mean_delta_p2p: float,
    mean_delta_sat: float,
    mean_delta_lim: float,
    mean_sat_zero: float,
    mean_lim_zero: float,
) -> str:
    """paired CSV 평균으로 baseline-relative 학습 목표 성공 여부 등을 라벨링."""
    tol_ee = max(5e-6, abs(0.005 * ee_zero_mean)) if ee_zero_mean == ee_zero_mean else 5e-6
    ftol = max(5e-6, 0.01 * abs(mean_fe_zero)) if mean_fe_zero == mean_fe_zero else 5e-6
    osc_tol_ev = max(8e-4, abs(5e-3 * ee_zero_mean)) if ee_zero_mean == ee_zero_mean else 8e-4
    osc_tol_hf = max(8e-4, abs(5e-3 * ee_zero_mean)) if ee_zero_mean == ee_zero_mean else 8e-4
    osc_tol_p2p = max(8e-4, abs(5e-3 * ee_zero_mean)) if ee_zero_mean == ee_zero_mean else 8e-4

    sz = float(mean_sat_zero) if np.isscalar(mean_sat_zero) else float(np.nanmean(np.asarray(mean_sat_zero)))
    lz = float(mean_lim_zero) if np.isscalar(mean_lim_zero) else float(np.nanmean(np.asarray(mean_lim_zero)))
    sat_tol = max(5e-4, 0.2 * sz) if sz == sz else 5e-4
    lim_tol = max(5e-4, 0.3 * lz) if lz == lz else 5e-4

    if not np.isfinite(mean_delta_ee):
        return "inconclusive"

    tracking_worse_sig = np.isfinite(mean_delta_ee) and mean_delta_ee > tol_ee
    osc_worse_sig = (
        np.isfinite(mean_delta_ev) and mean_delta_ev > osc_tol_ev * 3.0
    ) or (np.isfinite(mean_delta_hf) and mean_delta_hf > osc_tol_hf * 3.0)
    succ = (
        mean_delta_ee < -tol_ee
        and np.isfinite(mean_delta_final)
        and mean_delta_final <= ftol
        and np.isfinite(mean_delta_ev)
        and mean_delta_ev <= osc_tol_ev
        and np.isfinite(mean_delta_hf)
        and mean_delta_hf <= osc_tol_hf
        and np.isfinite(mean_delta_p2p)
        and mean_delta_p2p <= osc_tol_p2p
        and np.isfinite(mean_delta_sat)
        and mean_delta_sat <= sat_tol
        and np.isfinite(mean_delta_lim)
        and mean_delta_lim <= lim_tol
    )

    if succ:
        return "baseline_relative_success"
    if mean_delta_ee < -tol_ee:
        return "rms_only_improved"
    if tracking_worse_sig or osc_worse_sig:
        return "failed"
    return "ambiguous"


def _recommendation_for(classification: str, *, final_err_cls: str | None) -> str:
    parts: list[str] = []
    if classification == "tracking_improved":
        parts.append(
            "EE RMS가 짝 평가에서 일관되게 개선되었습니다. 더 긴 학습 또는 `medium` 등 다른 프로파일에서 추가 평가를 검토할 수 있습니다."
        )
    elif classification == "weak_tracking_improvement":
        parts.append(
            "약한 개선입니다. 학습 길이·시드를 바꿔 재평가하거나 보상 가중치를 미세 조정하세요."
        )
    elif classification == "no_clear_difference":
        parts.append(
            "평균 차이가 매우 작습니다. 보상 스케일·액션(`residual_force_scale`)·관측 정규화를 조정한 뒤 다시 짝 평가하세요."
        )
    else:
        parts.append(
            "이 체크포인트로 학습 스텝만 늘리기보다 보상·잔차 스케일을 조정하고 다른 체크포인트를 검토하는 편이 낫습니다."
        )

    if final_err_cls == "tracking_improved_but_final_error_worse":
        parts.append(
            "**종료 시점 EE:** 평균 EE RMS는 좋아졌으나 마지막 스텝 EE 오차가 나빠졌습니다. "
            "`configs/rl_sac.yaml`의 `use_terminal_final_error_penalty`와 `w_final_ee`를 사용해 학습을 이어가세요."
        )

    return "\n\n".join(parts)


def _try_save_paired_plots(
    *,
    plots_dir: Path,
    ee_z: np.ndarray,
    ee_s: np.ndarray,
    d_ee: np.ndarray,
    sat_z: np.ndarray,
    sat_s: np.ndarray,
    lim_z: np.ndarray,
    lim_s: np.ndarray,
    seeds: np.ndarray,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[evaluate_sac_residual] matplotlib 미설치로 플롯을 건너뜁니다.", file=sys.stderr)
        return

    x_ep = np.arange(len(ee_z))

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.bar(["zero", "SAC"], [float(np.nanmean(ee_z)), float(np.nanmean(ee_s))], color=["#4477aa", "#cc8844"])
    ax.set_ylabel("mean EE RMS (m)")
    ax.set_title("Mean EE RMS: zero vs SAC")
    fig.tight_layout()
    fig.savefig(plots_dir / "bar_mean_ee_rms.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.hist(d_ee, bins=min(20, max(5, len(d_ee) // 3)), color="#666688", edgecolor="white")
    ax.axvline(0.0, color="crimson", linestyle="--", linewidth=1)
    ax.set_xlabel("delta EE RMS (SAC − zero)")
    ax.set_title("Histogram of delta EE RMS")
    fig.tight_layout()
    fig.savefig(plots_dir / "hist_delta_ee_rms.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(x_ep, ee_z, marker="o", label="ee_rms_zero", linewidth=1)
    ax.plot(x_ep, ee_s, marker="s", label="ee_rms_sac", linewidth=1)
    ax.set_xlabel("episode index")
    ax.set_ylabel("EE RMS (m)")
    ax.legend()
    ax.set_title("Per-episode EE RMS (paired seeds)")
    fig.tight_layout()
    fig.savefig(plots_dir / "line_ee_rms_per_seed.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(5, 5))
    ax.scatter(ee_z, ee_s, alpha=0.7, edgecolors="k", linewidths=0.3)
    lo = float(np.nanmin(np.concatenate([ee_z, ee_s])))
    hi = float(np.nanmax(np.concatenate([ee_z, ee_s])))
    if np.isfinite(lo) and np.isfinite(hi) and hi > lo:
        ax.plot([lo, hi], [lo, hi], "r--", linewidth=1, label="y=x")
    ax.set_xlabel("ee_rms_zero")
    ax.set_ylabel("ee_rms_sac")
    ax.set_title("Scatter: EE RMS zero vs SAC")
    ax.legend()
    ax.set_aspect("equal", adjustable="box")
    fig.tight_layout()
    fig.savefig(plots_dir / "scatter_ee_rms_zero_vs_sac.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.bar(["zero", "SAC"], [float(np.nanmean(sat_z)), float(np.nanmean(sat_s))], color=["#4477aa", "#cc8844"])
    ax.set_ylabel("mean saturation fraction")
    ax.set_title("Mean saturation fraction")
    fig.tight_layout()
    fig.savefig(plots_dir / "bar_saturation_frac.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.bar(["zero", "SAC"], [float(np.nanmean(lim_z)), float(np.nanmean(lim_s))], color=["#4477aa", "#cc8844"])
    ax.set_ylabel("mean limit violation fraction")
    ax.set_title("Mean limit violation fraction")
    fig.tight_layout()
    fig.savefig(plots_dir / "bar_limit_frac.png", dpi=150)
    plt.close(fig)


def run_paired(args: argparse.Namespace) -> None:
    model_path_in = Path(args.model_path).expanduser().resolve()
    model = load_sac_maybe(model_path_in)
    if model is None:
        raise SystemExit("[evaluate_sac_residual] --paired-seeds requires valid --model-path to a .zip checkpoint.")

    checkpoint_display = model_path_in if model_path_in.suffix == ".zip" else model_path_in.with_suffix(".zip")

    cfg = Path(args.config).resolve()
    profile = str(args.profile)

    run_dir = infer_run_dir(model_path_in, Path(args.run_dir).expanduser().resolve() if args.run_dir else None)
    csv_path, md_path, plots_dir = resolve_paired_paths(args.paired_out, run_dir=run_dir, model_path=model_path_in)

    env_overrides = build_paired_overrides(profile, run_dir)

    vn_path = Path(args.vecnormalize_path).expanduser().resolve() if args.vecnormalize_path else None
    if vn_path is not None and not vn_path.is_file():
        raise SystemExit(f"[evaluate_sac_residual] vecnormalize not found: {vn_path}")

    def sac_policy(obs: np.ndarray) -> np.ndarray:
        a, _ = model.predict(obs, deterministic=True)
        return np.asarray(a, dtype=np.float32)

    n_ep = int(args.num_episodes)
    seed0 = int(args.seed_start)
    seeds = [seed0 + k for k in range(n_ep)]

    cols = [
        "seed",
        "ee_rms_zero",
        "ee_rms_sac",
        "delta_ee_rms",
        "final_ee_zero",
        "final_ee_sac",
        "delta_final_ee",
        "max_ee_zero",
        "max_ee_sac",
        "delta_max_ee",
        "reward_zero",
        "reward_sac",
        "delta_reward",
        "saturation_frac_zero",
        "saturation_frac_sac",
        "delta_saturation_frac",
        "limit_frac_zero",
        "limit_frac_sac",
        "delta_limit_frac",
        "ncon_zero",
        "ncon_sac",
        "mean_action_norm_sac",
        "mean_residual_force_norm_sac",
        "mean_residual_tau_norm_sac",
        "rms_ee_error_velocity_zero",
        "rms_ee_error_velocity_sac",
        "delta_rms_ee_error_velocity",
        "rms_ee_error_highfreq_zero",
        "rms_ee_error_highfreq_sac",
        "delta_rms_ee_error_highfreq",
        "p2p_error_norm_zero",
        "p2p_error_norm_sac",
        "delta_p2p_error_norm",
        "rms_residual_force_rate_zero",
        "rms_residual_force_rate_sac",
        "delta_rms_residual_force_rate",
        "rms_tau_total_rate_zero",
        "rms_tau_total_rate_sac",
        "delta_rms_tau_total_rate",
        "smooth_tracking_score_zero",
        "smooth_tracking_score_sac",
        "delta_smooth_tracking_score",
    ]

    rows: list[dict[str, Any]] = []
    for seed in seeds:
        z = rollout_one_episode_vec(
            obs_policy=_zero_obs_policy,
            config_path=cfg,
            profile=profile,
            seed=int(seed),
            vec_normalize_path=vn_path,
            config_overrides=env_overrides,
        )
        s = rollout_one_episode_vec(
            obs_policy=sac_policy,
            config_path=cfg,
            profile=profile,
            seed=int(seed),
            vec_normalize_path=vn_path,
            config_overrides=env_overrides,
        )

        ee0 = float(z["rms_ee"])
        ees = float(s["rms_ee"])
        fe0 = float(z["final_ee_error"])
        fes = float(s["final_ee_error"])
        mx0 = float(z["max_ee_error"])
        mxs = float(s["max_ee_error"])
        r0 = float(z["episode_return"])
        rs = float(s["episode_return"])
        sz = float(z["sat_frac"])
        ss = float(s["sat_frac"])
        lz = float(z["lim_frac"])
        ls = float(s["lim_frac"])
        nz = int(z["ncon_max"])
        ns = int(s["ncon_max"])

        def gv(d: dict[str, Any], key: str) -> float:
            try:
                return float(d[key])
            except (KeyError, TypeError, ValueError):
                return float("nan")

        rev_z, rev_s = gv(z, "rms_residual_force_rate"), gv(s, "rms_residual_force_rate")

        rows.append(
            {
                "seed": int(seed),
                "ee_rms_zero": ee0,
                "ee_rms_sac": ees,
                "delta_ee_rms": ees - ee0,
                "final_ee_zero": fe0,
                "final_ee_sac": fes,
                "delta_final_ee": fes - fe0,
                "max_ee_zero": mx0,
                "max_ee_sac": mxs,
                "delta_max_ee": mxs - mx0,
                "reward_zero": r0,
                "reward_sac": rs,
                "delta_reward": rs - r0,
                "saturation_frac_zero": sz,
                "saturation_frac_sac": ss,
                "delta_saturation_frac": ss - sz,
                "limit_frac_zero": lz,
                "limit_frac_sac": ls,
                "delta_limit_frac": ls - lz,
                "ncon_zero": nz,
                "ncon_sac": ns,
                "mean_action_norm_sac": float(s["mean_action_norm"]),
                "mean_residual_force_norm_sac": float(s["mean_residual_force_norm"]),
                "mean_residual_tau_norm_sac": float(s["mean_residual_tau_norm"]),
                "rms_ee_error_velocity_zero": gv(z, "rms_ee_error_velocity"),
                "rms_ee_error_velocity_sac": gv(s, "rms_ee_error_velocity"),
                "delta_rms_ee_error_velocity": gv(s, "rms_ee_error_velocity") - gv(z, "rms_ee_error_velocity"),
                "rms_ee_error_highfreq_zero": gv(z, "rms_ee_error_highfreq"),
                "rms_ee_error_highfreq_sac": gv(s, "rms_ee_error_highfreq"),
                "delta_rms_ee_error_highfreq": gv(s, "rms_ee_error_highfreq") - gv(z, "rms_ee_error_highfreq"),
                "p2p_error_norm_zero": gv(z, "p2p_error_norm"),
                "p2p_error_norm_sac": gv(s, "p2p_error_norm"),
                "delta_p2p_error_norm": gv(s, "p2p_error_norm") - gv(z, "p2p_error_norm"),
                "rms_residual_force_rate_zero": rev_z,
                "rms_residual_force_rate_sac": rev_s,
                "delta_rms_residual_force_rate": rev_s - rev_z,
                "rms_tau_total_rate_zero": gv(z, "rms_tau_total_rate"),
                "rms_tau_total_rate_sac": gv(s, "rms_tau_total_rate"),
                "delta_rms_tau_total_rate": gv(s, "rms_tau_total_rate") - gv(z, "rms_tau_total_rate"),
                "smooth_tracking_score_zero": gv(z, "smooth_tracking_score"),
                "smooth_tracking_score_sac": gv(s, "smooth_tracking_score"),
                "delta_smooth_tracking_score": gv(s, "smooth_tracking_score") - gv(z, "smooth_tracking_score"),
            }
        )

    with csv_path.open("w", newline="", encoding="utf-8") as fcsv:
        w = csv.DictWriter(fcsv, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in cols})

    ee_z = np.asarray([float(r["ee_rms_zero"]) for r in rows], dtype=np.float64)
    ee_s = np.asarray([float(r["ee_rms_sac"]) for r in rows], dtype=np.float64)
    d_ee = np.asarray([float(r["delta_ee_rms"]) for r in rows], dtype=np.float64)
    fe_z = np.asarray([float(r["final_ee_zero"]) for r in rows], dtype=np.float64)
    fe_s = np.asarray([float(r["final_ee_sac"]) for r in rows], dtype=np.float64)
    d_fe = np.asarray([float(r["delta_final_ee"]) for r in rows], dtype=np.float64)
    rw_z = np.asarray([float(r["reward_zero"]) for r in rows], dtype=np.float64)
    rw_s = np.asarray([float(r["reward_sac"]) for r in rows], dtype=np.float64)
    d_rw = np.asarray([float(r["delta_reward"]) for r in rows], dtype=np.float64)
    sat_z = np.asarray([float(r["saturation_frac_zero"]) for r in rows], dtype=np.float64)
    sat_s = np.asarray([float(r["saturation_frac_sac"]) for r in rows], dtype=np.float64)
    d_sat = np.asarray([float(r["delta_saturation_frac"]) for r in rows], dtype=np.float64)
    lim_z = np.asarray([float(r["limit_frac_zero"]) for r in rows], dtype=np.float64)
    lim_s = np.asarray([float(r["limit_frac_sac"]) for r in rows], dtype=np.float64)
    d_lim = np.asarray([float(r["delta_limit_frac"]) for r in rows], dtype=np.float64)
    seed_arr = np.asarray(seeds, dtype=np.int64)

    d_ev = np.asarray([float(r["delta_rms_ee_error_velocity"]) for r in rows], dtype=np.float64)
    d_hf = np.asarray([float(r["delta_rms_ee_error_highfreq"]) for r in rows], dtype=np.float64)
    d_p2p = np.asarray([float(r["delta_p2p_error_norm"]) for r in rows], dtype=np.float64)
    d_rfr = np.asarray([float(r["delta_rms_residual_force_rate"]) for r in rows], dtype=np.float64)
    d_rtr = np.asarray([float(r["delta_rms_tau_total_rate"]) for r in rows], dtype=np.float64)
    mean_d_ev = float(np.nanmean(d_ev))
    mean_d_hf = float(np.nanmean(d_hf))
    mean_d_p2p = float(np.nanmean(d_p2p))
    mean_d_rfr = float(np.nanmean(d_rfr))
    mean_d_rtr = float(np.nanmean(d_rtr))

    improved = int(np.sum(d_ee < 0))
    worsened = int(np.sum(d_ee > 0))
    n = max(1, len(rows))
    improvement_ratio = improved / float(n)
    mean_delta_ee = float(np.nanmean(d_ee))

    mean_fe_z = float(np.nanmean(fe_z))
    mean_fe_s = float(np.nanmean(fe_s))
    mean_d_fe = float(np.nanmean(d_fe))

    paired_cls = _classify_paired(mean_delta_ee, improvement_ratio, float(np.nanmean(ee_z)))
    fec = _final_error_pair_classification(mean_delta_ee, mean_d_fe, mean_fe_z)
    multilabel = _paired_multilabel_tags(
        mean_delta_ee=mean_delta_ee,
        improvement_ratio=improvement_ratio,
        mean_delta_final=mean_d_fe,
        mean_delta_hf=mean_d_hf,
        mean_delta_ev=mean_d_ev,
        mean_fe_zero=mean_fe_z,
        ee_zero_mean=float(np.nanmean(ee_z)),
    )
    recommend = _recommendation_for(paired_cls, final_err_cls=fec)

    joint_smooth_cls = _paired_joint_smooth_classification(
        mean_delta_ee=mean_delta_ee,
        mean_delta_final=mean_d_fe,
        mean_fe_zero=mean_fe_z,
        ee_zero_mean=float(np.nanmean(ee_z)),
        mean_delta_ev=mean_d_ev,
        mean_delta_hf=mean_d_hf,
        mean_delta_p2p=mean_d_p2p,
        mean_delta_sat=float(np.nanmean(d_sat)),
        mean_delta_lim=float(np.nanmean(d_lim)),
        mean_sat_zero=float(np.nanmean(sat_z)),
        mean_lim_zero=float(np.nanmean(lim_z)),
    )
    if joint_smooth_cls == "baseline_relative_success":
        recommend += (
            "\n\n**Joint smooth-tracking (baseline-relative 목표 준수):** "
            "평균 Δ RMS·종료 EE·진동 고주파·속도 RMS·포화·리밋이 모두 허용 범위 안에서 개선 또는 유지되는 패턴입니다."
        )
    ncon_max_zero = int(np.nanmax(np.asarray([float(r["ncon_zero"]) for r in rows], dtype=np.float64)))
    ncon_max_sac = int(np.nanmax(np.asarray([float(r["ncon_sac"]) for r in rows], dtype=np.float64)))

    md_lines = [
        "# Paired SAC residual evaluation",
        "",
        "## Setup",
        "",
        f"1. **Model path (SAC checkpoint)**: `{checkpoint_display}`",
        f"2. **Profile**: `{profile}`",
        f"3. **Model checkpoint path**: `{checkpoint_display}`",
        f"4. **VecNormalize path**: `{vn_path}`" if vn_path else "4. **VecNormalize path**: *(none)*",
        f"5. **seed_start**: `{seed0}`",
        f"6. **num_episodes**: `{n_ep}`",
        "",
        f"- Config YAML: `{cfg}`",
        (
            f"- Training ``rl_overrides`` merged from: `{run_dir / 'logs' / 'training_args.yaml'}` *(if file exists)*"
            if run_dir is not None
            else "- Training ``rl_overrides``: *(unknown run dir — use ``runs/<name>/checkpoints/*.zip`` or pass ``--run-dir``)*"
        ),
        f"- CSV: `{csv_path}`",
        f"- Report: `{md_path}`",
        f"- Plots: `{plots_dir}`",
        "",
        "## Summary metrics",
        "",
        "### Final EE error",
        "",
        f"- mean final EE zero: {mean_fe_z:.6g}",
        f"- mean final EE SAC: {mean_fe_s:.6g}",
        f"- mean delta final EE: {mean_d_fe:.6g}",
        "",
        "### EE RMS",
        "",
        f"- mean ee_rms_zero: {float(np.nanmean(ee_z)):.6g}",
        f"- mean ee_rms_sac: {float(np.nanmean(ee_s)):.6g}",
        f"- mean delta_ee_rms: {mean_delta_ee:.6g}",
        f"- median delta_ee_rms: {float(np.nanmedian(d_ee)):.6g}",
        f"- std delta_ee_rms: {float(np.nanstd(d_ee)):.6g}",
        "",
        "### Reward",
        "",
        f"- mean reward_zero: {float(np.nanmean(rw_z)):.6g}",
        f"- mean reward_sac: {float(np.nanmean(rw_s)):.6g}",
        f"- mean delta_reward: {float(np.nanmean(d_rw)):.6g}",
        "",
        "### Saturation / limit fractions",
        "",
        f"- mean saturation_frac_zero: {float(np.nanmean(sat_z)):.6g}",
        f"- mean saturation_frac_sac: {float(np.nanmean(sat_s)):.6g}",
        f"- mean delta_saturation_frac: {float(np.nanmean(d_sat)):.6g}",
        f"- mean limit_frac_zero: {float(np.nanmean(lim_z)):.6g}",
        f"- mean limit_frac_sac: {float(np.nanmean(lim_s)):.6g}",
        f"- mean delta_limit_frac: {float(np.nanmean(d_lim)):.6g}",
        "",
        "### Oscillation / smoothness (mean paired deltas = SAC − zero)",
        "",
        f"- mean delta RMS EE error velocity: {mean_d_ev:.6g}",
        f"- mean delta RMS high-frequency EE error: {mean_d_hf:.6g}",
        f"- mean delta peak-to-peak EE error norm: {mean_d_p2p:.6g}",
        f"- mean delta RMS residual force rate: {mean_d_rfr:.6g}",
        f"- mean delta RMS tau_total rate: {mean_d_rtr:.6g}",
        "",
        "### Episode counts",
        "",
        f"- number of improved episodes (delta_ee_rms < 0): {improved}",
        f"- number of worsened episodes (delta_ee_rms > 0): {worsened}",
        f"- improvement ratio (improved / num_episodes): {improvement_ratio:.4f}",
        "",
        "### Contacts / collisions",
        "",
        f"- max ncon_zero (over episodes): {ncon_max_zero}",
        f"- max ncon_sac (over episodes): {ncon_max_sac}",
        "",
        "## Classification",
        "",
        f"- **paired_result** (aggregate EE RMS): `{paired_cls}`",
        f"- **paired_final_error_classification**: `{fec}`" if fec else "- **paired_final_error_classification**: *(none)*",
        "",
        "### Multi-label (RMS-only classification is ambiguous without oscillation)",
        "",
        *(
            [f"- `{tag}`" for tag in multilabel]
            if multilabel
            else ["- *(no tag matched thresholds)*"]
        ),
        "",
        "### Joint smooth-tracking classification (paired means)",
        "",
        "정의: RMS만이 아니라 종료 EE·진동 관련 평균 델타(SAC − zero)·포화/리밋 증가를 함께 본 결과입니다.",
        f"- **`{joint_smooth_cls}`** (`baseline_relative_success` / `rms_only_improved` / `failed` / `ambiguous` / `inconclusive`)",
        "",
        "## Recommendation",
        "",
        recommend,
        "",
    ]
    md_path.write_text("\n".join(md_lines) + "\n", encoding="utf-8")

    _try_save_paired_plots(
        plots_dir=plots_dir,
        ee_z=ee_z,
        ee_s=ee_s,
        d_ee=d_ee,
        sat_z=sat_z,
        sat_s=sat_s,
        lim_z=lim_z,
        lim_s=lim_s,
        seeds=seed_arr,
    )

    print(f"Wrote {csv_path}")
    print(f"Wrote {md_path}")
    print(f"Plots directory: {plots_dir}")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="SAC residual task-space force evaluation (including paired seeds).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=PAIRED_EPILOG,
    )

    ap.add_argument("--model-path", type=Path, default=None, help="학습된 SAC .zip (--final_model.zip 제외 접두 경로 또는 SB3 저장 경로)")

    ap.add_argument("--config", type=Path, default=_ROOT / "configs" / "rl_sac.yaml")

    ap.add_argument("--profiles", nargs="+", default=["mild", "medium_v2", "medium_train", "medium"])
    ap.add_argument("--profile", type=str, default="medium_train", help="단일 프로파일 (--paired-seeds 시 사용)")
    ap.add_argument("--num-episodes", type=int, default=20)
    ap.add_argument("--out-dir", type=Path, default=OUT_DEF)
    ap.add_argument("--seed-base", type=int, default=4242)

    ap.add_argument("--paired-seeds", action="store_true", help="동일 시드에서 zero vs SAC 짝 평가")
    ap.add_argument("--seed-start", type=int, default=10000, help="짝 평가: 연속 시드 구간 시작")
    ap.add_argument(
        "--vecnormalize-path",
        type=Path,
        default=None,
        help="학습 시 VecNormalize.pkl (짝 평가 시 학습과 동일하게 권장)",
    )
    ap.add_argument(
        "--paired-out",
        type=Path,
        default=None,
        help="짝 평가: 디렉터리면 그 안에 paired_evaluation*.csv/md 작성; `.csv` 파일 경로면 그 옆에 보고서 작성; 생략 시 <run>/logs/",
    )
    ap.add_argument(
        "--run-dir",
        type=Path,
        default=None,
        help="학습 런 루트 (--paired-out 미지정 시 logs/ 출력 위치 및 training_args 병합용)",
    )

    ap.add_argument(
        "--eval-tag",
        type=str,
        default=None,
        help="runs/evaluations/<eval-tag>/ 에 CSV·MD 작성 (기본: 타임스탬프)",
    )

    args = ap.parse_args()
    if args.paired_seeds:
        run_paired(args)
        return

    rng = np.random.default_rng(args.seed_base)

    model = load_sac_maybe(args.model_path)

    eval_tag = args.eval_tag or datetime.now().strftime("%Y%m%d_%H%M%S_eval")
    out_dir = Path(args.out_dir).resolve() / "evaluations" / str(eval_tag)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "evaluation_summary.csv"
    md_path = out_dir / "evaluation_report.md"

    rows: list[dict[str, Any]] = []

    suites: list[tuple[str, Callable[[np.ndarray], np.ndarray]]] = [
        ("baseline_zero", lambda o: np.zeros(3, dtype=np.float32)),
    ]
    if model is not None:
        sac_m = model

        def sac_pol(o: np.ndarray) -> np.ndarray:
            a, _ = sac_m.predict(o, deterministic=True)

            return np.asarray(a, dtype=np.float32).reshape(3)

        suites.append(("trained_sac", sac_pol))

    for profile in args.profiles:
        env_overrides = {"env": {"randomization_profile": str(profile), "randomize_cable": True}}

        opts = {"randomization_profile": str(profile), "randomize_cable": True}
        for pol_name, pol_fn in suites:
            for ep in range(int(args.num_episodes)):
                seed = int(rng.integers(0, 2**31 - 1))

                env = PMICableResidualEnv(config_path=args.config, overrides=env_overrides)


                metrics = rollout_once(env, pol_fn, seed=seed, options=opts)


                rows.append({"profile": profile, "policy": pol_name, "episode": ep, **metrics})
                env.close()

    cols = ["profile", "policy", "episode"] + [
        "n_steps",
        "rms_ee",
        "rms_q_error",
        "final_ee",
        "max_ee",
        "total_reward",
        "sat_steps",
        "lim_steps",
        "sat_frac",
        "lim_frac",
        "ncon_max",
        "mean_residual_force_norm",
        "mean_residual_force_rate",
        "mean_residual_torque_norm",
        "finite",
        "truncated",
        "terminated",
    ]

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as fcsv:
        wcsv = csv.DictWriter(fcsv, fieldnames=cols)
        wcsv.writeheader()
        for r in rows:
            wcsv.writerow({k: r.get(k, "") for k in cols})

    pooled: dict[tuple[str, str], dict[str, list[float]]] = {}
    for r in rows:
        kk = (str(r["profile"]), str(r["policy"]))
        pooled.setdefault(
            kk,
            {
                "rms_ee": [],
                "rms_q_error": [],
                "total_reward": [],
                "final_ee": [],
                "mean_F": [],
                "tau_n": [],
                "sat": [],
                "lim": [],
                "sat_frac": [],
                "lim_frac": [],
            },
        )
        pooled[kk]["rms_ee"].append(float(r["rms_ee"]))
        pooled[kk]["rms_q_error"].append(float(r["rms_q_error"]))
        pooled[kk]["total_reward"].append(float(r["total_reward"]))
        pooled[kk]["final_ee"].append(float(r["final_ee"]))
        pooled[kk]["mean_F"].append(float(r["mean_residual_force_norm"]))
        pooled[kk]["tau_n"].append(float(r["mean_residual_torque_norm"]))
        pooled[kk]["sat"].append(float(r["sat_steps"]))
        pooled[kk]["lim"].append(float(r["lim_steps"]))
        pooled[kk]["sat_frac"].append(float(r["sat_frac"]))
        pooled[kk]["lim_frac"].append(float(r["lim_frac"]))

    def _mean(xs: list[float]) -> float:

        return float(sum(xs) / max(1, len(xs)))

    def _stdev(xs: list[float]) -> float:
        if len(xs) < 2:
            return 0.0
        m = _mean(xs)

        v = sum((x - m) ** 2 for x in xs) / float(len(xs) - 1)


        return float(v**0.5)

    md_lines = [
        "# SAC residual task-force evaluation",
        "",
        "If `--model-path` is missing or invalid, only `baseline_zero` rows are written.",
        "",
        "| profile | policy | mean RMS EE | mean RMS q | mean sat_frac | mean lim_frac | mean R |",
        "|---------|--------|------------|-------------|---------------|---------------|-------|",
    ]

    for (prof, pname), bag in sorted(pooled.items()):
        md_lines.append(
            f"| {prof} | {pname} | {_mean(bag['rms_ee']):.6f} | {_mean(bag['rms_q_error']):.6f} | "
            f"{_mean(bag['sat_frac']):.4f} | {_mean(bag['lim_frac']):.4f} | {_mean(bag['total_reward']):.3f} |"
        )

    md_lines += [
        "",
        "## Notes",
        "",
        "1. medium_train: trained_sac 대비 baseline_zero의 mean RMS EE.",
        "2. mild/medium_v2 등 일반화 프로파일.",
        "3. `sat_frac`,`lim_frac`는 스텝 대비 카운트 비율.",
        "4. `rms_q_error`는 info의 q_error_norm 기반 RMS.",
        "",
        f"출력 디렉터리: `{out_dir}`",
        f"- CSV: `{csv_path}`",
        f"- MD: `{md_path}`",
    ]

    md_path.write_text("\n".join(md_lines) + "\n", encoding="utf-8")
    print(f"Wrote {csv_path}")
    print(f"Wrote {md_path}")


if __name__ == "__main__":
    main()
