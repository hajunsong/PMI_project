#!/usr/bin/env python3
"""SAC training on PMICableResidualEnv with run layout, baseline eval, diagnostics callback.

    Layout per run::

    debug_outputs/sac_residual_task_force/runs/<run_name>/
      checkpoints/   (checkpoint_<step>.zip, best_model_by_reward.zip, best_model_by_ee_rms.zip,
                      best_model_by_combined_tracking.zip, best_model_by_smooth_tracking.zip,
                      baseline-relative 학습 시 best_model_by_relative_smooth_score.zip, final_model.zip, stopped_model.zip)
      replay_buffer/replay_buffer.pkl
      vecnormalize/vecnormalize.pkl  (if enabled)
      logs/  (CSV/YAML/MD, train_monitor.csv, early_stop_reason.txt, stop_status.yaml, best_metrics.yaml, monitor.csv)
      tensorboard/

Example::

    cd pmi_hybrid_arm_mujoco
    python scripts/train_sac_residual.py --timesteps 1000 --profile medium_train \\
        --run-name smoke_progress_test --progress --early-stop
"""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
import time
import traceback
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np
import yaml

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from stable_baselines3 import SAC
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from callbacks.sac_diagnostics_callback import (
    EarlyStopConfig,
    SACDiagnosticsCallback,
    clear_run_csv_logs,
    evaluate_policy_vec,
    write_eval_baseline_row,
)
from callbacks.train_monitor_csv import TrainMonitorCsvWrapper
from envs.pmi_cable_residual_env import PMICableResidualEnv, _deep_merge
from utils.mujoco_helpers import load_yaml


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=_ROOT / "configs" / "rl_sac.yaml")
    ap.add_argument("--profile", type=str, default="medium_train")
    ap.add_argument("--timesteps", type=int, required=True)
    ap.add_argument("--run-name", type=str, required=True)
    ap.add_argument(
        "--out-dir",
        type=Path,
        default=_ROOT / "debug_outputs" / "sac_residual_task_force",
        help="Root under which runs/<run_name>/ is created",
    )
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--resume-from", type=Path, default=None, help="Optional path to saved SAC .zip (prefix)")

    ap.add_argument("--use-vecnormalize", dest="use_vecnormalize", action="store_true")
    ap.add_argument("--no-vecnormalize", dest="use_vecnormalize", action="store_false")
    ap.set_defaults(use_vecnormalize=True)

    ap.add_argument("--progress", dest="progress", action="store_true")
    ap.add_argument("--no-progress", dest="progress", action="store_false")
    ap.set_defaults(progress=False)

    ap.add_argument("--eval-freq", type=int, default=5000)
    ap.add_argument("--eval-episodes", type=int, default=20)
    ap.add_argument("--checkpoint-freq", type=int, default=10000)
    ap.add_argument("--baseline-eval-episodes", type=int, default=5)
    ap.add_argument("--eval-seed-base", type=int, default=4242)
    ap.add_argument(
        "--eval-seed-start",
        type=int,
        default=10000,
        help="Periodic/final eval: consecutive env seeds [start, start+count)",
    )
    ap.add_argument(
        "--eval-seed-count",
        type=int,
        default=None,
        help="Fixed eval seeds count (default: eval_episodes)",
    )

    ap.add_argument("--early-stop", action="store_true")
    ap.add_argument("--patience-evals", type=int, default=8)
    ap.add_argument("--min-improvement", type=float, default=0.01)
    ap.add_argument("--min-train-steps-before-stop", type=int, default=30000)
    ap.add_argument("--min-return-vs-baseline", type=float, default=None)
    ap.add_argument("--saturation-margin-over-baseline", type=float, default=0.10)
    ap.add_argument("--limit-margin-over-baseline", type=float, default=0.01)
    ap.add_argument("--hard-max-saturation-fraction", type=float, default=0.95)
    ap.add_argument("--hard-max-limit-violation-fraction", type=float, default=0.10)
    ap.add_argument("--max-saturation-step-frac", type=float, default=None)
    ap.add_argument("--max-limit-step-frac", type=float, default=None)
    ap.add_argument("--max-mean-ee-error", type=float, default=None)
    ap.add_argument("--action-l2-collapse-frac", type=float, default=0.02)
    ap.add_argument("--action-abs-explosion", type=float, default=1.02)
    ap.add_argument("--anomaly-patience-evals", type=int, default=4)

    ap.add_argument("--ent-coef", type=str, default="auto", help="'auto' or float")
    ap.add_argument("--learning-rate", type=float, default=None, help="Override YAML learning_rate")
    ap.add_argument("--batch-size", type=int, default=None, help="Override YAML batch_size")
    ap.add_argument("--buffer-size", type=int, default=None, help="Override YAML buffer_size")
    ap.add_argument("--learning-starts", type=int, default=None, help="Override YAML learning_starts")
    ap.add_argument("--residual-force-scale", type=float, default=None, help="Override residual.residual_force_scale")
    ap.add_argument("--tau-jnt-limit", type=float, default=None, help="Override env.tau_jnt_limit")
    ap.add_argument(
        "--reward-preset",
        type=str,
        default="default",
        help="Name under ``reward_presets`` in configs/rl_sac.yaml "
        "(e.g. default, tracking_focused, tracking_smooth_final, baseline_relative_smooth)",
    )
    ap.add_argument("--residual-filter", dest="residual_filter", action="store_true", help="Enable residual_filter.enabled in RL overrides")
    ap.add_argument("--no-residual-filter", dest="residual_filter", action="store_false", help="Force-disable residual force low-pass filter")
    ap.set_defaults(residual_filter=None)
    ap.add_argument("--residual-filter-tau", type=float, default=None, help="Residual low-pass time constant tau (seconds)")
    ap.add_argument(
        "--baseline-relative-reward",
        dest="baseline_relative_reward",
        action="store_true",
        help="Enable baseline_relative_reward.enabled (cached zero-rollout reference per episode).",
    )
    ap.add_argument(
        "--no-baseline-relative-reward",
        dest="baseline_relative_reward",
        action="store_false",
        help="Disable baseline-relative reward overrides.",
    )
    ap.set_defaults(baseline_relative_reward=None)
    ap.add_argument(
        "--include-baseline-reference-in-obs",
        dest="include_baseline_reference_in_obs",
        action="store_true",
        help="Append baseline-vs-current EE features to observations (OBS dim +8; requires compatible VecNormalize).",
    )
    ap.add_argument(
        "--no-include-baseline-reference-in-obs",
        dest="include_baseline_reference_in_obs",
        action="store_false",
    )
    ap.set_defaults(include_baseline_reference_in_obs=None)
    ap.add_argument(
        "--action-smoothing",
        dest="action_smoothing",
        action="store_true",
        help="Enable action_smoothing.enabled before residual_filter stage.",
    )
    ap.add_argument("--no-action-smoothing", dest="action_smoothing", action="store_false")
    ap.set_defaults(action_smoothing=None)
    ap.add_argument(
        "--max-delta-force-per-step",
        type=float,
        default=None,
        help="Optional action_smoothing.max_delta_force_per_step override (slew-limit on residual force increment).",
    )

    ap.add_argument("--analyze-after", action="store_true", help="Run scripts/analyze_sac_training_logs.py at end")

    return ap.parse_args()


def _zero_policy(obs: np.ndarray) -> np.ndarray:
    return np.zeros((obs.shape[0], 3), dtype=np.float32)


def _parse_ent_coef(s: str) -> Any:
    s = str(s).strip().lower()
    if s == "auto":
        return "auto"
    return float(s)


def _sb3_progress_deps_available() -> bool:
    try:
        import rich  # noqa: F401
        import tqdm  # noqa: F401
    except ImportError:
        return False
    return True


def _write_training_args(run_dir: Path, payload: dict[str, Any]) -> None:
    p = run_dir / "logs" / "training_args.yaml"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")


def _latest_eval_csv_row(logs_dir: Path, eval_type: str, run_name: str) -> dict[str, str] | None:
    path = logs_dir / "eval_log.csv"
    if not path.is_file():
        return None
    with path.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))

    def row_gs(rr: dict[str, str]) -> int:
        try:
            return int(float(rr.get("global_step") or -1))
        except (TypeError, ValueError):
            return -1

    cand: list[dict[str, str]] = []
    for r in rows:
        if (r.get("eval_type") or "").strip() != eval_type:
            continue
        rn = (r.get("run_name") or "").strip()
        if rn and rn != run_name:
            continue
        cand.append(r)
    if not cand:
        return None
    return max(cand, key=row_gs)


def _load_paired_evaluation_summary(logs_dir: Path) -> dict[str, Any] | None:
    """``paired_evaluation.csv`` 가 있으면 평균 final EE 및 분류 문자열을 반환."""
    p = logs_dir / "paired_evaluation.csv"
    if not p.is_file():
        return None
    with p.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None

    def col_mean(key: str) -> float:
        xs: list[float] = []
        for r in rows:
            try:
                xs.append(float(r[key]))
            except (KeyError, TypeError, ValueError):
                pass
        return float(np.nanmean(np.asarray(xs, dtype=np.float64))) if xs else float("nan")

    mfz = col_mean("final_ee_zero")
    mfs = col_mean("final_ee_sac")
    mdf = col_mean("delta_final_ee")
    mdz = col_mean("delta_ee_rms")
    ftol = max(5e-7, 0.002 * abs(mfz)) if mfz == mfz else 5e-7
    fin_cls: str | None = None
    if mdz == mdz and mdf == mdf and mdz < 0 and mdf > ftol:
        fin_cls = "tracking_improved_but_final_error_worse"

    return {
        "mean_final_ee_zero": mfz,
        "mean_final_ee_sac": mfs,
        "mean_delta_final_ee": mdf,
        "mean_delta_ee_rms": mdz,
        "paired_final_error_classification": fin_cls,
        "csv_path": str(p.resolve()),
    }


def _write_summary_md(
    run_dir: Path,
    *,
    timesteps: int,
    profile: str,
    stopped_reason: str | None,
    baseline_metrics: dict[str, Any],
    run_name: str,
) -> None:
    logs_dir = run_dir / "logs"
    bl_ret = float(baseline_metrics.get("mean_episode_return", float("nan")))
    bl_ee = float(baseline_metrics.get("mean_ee_rms", float("nan")))
    bl_sat = float(baseline_metrics.get("mean_sat_frac", float("nan")))
    bl_lim = float(baseline_metrics.get("mean_lim_frac", float("nan")))

    fin = _latest_eval_csv_row(logs_dir, "final_eval", run_name)

    bm_path = logs_dir / "best_metrics.yaml"
    best_blob: dict[str, Any] = {}
    if bm_path.is_file():
        try:
            raw = yaml.safe_load(bm_path.read_text(encoding="utf-8"))
            if isinstance(raw, dict):
                best_blob = raw
        except Exception:
            best_blob = {}

    fr_ret = fr_ee = fr_sat = fr_lim = float("nan")
    if fin is not None:
        try:
            fr_ret = float(fin["mean_episode_return"])
            fr_ee = float(fin["mean_ee_rms"])
            fr_sat = float(fin["mean_sat_frac"])
            fr_lim = float(fin["mean_lim_frac"])
        except (KeyError, TypeError, ValueError):
            pass

    successful_tracking = False
    if bl_ee == bl_ee and fr_ee == fr_ee:
        successful_tracking = fr_ee < bl_ee

    def cell(x: float) -> str:
        return "—" if x != x else f"{x:.6g}"

    lines = [
        "# SAC residual training summary",
        "",
        f"- **profile**: {profile}",
        f"- **run_name**: {run_name}",
        f"- **requested timesteps**: {timesteps}",
        f"- **baseline mean episode return (zero policy)**: {bl_ret}",
        f"- **stop / completion reason**: {stopped_reason or 'completed'}",
        f"- **successful_tracking_improvement** (final mean EE RMS < baseline mean EE RMS): {successful_tracking}",
        "",
        f"Artifacts: `{run_dir}/checkpoints/`, `{run_dir}/logs/`, `{run_dir}/tensorboard/`.",
        "",
        "## Best checkpoints (from periodic eval)",
        "",
    ]

    if best_blob:
        lines.append(
            f"- **best reward checkpoint** (`best_model_by_reward.zip`): reward={best_blob.get('best_reward')}, "
            f"step={best_blob.get('best_reward_step')}"
        )
        lines.append(
            f"- **best EE RMS checkpoint** (`best_model_by_ee_rms.zip`): mean_ee_rms={best_blob.get('best_ee_rms')}, "
            f"step={best_blob.get('best_ee_rms_step')} "
            f"(mean final EE error at that eval: {best_blob.get('best_final_ee_error')}, "
            f"sat={best_blob.get('best_saturation_fraction')}, lim={best_blob.get('best_limit_violation_fraction')})"
        )
        if best_blob.get("best_combined_tracking_score") is not None:
            lines.append(
                f"- **best combined tracking** (`best_model_by_combined_tracking.zip`): "
                f"score={best_blob.get('best_combined_tracking_score')} "
                f"(mean EE RMS + 0.5×mean final EE error), step={best_blob.get('best_combined_tracking_step')}"
            )
        bss = best_blob.get("best_smooth_tracking_score")
        if bss is not None and bss == bss:
            lines.append(
                f"- **best smooth tracking** (`best_model_by_smooth_tracking.zip`): score={bss}, "
                f"step={best_blob.get('best_smooth_tracking_step')} "
                f"(lower = better RMS + weighted final / velocity / HF / torque rates; see `logs/best_metrics.yaml`)"
            )
        brls = best_blob.get("best_relative_smooth_score")
        if brls is not None and brls == brls:
            lines.append(
                f"- **best baseline-relative (proxy) smooth** (`best_model_by_relative_smooth_score.zip`): score={brls}, "
                f"step={best_blob.get('best_relative_smooth_step')} "
                f"(periodic eval uses `mean_smooth_tracking_score` as proxy; 짝 평가로 상대 지표 확정)"
            )
    else:
        lines.append("- *(no `logs/best_metrics.yaml` — no periodic eval best summary)*")

    lines.extend(
        [
            "",
            "## Diagnostic table (baseline vs final eval)",
            "",
            "| Metric | Baseline (zero policy) | Final eval |",
            "| --- | --- | --- |",
        ]
    )

    if fin is None:
        lines.append("| *(final eval row missing)* | — | — |")
        lines.extend(["", "(학습 종료 후 `eval_log.csv`에 `final_eval` 행이 없으면 표를 채울 수 없습니다.)", ""])
    else:
        lines.append(f"| mean episode return | {cell(bl_ret)} | {cell(fr_ret)} |")
        lines.append(f"| mean EE RMS (m) | {cell(bl_ee)} | {cell(fr_ee)} |")
        lines.append(f"| mean saturation fraction | {cell(bl_sat)} | {cell(fr_sat)} |")
        lines.append(f"| mean limit violation fraction | {cell(bl_lim)} | {cell(fr_lim)} |")

        pct_ee_txt = "—"
        if bl_ee == bl_ee and fr_ee == fr_ee and abs(bl_ee) > 1e-12:
            pct_ee_txt = f"{100.0 * (bl_ee - fr_ee) / bl_ee:.2f}% (baseline 대비 EE RMS 감소율)"

        pct_ret_txt = "—"
        if bl_ret == bl_ret and fr_ret == fr_ret and abs(bl_ret) > 1e-6:
            pct_ret_txt = f"{100.0 * (fr_ret - bl_ret) / abs(bl_ret):.2f}% (return 변화율; 음수 보상 스케일에서 해석 유의)"

        lines.extend(
            [
                "",
                f"- **Percent improvement in EE RMS** (baseline 대비 감소): {pct_ee_txt}",
                f"- **Percent change in mean return** (의미 있을 때만 참고): {pct_ret_txt}",
                "",
            ]
        )

    paired_sum = _load_paired_evaluation_summary(logs_dir)

    lines.extend(
        [
            "## Paired evaluation",
            "",
            "같은 케이블/환경 시드에서 잔차 0 베이스라인과 SAC를 짝지어 평가하면, 랜덤 시드 변동에 가려진 개선 여부를 확인할 수 있습니다.",
            "",
        ]
    )

    if paired_sum is not None:
        pc = paired_sum.get("paired_final_error_classification")
        lines.extend(
            [
                "### Results from `logs/paired_evaluation.csv`",
                "",
                f"- **mean final EE (zero policy)**: {paired_sum['mean_final_ee_zero']}",
                f"- **mean final EE (SAC)**: {paired_sum['mean_final_ee_sac']}",
                f"- **mean delta final EE** (SAC − zero): {paired_sum['mean_delta_final_ee']}",
                f"- **mean delta EE RMS** (SAC − zero): {paired_sum['mean_delta_ee_rms']}",
                "",
                f"- **paired_final_error_classification**: `{pc}`"
                if pc
                else "- **paired_final_error_classification**: *(not applicable)*",
                "",
            ]
        )
        if pc == "tracking_improved_but_final_error_worse":
            lines.extend(
                [
                    "**Recommendation:** 평균 EE RMS는 개선되었으나 종료 시점 EE 오차가 악화되었습니다. "
                    "`tracking_focused` 프리셋의 종료 시점 final EE 패널티(`use_terminal_final_error_penalty`)를 켠 채 학습을 이어가는 것을 권장합니다.",
                    "",
                ]
            )

    lines.extend(
        [
            "### 권장: baseline-relative 학습 (다음 실험 후보)",
            "",
            "베이스라인 VSD(동일 케이블 시드에서 잔차 0) 대비 개선을 보상에 직접 넣을 때 사용합니다. "
            "주기적 평가에서는 `best_model_by_relative_smooth_score.zip` 선택에 `mean_smooth_tracking_score` 프록시가 쓰이며, "
            "학습 후에는 아래 짝 평가로 최종 판정합니다.",
            "",
            "```bash",
            "python scripts/train_sac_residual.py \\",
            "  --timesteps 30000 --profile medium_train --run-name sac_tf_baseline_relative_rs1_30k_s6 --seed 6 \\",
            "  --progress --early-stop \\",
            "  --eval-freq 5000 --eval-episodes 30 --checkpoint-freq 5000 \\",
            "  --min-train-steps-before-stop 20000 --patience-evals 6 --min-improvement 0.003 \\",
            "  --learning-rate 0.0001 --batch-size 256 --buffer-size 200000 --learning-starts 2000 \\",
            "  --residual-force-scale 1.0 --tau-jnt-limit 30 \\",
            "  --reward-preset baseline_relative_smooth \\",
            "  --baseline-relative-reward --include-baseline-reference-in-obs \\",
            "  --residual-filter --residual-filter-tau 0.08 \\",
            "  --action-smoothing --max-delta-force-per-step 0.2 \\",
            "  --use-vecnormalize",
            "```",
            "",
            "```bash",
            "python scripts/evaluate_sac_residual.py --paired-seeds --model-path <run_dir>/checkpoints/final_model.zip \\",
            "  --vecnormalize-path <run_dir>/vecnormalize/vecnormalize.pkl \\",
            "  --config configs/rl_sac.yaml --profile medium_train --seed-start 10000 --num-episodes 20",
            "```",
            "",
        ]
    )

    (logs_dir / "training_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_next_run_recommendation_md(
    run_dir: Path,
    *,
    baseline_metrics: dict[str, Any],
    run_name: str,
) -> None:
    """Heuristic 다음 런 권장사항 (`logs/next_run_recommendation.md`)."""
    logs_dir = run_dir / "logs"
    fin = _latest_eval_csv_row(logs_dir, "final_eval", run_name)
    bl_ret = float(baseline_metrics.get("mean_episode_return", float("nan")))
    bl_ee = float(baseline_metrics.get("mean_ee_rms", float("nan")))
    bl_sat = float(baseline_metrics.get("mean_sat_frac", float("nan")))

    fr_ret = fr_ee = fr_sat = float("nan")
    if fin:
        try:
            fr_ret = float(fin["mean_episode_return"])
            fr_ee = float(fin["mean_ee_rms"])
            fr_sat = float(fin["mean_sat_frac"])
        except (KeyError, TypeError, ValueError):
            pass

    ret_improved = bl_ret == bl_ret and fr_ret == fr_ret and fr_ret > bl_ret
    ee_worse = bl_ee == bl_ee and fr_ee == fr_ee and fr_ee > bl_ee
    ee_better = bl_ee == bl_ee and fr_ee == fr_ee and fr_ee < bl_ee
    sat_up = bl_sat == bl_sat and fr_sat == fr_sat and fr_sat > bl_sat + 1e-6

    bullets: list[str] = []
    if ret_improved and ee_worse:
        bullets.extend(
            [
                "- Return은 개선되었으나 EE RMS가 악화되었습니다.",
                "- `tracking_focused` 보상(`--reward-preset tracking_focused`) 전환을 검토하세요.",
                "- `--residual-force-scale`을 낮추는 것을 검토하세요.",
                "- 베스트 모델은 EE RMS 기준(`best_model_by_ee_rms.zip`)을 사용하세요.",
                "- `scripts/evaluate_sac_residual.py --paired-seeds`로 짝 평가를 먼저 수행하세요.",
                "- 동일 설정으로 타임스텝만 늘리는 것은 권장하지 않습니다.",
            ]
        )
    elif ee_better and not sat_up:
        bullets.extend(
            [
                "- EE RMS가 개선되었고 포화율이 크게 나빠지지 않았습니다.",
                "- 동일 보상으로 더 긴 학습을 검토할 수 있습니다.",
                "- 필요 시 `medium` 등 일반화 프로파일에서 `evaluate_sac_residual.py`로 추가 평가하세요.",
            ]
        )
    else:
        bullets.append("- 결과를 바탕으로 하이퍼파라미터·보상 가중치·잔차 스케일을 조정해 반복 실험하세요.")

    text = "\n".join(
        [
            "# Next run recommendation",
            "",
            *bullets,
            "",
        ]
    )
    (logs_dir / "next_run_recommendation.md").write_text(text + "\n", encoding="utf-8")


def _save_all(
    model: SAC,
    vec: Any,
    *,
    run_dir: Path,
    replay_path: Path,
    vn_path: Path,
    use_vn: bool,
    final_stem: str = "final_model",
) -> None:
    replay_path.parent.mkdir(parents=True, exist_ok=True)
    model.save(str(run_dir / "checkpoints" / final_stem))
    try:
        model.save_replay_buffer(str(replay_path))
    except Exception:
        pass
    if use_vn and isinstance(vec, VecNormalize):
        vec.save(str(vn_path))


def main() -> None:
    args = parse_args()
    cfg_path = Path(args.config)
    sac_yaml = load_yaml(cfg_path)
    sac_cfg = sac_yaml["sac"]

    run_dir = Path(args.out_dir) / "runs" / str(args.run_name)
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "checkpoints").mkdir(parents=True, exist_ok=True)
    (run_dir / "replay_buffer").mkdir(parents=True, exist_ok=True)
    (run_dir / "vecnormalize").mkdir(parents=True, exist_ok=True)
    (run_dir / "logs").mkdir(parents=True, exist_ok=True)
    (run_dir / "tensorboard").mkdir(parents=True, exist_ok=True)

    vn_path = run_dir / "vecnormalize" / "vecnormalize.pkl"
    replay_path = run_dir / "replay_buffer" / "replay_buffer.pkl"
    tm_csv = run_dir / "logs" / "train_monitor.csv"
    logs_dir = run_dir / "logs"

    resume = args.resume_from is not None
    if not resume:
        clear_run_csv_logs(logs_dir, resume=False)

    rl_overrides: dict[str, Any] = {"env": {"randomization_profile": str(args.profile), "randomize_cable": True}}
    if args.tau_jnt_limit is not None:
        rl_overrides["env"]["tau_jnt_limit"] = float(args.tau_jnt_limit)
    if args.residual_force_scale is not None:
        rl_overrides.setdefault("residual", {})["residual_force_scale"] = float(args.residual_force_scale)

    preset_name = str(args.reward_preset)
    presets_map = sac_yaml.get("reward_presets") or {}
    if preset_name not in presets_map:
        raise SystemExit(
            f"[train_sac_residual] unknown --reward-preset {preset_name!r}; "
            f"expected one of {sorted(presets_map)}"
        )

    if args.residual_filter is not None:
        rl_overrides.setdefault("residual_filter", {})["enabled"] = bool(args.residual_filter)
    if args.residual_filter_tau is not None:
        rl_overrides.setdefault("residual_filter", {})["tau"] = float(args.residual_filter_tau)
    base_r = dict(sac_yaml.get("reward") or {})
    rl_overrides["reward"] = _deep_merge(base_r, dict(presets_map[preset_name]))

    br = getattr(args, "baseline_relative_reward", None)
    if br is not None:
        rl_overrides.setdefault("baseline_relative_reward", {})["enabled"] = bool(br)
    ibo = getattr(args, "include_baseline_reference_in_obs", None)
    if ibo is not None:
        rl_overrides.setdefault("baseline_relative_reward", {})["include_baseline_reference_in_obs"] = bool(ibo)
    asm = getattr(args, "action_smoothing", None)
    if asm is not None:
        rl_overrides.setdefault("action_smoothing", {})["enabled"] = bool(asm)
    if getattr(args, "max_delta_force_per_step", None) is not None:
        rl_overrides.setdefault("action_smoothing", {})["max_delta_force_per_step"] = float(args.max_delta_force_per_step)

    if preset_name == "baseline_relative_smooth":
        bro = rl_overrides.setdefault("baseline_relative_reward", {})
        if getattr(args, "baseline_relative_reward", None) is None:
            bro["enabled"] = True
        if getattr(args, "include_baseline_reference_in_obs", None) is None:
            bro.setdefault("include_baseline_reference_in_obs", True)

    eval_seed_count = int(args.eval_seed_count) if args.eval_seed_count is not None else int(args.eval_episodes)
    if eval_seed_count != int(args.eval_episodes):
        print(
            "[train_sac_residual] eval_seed_count 가 eval_episodes 와 다릅니다. eval_episodes 에 맞춥니다.",
            file=sys.stderr,
        )
        eval_seed_count = int(args.eval_episodes)
    eval_episode_seeds = list(range(int(args.eval_seed_start), int(args.eval_seed_start) + eval_seed_count))
    baseline_episode_seeds = list(
        range(int(args.eval_seed_start), int(args.eval_seed_start) + int(args.baseline_eval_episodes))
    )

    learning_starts = (
        int(args.learning_starts)
        if args.learning_starts is not None
        else int(sac_cfg.get("learning_starts", 1000))
    )
    ts_req = int(args.timesteps)
    if learning_starts >= ts_req > 0:
        learning_starts = max(1, min(ts_req - 1, max(50, ts_req // 10)))

    sac_kwargs = dict(
        learning_rate=float(sac_cfg.get("learning_rate", 3e-4)),
        buffer_size=int(sac_cfg.get("buffer_size", 200000)),
        learning_starts=learning_starts,
        batch_size=int(sac_cfg.get("batch_size", 256)),
        tau=float(sac_cfg.get("tau", 0.005)),
        gamma=float(sac_cfg.get("gamma", 0.99)),
        train_freq=int(sac_cfg.get("train_freq", 1)),
        gradient_steps=int(sac_cfg.get("gradient_steps", 1)),
        ent_coef=_parse_ent_coef(args.ent_coef),
        verbose=1,
        tensorboard_log=str(run_dir / "tensorboard"),
    )
    if args.learning_rate is not None:
        sac_kwargs["learning_rate"] = float(args.learning_rate)
    if args.batch_size is not None:
        sac_kwargs["batch_size"] = int(args.batch_size)
    if args.buffer_size is not None:
        sac_kwargs["buffer_size"] = int(args.buffer_size)

    early = EarlyStopConfig(
        enabled=bool(args.early_stop),
        patience_evals=int(args.patience_evals),
        min_improvement=float(args.min_improvement),
        min_train_steps_before_stop=int(args.min_train_steps_before_stop),
        min_return_vs_baseline=args.min_return_vs_baseline,
        saturation_margin_over_baseline=float(args.saturation_margin_over_baseline),
        limit_margin_over_baseline=float(args.limit_margin_over_baseline),
        hard_max_saturation_fraction=float(args.hard_max_saturation_fraction),
        hard_max_limit_violation_fraction=float(args.hard_max_limit_violation_fraction),
        max_saturation_step_frac=args.max_saturation_step_frac,
        max_limit_step_frac=args.max_limit_step_frac,
        max_mean_ee_error=args.max_mean_ee_error,
        action_l2_collapse_frac=float(args.action_l2_collapse_frac),
        action_abs_explosion=float(args.action_abs_explosion),
        anomaly_patience_evals=int(args.anomaly_patience_evals),
    )

    baseline_metrics: dict[str, Any]
    bl_yaml = logs_dir / "baseline_eval.yaml"
    need_baseline_rollout = True
    if resume and bl_yaml.is_file():
        try:
            raw = yaml.safe_load(bl_yaml.read_text(encoding="utf-8"))
            m = raw.get("baseline_zero_policy_metrics") if isinstance(raw, dict) else None
            if isinstance(m, dict) and m.get("mean_episode_return") is not None:
                baseline_metrics = {str(k): float(v) for k, v in m.items()}
                need_baseline_rollout = False
        except Exception:
            need_baseline_rollout = True
    if not need_baseline_rollout:
        for k in ("mean_sat_frac", "mean_lim_frac"):
            if k not in baseline_metrics:
                need_baseline_rollout = True
                break
    if need_baseline_rollout:
        baseline_metrics = evaluate_policy_vec(
            _zero_policy,
            config_path=cfg_path,
            profile=str(args.profile),
            n_episodes=int(args.baseline_eval_episodes),
            seed_base=int(args.eval_seed_base),
            vec_normalize_path=None,
            config_overrides=rl_overrides,
            episode_seeds=baseline_episode_seeds,
        )
        bl_yaml.write_text(
            yaml.safe_dump({"baseline_zero_policy_metrics": baseline_metrics}, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )

    baseline_ret = float(baseline_metrics["mean_episode_return"])
    baseline_sat_frac = float(baseline_metrics["mean_sat_frac"])
    baseline_lim_frac = float(baseline_metrics["mean_lim_frac"])

    use_vn = bool(args.use_vecnormalize)

    training_payload = {
        "config": str(cfg_path),
        "profile": args.profile,
        "run_name": args.run_name,
        "timesteps": int(args.timesteps),
        "seed": args.seed,
        "use_vecnormalize": use_vn,
        "resume": resume,
        "reward_preset": preset_name,
        "residual_filter_cli": args.residual_filter,
        "residual_filter_tau_cli": args.residual_filter_tau,
        "baseline_relative_reward_cli": getattr(args, "baseline_relative_reward", None),
        "include_baseline_reference_in_obs_cli": getattr(args, "include_baseline_reference_in_obs", None),
        "action_smoothing_cli": getattr(args, "action_smoothing", None),
        "max_delta_force_per_step_cli": getattr(args, "max_delta_force_per_step", None),
        "rl_overrides": rl_overrides,
        "eval_freq": int(args.eval_freq),
        "eval_episodes": int(args.eval_episodes),
        "eval_seed_start": int(args.eval_seed_start),
        "eval_seed_count": int(eval_seed_count),
        "checkpoint_freq": int(args.checkpoint_freq),
        "early_stop": bool(args.early_stop),
        "early_stop_fields": asdict(early),
        "sac_kwargs": sac_kwargs,
    }
    _write_training_args(run_dir, training_payload)

    if not resume:
        write_eval_baseline_row(
            logs_dir / "eval_log.csv",
            run_name=str(args.run_name),
            seed=args.seed,
            profile=str(args.profile),
            metrics=baseline_metrics,
        )

    diag_cb = SACDiagnosticsCallback(
        run_dir=run_dir,
        run_name=str(args.run_name),
        seed=args.seed,
        config_path=cfg_path,
        profile=str(args.profile),
        eval_freq=int(args.eval_freq),
        eval_episodes=int(args.eval_episodes),
        checkpoint_freq=int(args.checkpoint_freq),
        vec_normalize_save_path=vn_path if use_vn else None,
        eval_use_vec_normalize=use_vn,
        early=early,
        baseline_mean_return=baseline_ret,
        baseline_sat_frac=baseline_sat_frac,
        baseline_lim_frac=baseline_lim_frac,
        rl_config_overrides=rl_overrides,
        eval_episode_seeds=eval_episode_seeds,
        verbose=0,
    )

    def factory():
        env = PMICableResidualEnv(config_path=cfg_path, overrides=rl_overrides)
        env = TrainMonitorCsvWrapper(
            env,
            tm_csv,
            run_name=str(args.run_name),
            seed=args.seed,
            profile=str(args.profile),
            fresh=not resume,
        )
        env = Monitor(env, str(logs_dir))
        return env

    vec: Any = DummyVecEnv([factory])
    if use_vn:
        vec = VecNormalize(vec, norm_obs=True, norm_reward=True, clip_obs=10.0)
        vec.save(str(vn_path))

    stopped_reason: str | None = None
    model: SAC | None = None
    train_wall_start: float | None = None

    try:
        if args.resume_from is not None:
            rf = Path(args.resume_from)
            zp = rf if rf.suffix == ".zip" else rf.with_suffix(".zip")
            if not zp.is_file():
                raise FileNotFoundError(f"SAC checkpoint not found: {zp}")
            model = SAC.load(str(zp), env=vec, verbose=1)
        else:
            model = SAC("MlpPolicy", vec, seed=args.seed if args.seed is not None else None, **sac_kwargs)

        assert model is not None
        use_progress = bool(args.progress) and _sb3_progress_deps_available()
        if bool(args.progress) and not use_progress:
            print(
                "[train_sac_residual] tqdm/rich 미설치로 진행 표시를 끕니다. "
                "설치: `pip install tqdm rich` 또는 `pip install stable-baselines3[extra]`. "
                "또는 `--no-progress` 사용.",
                file=sys.stderr,
            )

        train_wall_start = time.perf_counter()
        model.learn(total_timesteps=int(args.timesteps), callback=diag_cb, progress_bar=use_progress)
        stopped_reason = str(diag_cb.stopped_reason) if diag_cb.stopped_reason else None

    except KeyboardInterrupt:
        stopped_reason = "keyboard_interrupt"

    except Exception as exc:
        stopped_reason = f"exception:{exc!r}"
        traceback.print_exc()

    finally:
        cb_reason = diag_cb.stopped_reason
        reason_final = stopped_reason if stopped_reason else (str(cb_reason) if cb_reason else None)
        abnormal = reason_final not in (None, "")

        n_ts = int(model.num_timesteps) if model is not None else 0
        ts_req = int(args.timesteps)
        wall_elapsed = (time.perf_counter() - train_wall_start) if train_wall_start is not None else 0.0

        is_interrupt = stopped_reason == "keyboard_interrupt"
        is_exception = bool(stopped_reason) and str(stopped_reason).startswith("exception:")
        is_early = bool(cb_reason) and str(cb_reason).startswith("early_stop:")
        completed_ok = (
            not is_interrupt
            and not is_exception
            and not is_early
            and n_ts >= ts_req
            and model is not None
        )

        if is_interrupt:
            early_stop_line = "KeyboardInterrupt"
        elif is_exception:
            early_stop_line = str(stopped_reason)
        elif is_early and cb_reason:
            early_stop_line = str(cb_reason)
        else:
            early_stop_line = "completed"

        esp = run_dir / "logs" / "early_stop_reason.txt"
        esp.parent.mkdir(parents=True, exist_ok=True)
        esp.write_text(early_stop_line + "\n", encoding="utf-8")

        stop_tag = early_stop_line
        status_payload: dict[str, Any] = {
            "stop_reason": stop_tag,
            "completed": completed_ok,
            "early_stopped": is_early,
            "interrupted": is_interrupt,
            "exception": is_exception,
            "final_timesteps": n_ts,
            "wall_time": float(wall_elapsed),
        }
        (run_dir / "logs" / "stop_status.yaml").write_text(
            yaml.safe_dump(status_payload, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )

        stopped_zip = run_dir / "checkpoints" / "stopped_model.zip"
        if not abnormal and stopped_zip.is_file():
            stopped_zip.unlink(missing_ok=True)

        if model is not None:
            try:
                diag_cb.record_final_eval_csv(timestep_override=int(model.num_timesteps))
            except Exception:
                pass
            if abnormal:
                try:
                    model.save(str(run_dir / "checkpoints" / "stopped_model"))
                except Exception:
                    pass
            try:
                _save_all(
                    model,
                    vec,
                    run_dir=run_dir,
                    replay_path=replay_path,
                    vn_path=vn_path,
                    use_vn=use_vn,
                    final_stem="final_model",
                )
            except Exception:
                pass

        try:
            if use_vn and isinstance(vec, VecNormalize):
                vec.save(str(vn_path))
        except Exception:
            pass

        summary_stop = str(reason_final) if reason_final else "completed"
        _write_summary_md(
            run_dir,
            timesteps=int(args.timesteps),
            profile=str(args.profile),
            stopped_reason=summary_stop,
            baseline_metrics=dict(baseline_metrics),
            run_name=str(args.run_name),
        )
        _write_next_run_recommendation_md(
            run_dir,
            baseline_metrics=dict(baseline_metrics),
            run_name=str(args.run_name),
        )

        try:
            vec.close()
        except Exception:
            pass

        if args.analyze_after:
            an_py = _ROOT / "scripts" / "analyze_sac_training_logs.py"
            if an_py.is_file():
                subprocess.run([sys.executable, str(an_py), "--run-dir", str(run_dir)], check=False)

    print(f"[train_sac_residual] run_dir={run_dir}")


if __name__ == "__main__":
    main()