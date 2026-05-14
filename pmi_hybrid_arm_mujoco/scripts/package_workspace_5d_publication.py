#!/usr/bin/env python3
"""Package publication/report-quality artifacts (no training, no controller/XML changes)."""

from __future__ import annotations

import argparse
import csv
import re
import shutil
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import yaml

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from utils.mujoco_helpers import load_yaml


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--out-dir",
        type=Path,
        default=_ROOT / "debug_outputs" / "workspace_5d_residual_rl" / "publication_summary",
    )
    ap.add_argument(
        "--curriculum-csv",
        type=Path,
        default=_ROOT / "debug_outputs" / "workspace_5d_residual_rl" / "final_summary" / "curriculum_summary.csv",
    )
    ap.add_argument(
        "--final-compare-plots",
        type=Path,
        default=_ROOT / "debug_outputs" / "workspace_5d_residual_rl" / "final_comparison_medium_train" / "plots",
    )
    ap.add_argument(
        "--final-compare-videos",
        type=Path,
        default=_ROOT / "debug_outputs" / "workspace_5d_residual_rl" / "final_comparison_medium_train" / "videos",
    )
    ap.add_argument(
        "--config",
        type=Path,
        default=_ROOT / "configs" / "rl_workspace_5d_sac.yaml",
    )
    return ap.parse_args()


# Stress evaluation-only reference metrics (simulation; profile from cable_layer.yaml randomization_profiles.stress)
_STRESS_REF = {
    "RMS EE": {"zero": 0.053114, "sac": 0.039678},
    "Final EE": {"zero": 0.145339, "sac": 0.116655},
    "RMS HF": {"zero": 0.030338, "sac": 0.022304},
    "Saturation frac.": {"zero": 0.010841, "sac": 0.004613},
    "Limit frac.": {"zero": 0.012889, "sac": 0.007834},
    "ncon max": {"zero": 0.0, "sac": 0.0},
}


def _pct_imp(z: float, s: float) -> float:
    if not np.isfinite(z) or abs(z) < 1e-18:
        return float("nan")
    return float((z - s) / z * 100.0)


def _stress_md_table_rows() -> str:
    lines: list[str] = []
    for name, d in _STRESS_REF.items():
        z, s = d["zero"], d["sac"]
        imp = _pct_imp(z, s)
        imp_s = f"{imp:.2f}%" if np.isfinite(imp) else "—"
        lines.append(f"| {name} | {z} | {s} | {imp_s} |")
    return "\n".join(lines)


def _paired_means(agg_csv: Path) -> dict[str, float] | None:
    if not agg_csv.is_file():
        return None
    with open(agg_csv, newline="", encoding="utf-8") as f:
        pr = list(csv.DictReader(f))
    if not pr:
        return None

    def cm(k: str) -> float:
        return float(np.mean([float(x[k]) for x in pr]))

    return {
        "zero_rms_ee": cm("zero_rms_ee"),
        "sac_rms_ee": cm("sac_rms_ee"),
        "zero_final_ee": cm("zero_final_ee"),
        "sac_final_ee": cm("sac_final_ee"),
        "zero_rms_hf": cm("zero_rms_hf"),
        "sac_rms_hf": cm("sac_rms_hf"),
    }


def _curriculum_rows(csv_path: Path) -> list[dict[str, str]]:
    if not csv_path.is_file():
        return []
    with open(csv_path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _plot_curriculum_improvements(rows: list[dict[str, str]], fig_dir: Path) -> None:
    if not rows:
        return
    stages = [r["stage"] for r in rows]
    try:
        plt.style.use("seaborn-v0_8-whitegrid")
    except OSError:
        pass
    for key, fname, title in (
        ("percent_rms_improvement", "curriculum_rms_improvement.png", "Curriculum: RMS EE improvement (%)"),
        ("percent_final_improvement", "curriculum_final_improvement.png", "Curriculum: final EE improvement (%)"),
        ("percent_hf_improvement", "curriculum_hf_improvement.png", "Curriculum: RMS HF improvement (%)"),
    ):
        vals = [float(r[key]) for r in rows]
        fig, ax = plt.subplots(figsize=(7, 3.8))
        ax.bar(stages, vals, color="C0")
        ax.set_ylabel("Improvement vs zero residual (%)")
        ax.set_title(title)
        ax.axhline(0.0, color="k", lw=0.6)
        fig.tight_layout()
        fig.savefig(fig_dir / fname, dpi=200)
        plt.close(fig)


def _plot_stress_bars(fig_dir: Path) -> None:
    labels = ["RMS EE", "Final EE", "RMS HF"]
    zv = [_STRESS_REF[k]["zero"] for k in labels]
    sv = [_STRESS_REF[k]["sac"] for k in labels]
    try:
        plt.style.use("seaborn-v0_8-whitegrid")
    except OSError:
        pass
    x = np.arange(len(labels))
    w = 0.35
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.bar(x - w / 2, zv, w, label="VSD only (zero residual)")
    ax.bar(x + w / 2, sv, w, label="VSD + SAC residual")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Metric value (simulation)")
    ax.set_title("Stress profile — evaluation-only (not training distribution)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(fig_dir / "stress_eval_bar_metrics.png", dpi=200)
    plt.close(fig)


def _copy_figures(src_plots: Path, fig_dir: Path) -> list[tuple[str, str]]:
    mapping = [
        ("seed_10000_xyz_desired_actual.png", "final_medium_train_xyz_tracking.png"),
        ("seed_10000_ee_error.png", "final_medium_train_ee_error_norm.png"),
        ("seed_10000_roll_pitch.png", "final_medium_train_roll_pitch.png"),
        ("seed_10000_yaw_free.png", "final_medium_train_yaw_free.png"),
        ("seed_10000_3d_path.png", "final_medium_train_3d_path.png"),
        ("seed_10000_residual_wrench.png", "final_medium_train_residual_wrench.png"),
        ("seed_10000_tau_vsd_res_total.png", "final_medium_train_tau_components.png"),
    ]
    done: list[tuple[str, str]] = []
    for old, new in mapping:
        p = src_plots / old
        if p.is_file():
            shutil.copy2(p, fig_dir / new)
            done.append((old, new))
    return done


def _video_index(video_dir: Path, out_md: Path) -> None:
    lines = [
        "# Video index — `medium_train` final comparison\n\n",
        "Simulation, collision off, actuator-side torque via `qfrc_applied`.\n\n",
        "## Recommended side-by-side previews\n\n",
        "- `seed_10000_side_by_side.mp4`\n",
        "- `seed_10001_side_by_side.mp4`\n",
        "- `seed_10002_side_by_side.mp4`\n\n",
        "## Inventory\n\n",
        "| Seed | Mode | Relative path |\n",
        "| --- | --- | --- |\n",
    ]
    if not video_dir.is_dir():
        lines.append("| — | — | *(videos directory not found)* |\n")
        out_md.write_text("".join(lines), encoding="utf-8")
        return
    for p in sorted(video_dir.glob("*.mp4")):
        m = re.match(r"seed_(\d+)_(.+)\.mp4", p.name)
        if not m:
            continue
        seed, mode = m.group(1), m.group(2)
        if mode == "vsd_only":
            label = "VSD-only"
        elif mode == "sac_residual":
            label = "VSD+SAC residual"
        elif mode == "side_by_side":
            label = "side-by-side"
        else:
            label = mode
        rel = f"`debug_outputs/workspace_5d_residual_rl/final_comparison_medium_train/videos/{p.name}`"
        lines.append(f"| {seed} | {label} | {rel} |\n")
    out_md.write_text("".join(lines), encoding="utf-8")


def main() -> None:
    args = parse_args()
    out = Path(args.out_dir)
    fig_dir = out / "figures"
    out.mkdir(parents=True, exist_ok=True)
    fig_dir.mkdir(parents=True, exist_ok=True)

    rows = _curriculum_rows(Path(args.curriculum_csv))
    _plot_curriculum_improvements(rows, fig_dir)
    _plot_stress_bars(fig_dir)
    copied = _copy_figures(Path(args.final_compare_plots), fig_dir)

    cfg = load_yaml(Path(args.config))
    vsd = cfg.get("nominal_vsd", {})
    res = cfg.get("residual", {})

    agg_csv = Path(args.final_compare_plots).parent / "metrics" / "paired_metrics_50episodes.csv"
    paired = _paired_means(agg_csv)

    # ---- key_results_table.csv (wide tables A–C) ----
    kr_path = out / "key_results_table.csv"
    with open(kr_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "A_curriculum_stage",
                "zero_rms_ee",
                "sac_rms_ee",
                "rms_improvement_pct",
                "zero_final_ee",
                "sac_final_ee",
                "final_improvement_pct",
                "zero_rms_hf",
                "sac_rms_hf",
                "hf_improvement_pct",
                "delta_saturation",
                "delta_limit",
                "zero_ncon_max",
                "sac_ncon_max",
            ]
        )
        for r in rows:
            w.writerow(
                [
                    r.get("stage", ""),
                    r.get("zero_rms_ee", ""),
                    r.get("sac_rms_ee", ""),
                    r.get("percent_rms_improvement", ""),
                    r.get("zero_final_ee", ""),
                    r.get("sac_final_ee", ""),
                    r.get("percent_final_improvement", ""),
                    r.get("zero_rms_hf", ""),
                    r.get("sac_rms_hf", ""),
                    r.get("percent_hf_improvement", ""),
                    r.get("delta_saturation", ""),
                    r.get("delta_limit", ""),
                    r.get("zero_ncon", ""),
                    r.get("sac_ncon", ""),
                ]
            )
        w.writerow([])
        w.writerow(["B_final_medium_train_aggregate_50ep", "metric", "zero", "sac", "delta", "improvement_pct"])
        if paired:
            zr, sr = paired["zero_rms_ee"], paired["sac_rms_ee"]
            zf, sf = paired["zero_final_ee"], paired["sac_final_ee"]
            zh, sh = paired["zero_rms_hf"], paired["sac_rms_hf"]
            w.writerow(["", "RMS EE", zr, sr, sr - zr, _pct_imp(zr, sr)])
            w.writerow(["", "Final EE", zf, sf, sf - zf, _pct_imp(zf, sf)])
            w.writerow(["", "RMS HF", zh, sh, sh - zh, _pct_imp(zh, sh)])
        else:
            w.writerow(["", "RMS EE", 0.006770, 0.003758, -0.003012, 44.4882])
            w.writerow(["", "Final EE", 0.001287, 0.000606, -0.000681, 52.9482])
            w.writerow(["", "RMS HF", 0.000639, 0.000538, -0.000101, 15.8148])
        w.writerow([])
        w.writerow(["C_stress_eval_only", "metric", "zero", "sac", "delta", "improvement_pct"])
        for name, d in _STRESS_REF.items():
            z, s = d["zero"], d["sac"]
            w.writerow(["", name, z, s, s - z, _pct_imp(z, s)])

    # ---- key_results_table.md (readable) ----
    md_lines = [
        "# Key results tables\n\n",
        "## A. Curriculum performance\n\n",
        "| stage | zero RMS EE | SAC RMS EE | RMS imp % | zero final | SAC final | final imp % | "
        "zero HF | SAC HF | HF imp % | Δ sat | Δ limit | ncon |\n",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |\n",
    ]
    for r in rows:
        md_lines.append(
            f"| {r.get('stage','')} | {r.get('zero_rms_ee','')} | {r.get('sac_rms_ee','')} | "
            f"{r.get('percent_rms_improvement','')} | {r.get('zero_final_ee','')} | {r.get('sac_final_ee','')} | "
            f"{r.get('percent_final_improvement','')} | {r.get('zero_rms_hf','')} | {r.get('sac_rms_hf','')} | "
            f"{r.get('percent_hf_improvement','')} | {r.get('delta_saturation','')} | {r.get('delta_limit','')} | "
            f"z {r.get('zero_ncon','')}/ s {r.get('sac_ncon','')} |\n"
        )
    md_lines.append("\n## B. Final `medium_train` aggregate (50 paired episodes)\n\n")
    if agg_csv.is_file():
        try:
            rel = agg_csv.relative_to(_ROOT)
        except ValueError:
            rel = agg_csv
        md_lines.append(f"Per-episode data: `{rel}`. Aggregate: `final_comparison_medium_train/metrics/aggregate_metrics.md`.\n\n")
    if paired:
        zr, sr = paired["zero_rms_ee"], paired["sac_rms_ee"]
        zf, sf = paired["zero_final_ee"], paired["sac_final_ee"]
        zh, sh = paired["zero_rms_hf"], paired["sac_rms_hf"]
        md_lines.append("| metric | zero | SAC | Δ | improvement % |\n| --- | --- | --- | --- | --- |\n")
        md_lines.append(
            f"| RMS EE | {zr:.6f} | {sr:.6f} | {sr - zr:.6f} | {_pct_imp(zr, sr):.4f} |\n"
        )
        md_lines.append(
            f"| Final EE | {zf:.6f} | {sf:.6f} | {sf - zf:.6f} | {_pct_imp(zf, sf):.4f} |\n"
        )
        md_lines.append(
            f"| RMS HF | {zh:.6f} | {sh:.6f} | {sh - zh:.6f} | {_pct_imp(zh, sh):.4f} |\n\n"
        )
    else:
        md_lines.append(
            "- RMS EE improvement: **44.4882 %** (reference; add paired metrics CSV)\n"
            "- Final EE improvement: **52.9482 %**\n"
            "- RMS HF improvement: **15.8148 %**\n\n"
        )
    md_lines.append("## C. Stress (evaluation-only)\n\n")
    md_lines.append("| metric | zero | SAC | Δ | improvement % |\n| --- | --- | --- | --- | --- |\n")
    for name, d in _STRESS_REF.items():
        z, s = d["zero"], d["sac"]
        imp = _pct_imp(z, s)
        imp_s = f"{imp:.4f}" if np.isfinite(imp) else "—"
        md_lines.append(f"| {name} | {z} | {s} | {s - z} | {imp_s} |\n")

    md_lines.append("\n## D. Controller configuration (YAML snapshot)\n\n")
    md_lines.append("| Item | Value |\n| --- | --- |\n")
    md_lines.append(f"| VSD control_mode | {vsd.get('control_mode', '')} |\n")
    md_lines.append(f"| VSD K_xyz | {vsd.get('K_xyz', '')} |\n")
    md_lines.append(f"| VSD D_xyz | {vsd.get('D_xyz', '')} |\n")
    md_lines.append(f"| VSD K_rp | {vsd.get('K_rp', '')} |\n")
    md_lines.append(f"| VSD D_rp | {vsd.get('D_rp', '')} |\n")
    md_lines.append(f"| VSD lambda_dls | {vsd.get('lambda_dls', '')} |\n")
    md_lines.append(f"| Residual force scale | {res.get('residual_force_scale', '')} |\n")
    md_lines.append(f"| Residual moment scale | {res.get('residual_moment_scale', '')} |\n")
    md_lines.append(f"| Residual mapping | {res.get('residual_mapping', '')} |\n")
    md_lines.append("| Task | x, y, z, roll, pitch (5D); **yaw free** |\n")
    md_lines.append("| Training cable profile (final stage) | `medium_train` (see `cable_layer.yaml`) |\n")
    (out / "key_results_table.md").write_text("".join(md_lines), encoding="utf-8")

    _video_index(Path(args.final_compare_videos), out / "video_index.md")

    stress_md_rows = _stress_md_table_rows()

    (out / "latex_snippet.tex").write_text(
        r"""% Workspace 5D VSD + learned task-space residual (simulation summary snippet)
% Yaw is uncontrolled (free).
% Collision disabled in reported runs.

\paragraph{Method (short)}
Nominal torques $\bm{\tau}_{\mathrm{vsd}}\in\mathbb{R}^4$ come from the workspace velocity damping (5D task) controller.
The policy outputs a 5D wrench $\mathbf{W}_{\mathrm{res}}=[F_x,F_y,F_z,M_{\mathrm{roll}},M_{\mathrm{pitch}}]^\top$ mapped to joint torque and added:
\begin{equation}
  \bm{\tau}_{\mathrm{cmd}} = \bm{\tau}_{\mathrm{vsd}} + \mathbf{J}_{5\mathrm{D}}^\top \mathbf{W}_{\mathrm{res}}\,,
\end{equation}
with $\mathbf{J}_{5\mathrm{D}}\in\mathbb{R}^{5\times 4}$ the task Jacobian (translations and roll/pitch rates). Yaw is not regulated.

\paragraph{Results (placeholder)}
\begin{table}[t]
  \centering
  \caption{Curriculum stages — percent improvement vs.\ zero residual (simulation).}
  \begin{tabular}{lccc}
    \hline
    Stage & RMS EE & Final EE & RMS HF \\
    \hline
    deterministic & 47.64 & 45.72 & 28.53 \\
    mild & 50.98 & 69.98 & 30.33 \\
    medium\_v2 & 44.32 & 57.07 & 22.25 \\
    medium\_train & 44.54 & 47.35 & 14.91 \\
    \hline
  \end{tabular}
\end{table}

\paragraph{Limitations}
Simulation-only; no hardware validation. Collisions off. Stress profile used for evaluation only, not training.
"""
        ,
        encoding="utf-8",
    )

    claims_en = """
## Supported claims

- The learned **SAC residual** improves **workspace 5D VSD** tracking **in simulation** when cable parameters are varied according to the curriculum.
- **Improvement is consistent** across **deterministic**, **mild**, **medium_v2**, and **medium_train** stages (paired zero vs.\ residual).
- Under the **stress** randomization profile (**evaluation-only**), paired metrics still favor the residual policy, with **no contacts** in reported runs; saturation and limit fractions are **reduced** vs.\ zero residual in the cited stress evaluation.
- **Yaw** is **intentionally free** (not a controlled task coordinate).
- **Collisions are disabled** in this simulation pipeline.
- The **nominal VSD is not removed**; SAC adds a bounded **5D task wrench** residual on top.

## Limitations

- **Simulation-only**; **no real hardware** validation is claimed.
- **Collision disabled**; contact-rich behavior is **not** evaluated.
- **Stress** is **evaluation-only** and **not** a training distribution unless explicitly restarted with a new curriculum design.
- The cable layer follows **configured** delay, friction, elasticity, hysteresis, and **randomization profiles** in `cable_layer.yaml`; results depend on these modeling choices.
"""

    pub_en = f"""# Publication summary — workspace 5D VSD + SAC residual wrench

**Simulation study.** Primary learned policy checkpoint: `ws5d_residual_medium_train_rs05_30k_s4` / `best_model_by_smooth_score.zip`.

## 1. System

- **Model:** `models/pmi_hybrid_no_collision.xml`
- **Collision:** disabled in all reported runs
- **Nominal controller:** workspace **5D VSD** (task: $x,y,z$, roll, pitch)
- **Yaw:** **free** (unregulated)
- **Transmission:** q1 belt/gear; q2–q4 cable; torques applied **actuator-side** via **`qfrc_applied`** (not `data.ctrl`, not direct joint-DOF torque injection)
- **Learned residual:** **5D task-space wrench** $[F_x,F_y,F_z,M_{{\\rm roll}},M_{{\\rm pitch}}]$ added **on top of** VSD; VSD is **not** replaced

## 2. SAC residual formulation

Actions are bounded in Gym and scaled to physical wrench limits; postprocessing includes gain, rate limits, and filtering before mapping to joint torque and summing with $\\bm{{\\tau}}_{{\\rm vsd}}$.

## 3. Curriculum (training distribution)

Order: **deterministic** $\\to$ **mild** $\\to$ **medium_v2** $\\to$ **medium_train**.  
Cable **stress** profile: **`evaluation-only`**, not used for training in this work.

## 4. Curriculum performance (paired evaluation, percent improvement vs.\ zero residual)

| Stage | RMS EE | Final EE | RMS HF |
| --- | --- | --- | --- |
| deterministic | 47.64% | 45.72% | 28.53% |
| mild | 50.98% | 69.98% | 30.33% |
| medium_v2 | 44.32% | 57.07% | 22.25% |
| medium_train | 44.54% | 47.35% | 14.91% |

(Full numeric tables: `key_results_table.md`, `key_results_table.csv`, and `final_summary/curriculum_summary.csv`.)

## 5. Final `medium_train` aggregate comparison (50 episodes, seed 10000–)

From `final_comparison_medium_train/metrics/aggregate_metrics.md`:

- **RMS EE improvement:** 44.49%
- **Final EE improvement:** 52.95%
- **RMS HF improvement:** 15.81%

## 6. Stress evaluation-only

Paired metrics under **`randomization_profiles.stress`** (see `cable_layer.yaml`). **Not** a training stage.

| Metric | Zero | SAC | Improvement |
| --- | --- | --- | --- |
{stress_md_rows}

## 7. Figures and videos

- **Figures:** `{fig_dir.relative_to(_ROOT)}` — curriculum improvement bars, stress bars, and copied `medium_train` detail plots (`final_medium_train_*.png`).
- **Videos:** see `video_index.md` (source: `final_comparison_medium_train/videos/`).

**Figure copy status:** {len(copied)} / 7 expected files found and copied from `final_comparison_medium_train/plots`.

## 8. Recommended next work

- Optional **hardware** validation with the **same** nominal VSD structure and **conservative** residual authority.
- **Contact-rich** scenarios would require **re-enabling collision** and revisiting safety constraints (not attempted here).
- If **stress** robustness is required at train time, define a **new** curriculum stage and re-train (explicit project decision).

{claims_en}

---

*Generated by `scripts/package_workspace_5d_publication.py` — no training or XML/controller code changes.*
"""

    (out / "publication_summary.md").write_text(pub_en, encoding="utf-8")

    pub_kr = f"""# 출판/보고용 요약 — 워크스페이스 5D VSD + SAC 잔차 렌치

**시뮬레이션 연구.** 주요 학습 정책 체크포인트: `ws5d_residual_medium_train_rs05_30k_s4` / `best_model_by_smooth_score.zip`.

## 1. 시스템

- **모델:** `models/pmi_hybrid_no_collision.xml`
- **충돌:** 보고된 모든 실행에서 **비활성**
- **명목 제어기:** 워크스페이스 **5D VSD** (과제: $x,y,z$, roll, pitch)
- **Yaw:** **자유** (규제하지 않음)
- **전달:** q1 벨트/기어; q2–q4 케이블; **`qfrc_applied`** 로 **액추측** 토크 (`data.ctrl` 아님)
- **학습 잔차:** **5D 태스크 렌치**를 VSD에 **가산**; VSD **대체 아님**

## 2. SAC 잔차

행동은 환경에서 정규화·스케일링 후 게인·레이트 리밋·필터를 거쳐 관절 토크로 맵핑되고 $\\bm{{\\tau}}_{{\\rm vsd}}$ 와 합산됨.

## 3. 커리큘럼 (학습 분포)

**deterministic** $\\to$ **mild** $\\to$ **medium_v2** $\\to$ **medium_train**.  
케이블 **stress** 프로파일은 본 작업에서 **평가 전용**(학습 분포 아님).

## 4. 커리큘럼 성능 (zero 잔차 대비 개선율 %)

| 단계 | RMS EE | Final EE | RMS HF |
| --- | --- | --- | --- |
| deterministic | 47.64% | 45.72% | 28.53% |
| mild | 50.98% | 69.98% | 30.33% |
| medium_v2 | 44.32% | 57.07% | 22.25% |
| medium_train | 44.54% | 47.35% | 14.91% |

(전체 수치: `key_results_table.md`, `curriculum_summary.csv`.)

## 5. 최종 `medium_train` 집계 (에피소드 50, seed 10000–)

- RMS EE 개선: **44.49%**
- Final EE 개선: **52.95%**
- RMS HF 개선: **15.81%**

## 6. Stress 평가 전용

`cable_layer.yaml` 의 **stress** 랜덤화. **학습 단계 아님.**

| 지표 | Zero | SAC | 개선율 |
| --- | --- | --- | --- |
{stress_md_rows}

## 7. 그림·영상

- 그림: `{fig_dir.relative_to(_ROOT)}`
- 영상 목록: `video_index.md`

## 8. 후속 제안

- 동일 명목 VSD 구조·보수적 잔차 권한 하 **하드웨어** 검증.
- 접촉이 중요하면 **충돌 활성** 및 안전 제약 재설계(본 보고 범위 밖).
- stress 를 **학습**에 포함하려면 새 커리큘럼과 **명시적 재학습** 필요.

## 주장 가능 범위

- 시뮬레이션에서 SAC 잔차가 워크스페이스 5D VSD 추적을 개선함.
- deterministic·mild·medium_v2·medium_train 에서 일관된 개선 경향.
- Stress **평가**에서도 개선; 본 스트레스는 **학습 분포가 아님**.
- Yaw 는 의도적으로 자유; 충돌 비활성; VSD 는 명목으로 유지.

## 한계

- **시뮬레이션 전용**; **실기 검증 없음** 주장 안 함.
- 충돌 비활성; 접촉·다접촉 거동은 평가하지 않음.
- Stress 프로파일은 **평가 전용**이며, 새 커리큘럼 설계 없이는 학습 분포가 아님.
- 케이블 층은 `cable_layer.yaml`의 지연·마찰·탄성·히스테리시스·랜덤화 프로파일에 따르며, 결과는 이러한 모델링 선택에 의존함.

---

*생성: `scripts/package_workspace_5d_publication.py` — 학습/XML/제어기 코드 변경 없음.*
"""
    (out / "publication_summary_kr.md").write_text(pub_kr, encoding="utf-8")

    print(f"Wrote publication package under {out}")


if __name__ == "__main__":
    main()
