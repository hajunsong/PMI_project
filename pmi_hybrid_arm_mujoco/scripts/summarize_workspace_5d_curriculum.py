#!/usr/bin/env python3
"""Aggregate workspace 5D residual RL evaluation CSVs/reports across curriculum stages."""

from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))


def _pct(z: float, s: float) -> float:
    if not np.isfinite(z) or abs(z) < 1e-18:
        return float("nan")
    return float((z - s) / z * 100.0)


def _mean_col(rows: list[dict[str, str]], key: str) -> float:
    vs = [float(r[key]) for r in rows if key in r and r[key] != ""]
    return float(np.mean(vs)) if vs else float("nan")


def _parse_md_table(report_path: Path) -> dict[str, Any] | None:
    if not report_path.is_file():
        return None
    text = report_path.read_text(encoding="utf-8")
    m = re.search(r"## Mean metrics\s*\n\n(\|[^\n]+\|\s*\n\|[^\n]+\|\s*\n(?:\|[^\n]+\|\s*\n)+)", text)
    if not m:
        return None
    block = m.group(1)
    lines = [ln.strip() for ln in block.splitlines() if ln.strip().startswith("|") and "---" not in ln]
    if len(lines) < 2:
        return None
    out: dict[str, tuple[float, float, float]] = {}
    for ln in lines[2:]:
        parts = [p.strip() for p in ln.split("|") if p.strip()]
        if len(parts) < 4:
            continue
        name, zv, sv, dv = parts[0], parts[1], parts[2], parts[3]
        try:
            out[name] = (float(zv), float(sv), float(dv))
        except ValueError:
            continue

    def g(metric: str) -> tuple[float, float, float] | None:
        return out.get(metric)

    rms = g("RMS EE")
    fin = g("Final EE")
    hf = g("RMS HF")
    sm = g("Smooth")
    sat = g("Saturation frac.")
    lim = g("Limit frac.")
    nc = g("ncon max")
    if rms is None or fin is None or hf is None:
        return None
    z_r, s_r, d_r = rms
    z_f, s_f, d_f = fin
    z_h, s_h, d_h = hf
    z_sm = s_sm = d_sm = float("nan")
    if sm:
        z_sm, s_sm, d_sm = sm
    z_sa = s_sa = d_sa = float("nan")
    z_li = s_li = d_li = float("nan")
    z_nc = s_nc = d_nc = float("nan")
    if sat:
        z_sa, s_sa, d_sa = sat
    if lim:
        z_li, s_li, d_li = lim
    if nc:
        z_nc, s_nc, d_nc = nc

    pct_md = {}
    for label, pattern in (
        ("rms", r"RMS EE:\s*([\d.\-]+)\s*%"),
        ("fin", r"Final EE:\s*([\d.\-]+)\s*%"),
        ("hf", r"RMS HF:\s*([\d.\-]+)\s*%"),
    ):
        mm = re.search(pattern, text)
        pct_md[label] = float(mm.group(1)) if mm else float("nan")

    flags: list[str] = []
    for fn in ("deterministic_improved", "medium_v2_improved", "medium_train_improved"):
        fm = re.search(rf"\*\*`{fn}`\*\*:\s*`(true|false)`", text, re.I)
        if fm and fm.group(1).lower() == "true":
            flags.append(fn)

    return {
        "zero_rms_ee": z_r,
        "sac_rms_ee": s_r,
        "delta_rms_ee": d_r,
        "pct_rms": pct_md.get("rms", _pct(z_r, s_r)),
        "zero_final_ee": z_f,
        "sac_final_ee": s_f,
        "delta_final_ee": d_f,
        "pct_final": pct_md.get("fin", _pct(z_f, s_f)),
        "zero_rms_hf": z_h,
        "sac_rms_hf": s_h,
        "delta_rms_hf": d_h,
        "pct_hf": pct_md.get("hf", _pct(z_h, s_h)),
        "zero_smooth": z_sm,
        "sac_smooth": s_sm,
        "delta_smooth": d_sm,
        "zero_saturation": z_sa,
        "sac_saturation": s_sa,
        "delta_saturation": d_sa,
        "zero_limit": z_li,
        "sac_limit": s_li,
        "delta_limit": d_li,
        "zero_ncon": z_nc,
        "sac_ncon": s_nc,
        "pass_flag": ";".join(flags) if flags else "",
    }


def _from_csv(csv_path: Path) -> dict[str, Any] | None:
    if not csv_path.is_file():
        return None
    with open(csv_path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None

    def zm(k: str) -> float:
        return _mean_col(rows, f"zero_{k}")

    def sm(k: str) -> float:
        return _mean_col(rows, f"sac_{k}")

    def dm(k: str) -> float:
        return _mean_col(rows, f"delta_{k}")

    z_r, s_r = zm("rms_ee_error"), sm("rms_ee_error")
    z_f, s_f = zm("final_ee_error"), sm("final_ee_error")
    z_h, s_h = zm("rms_highfreq"), sm("rms_highfreq")
    z_sm, s_sm = zm("smooth_score"), sm("smooth_score")
    z_sa, s_sa = zm("saturation_fraction"), sm("saturation_fraction")
    z_li, s_li = zm("limit_fraction"), sm("limit_fraction")
    z_nc, s_nc = zm("ncon_max"), sm("ncon_max")

    report_path = csv_path.parent / "evaluation_report.md"
    pf = ""
    if report_path.is_file():
        txt = report_path.read_text(encoding="utf-8")
        flags: list[str] = []
        for fn in ("deterministic_improved", "medium_v2_improved", "medium_train_improved"):
            fm = re.search(rf"\*\*`{fn}`\*\*:\s*`(true|false)`", txt, re.I)
            if fm and fm.group(1).lower() == "true":
                flags.append(fn)
        pf = ";".join(flags)

    return {
        "zero_rms_ee": z_r,
        "sac_rms_ee": s_r,
        "delta_rms_ee": dm("rms_ee_error"),
        "pct_rms": _pct(z_r, s_r),
        "zero_final_ee": z_f,
        "sac_final_ee": s_f,
        "delta_final_ee": dm("final_ee_error"),
        "pct_final": _pct(z_f, s_f),
        "zero_rms_hf": z_h,
        "sac_rms_hf": s_h,
        "delta_rms_hf": dm("rms_highfreq"),
        "pct_hf": _pct(z_h, s_h),
        "zero_smooth": z_sm,
        "sac_smooth": s_sm,
        "delta_smooth": dm("smooth_score"),
        "zero_saturation": z_sa,
        "sac_saturation": s_sa,
        "delta_saturation": dm("saturation_fraction"),
        "zero_limit": z_li,
        "sac_limit": s_li,
        "delta_limit": dm("limit_fraction"),
        "zero_ncon": z_nc,
        "sac_ncon": s_nc,
        "pass_flag": pf,
    }


def _load_stage(stage: str, eval_dir: Path) -> dict[str, Any]:
    csv_path = eval_dir / "evaluation_summary.csv"
    report_path = eval_dir / "evaluation_report.md"
    row: dict[str, Any] = {"stage": stage, "source": ""}
    data = _from_csv(csv_path)
    if data:
        row["source"] = str(csv_path)
    else:
        data = _parse_md_table(report_path)
        if data:
            row["source"] = str(report_path)
    if not data:
        row["note"] = "missing_csv_and_md"
        for k in (
            "zero_rms_ee",
            "sac_rms_ee",
            "delta_rms_ee",
            "pct_rms",
            "zero_final_ee",
            "sac_final_ee",
            "delta_final_ee",
            "pct_final",
            "zero_rms_hf",
            "sac_rms_hf",
            "delta_rms_hf",
            "pct_hf",
            "zero_smooth",
            "sac_smooth",
            "delta_smooth",
            "zero_saturation",
            "sac_saturation",
            "delta_saturation",
            "zero_limit",
            "sac_limit",
            "delta_limit",
            "zero_ncon",
            "sac_ncon",
            "pass_flag",
        ):
            row[k] = float("nan") if k not in ("pass_flag",) else ""
        return row
    row.update(data)
    return row


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = [
        "stage",
        "zero_rms_ee",
        "sac_rms_ee",
        "delta_rms_ee",
        "percent_rms_improvement",
        "zero_final_ee",
        "sac_final_ee",
        "delta_final_ee",
        "percent_final_improvement",
        "zero_rms_hf",
        "sac_rms_hf",
        "delta_rms_hf",
        "percent_hf_improvement",
        "zero_smooth",
        "sac_smooth",
        "delta_smooth",
        "zero_saturation",
        "sac_saturation",
        "delta_saturation",
        "zero_limit",
        "sac_limit",
        "delta_limit",
        "zero_ncon",
        "sac_ncon",
        "pass_flag",
        "source",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            out = {
                "stage": r.get("stage", ""),
                "zero_rms_ee": r.get("zero_rms_ee", ""),
                "sac_rms_ee": r.get("sac_rms_ee", ""),
                "delta_rms_ee": r.get("delta_rms_ee", ""),
                "percent_rms_improvement": r.get("pct_rms", ""),
                "zero_final_ee": r.get("zero_final_ee", ""),
                "sac_final_ee": r.get("sac_final_ee", ""),
                "delta_final_ee": r.get("delta_final_ee", ""),
                "percent_final_improvement": r.get("pct_final", ""),
                "zero_rms_hf": r.get("zero_rms_hf", ""),
                "sac_rms_hf": r.get("sac_rms_hf", ""),
                "delta_rms_hf": r.get("delta_rms_hf", ""),
                "percent_hf_improvement": r.get("pct_hf", ""),
                "zero_smooth": r.get("zero_smooth", ""),
                "sac_smooth": r.get("sac_smooth", ""),
                "delta_smooth": r.get("delta_smooth", ""),
                "zero_saturation": r.get("zero_saturation", ""),
                "sac_saturation": r.get("sac_saturation", ""),
                "delta_saturation": r.get("delta_saturation", ""),
                "zero_limit": r.get("zero_limit", ""),
                "sac_limit": r.get("sac_limit", ""),
                "delta_limit": r.get("delta_limit", ""),
                "zero_ncon": r.get("zero_ncon", ""),
                "sac_ncon": r.get("sac_ncon", ""),
                "pass_flag": r.get("pass_flag", ""),
                "source": r.get("source", ""),
            }
            w.writerow(out)


def _plots(rows: list[dict[str, Any]], plot_dir: Path) -> None:
    plot_dir.mkdir(parents=True, exist_ok=True)
    valid: list[dict[str, Any]] = []
    for r in rows:
        try:
            if np.isfinite(float(r.get("zero_rms_ee", np.nan))):
                valid.append(r)
        except (TypeError, ValueError):
            continue
    if not valid:
        return
    stages = [r["stage"] for r in valid]
    x = np.arange(len(stages))
    w = 0.35

    try:
        plt.style.use("seaborn-v0_8-whitegrid")
    except OSError:
        pass

    def bar_pair(zero_key: str, sac_key: str, ylabel: str, fname: str) -> None:
        zv = [float(r[zero_key]) for r in valid]
        sv = [float(r[sac_key]) for r in valid]
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.bar(x - w / 2, zv, w, label="zero")
        ax.bar(x + w / 2, sv, w, label="SAC")
        ax.set_xticks(x)
        ax.set_xticklabels(stages)
        ax.set_ylabel(ylabel)
        ax.legend()
        fig.tight_layout()
        fig.savefig(plot_dir / fname, dpi=140)
        plt.close(fig)

    bar_pair("zero_rms_ee", "sac_rms_ee", "RMS EE [m]", "01_rms_ee_zero_vs_sac.png")
    bar_pair("zero_final_ee", "sac_final_ee", "Final EE [m]", "02_final_ee_zero_vs_sac.png")
    bar_pair("zero_rms_hf", "sac_rms_hf", "RMS HF", "03_rms_hf_zero_vs_sac.png")
    bar_pair("zero_smooth", "sac_smooth", "Smooth score", "04_smooth_zero_vs_sac.png")

    fig, ax = plt.subplots(figsize=(8, 4))
    for i, key in enumerate(("pct_rms", "pct_final", "pct_hf")):
        vals = [float(r.get(key, np.nan)) for r in valid]
        ax.bar(x + (i - 1) * w / 3, vals, w / 3, label=key.replace("pct_", "% "))
    ax.set_xticks(x)
    ax.set_xticklabels(stages)
    ax.axhline(0.0, color="k", lw=0.5)
    ax.set_ylabel("percent improvement (%)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_dir / "05_percent_improvement_by_stage.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(11, 3.5))
    for ax, zk, sk, tit in zip(
        axes,
        ("zero_saturation", "zero_limit", "zero_ncon"),
        ("sac_saturation", "sac_limit", "sac_ncon"),
        ("Saturation", "Limit frac.", "ncon (mean)"),
    ):
        zv = [float(r.get(zk, np.nan)) for r in valid]
        sv = [float(r.get(sk, np.nan)) for r in valid]
        ax.bar(x - w / 2, zv, w, label="zero")
        ax.bar(x + w / 2, sv, w, label="SAC")
        ax.set_xticks(x)
        ax.set_xticklabels(stages, rotation=15, ha="right")
        ax.set_title(tit)
        ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(plot_dir / "06_safety_sat_limit_ncon.png", dpi=140)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--out-root",
        type=Path,
        default=_ROOT / "debug_outputs" / "workspace_5d_residual_rl" / "final_summary",
    )
    args = ap.parse_args()
    base = _ROOT / "debug_outputs" / "workspace_5d_residual_rl" / "evaluation"
    specs = [
        ("deterministic", base / "ws5d_residual_det_rs05_resume_30k_s1"),
        ("mild", base / "ws5d_residual_mild_rs05_30k_s2"),
        ("medium_v2", base / "ws5d_residual_medium_v2_rs05_30k_s3"),
        ("medium_train", base / "ws5d_residual_medium_train_rs05_30k_s4"),
    ]
    rows = [_load_stage(st, ed) for st, ed in specs]

    out_root = Path(args.out_root)
    out_root.mkdir(parents=True, exist_ok=True)
    csv_out = out_root / "curriculum_summary.csv"
    md_out = out_root / "curriculum_summary.md"
    _write_csv(csv_out, rows)

    lines = [
        "# Workspace 5D curriculum evaluation summary\n\n",
        "Aggregated from per-stage `evaluation_summary.csv` or `evaluation_report.md`.\n\n",
        f"- CSV: `{csv_out}`\n\n",
        "## Table\n\n",
        "| stage | Δ RMS EE | % RMS | Δ final | % final | Δ HF | % HF | pass flags |\n",
        "| --- | --- | --- | --- | --- | --- | --- | --- |\n",
    ]
    for r in rows:
        def fmt(x: Any, spec: str) -> str:
            try:
                v = float(x)
                if np.isfinite(v):
                    return format(v, spec)
            except (TypeError, ValueError):
                pass
            return str(x)

        lines.append(
            f"| {r.get('stage','')} | {fmt(r.get('delta_rms_ee'), '.6f')} | {fmt(r.get('pct_rms'), '.4f')} | "
            f"{fmt(r.get('delta_final_ee'), '.6f')} | {fmt(r.get('pct_final'), '.4f')} | {fmt(r.get('delta_rms_hf'), '.6f')} | "
            f"{fmt(r.get('pct_hf'), '.4f')} | `{r.get('pass_flag','')}` |\n"
        )
    lines.append("\n## Plots\n\n")
    lines.append(f"See `{out_root / 'plots'}`.\n")
    md_out.write_text("".join(lines), encoding="utf-8")

    _plots(rows, out_root / "plots")
    print(f"Wrote {csv_out}, {md_out}, plots under {out_root / 'plots'}")


if __name__ == "__main__":
    main()
