#!/usr/bin/env python3
"""Soft orientation-VSD ablation: deterministic VSD-only vs VSD+SAC+postprocess, no training.

Runs three controller.orientation_soft_mode settings (baseline none, soft roll, soft roll+pitch)
with fixed postprocess overrides from the final comparison script. Does not modify the MuJoCo XML.
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

from compare_vsd_vs_sac_residual import (
    build_vec_normalize,
    episode_metrics,
    infer_run_dir,
    load_sac,
)
from envs.pmi_cable_residual_env import _deep_merge
from final_compare_vsd_vs_sac_postprocessed import (
    CFG_DEF,
    CKPT_DEF,
    VN_DEF,
    merge_env_overrides,
    rollout_logged,
)

OUT_DEF = _PKG / "debug_outputs" / "sac_residual_task_force" / "orientation_ablation"

MODES: tuple[tuple[str, str], ...] = (
    ("A_xyz_only", "none"),
    ("B_xyz_plus_roll_soft", "xyz_plus_roll_soft"),
    ("C_xyz_plus_roll_pitch_soft", "xyz_plus_roll_pitch_soft"),
)


def _merge_soft_defaults(ov: dict[str, Any]) -> dict[str, Any]:
    return _deep_merge(
        copy.deepcopy(ov),
        {
            "controller": {
                "orientation_soft_roll": {"Kp": 5.0, "Dp": 0.5},
                "orientation_soft_pitch": {"Kp": 5.0, "Dp": 0.5},
            }
        },
    )


def _set_orient_mode(ov: dict[str, Any], mode_val: str) -> dict[str, Any]:
    o2 = _merge_soft_defaults(ov)
    ctl = o2.setdefault("controller", {})
    if not isinstance(ctl, dict):
        raise TypeError("controller overrides must be a dict")
    ctl["orientation_soft_mode"] = str(mode_val)
    return o2


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", type=Path, default=CFG_DEF)
    ap.add_argument("--model-path", type=Path, default=CKPT_DEF)
    ap.add_argument("--vecnormalize-path", type=Path, default=VN_DEF)
    ap.add_argument("--profile", type=str, default="medium_train")
    ap.add_argument("--seed-start", type=int, default=10000)
    ap.add_argument("--num-episodes", type=int, default=5, help="e.g. 5 → seeds 10000…10004")
    ap.add_argument("--out-dir", type=Path, default=OUT_DEF)
    args = ap.parse_args()

    cfg = Path(args.config).resolve()
    mp = Path(args.model_path).resolve()
    vn_p = Path(args.vecnormalize_path).resolve()
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    seed0 = int(args.seed_start)
    n_ep = int(args.num_episodes)
    model = load_sac(mp)
    base_ov = merge_env_overrides(str(args.profile), infer_run_dir(mp))

    csv_rows: list[dict[str, Any]] = []

    for label, ori_yaml in MODES:
        ov_m = _set_orient_mode(base_ov, ori_yaml)
        for ep in range(n_ep):
            cable_seed = seed0 + ep
            vn_z = build_vec_normalize(cfg, vn_p, ov_m)
            rz, _ = rollout_logged(
                vn_z,
                profile=str(args.profile),
                mode="zero",
                cable_seed=cable_seed,
                model=None,
                capture_frames=False,
            )
            vn_s = build_vec_normalize(cfg, vn_p, ov_m)
            rs, _ = rollout_logged(
                vn_s,
                profile=str(args.profile),
                mode="sac",
                cable_seed=cable_seed,
                model=model,
                capture_frames=False,
            )
            mz = episode_metrics(rz)
            ms = episode_metrics(rs)
            row = {
                "mode_label": label,
                "orientation_soft_mode": ori_yaml,
                "seed": cable_seed,
            }
            for k, v in mz.items():
                row[f"vsd_{k}"] = v
            for k, v in ms.items():
                row[f"sac_{k}"] = v
            csv_rows.append(row)

    csv_path = out_dir / "orientation_ablation_metrics.csv"
    if csv_rows:
        keys = sorted({kk for r in csv_rows for kk in r})
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for r in csv_rows:
                w.writerow({k: r.get(k, "") for k in keys})

    def mean_for(mode_l: str, key: str, side: str) -> float:
        vals = [
            float(r[f"{side}_{key}"])
            for r in csv_rows
            if r["mode_label"] == mode_l and f"{side}_{key}" in r and np.isfinite(float(r[f"{side}_{key}"]))
        ]
        return float(np.mean(vals)) if vals else float("nan")

    lines = [
        "# Orientation soft-VSD ablation (deterministic, no SAC training)",
        "",
        f"- Policy: `{mp}`",
        f"- VecNormalize: `{vn_p}`",
        f"- Profile: `{args.profile}`",
        f"- Seeds: `{seed0}` … `{seed0 + n_ep - 1}`",
        "- Post-process overrides match `scripts/final_compare_vsd_vs_sac_postprocessed.POSTPROCESS_OVERRIDES`.",
        "- When orientation mode is not `none`, control uses `orientation_soft_roll` / `orientation_soft_pitch` with **Kp=5**, **Dp=0.5** (script-enforced merge).",
        "",
        "## Metrics (mean over seeds per mode)",
        "",
        "| Mode | RMS_EE_V | RMS_EE_S | fin_EE_V | fin_EE_S | HF_V | HF_S | P2P_V | P2P_S | rmsR_V | rmsR_S | rmsP_V | rmsP_S | sat_V | sat_S | lim_V | lim_S | ncon_V | ncon_S |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]

    for label, ori_yaml in MODES:
        def mf(k: str, side: str) -> float:
            return mean_for(label, k, side)

        lines.append(
            f"| `{label}` (`{ori_yaml}`) "
            f"| {mf('rms_ee_error', 'vsd'):.7g} | {mf('rms_ee_error', 'sac'):.7g} "
            f"| {mf('final_ee_error', 'vsd'):.7g} | {mf('final_ee_error', 'sac'):.7g} "
            f"| {mf('rms_ee_error_highfreq', 'vsd'):.7g} | {mf('rms_ee_error_highfreq', 'sac'):.7g} "
            f"| {mf('p2p_error_norm', 'vsd'):.7g} | {mf('p2p_error_norm', 'sac'):.7g} "
            f"| {mf('rms_roll_error', 'vsd'):.7g} | {mf('rms_roll_error', 'sac'):.7g} "
            f"| {mf('rms_pitch_error', 'vsd'):.7g} | {mf('rms_pitch_error', 'sac'):.7g} "
            f"| {mf('saturation_fraction', 'vsd'):.7g} | {mf('saturation_fraction', 'sac'):.7g} "
            f"| {mf('limit_violation_fraction', 'vsd'):.7g} | {mf('limit_violation_fraction', 'sac'):.7g} "
            f"| {mf('ncon_max', 'vsd'):.7g} | {mf('ncon_max', 'sac'):.7g} |"
        )

    lines.extend(
        [
            "",
            "## Interpretation (read with 3D path plots)",
            "",
            "1. **Baseline (`none`)** matches nominal xyz IK + joint VSD without extra orientation torque.",
            "2. Soft roll/pitch rows add **small stabilization** torque; **4-DoF** arm cannot independently hold 5 unrelated constraints.",
            "3. Compare **HF EE RMS** and **P2P** in `orientation_ablation_metrics.csv` alongside **saturation_fraction** / **limit_violation_fraction**.",
            "",
            "4. SAC remains **Fx,Fy,Fz**-only; gains here apply to **VSD nominal** torque only (+ same residual pipeline).",
            "",
            f"- Raw rows: `{csv_path}`",
        ]
    )

    (out_dir / "orientation_ablation_report.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote {csv_path}")
    print(f"Wrote {out_dir / 'orientation_ablation_report.md'}")


if __name__ == "__main__":
    main()
