#!/usr/bin/env python3
"""Rollout PMICableResidualEnv with scripted policies (no SAC training required)."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Any, Callable

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np

from envs.pmi_cable_residual_env import PMICableResidualEnv

OUT_DIR = _ROOT / "debug_outputs" / "rl_env"


def rollout_one(
    env: PMICableResidualEnv,
    policy_fn: Callable[[np.ndarray, int], np.ndarray],
    *,
    seed: int,
    options: dict[str, Any],
) -> dict[str, Any]:
    obs, inf0 = env.reset(seed=seed, options=options)
    total_r = 0.0
    ee_list: list[float] = []
    qe_list: list[float] = []
    fn_list: list[float] = []
    tn_list: list[float] = []
    sat_steps = jlim = alim = ncon_max = 0
    nan_flag = False
    trunc = term = False

    if not np.isfinite(obs).all():
        nan_flag = True

    step_idx = 0
    while True:
        a = policy_fn(obs, step_idx)
        obs, r, term, trunc, inf = env.step(a)
        step_idx += 1
        total_r += float(r)
        ee_list.append(float(inf.get("ee_error_norm", 0.0)))
        qe_list.append(float(inf.get("q_error_norm", 0.0)))
        F = np.asarray(inf.get("F_residual_xyz", np.zeros(3)), dtype=float)
        tau = np.asarray(inf.get("tau_residual_jnt", np.zeros(4)), dtype=float)
        fn_list.append(float(np.linalg.norm(F)))
        tn_list.append(float(np.linalg.norm(tau)))
        sat_steps += int(inf.get("saturation_count", 0) or 0)
        jlim += int(inf.get("joint_limit_violation", 0) or 0)
        alim += int(inf.get("actuator_limit_violation", 0) or 0)
        ncon_max = max(ncon_max, int(inf.get("ncon", 0) or 0))
        if not np.isfinite(obs).all() or not np.isfinite(r):
            nan_flag = True
        if term or trunc:
            break

    ee = np.asarray(ee_list, dtype=float)
    qe = np.asarray(qe_list, dtype=float)
    return {
        "total_reward": total_r,
        "mean_reward": float(total_r / max(1, len(ee))),
        "mean_ee_error": float(np.mean(ee)) if ee.size else 0.0,
        "max_ee_error": float(np.max(ee)) if ee.size else 0.0,
        "final_ee_error": float(ee[-1]) if ee.size else 0.0,
        "mean_q_error": float(np.mean(qe)) if qe.size else 0.0,
        "max_q_error": float(np.max(qe)) if qe.size else 0.0,
        "mean_force_residual_norm": float(np.mean(fn_list)) if fn_list else 0.0,
        "max_force_residual_norm": float(np.max(fn_list)) if fn_list else 0.0,
        "mean_tau_residual_norm": float(np.mean(tn_list)) if tn_list else 0.0,
        "max_tau_residual_norm": float(np.max(tn_list)) if tn_list else 0.0,
        "saturation_steps": sat_steps,
        "joint_limit_steps": jlim,
        "actuator_limit_steps": alim,
        "ncon_max": ncon_max,
        "nan_flag": int(nan_flag),
        "terminated": int(term),
        "truncated": int(trunc),
        "num_steps": step_idx,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--num-episodes", type=int, default=10)
    ap.add_argument("--profile", type=str, default="medium_train")
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR)
    args = ap.parse_args()

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    csv_path = out / "rollout_summary.csv"
    md_path = out / "rollout_report.md"

    base_opts: dict[str, Any] = {
        "randomize_cable": True,
        "randomization_profile": str(args.profile),
    }

    rows: list[dict[str, Any]] = []
    rng = np.random.default_rng(2026)

    for pname in ("zero_force", "random_force", "sinusoid_force"):
        env = PMICableResidualEnv()
        for ep in range(int(args.num_episodes)):
            seed = int(rng.integers(0, 2**31 - 1))

            if pname == "zero_force":

                def pol(o: np.ndarray, i: int, _p=pname) -> np.ndarray:
                    return np.zeros(3, dtype=np.float32)

            elif pname == "random_force":
                rloc = np.random.default_rng(seed ^ 0x9E3779B9)

                def pol(o: np.ndarray, i: int, _rl=rloc) -> np.ndarray:
                    return _rl.uniform(-1.0, 1.0, size=3).astype(np.float32)

            else:

                def pol(o: np.ndarray, i: int, _p=pname) -> np.ndarray:
                    return np.array(
                        [
                            0.08 * math.sin(0.2 * math.pi * i),
                            0.08 * math.cos(0.15 * math.pi * i),
                            0.05 * math.sin(0.1 * math.pi * i),
                        ],
                        dtype=np.float32,
                    )

            row = rollout_one(env, pol, seed=seed, options=base_opts)
            row["policy"] = pname
            row["episode"] = ep
            row["profile"] = args.profile
            rows.append(row)

    fieldnames = [
        "policy",
        "episode",
        "profile",
        "total_reward",
        "mean_reward",
        "mean_ee_error",
        "max_ee_error",
        "final_ee_error",
        "mean_q_error",
        "max_q_error",
        "mean_force_residual_norm",
        "max_force_residual_norm",
        "mean_tau_residual_norm",
        "max_tau_residual_norm",
        "saturation_steps",
        "joint_limit_steps",
        "actuator_limit_steps",
        "ncon_max",
        "nan_flag",
        "terminated",
        "truncated",
        "num_steps",
    ]
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in fieldnames})

    def summarize(pol: str) -> dict[str, float]:
        sub = [r for r in rows if r["policy"] == pol]
        if not sub:
            return {}

        def mean(k: str) -> float:
            return float(np.mean([float(r[k]) for r in sub]))

        keys = [
            "total_reward",
            "mean_reward",
            "mean_ee_error",
            "max_ee_error",
            "final_ee_error",
            "mean_q_error",
            "max_q_error",
            "mean_force_residual_norm",
            "max_force_residual_norm",
            "mean_tau_residual_norm",
            "max_tau_residual_norm",
            "saturation_steps",
            "joint_limit_steps",
            "actuator_limit_steps",
            "ncon_max",
            "nan_flag",
            "terminated",
            "truncated",
        ]
        return {f"m_{k}": mean(k) for k in keys}

    zs = summarize("zero_force")
    rs = summarize("random_force")
    ss = summarize("sinusoid_force")

    zsub = [r for r in rows if r["policy"] == "zero_force"]
    ncon_zero = max((int(r["ncon_max"]) for r in zsub), default=0)

    lines = [
        "# RL environment rollout (`PMICableResidualEnv`)",
        "",
        f"- Profile: `{args.profile}` (`randomize_cable=True`).",
        f"- Episodes per policy: {args.num_episodes}.",
        "",
        "## Report answers",
        "",
        "### 1. Zero residual × medium_train — VSD baseline?",
        f"- Mean total reward: **{zs.get('m_total_reward', float('nan')):.4f}**, mean EE error: **{zs.get('m_mean_ee_error', float('nan')):.6f}**.",
        f"- Max `ncon_max` over zero-residual runs: **{ncon_zero}**.",
        "",
        "### 2. Random residual — stable?",
        f"- Mean EE error: **{rs.get('m_mean_ee_error', float('nan')):.6f}**, mean terminated: **{rs.get('m_terminated', float('nan')):.3f}**, mean NaN flag: **{rs.get('m_nan_flag', float('nan')):.3f}**.",
        "",
        "### 3. Sinusoidal residual — finite / interpretable?",
        f"- Mean EE error: **{ss.get('m_mean_ee_error', float('nan')):.6f}**, mean ‖F‖: **{ss.get('m_mean_force_residual_norm', float('nan')):.6f}**.",
        "",
        "### 4–5. Finite obs / rewards?",
        f"- Total NaN-flagged rollouts: **{sum(int(r['nan_flag']) for r in rows)} / {len(rows)}**.",
        "",
        "### 6. Limits / contacts",
        f"- Zero policy soft-limit steps (mean jl / act): **{zs.get('m_joint_limit_steps', float('nan')):.1f}** / "
        f"**{zs.get('m_actuator_limit_steps', float('nan')):.1f}**.",
        "",
        "### 7. SAC smoke readiness",
        "- If Items 1–5 pass (stable, finite, `ncon_max=0`, rare limits), run "
        "`python scripts/train_sac_residual.py --timesteps 1000 --profile medium_train`.",
        "",
        "## Per-policy means",
        "",
        "| policy | mean total R | mean EE err | max EE err | sat | jl | al | nan | term |",
        "|--------|-------------|-------------|------------|-----|----|----|-----|-----|",
    ]
    for pol, s in [("zero_force", zs), ("random_force", rs), ("sinusoid_force", ss)]:
        lines.append(
            f"| {pol} | {s.get('m_total_reward', float('nan')):.4f} | {s.get('m_mean_ee_error', float('nan')):.6f} | "
            f"{s.get('m_max_ee_error', float('nan')):.6f} | {s.get('m_saturation_steps', float('nan')):.1f} | "
            f"{s.get('m_joint_limit_steps', float('nan')):.1f} | {s.get('m_actuator_limit_steps', float('nan')):.1f} | "
            f"{s.get('m_nan_flag', float('nan')):.3f} | {s.get('m_terminated', float('nan')):.3f} |"
        )
    lines.append("")
    lines.append(f"CSV: `{csv_path}`")
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {csv_path}")
    print(f"Wrote {md_path}")


if __name__ == "__main__":
    main()
