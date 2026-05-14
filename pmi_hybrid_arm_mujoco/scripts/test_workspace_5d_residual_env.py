#!/usr/bin/env python3
"""Rollout PMIWorkspace5DResidualEnv: zero / random / small sinusoid residual (no SAC)."""

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

from envs.pmi_workspace_5d_residual_env import PMIWorkspace5DResidualEnv
from utils.smooth_tracking_metrics import episode_smooth_tracking_metrics

OUT_DIR = _ROOT / "debug_outputs" / "workspace_5d_residual_rl" / "rollout_test"


def _episode_metrics(
    *,
    ee_xyz: list[np.ndarray],
    ee_norm: list[float],
    e_roll: list[float],
    e_pitch: list[float],
    ee_hf: list[np.ndarray],
    sat: list[int],
    jl: list[int],
    al: list[int],
    ncon: list[int],
    rewards: list[float],
    dt: float,
) -> dict[str, float]:
    exyz = np.stack(ee_xyz, axis=0) if ee_xyz else np.zeros((0, 3), dtype=np.float64)
    en = np.asarray(ee_norm, dtype=np.float64)
    re = np.asarray(e_roll, dtype=np.float64)
    pe = np.asarray(e_pitch, dtype=np.float64)
    osc = episode_smooth_tracking_metrics(ee_err_xyz=exyz, ee_err_norm=en, dt=float(dt))
    n = max(1, len(sat))
    lim_steps = sum(1 for i in range(len(jl)) if jl[i] or al[i])
    return {
        "rms_ee_error": osc.get("rms_ee", float(np.sqrt(np.mean(en**2))) if en.size else float("nan")),
        "final_ee_error": float(en[-1]) if en.size else float("nan"),
        "rms_roll_error": float(np.sqrt(np.mean(re**2))) if re.size else float("nan"),
        "rms_pitch_error": float(np.sqrt(np.mean(pe**2))) if pe.size else float("nan"),
        "rms_highfreq": float(osc.get("rms_ee_error_highfreq", float("nan"))),
        "p2p_error_norm": float(osc.get("p2p_error_norm", float("nan"))),
        "saturation_fraction": float(np.mean(sat)) if sat else 0.0,
        "limit_fraction": float(lim_steps / n),
        "ncon_max": float(max(ncon) if ncon else 0),
        "mean_reward": float(np.mean(rewards)) if rewards else 0.0,
    }


def rollout_policy(
    env: PMIWorkspace5DResidualEnv,
    policy_fn: Callable[[np.ndarray, int], np.ndarray],
    *,
    seed: int,
    options: dict[str, Any],
) -> dict[str, Any]:
    obs, _ = env.reset(seed=seed, options=options)
    dt = float(env.control_dt)
    ee_xyz: list[np.ndarray] = []
    ee_norm: list[float] = []
    e_roll: list[float] = []
    e_pitch: list[float] = []
    ee_hf: list[np.ndarray] = []
    sat: list[int] = []
    jl: list[int] = []
    al: list[int] = []
    ncon: list[int] = []
    rewards: list[float] = []
    obs_finite = True
    rew_finite = True
    term = trunc = False
    step_i = 0
    while True:
        if not np.isfinite(obs).all():
            obs_finite = False
        a = policy_fn(obs, step_i)
        obs, r, term, trunc, inf = env.step(a)
        rewards.append(float(r))
        if not np.isfinite(float(r)):
            rew_finite = False
        ee_xyz.append(np.asarray(inf["ee_err_xyz"], dtype=np.float64).reshape(3).copy())
        ee_norm.append(float(inf["ee_error_norm"]))
        e_roll.append(float(inf["e_roll"]))
        e_pitch.append(float(inf["e_pitch"]))
        ee_hf.append(np.asarray(inf.get("ee_err_highfreq_xyz", np.zeros(3)), dtype=np.float64).reshape(3).copy())
        sat.append(int(inf.get("saturation_count", 0)))
        jl.append(int(inf.get("joint_limit_violation", 0)))
        al.append(int(inf.get("actuator_limit_violation", 0)))
        ncon.append(int(inf.get("ncon", 0)))
        step_i += 1
        if term or trunc:
            break

    m = _episode_metrics(
        ee_xyz=ee_xyz,
        ee_norm=ee_norm,
        e_roll=e_roll,
        e_pitch=e_pitch,
        ee_hf=ee_hf,
        sat=sat,
        jl=jl,
        al=al,
        ncon=ncon,
        rewards=rewards,
        dt=dt,
    )
    m["reward_finite_flag"] = 1 if rew_finite else 0
    m["observation_finite_flag"] = 1 if obs_finite else 0
    m["num_steps"] = float(step_i)
    m["terminated"] = float(term)
    m["truncated"] = float(trunc)
    return m


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=_ROOT / "configs" / "rl_workspace_5d_sac.yaml")
    ap.add_argument("--episodes", type=int, default=1)
    ap.add_argument("--seed0", type=int, default=0)
    args = ap.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = OUT_DIR / "rollout_summary.csv"
    md_path = OUT_DIR / "rollout_report.md"

    rows: list[dict[str, Any]] = []
    policies = {
        "zero_residual": lambda o, i: np.zeros(5, dtype=np.float32),
        "sinusoid_residual": lambda o, i: np.array(
            [
                0.05 * math.sin(0.2 * math.pi * i),
                0.05 * math.cos(0.15 * math.pi * i),
                0.04 * math.sin(0.1 * math.pi * i),
                0.08 * math.sin(0.12 * math.pi * i),
                0.08 * math.cos(0.1 * math.pi * i),
            ],
            dtype=np.float32,
        ),
    }

    base_opts: dict[str, Any] = {"randomize_cable": False}

    for pname, pol in policies.items():
        for ep in range(int(args.episodes)):
            seed = int(args.seed0) + ep
            env = PMIWorkspace5DResidualEnv(config_path=args.config)
            row = rollout_policy(env, pol, seed=seed, options=base_opts)
            row["policy"] = pname
            row["episode"] = ep
            row["seed"] = seed
            rows.append(row)
            env.close()

    # random residual: separate RNG per episode
    for ep in range(int(args.episodes)):
        seed = int(args.seed0) + ep + 10000
        rng_loc = np.random.default_rng(seed ^ 0x9E3779B97F4A7C15)

        def random_pol(o: np.ndarray, i: int, _rng=rng_loc) -> np.ndarray:
            return _rng.uniform(-1.0, 1.0, size=5).astype(np.float32)

        env = PMIWorkspace5DResidualEnv(config_path=args.config)
        row = rollout_policy(env, random_pol, seed=seed, options=base_opts)
        row["policy"] = "random_residual"
        row["episode"] = ep
        row["seed"] = seed
        rows.append(row)
        env.close()

    keys = [
        "policy",
        "episode",
        "seed",
        "rms_ee_error",
        "final_ee_error",
        "rms_roll_error",
        "rms_pitch_error",
        "rms_highfreq",
        "p2p_error_norm",
        "saturation_fraction",
        "limit_fraction",
        "ncon_max",
        "mean_reward",
        "reward_finite_flag",
        "observation_finite_flag",
        "num_steps",
        "terminated",
        "truncated",
    ]
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in keys})

    def lines_for(pol: str) -> str:
        sub = [r for r in rows if r["policy"] == pol]
        if not sub:
            return ""
        r0 = sub[0]
        return (
            f"- **{pol}**: RMS EE={r0['rms_ee_error']:.6f}, final EE={r0['final_ee_error']:.6f}, "
            f"RMS HF={r0['rms_highfreq']:.6f}, sat={r0['saturation_fraction']:.4f}, "
            f"ncon_max={int(r0['ncon_max'])}, reward_fin={r0['reward_finite_flag']}, obs_fin={r0['observation_finite_flag']}\n"
        )

    md = "# Workspace 5D residual env rollout test\n\n"
    md += f"Config: `{args.config}`\n\n"
    md += "## Summary\n\n"
    order = ["zero_residual", "random_residual", "sinusoid_residual"]
    for p in order:
        md += lines_for(p)
    md += f"\nCSV: `{csv_path}`\n"
    md_path.write_text(md, encoding="utf-8")

    print(f"Wrote {csv_path} and {md_path}")


if __name__ == "__main__":
    main()
