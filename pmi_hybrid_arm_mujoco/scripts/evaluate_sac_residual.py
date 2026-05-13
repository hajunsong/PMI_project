#!/usr/bin/env python3
"""평가: 잔차 0 베이스라인 대비 학습된 SAC 작업공간 잔차 힘 정책."""

from __future__ import annotations

import argparse
import csv
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from stable_baselines3 import SAC

from envs.pmi_cable_residual_env import PMICableResidualEnv

OUT_DEF = _ROOT / "debug_outputs" / "sac_residual_task_force"


def load_sac_maybe(p: Path | None) -> SAC | None:
    if p is None:
        return None
    rf = Path(p)
    zp = rf if rf.suffix == ".zip" else rf.with_suffix(".zip")
    if not zp.is_file():
        return None
    return SAC.load(str(zp))


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


def main() -> None:
    ap = argparse.ArgumentParser()

    ap.add_argument("--model-path", type=Path, default=None, help="학습된 SAC .zip (--final_model.zip 제외 접두 경로 또는 SB3 저장 경로)")

    ap.add_argument("--config", type=Path, default=_ROOT / "configs" / "rl_sac.yaml")

    ap.add_argument("--profiles", nargs="+", default=["mild", "medium_v2", "medium_train", "medium"])
    ap.add_argument("--num-episodes", type=int, default=20)
    ap.add_argument("--out-dir", type=Path, default=OUT_DEF)
    ap.add_argument("--seed-base", type=int, default=4242)

    ap.add_argument(
        "--eval-tag",
        type=str,
        default=None,
        help="runs/evaluations/<eval-tag>/ 에 CSV·MD 작성 (기본: 타임스탬프)",
    )

    args = ap.parse_args()
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
