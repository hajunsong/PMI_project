#!/usr/bin/env python3
"""
학습된 SAC 체크포인트(.zip)를 불러와 MuJoCo 환경에서 에피소드 실행 (영상 없음).

환경 옵션은 학습 때와 맞추세요 (`train_sac.py` 와 동일 플래그).

  ./.venv/bin/python run_policy.py --model checkpoints/sac_pmi_track.zip --episodes 3
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT.parent))

from env_args import add_pmi_track_env_arguments, pmi_track_env_kwargs  # noqa: E402
from envs.pmi_track_env import PMITrackEnv  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Run trained SAC policy on PMITrackEnv (no video)")
    parser.add_argument("--model", type=Path, required=True, help="SB3 SAC .zip")
    parser.add_argument("--episodes", type=int, default=3)
    add_pmi_track_env_arguments(parser)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--stochastic",
        action="store_true",
        help="정책을 확률적으로 샘플 (기본 deterministic)",
    )
    args = parser.parse_args()

    try:
        pmi_track_env_kwargs(args)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 1

    deterministic = not args.stochastic

    try:
        from stable_baselines3 import SAC
    except ImportError:
        print("pip install stable-baselines3", file=sys.stderr)
        return 1

    env = PMITrackEnv(**pmi_track_env_kwargs(args))
    model = SAC.load(args.model, device="cpu")

    rng = np.random.default_rng(args.seed)
    returns: list[float] = []
    track_costs: list[float] = []
    for ep in range(args.episodes):
        obs, _ = env.reset(seed=int(rng.integers(0, 2**31 - 1)))
        total = 0.0
        steps = 0
        term = trunc = False
        sum_err_pos = 0.0
        sum_track_cost = 0.0
        last_info: dict = {}
        while not (term or trunc):
            action, _ = model.predict(obs, deterministic=deterministic)
            obs, rew, term, trunc, info = env.step(action)
            total += float(rew)
            steps += 1
            last_info = info
            sum_err_pos += float(info.get("err_pos_norm", 0.0))
            sum_track_cost += float(info.get("tracking_cost", 0.0))
        mean_err = sum_err_pos / max(steps, 1)
        mean_cost = sum_track_cost / max(steps, 1)
        end = "truncated" if trunc and not term else ("terminated" if term else "done")
        # 한 줄이 너무 길면 터미널 폭에서 두 줄로 깨져 중복처럼 보이므로 두 줄로 출력
        print(
            f"episode {ep + 1}/{args.episodes}: steps={steps} return={total:.4f} ({end})"
        )
        print(
            f"  err_pos_mean={mean_err:.5f} err_pos_last={last_info.get('err_pos_norm', 0):.5f} "
            f"track_cost_mean={mean_cost:.6f}"
        )
        returns.append(total)
        track_costs.append(mean_cost)

    if len(returns) >= 1:
        r = np.asarray(returns, dtype=float)
        c = np.asarray(track_costs, dtype=float)
        print("---")
        print(
            f"요약 ({args.episodes} episodes): return mean={r.mean():.4f} median={np.median(r):.4f} "
            f"std={r.std():.4f} min={r.min():.4f} max={r.max():.4f}"
        )
        print(
            f"  track_cost_mean: mean={c.mean():.6f} median={np.median(c):.6f} "
            f"std={c.std():.6f} min={c.min():.6f} max={c.max():.6f}"
        )

    env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
