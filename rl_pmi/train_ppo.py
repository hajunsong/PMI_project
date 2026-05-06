#!/usr/bin/env python3
"""
PMI 작업공간 추적 환경에서 PPO 학습 (보상 = ``run_vsd`` 와 같은 오차 기반 비용의 음수).

실행:

  cd /path/to/PMI
  .venv/bin/pip install -r requirements.txt
  cd rl_pmi
  ../.venv/bin/python train_ppo.py
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT.parent))

from env_args import add_pmi_track_env_arguments, pmi_track_env_kwargs  # noqa: E402
from envs.pmi_track_env import PMITrackEnv  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="PPO for PMI track env")
    add_pmi_track_env_arguments(parser)
    parser.add_argument("--timesteps", type=int, default=500_000)
    parser.add_argument("--save", type=Path, default=_ROOT / "checkpoints" / "ppo_pmi_track")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--no-tensorboard",
        action="store_true",
        help="TensorBoard 로그를 쓰지 않습니다.",
    )
    args = parser.parse_args()

    try:
        pmi_track_env_kwargs(args)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 1

    tensorboard_log = None
    if not args.no_tensorboard:
        try:
            import tensorboard  # noqa: F401
        except ImportError:
            print(
                "경고: tensorboard 미설치로 TensorBoard 로그를 건너뜁니다. "
                "`pip install tensorboard` 또는 `pip install -r requirements.txt` 권장.",
                file=sys.stderr,
            )
        else:
            tensorboard_log = str(_ROOT / "tensorboard_logs")

    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.vec_env import DummyVecEnv
    except ImportError:
        print(
            "stable-baselines3 가 필요합니다: pip install -r requirements.txt",
            file=sys.stderr,
        )
        return 1

    def make_env():
        return PMITrackEnv(**pmi_track_env_kwargs(args))

    venv = DummyVecEnv([make_env])
    model = PPO(
        "MlpPolicy",
        venv,
        verbose=1,
        seed=args.seed,
        tensorboard_log=tensorboard_log,
    )
    try:
        model.learn(total_timesteps=args.timesteps)
        save_path = args.save.resolve()
        save_path.parent.mkdir(parents=True, exist_ok=True)
        model.save(str(save_path))
        print(f"저장: {save_path}.zip")
    finally:
        model.env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
