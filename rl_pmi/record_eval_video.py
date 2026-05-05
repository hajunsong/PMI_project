#!/usr/bin/env python3
"""
학습된 SAC 정책으로 에피소드 rollout 영상(mp4) 저장.

  ./.venv/bin/python record_eval_video.py --model checkpoints/sac_pmi_track.zip \\
      --out videos/eval_run.mp4
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
    parser = argparse.ArgumentParser(description="Record MuJoCo rollout video from a trained SAC zip")
    parser.add_argument("--model", type=Path, required=True, help="SB3 SAC .zip 체크포인트")
    parser.add_argument("--out", type=Path, default=_ROOT / "videos" / "eval_sac.mp4")
    add_pmi_track_env_arguments(parser)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--stochastic",
        action="store_true",
        help="정책 출력을 확률적으로 샘플 (기본은 deterministic)",
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
        from moviepy.video.io.ImageSequenceClip import ImageSequenceClip
    except ImportError as exc:
        print("필요 패키지: pip install stable-baselines3 moviepy", file=sys.stderr)
        raise SystemExit(1) from exc

    env = PMITrackEnv(**pmi_track_env_kwargs(args, render_mode="rgb_array"))
    model = SAC.load(args.model)

    frames: list = []
    obs, _ = env.reset(seed=args.seed)
    frames.append(env.render())
    term = trunc = False
    while not (term or trunc):
        action, _ = model.predict(obs, deterministic=deterministic)
        obs, _rew, term, trunc, _info = env.step(action)
        frames.append(env.render())

    if len(frames) < 2:
        print("프레임이 부족합니다. 에피소드가 바로 끝났을 수 있습니다.", file=sys.stderr)
        return 1

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fps = float(env.metadata.get("render_fps", 60))
    clip = ImageSequenceClip(frames, fps=fps)
    clip.write_videofile(str(args.out.resolve()))
    print(f"저장: {args.out.resolve()}")
    env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
