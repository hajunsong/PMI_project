#!/usr/bin/env python3
"""
PMI 작업공간 추적 환경에서 SAC 학습 (잔여 정책: 작업공간 ΔF → τ = Jᵀ(F_VSD + ΔF)).

실행:

  cd /path/to/PMI_project/rl_pmi
  ./.venv/bin/pip install -r requirements.txt
  ./.venv/bin/python train_sac.py --timesteps 500000
  ./.venv/bin/python train_sac.py --timesteps 500000 --record-video  # 주기적 mp4 (moviepy)
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional

_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT.parent))

from env_args import add_pmi_track_env_arguments, pmi_track_env_kwargs  # noqa: E402
from envs.pmi_track_env import PMITrackEnv  # noqa: E402


def _tensorboard_log_dir(args: argparse.Namespace) -> Optional[str]:
    if args.no_tensorboard:
        return None
    try:
        import tensorboard  # noqa: F401
    except ImportError:
        print(
            "경고: tensorboard 미설치로 TensorBoard 로그를 건너뜁니다. "
            "`pip install tensorboard` 또는 `pip install -r requirements.txt` 권장.",
            file=sys.stderr,
        )
        return None
    return str(_ROOT / "tensorboard_logs")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="SAC (Soft Actor-Critic) for PMI track env — residual ΔF in task space"
    )
    add_pmi_track_env_arguments(parser)
    parser.add_argument("--timesteps", type=int, default=500_000)
    parser.add_argument("--save", type=Path, default=_ROOT / "checkpoints" / "sac_pmi_track")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--no-tensorboard",
        action="store_true",
        help="TensorBoard 로그를 쓰지 않습니다.",
    )
    parser.add_argument(
        "--device",
        default="cuda",
        help="PyTorch 장치 (MLP 정책은 cpu 권장). 예: cpu, cuda, auto",
    )
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument(
        "--buffer-size",
        type=int,
        default=1_000_000,
        help="리플레이 버퍼 크기 (메모리 부족 시 500000 등으로 줄이기)",
    )
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument(
        "--learning-starts",
        type=int,
        default=10_000,
        help="리플레이 버퍼에 이 스텝만큼 쌓인 뒤 학습 시작",
    )
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--tau", type=float, default=0.005, help="소프트 타깃 업데이트 계수")
    parser.add_argument(
        "--record-video",
        action="store_true",
        help="학습 중 주기적으로 MuJoCo 시점 mp4 저장 (moviepy 필요)",
    )
    parser.add_argument(
        "--video-folder",
        type=Path,
        default=_ROOT / "videos" / "train",
        help="학습 중 녹화 파일 디렉터리",
    )
    parser.add_argument(
        "--video-freq",
        type=int,
        default=50_000,
        help="몇 환경 스텝마다 새 영상을 시작할지 (VecVideoRecorder)",
    )
    parser.add_argument(
        "--video-length",
        type=int,
        default=400,
        help="영상당 프레임 수(환경 스텝 수)",
    )
    args = parser.parse_args()

    try:
        pmi_track_env_kwargs(args)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 1

    tensorboard_log = _tensorboard_log_dir(args)

    try:
        from stable_baselines3 import SAC
        from stable_baselines3.common.vec_env import DummyVecEnv, VecVideoRecorder
    except ImportError:
        print(
            "stable-baselines3 가 필요합니다: pip install -r requirements.txt",
            file=sys.stderr,
        )
        return 1

    if args.record_video:
        try:
            import moviepy  # noqa: F401
        except ImportError:
            print(
                "`--record-video` 는 moviepy 가 필요합니다:\n"
                "  ./.venv/bin/pip install moviepy\n"
                "  또는 ./.venv/bin/pip install -r requirements.txt",
                file=sys.stderr,
            )
            return 1

    def make_env():
        rm = "rgb_array" if args.record_video else None
        return PMITrackEnv(**pmi_track_env_kwargs(args, render_mode=rm))

    venv = DummyVecEnv([make_env])
    if args.record_video:
        args.video_folder.mkdir(parents=True, exist_ok=True)
        # step_id==0 에서도 트리거되면 무작위 초기 정책 영상이 나오므로 step>0 조건
        venv = VecVideoRecorder(
            venv,
            video_folder=str(args.video_folder.resolve()),
            record_video_trigger=lambda step: step > 0 and step % args.video_freq == 0,
            video_length=args.video_length,
            name_prefix="sac_train",
        )
    model = SAC(
        "MlpPolicy",
        venv,
        learning_rate=args.learning_rate,
        buffer_size=args.buffer_size,
        learning_starts=args.learning_starts,
        batch_size=args.batch_size,
        tau=args.tau,
        gamma=args.gamma,
        train_freq=1,
        gradient_steps=1,
        verbose=1,
        seed=args.seed,
        tensorboard_log=tensorboard_log,
        device=args.device,
    )
    try:
        model.learn(total_timesteps=args.timesteps)
        save_path = args.save.resolve()
        save_path.parent.mkdir(parents=True, exist_ok=True)
        model.save(str(save_path))
        print(f"저장: {save_path}.zip")
    finally:
        # VecVideoRecorder 미저장 경고 방지, MuJoCo Renderer/EGL 정리 순서 개선
        model.env.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
