#!/usr/bin/env python3
"""시뮬레이션을 오프스크린 렌더링 후 MP4로 저장 (창 없이 동작).

예::

  PYTHONPATH=. python scripts/record_video.py -o outputs/pmi.mp4

작은 무작위 제어::

  PYTHONPATH=. python scripts/record_video.py -o outputs/pmi.mp4 --noise 0.05

시스템에 ffmpeg가 없으면 ``pip install imageio-ffmpeg`` 로 번들 ffmpeg 사용.

시각적으로 위아래가 뒤집히면 ``recording.py`` 의 ``np.flipud`` 를 조정하세요.

출력이 한꺼번에만 보이면 ``python -u scripts/record_video.py`` 또는
``PYTHONUNBUFFERED=1 python ...`` 사용. 진행 막대가 안 보이면 ``--progress-newlines``.

X11 DISPLAY(VcXsrv 등)가 깨져 ``Failed to open display`` 가 나오면, 이 스크립트는
기본으로 ``MUJOCO_GL=egl``(오프스크린)을 쓰므로 창 없이 녹화됩니다.
GLFW를 쓰려면 ``--glfw`` 또는 ``MUJOCO_GL=glfw`` (DISPLAY 필요).
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

# mujoco 첫 import 전에 적용돼야 함 → 모듈 로드 시 기본값만 설정
if not os.environ.get("MUJOCO_GL", "").strip():
    os.environ["MUJOCO_GL"] = "egl"

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))


def main() -> None:
    ap = argparse.ArgumentParser(description="MuJoCo 오프스크린 녹화 → MP4")
    ap.add_argument("-o", "--output", default="outputs/pmi_rollout.mp4", help="저장 경로")
    ap.add_argument("--steps", type=int, default=400)
    ap.add_argument("--frame-skip", type=int, default=2)
    ap.add_argument("--fps", type=float, default=30.0, help="동영상 재생 FPS")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--noise", type=float, default=0.0, help=">0 이면 균일 무작위 제어 크기")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--collision", action="store_true")
    ap.add_argument(
        "--no-progress",
        action="store_true",
        help="터미널 진행 표시 끄기",
    )
    ap.add_argument(
        "--progress-every",
        type=int,
        default=None,
        metavar="N",
        help="N 스텝마다 진행 줄 갱신 (기본: 총 스텝의 약 1/40). 첫 스텝 후에도 항상 1회 출력",
    )
    ap.add_argument(
        "--progress-newlines",
        action="store_true",
        help="진행을 한 줄 덮어쓰기(\\r) 대신 줄마다 출력 (터미널에 진행이 안 보일 때)",
    )
    ap.add_argument(
        "--glfw",
        action="store_true",
        help="OpenGL을 GLFW+X11로 (유효한 DISPLAY 필요). 기본은 EGL 오프스크린",
    )
    args = ap.parse_args()

    if args.glfw:
        os.environ["MUJOCO_GL"] = "glfw"

    print("record_video: 패키지 로드 중…", flush=True)
    from pmi_mujoco_rl.model import LoadOptions
    from pmi_mujoco_rl.recording import random_ctrl_fn, record_rollout_mp4

    opts = LoadOptions(disable_collision=not args.collision)

    ctrl_fn = None
    if args.noise > 0:
        ctrl_fn = random_ctrl_fn(scale=args.noise, seed=args.seed)

    try:
        path = record_rollout_mp4(
            args.output,
            load_options=opts,
            steps=args.steps,
            frame_skip=args.frame_skip,
            video_fps=args.fps,
            height=args.height,
            width=args.width,
            ctrl_fn=ctrl_fn,
            show_progress=not args.no_progress,
            progress_every=args.progress_every,
            progress_newlines=args.progress_newlines,
        )
    except KeyboardInterrupt:
        # 부분 저장 메시지는 record_rollout_mp4 에서 이미 출력
        raise SystemExit(130)

    print(f"저장 완료: {path}")


if __name__ == "__main__":
    main()
