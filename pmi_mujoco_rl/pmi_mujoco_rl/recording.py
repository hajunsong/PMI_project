"""오프스크린 렌더링으로 시뮬레이션을 동영상으로 저장."""

from __future__ import annotations

import sys
from collections.abc import Callable
from pathlib import Path

import mujoco
import numpy as np
from mujoco import Renderer
from numpy.typing import NDArray

from pmi_mujoco_rl.model import LoadOptions, ctrl_indices_for_actuators, load_pmi_model


def _configure_progress_streams() -> None:
    """Cursor/파이프 환경에서도 줄이 바로 보이게 stderr/stdout 줄 버퍼링 시도."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(line_buffering=True)
        except (AttributeError, OSError, ValueError):
            pass


def _clamp_resolution(
    model: mujoco.MjModel, height: int, width: int
) -> tuple[int, int]:
    max_w = model.vis.global_.offwidth
    max_h = model.vis.global_.offheight
    w = min(width, max_w)
    h = min(height, max_h)
    if w != width or h != height:
        print(
            f"해상도를 프레임버퍼 한도에 맞춤: 요청 {width}x{height} → {w}x{h} "
            f"(모델 visual/global offwidth/offheight 상한)"
        )
    return h, w


def _format_progress_line(
    step_done: int,
    steps_total: int,
    num_frames: int,
    bar_width: int = 32,
) -> str:
    """한 줄 진행 표시(\\r 로 같은 줄 갱신)."""
    if steps_total <= 0:
        return ""
    pct = 100.0 * step_done / steps_total
    filled = int(bar_width * step_done / steps_total)
    filled = min(filled, bar_width)
    bar = "#" * filled + "-" * (bar_width - filled)
    return (
        f"\r시뮬 [{bar}] {step_done}/{steps_total} ({pct:5.1f}%) "
        f"| 캡처 프레임 {num_frames}"
    )


def _write_mp4(
    output_path: Path,
    frames: list[np.ndarray],
    video_fps: float,
) -> None:
    import imageio.v2 as imageio

    imageio.mimsave(
        output_path,
        frames,
        fps=video_fps,
        codec="libx264",
        pixelformat="yuv420p",
    )


def record_rollout_mp4(
    output_path: str | Path,
    *,
    load_options: LoadOptions | None = None,
    steps: int = 400,
    frame_skip: int = 2,
    video_fps: float = 30.0,
    height: int = 480,
    width: int = 640,
    ctrl_fn: Callable[[int, mujoco.MjData], NDArray[np.float64]] | None = None,
    show_progress: bool = True,
    progress_every: int | None = None,
    progress_newlines: bool = False,
) -> Path:
    """시뮬을 돌리며 프레임을 모아 MP4로 저장.

    GUI 없이 ``mujoco.Renderer``만 사용하므로 WSL·헤드리스에서도 동작하기 쉽습니다.

    Parameters
    ----------
    ctrl_fn
        ``(step_index, data) -> shape (4,)`` 제어 입력. None이면 매 스텝 0.
    frame_skip
        시뮬 스텝당 렌더 간격. 2면 두 번에 한 번만 프레임 저장.
    video_fps
        재생 시 초당 프레임 수 (파일 메타데이터).

    Ctrl+C (KeyboardInterrupt) 로 중단하면 **그 시점까지 모은 프레임만** 같은 경로에
    저장한 뒤 예외를 다시 던집니다 (종료 코드 130). 프레임이 하나도 없으면 파일을 만들지 않습니다.

    show_progress
        ``True`` 이면 stderr 에 같은 줄을 갱신하는 진행 표시를 출력합니다.
    progress_every
        몇 시뮬 스텝마다 줄을 갱신할지. ``None`` 이면 ``max(1, steps // 40)``.
        **첫 번째 스텝 직후에는 항상** 한 번 진행을 출력합니다.
    progress_newlines
        ``True`` 이면 ``\\r`` 한 줄 덮어쓰기 대신 매번 줄바꿈으로 출력 (터미널이 진행이
        안 보일 때).
    """
    if show_progress:
        _configure_progress_streams()
        print("PMI 모델 컴파일 중 (URDF·STL·동등 제약)…", file=sys.stderr, flush=True)

    opts = load_options or LoadOptions()
    model = load_pmi_model(opts)
    if show_progress:
        print(
            f"모델 준비 완료: nq={model.nq} nv={model.nv} nu={model.nu}",
            file=sys.stderr,
            flush=True,
        )
    data = mujoco.MjData(model)
    ctrl_idx = ctrl_indices_for_actuators(model)

    h, w = _clamp_resolution(model, height, width)
    renderer = Renderer(model, height=h, width=w)

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    mujoco.mj_forward(model, data)

    frames: list[np.ndarray] = []
    interrupted = False
    tick = progress_every if progress_every is not None else max(1, steps // 40)

    try:
        try:
            if show_progress and steps > 0:
                print(
                    f"시뮬 시작 (총 {steps} 스텝, frame_skip={frame_skip})",
                    file=sys.stderr,
                    flush=True,
                )
            for t in range(steps):
                if ctrl_fn is not None:
                    u = np.asarray(ctrl_fn(t, data), dtype=np.float64).reshape(-1)
                    if u.size != ctrl_idx.size:
                        raise ValueError(
                            f"ctrl_fn must return length {ctrl_idx.size}, got {u.size}"
                        )
                    data.ctrl[ctrl_idx] = u
                else:
                    data.ctrl[ctrl_idx] = 0.0

                mujoco.mj_step(model, data)

                if t % frame_skip == 0:
                    renderer.update_scene(data, camera=-1)
                    rgb = renderer.render()
                    frames.append(np.flipud(rgb).copy())

                if show_progress and steps > 0:
                    done = t + 1
                    first_step = done == 1
                    periodic = done % tick == 0
                    last_step = done == steps
                    if first_step or periodic or last_step:
                        line = _format_progress_line(done, steps, len(frames))
                        if progress_newlines:
                            print(line.lstrip("\r"), file=sys.stderr, flush=True)
                        else:
                            print(line, end="", file=sys.stderr, flush=True)
        except KeyboardInterrupt:
            interrupted = True
            if show_progress:
                print(file=sys.stderr, flush=True)
            print(
                "Ctrl+C — 지금까지 캡처한 프레임을 저장합니다.",
                file=sys.stderr,
                flush=True,
            )
    finally:
        renderer.close()

    if show_progress and steps > 0 and not interrupted:
        print(file=sys.stderr, flush=True)

    if not frames:
        if interrupted:
            print("저장할 프레임이 없어 파일을 만들지 않습니다.", file=sys.stderr)
            raise KeyboardInterrupt from None
        raise RuntimeError("저장할 프레임이 없습니다 (steps/frame_skip 확인).")

    if show_progress:
        print(
            f"MP4 인코딩 중… ({len(frames)}프레임, {video_fps}fps)",
            file=sys.stderr,
            flush=True,
        )
    _write_mp4(output_path, frames, video_fps)
    if show_progress:
        print("인코딩 완료.", file=sys.stderr, flush=True)
    if interrupted:
        print(
            f"부분 저장 완료 ({len(frames)}프레임): {output_path.resolve()}",
            file=sys.stderr,
            flush=True,
        )
        raise KeyboardInterrupt from None

    return output_path.resolve()


def random_ctrl_fn(
    scale: float = 0.05,
    seed: int = 0,
) -> Callable[[int, mujoco.MjData], NDArray[np.float64]]:
    rng = np.random.default_rng(seed)

    def _fn(_t: int, _data: mujoco.MjData) -> NDArray[np.float64]:
        return rng.uniform(-scale, scale, size=4)

    return _fn
