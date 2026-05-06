#!/usr/bin/env python3
"""
`analysis/python/main.py` 의 ``run_ik`` 가 저장하는 CSV(``data_save`` 형식)에서
관절 각(q1..q4)을 읽어 MuJoCo 모델에 적용해 가시화한다.

기본 CSV: ``analysis/python/python_data_path.csv`` (run_ik 실행 후 생성)
관절 열: `data_save` 기준 generalized coordinate ``qi`` → 컬럼 인덱스 31~34 (0-based).

사용 예:
  ../.venv/bin/python play_trajectory.py
  ../.venv/bin/python play_trajectory.py --csv ../analysis/python/python_data_vsd.csv
  ../.venv/bin/python play_trajectory.py --csv ../analysis/python/python_data_vsd.csv --record traj.mp4
  ../.venv/bin/python play_trajectory.py --csv ../analysis/python/python_data_vsd.csv --record traj.mp4 --no-viewer
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

# data_save: t_c, re(3), rpy(3), dre(3), wi(3), (중복 블록), qi_act(4), ...
COL_QI_START = 31
COL_QI_END = 35
COL_QI_ACT_START = 19
COL_QI_ACT_END = 23

_PKG_ROOT = Path(__file__).resolve().parent
_DEFAULT_CSV = (_PKG_ROOT.parent / "analysis/python/python_data_path.csv").resolve()
_MESH_DIR = (_PKG_ROOT.parent / "ros_ws/pmi_description/meshes").resolve()


def _parse_csv_row(line: str) -> list[float]:
    parts = [p.strip() for p in line.split(",") if p.strip() != ""]
    return [float(x) for x in parts]


def load_trajectory(
    csv_path: Path,
    use_actuator_coords: bool,
) -> tuple[np.ndarray, np.ndarray]:
    """시간 벡터 t와 (N,4) 관절 각 배열을 반환한다."""
    times: list[float] = []
    qs: list[list[float]] = []
    with csv_path.open(encoding="utf-8") as fp:
        for raw in fp:
            raw = raw.strip()
            if not raw:
                continue
            try:
                row = _parse_csv_row(raw)
            except ValueError:
                continue
            q_start = COL_QI_ACT_START if use_actuator_coords else COL_QI_START
            q_end = COL_QI_ACT_END if use_actuator_coords else COL_QI_END
            if len(row) < q_end:
                raise ValueError(
                    f"열 개수 부족: {len(row)} < {q_end} — 파일이 data_save 형식인지 확인하세요."
                )
            times.append(float(row[0]))
            qs.append([row[q_start + k] for k in range(4)])
    if not times:
        raise ValueError(f"유효한 데이터 행이 없습니다: {csv_path}")
    return np.asarray(times, dtype=float), np.asarray(qs, dtype=float)


def pick_model_path(explicit: Path | None) -> Path:
    if explicit is not None:
        return explicit.resolve()
    mesh_xml = _PKG_ROOT / "models/pmi_arm_mesh.xml"
    prim_xml = _PKG_ROOT / "models/pmi_arm_primitive.xml"
    stl = _MESH_DIR / "base_link.STL"
    if stl.is_file():
        return mesh_xml.resolve()
    return prim_xml.resolve()


def record_trajectory_mp4(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    t_series: np.ndarray,
    q_series: np.ndarray,
    out_path: Path,
    *,
    speed: float,
    fps_max: float,
    width: int,
    height: int,
) -> None:
    """오프스크린 렌더로 궤적 전체를 MP4로 저장 (인터랙티브 뷰어 카메라와는 다를 수 있음)."""
    try:
        import imageio.v2 as imageio
    except ImportError as exc:
        raise ImportError(
            "녹화하려면: pip install imageio imageio-ffmpeg"
        ) from exc

    n = q_series.shape[0]
    if n < 1:
        raise ValueError("녹화할 샘플이 없습니다.")

    if len(t_series) > 1:
        dt_med = float(np.median(np.diff(t_series))) / float(speed)
    else:
        dt_med = 0.001
    if not np.isfinite(dt_med) or dt_med <= 0:
        dt_med = 0.001
    fps = min(max(1.0 / dt_med, 1.0), fps_max)

    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, cam)
        cam.lookat[:] = [0.0, 0.0, 0.15]
        cam.distance = 2.2
        cam.elevation = -20.0
        cam.azimuth = 135.0

        frames: list[np.ndarray] = []
        for i in range(n):
            data.qpos[:] = q_series[i]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=cam)
            rgb = renderer.render()
            frames.append(np.asarray(rgb, dtype=np.uint8))
    finally:
        renderer.close()
    out_path = out_path.resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(
        str(out_path),
        frames,
        fps=fps,
        codec="libx264",
        quality=8,
        pixelformat="yuv420p",
    )
    print(f"녹화 저장: {out_path} ({n} 프레임, fps≈{fps:.2f})")


def main() -> int:
    parser = argparse.ArgumentParser(description="PMI arm MuJoCo trajectory viewer")
    parser.add_argument(
        "--csv",
        type=Path,
        default=_DEFAULT_CSV,
        help=f"궤적 CSV 경로 (기본: {_DEFAULT_CSV})",
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=None,
        help="MJCF 경로 (미지정 시 STL 있으면 mesh, 없으면 primitive)",
    )
    parser.add_argument(
        "--use-actuator-q",
        action="store_true",
        help="qi 대신 qi_act(모터측) 열(19~22) 사용 — URDF 관절과 안 맞을 수 있음",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="CSV 시간 대비 재생 배속 (1.0 = CSV dt 그대로)",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="끝까지 재생 후 처음부터 반복",
    )
    parser.add_argument(
        "--record",
        type=Path,
        default=None,
        help="종료 후 동일 궤적을 오프스크린 렌더하여 MP4로 저장 (예: traj.mp4)",
    )
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="인터랙티브 뷰어를 띄우지 않음 (--record 와 함께 헤드리스 녹화)",
    )
    parser.add_argument(
        "--record-fps-max",
        type=float,
        default=60.0,
        help="녹화 FPS 상한 (CSV dt 로부터 계산한 fps 가 이 값으로 캡)",
    )
    parser.add_argument(
        "--record-width",
        type=int,
        default=640,
        help="기본 640 (MuJoCo 오프스크린 버퍼 한계; 더 크게 하려면 MJCF <visual><global offwidth/...)",
    )
    parser.add_argument("--record-height", type=int, default=480)
    args = parser.parse_args()

    csv_path = args.csv.resolve()
    if not csv_path.is_file():
        print(
            f"CSV 없음: {csv_path}\n"
            "먼저 analysis/python 에서 run_ik 등으로 python_data_path.csv 를 생성하세요.",
            file=sys.stderr,
        )
        return 1

    model_path = pick_model_path(args.model)
    if not model_path.is_file():
        print(f"모델 파일 없음: {model_path}", file=sys.stderr)
        return 1

    t_series, q_series = load_trajectory(csv_path, args.use_actuator_q)
    n = len(t_series)
    if model_path.name == "pmi_arm_mesh.xml" and not (_MESH_DIR / "base_link.STL").is_file():
        print(
            f"경고: mesh 모델을 선택했지만 STL 이 없습니다 ({_MESH_DIR}).\n"
            "  ros 패키지 빌드 후 meshes 를 채우거나 --model models/pmi_arm_primitive.xml 사용.",
            file=sys.stderr,
        )

    try:
        model = mujoco.MjModel.from_xml_path(str(model_path))
    except Exception as exc:
        print(f"모델 로드 실패 ({model_path}): {exc}", file=sys.stderr)
        print("primitive 모델로 재시도: models/pmi_arm_primitive.xml", file=sys.stderr)
        fallback = _PKG_ROOT / "models/pmi_arm_primitive.xml"
        model = mujoco.MjModel.from_xml_path(str(fallback))

    data = mujoco.MjData(model)
    if model.nq != 4:
        print(f"예상 nq=4 인데 nq={model.nq} — 관절 정의를 확인하세요.", file=sys.stderr)
        return 1

    dt_nominal = float(np.median(np.diff(t_series))) if n > 1 else 0.001
    if not np.isfinite(dt_nominal) or dt_nominal <= 0:
        dt_nominal = 0.001

    print(f"모델: {model_path}")
    print(f"CSV: {csv_path} ({n} 샘플)")
    print(f"관절 열: {'qi_act 19-22' if args.use_actuator_q else 'qi 31-34'}")
    print(f"중간 dt ≈ {dt_nominal:.6g} s, 배속 {args.speed}")

    if not args.no_viewer:
        idx = 0
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                data.qpos[:] = q_series[idx]
                mujoco.mj_forward(model, data)
                viewer.sync()

                if idx < n - 1:
                    dt = (t_series[idx + 1] - t_series[idx]) / args.speed
                    time.sleep(max(float(dt), 1e-6))
                    idx += 1
                else:
                    if args.loop:
                        idx = 0
                    else:
                        time.sleep(0.02)

    if args.record is not None:
        try:
            record_trajectory_mp4(
                model,
                data,
                t_series,
                q_series,
                args.record,
                speed=args.speed,
                fps_max=args.record_fps_max,
                width=args.record_width,
                height=args.record_height,
            )
        except ImportError as exc:
            print(exc, file=sys.stderr)
            return 1
        except Exception as exc:
            print(f"녹화 실패: {exc}", file=sys.stderr)
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
