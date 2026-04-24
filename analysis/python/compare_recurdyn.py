#!/usr/bin/env python3
"""
python_data.csv와 RecurDyn rec_data_*.csv 비교 플롯.

analysis/matlab/plotting.m 과 동일하게 여러 figure(파일)로 나눕니다.
- 2×3: 엔드이펙터 위치·자세, 속도, 가속도 (각 6채널)
- 2×2: 모터/관절 각·각속도·각가속도 (각 4채널)

열 인덱스는 plotting.m 의 index_* (MATLAB 1-기반)을
main.py 의 rec_data = raw[:, 1:] / python_data 저장 순서와 맞춘 0-기반으로 사용합니다.

plotting.m 172–204행은 모터/관절 각가속도 루프에서 index_ddq 와 index_ddq_act 가
서로 뒤바뀐 것으로 보이며, 본 스크립트는 제목 의미에 맞게 바로잡았습니다.

Python과 RecurDyn의 **시간 간격·행 수가 다르면**, 짧게 자르지 않고 참조 CSV의
시간축에 맞춰 Analysis 곡선을 **선형 보간**해 겹친다 (그래프가 중간에서 끊기는
현상 방지).

예:
  python compare_recurdyn.py
  python compare_recurdyn.py --reference ../recurdyn/rec_data_motion.csv \\
      --output-dir ../figures --prefix compare_motion
  python compare_recurdyn.py --no-show       # 창 없이 PNG만 (SSH/배치용)

기본값은 저장 후 **matplotlib 창**을 띄운다. 분할 9개 figure를 모두 만든 뒤
**한 번의** ``plt.show(block=True)`` 로 동시에 띄우고, 전부 닫을 때까지 대기한다.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

try:
    import matplotlib.pyplot as plt
except ImportError as e:
    raise SystemExit(
        "matplotlib가 없습니다. 현재 사용 중인 Python(가상환경)에서 설치하세요:\n"
        "  python -m pip install matplotlib\n"
        "또는: python -m pip install -r requirements.txt\n"
    ) from e


# plotting.m 과 동일 (MATLAB 1-기반 시작 열 → Python rec_data / python_data 의 0-기반)
# rec_data / python_data: 열 0 = 시간, 이후 본문은 plotting.m 의 index_* 와 동일 오프셋.
INDEX_END_POS = 1
INDEX_END_VEL = 7
INDEX_END_ACC = 13
INDEX_Q_ACT = 19
INDEX_DQ_ACT = 23
INDEX_DDQ_ACT = 27
INDEX_Q = 31
INDEX_DQ = 35
INDEX_DDQ = 39


def load_python_output_csv(path: Path) -> np.ndarray:
    rows: list[list[float]] = []
    with path.open(encoding="utf-8") as f:
        for line in f:
            line = line.rstrip().rstrip(",").strip()
            if not line:
                continue
            rows.append([float(x.strip()) for x in line.split(",")])
    return np.asarray(rows, dtype=float)


def load_recurdyn_csv(path: Path, strip_step_column: bool) -> np.ndarray:
    """
    ``np.loadtxt`` 대신 줄 단위로 읽는다. ``main.py`` / C++ ``data_save`` 가
    마지막 열 뒤에 쉼표를 두면 빈 필드가 생겨 loadtxt가 실패할 수 있음.
    ``load_python_output_csv`` 와 동일하게 trailing comma 제거.
    """
    rows: list[list[float]] = []
    with path.open(encoding="utf-8") as f:
        for line in f:
            line = line.rstrip().rstrip(",").strip()
            if not line:
                continue
            rows.append([float(x.strip()) for x in line.split(",")])
    if not rows:
        return np.empty((0, 0), dtype=float)
    raw = np.asarray(rows, dtype=float)
    if strip_step_column:
        return raw[:, 1:]
    return raw


def prepare_plot_series(
    py: np.ndarray, rd: np.ndarray
) -> tuple[np.ndarray, np.ndarray, str]:
    """
    비교용 (py, rd) 쌍을 맞춤.

    행 수만 맞추면 안 됨: Python이 더 잘게 샘플링(예: h=1e-4)이고 RecurDyn이
    h=1e-3이면, 단순히 min(len)으로 자르면 Python은 앞쪽 시간 구간만 남아
    그래프에 Analysis 곡선이 짧게만 보인다.

    시간열이 같지 않거나 길이가 다르면, **참조(rd)의 시간열**에 맞춰
    Python 쪽 각 데이터 열을 `numpy.interp`로 보간한다.
    """
    t_py = py[:, 0]
    t_rd = rd[:, 0]
    atol = 1e-5 * max(1.0, float(np.max(np.abs(t_rd))))
    same_grid = (
        len(py) == len(rd)
        and len(py) > 0
        and np.allclose(t_py, t_rd, rtol=0, atol=atol)
    )
    if same_grid:
        n = min(len(py), len(rd))
        return py[:n], rd[:n], ""

    py_rs = np.empty((len(rd), py.shape[1]), dtype=float)
    py_rs[:, 0] = t_rd
    for j in range(1, py.shape[1]):
        py_rs[:, j] = np.interp(
            t_rd, t_py, py[:, j], left=py[0, j], right=py[-1, j]
        )
    return py_rs, rd, " (Analysis → RecurDyn time, linear interp)"


# 플롯 시 유한값이라도 이 범위를 넘으면 Matplotlib 틱에서 오버플로우가 날 수 있음 (시뮘 폭주 등)
_DISPLAY_ABS_CLIP = 1e12


def _finite_series(y: np.ndarray) -> np.ndarray:
    """플롯용: ``inf``/``nan`` 및 비정상적으로 큰 유한값은 그리지 않는다."""
    y = np.asarray(y, dtype=float)
    out = np.where(np.isfinite(y), y, np.nan)
    out = np.where(np.abs(out) <= _DISPLAY_ABS_CLIP, out, np.nan)
    return out


def _tight_layout_safe(fig: plt.Figure) -> None:
    try:
        fig.tight_layout()
    except (ValueError, RuntimeError):
        fig.subplots_adjust(
            left=0.08, right=0.98, top=0.93, bottom=0.07, hspace=0.35, wspace=0.28
        )


def _ylim_from_finite(ax: plt.Axes, y_rd: np.ndarray, y_py: np.ndarray) -> None:
    """틱 오버플로우 방지: 유한 샘플만으로 y 범위 설정 (전부 nan 이면 기본 구간)."""
    y = np.concatenate(
        [np.asarray(y_rd, dtype=float).ravel(), np.asarray(y_py, dtype=float).ravel()]
    )
    y = y[np.isfinite(y)]
    if y.size == 0:
        ax.set_ylim(-1.0, 1.0)
        return
    lo, hi = float(np.min(y)), float(np.max(y))
    if lo == hi:
        ax.set_ylim(lo - 1.0, hi + 1.0)
    else:
        pad = 0.05 * (hi - lo)
        ax.set_ylim(lo - pad, hi + pad)


def _save_figure(
    fig: plt.Figure, path: Path, dpi: int, *, show: bool = False
) -> None:
    """저장 후 ``show``이면 figure를 닫지 않고 남겨 두어, 호출부에서 한꺼번에 ``plt.show`` 한다."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=dpi, bbox_inches="tight")
    if show:
        return
    plt.close(fig)


def plot_figure_2x3(
    t_py: np.ndarray,
    py: np.ndarray,
    t_rd: np.ndarray,
    rd: np.ndarray,
    index_start: int,
    out_path: Path,
    suptitle: str,
    ylabels: list[str],
    titles: list[str],
    dpi: int,
    legend_subplot_idx: int = 2,
    *,
    show: bool = False,
) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(12, 6), squeeze=False)
    fig.suptitle(suptitle)

    for i in range(6):
        r, c = divmod(i, 3)
        ax = axes[r][c]
        ic = index_start + i
        yr = _finite_series(rd[:, ic])
        yp = _finite_series(py[:, ic])
        ax.plot(t_rd, yr, "b-", lw=1.5, label="RecurDyn")
        ax.plot(t_py, yp, "r--", lw=1.5, label="Analysis")
        _ylim_from_finite(ax, yr, yp)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(ylabels[i])
        ax.set_title(titles[i])
        ax.grid(True, alpha=0.4)
        if i == legend_subplot_idx:
            ax.legend(loc="upper right", fontsize=8)

    _tight_layout_safe(fig)
    _save_figure(fig, out_path, dpi, show=show)


def plot_figure_2x2(
    t_py: np.ndarray,
    py: np.ndarray,
    t_rd: np.ndarray,
    rd: np.ndarray,
    index_start: int,
    out_path: Path,
    suptitle: str,
    ylabels: list[str],
    titles: list[str],
    dpi: int,
    legend_subplot_idx: int = 1,
    *,
    show: bool = False,
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), squeeze=False)
    fig.suptitle(suptitle)

    for i in range(4):
        r, c = divmod(i, 2)
        ax = axes[r][c]
        ic = index_start + i
        yr = _finite_series(rd[:, ic])
        yp = _finite_series(py[:, ic])
        ax.plot(t_rd, yr, "b-", lw=1.5, label="RecurDyn")
        ax.plot(t_py, yp, "r--", lw=1.5, label="Analysis")
        _ylim_from_finite(ax, yr, yp)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(ylabels[i])
        ax.set_title(titles[i])
        ax.grid(True, alpha=0.4)
        if i == legend_subplot_idx:
            ax.legend(loc="upper right", fontsize=8)

    _tight_layout_safe(fig)
    _save_figure(fig, out_path, dpi, show=show)


def max_column_needed() -> int:
    return max(INDEX_END_POS + 5, INDEX_END_VEL + 5, INDEX_END_ACC + 5, INDEX_DDQ + 3)


def run_all_plots(
    py: np.ndarray,
    rd: np.ndarray,
    output_dir: Path,
    file_stem: str,
    dpi: int,
    *,
    show: bool = False,
) -> list[Path]:
    t_py = py[:, 0]
    t_rd = rd[:, 0]
    written: list[Path] = []

    # 1) 엔드 이펙터 위치·자세 (rx,ry,rz, θ1,θ2,θ3)
    ylabels_pos = [
        "r_x (m)",
        "r_y (m)",
        "r_z (m)",
        r"$\theta_1$ (rad)",
        r"$\theta_2$ (rad)",
        r"$\theta_3$ (rad)",
    ]
    titles_pos = [
        "End-Effector Position r_x",
        "End-Effector Position r_y",
        "End-Effector Position r_z",
        r"End-Effector Orientation $\theta_1$",
        r"End-Effector Orientation $\theta_2$",
        r"End-Effector Orientation $\theta_3$",
    ]
    p = output_dir / f"{file_stem}_ee_position.png"
    plot_figure_2x3(
        t_py,
        py,
        t_rd,
        rd,
        INDEX_END_POS,
        p,
        f"End-effector position & orientation",
        ylabels_pos,
        titles_pos,
        dpi,
        legend_subplot_idx=2,
        show=show,
    )
    written.append(p)

    # 2) 엔드 이펙터 속도
    ylabels_vel = [
        "v_x (m/s)",
        "v_y (m/s)",
        "v_z (m/s)",
        r"$\omega_x$ (rad/s)",
        r"$\omega_y$ (rad/s)",
        r"$\omega_z$ (rad/s)",
    ]
    titles_vel = [
        "End-Effector Linear Velocity v_x",
        "End-Effector Linear Velocity v_y",
        "End-Effector Linear Velocity v_z",
        r"End-Effector Angular Velocity $\omega_x$",
        r"End-Effector Angular Velocity $\omega_y$",
        r"End-Effector Angular Velocity $\omega_z$",
    ]
    p = output_dir / f"{file_stem}_ee_velocity.png"
    plot_figure_2x3(
        t_py,
        py,
        t_rd,
        rd,
        INDEX_END_VEL,
        p,
        f"End-effector velocity",
        ylabels_vel,
        titles_vel,
        dpi,
        legend_subplot_idx=2,
        show=show,
    )
    written.append(p)

    # # 3) 엔드 이펙터 가속도
    # ylabels_acc = [
    #     "a_x (m/s²)",
    #     "a_y (m/s²)",
    #     "a_z (m/s²)",
    #     r"$\alpha_x$ (rad/s²)",
    #     r"$\alpha_y$ (rad/s²)",
    #     r"$\alpha_z$ (rad/s²)",
    # ]
    # titles_acc = [
    #     "End-Effector Linear Acceleration a_x",
    #     "End-Effector Linear Acceleration a_y",
    #     "End-Effector Linear Acceleration a_z",
    #     r"End-Effector Angular Acceleration $\alpha_x$",
    #     r"End-Effector Angular Acceleration $\alpha_y$",
    #     r"End-Effector Angular Acceleration $\alpha_z$",
    # ]
    # p = output_dir / f"{file_stem}_ee_acceleration.png"
    # plot_figure_2x3(
    #     t_py,
    #     py,
    #     t_rd,
    #     rd,
    #     INDEX_END_ACC,
    #     p,
    #     f"{figure_title} — End-effector acceleration",
    #     ylabels_acc,
    #     titles_acc,
    #     dpi,
    #     legend_subplot_idx=2,
    #     show=show,
    # )
    # written.append(p)

    # 4) 모터 각도 q_act
    p = output_dir / f"{file_stem}_motor_angle.png"
    plot_figure_2x2(
        t_py,
        py,
        t_rd,
        rd,
        INDEX_Q_ACT,
        p,
        f"Motor angle",
        [f"q_{i} (rad)" for i in range(1, 5)],
        [f"Motor Angle q_{i}" for i in range(1, 5)],
        dpi,
        legend_subplot_idx=1,
        show=show,
    )
    written.append(p)

    # 5) 관절 각도 q
    p = output_dir / f"{file_stem}_joint_angle.png"
    plot_figure_2x2(
        t_py,
        py,
        t_rd,
        rd,
        INDEX_Q,
        p,
        f"Joint angle",
        [f"q_{i} (rad)" for i in range(1, 5)],
        [f"Joint Angle q_{i}" for i in range(1, 5)],
        dpi,
        legend_subplot_idx=1,
        show=show,
    )
    written.append(p)

    # 6) 모터 각속도 dq_act
    p = output_dir / f"{file_stem}_motor_velocity.png"
    plot_figure_2x2(
        t_py,
        py,
        t_rd,
        rd,
        INDEX_DQ_ACT,
        p,
        f"Motor angle velocity",
        [f"dq_{i} act (rad/s)" for i in range(1, 5)],
        [f"Motor Angle Velocity dq_{i}_act" for i in range(1, 5)],
        dpi,
        legend_subplot_idx=1,
        show=show,
    )
    written.append(p)

    # 7) 관절 각속도 dq
    p = output_dir / f"{file_stem}_joint_velocity.png"
    plot_figure_2x2(
        t_py,
        py,
        t_rd,
        rd,
        INDEX_DQ,
        p,
        f"Joint angle velocity",
        [f"dq_{i} (rad/s)" for i in range(1, 5)],
        [f"Joint Angle Velocity dq_{i}" for i in range(1, 5)],
        dpi,
        legend_subplot_idx=1,
        show=show,
    )
    written.append(p)

    # # 8) 모터 각가속도 ddq_act (plotting.m 172행 루프의 index 오류 보정)
    # p = output_dir / f"{file_stem}_motor_acceleration.png"
    # plot_figure_2x2(
    #     t_py,
    #     py,
    #     t_rd,
    #     rd,
    #     INDEX_DDQ_ACT,
    #     p,
    #     f"{figure_title} — Motor angle acceleration",
    #     [f"ddq_{i} act (rad/s²)" for i in range(1, 5)],
    #     [f"Motor Angle Acceleration ddq_{i}_act" for i in range(1, 5)],
    #     dpi,
    #     legend_subplot_idx=1,
    #     show=show,
    # )
    # written.append(p)

    # # 9) 관절 각가속도 ddq
    # p = output_dir / f"{file_stem}_joint_acceleration.png"
    # plot_figure_2x2(
    #     t_py,
    #     py,
    #     t_rd,
    #     rd,
    #     INDEX_DDQ,
    #     p,
    #     f"{figure_title} — Joint angle acceleration",
    #     [f"ddq_{i} (rad/s²)" for i in range(1, 5)],
    #     [f"Joint Angle Acceleration ddq_{i}" for i in range(1, 5)],
    #     dpi,
    #     legend_subplot_idx=1,
    #     show=show,
    # )
    # written.append(p)

    return written


def main() -> None:
    here = Path(__file__).resolve().parent
    default_py = here / "python_data.csv"
    default_ref = here / "../recurdyn/rec_data_free_fall.csv"

    parser = argparse.ArgumentParser(
        description="analysis_data vs rec_data"
    )
    parser.add_argument(
        "--analysis",
        dest="python_csv",
        type=Path,
        default=default_py,
        help=f"시뮬 결과 CSV (기본: {default_py})",
    )
    parser.add_argument(
        "--reference",
        type=Path,
        default=default_ref,
        help=f"RecurDyn 참조 CSV (기본: {default_ref})",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=here,
        help="PNG 저장 디렉터리 (기본: 스크립트와 같은 폴더)",
    )
    parser.add_argument(
        "--prefix",
        type=str,
        default="compare_recurdyn",
        help="파일 이름 접두사 (기본: compare_recurdyn)",
    )
    parser.add_argument(
        "--keep-step-column",
        action="store_true",
        help=(
            "참조 CSV의 첫 열을 유지한다(기본은 제거). "
            "RecurDyn보내기처럼 첫 열이 스텝/인덱스이고 둘째 열부터가 시간·데이터일 때는 플래그 없음. "
            "python_data.csv처럼 첫 열이 곧 시간이면 이 플래그를 줘야 열 개수가 맞는다."
        ),
    )
    parser.add_argument("--dpi", type=int, default=150)
    parser.add_argument(
        "--no-show",
        action="store_false",
        dest="show",
        help="matplotlib 창 표시 생략 (PNG만 저장)",
    )
    parser.set_defaults(show=True)
    args = parser.parse_args()

    py_raw = load_python_output_csv(args.python_csv)
    rd = load_recurdyn_csv(args.reference, strip_step_column=not args.keep_step_column)

    need = max_column_needed()
    if py_raw.shape[1] <= need or rd.shape[1] <= need:
        raise SystemExit(
            f"열 개수 부족: python {py_raw.shape}, reference {rd.shape}. "
            f"최소 {need + 1}열이 필요합니다 (0-기반 열 {need}까지)."
        )

    py, rd, align_note = prepare_plot_series(py_raw, rd)
    paths = run_all_plots(
        py,
        rd,
        args.output_dir,
        args.prefix,
        args.dpi,
        show=args.show,
    )
    for p in paths:
        print(f"저장: {p}")

    if args.show:
        plt.show(block=True)
        plt.close("all")


if __name__ == "__main__":
    main()
