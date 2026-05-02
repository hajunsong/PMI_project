#!/usr/bin/env python3
"""
CSV(`data_save`)의 EE 위치 ``re``(열 1~3)와 MuJoCo 사이트 ``ee``의 월드 좌표를 비교한다.

관절각은 CSV의 ``qi``(열 31~34)를 모델에 넣고 ``mj_forward`` 후 ``site_xpos`` 로 EE를 계산한다.

예:
  ./.venv/bin/python compare_ee_trajectory.py --csv ../analysis/python/python_data_path.csv
  ./.venv/bin/python compare_ee_trajectory.py --csv runA.csv --csv-q-b runB.csv --save ee_compare.png
  # 저장: ee_compare.png, ee_compare_error.png, ee_compare_joints.png, ee_compare_joints_diff.png
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np

# data_save 열 (0-based): t, re(1..3), rpy(3), ...
COL_RE_START = 1
COL_RE_END = 4
COL_QI_START = 31
COL_QI_END = 35
COL_QI_ACT_START = 19
COL_QI_ACT_END = 23

_PKG_ROOT = Path(__file__).resolve().parent
_DEFAULT_CSV = (_PKG_ROOT.parent / "analysis/python/python_data_path.csv").resolve()
_MESH_DIR = (_PKG_ROOT.parent / "ros_ws/pmi_description/meshes").resolve()

_JOINT_NAMES = ("q1", "q2", "q3", "q4")


def _wrap_angle_diff(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """각도 차이를 (-pi, pi] 로 줄임. a,b 와 동일 shape."""
    d = a - b
    return np.arctan2(np.sin(d), np.cos(d))


def _parse_csv_row(line: str) -> list[float]:
    parts = [p.strip() for p in line.split(",") if p.strip() != ""]
    return [float(x) for x in parts]


def load_csv_trajectory(
    csv_path: Path,
    use_actuator_q: bool,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """t (N,), re_csv (N,3), q (N,4)."""
    times: list[float] = []
    re_list: list[list[float]] = []
    qs: list[list[float]] = []
    q_start = COL_QI_ACT_START if use_actuator_q else COL_QI_START
    q_end = COL_QI_ACT_END if use_actuator_q else COL_QI_END

    with csv_path.open(encoding="utf-8") as fp:
        for raw in fp:
            raw = raw.strip()
            if not raw:
                continue
            try:
                row = _parse_csv_row(raw)
            except ValueError:
                continue
            if len(row) < max(COL_RE_END, q_end):
                raise ValueError(
                    f"열 개수 부족: {len(row)} — data_save 형식인지 확인하세요."
                )
            times.append(float(row[0]))
            re_list.append(
                [float(row[COL_RE_START]), float(row[COL_RE_START + 1]), float(row[COL_RE_START + 2])]
            )
            qs.append([float(row[q_start + k]) for k in range(4)])

    if not times:
        raise ValueError(f"유효한 데이터 행이 없습니다: {csv_path}")
    return (
        np.asarray(times, dtype=float),
        np.asarray(re_list, dtype=float),
        np.asarray(qs, dtype=float),
    )


def pick_model_path(explicit: Path | None) -> Path:
    if explicit is not None:
        return explicit.resolve()
    mesh_xml = _PKG_ROOT / "models/pmi_arm_mesh.xml"
    prim_xml = _PKG_ROOT / "models/pmi_arm_primitive.xml"
    stl = _MESH_DIR / "base_link.STL"
    if stl.is_file():
        return mesh_xml.resolve()
    return prim_xml.resolve()


def mujoco_ee_positions(
    model_path: Path,
    qs: np.ndarray,
    site_name: str = "ee",
) -> np.ndarray:
    """각 행 관절각에 대해 사이트 월드 위치 (N,3)."""
    try:
        model = mujoco.MjModel.from_xml_path(str(model_path))
    except Exception:
        fallback = _PKG_ROOT / "models/pmi_arm_primitive.xml"
        model = mujoco.MjModel.from_xml_path(str(fallback))

    data = mujoco.MjData(model)
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    if sid < 0:
        raise RuntimeError(f'MJCF에 site name="{site_name}" 가 없습니다.')

    if model.nq != qs.shape[1]:
        raise ValueError(f"모델 nq={model.nq}, CSV 관절 수={qs.shape[1]}")

    n = qs.shape[0]
    out = np.zeros((n, 3), dtype=float)
    for i in range(n):
        data.qpos[:] = qs[i]
        mujoco.mj_forward(model, data)
        out[i] = data.site_xpos[sid].copy()
    return out


def mujoco_qpos_readback(model_path: Path, qs: np.ndarray) -> np.ndarray:
    """각 행에서 ``qpos <- qs[i]`` 후 ``mj_forward`` 한 뒤 ``qpos`` 를 다시 읽는다 (재생 일치 확인용)."""
    try:
        model = mujoco.MjModel.from_xml_path(str(model_path))
    except Exception:
        fallback = _PKG_ROOT / "models/pmi_arm_primitive.xml"
        model = mujoco.MjModel.from_xml_path(str(fallback))

    data = mujoco.MjData(model)
    if model.nq != qs.shape[1]:
        raise ValueError(f"모델 nq={model.nq}, CSV 관절 수={qs.shape[1]}")

    n = qs.shape[0]
    out = np.zeros((n, model.nq), dtype=float)
    for i in range(n):
        data.qpos[:] = qs[i]
        mujoco.mj_forward(model, data)
        out[i] = data.qpos.copy()
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="CSV EE vs MuJoCo site ee 비교 플롯")
    parser.add_argument("--csv", type=Path, default=_DEFAULT_CSV, help="data_save 형식 CSV")
    parser.add_argument("--model", type=Path, default=None, help="MJCF 경로")
    parser.add_argument(
        "--use-actuator-q",
        action="store_true",
        help="관절열로 qi_act(19~22) 사용",
    )
    parser.add_argument("--site", type=str, default="ee", help="MuJoCo site 이름")
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="그림 저장 경로 (png 등). 미지정이면 plt.show()",
    )
    parser.add_argument("--dpi", type=int, default=150)
    parser.add_argument(
        "--csv-q-b",
        type=Path,
        default=None,
        help=(
            "관절각 비교용 두 번째 CSV (동일 data_save 형식, 행 수 동일). "
            "예: 해석 적분 vs MuJoCo 적분으로 각각 저장한 python_data_vsd.csv 두 개."
        ),
    )
    args = parser.parse_args()

    csv_path = args.csv.resolve()
    if not csv_path.is_file():
        print(f"CSV 없음: {csv_path}", file=sys.stderr)
        return 1

    model_path = pick_model_path(args.model)

    t, re_csv, qs = load_csv_trajectory(csv_path, args.use_actuator_q)
    try:
        re_mj = mujoco_ee_positions(model_path, qs, args.site)
    except Exception as exc:
        print(f"MuJoCo EE 계산 실패: {exc}", file=sys.stderr)
        return 1

    err = re_mj - re_csv
    err_norm = np.linalg.norm(err, axis=1)

    labels = ("x", "y", "z")
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("EE position: CSV (analysis re) vs MuJoCo (site)")

    for k in range(3):
        ax = axes[k]
        ax.plot(t, re_csv[:, k], label="CSV re", linewidth=1.2)
        ax.plot(t, re_mj[:, k], label="MuJoCo site", linewidth=1.0, alpha=0.85)
        ax.set_ylabel(f"{labels[k]} [m]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("time [s]")

    fig2, ax_e = plt.subplots(figsize=(10, 3.5))
    ax_e.plot(t, err[:, 0], label=r"$\Delta x$", linewidth=1.0)
    ax_e.plot(t, err[:, 1], label=r"$\Delta y$", linewidth=1.0)
    ax_e.plot(t, err[:, 2], label=r"$\Delta z$", linewidth=1.0)
    ax_e.plot(t, err_norm, "k--", label=r"$\|\Delta\|$", linewidth=1.2)
    ax_e.set_xlabel("time [s]")
    ax_e.set_ylabel("error [m]")
    ax_e.set_title("MuJoCo − CSV")
    ax_e.grid(True, alpha=0.3)
    ax_e.legend(ncol=4, fontsize=8)

    rmse = float(np.sqrt(np.mean(err**2)))
    stats = (
        f"RMSE (xyz components): {rmse:.6g} m\n"
        f"max ||Δ||: {np.max(err_norm):.6g} m\n"
        f"mean ||Δ||: {np.mean(err_norm):.6g} m"
    )
    print(stats)

    # --- Joint angle figures ---
    fig_q, axes_q = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    fig_q.suptitle("Joint angles q")

    fig_dq, axes_dq = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    fig_dq.suptitle("Joint angle difference (wrapped to [-pi, pi])")

    if args.csv_q_b is not None:
        path_b = args.csv_q_b.resolve()
        if not path_b.is_file():
            print(f"--csv-q-b 없음: {path_b}", file=sys.stderr)
            return 1
        t_b, _, qs_b = load_csv_trajectory(path_b, args.use_actuator_q)
        if len(t) != len(t_b) or not np.allclose(t, t_b, rtol=0, atol=1e-9):
            print(
                "경고: 두 CSV의 행 수 또는 시간 열이 다릅니다. 동일 샘플링인지 확인하세요.",
                file=sys.stderr,
            )
        n_cmp = min(len(t), len(t_b), qs.shape[0], qs_b.shape[0])
        t_q = t[:n_cmp]
        qa = qs[:n_cmp]
        qb = qs_b[:n_cmp]
        dq = _wrap_angle_diff(qa, qb)
        for j in range(4):
            axes_q[j].plot(t_q, qa[:, j], label="CSV A", linewidth=1.1)
            axes_q[j].plot(t_q, qb[:, j], label="CSV B", linewidth=1.0, alpha=0.85)
            axes_q[j].set_ylabel(f"{_JOINT_NAMES[j]} [rad]")
            axes_q[j].grid(True, alpha=0.3)
            axes_q[j].legend(loc="upper right", fontsize=7)

            axes_dq[j].plot(t_q, dq[:, j], color="C2", linewidth=1.0)
            axes_dq[j].set_ylabel(f"Δ{_JOINT_NAMES[j]} [rad]")
            axes_dq[j].grid(True, alpha=0.3)
        axes_q[-1].set_xlabel("time [s]")
        axes_dq[-1].set_xlabel("time [s]")
        axes_dq[0].set_title("A − B (wrapped)")

        dq_rmse = float(np.sqrt(np.mean(dq**2)))
        print(f"Joint q RMSE (A vs B, per component): {dq_rmse:.6g} rad")
    else:
        try:
            q_read = mujoco_qpos_readback(model_path, qs)
        except Exception as exc:
            print(f"MuJoCo qpos readback 실패: {exc}", file=sys.stderr)
            q_read = qs.copy()
        dq_sr = _wrap_angle_diff(q_read, qs)
        for j in range(4):
            axes_q[j].plot(t, qs[:, j], label="CSV qi", linewidth=1.1)
            axes_q[j].plot(t, q_read[:, j], label="MuJoCo qpos (readback)", linewidth=1.0, alpha=0.85)
            axes_q[j].set_ylabel(f"{_JOINT_NAMES[j]} [rad]")
            axes_q[j].grid(True, alpha=0.3)
            axes_q[j].legend(loc="upper right", fontsize=7)

            axes_dq[j].plot(t, dq_sr[:, j], color="C2", linewidth=1.0)
            axes_dq[j].set_ylabel(f"Δ{_JOINT_NAMES[j]} [rad]")
            axes_dq[j].grid(True, alpha=0.3)
        axes_q[-1].set_xlabel("time [s]")
        axes_dq[-1].set_xlabel("time [s]")
        axes_dq[0].set_title("readback − CSV (replay sanity; ~0 if consistent)")

    plt.tight_layout()

    if args.save is not None:
        out = args.save.resolve()
        fig.savefig(out, dpi=args.dpi, bbox_inches="tight")
        err_path = out.with_name(out.stem + "_error" + out.suffix)
        fig2.savefig(err_path, dpi=args.dpi, bbox_inches="tight")
        joint_path = out.with_name(out.stem + "_joints" + out.suffix)
        joint_dq_path = out.with_name(out.stem + "_joints_diff" + out.suffix)
        fig_q.savefig(joint_path, dpi=args.dpi, bbox_inches="tight")
        fig_dq.savefig(joint_dq_path, dpi=args.dpi, bbox_inches="tight")
        print(
            f"저장: {out}\n저장: {err_path}\n저장: {joint_path}\n저장: {joint_dq_path}"
        )
    else:
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
