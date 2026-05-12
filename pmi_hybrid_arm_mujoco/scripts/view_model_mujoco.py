#!/usr/bin/env python3
"""
MuJoCo 뷰어에서 모델을 연다.

기본값은 RViz TF와 비슷하게:
  • 각 rigid body 원점 기준 XYZ 프레임 (Rendering → Coordinate frames / ``frame`` 옵션)
  • 각 조인트의 회전축 표시 (`mjVIS_JOINT`)
  • body 이름 라벨 (옵션)

GUI: Passive Viewer 패널(오른쪽)에서 동일 옵션을 추가로 바꿀 수 있다.

사용법:
  python scripts/view_model_mujoco.py
  python scripts/view_model_mujoco.py --frame site --labels joint

기본적으로 ``configs/path_tracking.yaml`` 의 ``initial_actuator_rad`` 를
``q*_act`` 에 쓰고 전달비로 ``jnt*`` 를 동기화한다 (Phase A 스크립트와 동일).
모델 XML 기본값만 보고 싶으면 ``--neutral`` 이나 (슬라이더로 조작하지 말 것).
``--classic`` 은 MuJoCo ``launch_from_path`` 만 호출하여 이 초기화는 적용되지 않는다.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import mujoco as mj
import numpy as np
from mujoco import viewer as mj_view

from utils.mujoco_helpers import DEFAULT_MODEL_PATH, apply_ideal_qjnt_equals_ratio_qact, joint_id

_FRAME_MAP: dict[str, mj.mjtFrame] = {
    "none": mj.mjtFrame.mjFRAME_NONE,
    "world": mj.mjtFrame.mjFRAME_WORLD,
    "body": mj.mjtFrame.mjFRAME_BODY,
    "geom": mj.mjtFrame.mjFRAME_GEOM,
    "camera": mj.mjtFrame.mjFRAME_CAMERA,
    "site": mj.mjtFrame.mjFRAME_SITE,
}


def _label_from_arg(s: str) -> mj.mjtLabel | None:
    s = str(s).strip().lower()
    if s == "none":
        return mj.mjtLabel.mjLABEL_NONE
    return {
        "body": mj.mjtLabel.mjLABEL_BODY,
        "joint": mj.mjtLabel.mjLABEL_JOINT,
        "site": mj.mjtLabel.mjLABEL_SITE,
        "geom": mj.mjtLabel.mjLABEL_GEOM,
    }.get(s, mj.mjtLabel.mjLABEL_NONE)


def apply_initial_from_path_tracking(
    model: mj.MjModel,
    data: mj.MjData,
    *,
    cfg_path: Path | None = None,
) -> tuple[np.ndarray, np.ndarray, list[str], list[str]]:
    """``path_tracking.yaml``의 ``initial_actuator_rad`` + 전달 동기화 → ``data.qpos``."""
    from utils.path_tracking_io import load_path_tracking_yaml, ordered_transmission_arrays

    cfg = load_path_tracking_yaml(cfg_path)
    j_ord, a_ord, ratios = ordered_transmission_arrays(cfg)
    r = np.asarray(ratios, dtype=np.float64)
    qa0 = np.asarray(cfg["initial_actuator_rad"], dtype=np.float64)

    if qa0.shape[0] != len(a_ord):
        raise ValueError("initial_actuator_rad length must match actuator count")

    for i, nm in enumerate(a_ord):
        adr = int(model.jnt_qposadr[joint_id(model, nm)])
        data.qpos[adr] = float(qa0[i])
    apply_ideal_qjnt_equals_ratio_qact(model, data, j_ord, a_ord, r)
    mj.mj_forward(model, data)

    qj = np.array(
        [float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in j_ord],
        dtype=np.float64,
    )

    qa_read = np.array(
        [float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in a_ord],
        dtype=np.float64,
    )

    return qa_read, qj, j_ord, a_ord


def configure_tf_style(
    vopt: mj.MjvOption,
    *,
    frame: mj.mjtFrame,
    show_joint_axes: bool,
    label_kind: mj.mjtLabel | None,
) -> None:
    """Passive viewer의 MjvOption에 RViz TF에 가까운 초기 상태를 적용."""
    # 정리 후 기본 활성 플래그
    if show_joint_axes:
        vopt.flags[int(mj.mjtVisFlag.mjVIS_JOINT)] = True
    else:
        vopt.flags[int(mj.mjtVisFlag.mjVIS_JOINT)] = False

    # COM 등은 과밀하게 보이므로 끈다
    vopt.flags[int(mj.mjtVisFlag.mjVIS_COM)] = False
    vopt.flags[int(mj.mjtVisFlag.mjVIS_CONTACTPOINT)] = False

    vopt.frame = frame
    if label_kind is not None:
        vopt.label = label_kind


def main() -> None:
    parser = argparse.ArgumentParser(description="pmi_hybrid_arm MuJoCo 뷰어 (좌표계/조인트 시각화)")
    parser.add_argument(
        "--frame",
        choices=("none", "world", "body", "geom", "camera", "site"),
        default="body",
        help="좌표계 프레임 (기본 body: 각 링크/모터 body 원점 XYZ, RViz TF와 유사)",
    )
    parser.add_argument("--no-joint-axes", action="store_true", help="힌지 회전축 하이라이트 끔")
    parser.add_argument(
        "--labels",
        choices=("none", "body", "joint", "site", "geom"),
        default="body",
        help="이름 라벨 (기본 body)",
    )
    parser.add_argument(
        "--classic",
        action="store_true",
        help=(
            "기존처럼 launch_from_path 만 (coordinate frame/패시브 옵션 없음)."
            " path_tracking 초기 자세 미적용 — 모델 XML 기본 qpos."
        ),
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="path_tracking.yaml 경로 (기본: 패키지 내 configs/path_tracking.yaml)",
    )
    parser.add_argument(
        "--neutral",
        action="store_true",
        help="모델 XML 기본 qpos 유지(initial_actuator_rad 및 전달 동기화 미적용)",
    )
    args = parser.parse_args()

    xml = str(DEFAULT_MODEL_PATH)
    if args.classic:
        mj_view.launch_from_path(xml)
        return

    model = mj.MjModel.from_xml_path(xml)
    data = mj.MjData(model)
    cfg_path_opt: Path | None = args.config
    if args.neutral:
        mj.mj_forward(model, data)
    else:
        qa_act, qj, j_ord, a_ord = apply_initial_from_path_tracking(
            model, data, cfg_path=cfg_path_opt
        )
        cfg_repr = str(cfg_path_opt.resolve()) if cfg_path_opt else "패키지 기본 configs/path_tracking.yaml"
        print("[view_model_mujoco] path_tracking 초기 자세 적용 (cfg=%s)" % cfg_repr)
        print("               %s" % ", ".join("%s=%.4f" % (n, float(qa_act[i])) for i, n in enumerate(a_ord)))
        print("               %s" % ", ".join("%s=%.4f" % (n, float(qj[i])) for i, n in enumerate(j_ord)))

    frame = _FRAME_MAP[args.frame]
    lab = None if args.labels == "none" else _label_from_arg(args.labels)

    with mj_view.launch_passive(model, data) as v:
        configure_tf_style(v.opt, frame=frame, show_joint_axes=(not args.no_joint_axes), label_kind=lab)
        print(
            "[view_model_mujoco] TF 스타일: frame=%s, joint_axes=%s, labels=%s"
            % (
                args.frame,
                str(not args.no_joint_axes),
                args.labels,
            )
        )
        print("               오른쪽 패널 Rendering에서 Frame / Visualization을 추가로 변경할 수 있습니다.")
        print("               종료: 창 닫기")
        while v.is_running():
            v.sync()


if __name__ == "__main__":
    main()
