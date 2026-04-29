#!/usr/bin/env python3
"""PMI URDF를 MuJoCo Simulate 창으로 표시.

WSL2에서 GUI가 필요할 때:
- Windows 11 + WSLg: 대부분 추가 설정 없이 동작. 안 되면 터미널에서 ``echo $DISPLAY`` 로 ``:0`` 근처인지 확인.
- 구형 구성(VcXsrv 등): Windows에 X 서버 실행 후, WSL에서 Windows 호스트 IP로 DISPLAY 설정.
  예: ``export DISPLAY=$(grep -m1 nameserver /etc/resolv.conf | awk '{print $2}'):0``

텍스트만으로 URDF 반영 여부를 보려면: ``python view_model.py --summary``
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import mujoco
import mujoco.viewer  # 서브모듈을 로드해야 mujoco.viewer.launch 사용 가능

from pmi_mujoco_rl.model import LoadOptions, format_model_summary, load_pmi_model


def _is_wsl() -> bool:
    try:
        with open("/proc/version", encoding="utf-8") as f:
            return "microsoft" in f.read().lower()
    except OSError:
        return False


def _print_gui_env_hints() -> None:
    """GUI가 안 뜰 때 흔한 원인을 터미널에 안내 (WSL·MuJoCo GL 백엔드)."""
    display = os.environ.get("DISPLAY", "")
    mujoco_gl = os.environ.get("MUJOCO_GL", "").strip().lower()
    glfw_plat = os.environ.get("GLFW_PLATFORM", "").strip().lower()

    print("\n[MuJoCo GUI 점검]", file=sys.stderr)
    print(f"  DISPLAY={display!r}", file=sys.stderr)
    _gl = mujoco_gl or "(미설정·기본 glfw)"
    print(f"  MUJOCO_GL={_gl!r}", file=sys.stderr)
    if glfw_plat:
        print(f"  GLFW_PLATFORM={glfw_plat!r}", file=sys.stderr)

    if sys.platform == "linux" and not display:
        print(
            "  → DISPLAY가 비어 있으면 창을 못 뜁니다. WSLg:  export DISPLAY=:0",
            file=sys.stderr,
        )
        if _is_wsl():
            print(
                "  → WSLg 설치/업데이트: Windows 11 쪽 ‘Windows Subsystem for Linux’ 확인.",
                file=sys.stderr,
            )

    if sys.platform == "linux" and mujoco_gl in ("egl", "osmesa"):
        print(
            "  → MUJOCO_GL가 egl/osmesa면 ‘창 없는’ 렌더링입니다. "
            "인터랙티브 뷰어는 다음 후 다시 실행하세요:\n"
            "       unset MUJOCO_GL\n"
            "       export MUJOCO_GL=glfw",
            file=sys.stderr,
        )

    if sys.platform == "linux" and glfw_plat == "wayland":
        print(
            "  → WSL에서 Wayland만 잡히면 창이 안 뜨는 경우가 있습니다. 시도:\n"
            "       export GLFW_PLATFORM=x11",
            file=sys.stderr,
        )


def main() -> None:
    ap = argparse.ArgumentParser(description="MuJoCo Simulate로 PMI 모델 표시")
    ap.add_argument(
        "--summary",
        action="store_true",
        help="창 없이 메시·관절 요약만 출력",
    )
    ap.add_argument(
        "--collision",
        action="store_true",
        help="geom 접촉 활성화(LoadOptions.disable_collision=False)",
    )
    ap.add_argument(
        "--no-equality",
        action="store_true",
        help="mimic equality 비활성화(디버그용)",
    )
    args = ap.parse_args()

    opts = LoadOptions(
        disable_collision=not args.collision,
        mimic_equalities=not args.no_equality,
    )
    model = load_pmi_model(opts)
    if args.summary:
        print(format_model_summary(model))
        if os.environ.get("DISPLAY", "") == "" and sys.platform == "linux":
            print(
                "\n(참고) DISPLAY가 비어 있으면 GUI는 실패할 수 있습니다. "
                "WSLg면 보통 :0 입니다: export DISPLAY=:0",
                file=sys.stderr,
            )
        return

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    print(format_model_summary(model))
    _print_gui_env_hints()
    print("\nSimulate 창을 닫으면 종료합니다.", flush=True)
    try:
        mujoco.viewer.launch(model, data)
    except Exception as e:
        print(f"\nviewer.launch 실패: {e}", file=sys.stderr)
        _print_gui_env_hints()
        raise


if __name__ == "__main__":
    main()
