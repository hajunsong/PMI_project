#!/usr/bin/env python3
"""작업공간 VSD + 토크 모터로 EE 경로 추종 (``pmi_cable_arm.xml``).

``demo_vsd_torque_path_follow.py`` 와 **동일한 구현**을 실행한다. 이 파일은 경로 추종 데모용
진입점만 분리한 것이다 (설정·CLI·뷰어 오버레이·웨이포인트 도찴 종료 등 전부 동일).

기본 설정: ``configs/control_params_vsd_task.yaml`` (``trajectory``, ``task_space_vsd``, ``simulation``).
기본으로 ``configs/belt_params.yaml`` + ``configs/cable_params.yaml`` 전달 모델이 VSD 토크에 합산된다. 끄려면 ``--no-transmission``.

실행 예::

    cd pmi_cable_mujoco
    python scripts/path_follow_vsd.py
    python scripts/path_follow_vsd.py --config configs/control_params_vsd_task.yaml
    python scripts/path_follow_vsd.py --headless --steps 3000 --settle-steps 120
    python scripts/path_follow_vsd.py --ks 150,150,150,0,0 --kd 25,25,25,0,0

그래프 저장은 ``plot_vsd_torque_path_desired_vs_ee.py`` 를 사용한다.
"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    rs = str(root)
    if rs not in sys.path:
        sys.path.insert(0, rs)

    target = Path(__file__).resolve().parent / "demo_vsd_torque_path_follow.py"
    if not target.is_file():
        raise FileNotFoundError(f"Missing implementation file: {target}")

    runpy.run_path(str(target), run_name="__main__")


if __name__ == "__main__":
    main()
