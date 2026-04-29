#!/usr/bin/env python3
"""모델 로드 후 무작위 행동으로 짧게 스텝하여 안정성 확인."""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

# 개발 시 패키지 루트(pmi_mujoco_rl/)를 path에 넣기
_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from pmi_mujoco_rl.model import LoadOptions, ctrl_indices_for_actuators, load_pmi_model


def main() -> None:
    # 중력 0: URDF equalities·STL 없이도 초기 스모크 테스트만 빠르게 확인.
    opts = LoadOptions(
        disable_collision=True,
        mimic_equalities=True,
        gravity=(0.0, 0.0, 0.0),
    )
    m = load_pmi_model(opts)
    d = mujoco.MjData(m)
    ci = ctrl_indices_for_actuators(m)
    rng = np.random.default_rng(0)
    for _ in range(100):
        mujoco.mj_step(m, d)
    print("nq", m.nq, "nv", m.nv, "nu", m.nu, "neq", m.neq)
    print("idle max |qacc|", np.abs(d.qacc).max())
    for _ in range(300):
        d.ctrl[ci] = rng.uniform(-0.02, 0.02, size=4)
        mujoco.mj_step(m, d)
    print("small_noise max |qacc|", np.abs(d.qacc).max())


if __name__ == "__main__":
    main()
