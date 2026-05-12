"""MuJoCo 관절 공간 순수 중력(정적) 참조 토크 — ``qvel=0`` 일 때 ``qfrc_bias`` 와 동일 성분."""

from __future__ import annotations

from typing import Sequence

import mujoco
import numpy as np


def joint_gravity_reference_torques(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    dof_adr: Sequence[int],
) -> np.ndarray:
    """현재 ``qpos`` 에서 ``qvel=0`` 이라 가정할 때의 바이어스 토크 (중력 위주).

    ``mj_forward`` 후 ``qfrc_bias`` 를 읽고 ``qvel`` 을 원래대로 복구한다.
    """
    qvel = np.array(data.qvel, dtype=np.float64, copy=True)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    out = np.array([float(data.qfrc_bias[int(i)]) for i in dof_adr], dtype=np.float64)
    data.qvel[:] = qvel
    mujoco.mj_forward(model, data)
    return out
