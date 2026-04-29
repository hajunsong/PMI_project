"""기본 Gymnasium 환경: 행동은 4축 모터, 상태는 전역 관절각·속도."""

from __future__ import annotations

from typing import Any, SupportsFloat

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces
from gymnasium.core import ActType, ObsType, RenderFrame
from numpy.typing import NDArray

from pmi_mujoco_rl.model import (
    LoadOptions,
    ACTUATED_JOINTS,
    ctrl_indices_for_actuators,
    load_pmi_model,
)


class PmiArmEnv(gym.Env):
    metadata = {"render_modes": [None, "human"], "render_fps": 50}

    def __init__(
        self,
        load_options: LoadOptions | None = None,
        render_mode: str | None = None,
    ) -> None:
        super().__init__()
        self._load_options = load_options or LoadOptions()
        self.model = load_pmi_model(self._load_options)
        self.data = mujoco.MjData(self.model)
        self._ctrl_i = ctrl_indices_for_actuators(self.model)
        nq, nv = self.model.nq, self.model.nv
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(len(ACTUATED_JOINTS),), dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(nq + nv,), dtype=np.float32
        )
        self.render_mode = render_mode
        self._viewer: Any = None

    def _get_obs(self) -> NDArray[np.float32]:
        return np.concatenate([self.data.qpos, self.data.qvel]).astype(np.float32)

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[ObsType, dict[str, Any]]:
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        if options and "qpos" in options:
            self.data.qpos[:] = options["qpos"]
        if options and "qvel" in options:
            self.data.qvel[:] = options["qvel"]
        mujoco.mj_forward(self.model, self.data)
        return self._get_obs(), {}

    def step(
        self, action: ActType
    ) -> tuple[ObsType, SupportsFloat, bool, bool, dict[str, Any]]:
        a = np.asarray(action, dtype=np.float64).reshape(-1)
        if a.shape[0] != self._ctrl_i.size:
            raise ValueError(f"action dim {a.shape[0]} != {self._ctrl_i.size}")
        self.data.ctrl[self._ctrl_i] = a
        mujoco.mj_step(self.model, self.data)
        obs = self._get_obs()
        reward = 0.0
        terminated = False
        truncated = False
        return obs, reward, terminated, truncated, {}

    def render(self) -> RenderFrame | list[RenderFrame] | None:
        if self.render_mode != "human":
            return None
        import mujoco.viewer

        if self._viewer is None or not self._viewer.is_running():
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self._viewer.sync()
        return None

    def close(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
