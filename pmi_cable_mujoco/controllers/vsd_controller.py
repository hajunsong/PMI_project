"""Virtual spring-damper tracking controller + simple reference trajectories."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Literal, Tuple

import numpy as np


Mode = Literal["sine", "step_hold"]


@dataclass
class ReferenceTrajectory:
    """Generates :math:`q_{des}(t), \\dot q_{des}(t)` from ``control_params.yaml`` trajectory section."""

    mode: Mode
    amplitude: np.ndarray
    frequency_hz: np.ndarray
    phase: np.ndarray
    step_target: np.ndarray
    step_switch_step: int
    dt: float
    _step_count: int = 0

    @classmethod
    def from_config(cls, traj_cfg: Dict[str, Any], dt: float) -> "ReferenceTrajectory":
        mode = str(traj_cfg.get("mode", "sine"))
        if mode not in ("sine", "step_hold"):
            mode = "sine"
        return cls(
            mode=mode,  # type: ignore[assignment]
            amplitude=np.asarray(traj_cfg["amplitude"], dtype=float),
            frequency_hz=np.asarray(traj_cfg["frequency_hz"], dtype=float),
            phase=np.asarray(traj_cfg["phase"], dtype=float),
            step_target=np.asarray(traj_cfg["step_target"], dtype=float),
            step_switch_step=int(traj_cfg.get("step_switch_step", 400)),
            dt=float(dt),
            _step_count=0,
        )

    def reset(self, phase_scale: float = 1.0) -> None:
        self._step_count = 0

    def _eval_at_count(self, k: int) -> Tuple[np.ndarray, np.ndarray]:
        t = k * self.dt
        n = self.amplitude.size
        q_des = np.zeros(n, dtype=float)
        qdot_des = np.zeros(n, dtype=float)

        if self.mode == "sine":
            two_pi_f = 2.0 * np.pi * self.frequency_hz
            q_des = self.amplitude * np.sin(two_pi_f * t + self.phase)
            qdot_des = self.amplitude * two_pi_f * np.cos(two_pi_f * t + self.phase)
        else:
            if k < self.step_switch_step:
                q_des[:] = 0.0
                qdot_des[:] = 0.0
            else:
                q_des[:] = self.step_target
                qdot_des[:] = 0.0

        return q_des, qdot_des

    def peek(self) -> Tuple[np.ndarray, np.ndarray]:
        """Reference at current step index without advancing time."""
        return self._eval_at_count(self._step_count)

    def step(self) -> Tuple[np.ndarray, np.ndarray]:
        """Advance internal clock and return (q_des, qdot_des)."""
        out = self._eval_at_count(self._step_count)
        self._step_count += 1
        return out


class VSDController:
    """``tau = clip(Kp * e + Kd * edot, ±tau_max)``."""

    def __init__(
        self,
        Kp: np.ndarray,
        Kd: np.ndarray,
        tau_saturation: np.ndarray,
    ) -> None:
        self.Kp = Kp.astype(float)
        self.Kd = Kd.astype(float)
        self.tau_sat = tau_saturation.astype(float)

    @classmethod
    def from_config(cls, ctrl: Dict[str, Any]) -> "VSDController":
        return cls(
            Kp=np.asarray(ctrl["Kp"], dtype=float),
            Kd=np.asarray(ctrl["Kd"], dtype=float),
            tau_saturation=np.asarray(ctrl["tau_saturation"], dtype=float),
        )

    def compute(
        self,
        q: np.ndarray,
        qdot: np.ndarray,
        q_des: np.ndarray,
        qdot_des: np.ndarray,
    ) -> np.ndarray:
        e = q_des - q
        ed = qdot_des - qdot
        tau = self.Kp * e + self.Kd * ed
        return np.clip(tau, -self.tau_sat, self.tau_sat)
