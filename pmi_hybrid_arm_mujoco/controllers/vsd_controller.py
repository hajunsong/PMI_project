"""Motor-side virtual spring–damper torque (actuator coordinates)."""


from dataclasses import dataclass

import numpy as np


@dataclass
class VSDParams:
    Kp: np.ndarray
    Kd: np.ndarray
    tau_max: np.ndarray
    joint_names_order: tuple[str, ...]


class VSDController:
    """tau_act_cmd = diag(Kp)(q_des - q) + diag(Kd)(qdot_des - qdot) with clamping."""

    def __init__(self, params: VSDParams):
        self.params = params
        self.dim = params.Kp.size

    def compute(
        self,
        q_act: np.ndarray,
        qdot_act: np.ndarray,
        q_des: np.ndarray,
        qdot_des: np.ndarray,
    ) -> np.ndarray:
        e = np.asarray(q_des, dtype=np.float64) - np.asarray(q_act, dtype=np.float64)
        edot = np.asarray(qdot_des, dtype=np.float64) - np.asarray(qdot_act, dtype=np.float64)
        tau = self.params.Kp * e + self.params.Kd * edot
        lim = np.asarray(self.params.tau_max, dtype=np.float64)
        tau = np.clip(tau, -lim, lim)
        return tau
