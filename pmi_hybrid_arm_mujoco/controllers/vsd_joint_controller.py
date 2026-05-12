"""Joint-space Virtual Spring Damper (for Phase B IK tracking)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class JointVSDParams:
    Kp: np.ndarray  # shape (4,)
    Kd: np.ndarray
    tau_jnt_limit: np.ndarray


class JointSpaceVSD:
    """tau_jnt_cmd = diag(Kp) e + diag(Kd) edot ; elementwise saturate."""

    def __init__(self, params: JointVSDParams) -> None:
        self.p = params

    def compute(
        self,
        q_actual: np.ndarray,
        qdot_actual: np.ndarray,
        q_des: np.ndarray,
        qdot_des: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return (tau_jnt clipped, saturation flag per joint)."""
        e = np.asarray(q_des, dtype=np.float64) - np.asarray(q_actual, dtype=np.float64)
        edot = np.asarray(qdot_des, dtype=np.float64) - np.asarray(qdot_actual, dtype=np.float64)
        tau_unc = self.p.Kp * e + self.p.Kd * edot
        lim = np.asarray(self.p.tau_jnt_limit, dtype=np.float64)
        tau_sat = np.clip(tau_unc, -lim, lim)
        sat = np.abs(tau_unc - tau_sat) > 1e-9
        return tau_sat.astype(np.float64), sat.astype(bool)


def actuator_torques_from_joint(tau_joint: np.ndarray, ratios: np.ndarray) -> np.ndarray:
    """
    Power-consistency mapping for q_jnt = ratio * q_act (same kinematic pairing as 요구 명세 Phase B):

        tau_act,i = ratio_i * tau_joint,i

    즉 actuator 토크는 관절 토크에 ratio 만큼 더 곱합니다.
    """
    r = np.asarray(ratios, dtype=np.float64)
    tj = np.asarray(tau_joint, dtype=np.float64)
    return r * tj
