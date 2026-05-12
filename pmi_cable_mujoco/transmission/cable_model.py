"""Cable+pulley parasitics on q2–4_jnt (MuJoCo ``jnt2``..``jnt4``): motor shafts q2_act..q4_act; 3 joints."""

from __future__ import annotations

from typing import Any, Dict, List

import numpy as np

from .backlash import DeadzoneBacklash
from .hysteresis import BoucWenState


class CableTransmissionModel:
    """Equivalent cable forces mapped to joint torque via pulley radius (jnt2–jnt4).

    Internal index ``i = 0..2`` maps to MuJoCo joints jnt2, jnt3, jnt4.
    """

    def __init__(
        self,
        *,
        num_joints: int,
        pulley_radius: np.ndarray,
        stiffness: np.ndarray,
        damping: np.ndarray,
        coulomb: np.ndarray,
        viscous: np.ndarray,
        v_eps: np.ndarray,
        deadzone_width: np.ndarray,
        backlash_width: np.ndarray,
        backlash_slope: np.ndarray,
        bouc_wen_alpha: np.ndarray,
        bouc_wen_A: np.ndarray,
        bouc_wen_beta: np.ndarray,
        bouc_wen_gamma: np.ndarray,
        bouc_wen_n: np.ndarray,
        bouc_wen_z_clip: np.ndarray,
        max_cable_force: float,
        max_hysteresis_force: float,
    ) -> None:
        self.num_joints = num_joints
        self.r = pulley_radius.astype(float)
        self.k = stiffness.astype(float)
        self.d = damping.astype(float)
        self.Fc = coulomb.astype(float)
        self.Fv = viscous.astype(float)
        self.v_eps = np.maximum(v_eps.astype(float), 1e-6)
        self.max_cable_force = float(max_cable_force)
        self.max_hysteresis_force = float(max_hysteresis_force)

        self.backlash: List[DeadzoneBacklash] = [
            DeadzoneBacklash(
                deadzone_width=float(deadzone_width[i]),
                backlash_width=float(backlash_width[i]),
                backlash_slope=float(backlash_slope[i]),
            )
            for i in range(num_joints)
        ]
        self.hysteresis: List[BoucWenState] = [
            BoucWenState(
                alpha=float(bouc_wen_alpha[i]),
                A=float(bouc_wen_A[i]),
                beta=float(bouc_wen_beta[i]),
                gamma=float(bouc_wen_gamma[i]),
                n=float(bouc_wen_n[i]),
                z_clip=float(bouc_wen_z_clip[i]),
            )
            for i in range(num_joints)
        ]

    def reset(self) -> None:
        for b in self.backlash:
            b.reset()
        for h in self.hysteresis:
            h.reset(0.0)

    def compute_cable_torque(
        self,
        q_cable: np.ndarray,
        qdot_cable: np.ndarray,
        q_des_cable: np.ndarray,
        qdot_des_cable: np.ndarray,
        dt: float,
    ) -> tuple[np.ndarray, Dict[str, np.ndarray]]:
        """Return ``tau_cable`` shape (num_joints,) for jnt2–jnt4 order."""
        tau = np.zeros(self.num_joints, dtype=float)
        forces = np.zeros(self.num_joints, dtype=float)
        z_states = np.zeros(self.num_joints, dtype=float)
        extensions = np.zeros(self.num_joints, dtype=float)

        for i in range(self.num_joints):
            r = self.r[i]
            xdot = r * qdot_cable[i]

            err_eff = self.backlash[i].effective_tracking_error(
                float(q_des_cable[i]),
                float(q_cable[i]),
                float(qdot_cable[i]),
                float(dt),
            )
            extension = r * err_eff
            extensions[i] = extension

            F_el = self.k[i] * extension
            F_d = self.d[i] * xdot
            F_f = self.Fc[i] * np.tanh(xdot / self.v_eps[i]) + self.Fv[i] * xdot

            F_h = self.hysteresis[i].step(xdot, dt, self.max_hysteresis_force)
            z_states[i] = self.hysteresis[i].z

            F_total = F_el + F_d + F_f + F_h
            F_total = float(np.clip(F_total, -self.max_cable_force, self.max_cable_force))
            forces[i] = F_total
            tau[i] = r * F_total

        diag: Dict[str, np.ndarray] = {
            "cable_force": forces,
            "z_cable": z_states,
            "extension": extensions,
        }
        return tau, diag


def build_cable_model_from_config(cfg: Dict[str, Any]) -> CableTransmissionModel:
    """Build from ``cable_params.yaml`` (``num_joints`` must be 3)."""
    n = int(cfg["num_joints"])
    bw = cfg["bouc_wen"]
    return CableTransmissionModel(
        num_joints=n,
        pulley_radius=np.asarray(cfg["pulley_radius"], dtype=float),
        stiffness=np.asarray(cfg["stiffness"], dtype=float),
        damping=np.asarray(cfg["damping"], dtype=float),
        coulomb=np.asarray(cfg["coulomb"], dtype=float),
        viscous=np.asarray(cfg["viscous"], dtype=float),
        v_eps=np.asarray(cfg["v_eps"], dtype=float),
        deadzone_width=np.asarray(cfg["deadzone_width"], dtype=float),
        backlash_width=np.asarray(cfg["backlash_width"], dtype=float),
        backlash_slope=np.asarray(cfg["backlash_slope"], dtype=float),
        bouc_wen_alpha=np.asarray(bw["alpha"], dtype=float),
        bouc_wen_A=np.asarray(bw["A"], dtype=float),
        bouc_wen_beta=np.asarray(bw["beta"], dtype=float),
        bouc_wen_gamma=np.asarray(bw["gamma"], dtype=float),
        bouc_wen_n=np.asarray(bw["n"], dtype=float),
        bouc_wen_z_clip=np.asarray(bw["z_clip"], dtype=float),
        max_cable_force=float(cfg.get("max_cable_force", 500.0)),
        max_hysteresis_force=float(cfg.get("max_hysteresis_force", 80.0)),
    )
