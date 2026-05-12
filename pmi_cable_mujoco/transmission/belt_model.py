"""Belt/gear parasitics on q1_jnt (MuJoCo ``jnt1``): motor shaft q1_act ↔ arm via timing belt; no cable Bouc–Wen."""

from __future__ import annotations

from typing import Any, Dict

import numpy as np

from .backlash import DeadzoneBacklash


class BeltTransmissionModel:
    """Additive belt disturbance torque on joint 1.

    .. math::

        \\tau_1^{belt} =
            (\\eta - 1)\\,\\tau_{drive}
            - (b_{belt} G^2 + b_v)\\,\\dot q
            - F_c G \\tanh(\\dot q / v_\\varepsilon)
            - k_{bl}\\, e_{eff}

    where :math:`\\tau_{drive} = \\tau_{vsd,1} + \\lambda_1 \\tau_{res,1}`,
    :math:`G` is ``gear_ratio``, :math:`\\eta` is ``efficiency``,
    and :math:`e_{eff}` comes from a small dead-zone/backlash block (optional).

    The total joint torque remains::

        \\tau_1 = \\tau_{drive} + \\tau_1^{belt}

    (clip applied to :math:`\\tau_1^{belt}` only).
    """

    def __init__(
        self,
        *,
        gear_ratio: float,
        efficiency: float,
        belt_damping: float,
        belt_coulomb: float,
        belt_viscous: float,
        v_eps: float,
        deadzone_width: float,
        backlash_width: float,
        backlash_slope: float,
        backlash_elastic_gain: float,
        max_belt_effect_torque: float,
    ) -> None:
        self.gear_ratio = float(gear_ratio)
        self.efficiency = float(np.clip(efficiency, 0.05, 1.0))
        self.belt_damping = float(belt_damping)
        self.belt_coulomb = float(belt_coulomb)
        self.belt_viscous = float(belt_viscous)
        self.v_eps = max(float(v_eps), 1e-6)
        self.backlash_elastic_gain = float(backlash_elastic_gain)
        self.max_belt_effect = float(max_belt_effect_torque)
        self._backlash = DeadzoneBacklash(
            deadzone_width=float(deadzone_width),
            backlash_width=float(backlash_width),
            backlash_slope=float(backlash_slope),
        )

    def reset(self) -> None:
        self._backlash.reset()

    def compute_effect(
        self,
        q: float,
        qdot: float,
        q_des: float,
        dt: float,
        tau_drive: float,
    ) -> tuple[float, Dict[str, float]]:
        """Return ``tau_belt_effect`` and scalar diagnostics."""
        g = self.gear_ratio
        # Efficiency loss relative to ideal torque command (additive form).
        tau_eta = (self.efficiency - 1.0) * tau_drive

        tau_visc = -(self.belt_damping * (g ** 2) + self.belt_viscous) * qdot
        tau_coul = -self.belt_coulomb * abs(g) * np.tanh(qdot / self.v_eps)

        err_eff = self._backlash.effective_tracking_error(q_des, q, qdot, dt)
        tau_bl = -self.backlash_elastic_gain * err_eff

        tau_belt = tau_eta + tau_visc + tau_coul + tau_bl
        tau_belt = float(np.clip(tau_belt, -self.max_belt_effect, self.max_belt_effect))

        diag: Dict[str, float] = {
            "tau_eta_loss": float(tau_eta),
            "tau_viscous": float(tau_visc),
            "tau_coulomb": float(tau_coul),
            "tau_backlash": float(tau_bl),
            "tau_belt_total": tau_belt,
        }
        return tau_belt, diag


def build_belt_model_from_config(cfg: Dict[str, Any]) -> BeltTransmissionModel:
    return BeltTransmissionModel(
        gear_ratio=float(cfg["gear_ratio"]),
        efficiency=float(cfg["efficiency"]),
        belt_damping=float(cfg["belt_damping"]),
        belt_coulomb=float(cfg["belt_coulomb"]),
        belt_viscous=float(cfg["belt_viscous"]),
        v_eps=float(cfg["v_eps"]),
        deadzone_width=float(cfg["deadzone_width"]),
        backlash_width=float(cfg["backlash_width"]),
        backlash_slope=float(cfg["backlash_slope"]),
        backlash_elastic_gain=float(cfg.get("backlash_elastic_gain", 6.0)),
        max_belt_effect_torque=float(cfg.get("max_belt_effect_torque", 14.0)),
    )
