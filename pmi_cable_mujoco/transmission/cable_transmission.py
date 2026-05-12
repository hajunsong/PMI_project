"""Scalar cable-tendon tension path: ``T_in`` [N] → ``T_out`` [N] with friction, Bouc–Wen, dead-zone, clip, optional lag.

Used by ``antagonistic_cable_joint`` (two tendons per joint). This is **not** an additive joint torque;
motor-side tension command is **filtered / distorted** along the path.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict

import numpy as np

from .backlash import StretchBacklash
from .hysteresis import BoucWenState


@dataclass
class CableTendonTransmission:
    """One tendon (positive tension only after dead-zone / clip)."""

    Fc: float
    Fv: float
    v_eps: float
    T_max: float
    tension_deadzone: float
    k_stretch: float
    bouc_wen_alpha: float
    bouc_wen_A: float
    bouc_wen_beta: float
    bouc_wen_gamma: float
    bouc_wen_n: float
    bouc_wen_z_clip: float
    max_hysteresis_force: float
    tau_first_order: float = 0.0
    stretch_backlash_width_m: float = 0.0
    stretch_backlash_slack_rate: float = 40.0

    _hyst: BoucWenState = field(init=False, repr=False)
    _T_lag: float = field(default=0.0, repr=False)
    _stretch_bl: StretchBacklash = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.v_eps = max(float(self.v_eps), 1e-9)
        self._hyst = BoucWenState(
            alpha=float(self.bouc_wen_alpha),
            A=float(self.bouc_wen_A),
            beta=float(self.bouc_wen_beta),
            gamma=float(self.bouc_wen_gamma),
            n=float(self.bouc_wen_n),
            z_clip=float(self.bouc_wen_z_clip),
        )
        self._stretch_bl = StretchBacklash(
            width_m=float(self.stretch_backlash_width_m),
            slack_rate=float(self.stretch_backlash_slack_rate),
        )

    def reset(self) -> None:
        self._hyst.reset(0.0)
        self._T_lag = 0.0
        self._stretch_bl.reset()

    @property
    def hysteresis_z(self) -> float:
        return float(self._hyst.z)

    def transmit(self, T_in: float, x: float, xdot: float, dt: float) -> tuple[float, Dict[str, Any]]:
        """Return ``(T_out, diag)`` with ``T_out ∈ [0, T_max]``."""
        dt = max(float(dt), 1e-9)
        T_in = float(T_in)

        # Optional first-order lag on commanded tension
        tau_fc = float(self.tau_first_order)
        if tau_fc > 1e-9:
            a = dt / (tau_fc + dt)
            self._T_lag = (1.0 - a) * self._T_lag + a * T_in
            T_cmd = self._T_lag
        else:
            T_cmd = T_in

        x_eff, backlash_active = self._stretch_bl.effective_stretch(float(x), float(xdot), dt)
        fh = float(self._hyst.step(float(xdot), dt, float(self.max_hysteresis_force)))
        T_raw = (
            T_cmd
            - float(self.Fc) * float(np.tanh(float(xdot) / self.v_eps))
            - float(self.Fv) * float(xdot)
            - float(self.k_stretch) * float(x_eff)
            - fh
        )

        # Non-negative tendon: dead-zone in tension [N]
        half = max(float(self.tension_deadzone), 0.0) * 0.5
        T_pos = max(0.0, T_raw)
        deadzone_active = False
        if half > 0.0 and T_pos <= half:
            T_pos = 0.0
            deadzone_active = True
        elif half > 0.0:
            T_pos -= half

        T_out = float(np.clip(T_pos, 0.0, float(self.T_max)))

        diag: Dict[str, Any] = {
            "T_in": T_in,
            "T_cmd_lagged": float(T_cmd),
            "T_raw": float(T_raw),
            "T_out": T_out,
            "x": float(x),
            "x_elastic": float(x_eff),
            "xdot": float(xdot),
            "z": float(self._hyst.z),
            "Fh": float(fh),
            "deadzone_active": bool(deadzone_active),
            "backlash_active": bool(backlash_active),
        }
        return T_out, diag
