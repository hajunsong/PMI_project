"""Nonlinear cable tension transmission (motor torque → tension / joint torque). Phase-3 path."""

from dataclasses import dataclass

import numpy as np

from .backlash import Backlash1D, BacklashParams
from .hysteresis import BoucWenHysteresis, BoucWenParams


@dataclass
class CableTransmissionParams:
    tau_delay_sec: float
    k_elastic: float
    c_viscous_stretch: float
    Fc: float
    v_eps: float
    Fv: float
    deadzone_xdot: float
    backlash_x: float
    T_clip: float
    bw: BoucWenParams


class ImperfectCableTransmission:
    """
    Motor torque converts to commanded cable intake tension via pulley radius r_motor:
        T_in_cmd = tau_motor_cmd / r_motor

    Then this model emits T_out (>=0 for taut cable strand) distorted by elasticity,
    Coulomb friction, stretch-rate damping, Bouc-Wen hysteresis, backlash on stretch.

    IMPORTANT: Stretch must be kinematic from states:
        x = r_motor * q_act - r_joint * q_joint - x0
    NEVER from desired-minus-feedback on joint angle alone.
    """

    def __init__(self, p: CableTransmissionParams):
        self.p = p
        self._T_delayed = 0.0
        self._backlash = Backlash1D(BacklashParams(width=p.backlash_x))
        self._hyst = BoucWenHysteresis(p.bw)

    def reset(self) -> None:
        self._T_delayed = 0.0
        self._backlash.reset(0.0)
        self._hyst.reset()

    def transmit(
        self,
        tau_motor_cmd: float,
        r_motor: float,
        x_raw: float,
        xdot: float,
        dt: float,
    ) -> tuple[float, dict]:
        """
        Returns (T_out, diag) where downstream joint torque is tau_joint=r_joint*T_out.

        Structural rule (no parallel torque injection):
          VSD torque only enters THROUGH this transmitter as tau_motor_cmd.
        """
        rm = float(r_motor)
        if abs(rm) < 1e-12:
            raise ValueError("r_motor too small")

        Tin = float(tau_motor_cmd) / rm
        tau_d = float(max(self.p.tau_delay_sec, 0.0))
        td = float(max(dt, 1e-9))
        if tau_d <= 1e-9:
            self._T_delayed = Tin
        else:
            a = td / (tau_d + td)
            self._T_delayed = (1.0 - a) * self._T_delayed + a * Tin
        Tin_f = float(self._T_delayed)

        x_filt, backlash_active = self._backlash.step(x_raw)
        _fh, z = self._hyst.force(xdot, dt)

        vz = abs(float(xdot))
        dz = abs(float(x_filt))

        dz_gate = vz < float(max(self.p.deadzone_xdot, 0.0))
        # Small dead-zone on stiffness term to avoid jitter at rest
        if dz_gate:
            Fel = 0.0
        else:
            Fel = float(self.p.k_elastic) * dz + float(self.p.c_viscous_stretch) * vz

        Fcoup = Fel + float(self.p.Fc) * np.tanh(float(xdot) / max(self.p.v_eps, 1e-6))
        Fcoup += float(self.p.Fv) * float(xdot)
        Fcoup += _fh

        T_raw = Tin_f - Fcoup
        T_clip = abs(float(max(self.p.T_clip, 0.0)))
        # Single cable taut-only branch
        T_out = float(max(0.0, T_raw))
        T_out = float(min(T_out, T_clip))

        diag = {
            "T_in_cmd_raw": Tin,
            "T_in_delayed": Tin_f,
            "x_raw": float(x_raw),
            "x_backlash_filtered": float(x_filt),
            "xdot": float(xdot),
            "bouc_z": float(z),
            "hysteresis_force": float(_fh),
            "Fel": float(Fel),
            "backlash_sliding_active": bool(backlash_active),
            "deadzone_xdot_gate": bool(dz_gate),
            "T_out": T_out,
        }
        return T_out, diag


def cable_stretch(
    q_act: float,
    q_joint: float,
    r_motor: float,
    r_joint: float,
    x0: float,
) -> float:
    return float(r_motor) * float(q_act) - float(r_joint) * float(q_joint) - float(x0)


def cable_stretch_rate(
    qdot_act: float,
    qdot_joint: float,
    r_motor: float,
    r_joint: float,
) -> float:
    return float(r_motor) * float(qdot_act) - float(r_joint) * float(qdot_joint)
