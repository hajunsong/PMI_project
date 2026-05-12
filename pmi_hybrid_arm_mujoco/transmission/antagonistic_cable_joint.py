"""Antagonistic cable pair routed through two CableTransmission elements."""


from dataclasses import dataclass

from .cable_transmission import CableTransmission, CableTransmissionParams, cable_stretch, cable_stretch_rate


@dataclass
class AntagonisticParams:
    preload_tension: float


class AntagonisticCableJoint:
    """
    User-spec branch commands (preload + quadrant split at MOTOR side):

        T_plus_cmd  = T_preload + max(tau_motor_cmd, 0) / max(r_joint, eps)
        T_minus_cmd = T_preload + max(-tau_motor_cmd, 0) / max(r_joint, eps)

    Each CableTransmission receives motor-side tau_branch = r_motor * T_branch_cmd interpreted
    as equivalent pulley winding torque before nonlinearities.

        tau_joint = r_joint * (T_plus_out - T_minus_out)

    Structural rule:
        tau_motor_cmd is ONLY the Phase-2/VSD actuator torque; imperfections enter solely via CableTransmission().
    """

    def __init__(
        self,
        cable_params: CableTransmissionParams,
        antag: AntagonisticParams,
    ):
        self._preload = float(max(antag.preload_tension, 0.0))
        self._plus = CableTransmission(cable_params)
        self._minus = CableTransmission(cable_params)

    def reset(self) -> None:
        self._plus.reset()
        self._minus.reset()

    def transmit(
        self,
        tau_motor_cmd: float,
        r_joint: float,
        r_motor: float,
        q_act: float,
        q_joint: float,
        qdot_act: float,
        qdot_joint: float,
        x0_plus: float,
        x0_minus: float,
        dt: float,
    ) -> tuple[float, dict]:
        rj = max(abs(float(r_joint)), 1e-9)
        rm = max(abs(float(r_motor)), 1e-9)
        tm = float(tau_motor_cmd)

        Tplus_cmd = self._preload + max(tm, 0.0) / rj
        Tminus_cmd = self._preload + max(-tm, 0.0) / rj

        xp = cable_stretch(q_act, q_joint, rm, float(r_joint), float(x0_plus))
        xm = cable_stretch(q_act, q_joint, rm, float(r_joint), float(x0_minus))
        xv = cable_stretch_rate(qdot_act, qdot_joint, rm, float(r_joint))

        tau_plus_act = rm * Tplus_cmd
        tau_minus_act = rm * Tminus_cmd

        Tp_out, dp = self._plus.transmit(tau_plus_act, rm, xp, xv, dt)
        Tm_out, dm = self._minus.transmit(tau_minus_act, rm, xm, xv, dt)

        T_net = Tp_out - Tm_out
        tau_joint = float(r_joint) * T_net

        diag = {
            "T_plus_cmd": Tplus_cmd,
            "T_minus_cmd": Tminus_cmd,
            "T_plus_out": Tp_out,
            "T_minus_out": Tm_out,
            "T_net": T_net,
            "tau_joint_transmitted": tau_joint,
            "tau_motor_reference": tm,
            "branch_plus": dp,
            "branch_minus": dm,
        }
        return tau_joint, diag
