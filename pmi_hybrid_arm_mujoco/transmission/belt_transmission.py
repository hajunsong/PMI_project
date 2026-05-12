"""
Ideal belt path for q1_act → jnt1 (equality model already couples positions).

Torque stage: actuator-side identity so `tau_act_out` matches the ideal mapping from joint VSD.
"""


class BeltTransmission:
    """Belt joint: jnt1 / q1_act. Scaffold applies ideal pass-through on actuator torque."""

    @staticmethod
    def transmit(tau_act_ideal: float) -> float:
        return float(tau_act_ideal)

    @staticmethod
    def transmit_torque(tau_act: float, ratio: float) -> float:
        """
        Kinematic joint torque implied by actuator torque when qdot_jnt = ratio * qdot_act:
            tau_joint = tau_act / ratio
        (Not used by the cable-layer scaffold; kept for power-consistency reference.)
        """
        r = float(ratio)
        if abs(r) < 1e-12:
            raise ValueError("transmission ratio too small")
        return float(tau_act) / r
