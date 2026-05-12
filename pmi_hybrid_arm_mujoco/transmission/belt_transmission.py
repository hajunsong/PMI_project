"""
Ideal belt/gear torque mapping (motor-side actuator torque -> joint torque).

Position law (same as mimic):
    theta_joint = ratio * theta_actuator

Consistent instantaneous power approximate identity:
    tau_act * qdot_act = tau_joint * qdot_joint ,   qdot_joint = ratio * qdot_act
=> tau_joint = tau_act / ratio
"""


class BeltTransmission:
    """Simple ratio element for jnt1 / q1_act only (no elasticity in Phase 3 path)."""

    @staticmethod
    def transmit_torque(tau_act: float, ratio: float) -> float:
        r = float(ratio)
        if abs(r) < 1e-12:
            raise ValueError("transmission ratio too small")
        return float(tau_act) / r
