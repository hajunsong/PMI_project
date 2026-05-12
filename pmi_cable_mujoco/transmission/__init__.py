"""Hybrid transmission: belt/gear q1_act↔q1_jnt (MuJoCo jnt1) + antagonistic cable q2–4_act↔q2–4_jnt (jnt2–4)."""

from .antagonistic_cable_joint import (
    AntagonisticCableJoint,
    AntagonisticCableStack,
    build_antagonistic_stack_from_config,
    build_antagonistic_stack_from_yaml,
)
from .belt_model import BeltTransmissionModel, build_belt_model_from_config
from .cable_model import CableTransmissionModel, build_cable_model_from_config
from .cable_transmission import CableTendonTransmission
from .hysteresis import BoucWenState
from .backlash import DeadzoneBacklash, StretchBacklash
from .hybrid_joint_torque import apply_transmission_joint_torque, build_belt_cable_models
from .randomization import HybridTransmissionSampler

BELT_JOINT_NAMES = ("jnt1",)
CABLE_JOINT_NAMES = ("jnt2", "jnt3", "jnt4")

__all__ = [
    "BeltTransmissionModel",
    "build_belt_model_from_config",
    "CableTransmissionModel",
    "build_cable_model_from_config",
    "CableTendonTransmission",
    "AntagonisticCableJoint",
    "AntagonisticCableStack",
    "build_antagonistic_stack_from_config",
    "build_antagonistic_stack_from_yaml",
    "apply_transmission_joint_torque",
    "build_belt_cable_models",
    "BoucWenState",
    "DeadzoneBacklash",
    "StretchBacklash",
    "HybridTransmissionSampler",
    "BELT_JOINT_NAMES",
    "CABLE_JOINT_NAMES",
]
