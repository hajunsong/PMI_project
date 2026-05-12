"""Antagonistic cable pair: motor torque command → two tendon tensions → net joint torque."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
import yaml

from kinematics.pmi_chain import MOTOR_TO_JOINT_GEAR

from .cable_transmission import CableTendonTransmission


def _load_yaml(path: Path) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


class AntagonisticCableJoint:
    """One antagonistic joint (e.g. jnt2): ``tau_act_cmd`` [N·m] on actuator → ``tau_joint`` [N·m]."""

    def __init__(
        self,
        *,
        gear: float,
        r_motor: float,
        r_joint: float,
        x0_plus: float,
        x0_minus: float,
        T_preload: float,
        T_max: float,
        plus: CableTendonTransmission,
        minus: CableTendonTransmission,
    ) -> None:
        self.gear = float(gear)
        self.r_motor = float(r_motor)
        self.r_joint = float(r_joint)
        self.x0_plus = float(x0_plus)
        self.x0_minus = float(x0_minus)
        self.T_preload = float(T_preload)
        self.T_max = float(T_max)
        self.plus = plus
        self.minus = minus

    def reset(self) -> None:
        self.plus.reset()
        self.minus.reset()

    def transmit(
        self,
        tau_act_cmd: float,
        q_joint: float,
        qdot_joint: float,
        dt: float,
    ) -> tuple[float, Dict[str, Any]]:
        """``tau_act_cmd`` is **actuator-shaft** torque command [N·m] (MuJoCo ``q*_act``)."""
        theta_m = float(q_joint) / self.gear
        theta_m_dot = float(qdot_joint) / self.gear

        x_p = self.r_motor * theta_m - self.r_joint * float(q_joint) - self.x0_plus
        x_m = self.r_motor * theta_m + self.r_joint * float(q_joint) - self.x0_minus
        xdot_p = self.r_motor * theta_m_dot - self.r_joint * float(qdot_joint)
        xdot_m = self.r_motor * theta_m_dot + self.r_joint * float(qdot_joint)

        rj = max(self.r_joint, 1e-9)
        T_plus_cmd = self.T_preload + max(float(tau_act_cmd), 0.0) / rj
        T_minus_cmd = self.T_preload + max(-float(tau_act_cmd), 0.0) / rj

        Tp_out, d_p = self.plus.transmit(T_plus_cmd, x_p, xdot_p, dt)
        Tm_out, d_m = self.minus.transmit(T_minus_cmd, x_m, xdot_m, dt)

        tau_joint = self.r_joint * (Tp_out - Tm_out)
        tau_ideal_joint = float(tau_act_cmd) * self.gear

        diag: Dict[str, Any] = {
            "tau_motor_cmd": float(tau_act_cmd),
            "T_plus_cmd": float(T_plus_cmd),
            "T_minus_cmd": float(T_minus_cmd),
            "T_plus_out": float(Tp_out),
            "T_minus_out": float(Tm_out),
            "tau_joint_transmitted": float(tau_joint),
            "tau_ideal_joint_no_cable": float(tau_ideal_joint),
            "torque_loss_joint": float(tau_ideal_joint - tau_joint),
            "x_plus": float(x_p),
            "x_minus": float(x_m),
            "xdot_plus": float(xdot_p),
            "xdot_minus": float(xdot_m),
            "z_plus": float(d_p["z"]),
            "z_minus": float(d_m["z"]),
            "plus": d_p,
            "minus": d_m,
        }
        return float(tau_joint), diag


class AntagonisticCableStack:
    """Three joints (jnt2–jnt4)."""

    def __init__(self, joints: List[AntagonisticCableJoint]) -> None:
        self._joints = joints

    def reset(self) -> None:
        for j in self._joints:
            j.reset()

    def transmit(
        self,
        tau_act_cmd: np.ndarray,
        q_joint234: np.ndarray,
        qdot_joint234: np.ndarray,
        dt: float,
    ) -> tuple[np.ndarray, Dict[str, Any]]:
        """``tau_act_cmd`` shape (3,) actuator torques for axes 2–4."""
        out = np.zeros(3, dtype=np.float64)
        diags: List[Dict[str, Any]] = []
        for i in range(3):
            tj, d = self._joints[i].transmit(
                float(tau_act_cmd[i]),
                float(q_joint234[i]),
                float(qdot_joint234[i]),
                float(dt),
            )
            out[i] = tj
            diags.append(d)
        return out, {
            "per_joint": diags,
            "tau_joint234": out.copy(),
        }

    def z_states(self) -> np.ndarray:
        """Plus-cable hysteresis ``z`` per joint (length 3; matches 이전 obs 확장)."""
        return np.array([j.plus.hysteresis_z for j in self._joints], dtype=np.float64)


def build_antagonistic_stack_from_config(cfg: Dict[str, Any]) -> AntagonisticCableStack:
    """Build from ``cable_params.yaml`` (``antagonistic`` 블록; 없으면 기본값)."""
    n = int(cfg["num_joints"])
    if n != 3:
        raise ValueError("num_joints must be 3 for jnt2–jnt4 stack")

    ant = cfg.setdefault("antagonistic", {})
    T_preload = float(ant.get("T_preload", 2.0))
    r_motor = np.asarray(ant.get("r_motor", cfg["pulley_radius"]), dtype=float).reshape(3)
    r_joint = np.asarray(ant.get("r_joint", cfg["pulley_radius"]), dtype=float).reshape(3)
    x0p = np.asarray(ant.get("x0_plus", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
    x0m = np.asarray(ant.get("x0_minus", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
    T_max = float(ant.get("T_max", cfg.get("max_cable_force", 500.0)))
    tension_deadzone = np.asarray(
        ant.get("tension_deadzone_N", [0.0, 0.0, 0.0]), dtype=float
    ).reshape(3)
    k_stretch = np.asarray(
        ant.get("k_stretch_N_per_m", cfg["stiffness"]), dtype=float
    ).reshape(3)
    tau_fc = np.asarray(
        ant.get("tau_first_order_s", [0.0, 0.0, 0.0]), dtype=float
    ).reshape(3)

    k = np.asarray(cfg["stiffness"], dtype=float).reshape(3)
    d = np.asarray(cfg["damping"], dtype=float).reshape(3)
    Fc = np.asarray(cfg["coulomb"], dtype=float).reshape(3)
    Fv = np.asarray(cfg["viscous"], dtype=float).reshape(3)
    v_eps = np.asarray(cfg["v_eps"], dtype=float).reshape(3)
    bw = cfg["bouc_wen"]
    max_h = float(cfg.get("max_hysteresis_force", 80.0))
    bl_w = np.asarray(cfg["backlash_width"], dtype=float).reshape(3)
    bl_sr = ant.get("stretch_backlash_slack_rate", 40.0)
    if np.isscalar(bl_sr):
        bl_sr_arr = np.full(3, float(bl_sr), dtype=float)
    else:
        bl_sr_arr = np.asarray(bl_sr, dtype=float).reshape(3)

    gears = MOTOR_TO_JOINT_GEAR[1:4]

    def make_tendon(ii: int) -> CableTendonTransmission:
        return CableTendonTransmission(
            Fc=float(Fc[ii]),
            Fv=float(Fv[ii]),
            v_eps=float(v_eps[ii]),
            T_max=T_max,
            tension_deadzone=float(tension_deadzone[ii]),
            k_stretch=float(k_stretch[ii]),
            bouc_wen_alpha=float(np.asarray(bw["alpha"], dtype=float).reshape(3)[ii]),
            bouc_wen_A=float(np.asarray(bw["A"], dtype=float).reshape(3)[ii]),
            bouc_wen_beta=float(np.asarray(bw["beta"], dtype=float).reshape(3)[ii]),
            bouc_wen_gamma=float(np.asarray(bw["gamma"], dtype=float).reshape(3)[ii]),
            bouc_wen_n=float(np.asarray(bw["n"], dtype=float).reshape(3)[ii]),
            bouc_wen_z_clip=float(np.asarray(bw["z_clip"], dtype=float).reshape(3)[ii]),
            max_hysteresis_force=max_h,
            tau_first_order=float(tau_fc[ii]),
            stretch_backlash_width_m=float(bl_w[ii]),
            stretch_backlash_slack_rate=float(bl_sr_arr[ii]),
        )

    joints: List[AntagonisticCableJoint] = []
    for i in range(3):
        joints.append(
            AntagonisticCableJoint(
                gear=float(gears[i]),
                r_motor=float(r_motor[i]),
                r_joint=float(r_joint[i]),
                x0_plus=float(x0p[i]),
                x0_minus=float(x0m[i]),
                T_preload=T_preload,
                T_max=T_max,
                plus=make_tendon(i),
                minus=make_tendon(i),
            )
        )
    return AntagonisticCableStack(joints)


def build_antagonistic_stack_from_yaml(path: Path, *, randomize: bool = False) -> AntagonisticCableStack:
    cfg = _load_yaml(path)
    if not randomize:
        cfg.setdefault("randomization", {})["enabled"] = False
    return build_antagonistic_stack_from_config(cfg)
