"""Task-space VSD: ``F = K e + D e_dot``, ``tau_total = tau_bias + J.T @ F`` (+ limits)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from controllers.vsd_joint_controller import actuator_torques_from_joint
from kinematics.orientation_utils import angle_error
from kinematics.task_jacobian import TaskJacobianMode, task_dim

TaskMode = TaskJacobianMode


def apply_joint_limit_torque_gate(
    tau: np.ndarray,
    q_joint: np.ndarray,
    *,
    q_min: np.ndarray,
    q_max: np.ndarray,
    margin: float,
    enabled: bool,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    상단 근처: ``tau_i = min(tau_i, 0)``, 하단 근처: ``tau_i = max(tau_i, 0)``.

    Returns ``(tau_out, joint_limit_active(4,), clip_delta_before_minus_after)``
    clip_delta は gate 직전 토크 대비 줄어든 양 (active 축만).
    """
    tau2 = np.asarray(tau, dtype=np.float64).reshape(4).copy()
    if not enabled:
        z = np.zeros(4, dtype=bool)
        return tau2, z, np.zeros(4, dtype=np.float64)

    q = np.asarray(q_joint, dtype=np.float64).reshape(4)
    lo = np.asarray(q_min, dtype=np.float64).reshape(4)
    hi = np.asarray(q_max, dtype=np.float64).reshape(4)
    m = float(margin)

    active = np.zeros(4, dtype=bool)
    delta = np.zeros(4, dtype=np.float64)

    for i in range(4):
        tau_before = float(tau2[i])
        if q[i] > hi[i] - m:
            tau2[i] = min(tau2[i], 0.0)
            if tau_before > tau2[i]:
                active[i] = True
                delta[i] = tau_before - float(tau2[i])
        elif q[i] < lo[i] + m:
            tau2[i] = max(tau2[i], 0.0)
            if tau_before < tau2[i]:
                active[i] = True
                delta[i] = tau_before - float(tau2[i])

    return tau2, active, delta


@dataclass
class TaskSpaceVSDParams:
    """작업 공간 VSD 파라미터 (과제 차원은 ``task_mode`` 에 따름)."""

    task_mode: TaskMode
    K_task: np.ndarray
    D_task: np.ndarray
    F_task_limit: np.ndarray
    tau_jnt_limit: np.ndarray
    tau_act_limit: np.ndarray
    jac_flip_sign: float = 1.0
    use_bias_compensation: bool = True
    use_desired_velocity: bool = True
    joint_limit_enabled: bool = True
    joint_limit_margin: float = 0.05
    q_joint_min: np.ndarray | None = None
    q_joint_max: np.ndarray | None = None


class TaskSpaceVSDController:
    """가상 스프링-댐퍼 + (선택) bias 보상 + 관절 한계 토크 게이트."""

    def __init__(self, params: TaskSpaceVSDParams) -> None:
        self.p = params
        m = task_dim(params.task_mode)
        self._dim = int(m)
        assert np.asarray(params.K_task).size == self._dim
        assert np.asarray(params.D_task).size == self._dim
        assert np.asarray(params.F_task_limit).size == self._dim

    def compute(
        self,
        *,
        y_des: np.ndarray,
        ydot_des: np.ndarray,
        y_actual: np.ndarray,
        qdot_joint: np.ndarray,
        q_joint: np.ndarray,
        J_task: np.ndarray,
        ratios_actuator: np.ndarray,
        tau_bias_joint: np.ndarray | None = None,
    ) -> dict[str, Any]:
        sg = float(self.p.jac_flip_sign)
        if not (np.isclose(sg, 1.0) or np.isclose(sg, -1.0)):
            raise ValueError("jac_flip_sign must be +1 or -1")

        m = self._dim
        Jh = sg * np.asarray(J_task, dtype=np.float64).reshape(m, 4)
        qdj = np.asarray(qdot_joint, dtype=np.float64).reshape(4)
        qj = np.asarray(q_joint, dtype=np.float64).reshape(4)

        K = np.asarray(self.p.K_task, dtype=np.float64).reshape(m)
        De = np.asarray(self.p.D_task, dtype=np.float64).reshape(m)
        rl = np.asarray(self.p.F_task_limit, dtype=np.float64).reshape(m)

        yd = np.asarray(y_des, dtype=np.float64).reshape(m)
        ya = np.asarray(y_actual, dtype=np.float64).reshape(m)
        dy = np.asarray(ydot_des, dtype=np.float64).reshape(m)

        e = np.zeros(m, dtype=np.float64)
        edot = np.zeros(m, dtype=np.float64)

        if self.p.task_mode == "xyz":
            e[:] = yd - ya
        elif self.p.task_mode == "xyz_pitch":
            e[0:3] = yd[0:3] - ya[0:3]
            e[3] = angle_error(float(yd[3]), float(ya[3]))
        else:
            e[0:3] = yd[0:3] - ya[0:3]
            e[3] = angle_error(float(yd[3]), float(ya[3]))
            e[4] = angle_error(float(yd[4]), float(ya[4]))

        ydot_actual = Jh @ qdj
        dy_used = dy if bool(self.p.use_desired_velocity) else np.zeros(m, dtype=np.float64)
        edot = dy_used - ydot_actual

        F_unc = K * e + De * edot
        F_raw = np.array(F_unc, copy=True)
        rl_nf = np.where(np.isfinite(rl), rl, np.inf)
        F = np.clip(np.asarray(F_unc, dtype=np.float64), -rl_nf, rl_nf)
        sf = np.abs(F_raw - F) > 1e-9

        tau_task = (Jh.T @ F).reshape(4)

        if self.p.use_bias_compensation:
            if tau_bias_joint is None:
                raise ValueError("use_bias_compensation True but tau_bias_joint is None")
            tau_bias = np.asarray(tau_bias_joint, dtype=np.float64).reshape(4).copy()
        else:
            tau_bias = np.zeros(4, dtype=np.float64)

        tau_total_unc = tau_bias + tau_task

        q_min = np.asarray(self.p.q_joint_min, dtype=np.float64).reshape(4) if self.p.q_joint_min is not None else None
        q_max = np.asarray(self.p.q_joint_max, dtype=np.float64).reshape(4) if self.p.q_joint_max is not None else None
        if self.p.joint_limit_enabled and (q_min is None or q_max is None):
            raise ValueError("joint_limit_enabled requires q_joint_min and q_joint_max in params")

        gate_lo = np.full(4, -np.inf) if q_min is None else q_min
        gate_hi = np.full(4, np.inf) if q_max is None else q_max

        tau_after_gate, jl_active, jl_clip = apply_joint_limit_torque_gate(
            tau_total_unc,
            qj,
            q_min=gate_lo,
            q_max=gate_hi,
            margin=float(self.p.joint_limit_margin),
            enabled=bool(self.p.joint_limit_enabled),
        )

        tlim = np.asarray(self.p.tau_jnt_limit, dtype=np.float64).reshape(4)
        tau_unc_clip = tau_after_gate.copy()
        tau_j = np.clip(tau_unc_clip, -tlim, tlim)
        sj = np.abs(tau_unc_clip - tau_j) > 1e-9

        ratios = np.asarray(ratios_actuator, dtype=np.float64).reshape(4)
        tau_act_unc = actuator_torques_from_joint(tau_j, ratios)
        tac_raw = tau_act_unc.copy()
        ta_lim = np.asarray(self.p.tau_act_limit, dtype=np.float64).reshape(4)
        tau_act = np.clip(tau_act_unc, -ta_lim, ta_lim)
        sa = np.abs(tau_act_unc - tau_act) > 1e-9

        return {
            "e_task": e,
            "edot_task": edot,
            "F_raw": F_raw,
            "F_task": F,
            "F_saturated_axes": sf,
            "ydot_actual": ydot_actual,
            "tau_bias_jnt": tau_bias.copy(),
            "tau_task_jnt": tau_task.copy(),
            "tau_total_unc": tau_total_unc.copy(),
            "tau_after_joint_limit_gate": tau_after_gate.copy(),
            "joint_limit_active": jl_active.copy(),
            "joint_limit_clipped_tau": jl_clip.copy(),
            "tau_jnt_raw": tau_after_gate.copy(),
            "tau_joint": tau_j,
            "tau_joint_saturated_axes": sj,
            "tau_act_raw": tac_raw,
            "tau_act": tau_act,
            "tau_act_saturated_axes": sa,
            "tau_act_mapping_check": actuator_torques_from_joint(tau_j, ratios),
            "ydot_des_effective": dy_used.copy(),
        }


__all__ = [
    "TaskSpaceVSDController",
    "TaskSpaceVSDParams",
    "TaskMode",
    "apply_joint_limit_torque_gate",
]
