"""
PMI 4R arm FK / task Jacobian — numeric port of ``pmi_kinematics`` + ``PathPlanner`` / ControlMain.

Same constants as ``pmi_kinematics/src/pmi_kinematics.cpp`` and ``PMI_Server/src/tcp_server.cpp`` ``kGear``.
"""

from __future__ import annotations

import numpy as np

# PMI_Server tcp_server.cpp / pmi_kinematics kMotorToJointGear  (q_jnt = gear * q_act)
MOTOR_TO_JOINT_GEAR = np.array(
    [32.0 / 60.0, 54.0 / 360.0, 108.0 / 360.0, 108.0 / 360.0], dtype=np.float64
)

# PathPlanner / server IK: jnt1 ∈ [-π,π], jnt2–4 ∈ [-π/2, π/2]
_J1_MIN, _J1_MAX = -np.pi, np.pi
_J234_MIN, _J234_MAX = -np.pi / 2.0, np.pi / 2.0
JOINT_LIMIT_RAD_MIN = np.array([_J1_MIN, _J234_MIN, _J234_MIN, _J234_MIN], dtype=np.float64)
JOINT_LIMIT_RAD_MAX = np.array([_J1_MAX, _J234_MAX, _J234_MAX, _J234_MAX], dtype=np.float64)
# q_act = q_jnt / gear  (all gears > 0 → same min/max ordering)
ACTUATOR_LIMIT_RAD_MIN = (JOINT_LIMIT_RAD_MIN / MOTOR_TO_JOINT_GEAR).astype(np.float64)
ACTUATOR_LIMIT_RAD_MAX = (JOINT_LIMIT_RAD_MAX / MOTOR_TO_JOINT_GEAR).astype(np.float64)


def joint_rad_from_actuator_rad(q_act: np.ndarray) -> np.ndarray:
    return (MOTOR_TO_JOINT_GEAR * np.asarray(q_act, dtype=np.float64)).reshape(4)


def actuator_rad_from_joint_rad(q_joint: np.ndarray) -> np.ndarray:
    return (np.asarray(q_joint, dtype=np.float64) / MOTOR_TO_JOINT_GEAR).reshape(4)


def actuator_torque_from_joint_torque(tau_joint: np.ndarray) -> np.ndarray:
    """
    등식 ``q_jnt = MOTOR_TO_JOINT_GEAR ⊙ q_act`` (원소별) 일 때 가상일 원리로

    ``τ_act = τ_joint ⊙ MOTOR_TO_JOINT_GEAR`` [N·m].

    MuJoCo ``motor`` 를 ``q*_act`` 에 걸고 ``ctrl = τ_act`` 로 두면 관절 쪽 일반화력은
    제약을 통해 ``τ_joint`` 와 정합된다.
    """
    tj = np.asarray(tau_joint, dtype=np.float64).reshape(4)
    return (tj * MOTOR_TO_JOINT_GEAR).reshape(4)


def joint_torque_from_actuator_torque(tau_act: np.ndarray) -> np.ndarray:
    """``τ_joint = τ_act / MOTOR_TO_JOINT_GEAR`` (원소별)."""
    ta = np.asarray(tau_act, dtype=np.float64).reshape(4)
    return (ta / MOTOR_TO_JOINT_GEAR).reshape(4)


def _euler_zxz(phi: float, theta: float, psi: float) -> np.ndarray:
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(theta), np.sin(theta)
    cpsi, spsi = np.cos(psi), np.sin(psi)
    rot_phi = np.array([[cphi, -sphi, 0.0], [sphi, cphi, 0.0], [0.0, 0.0, 1.0]])
    rot_theta = np.array([[1.0, 0.0, 0.0], [0.0, cth, -sth], [0.0, sth, cth]])
    rot_psi = np.array([[cpsi, -spsi, 0.0], [spsi, cpsi, 0.0], [0.0, 0.0, 1.0]])
    return rot_phi @ rot_theta @ rot_psi


def _rot_z(q: float) -> np.ndarray:
    c, s = np.cos(q), np.sin(q)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rot_z_deriv(q: float) -> np.ndarray:
    s, c = np.sin(q), np.cos(q)
    return np.array([[-s, -c, 0.0], [c, -s, 0.0], [0.0, 0.0, 0.0]])


def mat2rpy(A: np.ndarray) -> np.ndarray:
    roll = np.arctan2(A[2, 1], A[2, 2])
    pitch = np.arctan2(-A[2, 0], np.sqrt(A[0, 0] ** 2 + A[1, 0] ** 2))
    yaw = np.arctan2(A[1, 0], A[0, 0])
    return np.array([roll, pitch, yaw], dtype=np.float64)


def _droll_dA(A: np.ndarray, eps: float = 1e-15) -> np.ndarray:
    y, x = A[2, 1], A[2, 2]
    den = y * y + x * x + eps
    G = np.zeros((3, 3))
    G[2, 1] = x / den
    G[2, 2] = -y / den
    return G


def _dpitch_dA(A: np.ndarray, eps: float = 1e-15) -> np.ndarray:
    y = -A[2, 0]
    c = np.sqrt(A[0, 0] ** 2 + A[1, 0] ** 2 + eps)
    den = y * y + c * c + eps
    G = np.zeros((3, 3))
    G[2, 0] = -c / den
    if c > eps:
        fac = -y / den / c
        G[0, 0] = fac * A[0, 0]
        G[1, 0] = fac * A[1, 0]
    return G


def _roll_pitch_jacobian_wrt_q(
    A: np.ndarray, dA_dq: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray], eps: float = 1e-5
) -> np.ndarray:
    Gr = _droll_dA(A, eps)
    Gp = _dpitch_dA(A, eps)
    J = np.zeros((2, 4))
    for k in range(4):
        J[0, k] = float(np.sum(Gr * dA_dq[k]))
        J[1, k] = float(np.sum(Gp * dA_dq[k]))
    return J


def forward_kinematics_chain(
    q_joint_rad: np.ndarray,
) -> tuple[
    list[np.ndarray],
    list[np.ndarray],
    list[np.ndarray],
    list[np.ndarray],
    np.ndarray,
    np.ndarray,
]:
    """Returns Ai[4], ri[4], Hi[4], sij[4], ee_position (3,), ee_orientation_Ae (3,3)."""
    q = np.asarray(q_joint_rad, dtype=np.float64).reshape(4)
    sijp = [
        np.zeros(3),
        np.array([0.0, 0.0, -0.22]),
        np.array([0.0, -0.23, 0.0]),
        np.array([0.23, 0.0, 0.0]),
    ]
    Cij = [
        _euler_zxz(0.0, np.pi, 0.0),
        _euler_zxz(0.0, np.pi / 2, 0.0),
        _euler_zxz(-np.pi / 2, 0.0, 0.0),
        _euler_zxz(0.0, np.pi, 0.0),
    ]
    sep = np.array([0.18, 0.0, 0.0])
    Ce = _euler_zxz(-np.pi / 2, 0.0, 0.0)
    u_vec = np.array([0.0, 0.0, 1.0])

    Ai: list[np.ndarray] = []
    ri: list[np.ndarray] = []
    Hi: list[np.ndarray] = []
    sij: list[np.ndarray] = []

    prevAi = np.eye(3)
    prevRi = np.zeros(3)
    for i in range(4):
        Aijpp = _rot_z(float(q[i]))
        Ai_i = prevAi @ Cij[i] @ Aijpp
        sij_i = prevAi @ sijp[i]
        ri_i = prevRi + sij_i
        Hi_i = prevAi @ Cij[i] @ u_vec
        Ai.append(Ai_i)
        ri.append(ri_i)
        Hi.append(Hi_i)
        sij.append(sij_i)
        prevAi = Ai_i
        prevRi = ri_i

    se = Ai[3] @ sep
    ee_position = ri[3] + se
    ee_orientation_Ae = Ai[3] @ Ce
    return Ai, ri, Hi, sij, ee_position, ee_orientation_Ae


def fk_ee_pose_joint_rad(q_joint_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    _, ri, Hi, _, ee_pos, Ae = forward_kinematics_chain(q_joint_rad)
    _ = ri, Hi  # unused names for symmetry with C++ API
    rpy = mat2rpy(Ae)
    return ee_pos, rpy


def jacobian_5x4_joint_rad(q_joint_rad: np.ndarray) -> np.ndarray:
    """5×4 task Jacobian (xyz + roll + pitch), same as ``pmi::jacobian_5x4_joint_rad``."""
    q = np.asarray(q_joint_rad, dtype=np.float64).reshape(4)
    Cij = [
        _euler_zxz(0.0, np.pi, 0.0),
        _euler_zxz(0.0, np.pi / 2, 0.0),
        _euler_zxz(-np.pi / 2, 0.0, 0.0),
        _euler_zxz(0.0, np.pi, 0.0),
    ]
    Ce = _euler_zxz(-np.pi / 2, 0.0, 0.0)
    Ai, ri, Hi, _, re, Ae = forward_kinematics_chain(q)

    Rz = [_rot_z(float(q[i])) for i in range(4)]
    Rzdot = [_rot_z_deriv(float(q[i])) for i in range(4)]

    A1_q1 = Cij[0] @ Rzdot[0]
    A2_q1 = A1_q1 @ Cij[1] @ Rz[1]
    A3_q1 = A2_q1 @ Cij[2] @ Rz[2]
    A4_q1 = A3_q1 @ Cij[3] @ Rz[3]

    A2_q2 = Ai[0] @ Cij[1] @ Rzdot[1]
    A3_q2 = A2_q2 @ Cij[2] @ Rz[2]
    A4_q2 = A3_q2 @ Cij[3] @ Rz[3]

    A3_q3 = Ai[1] @ Cij[2] @ Rzdot[2]
    A4_q3 = A3_q3 @ Cij[3] @ Rz[3]

    A4_q4 = Ai[2] @ Cij[3] @ Rzdot[3]

    Ae_q1 = A4_q1 @ Ce
    Ae_q2 = A4_q2 @ Ce
    Ae_q3 = A4_q3 @ Ce
    Ae_q4 = A4_q4 @ Ce

    jac_pos = np.zeros((3, 4))
    for k in range(4):
        jac_pos[:, k] = np.cross(Hi[k], re - ri[k])

    dAe_dq = (Ae_q1, Ae_q2, Ae_q3, Ae_q4)
    jac_rp = _roll_pitch_jacobian_wrt_q(Ae, dAe_dq)

    J = np.zeros((5, 4))
    J[:3, :] = jac_pos
    J[3:, :] = jac_rp
    return J


def wrap_to_pi(angle: float) -> float:
    wrapped = (angle + np.pi) % (2.0 * np.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * np.pi
    return float(wrapped - np.pi)
