"""회전·왜곡 등 공통 수학 유틸."""

from __future__ import annotations

import numpy as np


def wrap_to_pi(angle: float | np.ndarray) -> float | np.ndarray:
    """각도를 ``[-π, π)`` 로 맞춘다 (스칼라·배열 요소별)."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def skew(x):
    return np.array(
        [
            [0, -x[2], x[1]],
            [x[2], 0, -x[0]],
            [-x[1], x[0], 0],
        ]
    )


def euler_zxz(phi, theta, psi):
    return np.array(
        [
            [
                np.cos(phi) * np.cos(psi)
                - np.sin(phi) * np.sin(psi) * np.cos(theta),
                -np.cos(phi) * np.sin(psi)
                - np.sin(phi) * np.cos(psi) * np.cos(theta),
                np.sin(phi) * np.sin(theta),
            ],
            [
                np.sin(phi) * np.cos(psi)
                + np.cos(phi) * np.sin(psi) * np.cos(theta),
                -np.sin(phi) * np.sin(psi)
                + np.cos(phi) * np.cos(psi) * np.cos(theta),
                -np.cos(phi) * np.sin(theta),
            ],
            [
                np.sin(psi) * np.sin(theta),
                np.cos(psi) * np.sin(theta),
                np.cos(theta),
            ],
        ]
    )


def mat2rpy(A):
    return np.array(
        [
            np.arctan2(A[2, 1], A[2, 2]),
            np.arctan2(-A[2, 0], np.sqrt(A[0, 0] ** 2 + A[1, 0] ** 2)),
            np.arctan2(A[1, 0], A[0, 0]),
        ]
    )


def droll_dA(A, eps: float = 1e-15):
    """``mat2rpy`` 와 동일한 roll = arctan2(A[2,1], A[2,2]) 의 ∂roll/∂A (3×3)."""
    A = np.asarray(A, dtype=float)
    y, x = A[2, 1], A[2, 2]
    den = y * y + x * x + eps
    G = np.zeros((3, 3), dtype=float)
    G[2, 1] = x / den
    G[2, 2] = -y / den
    return G


def dpitch_dA(A, eps: float = 1e-15):
    """``mat2rpy`` 와 동일한 pitch 정의에 대한 ∂pitch/∂A (3×3).

    pitch = arctan2(-A[2,0], c), c = sqrt(A[0,0]^2 + A[1,0]^2).
    c≈0 (짐벌) 근처에서는 ``eps`` 로 분모를 안정화한다.
    """
    A = np.asarray(A, dtype=float)
    y = -A[2, 0]
    c = np.sqrt(A[0, 0] ** 2 + A[1, 0] ** 2 + eps)
    den = y * y + c * c + eps
    G = np.zeros((3, 3), dtype=float)
    G[2, 0] = -c / den
    if c > eps:
        fac = -y / den / c
        G[0, 0] = fac * A[0, 0]
        G[1, 0] = fac * A[1, 0]
    return G


def roll_pitch_jacobian_wrt_q(A, dA_dq, eps: float = 1e-15):
    """∂(roll, pitch)/∂q (2×nq). ``dA_dq[:,:,k]`` = ∂A/∂q_k.

    ∂angle/∂q_k = sum_ij (∂angle/∂A_ij)(∂A_ij/∂q_k) = ⟨G, ∂A/∂q_k⟩.
    """
    A = np.asarray(A, dtype=float)
    dA_dq = np.asarray(dA_dq, dtype=float)
    if dA_dq.ndim != 3 or dA_dq.shape[:2] != (3, 3):
        raise ValueError("dA_dq must have shape (3, 3, nq)")
    nq = dA_dq.shape[2]
    Gr = droll_dA(A, eps=eps)
    Gp = dpitch_dA(A, eps=eps)
    J = np.zeros((2, nq), dtype=float)
    for k in range(nq):
        dAk = dA_dq[:, :, k]
        J[0, k] = float(np.sum(Gr * dAk))
        J[1, k] = float(np.sum(Gp * dAk))
    return J
