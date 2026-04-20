"""회전·왜곡 등 공통 수학 유틸."""

import numpy as np


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
