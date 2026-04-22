#!/usr/bin/env python3

from __future__ import annotations

import numpy as np

__all__ = ["path_generation"]


def path_generation(
    x0: float,
    xf: float,
    tf: float,
    ta: float,
    h: float,
    full_quintic: bool = False,
) -> np.ndarray:
    """
    각 샘플마다 ``[위치, 속도, 가속도]`` 인 ``(N, 3)`` 배열을 반환한다.

    - 열 0: ``x(t)``, 열 1: ``dx/dt``, 열 2: ``d²x/dt²`` (5차 다항의 해석 미분).
    - ``path_generation(...)[:-1]`` 로 슬라이스하면 세 양이 **같은 행** 단위로 잘린다.
    - 구간 끝 시각 ``tf`` 샘플은 **포함**된다(``round(tf/h)+1`` 점).

    ``full_quintic=False`` (기본): MATLAB 트래페조이드 — 가속·등속·감속 3구간 5차 연결.
    ``full_quintic=True``: ``x0``→``xf`` 를 시간 ``tf`` 안에 **한 번의** 5차 다항(휴지–휴지).
    """
    if full_quintic:
        return quintic_path(x0, xf, 0.0, 0.0, 0.0, 0.0, tf, h)

    td = tf - ta
    vd = (xf - x0) / td
    xa = x0 + 0.5 * ta * vd
    xd = xf - 0.5 * ta * vd

    pv1 = quintic_path(x0, xa, 0.0, vd, 0.0, 0.0, ta, h)
    pv2 = quintic_path(xa, xd, vd, vd, 0.0, 0.0, td - ta, h)
    pv3 = quintic_path(xd, xf, vd, 0.0, 0.0, 0.0, tf - td, h)
    return np.concatenate((pv1, pv2, pv3), axis=0)


def quintic_path(
    pos0: float,
    posf: float,
    vel0: float,
    velf: float,
    acc0: float,
    accf: float,
    ts: float,
    h: float,
) -> np.ndarray:
    """한 구간: ``ts<=0`` 이면 ``(0, 3)``.

    샘플 시각은 ``0`` 부터 ``ts`` **끝점 포함**이며, 간격 수는 ``round(ts/h)`` 로 두어
    ``h`` 로 나누어떨어지는 구간(예: ``ts=0.5``, ``h=0.001`` → 501점)에서 RecurDyn과 같은
    시간 격자 길이와 맞추기 쉽다. (``np.arange(0, ts, h)`` 는 끝 ``ts`` 가 빠져 한 점 부족해짐.)
    """
    if ts <= 0:
        return np.zeros((0, 3), dtype=float)

    a0, a1, a2, a3, a4, a5 = quintic_coeffs(pos0, posf, vel0, velf, acc0, accf, ts)
    n_interval = max(1, int(round(ts / float(h))))
    t_samples = np.linspace(0.0, ts, n_interval + 1, dtype=float)
    pos_list: list[float] = []
    vel_list: list[float] = []
    acc_list: list[float] = []
    for t in t_samples:
        pos_list.append(a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5)
        vel_list.append(a1 + 2 * a2 * t + 3 * a3 * t**2 + 4 * a4 * t**3 + 5 * a5 * t**4)
        acc_list.append(2 * a2 + 6 * a3 * t + 12 * a4 * t**2 + 20 * a5 * t**3)
    pos = np.asarray(pos_list, dtype=float)
    vel = np.asarray(vel_list, dtype=float)
    acc = np.asarray(acc_list, dtype=float)
    return np.column_stack((pos, vel, acc))


def quintic_coeffs(
    pos0: float,
    posf: float,
    vel0: float,
    velf: float,
    acc0: float,
    accf: float,
    ts: float,
) -> tuple[float, float, float, float, float, float]:
    """5차 경로 다항식 계수 ``a0..a5`` (위치 기준)."""
    a0 = pos0
    a1 = vel0
    a2 = acc0 / 2.0
    a3 = (20 * (posf - pos0) - (8 * velf + 12 * vel0) * ts - (3 * acc0 - accf) * ts**2) / (
        2 * ts**3
    )
    a4 = (30 * (pos0 - posf) + (14 * velf + 16 * vel0) * ts + (3 * acc0 - 2 * accf) * ts**2) / (
        2 * ts**4
    )
    a5 = (12 * (posf - pos0) - (6 * velf + 6 * vel0) * ts - (acc0 - accf) * ts**2) / (
        2 * ts**5
    )

    return a0, a1, a2, a3, a4, a5
