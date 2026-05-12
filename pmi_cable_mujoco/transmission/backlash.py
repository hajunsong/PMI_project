"""Dead-zone and backlash for cable joints (jnt2–jnt4)."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class DeadzoneBacklash:
    """Joint-space dead-zone plus backlash slack for cable stretch error."""

    deadzone_width: float
    backlash_width: float
    backlash_slope: float = 1000.0
    slack_rate: float = 40.0

    _slack: float = field(default=0.0, repr=False)
    _prev_sign_qdot: int = field(default=0, repr=False)

    def reset(self) -> None:
        self._slack = 0.0
        self._prev_sign_qdot = 0

    @staticmethod
    def _deadzone_scalar(e: float, width: float) -> float:
        if width <= 0.0:
            return e
        half = width * 0.5
        if abs(e) <= half:
            return 0.0
        return e - np.sign(e) * half

    def effective_tracking_error(self, q_des: float, q: float, qdot: float, dt: float) -> float:
        raw = float(q_des - q)

        sign = 0
        if qdot > 1e-9:
            sign = 1
        elif qdot < -1e-9:
            sign = -1

        if sign != 0:
            self._prev_sign_qdot = sign

        bl = max(self.backlash_width, 0.0)
        limit = bl * 0.5
        if bl > 0.0 and sign != 0:
            target = sign * limit
            max_step = self.slack_rate * dt
            diff = target - self._slack
            step = np.clip(diff, -max_step, max_step)
            self._slack += float(step)
        self._slack = float(np.clip(self._slack, -limit, limit))

        adjusted = raw - self._slack
        return self._deadzone_scalar(adjusted, self.deadzone_width)


@dataclass
class StretchBacklash:
    """케이블 신장 ``x`` [m]에 대한 기하 백래시: 탄성항은 ``x_eff = x - slack`` 을 사용.

    ``DeadzoneBacklash`` 와 동일한 슬랙 적분(``slack_rate``)으로, 신장 변화가 먼저 갭을 메운 뒤
    스프링에 전달되는 양을 모델링한다.
    """

    width_m: float
    slack_rate: float = 40.0

    _slack: float = field(default=0.0, repr=False)
    _prev_sign_xdot: int = field(default=0, repr=False)

    def reset(self) -> None:
        self._slack = 0.0
        self._prev_sign_xdot = 0

    def effective_stretch(self, x: float, xdot: float, dt: float) -> tuple[float, bool]:
        """Return ``(x_elastic, backlash_active)`` for ``k_stretch * x`` 등."""
        b = max(float(self.width_m), 0.0)
        if b < 1e-12:
            return float(x), False

        limit = b * 0.5
        sign = 0
        if xdot > 1e-9:
            sign = 1
        elif xdot < -1e-9:
            sign = -1
        if sign != 0:
            self._prev_sign_xdot = sign

        eff_sign = sign if sign != 0 else self._prev_sign_xdot
        if b > 0.0 and eff_sign != 0:
            target = eff_sign * limit
            max_step = max(float(self.slack_rate), 0.0) * float(dt)
            diff = target - self._slack
            step = float(np.clip(diff, -max_step, max_step))
            self._slack += step
        self._slack = float(np.clip(self._slack, -limit, limit))

        x_eff = float(x) - self._slack
        backlash_active = bool(
            b > 0.0 and abs(float(xdot)) > 1e-9 and abs(self._slack) < limit - 1e-9
        )
        return x_eff, backlash_active
