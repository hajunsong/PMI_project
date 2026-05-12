"""One-dimensional backlash (play) on a stretch coordinate."""


from dataclasses import dataclass


@dataclass
class BacklashParams:
    """Total slack magnitude on stretch x (similar to symmetric dead band)."""

    width: float


class Backlash1D:
    """
    Piecewise backlash between input stretch x_in and latent output stretch x_out:

        If x_in > x_out + half:  x_out = x_in - half
        If x_in < x_out - half: x_out = x_in + half
        Else: x_out unchanged
    """

    def __init__(self, params: BacklashParams):
        self._half = max(0.5 * float(params.width), 0.0)
        self._x_out = 0.0

    def reset(self, x_initial: float = 0.0) -> None:
        self._x_out = float(x_initial)

    @property
    def x_out(self) -> float:
        return float(self._x_out)

    def step(self, x_in: float) -> tuple[float, bool]:
        """
        Returns (x_filtered, backlash_active_sliding).
        Sliding is considered active whenever the slack region is exploited (input inside band).
        """
        if self._half <= 0.0:
            self._x_out = float(x_in)
            return self._x_out, False

        active = False
        if x_in > self._x_out + self._half:
            self._x_out = x_in - self._half
        elif x_in < self._x_out - self._half:
            self._x_out = x_in + self._half
        else:
            active = True
        return float(self._x_out), active
