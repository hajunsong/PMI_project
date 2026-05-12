"""Combines belt (q1) and cable (q2–q4) actuator torque paths for the hybrid arm."""

from __future__ import annotations

from typing import Any

import numpy as np

from .belt_transmission import BeltTransmission
from .cable_transmission import CableTransmission, CableTransmitResult, cable_transmission_identity
from .randomization import CableParameterRandomizer, build_cable_transmission_from_full_config


class HybridTransmission:
    """
    Joint groups:
        - Belt: jnt1 / q1_act → :meth:`BeltTransmission.transmit` (no cable delay / loss)
        - Cable: jnt2–jnt4 → :meth:`CableTransmission.transmit` (delay + optional friction)
    """

    def __init__(
        self,
        cable: CableTransmission | None = None,
        *,
        cable_config: dict[str, Any] | None = None,
    ):
        self._cable = cable if cable is not None else cable_transmission_identity()
        self._cable_config = cable_config
        self._last_cable: CableTransmitResult | None = None
        self._sampled_flat: dict[str, float] | None = None
        self._last_reset_seed: int | None = None
        self._last_randomization_profile: str | None = None

    @property
    def cable(self) -> CableTransmission:
        return self._cable

    @property
    def last_cable_result(self) -> CableTransmitResult | None:
        """Diagnostics from the most recent cable ``transmit`` (q2–q4)."""
        return self._last_cable

    def reset_cable_state(self) -> None:
        self.reset(randomize=False)

    def reset(self, randomize: bool = False, seed: int | None = None, profile_name: str | None = None) -> None:
        """
        Reset cable internal state. If ``randomize=True`` and ``cable_config`` was passed at
        construction, resample q2–q4 cable parameters (Belt q1 remains ideal).

        ``profile_name`` selects ``randomization_profiles[profile]['ranges']`` when present;
        otherwise ``active_profile`` from yaml; otherwise legacy ``randomization.ranges``.
        """
        self._cable.reset()
        self._last_cable = None
        if not randomize:
            return
        if self._cable_config is None:
            raise ValueError("HybridTransmission.reset(randomize=True) requires cable_config=... in __init__")
        rnd_block = self._cable_config.get("randomization")
        if rnd_block is None:
            raise ValueError("cable_config must contain 'randomization' for randomize=True")
        rz = CableParameterRandomizer(
            rnd_block,
            profile_name=profile_name,
            profiles=self._cable_config.get("randomization_profiles"),
            active_profile=self._cable_config.get("active_profile"),
        )
        sample = rz.sample(seed=seed)
        self._cable = build_cable_transmission_from_full_config(self._cable_config, sample)
        self._sampled_flat = rz.sampled_params_to_flat_dict(sample)
        self._last_reset_seed = rz.last_seed
        self._last_randomization_profile = (
            profile_name
            if profile_name is not None
            else (self._cable_config.get("active_profile") or rz.resolved_profile_label)
        )

    @property
    def last_randomization_profile(self) -> str | None:
        """Profile key used by the last ``reset(randomize=True, ...)``, if any."""

        return self._last_randomization_profile

    def get_current_params(self) -> dict[str, float]:
        """Last sampled cable parameters (after ``reset(randomize=True)``), logging-friendly."""
        if not self._sampled_flat:
            return {}
        out = dict(self._sampled_flat)
        if self._last_reset_seed is not None:
            out["sample_seed"] = float(self._last_reset_seed)
        return out

    def transmit(
        self,
        tau_act_ideal: np.ndarray,
        dt: float,
        q_act: np.ndarray,
        qdot_act: np.ndarray,
    ) -> np.ndarray:
        t = np.asarray(tau_act_ideal, dtype=np.float64).reshape(4)
        qa = np.asarray(q_act, dtype=np.float64).reshape(4)
        qd = np.asarray(qdot_act, dtype=np.float64).reshape(4)
        out = np.empty(4, dtype=np.float64)
        out[0] = BeltTransmission.transmit(float(t[0]))
        res = self._cable.transmit(t[1:4], float(dt), qa[1:4], qd[1:4])
        self._last_cable = res
        out[1:4] = res.tau_out
        return out
