"""Separate randomization for belt (jnt1) and cable (jnt2–jnt4) parameters."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict, Tuple

import numpy as np


def _clip_sample(lo: float, hi: float, rng: np.random.Generator) -> float:
    if lo > hi:
        lo, hi = hi, lo
    return float(rng.uniform(lo, hi))


def _sample_scalar(rnd: Dict[str, Any], key: str, default: Tuple[float, float], rng: np.random.Generator) -> float:
    pair = rnd.get(key, list(default))
    lo, hi = float(pair[0]), float(pair[1])
    return _clip_sample(lo, hi, rng)


class HybridTransmissionSampler:
    """Loads base belt + cable YAML configs; ``sample()`` returns both dicts."""

    def __init__(
        self,
        belt_cfg: Dict[str, Any],
        cable_cfg: Dict[str, Any],
        *,
        seed: int | None = None,
    ) -> None:
        self._belt_base = deepcopy(belt_cfg)
        self._cable_base = deepcopy(cable_cfg)
        self._rng = np.random.default_rng(seed)
        self.belt_randomization_enabled = bool(
            belt_cfg.get("randomization", {}).get("enabled", False)
        )
        self.cable_randomization_enabled = bool(
            cable_cfg.get("randomization", {}).get("enabled", False)
        )

    def set_seed(self, seed: int | None) -> None:
        self._rng = np.random.default_rng(seed)

    def sample(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        belt = deepcopy(self._belt_base)
        cable = deepcopy(self._cable_base)

        if self.belt_randomization_enabled:
            br = belt["randomization"]
            belt["gear_ratio"] = _sample_scalar(br, "gear_ratio", (0.9, 1.1), self._rng)
            belt["efficiency"] = np.clip(
                _sample_scalar(br, "efficiency", (0.92, 0.99), self._rng), 0.05, 1.0
            )
            belt["belt_damping"] = _sample_scalar(br, "belt_damping", (0.06, 0.22), self._rng)
            belt["belt_coulomb"] = _sample_scalar(br, "belt_coulomb", (0.05, 0.22), self._rng)
            belt["belt_viscous"] = _sample_scalar(br, "belt_viscous", (0.02, 0.08), self._rng)
            belt["backlash_width"] = _sample_scalar(br, "backlash_width", (0.0005, 0.004), self._rng)
            belt["deadzone_width"] = _sample_scalar(br, "deadzone_width", (0.0002, 0.001), self._rng)
            belt["max_belt_effect_torque"] = _sample_scalar(
                br, "max_belt_effect_torque", (10.0, 18.0), self._rng
            )

        if self.cable_randomization_enabled:
            cr = cable["randomization"]
            n = int(cable["num_joints"])

            def arr_range(key: str, default_pair: Tuple[float, float]) -> np.ndarray:
                pair = cr.get(key, list(default_pair))
                lo, hi = float(pair[0]), float(pair[1])
                return np.array([_clip_sample(lo, hi, self._rng) for _ in range(n)], dtype=float)

            cable["pulley_radius"] = arr_range("pulley_radius", (0.015, 0.025))
            cable["stiffness"] = arr_range("stiffness", (80.0, 180.0))
            cable["damping"] = arr_range("damping", (4.0, 14.0))
            cable["coulomb"] = arr_range("coulomb", (0.05, 0.35))
            cable["viscous"] = arr_range("viscous", (0.2, 0.8))
            cable["deadzone_width"] = arr_range("deadzone_width", (0.0005, 0.006))
            cable["backlash_width"] = arr_range("backlash_width", (0.001, 0.008))

            bw = cable["bouc_wen"]
            bw["alpha"] = np.array(
                [_sample_scalar(cr, "bouc_wen_alpha", (1.0, 4.0), self._rng) for _ in range(n)],
                dtype=float,
            )
            bw["A"] = np.array(
                [_sample_scalar(cr, "bouc_wen_A", (0.2, 1.0), self._rng) for _ in range(n)],
                dtype=float,
            )
            bw["beta"] = np.array(
                [_sample_scalar(cr, "bouc_wen_beta", (0.2, 1.0), self._rng) for _ in range(n)],
                dtype=float,
            )
            bw["gamma"] = np.array(
                [_sample_scalar(cr, "bouc_wen_gamma", (0.2, 1.0), self._rng) for _ in range(n)],
                dtype=float,
            )
            bw["n"] = np.array(
                [_sample_scalar(cr, "bouc_wen_n", (0.8, 1.2), self._rng) for _ in range(n)],
                dtype=float,
            )

            cable["pulley_radius"] = np.clip(cable["pulley_radius"], 0.005, 0.08)
            cable["stiffness"] = np.clip(cable["stiffness"], 10.0, 500.0)
            cable["damping"] = np.clip(cable["damping"], 0.5, 50.0)

        return belt, cable
