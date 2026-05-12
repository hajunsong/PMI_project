"""Optional randomisation of passive cable/antagonistic parameters (q2~q4 only)."""

from __future__ import annotations

import random
from typing import Any

from .cable_transmission import CableTransmissionParams
from .hysteresis import BoucWenParams


def _u(scale: float, rng: random.Random) -> float:
    return 1.0 + float(rng.uniform(-scale, scale))


def randomize_cable_transmission_params(
    base_yaml: dict[str, Any],
    *,
    rng: random.Random | None = None,
    scale: float = 0.05,
) -> CableTransmissionParams:
    r = rng or random.Random()
    s = float(scale)
    g = base_yaml.get("generic", {})
    bw = g.get("bouc_wen", {})

    def nu(key: str) -> float:
        return float(g[key]) * _u(s, r)

    return CableTransmissionParams(
        tau_delay_sec=float(g["tau_delay_sec"]) * _u(s, r),
        k_elastic=nu("k_elastic_nominal"),
        c_viscous_stretch=nu("c_viscous_stretch_nominal"),
        Fc=nu("Fc_nominal"),
        v_eps=nu("v_eps_nominal"),
        Fv=nu("Fv_nominal"),
        deadzone_xdot=nu("deadzone_xdot_nominal"),
        backlash_x=nu("backlash_x_nominal"),
        T_clip=nu("T_clip_nominal"),
        bw=BoucWenParams(
            alpha=float(bw["alpha"]) * _u(s, r),
            A=float(bw["A"]) * _u(s, r),
            beta=float(bw["beta"]) * _u(s, r),
            gamma=float(bw["gamma"]) * _u(s, r),
            n=float(bw["n_bw"]),
            z_clip=float(bw["z_clip"]) * _u(s, r),
        ),
    )


def randomize_radii(
    r_motor: float,
    r_joint: float,
    *,
    rng: random.Random | None = None,
    scale: float = 0.05,
) -> tuple[float, float]:
    r = rng or random.Random()
    s = float(scale)
    return float(r_motor) * _u(s, r), float(r_joint) * _u(s, r)


def random_preload(nominal: float, *, rng: random.Random | None = None, scale: float = 0.05) -> float:
    r = rng or random.Random()
    return float(nominal) * _u(float(scale), r)
