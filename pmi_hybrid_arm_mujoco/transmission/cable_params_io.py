"""Build CableTransmissionParams from configs/cable_params.yaml."""

from __future__ import annotations

from typing import Any

from .imperfect_cable_transmission import CableTransmissionParams
from .hysteresis import BoucWenParams


def cable_transmission_nominal(global_cfg: dict[str, Any]) -> CableTransmissionParams:
    g = global_cfg["generic"]
    bw = g["bouc_wen"]
    return CableTransmissionParams(
        tau_delay_sec=float(g.get("tau_delay_sec", 0.0)),
        k_elastic=float(g["k_elastic_nominal"]),
        c_viscous_stretch=float(g["c_viscous_stretch_nominal"]),
        Fc=float(g["Fc_nominal"]),
        v_eps=float(g["v_eps_nominal"]),
        Fv=float(g["Fv_nominal"]),
        deadzone_xdot=float(g["deadzone_xdot_nominal"]),
        backlash_x=float(g["backlash_x_nominal"]),
        T_clip=float(g["T_clip_nominal"]),
        bw=BoucWenParams(
            alpha=float(bw["alpha"]),
            A=float(bw["A"]),
            beta=float(bw["beta"]),
            gamma=float(bw["gamma"]),
            n=float(bw["n_bw"]),
            z_clip=float(bw["z_clip"]),
        ),
    )
