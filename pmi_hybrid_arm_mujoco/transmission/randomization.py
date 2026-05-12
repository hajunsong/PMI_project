"""Cable-layer parameter sampling for q2_act–q4_act (uniform ranges, optional per-joint independence)."""

from __future__ import annotations

from typing import Any

import numpy as np

from .cable_transmission import (
    CableLayerBacklashDeadzoneConfig,
    CableLayerDelayConfig,
    CableLayerElasticityConfig,
    CableLayerFrictionConfig,
    CableLayerHysteresisConfig,
    CableTransmission,
)

JOINT_IDS = ("q2_act", "q3_act", "q4_act")

# Keys in configs/cable_layer.yaml randomization.ranges (and sampled arrays)
RANGE_KEYS_ARRAY = (
    "tau_delay",
    "viscous_b",
    "coulomb_fc",
    "tau_elastic",
    "tau_deadzone",
    "backlash_width",
    "hys_alpha",
    "hys_A",
    "hys_beta",
    "hys_gamma",
    "hys_n",
)
RANGE_KEY_SCALAR = "backlash_slope"


class CableParameterRandomizer:
    """
    Sample physical cable parameters for the three cable actuators from:

    - ``randomization_profiles[profile_name]["ranges"]`` when ``profiles`` contains that profile, or
    - ``randomization["ranges"]`` (legacy mild block / backward compat).

    Parameters
    ----------
    randomization_config
        The ``randomization`` subtree of ``cable_layer.yaml``.
    profile_name
        Key under ``randomization_profiles`` (e.g. ``"mild"``, ``"medium"``, ``"medium_v2"``, ``"medium_train"``, ``"stress"``) or ``None``.
        When ``None``, ``active_profile`` is used when ``profiles`` is provided; otherwise legacy ``ranges``.
    profiles
        The ``randomization_profiles`` mapping from ``cable_layer.yaml`` root.
    active_profile
        Default profile key from ``active_profile`` in yaml when ``profile_name`` is ``None``.
    """

    def __init__(
        self,
        randomization_config: dict[str, Any],
        profile_name: str | None = None,
        *,
        profiles: dict[str, Any] | None = None,
        active_profile: str | None = None,
    ):
        self._cfg = dict(randomization_config)
        self._mode = str(self._cfg.get("mode", "uniform")).lower()
        self._per_joint = bool(self._cfg.get("per_joint_independent", True))
        self._apply_to = list(self._cfg.get("apply_to", list(JOINT_IDS)))
        if len(self._apply_to) != 3:
            raise ValueError("randomization.apply_to must list exactly q2_act, q3_act, q4_act")
        self._last_seed: int | None = None
        self._last_sample: dict[str, Any] | None = None

        pmap = profiles or {}
        pname = profile_name if profile_name is not None else active_profile
        range_source_label: str

        rng_block: dict[str, Any] | None = None
        if pname is not None and isinstance(pmap.get(pname), dict):
            rng_block = pmap[pname].get("ranges")

        if rng_block is not None:
            self._ranges = dict(rng_block)
            range_source_label = str(pname)
        else:
            if "ranges" not in self._cfg:
                raise ValueError(
                    "randomization: missing legacy 'ranges' and no usable randomization_profiles[profile]"
                )
            self._ranges = dict(self._cfg["ranges"])
            range_source_label = "randomization_ranges" if pname is None else f"randomization_ranges_fallback"

        self._resolved_profile_label = range_source_label
        self._validate_ranges()

    @property
    def resolved_profile_label(self) -> str:
        """Where ``_ranges`` was taken from (profile name or fallback label)."""

        return self._resolved_profile_label

    def _validate_ranges(self) -> None:
        for k in RANGE_KEYS_ARRAY:
            r = self._ranges[k]
            if not (isinstance(r, (list, tuple)) and len(r) == 2):
                raise ValueError(f"randomization.ranges.{k} must be [lo, hi]")
            lo, hi = float(r[0]), float(r[1])
            if lo > hi:
                raise ValueError(f"randomization.ranges.{k}: lo > hi")
        rs = self._ranges[RANGE_KEY_SCALAR]
        if not (isinstance(rs, (list, tuple)) and len(rs) == 2):
            raise ValueError(f"randomization.ranges.{RANGE_KEY_SCALAR} must be [lo, hi]")
        lo, hi = float(rs[0]), float(rs[1])
        if lo > hi:
            raise ValueError("backlash_slope: lo > hi")

    def _draw_uniform(self, rng: np.random.Generator, lo: float, hi: float) -> float:
        return float(rng.uniform(float(lo), float(hi)))

    def sample(self, seed: int | None = None) -> dict[str, Any]:
        """
        Sample one set of parameters.

        Returns
        -------
        dict with keys ``per_joint``, ``arrays``, ``backlash_slope``.
        """
        s = int(seed if seed is not None else self._cfg.get("seed", 0))
        self._last_seed = s
        rng = np.random.default_rng(s)

        per_joint: dict[str, dict[str, float]] = {j: {} for j in JOINT_IDS}
        arrays: dict[str, np.ndarray] = {}

        if self._mode != "uniform":
            raise NotImplementedError(f"randomization mode {self._mode!r} — only 'uniform' supported")

        # Per-array parameters
        for key in RANGE_KEYS_ARRAY:
            lo, hi = float(self._ranges[key][0]), float(self._ranges[key][1])
            vec = np.zeros(3, dtype=np.float64)
            if self._per_joint:
                for i in range(3):
                    vec[i] = self._draw_uniform(rng, lo, hi)
            else:
                v = self._draw_uniform(rng, lo, hi)
                vec[:] = v
            arrays[key] = vec
            for i, jname in enumerate(JOINT_IDS):
                per_joint[jname][key] = float(vec[i])

        # Scalar backlash slope (single value for whole transmission)
        slo, shi = float(self._ranges[RANGE_KEY_SCALAR][0]), float(self._ranges[RANGE_KEY_SCALAR][1])
        backlash_slope = self._draw_uniform(rng, slo, shi)

        out = {
            "per_joint": per_joint,
            "arrays": arrays,
            "backlash_slope": float(backlash_slope),
        }
        self._validate_sample(out)
        self._last_sample = out
        return out

    def _validate_sample(self, sample: dict[str, Any]) -> None:
        for key in RANGE_KEYS_ARRAY:
            lo, hi = float(self._ranges[key][0]), float(self._ranges[key][1])
            for i in range(3):
                v = float(sample["arrays"][key][i])
                if v < lo - 1e-9 or v > hi + 1e-9:
                    raise ValueError(f"sampled {key}[{i}]={v} outside [{lo}, {hi}]")
        slo, shi = float(self._ranges[RANGE_KEY_SCALAR][0]), float(self._ranges[RANGE_KEY_SCALAR][1])
        b = float(sample["backlash_slope"])
        if b < slo - 1e-9 or b > shi + 1e-9:
            raise ValueError(f"sampled backlash_slope={b} outside [{slo}, {shi}]")

    def sampled_params_to_flat_dict(self, sample: dict[str, Any] | None = None) -> dict[str, float]:
        """Flatten ``per_joint`` parameters for CSV logging (one row prefix per joint)."""
        src = sample if sample is not None else self._last_sample
        if src is None:
            return {}
        flat: dict[str, float] = {"backlash_slope": float(src["backlash_slope"])}
        for jname in JOINT_IDS:
            for k, v in src["per_joint"][jname].items():
                flat[f"{jname}_{k}"] = float(v)
        return flat

    @property
    def last_seed(self) -> int | None:
        return self._last_seed


def build_cable_transmission_from_full_config(full_cable_yaml: dict[str, Any], sample: dict[str, Any]) -> CableTransmission:
    """
    Build :class:`CableTransmission` from full ``cable_layer.yaml`` content plus a :meth:`CableParameterRandomizer.sample` output.
    """
    a = sample["arrays"]
    backlash_slope = float(sample["backlash_slope"])

    tau_delay_s = np.asarray(a["tau_delay"], dtype=np.float64).reshape(3).copy()
    viscous_b = np.asarray(a["viscous_b"], dtype=np.float64).reshape(3).copy()
    coulomb_fc = np.asarray(a["coulomb_fc"], dtype=np.float64).reshape(3).copy()
    tau_elastic_s = np.asarray(a["tau_elastic"], dtype=np.float64).reshape(3).copy()
    tau_deadzone = np.asarray(a["tau_deadzone"], dtype=np.float64).reshape(3).copy()
    backlash_width = np.asarray(a["backlash_width"], dtype=np.float64).reshape(3).copy()
    hys_alpha = np.asarray(a["hys_alpha"], dtype=np.float64).reshape(3).copy()
    hys_A = np.asarray(a["hys_A"], dtype=np.float64).reshape(3).copy()
    hys_beta = np.asarray(a["hys_beta"], dtype=np.float64).reshape(3).copy()
    hys_gamma = np.asarray(a["hys_gamma"], dtype=np.float64).reshape(3).copy()
    hys_n = np.asarray(a["hys_n"], dtype=np.float64).reshape(3).copy()

    cd = full_cable_yaml["cable_delay"]
    cf = full_cable_yaml["cable_friction"]
    el = full_cable_yaml["cable_elasticity"]
    bd = full_cable_yaml["cable_backlash_deadzone"]
    hy = full_cable_yaml["cable_hysteresis"]

    delay_glob = bool(cd["enabled"])
    friction_glob = bool(cf["enabled"])
    elastic_glob = bool(el["enabled"])
    elastic_mode = str(el["mode"])
    bd_glob = bool(bd["enabled"])
    hys_glob = bool(hy["enabled"])

    init_d = bool(cd["initialize_state_to_input"])
    init_e = bool(el["initialize_state_to_input"])
    v_eps = float(cf["v_eps"])
    tau_eps = float(bd["tau_eps"])
    deadzone_mode = str(bd.get("deadzone_mode", "hard"))
    z_max = float(hy["z_max"])
    tau_hys_max = float(hy["tau_hys_max"])
    init_z_hys = bool(hy["initialize_z_to_zero"])
    tau_clip = cf.get("tau_out_clip")

    if delay_glob and np.any(tau_delay_s > 0.0):
        dcfg = CableLayerDelayConfig(enabled=True, tau_delay_s=tau_delay_s, initialize_state_to_input=init_d)
    else:
        dcfg = CableLayerDelayConfig(
            enabled=False, tau_delay_s=np.zeros(3, dtype=np.float64), initialize_state_to_input=init_d
        )

    if friction_glob and (np.any(np.abs(viscous_b) > 0.0) or np.any(np.abs(coulomb_fc) > 0.0)):
        fcfg = CableLayerFrictionConfig(
            enabled=True,
            viscous_b=viscous_b,
            coulomb_fc=coulomb_fc,
            v_eps=float(v_eps),
            tau_out_clip=float(tau_clip) if tau_clip is not None else None,
        )
    else:
        fcfg = CableLayerFrictionConfig(
            enabled=False,
            viscous_b=np.zeros(3, dtype=np.float64),
            coulomb_fc=np.zeros(3, dtype=np.float64),
            v_eps=float(v_eps),
            tau_out_clip=float(tau_clip) if tau_clip is not None else None,
        )

    if elastic_glob and np.any(tau_elastic_s > 0.0) and elastic_mode == "torque_compliance":
        ecfg = CableLayerElasticityConfig(
            enabled=True,
            mode="torque_compliance",
            tau_elastic_s=tau_elastic_s,
            initialize_state_to_input=init_e,
        )
    else:
        ecfg = CableLayerElasticityConfig(
            enabled=False,
            mode="torque_compliance",
            tau_elastic_s=np.zeros(3, dtype=np.float64),
            initialize_state_to_input=init_e,
        )

    if bd_glob and (
        np.any(tau_deadzone > 0.0) or np.any(backlash_width > 0.0) or deadzone_mode.lower() == "smooth"
    ):
        bz = CableLayerBacklashDeadzoneConfig(
            enabled=True,
            tau_deadzone=tau_deadzone,
            backlash_width=backlash_width,
            backlash_slope=backlash_slope,
            tau_eps=float(tau_eps),
            deadzone_mode=deadzone_mode,
        )
    else:
        bz = CableLayerBacklashDeadzoneConfig(enabled=False)

    if hys_glob and np.any(hys_alpha > 0.0):
        hcfg = CableLayerHysteresisConfig(
            enabled=True,
            mode="torque_bouc_wen",
            alpha=hys_alpha,
            A=hys_A,
            beta=hys_beta,
            gamma=hys_gamma,
            n=hys_n,
            z_max=z_max,
            tau_hys_max=tau_hys_max,
            initialize_z_to_zero=init_z_hys,
        )
    else:
        hcfg = CableLayerHysteresisConfig(enabled=False)

    return CableTransmission(dcfg, fcfg, ecfg, bz, hcfg)
