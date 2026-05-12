"""Cable-layer (q2_act–q4_act): delay → compliance → Bouc–Wen → dead-zone/backlash → friction."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def _alpha_first_order(dt: float, tau_time: float) -> float:
    if tau_time <= 0.0:
        return 1.0
    return float(np.clip(float(dt) / float(tau_time), 0.0, 1.0))


def _sign_sparse(x: float, eps: float = 1e-12) -> float:
    if abs(x) < eps:
        return 0.0
    return float(np.sign(x))


@dataclass
class CableLayerDelayConfig:
    """First-order lag: d(tau)/dt = (tau_in - tau) / tau_delay (per channel)."""

    enabled: bool = False
    tau_delay_s: np.ndarray | None = None  # (3,) for q2, q3, q4; seconds; <=0 => pass-through
    initialize_state_to_input: bool = True

    def __post_init__(self) -> None:
        if self.tau_delay_s is None:
            self.tau_delay_s = np.zeros(3, dtype=np.float64)
        else:
            self.tau_delay_s = np.asarray(self.tau_delay_s, dtype=np.float64).reshape(3).copy()


@dataclass
class CableLayerElasticityConfig:
    """
    Actuator-side torque compliance (intermediate model; not geometric stretch while equality q_jnt=ratio*q_act holds).
    First-order lag tracking ``tau_after_delay`` with time constant ``tau_elastic``.
    """

    enabled: bool = False
    mode: str = "torque_compliance"
    tau_elastic_s: np.ndarray | None = None  # (3,) seconds; <=0 => bypass
    initialize_state_to_input: bool = True

    def __post_init__(self) -> None:
        if self.tau_elastic_s is None:
            self.tau_elastic_s = np.zeros(3, dtype=np.float64)
        else:
            self.tau_elastic_s = np.asarray(self.tau_elastic_s, dtype=np.float64).reshape(3).copy()


@dataclass
class CableLayerHysteresisConfig:
    """
    Torque-domain Bouc–Wen driven by compliant torque ``u = tau_compliant`` and
    ``udot ≈ (u - u_prev) / dt``. Output ``tau_after_hysteresis = tau_compliant - tau_hys`` with
    ``tau_hys = alpha * z`` (clipped). Internal state ``z`` follows the standard Bouc–Wen ODE in ``z``.
    """

    enabled: bool = False
    mode: str = "torque_bouc_wen"
    alpha: np.ndarray | None = None  # (3,) scales hysteresis torque
    A: np.ndarray | None = None  # (3,)
    beta: np.ndarray | None = None  # (3,)
    gamma: np.ndarray | None = None  # (3,)
    n: np.ndarray | None = None  # (3,) exponent; values < 1 are clamped to 1
    z_max: float = 1.0  # clip on z after integration
    tau_hys_max: float = 1.0  # symmetric clip on tau_hys = alpha*z
    initialize_z_to_zero: bool = True

    def __post_init__(self) -> None:
        z3 = np.zeros(3, dtype=np.float64)

        def _r3(x: np.ndarray | None) -> np.ndarray:
            if x is None:
                return z3.copy()
            return np.asarray(x, dtype=np.float64).reshape(3).copy()

        self.alpha = _r3(self.alpha)
        self.A = _r3(self.A)
        self.beta = _r3(self.beta)
        self.gamma = _r3(self.gamma)
        self.n = _r3(self.n)


@dataclass
class CableLayerBacklashDeadzoneConfig:
    """
    Torque-domain dead-zone and direction-change backlash (not geometric; equality keeps q_jnt = ratio * q_act).

    Backlash: on sign change of ``tau_ref`` (torque after hysteresis: ``tau_after_hysteresis``), a bucket
    ``backlash_remaining`` is filled; while > 0, output is ``backlash_slope * tau_after_deadzone``;
    bucket decreases by ``|tau_ref - prev_tau_ref|`` each step.
    """

    enabled: bool = False
    tau_deadzone: np.ndarray | None = None  # (3,) N·m half-width, hard zone
    backlash_width: np.ndarray | None = None  # (3,) “slack” budget on sign reversal
    backlash_slope: float = 1.0  # transmission scale while slack remains
    tau_eps: float = 0.001  # for smooth dead-zone / tiny sign threshold
    deadzone_mode: str = "hard"  # "hard" | "smooth"

    def __post_init__(self) -> None:
        if self.tau_deadzone is None:
            self.tau_deadzone = np.zeros(3, dtype=np.float64)
        else:
            self.tau_deadzone = np.asarray(self.tau_deadzone, dtype=np.float64).reshape(3).copy()
        if self.backlash_width is None:
            self.backlash_width = np.zeros(3, dtype=np.float64)
        else:
            self.backlash_width = np.asarray(self.backlash_width, dtype=np.float64).reshape(3).copy()


@dataclass
class CableLayerFrictionConfig:
    """Actuator-side losses; applied after backlash stage."""

    enabled: bool = False
    viscous_b: np.ndarray | None = None  # (3,) N·m·s/rad
    coulomb_fc: np.ndarray | None = None  # (3,) N·m magnitude
    v_eps: float = 0.01  # rad/s smoothing for tanh
    tau_out_clip: float | None = None  # optional symmetric clip on tau_out [Nm]


@dataclass(frozen=True)
class CableTransmitResult:
    """Diagnostics for q2–q4."""

    tau_out: np.ndarray  # (3,)
    tau_after_delay: np.ndarray  # (3,)
    tau_compliant: np.ndarray  # (3,) after torque compliance
    tau_elastic_state: np.ndarray  # (3,)
    tau_elastic_error: np.ndarray  # (3,) tau_compliant - tau_after_delay
    tau_after_hysteresis: np.ndarray  # (3,) after Bouc–Wen
    tau_hys: np.ndarray  # (3,) hysteresis torque subtracted from tau_compliant
    hys_u: np.ndarray  # (3,) Bouc–Wen input (= tau_compliant)
    hys_udot: np.ndarray  # (3,)
    hys_z: np.ndarray  # (3,) internal Bouc–Wen state
    hys_zdot: np.ndarray  # (3,)
    tau_after_deadzone: np.ndarray  # (3,)
    tau_after_backlash: np.ndarray  # (3,)
    deadzone_active: np.ndarray  # (3,) 1.0 if hard dead-zone would zero small torque
    backlash_active: np.ndarray  # (3,) 1.0 if backlash_remaining > 0
    backlash_remaining: np.ndarray  # (3,)
    torque_sign: np.ndarray  # (3,) sign of tau_after_hysteresis (-1,0,1)
    tau_viscous: np.ndarray  # (3,)
    tau_coulomb: np.ndarray  # (3,)
    tau_loss: np.ndarray  # (3,)


class CableTransmission:
    """
    Pipeline::

        tau_in → delay → tau_after_delay
               → compliance → tau_compliant
               → Bouc–Wen → tau_after_hysteresis
               → dead-zone → tau_after_deadzone
               → backlash → tau_after_backlash
               → friction → tau_out
    """

    def __init__(
        self,
        delay: CableLayerDelayConfig | None = None,
        friction: CableLayerFrictionConfig | None = None,
        elasticity: CableLayerElasticityConfig | None = None,
        backlash_deadzone: CableLayerBacklashDeadzoneConfig | None = None,
        hysteresis: CableLayerHysteresisConfig | None = None,
    ):
        d = delay if delay is not None else CableLayerDelayConfig(
            enabled=False,
            tau_delay_s=np.zeros(3, dtype=np.float64),
            initialize_state_to_input=True,
        )

        f = friction if friction is not None else CableLayerFrictionConfig(
            enabled=False,
            viscous_b=np.zeros(3, dtype=np.float64),
            coulomb_fc=np.zeros(3, dtype=np.float64),
            v_eps=0.01,
            tau_out_clip=None,
        )
        e = elasticity if elasticity is not None else CableLayerElasticityConfig(
            enabled=False,
            mode="torque_compliance",
            tau_elastic_s=np.zeros(3, dtype=np.float64),
            initialize_state_to_input=True,
        )
        bz = backlash_deadzone if backlash_deadzone is not None else CableLayerBacklashDeadzoneConfig(
            enabled=False,
        )
        hy = hysteresis if hysteresis is not None else CableLayerHysteresisConfig(enabled=False)

        self._cfg = d
        self._fr = f
        self._el = e
        self._bz = bz
        self._hy = hy

        self._tau_delay = np.asarray(d.tau_delay_s, dtype=np.float64).reshape(3).copy()
        self._delay_on = bool(d.enabled) and np.any(self._tau_delay > 0.0)
        self._init_to_input = bool(d.initialize_state_to_input)
        self._b = np.asarray(f.viscous_b if f.viscous_b is not None else np.zeros(3), dtype=np.float64).reshape(3).copy()
        self._fc = np.asarray(f.coulomb_fc if f.coulomb_fc is not None else np.zeros(3), dtype=np.float64).reshape(3).copy()
        self._v_eps = float(max(f.v_eps, 1e-12))
        self._tau_clip = f.tau_out_clip
        self._friction_on = bool(f.enabled) and (
            np.any(np.abs(self._b) > 0.0) or np.any(np.abs(self._fc) > 0.0)
        )

        te = np.asarray(e.tau_elastic_s, dtype=np.float64).reshape(3).copy()
        self._tau_elastic = te
        self._elastic_on = (
            bool(e.enabled)
            and str(e.mode) == "torque_compliance"
            and np.any(te > 0.0)
        )
        self._init_elastic_to_input = bool(e.initialize_state_to_input)

        self._dz = np.asarray(bz.tau_deadzone, dtype=np.float64).reshape(3).copy()
        self._bw = np.asarray(bz.backlash_width, dtype=np.float64).reshape(3).copy()
        self._bslope = float(bz.backlash_slope)
        self._dz_tau_eps = float(max(bz.tau_eps, 1e-15))
        self._dz_mode = str(bz.deadzone_mode).lower()

        self._bd_on = bool(bz.enabled) and (
            np.any(self._dz > 0.0)
            or np.any(self._bw > 0.0)
            or self._dz_mode == "smooth"
        )

        ha = np.asarray(hy.alpha, dtype=np.float64).reshape(3).copy()
        self._hys_alpha = ha
        self._hys_A = np.asarray(hy.A, dtype=np.float64).reshape(3).copy()
        self._hys_beta = np.asarray(hy.beta, dtype=np.float64).reshape(3).copy()
        self._hys_gamma = np.asarray(hy.gamma, dtype=np.float64).reshape(3).copy()
        self._hys_n = np.asarray(hy.n, dtype=np.float64).reshape(3).copy()
        self._hys_z_max = float(max(hy.z_max, 0.0))
        self._hys_tau_max = float(max(hy.tau_hys_max, 0.0))
        self._hys_init_z0 = bool(hy.initialize_z_to_zero)
        self._hys_on = (
            bool(hy.enabled)
            and str(hy.mode).lower() == "torque_bouc_wen"
            and np.any(self._hys_alpha > 0.0)
        )

        self._delay_state = np.zeros(3, dtype=np.float64)
        self._primed = False
        self._elastic_state = np.zeros(3, dtype=np.float64)
        self._elastic_primed = False

        # Backlash / dead-zone memory
        self._bl_prev_tc = np.zeros(3, dtype=np.float64)
        self._bl_prev_sign = np.zeros(3, dtype=np.float64)
        self._bl_rem = np.zeros(3, dtype=np.float64)
        self._bl_have_prev = np.array([False, False, False], dtype=bool)

        self._z_hys = np.zeros(3, dtype=np.float64)
        self._u_prev = np.zeros(3, dtype=np.float64)
        self._hys_primed = False

    @property
    def tau_delay(self) -> np.ndarray:
        return self._tau_delay.copy()

    @property
    def delay_enabled_effective(self) -> bool:
        return self._delay_on

    def reset(self) -> None:
        self._delay_state.fill(0.0)
        self._primed = False
        self._elastic_state.fill(0.0)
        self._elastic_primed = False
        self._bl_prev_tc.fill(0.0)
        self._bl_prev_sign.fill(0.0)
        self._bl_rem.fill(0.0)
        self._bl_have_prev[:] = False
        self._z_hys.fill(0.0)
        self._u_prev.fill(0.0)
        self._hys_primed = False

    def _bouc_wen_hysteresis(
        self, tau_compliant: np.ndarray, dt: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        ``u = tau_compliant``, ``udot = (u - u_prev) / dt``; Bouc–Wen on ``z``;
        ``tau_hys = clip(alpha * z)``, output ``tau_after_hysteresis = u - tau_hys``.
        """
        u = np.asarray(tau_compliant, dtype=np.float64).reshape(3)
        zdot_out = np.zeros(3, dtype=np.float64)
        udot = np.zeros(3, dtype=np.float64)
        tau_hys = np.zeros(3, dtype=np.float64)
        z_snap = np.zeros(3, dtype=np.float64)

        if not self._hys_on:
            return u.copy(), tau_hys, self._z_hys.copy(), zdot_out, udot

        dt_f = float(dt)
        eps_z = 1e-15
        zm = self._hys_z_max
        tm = self._hys_tau_max

        if not self._hys_primed:
            self._u_prev = u.copy()
            if self._hys_init_z0:
                self._z_hys.fill(0.0)
            self._hys_primed = True
            return u.copy(), tau_hys, self._z_hys.copy(), zdot_out, udot

        if dt_f <= 0.0:
            for i in range(3):
                zi = float(self._z_hys[i])
                z_snap[i] = zi
                if self._hys_alpha[i] > 0.0:
                    th = self._hys_alpha[i] * zi
                    if tm > 0.0:
                        th = float(np.clip(th, -tm, tm))
                    tau_hys[i] = th
            u_out = u - tau_hys
            return u_out, tau_hys, z_snap, zdot_out, udot

        u_out = np.empty(3, dtype=np.float64)
        for i in range(3):
            ui = float(u[i])
            udot[i] = (ui - float(self._u_prev[i])) / dt_f
            zi = float(self._z_hys[i])

            if self._hys_alpha[i] <= 0.0:
                tau_hys[i] = 0.0
                u_out[i] = ui
                z_snap[i] = zi
                self._u_prev[i] = ui
                continue

            n_i = float(max(float(self._hys_n[i]), 1.0))
            az = max(abs(zi), eps_z)
            ai = float(self._hys_A[i])
            bi = float(self._hys_beta[i])
            gi = float(self._hys_gamma[i])
            ud = float(udot[i])

            zdot_i = (
                ai * ud
                - bi * abs(ud) * (az ** (n_i - 1.0)) * zi
                - gi * ud * (az**n_i)
            )
            if not np.isfinite(zdot_i):
                zdot_i = 0.0
            zdot_out[i] = zdot_i

            zi_new = zi + zdot_i * dt_f
            if zm > 0.0:
                zi_new = float(np.clip(zi_new, -zm, zm))
            self._z_hys[i] = zi_new
            z_snap[i] = zi_new
            self._u_prev[i] = ui

            th = self._hys_alpha[i] * zi_new
            if tm > 0.0:
                th = float(np.clip(th, -tm, tm))
            tau_hys[i] = th
            u_out[i] = ui - th

        return u_out, tau_hys, z_snap, zdot_out, udot

    def _first_order_delay(self, tau_in: np.ndarray, dt: float) -> np.ndarray:
        tin = np.asarray(tau_in, dtype=np.float64).reshape(3)
        if not self._delay_on:
            return tin.copy()

        if not self._primed and self._init_to_input:
            self._delay_state = tin.copy()
            self._primed = True
            return self._delay_state.copy()

        self._primed = True
        dt_f = float(dt)
        td = self._tau_delay
        for i in range(3):
            if td[i] <= 0.0:
                self._delay_state[i] = tin[i]
            else:
                a = _alpha_first_order(dt_f, float(td[i]))
                self._delay_state[i] += a * (tin[i] - self._delay_state[i])
        return self._delay_state.copy()

    def _torque_compliance(self, tau_after_delay: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        tad = np.asarray(tau_after_delay, dtype=np.float64).reshape(3)
        if not self._elastic_on:
            return tad.copy(), tad.copy()

        if not self._elastic_primed and self._init_elastic_to_input:
            self._elastic_state = tad.copy()
            self._elastic_primed = True
            return self._elastic_state.copy(), self._elastic_state.copy()

        self._elastic_primed = True
        dt_f = float(dt)
        te = self._tau_elastic
        for i in range(3):
            if te[i] <= 0.0:
                self._elastic_state[i] = tad[i]
            else:
                ae = _alpha_first_order(dt_f, float(te[i]))
                self._elastic_state[i] += ae * (tad[i] - self._elastic_state[i])
        return self._elastic_state.copy(), self._elastic_state.copy()

    def _apply_deadzone(self, tau_compliant: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Returns (tau_after_deadzone, deadzone_active flags)."""
        tc = np.asarray(tau_compliant, dtype=np.float64).reshape(3)
        if not self._bd_on or (not np.any(self._dz > 0.0) and self._dz_mode != "smooth"):
            z = np.zeros(3, dtype=np.float64)
            return tc.copy(), z

        out = np.empty(3, dtype=np.float64)
        active = np.zeros(3, dtype=np.float64)
        for i in range(3):
            t = float(tc[i])
            dz = float(self._dz[i])
            if self._dz_mode == "smooth":
                # smooth: subtract soft limit, avoids discontinuity
                out[i] = t - dz * np.tanh(t / self._dz_tau_eps)
                active[i] = 1.0 if abs(t) < dz else 0.0
            else:
                if abs(t) < dz:
                    out[i] = 0.0
                    active[i] = 1.0
                else:
                    out[i] = t - dz * np.sign(t)
                    active[i] = 0.0
        return out, active

    def _apply_backlash(
        self,
        tau_ref: np.ndarray,
        tau_after_deadzone: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Sign flips on **tau_ref** (``tau_after_hysteresis``) reload backlash_width; output reduced while
        backlash_remaining>0.

        Returns
        -------
        tau_after_backlash, backlash_active, backlash_remaining, torque_sign (all (3,))
        """
        tc = np.asarray(tau_ref, dtype=np.float64).reshape(3)
        tdz = np.asarray(tau_after_deadzone, dtype=np.float64).reshape(3)
        out = np.empty(3, dtype=np.float64)
        b_act = np.zeros(3, dtype=np.float64)
        rem_out = np.zeros(3, dtype=np.float64)
        sgn_out = np.zeros(3, dtype=np.float64)

        if not self._bd_on or not np.any(self._bw > 0.0):
            return tdz.copy(), b_act, self._bl_rem.copy(), np.array([_sign_sparse(float(tc[i])) for i in range(3)])

        for i in range(3):
            sn = _sign_sparse(float(tc[i]))
            sgn_out[i] = sn

            if not self._bl_have_prev[i]:
                self._bl_prev_tc[i] = tc[i]
                self._bl_prev_sign[i] = sn
                self._bl_have_prev[i] = True
            else:
                ps = float(self._bl_prev_sign[i])
                if ps != 0.0 and sn != 0.0 and ps != sn:
                    self._bl_rem[i] = float(self._bw[i])

                delta = abs(float(tc[i]) - float(self._bl_prev_tc[i]))
                if self._bl_rem[i] > 0.0:
                    self._bl_rem[i] = max(0.0, self._bl_rem[i] - delta)

                self._bl_prev_tc[i] = tc[i]
                self._bl_prev_sign[i] = sn

            rem_out[i] = float(self._bl_rem[i])
            b_act[i] = 1.0 if self._bl_rem[i] > 1e-15 else 0.0
            if self._bl_rem[i] > 1e-15:
                out[i] = float(self._bslope) * float(tdz[i])
            else:
                out[i] = float(tdz[i])

        return out, b_act, rem_out, sgn_out

    def _friction_terms(self, qdot_act: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        qd = np.asarray(qdot_act, dtype=np.float64).reshape(3)
        if not self._friction_on:
            z = np.zeros(3, dtype=np.float64)
            return z, z, z
        tv = self._b * qd
        tc = self._fc * np.tanh(qd / self._v_eps)
        loss = tv + tc
        return tv, tc, loss

    def transmit(
        self,
        tau_in: np.ndarray,
        dt: float,
        q_act: np.ndarray,
        qdot_act: np.ndarray,
    ) -> CableTransmitResult:
        _ = np.asarray(q_act, dtype=np.float64).reshape(3)
        qd = np.asarray(qdot_act, dtype=np.float64).reshape(3)

        tin = np.asarray(tau_in, dtype=np.float64).reshape(3)
        if tin.size != 3:
            raise ValueError("CableTransmission expects length-3 arrays for q2_act–q4_act")

        tau_after_delay = self._first_order_delay(tin, dt)
        tau_compliant, tau_elast_snap = self._torque_compliance(tau_after_delay, dt)
        tau_el_err = tau_compliant - tau_after_delay

        tau_ah, tau_hys, hys_z, hys_zdot, hys_udot = self._bouc_wen_hysteresis(tau_compliant, dt)
        hys_u = tau_compliant.copy()

        tau_adz, dz_act = self._apply_deadzone(tau_ah)
        tau_abk, bl_act, bl_rem, t_sign = self._apply_backlash(tau_ah, tau_adz)

        tv, tc, tloss = self._friction_terms(qd)
        tout = tau_abk - tloss

        if self._tau_clip is not None:
            c = float(abs(self._tau_clip))
            tout = np.clip(tout, -c, c)

        return CableTransmitResult(
            tau_out=tout.astype(np.float64),
            tau_after_delay=tau_after_delay.astype(np.float64),
            tau_compliant=tau_compliant.astype(np.float64),
            tau_elastic_state=tau_elast_snap.astype(np.float64),
            tau_elastic_error=tau_el_err.astype(np.float64),
            tau_after_hysteresis=tau_ah.astype(np.float64),
            tau_hys=tau_hys.astype(np.float64),
            hys_u=hys_u.astype(np.float64),
            hys_udot=hys_udot.astype(np.float64),
            hys_z=hys_z.astype(np.float64),
            hys_zdot=hys_zdot.astype(np.float64),
            tau_after_deadzone=tau_adz.astype(np.float64),
            tau_after_backlash=tau_abk.astype(np.float64),
            deadzone_active=dz_act.astype(np.float64),
            backlash_active=bl_act.astype(np.float64),
            backlash_remaining=bl_rem.astype(np.float64),
            torque_sign=t_sign.astype(np.float64),
            tau_viscous=tv.astype(np.float64),
            tau_coulomb=tc.astype(np.float64),
            tau_loss=tloss.astype(np.float64),
        )


def cable_transmission_identity() -> CableTransmission:
    """No dynamics (scaffold)."""
    return CableTransmission(
        CableLayerDelayConfig(
            enabled=False,
            tau_delay_s=np.zeros(3, dtype=np.float64),
            initialize_state_to_input=True,
        ),
        CableLayerFrictionConfig(
            enabled=False,
            viscous_b=np.zeros(3, dtype=np.float64),
            coulomb_fc=np.zeros(3, dtype=np.float64),
        ),
        CableLayerElasticityConfig(enabled=False, tau_elastic_s=np.zeros(3, dtype=np.float64)),
        CableLayerBacklashDeadzoneConfig(enabled=False),
        CableLayerHysteresisConfig(enabled=False),
    )
