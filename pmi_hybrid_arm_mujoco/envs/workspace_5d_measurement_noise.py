"""Evaluation-only measurement / actuator noise (true MuJoCo state stays uncorrupted)."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass
class MeasPacket:
    q_j: np.ndarray
    qd_j: np.ndarray
    q_a: np.ndarray
    qd_a: np.ndarray
    ee_xyz: np.ndarray
    roll: float
    pitch: float
    yaw: float
    xd5: np.ndarray


@dataclass
class _FilterState:
    q_j: np.ndarray | None = None
    qd_j: np.ndarray | None = None
    ee_xyz: np.ndarray | None = None
    roll: float | None = None
    pitch: float | None = None
    yaw: float | None = None
    xd5: np.ndarray | None = None


def parse_noise_config(raw: dict[str, Any] | None) -> dict[str, Any]:
    return dict(raw) if raw else {"enabled": False}


class MeasurementNoiseRuntime:
    """Delay buffer + optional first-order LP; actuator gain (episode) + torque noise (step)."""

    def __init__(
        self,
        noise_cfg: dict[str, Any],
        rng: np.random.Generator,
        *,
        control_dt: float,
        cutoff_hz: float,
        actuator_ratios: np.ndarray,
    ):
        self._ratios = np.asarray(actuator_ratios, dtype=np.float64).reshape(4)
        self.rng = rng
        self.dt = float(control_dt)
        self.master = bool(noise_cfg.get("enabled", False))

        sens = noise_cfg.get("sensor") or {}
        self.sensor_on = self.master and bool(sens.get("enabled", False))
        self.q_std = float(sens.get("q_std_rad", 0.0))
        self.qd_std = float(sens.get("qdot_std_rad_s", 0.0))
        self.ee_pos_std = float(sens.get("ee_pos_std_m", 0.0))
        self.ee_rp_std = float(sens.get("ee_rp_std_rad", 0.0))
        self.ee_vel_std = float(sens.get("ee_vel_std_m_s", 0.0))
        self.ee_omega_std = float(sens.get("ee_omega_std_rad_s", 0.0))

        md = noise_cfg.get("measurement_delay") or {}
        self.delay_on = self.master and bool(md.get("enabled", False))
        self.delay_steps = int(md.get("delay_steps", 0))

        mf = noise_cfg.get("measurement_filter") or {}
        self.filter_on = self.master and bool(mf.get("enabled", False))
        chz = float(mf.get("cutoff_hz", cutoff_hz))
        tau = 1.0 / (2.0 * np.pi * max(chz, 1e-6))
        self._alpha = float(self.dt / (tau + self.dt))
        self._cutoff_hz_logged = chz

        act = noise_cfg.get("actuator") or {}
        self.actuator_on = self.master and bool(act.get("enabled", False))
        self.torque_std = float(act.get("torque_std_nm", 0.0))
        self.gain_std = float(act.get("gain_std", 0.0))

        rw = noise_cfg.get("random_walk_bias") or {}
        self.rw_on = self.master and bool(rw.get("enabled", False))
        self.rw_pos_std = float(rw.get("ee_pos_bias_std_m", 0.0))
        self.rw_rp_std = float(rw.get("ee_rp_bias_std_rad", 0.0))
        self.rw_decay = float(rw.get("bias_decay", 0.995))

        self._history: deque[MeasPacket] = deque(maxlen=512)
        self._filt = _FilterState()
        self.filt_meas: MeasPacket | None = None
        self.raw_latest: MeasPacket | None = None
        self.delayed_latest: MeasPacket | None = None

        self._bias_ee = np.zeros(3, dtype=np.float64)
        self._bias_rp = np.zeros(2, dtype=np.float64)
        self._gain_episode = 1.0

    def log_settings(self) -> dict[str, Any]:
        return {
            "noise_master": self.master,
            "sensor_on": self.sensor_on,
            "delay_on": self.delay_on,
            "delay_steps": int(self.delay_steps if self.delay_on else 0),
            "filter_on": self.filter_on,
            "cutoff_hz": float(self._cutoff_hz_logged) if self.filter_on else float("nan"),
            "actuator_on": self.actuator_on,
            "torque_std_nm": float(self.torque_std),
            "gain_std": float(self.gain_std),
            "random_walk_on": self.rw_on,
        }

    def reset_episode(self) -> None:
        self._history.clear()
        self._filt = _FilterState()
        self.filt_meas = None
        self.raw_latest = None
        self.delayed_latest = None
        self._bias_ee[:] = 0.0
        self._bias_rp[:] = 0.0
        if self.actuator_on and self.gain_std > 0.0:
            self._gain_episode = float(1.0 + self.rng.normal(0.0, self.gain_std))
        else:
            self._gain_episode = 1.0

    def update_random_walk(self) -> None:
        if not self.rw_on:
            return
        self._bias_ee = self._bias_ee * self.rw_decay + self.rng.normal(0.0, self.rw_pos_std, 3)
        self._bias_rp = self._bias_rp * self.rw_decay + self.rng.normal(0.0, self.rw_rp_std, 2)

    def apply_sensor_white(
        self,
        q_j: np.ndarray,
        qd_j: np.ndarray,
        ee_xyz: np.ndarray,
        roll: float,
        pitch: float,
        yaw: float,
        xd5: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, float, float, np.ndarray]:
        q_j = np.asarray(q_j, dtype=np.float64).reshape(4).copy()
        qd_j = np.asarray(qd_j, dtype=np.float64).reshape(4).copy()
        ee = np.asarray(ee_xyz, dtype=np.float64).reshape(3).copy()
        xd = np.asarray(xd5, dtype=np.float64).reshape(5).copy()
        r, p, y = float(roll), float(pitch), float(yaw)
        if self.sensor_on:
            q_j = q_j + self.rng.normal(0.0, self.q_std, 4)
            qd_j = qd_j + self.rng.normal(0.0, self.qd_std, 4)
            ee = ee + self.rng.normal(0.0, self.ee_pos_std, 3)
            r = float(r + self.rng.normal(0.0, self.ee_rp_std))
            p = float(p + self.rng.normal(0.0, self.ee_rp_std))
            xd[:3] = xd[:3] + self.rng.normal(0.0, self.ee_vel_std, 3)
            xd[3] = float(xd[3] + self.rng.normal(0.0, self.ee_omega_std))
            xd[4] = float(xd[4] + self.rng.normal(0.0, self.ee_omega_std))
        return q_j, qd_j, ee, r, p, y, xd

    def add_ee_bias(self, ee_xyz: np.ndarray, roll: float, pitch: float) -> tuple[np.ndarray, float, float]:
        ee = np.asarray(ee_xyz, dtype=np.float64).reshape(3).copy()
        if self.rw_on:
            ee = ee + self._bias_ee
            roll = float(roll + self._bias_rp[0])
            pitch = float(pitch + self._bias_rp[1])
        return ee, roll, pitch

    def assemble_packet(
        self,
        q_j: np.ndarray,
        qd_j: np.ndarray,
        ratios: np.ndarray,
        ee_xyz: np.ndarray,
        roll: float,
        pitch: float,
        yaw: float,
        xd5: np.ndarray,
    ) -> MeasPacket:
        q_j = np.asarray(q_j, dtype=np.float64).reshape(4)
        qd_j = np.asarray(qd_j, dtype=np.float64).reshape(4)
        r = np.asarray(ratios, dtype=np.float64).reshape(4)
        qa = (q_j / r).astype(np.float64)
        qda = (qd_j / r).astype(np.float64)
        return MeasPacket(
            q_j=q_j.copy(),
            qd_j=qd_j.copy(),
            q_a=qa,
            qd_a=qda,
            ee_xyz=np.asarray(ee_xyz, dtype=np.float64).reshape(3).copy(),
            roll=float(roll),
            pitch=float(pitch),
            yaw=float(yaw),
            xd5=np.asarray(xd5, dtype=np.float64).reshape(5).copy(),
        )

    def _pick_delayed(self) -> MeasPacket:
        if not self._history:
            raise RuntimeError("measurement history empty")
        d = int(self.delay_steps) if self.delay_on else 0
        if d <= 0:
            return self._history[-1]
        idx = len(self._history) - 1 - d
        if idx < 0:
            return self._history[0]
        return self._history[idx]

    def _lp_vec(self, prev: np.ndarray | None, target: np.ndarray) -> np.ndarray:
        t = np.asarray(target, dtype=np.float64)
        if prev is None or not self.filter_on:
            return t.copy()
        p = np.asarray(prev, dtype=np.float64)
        return p + self._alpha * (t - p)

    def _lp_scalar(self, prev: float | None, target: float) -> float:
        if prev is None or not self.filter_on:
            return float(target)
        return float(prev + self._alpha * (float(target) - prev))

    def filter_apply(self, p: MeasPacket) -> MeasPacket:
        self.delayed_latest = p
        fq = self._lp_vec(self._filt.q_j, p.q_j)
        fqd_j = self._lp_vec(self._filt.qd_j, p.qd_j)
        fee = self._lp_vec(self._filt.ee_xyz, p.ee_xyz)
        fr = self._lp_scalar(self._filt.roll, p.roll)
        fp = self._lp_scalar(self._filt.pitch, p.pitch)
        fy = self._lp_scalar(self._filt.yaw, p.yaw)
        fxd = self._lp_vec(self._filt.xd5, p.xd5)
        self._filt.q_j = fq
        self._filt.qd_j = fqd_j
        self._filt.ee_xyz = fee
        self._filt.roll = fr
        self._filt.pitch = fp
        self._filt.yaw = fy
        self._filt.xd5 = fxd
        qa = (fq / self._ratios).astype(np.float64)
        qda = (fqd_j / self._ratios).astype(np.float64)
        return MeasPacket(
            q_j=fq,
            qd_j=fqd_j,
            q_a=qa,
            qd_a=qda,
            ee_xyz=fee,
            roll=fr,
            pitch=fp,
            yaw=fy,
            xd5=fxd,
        )

    def push_raw(self, raw: MeasPacket) -> MeasPacket:
        self.raw_latest = raw
        self._history.append(raw)
        delayed = self._pick_delayed()
        self.filt_meas = self.filter_apply(delayed)
        return self.filt_meas

    def actuator_torques(self, tau_act_ideal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        ideal = np.asarray(tau_act_ideal, dtype=np.float64).reshape(4).copy()
        if not self.actuator_on:
            return ideal, ideal.copy()
        noisy = ideal * self._gain_episode + self.rng.normal(0.0, self.torque_std, 4)
        return ideal, noisy
