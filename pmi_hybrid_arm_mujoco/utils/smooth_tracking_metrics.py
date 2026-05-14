"""Per-episode smooth-tracking / oscillation metrics from logged time series."""

from __future__ import annotations

import numpy as np


def _rms(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=np.float64).ravel()
    if x.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(x * x)))


def moving_average_along_axis0(x: np.ndarray, window: int) -> np.ndarray:
    """Simple trailing moving average along time (axis 0); same length as input (pads start)."""
    x = np.asarray(x, dtype=np.float64)
    if x.ndim == 1:
        x = x.reshape(-1, 1)
    t = x.shape[0]
    w = max(1, int(window))
    out = np.zeros_like(x)
    for i in range(t):
        a = max(0, i - w + 1)
        out[i] = np.mean(x[a : i + 1], axis=0)
    return out


def episode_smooth_tracking_metrics(
    *,
    ee_err_xyz: np.ndarray,
    ee_err_norm: np.ndarray | None,
    dt: float,
    F_res: np.ndarray | None = None,
    F_res_raw: np.ndarray | None = None,
    tau_residual: np.ndarray | None = None,
    tau_total: np.ndarray | None = None,
    smooth_window_sec: float = 0.10,
) -> dict[str, float]:
    """
    ee_err_xyz: (T, 3) = x_des - x_actual per control step.
    ee_err_norm: (T,) optional; if None, computed from ee_err_xyz.
    """
    e = np.asarray(ee_err_xyz, dtype=np.float64)
    if e.ndim != 2 or e.shape[1] != 3:
        raise ValueError(f"ee_err_xyz must be (T,3); got {e.shape}")
    t = e.shape[0]
    dt = float(dt)
    if t < 2 or dt <= 0.0:
        return _nan_metrics()

    if ee_err_norm is not None:
        en = np.asarray(ee_err_norm, dtype=np.float64).reshape(-1)
    else:
        en = np.linalg.norm(e, axis=1)

    # A–B derivatives (drop last diff length; RMS over shorter series is consistent)
    e_dot = np.diff(e, axis=0) / dt
    if e_dot.shape[0] == 0:
        return _nan_metrics()
    rms_ee_vel = _rms(np.linalg.norm(e_dot, axis=1))

    e_ddot = np.diff(e_dot, axis=0) / dt
    rms_ee_acc = _rms(np.linalg.norm(e_ddot, axis=1)) if e_ddot.shape[0] > 0 else float("nan")

    tv_ee = float(np.sum(np.linalg.norm(np.diff(e, axis=0), axis=1)))
    tv_norm = float(np.sum(np.abs(np.diff(en))))

    win = max(1, int(round(float(smooth_window_sec) / dt)))
    e_s = moving_average_along_axis0(e, win).reshape(-1, 3)
    e_hf = e - e_s
    rms_hf = _rms(np.linalg.norm(e_hf, axis=1))

    def p2p(a: np.ndarray) -> float:
        return float(np.max(a) - np.min(a)) if a.size else float("nan")

    p2p_x = p2p(e[:, 0])
    p2p_y = p2p(e[:, 1])
    p2p_z = p2p(e[:, 2])
    p2p_n = p2p(en)

    fin = float(en[-1]) if en.size else float("nan")
    def _rms_fdiff(arr: np.ndarray | None) -> float:
        if arr is None or arr.shape[0] < 2:
            return float("nan")
        d = np.diff(np.asarray(arr, dtype=np.float64), axis=0) / dt
        return _rms(np.linalg.norm(d, axis=1))

    def _rms_fdiff2(arr: np.ndarray | None) -> float:
        if arr is None or arr.shape[0] < 3:
            return float("nan")
        fd = np.diff(np.asarray(arr, dtype=np.float64), axis=0) / dt
        sd = np.diff(fd, axis=0) / dt
        return _rms(np.linalg.norm(sd, axis=1))

    rms_residual_force = _rms(np.linalg.norm(F_res, axis=1)) if F_res is not None and F_res.shape[0] else float("nan")
    max_residual_force = (
        float(np.max(np.linalg.norm(F_res, axis=1))) if F_res is not None and F_res.shape[0] else float("nan")
    )

    raw_for_rate = F_res_raw if F_res_raw is not None else F_res
    rms_F_rate = _rms_fdiff(raw_for_rate)
    rms_F_jerk = _rms_fdiff2(raw_for_rate)

    rms_residual_tau = (
        _rms(np.linalg.norm(tau_residual, axis=1))
        if tau_residual is not None and tau_residual.shape[0]
        else float("nan")
    )
    max_residual_tau = (
        float(np.max(np.linalg.norm(tau_residual, axis=1)))
        if tau_residual is not None and tau_residual.shape[0]
        else float("nan")
    )
    rms_residual_tau_rate = _rms_fdiff(tau_residual)

    rms_tau_total_rate = _rms_fdiff(tau_total)
    max_tau_total_rate = (
        float(np.max(np.linalg.norm(np.diff(np.asarray(tau_total, dtype=np.float64), axis=0) / dt, axis=1)))
        if tau_total is not None and tau_total.shape[0] >= 2
        else float("nan")
    )

    return {
        "rms_ee_error_velocity": rms_ee_vel,
        "rms_ee_error_acceleration": rms_ee_acc,
        "tv_ee_error": tv_ee,
        "tv_ee_error_norm": tv_norm,
        "rms_ee_error_highfreq": rms_hf,
        "p2p_error_x": p2p_x,
        "p2p_error_y": p2p_y,
        "p2p_error_z": p2p_z,
        "p2p_error_norm": p2p_n,
        "final_ee_error": fin,
        "rms_residual_force": rms_residual_force,
        "max_residual_force": max_residual_force,
        "rms_residual_force_rate": rms_F_rate,
        "rms_residual_force_jerk": rms_F_jerk,
        "rms_residual_tau": rms_residual_tau,
        "max_residual_tau": max_residual_tau,
        "rms_residual_tau_rate": rms_residual_tau_rate,
        "rms_tau_total_rate": rms_tau_total_rate,
        "max_tau_total_rate": max_tau_total_rate,
        "rms_ee": float(np.sqrt(np.mean(en**2))),
    }


def _nan_metrics() -> dict[str, float]:
    keys = [
        "rms_ee_error_velocity",
        "rms_ee_error_acceleration",
        "tv_ee_error",
        "tv_ee_error_norm",
        "rms_ee_error_highfreq",
        "p2p_error_x",
        "p2p_error_y",
        "p2p_error_z",
        "p2p_error_norm",
        "final_ee_error",
        "rms_residual_force",
        "max_residual_force",
        "rms_residual_force_rate",
        "rms_residual_force_jerk",
        "rms_residual_tau",
        "max_residual_tau",
        "rms_residual_tau_rate",
        "rms_tau_total_rate",
        "max_tau_total_rate",
        "rms_ee",
    ]
    return {k: float("nan") for k in keys}


def smooth_tracking_aggregate_score(metrics: dict[str, float], *, rms_ee: float | None = None) -> float:
    """Lower is better (mixed units; for relative ranking across checkpoints)."""
    rms = float(rms_ee) if rms_ee is not None and rms_ee == rms_ee else float(metrics.get("rms_ee", float("nan")))
    fe = float(metrics.get("final_ee_error", float("nan")))
    ev = float(metrics.get("rms_ee_error_velocity", float("nan")))
    hf = float(metrics.get("rms_ee_error_highfreq", float("nan")))
    tr = float(metrics.get("rms_tau_total_rate", float("nan")))
    if rms != rms:
        return float("inf")
    s = rms
    if fe == fe:
        s += 0.5 * fe
    if ev == ev:
        s += 0.05 * ev
    if hf == hf:
        s += 0.02 * hf
    if tr == tr:
        s += 0.001 * tr
    return float(s)
