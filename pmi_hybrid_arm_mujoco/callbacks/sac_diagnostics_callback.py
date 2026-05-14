"""Periodic SAC eval, rollout stats, checkpoints, optional early-stop (physics unchanged)."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Sequence

import yaml

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from envs.pmi_cable_residual_env import PMICableResidualEnv, _deep_merge
from utils.smooth_tracking_metrics import episode_smooth_tracking_metrics, smooth_tracking_aggregate_score


ACTION_L2_MAX = float(np.sqrt(3.0))

META_FIELDS = ("run_name", "seed", "profile", "eval_type")

EVAL_FIELDS = META_FIELDS + (
    "global_step",
    "utc_iso",
    "mean_episode_return",
    "std_episode_return",
    "mean_episode_length",
    "mean_ee_rms",
    "std_ee_rms_mean_across_eps",
    "mean_sat_frac",
    "mean_lim_frac",
    "frac_eps_any_ncon",
    "mean_q_err_rms",
    "mean_finite_frac",
    "mean_action_norm",
    "max_action_norm",
    "mean_residual_force_norm",
    "max_residual_force_norm",
    "mean_residual_tau_norm",
    "max_residual_tau_norm",
    "mean_action_rate",
    "mean_final_ee_error",
    "mean_combined_tracking_score",
    "mean_smooth_tracking_score",
    "mean_rms_ee_error_velocity",
    "mean_rms_ee_error_acceleration",
    "mean_tv_ee_error",
    "mean_tv_ee_error_norm",
    "mean_rms_ee_error_highfreq",
    "mean_p2p_error_norm",
    "mean_rms_residual_force_rate",
    "mean_rms_residual_force_jerk",
    "mean_rms_tau_total_rate",
    "mean_max_tau_total_rate",
    "mean_rms_residual_tau_rate",
)

ROLLOUT_FIELDS = META_FIELDS + (
    "global_step_after_rollout",
    "utc_iso",
    "rollout_steps",
    "mean_sat_frac",
    "mean_lim_frac",
    "mean_action_l2_ratio",
    "mean_max_abs_action",
)

CALLBACK_FIELDS = META_FIELDS + ("global_step", "utc_iso", "logger_keys_concat")


def clear_run_csv_logs(log_dir: Path, *, resume: bool) -> None:
    """새 런에서 이전 런과 CSV가 섞이지 않도록 삭제. ``--resume-from`` 사용 시 유지."""
    if resume:
        return
    log_dir = Path(log_dir)
    for fn in (
        "eval_log.csv",
        "callback_metrics.csv",
        "rollout_diagnostics.csv",
        "train_monitor.csv",
        "monitor.csv",
        "best_metrics.yaml",
        "stop_status.yaml",
    ):
        p = log_dir / fn
        if p.is_file():
            p.unlink()
    # SB3 Monitor 기본 이름
    for p in log_dir.glob("*.monitor.csv"):
        p.unlink(missing_ok=True)


def _utc_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _write_best_metrics_yaml(
    path: Path,
    *,
    best_reward: float | None,
    best_reward_step: int | None,
    best_ee_rms: float | None,
    best_ee_rms_step: int | None,
    best_final_ee_error: float | None,
    best_saturation_fraction: float | None,
    best_limit_violation_fraction: float | None,
    best_combined_tracking_score: float | None,
    best_combined_tracking_step: int | None,
    best_smooth_tracking_score: float | None,
    best_smooth_tracking_step: int | None,
    best_smooth_tracking_metrics: dict[str, Any] | None,
    best_relative_smooth_score: float | None = None,
    best_relative_smooth_step: int | None = None,
    best_relative_smooth_metrics: dict[str, Any] | None = None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload: dict[str, Any] = {
        "best_reward": best_reward,
        "best_reward_step": best_reward_step,
        "best_ee_rms": best_ee_rms,
        "best_ee_rms_step": best_ee_rms_step,
        "best_final_ee_error": best_final_ee_error,
        "best_saturation_fraction": best_saturation_fraction,
        "best_limit_violation_fraction": best_limit_violation_fraction,
        "best_combined_tracking_score": best_combined_tracking_score,
        "best_combined_tracking_step": best_combined_tracking_step,
        "best_smooth_tracking_score": best_smooth_tracking_score,
        "best_smooth_tracking_step": best_smooth_tracking_step,
        "best_smooth_tracking_metrics": best_smooth_tracking_metrics or {},
        "best_relative_smooth_score": best_relative_smooth_score,
        "best_relative_smooth_step": best_relative_smooth_step,
        "best_relative_smooth_metrics": best_relative_smooth_metrics or {},
    }
    path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")


def _csv_append(path: Path, fields: tuple[str, ...], row: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    new_file = not path.is_file()
    with path.open("a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(fields))
        if new_file:
            w.writeheader()
        w.writerow({k: row.get(k, "") for k in fields})


def write_eval_baseline_row(
    path: Path,
    *,
    run_name: str,
    seed: int | None,
    profile: str,
    metrics: dict[str, float],
) -> None:
    row: dict[str, Any] = {
        "run_name": run_name,
        "seed": "" if seed is None else str(int(seed)),
        "profile": profile,
        "eval_type": "baseline_zero",
        "global_step": 0,
        "utc_iso": _utc_iso(),
    }
    metric_keys = (
        "mean_episode_return",
        "std_episode_return",
        "mean_episode_length",
        "mean_ee_rms",
        "std_ee_rms_mean_across_eps",
        "mean_sat_frac",
        "mean_lim_frac",
        "frac_eps_any_ncon",
        "mean_q_err_rms",
        "mean_finite_frac",
        "mean_action_norm",
        "max_action_norm",
        "mean_residual_force_norm",
        "max_residual_force_norm",
        "mean_residual_tau_norm",
        "max_residual_tau_norm",
        "mean_action_rate",
        "mean_final_ee_error",
        "mean_combined_tracking_score",
        "mean_smooth_tracking_score",
        "mean_rms_ee_error_velocity",
        "mean_rms_ee_error_acceleration",
        "mean_tv_ee_error",
        "mean_tv_ee_error_norm",
        "mean_rms_ee_error_highfreq",
        "mean_p2p_error_norm",
        "mean_rms_residual_force_rate",
        "mean_rms_residual_force_jerk",
        "mean_rms_tau_total_rate",
        "mean_max_tau_total_rate",
        "mean_rms_residual_tau_rate",
    )
    for k in metric_keys:
        row[k] = metrics.get(k, "")
    _csv_append(path, EVAL_FIELDS, row)


def make_env_factory(
    config_path: Path,
    profile: str,
    config_overrides: dict[str, Any] | None = None,
) -> Callable[[], PMICableResidualEnv]:
    base = {"env": {"randomization_profile": str(profile), "randomize_cable": True}}
    merged = _deep_merge(base, config_overrides) if config_overrides else base

    def _make() -> PMICableResidualEnv:
        return PMICableResidualEnv(config_path=config_path, overrides=merged)

    return _make


def rollout_one_episode_vec(
    *,
    obs_policy: Callable[[np.ndarray], np.ndarray],
    config_path: Path,
    profile: str,
    seed: int,
    vec_normalize_path: Path | None,
    config_overrides: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Single episode on one DummyVecEnv; fresh env factory so reset(seed, options) is honored."""

    mk = make_env_factory(config_path, profile, config_overrides)
    v: Any = DummyVecEnv([mk])
    if vec_normalize_path is not None and Path(vec_normalize_path).is_file():
        v = VecNormalize.load(str(vec_normalize_path), v)
        v.training = False
        v.norm_reward = False

    v.seed(int(seed))
    v.set_options({"randomization_profile": str(profile), "randomize_cable": True})
    dt = float(v.get_attr("control_dt")[0])
    obs = v.reset()

    ret = 0.0
    ee: list[float] = []
    qe: list[float] = []
    exyz: list[np.ndarray] = []
    fn_raw: list[np.ndarray] = []
    f_used_ser: list[np.ndarray] = []
    f_filtered_ser: list[np.ndarray] = []
    gates_l: list[float] = []
    times_l: list[float] = []
    ee_hf_xyz: list[np.ndarray] = []
    ee_dot_ser: list[np.ndarray] = []
    ee_des_ser: list[np.ndarray] = []
    ee_act_ser: list[np.ndarray] = []
    taures: list[np.ndarray] = []
    tautot: list[np.ndarray] = []

    sat_steps = 0
    lim_steps = 0
    steps = 0
    saw_ncon = False
    ncon_peak = 0
    any_nan_obs = False
    finite = True

    anorms: list[float] = []
    axmax: list[float] = []
    fnorms: list[float] = []
    fxmax: list[float] = []
    tnlist: list[float] = []
    ttmax: list[float] = []
    frates: list[float] = []
    f_prev_vec = np.zeros(3, dtype=np.float64)

    while True:
        if not np.isfinite(obs).all():
            any_nan_obs = True
            finite = False
            break
        act = obs_policy(obs)
        if isinstance(act, tuple):
            act = act[0]
        aflat = np.asarray(act, dtype=np.float64).reshape(-1)
        anorms.append(float(np.linalg.norm(aflat)))
        axmax.append(float(np.max(np.abs(aflat))))
        obs, rew, done, infos = v.step(act)
        ret += float(rew[0])
        inf = infos[0]
        F_used = np.asarray(inf["F_residual_xyz"], dtype=np.float64).reshape(3)
        F_raw_inf = np.asarray(inf.get("F_residual_raw_xyz", F_used), dtype=np.float64).reshape(3)
        F_filt_inf = np.asarray(inf.get("F_residual_filtered_xyz", F_used), dtype=np.float64).reshape(3)
        gate_v = float(inf.get("residual_gate", 1.0))

        tau = np.asarray(inf["tau_residual_jnt"], dtype=np.float64).reshape(4)
        tau_jnt = np.asarray(inf["tau_jnt_cmd"], dtype=np.float64).reshape(4)
        ee_v = np.asarray(inf["ee_err_xyz"], dtype=np.float64).reshape(3)

        fnorms.append(float(np.linalg.norm(F_used)))
        fxmax.append(float(np.max(np.abs(F_used))))
        tnlist.append(float(np.linalg.norm(tau)))
        ttmax.append(float(np.max(np.abs(tau))))
        frates.append(float(np.linalg.norm(F_used - f_prev_vec)))
        f_prev_vec = F_used.copy()

        exyz.append(ee_v)
        fn_raw.append(F_raw_inf)
        f_used_ser.append(F_used.copy())
        f_filtered_ser.append(F_filt_inf.copy())
        gates_l.append(gate_v)
        times_l.append(float(inf.get("time", float("nan"))))
        ee_hf_xyz.append(np.asarray(inf["ee_err_highfreq_xyz"], dtype=np.float64).reshape(3).copy())
        ee_dot_ser.append(np.asarray(inf["ee_dot"], dtype=np.float64).reshape(3).copy())
        ee_des_ser.append(np.asarray(inf["ee_des_xyz"], dtype=np.float64).reshape(3).copy())
        ee_act_ser.append(np.asarray(inf["ee_act_xyz"], dtype=np.float64).reshape(3).copy())
        taures.append(tau.copy())
        tautot.append(tau_jnt.copy())

        ee.append(float(inf.get("ee_error_norm", np.nan)))
        qe.append(float(inf.get("q_error_norm", np.nan)))
        sat_steps += int(bool(inf.get("saturation_count")))
        jl = int(inf.get("joint_limit_violation", 0))
        al = int(inf.get("actuator_limit_violation", 0))
        if jl or al:
            lim_steps += 1
        nc = int(inf.get("ncon", 0))
        ncon_peak = max(ncon_peak, nc)
        saw_ncon = saw_ncon or nc > 0
        finite = finite and np.isfinite(obs).all() and np.isfinite(float(rew[0]))
        steps += 1
        if bool(done[0]):
            break

    ee_a = np.asarray(ee, dtype=np.float64)
    q_a = np.asarray(qe, dtype=np.float64)
    rms_ee = float(np.sqrt(np.nanmean(ee_a**2))) if ee_a.size else float("nan")
    final_ee_err = float(ee[-1]) if ee else float("nan")
    max_ee_err = float(np.nanmax(ee_a)) if ee_a.size else float("nan")
    rms_q = float(np.sqrt(np.nanmean(q_a**2))) if q_a.size else float("nan")

    fr_sat = sat_steps / max(1, steps)
    fr_lim = lim_steps / max(1, steps)

    ee_xyz_arr = np.stack(exyz, axis=0) if exyz else np.zeros((0, 3), dtype=np.float64)
    F_raw_arr = np.stack(fn_raw, axis=0) if fn_raw else None
    F_used_arr = np.stack(f_used_ser, axis=0) if f_used_ser else None
    F_filt_arr = np.stack(f_filtered_ser, axis=0) if f_filtered_ser else None
    tau_res_arr = np.stack(taures, axis=0) if taures else None
    tau_tot_arr = np.stack(tautot, axis=0) if tautot else None

    osc = episode_smooth_tracking_metrics(
        ee_err_xyz=ee_xyz_arr,
        ee_err_norm=ee_a,
        dt=float(dt),
        F_res=F_used_arr,
        F_res_raw=F_raw_arr,
        tau_residual=tau_res_arr,
        tau_total=tau_tot_arr,
        smooth_window_sec=0.10,
    )
    osc["smooth_tracking_score"] = smooth_tracking_aggregate_score(osc, rms_ee=rms_ee)
    osc.pop("rms_ee", None)

    v.close()
    n_s = int(steps)
    t_arr = np.asarray(times_l, dtype=np.float64) if times_l else np.zeros(0, dtype=np.float64)
    gate_arr = np.asarray(gates_l, dtype=np.float64) if gates_l else np.zeros(0, dtype=np.float64)
    ehf_arr = np.stack(ee_hf_xyz, axis=0) if ee_hf_xyz else np.zeros((0, 3), dtype=np.float64)
    edot_arr = np.stack(ee_dot_ser, axis=0) if ee_dot_ser else np.zeros((0, 3), dtype=np.float64)
    edes_arr = np.stack(ee_des_ser, axis=0) if ee_des_ser else np.zeros((0, 3), dtype=np.float64)
    eact_arr = np.stack(ee_act_ser, axis=0) if ee_act_ser else np.zeros((0, 3), dtype=np.float64)
    return {
        "episode_return": ret,
        "episode_length": float(steps),
        "rms_ee": rms_ee,
        "final_ee_error": final_ee_err,
        "max_ee_error": max_ee_err,
        "rms_q": rms_q,
        "sat_frac": fr_sat,
        "lim_frac": fr_lim,
        "any_ncon": saw_ncon,
        "ncon_max": int(ncon_peak),
        "finite": finite,
        "nan_obs_early": bool(any_nan_obs),
        "mean_action_norm": float(np.mean(anorms)) if anorms else float("nan"),
        "max_action_norm": float(np.max(axmax)) if axmax else float("nan"),
        "mean_residual_force_norm": float(np.mean(fnorms)) if fnorms else float("nan"),
        "max_residual_force_norm": float(np.max(fxmax)) if fxmax else float("nan"),
        "mean_residual_tau_norm": float(np.mean(tnlist)) if tnlist else float("nan"),
        "max_residual_tau_norm": float(np.max(ttmax)) if ttmax else float("nan"),
        "mean_action_rate": float(np.mean(frates)) if frates else float("nan"),
        "control_dt": float(dt),
        "n_steps": n_s,
        "t_control": t_arr,
        "residual_gate_series": gate_arr,
        "F_residual_raw_series": F_raw_arr,
        "F_residual_filtered_series": F_filt_arr,
        "F_residual_used_series": F_used_arr,
        "ee_err_xyz_series": ee_xyz_arr,
        "ee_err_highfreq_xyz_series": ehf_arr,
        "ee_dot_xyz_series": edot_arr,
        "ee_des_xyz_series": edes_arr,
        "ee_act_xyz_series": eact_arr,
        "tau_residual_series": tau_res_arr,
        "tau_total_series": tau_tot_arr,
        **osc,
    }


def evaluate_policy_vec(
    obs_policy: Callable[[np.ndarray], np.ndarray],
    *,
    config_path: Path,
    profile: str,
    n_episodes: int,
    seed_base: int,
    vec_normalize_path: Path | None,
    config_overrides: dict[str, Any] | None = None,
    episode_seeds: Sequence[int] | None = None,
) -> dict[str, float]:
    """If ``episode_seeds`` is set (length ``n_episodes``), those env seeds are used (reproducible eval)."""
    n = int(n_episodes)
    if episode_seeds is not None:
        seeds_list = [int(x) for x in episode_seeds]
        if len(seeds_list) != n:
            raise ValueError(f"episode_seeds length {len(seeds_list)} != n_episodes {n}")
    else:
        rng = np.random.default_rng(int(seed_base))
        seeds_list = [int(rng.integers(0, 2**31 - 1)) for _ in range(n)]

    eps_ret: list[float] = []
    eps_len: list[float] = []
    ee_rmss: list[float] = []
    q_rmss: list[float] = []
    sat_fracs: list[float] = []
    lim_fracs: list[float] = []
    ncon_flags: list[float] = []
    finite_eps: list[float] = []
    man: list[float] = []
    xa: list[float] = []
    mfn: list[float] = []
    xfn: list[float] = []
    mtn: list[float] = []
    xtn: list[float] = []
    mar: list[float] = []
    fin_ee: list[float] = []
    combined_scores: list[float] = []
    sm_sc: list[float] = []
    ev: list[float] = []
    ea: list[float] = []
    tv_ee: list[float] = []
    tv_n: list[float] = []
    hf: list[float] = []
    p2p_n: list[float] = []
    rfr: list[float] = []
    rfj: list[float] = []
    rtr: list[float] = []
    mtr: list[float] = []
    rrtau: list[float] = []

    for sid in seeds_list:
        d = rollout_one_episode_vec(
            obs_policy=obs_policy,
            config_path=config_path,
            profile=profile,
            seed=sid,
            vec_normalize_path=vec_normalize_path,
            config_overrides=config_overrides,
        )
        eps_ret.append(float(d["episode_return"]))
        eps_len.append(float(d["episode_length"]))
        ee_rmss.append(float(d["rms_ee"]))
        q_rmss.append(float(d["rms_q"]))
        sat_fracs.append(float(d["sat_frac"]))
        lim_fracs.append(float(d["lim_frac"]))
        ncon_flags.append(float(d["any_ncon"]))
        finite_eps.append(1.0 if bool(d["finite"]) else 0.0)
        man.append(float(d["mean_action_norm"]))
        xa.append(float(d["max_action_norm"]))
        mfn.append(float(d["mean_residual_force_norm"]))
        xfn.append(float(d["max_residual_force_norm"]))
        mtn.append(float(d["mean_residual_tau_norm"]))
        xtn.append(float(d["max_residual_tau_norm"]))
        mar.append(float(d["mean_action_rate"]))
        fev = float(d["final_ee_error"])
        fin_ee.append(fev)
        rmsi = float(d["rms_ee"])
        combined_scores.append(rmsi + 0.5 * fev if fev == fev else rmsi)
        sm_sc.append(float(d.get("smooth_tracking_score", float("nan"))))
        ev.append(float(d.get("rms_ee_error_velocity", float("nan"))))
        ea.append(float(d.get("rms_ee_error_acceleration", float("nan"))))
        tv_ee.append(float(d.get("tv_ee_error", float("nan"))))
        tv_n.append(float(d.get("tv_ee_error_norm", float("nan"))))
        hf.append(float(d.get("rms_ee_error_highfreq", float("nan"))))
        p2p_n.append(float(d.get("p2p_error_norm", float("nan"))))
        rfr.append(float(d.get("rms_residual_force_rate", float("nan"))))
        rfj.append(float(d.get("rms_residual_force_jerk", float("nan"))))
        rtr.append(float(d.get("rms_tau_total_rate", float("nan"))))
        mtr.append(float(d.get("max_tau_total_rate", float("nan"))))
        rrtau.append(float(d.get("rms_residual_tau_rate", float("nan"))))

    er = np.asarray(eps_ret, dtype=np.float64)
    ee = np.asarray(ee_rmss, dtype=np.float64)
    return {
        "mean_episode_return": float(np.mean(er)),
        "std_episode_return": float(np.std(er)),
        "mean_episode_length": float(np.mean(eps_len)),
        "mean_ee_rms": float(np.nanmean(ee)),
        "std_ee_rms_mean_across_eps": float(np.nanstd(ee)),
        "mean_sat_frac": float(np.mean(sat_fracs)),
        "mean_lim_frac": float(np.mean(lim_fracs)),
        "frac_eps_any_ncon": float(np.mean(ncon_flags)),
        "mean_q_err_rms": float(np.nanmean(np.asarray(q_rmss, dtype=np.float64))),
        "mean_finite_frac": float(np.mean(np.asarray(finite_eps, dtype=np.float64))) if finite_eps else float("nan"),
        "mean_action_norm": float(np.nanmean(np.asarray(man, dtype=np.float64))) if man else float("nan"),
        "max_action_norm": float(np.nanmax(np.asarray(xa, dtype=np.float64))) if xa else float("nan"),
        "mean_residual_force_norm": float(np.nanmean(np.asarray(mfn, dtype=np.float64))) if mfn else float("nan"),
        "max_residual_force_norm": float(np.nanmax(np.asarray(xfn, dtype=np.float64))) if xfn else float("nan"),
        "mean_residual_tau_norm": float(np.nanmean(np.asarray(mtn, dtype=np.float64))) if mtn else float("nan"),
        "max_residual_tau_norm": float(np.nanmax(np.asarray(xtn, dtype=np.float64))) if xtn else float("nan"),
        "mean_action_rate": float(np.nanmean(np.asarray(mar, dtype=np.float64))) if mar else float("nan"),
        "mean_final_ee_error": float(np.nanmean(np.asarray(fin_ee, dtype=np.float64))) if fin_ee else float("nan"),
        "mean_combined_tracking_score": float(np.nanmean(np.asarray(combined_scores, dtype=np.float64)))
        if combined_scores
        else float("nan"),
        "mean_smooth_tracking_score": float(np.nanmean(np.asarray(sm_sc, dtype=np.float64))) if sm_sc else float("nan"),
        "mean_rms_ee_error_velocity": float(np.nanmean(np.asarray(ev, dtype=np.float64))) if ev else float("nan"),
        "mean_rms_ee_error_acceleration": float(np.nanmean(np.asarray(ea, dtype=np.float64))) if ea else float("nan"),
        "mean_tv_ee_error": float(np.nanmean(np.asarray(tv_ee, dtype=np.float64))) if tv_ee else float("nan"),
        "mean_tv_ee_error_norm": float(np.nanmean(np.asarray(tv_n, dtype=np.float64))) if tv_n else float("nan"),
        "mean_rms_ee_error_highfreq": float(np.nanmean(np.asarray(hf, dtype=np.float64))) if hf else float("nan"),
        "mean_p2p_error_norm": float(np.nanmean(np.asarray(p2p_n, dtype=np.float64))) if p2p_n else float("nan"),
        "mean_rms_residual_force_rate": float(np.nanmean(np.asarray(rfr, dtype=np.float64))) if rfr else float("nan"),
        "mean_rms_residual_force_jerk": float(np.nanmean(np.asarray(rfj, dtype=np.float64))) if rfj else float("nan"),
        "mean_rms_tau_total_rate": float(np.nanmean(np.asarray(rtr, dtype=np.float64))) if rtr else float("nan"),
        "mean_max_tau_total_rate": float(np.nanmean(np.asarray(mtr, dtype=np.float64))) if mtr else float("nan"),
        "mean_rms_residual_tau_rate": float(np.nanmean(np.asarray(rrtau, dtype=np.float64))) if rrtau else float("nan"),
    }


@dataclass
class EarlyStopConfig:
    enabled: bool = False
    patience_evals: int = 8
    min_improvement: float = 0.01
    min_train_steps_before_stop: int = 30000
    min_return_vs_baseline: float | None = None
    saturation_margin_over_baseline: float = 0.10
    limit_margin_over_baseline: float = 0.01
    hard_max_saturation_fraction: float = 0.95
    hard_max_limit_violation_fraction: float = 0.10
    max_saturation_step_frac: float | None = None
    max_limit_step_frac: float | None = None
    max_mean_ee_error: float | None = None
    action_l2_collapse_frac: float = 0.02
    action_abs_explosion: float = 1.02
    anomaly_patience_evals: int = 4


class SACDiagnosticsCallback(BaseCallback):
    """Logs logs/eval_log.csv, logs/rollout_diagnostics.csv, logs/callback_metrics.csv.

    Saves checkpoints including ``best_model_by_smooth_tracking.zip``
    (lower ``smooth_tracking_score`` = ``mean EE RMS`` + weighted final / velocity / HF / torque rates).
    """

    def __init__(
        self,
        *,
        run_dir: Path,
        run_name: str,
        seed: int | None,
        config_path: Path,
        profile: str,
        eval_freq: int,
        eval_episodes: int,
        checkpoint_freq: int,
        vec_normalize_save_path: Path | None,
        eval_use_vec_normalize: bool,
        early: EarlyStopConfig,
        baseline_mean_return: float | None = None,
        baseline_sat_frac: float | None = None,
        baseline_lim_frac: float | None = None,
        rl_config_overrides: dict[str, Any] | None = None,
        eval_episode_seeds: Sequence[int] | None = None,
        verbose: int = 0,
    ) -> None:
        super().__init__(verbose)
        self.run_dir = Path(run_dir)
        self.run_name = str(run_name)
        self.seed_csv = "" if seed is None else str(int(seed))
        self.profile_p = str(profile)
        self.log_dir = self.run_dir / "logs"
        self.chk_dir = self.run_dir / "checkpoints"
        self.config_path = Path(config_path)
        self.profile = str(profile)
        self.eval_freq = int(eval_freq)
        self.eval_episodes = int(eval_episodes)
        self.checkpoint_freq = int(checkpoint_freq)
        self.vn_save_path = Path(vec_normalize_save_path) if vec_normalize_save_path else None
        self.eval_use_vn = bool(eval_use_vec_normalize)
        self.early = early
        self.baseline_mean_ret = baseline_mean_return
        self.baseline_sat_frac = baseline_sat_frac
        self.baseline_lim_frac = baseline_lim_frac
        self.rl_cfg_ov = rl_config_overrides
        seeds = [int(x) for x in eval_episode_seeds] if eval_episode_seeds is not None else None
        if seeds is not None and len(seeds) != self.eval_episodes:
            raise ValueError(f"eval_episode_seeds length {len(seeds)} != eval_episodes {self.eval_episodes}")
        self._eval_episode_seeds: list[int] | None = seeds

        br = self.rl_cfg_ov.get("baseline_relative_reward") if isinstance(self.rl_cfg_ov, dict) else None
        self._track_relative_smooth_checkpoint = bool((br or {}).get("enabled", False))
        self._best_relative_smooth = float("inf")
        self._best_relative_smooth_step: int | None = None
        self._best_relative_smooth_metrics: dict[str, Any] | None = None

        self._last_eval_bucket = -1
        self._running_eval_best_ret = float("-inf")
        self._best_mean_ret = float("-inf")
        self._best_reward_step: int | None = None
        self._best_ee_rms = float("inf")
        self._best_ee_rms_step: int | None = None
        self._best_final_ee_error: float | None = None
        self._best_saturation_fraction: float | None = None
        self._best_limit_violation_fraction: float | None = None
        self._best_combined_tracking = float("inf")
        self._best_combined_tracking_step: int | None = None
        self._best_smooth_tracking = float("inf")
        self._best_smooth_tracking_step: int | None = None
        self._best_smooth_tracking_metrics: dict[str, Any] | None = None
        self._no_improve_evals = 0
        self._anomaly_streak = 0

        self.stopped_reason: str | None = None
        self._rollout_sat = self._rollout_lim = self._rollout_al2 = self._rollout_amax_sum = self._rollout_steps = 0
        self._roll_bad_streak = 0
        self._last_ckpt_bucket = -1

    def _persist_best_metrics_yaml(self) -> None:
        br = float(self._best_mean_ret) if self._best_mean_ret > float("-inf") else None
        brs = self._best_reward_step
        bee = float(self._best_ee_rms) if self._best_ee_rms < float("inf") else None
        bees = self._best_ee_rms_step
        bfee = self._best_final_ee_error
        if bfee is not None and (bfee != bfee):  # nan
            bfee = None
        bsat = self._best_saturation_fraction
        blim = self._best_limit_violation_fraction
        bcomb = float(self._best_combined_tracking) if self._best_combined_tracking < float("inf") else None
        bcombs = self._best_combined_tracking_step
        bss = float(self._best_smooth_tracking) if self._best_smooth_tracking < float("inf") else None
        bsss = self._best_smooth_tracking_step
        bsm = dict(self._best_smooth_tracking_metrics) if isinstance(self._best_smooth_tracking_metrics, dict) else {}
        brls = (
            float(self._best_relative_smooth) if self._best_relative_smooth < float("inf") else None
        )
        brlss = self._best_relative_smooth_step
        brlm = dict(self._best_relative_smooth_metrics) if isinstance(self._best_relative_smooth_metrics, dict) else {}
        _write_best_metrics_yaml(
            self.log_dir / "best_metrics.yaml",
            best_reward=br,
            best_reward_step=brs,
            best_ee_rms=bee,
            best_ee_rms_step=bees,
            best_final_ee_error=bfee,
            best_saturation_fraction=bsat,
            best_limit_violation_fraction=blim,
            best_combined_tracking_score=bcomb,
            best_combined_tracking_step=bcombs,
            best_smooth_tracking_score=bss,
            best_smooth_tracking_step=bsss,
            best_smooth_tracking_metrics=bsm if bsm else None,
            best_relative_smooth_score=brls,
            best_relative_smooth_step=brlss,
            best_relative_smooth_metrics=brlm if brlm else None,
        )

    def _on_training_start(self) -> None:
        self.chk_dir.mkdir(parents=True, exist_ok=True)
        self.log_dir.mkdir(parents=True, exist_ok=True)

    def _csv_meta_eval(self, eval_type: str) -> dict[str, Any]:
        return {"run_name": self.run_name, "seed": self.seed_csv, "profile": self.profile_p, "eval_type": eval_type}

    # --- rollout aggregates ---
    def _on_rollout_start(self) -> None:
        self._rollout_sat = self._rollout_lim = self._rollout_al2 = self._rollout_amax_sum = self._rollout_steps = 0

    def _on_rollout_end(self) -> None:
        if self._rollout_steps <= 0:
            return
        rs = float(self._rollout_steps)
        mratio = (self._rollout_al2 / rs) / ACTION_L2_MAX
        mam = self._rollout_amax_sum / rs
        row = {
            **self._csv_meta_eval("train_rollout"),
            "global_step_after_rollout": int(self.num_timesteps),
            "utc_iso": _utc_iso(),
            "rollout_steps": int(self._rollout_steps),
            "mean_sat_frac": self._rollout_sat / rs,
            "mean_lim_frac": self._rollout_lim / rs,
            "mean_action_l2_ratio": mratio,
            "mean_max_abs_action": mam,
        }
        _csv_append(self.log_dir / "rollout_diagnostics.csv", ROLLOUT_FIELDS, row)
        self._rollout_early_signals(mratio, mam)

    def _update_rollout_locals(self, loc: dict[str, Any]) -> None:
        infos = loc.get("infos")
        actions = loc.get("actions")
        if infos is None or actions is None:
            return
        self._rollout_steps += 1

        acts = np.asarray(actions, dtype=np.float64)
        ai = infos[0] if isinstance(infos, (list, tuple)) and infos else infos
        if isinstance(ai, dict):
            self._rollout_sat += int(bool(ai.get("saturation_count")))
            jl = int(ai.get("joint_limit_violation", 0))
            al = int(ai.get("actuator_limit_violation", 0))
            if jl or al:
                self._rollout_lim += 1

        a0 = acts[0].reshape(-1)
        na = np.linalg.norm(a0)
        self._rollout_al2 += float(na)
        self._rollout_amax_sum += float(np.max(np.abs(a0)))

    def _logger_snapshot_row(self) -> dict[str, Any]:
        keys_txt = ""
        logger = getattr(self.model, "logger", None)
        ntov = getattr(logger, "name_to_value", None) if logger is not None else None
        if isinstance(ntov, dict) and ntov:
            parts: list[str] = []
            for k in sorted(ntov.keys())[-32:]:
                v = ntov[k]
                try:
                    if isinstance(v, (int, np.integer)):
                        parts.append(f"{k}={int(v)}")
                    elif isinstance(v, (float, np.floating)):
                        fv = float(v)
                        if np.isnan(fv) or np.isinf(fv):
                            parts.append(f"{k}=non_finite")
                        else:
                            parts.append(f"{k}={fv:.6g}")
                    else:
                        parts.append(f"{k}={str(v)[:220]}")
                except Exception:
                    parts.append(f"{k}=<?>")
            keys_txt = ";".join(parts)
        return {
            "global_step": self.num_timesteps,
            "utc_iso": _utc_iso(),
            "logger_keys_concat": keys_txt[:8000],
        }

    def _vn_path_eval(self) -> Path | None:
        if self.eval_use_vn and self.vn_save_path is not None and self.vn_save_path.is_file():
            return self.vn_save_path
        return None

    def _save_checkpoint_zip(self, name_stem: str) -> Path:
        self.chk_dir.mkdir(parents=True, exist_ok=True)
        path_prefix = self.chk_dir / name_stem
        assert self.model is not None
        self.model.save(str(path_prefix))
        return Path(str(path_prefix) + ".zip")

    def _maybe_periodic_checkpoint(self) -> None:
        if self.checkpoint_freq <= 0 or self.num_timesteps <= 0:
            return
        bucket = self.num_timesteps // self.checkpoint_freq
        if bucket <= self._last_ckpt_bucket:
            return
        self._last_ckpt_bucket = bucket
        stem = f"checkpoint_{int(self.num_timesteps)}"
        try:
            self._save_checkpoint_zip(stem)
        except Exception:
            pass

    def _maybe_periodic_eval(self) -> None:
        if self.eval_freq <= 0 or self.num_timesteps <= 0:
            return
        bucket = self.num_timesteps // self.eval_freq
        if bucket <= self._last_eval_bucket:
            return
        self._last_eval_bucket = bucket
        try:
            self._run_evaluation_once()
        except Exception as exc:
            self.stopped_reason = f"eval_failed:{exc}"

    def _evaluate_policy_metrics(self) -> dict[str, float]:
        assert self.model is not None

        def policy(obs_batch: np.ndarray) -> np.ndarray:
            a, _ = self.model.predict(obs_batch, deterministic=True)
            return np.asarray(a, dtype=np.float32)

        vn = self._vn_path_eval()
        kw: dict[str, Any] = dict(
            config_path=self.config_path,
            profile=self.profile,
            n_episodes=self.eval_episodes,
            seed_base=0,
            vec_normalize_path=vn,
            config_overrides=self.rl_cfg_ov,
        )
        if self._eval_episode_seeds is not None:
            kw["episode_seeds"] = self._eval_episode_seeds
        return evaluate_policy_vec(policy, **kw)

    def _append_eval_csv(self, metrics_f: dict[str, float], eval_type: str, global_step: int) -> None:
        fk_num = tuple(k for k in EVAL_FIELDS if k not in META_FIELDS and k not in ("global_step", "utc_iso"))
        row_eval: dict[str, Any] = {
            **self._csv_meta_eval(eval_type),
            "global_step": int(global_step),
            "utc_iso": _utc_iso(),
            **{fk: metrics_f[fk] for fk in fk_num},
        }
        _csv_append(self.log_dir / "eval_log.csv", EVAL_FIELDS, row_eval)

    def _append_callback_snap(self, global_step: int) -> None:
        row_cb = self._csv_meta_eval("callback_followup_periodic_eval")
        row_cb.update(self._logger_snapshot_row())
        row_cb["global_step"] = int(global_step)
        _csv_append(self.log_dir / "callback_metrics.csv", CALLBACK_FIELDS, row_cb)

    def _run_evaluation_once(self) -> dict[str, float]:
        metrics_f = self._evaluate_policy_metrics()
        gs = int(self.num_timesteps)
        score = float(metrics_f["mean_episode_return"])

        self._append_eval_csv(metrics_f, "periodic_eval", gs)
        self._append_callback_snap(gs)

        if score > self._running_eval_best_ret + float(self.early.min_improvement):
            self._running_eval_best_ret = float(score)
            self._no_improve_evals = 0
        else:
            self._no_improve_evals += 1

        if score > self._best_mean_ret:
            self._best_mean_ret = float(score)
            self._best_reward_step = gs
            try:
                self._save_checkpoint_zip("best_model_by_reward")
            except Exception:
                pass

        mee = float(metrics_f.get("mean_ee_rms", float("nan")))
        if mee == mee and mee < self._best_ee_rms:
            self._best_ee_rms = mee
            self._best_ee_rms_step = gs
            mfe = float(metrics_f.get("mean_final_ee_error", float("nan")))
            self._best_final_ee_error = mfe if mfe == mfe else None
            self._best_saturation_fraction = float(metrics_f["mean_sat_frac"])
            self._best_limit_violation_fraction = float(metrics_f["mean_lim_frac"])
            try:
                self._save_checkpoint_zip("best_model_by_ee_rms")
            except Exception:
                pass

        mcomb = float(metrics_f.get("mean_combined_tracking_score", float("nan")))
        if mcomb == mcomb and mcomb < self._best_combined_tracking:
            self._best_combined_tracking = mcomb
            self._best_combined_tracking_step = gs
            try:
                self._save_checkpoint_zip("best_model_by_combined_tracking")
            except Exception:
                pass

        msm = float(metrics_f.get("mean_smooth_tracking_score", float("nan")))
        if msm == msm and msm < self._best_smooth_tracking:
            self._best_smooth_tracking = msm
            self._best_smooth_tracking_step = gs
            snap_keys = (
                "mean_ee_rms",
                "mean_final_ee_error",
                "mean_rms_ee_error_velocity",
                "mean_rms_ee_error_highfreq",
                "mean_rms_tau_total_rate",
                "mean_smooth_tracking_score",
                "mean_sat_frac",
                "mean_lim_frac",
            )
            self._best_smooth_tracking_metrics = {k: metrics_f.get(k) for k in snap_keys}
            try:
                self._save_checkpoint_zip("best_model_by_smooth_tracking")
            except Exception:
                pass

        if self._track_relative_smooth_checkpoint:
            proxy_rs = float(metrics_f.get("mean_smooth_tracking_score", float("nan")))
            if proxy_rs == proxy_rs and proxy_rs < self._best_relative_smooth:
                self._best_relative_smooth = proxy_rs
                self._best_relative_smooth_step = gs
                self._best_relative_smooth_metrics = {
                    "mean_smooth_tracking_score": metrics_f.get("mean_smooth_tracking_score"),
                    "mean_ee_rms": metrics_f.get("mean_ee_rms"),
                    "mean_final_ee_error": metrics_f.get("mean_final_ee_error"),
                    "mean_rms_ee_error_velocity": metrics_f.get("mean_rms_ee_error_velocity"),
                    "mean_rms_ee_error_highfreq": metrics_f.get("mean_rms_ee_error_highfreq"),
                    "mean_rms_tau_total_rate": metrics_f.get("mean_rms_tau_total_rate"),
                    "mean_sat_frac": metrics_f.get("mean_sat_frac"),
                    "mean_lim_frac": metrics_f.get("mean_lim_frac"),
                    "relative_smooth_note": (
                        "proxy=mean_smooth_tracking_score during periodic eval "
                        "(true paired relative_smooth_score is from evaluate_sac_residual --paired-seeds)."
                    ),
                }
                try:
                    self._save_checkpoint_zip("best_model_by_relative_smooth_score")
                except Exception:
                    pass

        self._persist_best_metrics_yaml()

        self._try_eval_early_stop(score, metrics_f)
        return metrics_f

    def record_final_eval_csv(self, *, timestep_override: int | None = None) -> dict[str, float] | None:
        """평가 CSV에 ``final_eval`` 한 줄 추가(조기종료 카운터·best checkpoint 갱신 없음)."""
        if self.model is None:
            return None
        ts = timestep_override if timestep_override is not None else self.num_timesteps
        metrics_f = self._evaluate_policy_metrics()
        self._append_eval_csv(metrics_f, "final_eval", int(ts))
        return metrics_f

    def _try_eval_early_stop(self, score: float, metrics_f: dict[str, float]) -> None:
        if self.stopped_reason is not None or not self.early.enabled:
            return
        ts = int(self.num_timesteps)

        sat = float(metrics_f["mean_sat_frac"])
        lim = float(metrics_f["mean_lim_frac"])
        bs = self.baseline_sat_frac
        bl = self.baseline_lim_frac

        unhealthy_sat = (
            bs is not None and sat > float(bs) + float(self.early.saturation_margin_over_baseline)
        )
        unhealthy_lim = (
            bl is not None and lim > float(bl) + float(self.early.limit_margin_over_baseline)
        )
        hard_sat = sat > float(self.early.hard_max_saturation_fraction)
        hard_lim = lim > float(self.early.hard_max_limit_violation_fraction)

        emergency = (
            float(metrics_f["mean_finite_frac"]) < 1.0 - 1e-6
            or float(metrics_f["frac_eps_any_ncon"]) > 0.0
            or hard_sat
            or hard_lim
        )
        if emergency:
            self.stopped_reason = "early_stop:eval_emergency"
            return

        if ts < int(self.early.min_train_steps_before_stop):
            self._anomaly_streak = 0
            return

        thresh_soft = bool(unhealthy_sat or unhealthy_lim)
        if (
            self.early.max_saturation_step_frac is not None
            and sat > float(self.early.max_saturation_step_frac)
        ):
            thresh_soft = True
        if self.early.max_limit_step_frac is not None and lim > float(self.early.max_limit_step_frac):
            thresh_soft = True
        if (
            self.early.max_mean_ee_error is not None
            and float(metrics_f["mean_ee_rms"]) > float(self.early.max_mean_ee_error)
        ):
            thresh_soft = True

        baseline_fail = False
        if (
            self.baseline_mean_ret is not None
            and self.early.min_return_vs_baseline is not None
            and score < float(self.baseline_mean_ret) + float(self.early.min_return_vs_baseline)
        ):
            baseline_fail = True

        if thresh_soft or baseline_fail:
            self._anomaly_streak += 1
        else:
            self._anomaly_streak = 0

        plateau = self.early.patience_evals > 0 and self._no_improve_evals >= int(self.early.patience_evals)

        if plateau:
            self.stopped_reason = "early_stop:eval_plateau_patience"
        elif (
            self._anomaly_streak >= int(self.early.anomaly_patience_evals)
            and (thresh_soft or baseline_fail)
        ):
            self.stopped_reason = "early_stop:eval_threshold_patience"

    def _rollout_early_signals(self, mratio: float, mam: float) -> None:
        if self.stopped_reason is not None or not self.early.enabled:
            return
        ts = int(self.num_timesteps)
        if ts < int(self.early.min_train_steps_before_stop):
            return
        rs = float(self._rollout_steps)
        if rs < 32.0:
            return

        collapse = mratio <= float(self.early.action_l2_collapse_frac)
        explode = mam >= float(self.early.action_abs_explosion)
        roll_bad = collapse or explode
        if roll_bad:
            self._roll_bad_streak += 1
        else:
            self._roll_bad_streak = 0
            return

        rp = int(self.early.anomaly_patience_evals)
        if self._roll_bad_streak < rp:
            return
        if explode:
            self.stopped_reason = "early_stop:rollout_action_explosion_patience"
        elif collapse:
            self.stopped_reason = "early_stop:rollout_action_collapse_patience"

    def _on_step(self) -> bool:
        loc = getattr(self, "locals", None)
        if isinstance(loc, dict):
            self._update_rollout_locals(loc)
        self._maybe_periodic_checkpoint()
        self._maybe_periodic_eval()
        return self.stopped_reason is None

