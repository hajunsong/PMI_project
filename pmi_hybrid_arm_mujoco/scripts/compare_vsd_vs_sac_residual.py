#!/usr/bin/env python3
"""Deterministic paired comparison: VSD only vs VSD + SAC residual (task-space force).

Example::

    cd pmi_hybrid_arm_mujoco

    python scripts/compare_vsd_vs_sac_residual.py \\
      --model-path debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_100k_s2/checkpoints/best_model_by_ee_rms.zip \\
      --vecnormalize-path debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_100k_s2/vecnormalize/vecnormalize.pkl \\
      --profile medium_train \\
      --seed-start 10000 \\
      --num-episodes 5 \\
      --save-video
"""

from __future__ import annotations

import argparse
import copy
import csv
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

_ROOT = Path(__file__).resolve().parents[1]
COMPARE_ROOT_DEF = Path("debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac")
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from stable_baselines3 import SAC
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from envs.pmi_cable_residual_env import PMICableResidualEnv, _deep_merge
from utils.smooth_tracking_metrics import episode_smooth_tracking_metrics, smooth_tracking_aggregate_score

ORIENTATION_INFO_KEYS: tuple[str, ...] = (
    "desired_roll_rad",
    "desired_pitch_rad",
    "desired_yaw_rad",
    "ee_des_roll_rad_fk",
    "ee_des_pitch_rad_fk",
    "ee_des_yaw_rad_fk",
    "ee_roll_rad",
    "ee_pitch_rad",
    "ee_yaw_rad",
    "roll_error_rad",
    "pitch_error_rad",
    "roll_error_rad_vs_qdes_fk",
    "pitch_error_rad_vs_qdes_fk",
    "ee_omega_world_x",
    "ee_omega_world_y",
    "ee_omega_world_z",
)


def _safe_pearson(x: np.ndarray, y: np.ndarray) -> float:
    x = np.asarray(x, dtype=np.float64).reshape(-1)
    y = np.asarray(y, dtype=np.float64).reshape(-1)
    m = np.isfinite(x) & np.isfinite(y)
    if int(np.sum(m)) < 3:
        return float("nan")
    xa = x[m] - float(np.mean(x[m]))
    ya = y[m] - float(np.mean(y[m]))
    denom = float(np.sqrt(np.sum(xa * xa) * np.sum(ya * ya)))
    if denom < 1e-30:
        return float("nan")
    return float(np.sum(xa * ya) / denom)


def infer_run_dir(model_path: Path) -> Path | None:
    p = Path(model_path).resolve()
    return p.parent.parent if p.parent.name == "checkpoints" else None


def build_overrides(profile: str, run_dir: Path | None) -> dict[str, Any]:
    base: dict[str, Any] = {"env": {"randomization_profile": str(profile), "randomize_cable": True}}
    if run_dir is None:
        return base
    tap = Path(run_dir) / "logs" / "training_args.yaml"
    if not tap.is_file():
        return base
    raw = yaml.safe_load(tap.read_text(encoding="utf-8"))
    ro = raw.get("rl_overrides") if isinstance(raw, dict) else None
    if not isinstance(ro, dict):
        return base
    return _deep_merge(copy.deepcopy(base), ro)


def unwrap_pmi(vn: VecNormalize) -> PMICableResidualEnv:
    e = vn.venv.envs[0]
    while hasattr(e, "env") and not isinstance(e, PMICableResidualEnv):
        e = e.env
    if not isinstance(e, PMICableResidualEnv):
        raise TypeError(f"Expected PMICableResidualEnv inside VecNormalize, got {type(e)}")
    return e


def load_sac(zp_path: Path) -> SAC:
    zp_path = Path(zp_path).resolve()
    if any(part == "..." for part in zp_path.parts) or "..." in str(zp_path):
        raise FileNotFoundError(
            "SAC checkpoint path must be the full filesystem path, not `...`. "
            "Example: debug_outputs/sac_residual_task_force/runs/"
            "<run_name>/checkpoints/best_model_by_ee_rms.zip"
        )
    zp = zp_path if zp_path.suffix == ".zip" else zp_path.with_suffix(".zip")
    if zp.is_file():
        return SAC.load(str(zp), env=None)
    extra = ""
    d = zp.parent
    if d.is_dir():
        found = sorted(d.glob("*.zip"))
        if found:
            names = ", ".join(p.name for p in found[:12])
            more = "" if len(found) <= 12 else ", …"
            extra = f" Checkpoints dir `{d}` has: {names}{more}."
        else:
            extra = f" No `.zip` files in `{d}`."
    raise FileNotFoundError(f"SAC checkpoint not found: {zp}.{extra}")


def vec_options(profile: str, cable_seed: int) -> dict[str, Any]:
    return {
        "randomization_profile": str(profile),
        "randomize_cable": True,
        "cable_seed": int(cable_seed),
    }


def build_vec_normalize(cfg: Path, vn_path: Path, overrides: dict[str, Any]) -> VecNormalize:

    def factory() -> Monitor:
        return Monitor(PMICableResidualEnv(config_path=cfg, overrides=overrides))

    venv = DummyVecEnv([factory])
    vn = VecNormalize.load(str(vn_path), venv)
    vn.training = False
    vn.norm_reward = False
    return vn


def rollout_logged(
    vn: VecNormalize,
    *,
    profile: str,
    mode: str,
    cable_seed: int,
    model: SAC | None,
    capture_states: bool,
) -> tuple[list[dict[str, Any]], list[np.ndarray]]:
    vn.seed(int(cable_seed))
    vn.set_options(vec_options(profile, cable_seed))

    rows: list[dict[str, Any]] = []
    states: list[np.ndarray] = []

    obs = vn.reset()

    while True:
        if mode == "zero":
            adim = int(vn.action_space.shape[0])
            act = np.zeros((vn.num_envs, adim), dtype=np.float32)
        else:
            assert model is not None
            act, _ = model.predict(obs, deterministic=True)

        tup = vn.step(act)
        if len(tup) == 5:
            obs, rew, terminated, truncated, infos = tup
            episode_done = bool(np.asarray(terminated).reshape(-1)[0]) or bool(
                np.asarray(truncated).reshape(-1)[0]
            )
        elif len(tup) == 4:
            # SB3 VecNormalize / DummyVecEnv: dones merges terminated | truncated
            obs, rew, dones, infos = tup
            episode_done = bool(np.asarray(dones).reshape(-1)[0])
        else:
            raise RuntimeError(f"Unexpected VecEnv step arity {len(tup)}; check SB3 / gymnasium stack.")

        rew_f = float(np.asarray(rew).reshape(-1)[0])
        inf = infos[0]
        pmi = unwrap_pmi(vn)

        sac_vec = np.asarray(act.reshape(-1)[:3], dtype=np.float64)
        ee_d = np.asarray(inf["ee_des_xyz"], dtype=np.float64).reshape(3)
        ee_a = np.asarray(inf["ee_act_xyz"], dtype=np.float64).reshape(3)
        ee_e = ee_d - ee_a
        tn = float(inf["time"])
        q_j, qd_j, _, _ = pmi._read_arm_state()

        tau_vsd_nom = np.asarray(inf["tau_vsd"], dtype=np.float64).reshape(4)
        tau_clip = np.asarray(inf["tau_jnt_cmd"], dtype=np.float64).reshape(4)
        tau_res = np.asarray(inf["tau_residual_jnt"], dtype=np.float64).reshape(4)
        tau_act_ideal = np.asarray(inf["tau_act_ideal"], dtype=np.float64).reshape(4)
        tau_act_out = np.asarray(inf["tau_act_out"], dtype=np.float64).reshape(4)
        F_ap = np.asarray(inf["F_residual_xyz"], dtype=np.float64).reshape(3)
        Fr_raw = np.asarray(inf.get("F_residual_raw_xyz", F_ap), dtype=np.float64).reshape(3)
        ed = np.asarray(inf.get("ee_dot", np.zeros(3)), dtype=np.float64).reshape(3)
        ehf = np.asarray(inf.get("ee_err_highfreq_xyz", np.zeros(3)), dtype=np.float64).reshape(3)

        q_des, qd_des, *_ = pmi._ee_desired_kinematics(min(max(0.0, tn), float(pmi._traj_duration)))
        hz = np.asarray(inf["hys_z_q2q4"], dtype=np.float64).reshape(3)
        tl = np.asarray(inf["tau_loss_q2q4"], dtype=np.float64).reshape(3)
        th = np.asarray(inf["tau_hys_q2q4"], dtype=np.float64).reshape(3)
        terr = np.asarray(inf["tau_transmission_error_q2q4"], dtype=np.float64).reshape(3)

        rows.append({
            "mode": mode,
            "time": tn,
            **{f"ee_des_{axis}": float(ee_d[i]) for i, axis in enumerate("xyz")},
            **{f"ee_act_{axis}": float(ee_a[i]) for i, axis in enumerate("xyz")},
            **{f"ee_err_{axis}": float(ee_e[i]) for i, axis in enumerate("xyz")},
            "ee_err_norm": float(inf["ee_error_norm"]),
            **{f"q_jnt_{k+1}": float(q_j[k]) for k in range(4)},
            **{f"qdot_jnt_{k+1}": float(qd_j[k]) for k in range(4)},
            **{f"q_des_{k+1}": float(q_des[k]) for k in range(4)},
            **{f"qdot_des_{k+1}": float(qd_des[k]) for k in range(4)},
            "q_err_norm": float(inf["q_error_norm"]),
            **{f"sac_action_{axis}": float(sac_vec[i]) for i, axis in enumerate("xyz")},
            **{f"F_res_{axis}": float(F_ap[i]) for i, axis in enumerate("xyz")},
            **{f"F_res_raw_{axis}": float(Fr_raw[i]) for i, axis in enumerate("xyz")},
            **{f"ee_hf_{axis}": float(ehf[i]) for i, axis in enumerate("xyz")},
            **{f"ee_dot_{axis}": float(ed[i]) for i, axis in enumerate("xyz")},
            "ee_hf_norm": float(np.linalg.norm(ehf)),
            "ee_dot_norm": float(np.linalg.norm(ed)),
            "tau_res_norm": float(np.linalg.norm(tau_res)),
            "tau_vsd_norm": float(np.linalg.norm(tau_vsd_nom)),
            "tau_total_norm": float(np.linalg.norm(tau_clip)),
            **{f"tau_residual_{k+1}": float(tau_res[k]) for k in range(4)},
            **{f"tau_vsd_{k+1}": float(tau_vsd_nom[k]) for k in range(4)},
            **{f"tau_total_{k+1}": float(tau_clip[k]) for k in range(4)},
            **{f"tau_act_ideal_{k+1}": float(tau_act_ideal[k]) for k in range(4)},
            **{f"tau_act_out_{k+1}": float(tau_act_out[k]) for k in range(4)},
            **{f"tau_loss_q{k+2}": float(tl[k]) for k in range(3)},
            **{f"tau_hys_q{k+2}": float(th[k]) for k in range(3)},
            **{f"hys_z_q{k+2}": float(hz[k]) for k in range(3)},
            **{f"tau_trans_err_q{k+2}": float(terr[k]) for k in range(3)},
            "saturation_flag": int(inf["saturation_count"]),
            "joint_limit_violation": int(inf["joint_limit_violation"]),
            "actuator_limit_violation": int(inf["actuator_limit_violation"]),
            "ncon": int(inf["ncon"]),
            "reward_step": rew_f,
        })
        rlast = rows[-1]
        for ok in ORIENTATION_INFO_KEYS:
            if ok in inf:
                rlast[str(ok)] = inf[ok]

        if capture_states:
            qp = np.asarray(pmi.data.qpos, dtype=np.float64).ravel()
            qv = np.asarray(pmi.data.qvel, dtype=np.float64).ravel()
            states.append(np.concatenate([qp, qv]))

        if episode_done:
            break

    vn.close()
    return rows, states


def episode_metrics(rows: list[dict[str, Any]]) -> dict[str, float]:
    if not rows:
        return {}
    ee_n = np.array([float(r["ee_err_norm"]) for r in rows], dtype=np.float64)
    rews = np.array([float(r["reward_step"]) for r in rows], dtype=np.float64)
    n = max(1, len(rows))
    sat = np.array([float(r["saturation_flag"]) for r in rows])
    lim = np.array(
        [max(int(r["joint_limit_violation"]), int(r["actuator_limit_violation"])) for r in rows],
        dtype=float,
    )
    nc = np.array([int(r["ncon"]) for r in rows], dtype=np.float64)
    fn = np.array(
        [float(np.linalg.norm([float(r[f"F_res_{a}"]) for a in "xyz"])) for r in rows],
        dtype=np.float64,
    )
    trn = np.array([float(r["tau_res_norm"]) for r in rows], dtype=np.float64)

    ee_xyz = np.array([[float(r[f"ee_err_{a}"]) for a in "xyz"] for r in rows], dtype=np.float64)
    times = np.array([float(r["time"]) for r in rows], dtype=np.float64)
    dt = float(np.median(np.diff(times))) if len(times) > 1 else 0.01
    if not (dt > 0.0):
        dt = 0.01

    fr_stack = np.array([[float(r[f"F_res_{a}"]) for a in "xyz"] for r in rows], dtype=np.float64)
    raw_stack = np.array(
        [[float(r.get(f"F_res_raw_{a}", r[f"F_res_{a}"])) for a in "xyz"] for r in rows],
        dtype=np.float64,
    )
    tau_res_stack = np.array([[float(r[f"tau_residual_{k}"]) for k in range(1, 5)] for r in rows], dtype=np.float64)
    tau_tot_stack = np.array([[float(r[f"tau_total_{k}"]) for k in range(1, 5)] for r in rows], dtype=np.float64)

    osc = episode_smooth_tracking_metrics(
        ee_err_xyz=ee_xyz,
        ee_err_norm=ee_n,
        dt=dt,
        F_res=fr_stack,
        F_res_raw=raw_stack,
        tau_residual=tau_res_stack,
        tau_total=tau_tot_stack,
        smooth_window_sec=0.10,
    )
    rms_path = float(np.sqrt(np.mean(ee_n**2)))
    sm_score = smooth_tracking_aggregate_score(osc, rms_ee=rms_path)

    base: dict[str, float] = {
        "rms_ee_error": rms_path,
        "final_ee_error": float(ee_n[-1]),
        "max_ee_error": float(np.max(ee_n)),
        "mean_reward": float(np.mean(rews)),
        "saturation_fraction": float(np.sum(sat)) / n,
        "limit_violation_fraction": float(np.sum(lim)) / n,
        "ncon_max": int(np.max(nc)) if nc.size else 0,
        "mean_residual_force_norm": float(np.mean(fn)),
        "max_residual_force_norm": float(np.max(fn)),
        "mean_residual_torque_norm": float(np.mean(trn)),
        "max_residual_torque_norm": float(np.max(trn)),
        "smooth_tracking_score": sm_score,
    }
    for k, v in osc.items():
        if k != "rms_ee":
            base[str(k)] = float(v)

    if rows and all(k in rows[0] for k in ("roll_error_rad", "pitch_error_rad", "ee_hf_norm")):
        re = np.array([float(r["roll_error_rad"]) for r in rows], dtype=np.float64)
        pe = np.array([float(r["pitch_error_rad"]) for r in rows], dtype=np.float64)
        hf_n = np.array([float(r["ee_hf_norm"]) for r in rows], dtype=np.float64)
        abs_re = np.abs(re)
        abs_pe = np.abs(pe)
        base["rms_roll_error"] = float(np.sqrt(np.mean(re * re)))
        base["rms_pitch_error"] = float(np.sqrt(np.mean(pe * pe)))
        base["max_abs_roll_error"] = float(np.max(abs_re))
        base["max_abs_pitch_error"] = float(np.max(abs_pe))
        base["corr_ee_norm_abs_roll_err"] = _safe_pearson(ee_n, abs_re)
        base["corr_ee_norm_abs_pitch_err"] = _safe_pearson(ee_n, abs_pe)
        base["corr_hf_norm_abs_roll_err"] = _safe_pearson(hf_n, abs_re)
        base["corr_hf_norm_abs_pitch_err"] = _safe_pearson(hf_n, abs_pe)

    return base


def _plot_oscillation_pack(
    oscill_dir: Path,
    seed: int,
    rz: list[dict],
    rs: list[dict],
    metric_row: dict[str, Any],
) -> list[str]:
    """Time-series oscillation diagnostics for paired rollouts (saved under plots/oscillation/)."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return []

    oscill_dir.mkdir(parents=True, exist_ok=True)
    written: list[str] = []

    def tvec(rows: list[dict]) -> np.ndarray:
        return np.array([float(r["time"]) for r in rows], dtype=np.float64)

    def series(rows: list[dict], key: str) -> np.ndarray:
        return np.array([float(r[key]) for r in rows], dtype=np.float64)

    def stack_xyz(rows: list[dict], prefix: str) -> np.ndarray:
        return np.array([[float(r[f"{prefix}_{a}"]) for a in "xyz"] for r in rows], dtype=np.float64)

    tz, ts = tvec(rz), tvec(rs)
    dt_z = float(np.median(np.diff(tz))) if len(tz) > 1 else 0.01
    dt_s = float(np.median(np.diff(ts))) if len(ts) > 1 else 0.01
    if not (dt_z > 0.0):
        dt_z = 0.01
    if not (dt_s > 0.0):
        dt_s = 0.01

    # 1) EE error x/y/z + norm
    fig_e, axs_e = plt.subplots(4, 1, figsize=(8, 8.5), sharex=True)
    for ax_, axn in zip(axs_e[:3], "xyz"):
        ax_.plot(tz, series(rz, f"ee_err_{axn}"), color="#4477aa", linewidth=1.1, label="VSD only")
        ax_.plot(ts, series(rs, f"ee_err_{axn}"), color="#cc7733", linewidth=1.1, label="VSD+SAC")
        ax_.axhline(0.0, color="gray", linewidth=0.6, linestyle=":")
        ax_.set_ylabel(f"e_{axn} (m)")
        ax_.legend(loc="upper right", fontsize=7)
        ax_.grid(True, alpha=0.22)
    axs_e[3].plot(tz, series(rz, "ee_err_norm"), color="#4477aa", linewidth=1.2, label="VSD only")
    axs_e[3].plot(ts, series(rs, "ee_err_norm"), color="#cc7733", linewidth=1.2, label="VSD+SAC")
    axs_e[3].set_ylabel("|e| (m)")
    axs_e[3].set_xlabel("time (s)")
    axs_e[3].legend(loc="upper right", fontsize=7)
    axs_e[3].grid(True, alpha=0.22)
    fig_e.suptitle(f"EE error components + norm (seed {seed})", fontsize=11, y=1.01)
    fig_e.tight_layout()
    fp_e = oscill_dir / f"ee_err_xyz_norm_seed_{seed}.png"
    fig_e.savefig(fp_e, dpi=140, bbox_inches="tight")
    plt.close(fig_e)
    written.append(str(fp_e))

    # 2) High-frequency error norm (logged)
    fig, ax = plt.subplots(figsize=(8, 3.5))
    ax.plot(tz, series(rz, "ee_hf_norm"), color="#4477aa", label="VSD only ‖e_hf‖")
    ax.plot(ts, series(rs, "ee_hf_norm"), color="#cc7733", label="VSD+SAC ‖e_hf‖")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("‖e_highfreq‖ (m)")
    ax.set_title(f"High-frequency EE error (logged, seed {seed})")
    ax.legend()
    ax.grid(True, alpha=0.22)
    fig.tight_layout()
    fp = oscill_dir / f"ee_highfreq_norm_seed_{seed}.png"
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    written.append(str(fp))

    # 3) Error velocity norm (logged ee_dot)
    fig, ax = plt.subplots(figsize=(8, 3.5))
    ax.plot(tz, series(rz, "ee_dot_norm"), color="#4477aa", label="VSD only")
    ax.plot(ts, series(rs, "ee_dot_norm"), color="#cc7733", label="VSD+SAC")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("‖ė‖ (m/s)")
    ax.set_title(f"EE error velocity norm (seed {seed})")
    ax.legend()
    ax.grid(True, alpha=0.22)
    fig.tight_layout()
    fp = oscill_dir / f"ee_error_velocity_norm_seed_{seed}.png"
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    written.append(str(fp))

    # 4) Residual force raw vs filtered (SAC)
    Fr = stack_xyz(rs, "F_res_raw")
    Ff = stack_xyz(rs, "F_res")
    nr = np.linalg.norm(Fr, axis=1)
    nf = np.linalg.norm(Ff, axis=1)
    fig, ax = plt.subplots(figsize=(8, 3.6))
    ax.plot(ts, nr, label="‖F_res_raw‖", color="#553388", linewidth=1.2)
    ax.plot(ts, nf, label="‖F_res_filtered‖", color="#aa6633", linestyle="--", linewidth=1.2)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("‖F‖")
    ax.set_title(f"SAC residual force: raw vs filtered (seed {seed})")
    ax.legend()
    ax.grid(True, alpha=0.22)
    fig.tight_layout()
    fp = oscill_dir / f"residual_force_raw_vs_filtered_seed_{seed}.png"
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    written.append(str(fp))

    # 5–7) Rates from time series (SAC)
    def rate_norm(mat: np.ndarray, t_full: np.ndarray, dt_: float) -> tuple[np.ndarray, np.ndarray]:
        if mat.shape[0] < 2:
            return np.array([]), np.array([])
        d = np.diff(mat, axis=0) / dt_
        tn = t_full[1 : mat.shape[0]]
        return tn, np.linalg.norm(d, axis=1)

    tau_res_mat = np.array([[float(r[f"tau_residual_{k}"]) for k in range(1, 5)] for r in rs], dtype=np.float64)
    tau_tot_mat = np.array([[float(r[f"tau_total_{k}"]) for k in range(1, 5)] for r in rs], dtype=np.float64)
    tsd, rf_rate = rate_norm(Fr, ts, dt_s)
    _, tr_rate = rate_norm(tau_res_mat, ts, dt_s)
    _, tt_rate = rate_norm(tau_tot_mat, ts, dt_s)

    fig, ax = plt.subplots(figsize=(8, 3.5))
    ax.plot(tsd, rf_rate, color="#553388")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("‖dF_raw/dt‖")
    ax.set_title(f"SAC residual force rate norm (seed {seed})")
    ax.grid(True, alpha=0.22)
    fig.tight_layout()
    fp = oscill_dir / f"residual_force_rate_norm_seed_{seed}.png"
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    written.append(str(fp))

    fig, ax = plt.subplots(figsize=(8, 3.5))
    tn_tr = ts[1 : tau_res_mat.shape[0]]
    ax.plot(tn_tr, tr_rate, color="#335588")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("‖dτ_res/dt‖")
    ax.set_title(f"SAC residual joint torque rate norm (seed {seed})")
    ax.grid(True, alpha=0.22)
    fig.tight_layout()
    fp = oscill_dir / f"tau_residual_rate_norm_seed_{seed}.png"
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    written.append(str(fp))

    fig, ax = plt.subplots(figsize=(8, 3.5))
    tn_tt = ts[1 : tau_tot_mat.shape[0]]
    ax.plot(tn_tt, tt_rate, color="#226644")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("‖dτ_total/dt‖")
    ax.set_title(f"SAC total joint torque rate norm (seed {seed})")
    ax.grid(True, alpha=0.22)
    fig.tight_layout()
    fp = oscill_dir / f"tau_total_rate_norm_seed_{seed}.png"
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    written.append(str(fp))

    # 8) PSD of ‖e‖ (Welch preferred)
    def _welch_freq(y: np.ndarray, dt_: float):
        try:
            from scipy import signal as sg

            fs = 1.0 / dt_
            f, pxx = sg.welch(y, fs=fs, nperseg=min(256, max(16, len(y) // 4)))
            return f, pxx
        except Exception:
            fs = 1.0 / dt_
            y = np.asarray(y, dtype=np.float64).ravel()
            n = y.size
            if n < 16:
                return np.array([]), np.array([])
            y = y - float(np.mean(y))
            fft = np.fft.rfft(y * np.hamming(n))
            pf = np.abs(fft) ** 2 / (np.sum(np.hamming(n) ** 2) * fs * n)
            return np.fft.rfftfreq(n, d=1.0 / fs), pf

    ez_norm = series(rz, "ee_err_norm")
    es_norm = series(rs, "ee_err_norm")
    fz, pxz = _welch_freq(ez_norm, dt_z)
    fes, pxs = _welch_freq(es_norm, dt_s)
    fig, ax = plt.subplots(figsize=(8, 3.8))
    if fz.size:
        ax.semilogy(fz, pxz + 1e-20, color="#4477aa", label="VSD only")
    if fes.size:
        ax.semilogy(fes, pxs + 1e-20, color="#cc7733", label="VSD+SAC")
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("PSD (‖e‖)")
    ax.set_title(f"Spectral estimate: EE error norm (seed {seed})")
    ax.legend()
    ax.grid(True, alpha=0.25, which="both")
    fig.tight_layout()
    fp = oscill_dir / f"ee_err_norm_psd_seed_{seed}.png"
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    written.append(str(fp))

    # 9–10) Bars: peak-to-peak ‖e‖, smooth-tracking score
    p2pz = metric_row.get("zero_p2p_error_norm")
    p2ps = metric_row.get("sac_p2p_error_norm")
    try:
        zp, sp = float(p2pz), float(p2ps)
    except (TypeError, ValueError):
        zp, sp = float("nan"), float("nan")
    if zp == zp and sp == sp:
        fig, ax = plt.subplots(figsize=(5.8, 3.8))
        ax.bar([0, 1], [zp, sp], color=["#4477aa", "#cc7733"])
        ax.set_xticks([0, 1])
        ax.set_xticklabels(["VSD only", "VSD+SAC"])
        ax.set_ylabel("peak-to-peak ‖e‖ (m)")
        ax.set_title(f"Peak-to-peak error norm (seed {seed})")
        fp = oscill_dir / f"peak_to_peak_err_norm_seed_{seed}.png"
        fig.savefig(fp, dpi=140)
        plt.close(fig)
        written.append(str(fp))

    sz_sc = metric_row.get("zero_smooth_tracking_score")
    ss_sc = metric_row.get("sac_smooth_tracking_score")
    try:
        zsc, ssc = float(sz_sc), float(ss_sc)
    except (TypeError, ValueError):
        zsc, ssc = float("nan"), float("nan")
    if zsc == zsc and ssc == ssc:
        fig, ax = plt.subplots(figsize=(5.8, 3.8))
        ax.bar([0, 1], [zsc, ssc], color=["#4477aa", "#cc7733"])
        ax.set_xticks([0, 1])
        ax.set_xticklabels(["VSD only", "VSD+SAC"])
        ax.set_ylabel("smooth_tracking_score (↑ worse)")
        ax.set_title(f"Smooth tracking composite (seed {seed})")
        fp = oscill_dir / f"smooth_tracking_score_seed_{seed}.png"
        fig.savefig(fp, dpi=140)
        plt.close(fig)
        written.append(str(fp))

    return written


def _write_timeseries_csv(path: Path, seed: int, rows_z: list[dict], rows_s: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    all_rows: list[dict[str, Any]] = []
    for r in rows_z:
        all_rows.append({"seed": seed, **r})
    for r in rows_s:
        all_rows.append({"seed": seed, **r})
    if not all_rows:
        return
    fields = ["seed"] + sorted({k for r in all_rows for k in r if k != "seed"})
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in all_rows:
            w.writerow({k: r.get(k, "") for k in fields})


def _matplotlib_projection_3d_available() -> bool:
    """True if ``projection='3d'`` works (avoids mixed system/pip mpl_toolkits)."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(1.0, 1.0))
        fig.add_subplot(111, projection="3d")
        plt.close(fig)
        return True
    except Exception:
        return False


def _plot_suite(
    plot_dir: Path,
    seed: int,
    rz: list[dict],
    rs: list[dict],
    metric_row: dict[str, Any],
) -> list[str]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[compare_vsd_vs_sac] matplotlib 없음 — 플롯 생략", file=sys.stderr)
        return []

    plot_dir.mkdir(parents=True, exist_ok=True)
    written: list[str] = []

    def tvec(rows: list[dict]) -> np.ndarray:
        return np.array([float(r["time"]) for r in rows], dtype=np.float64)

    tz, ts = tvec(rz), tvec(rs)

    def series(rows: list[dict], key: str) -> np.ndarray:
        return np.array([float(r[key]) for r in rows], dtype=np.float64)

    # 1–3 EE components desired / zero / SAC
    for i, axn, lab in ((0, "x", "X"), (1, "y", "Y"), (2, "z", "Z")):
        fig, ax = plt.subplots(figsize=(8, 3.5))
        ax.plot(tz, series(rz, f"ee_des_{axn}"), "k--", label="desired", linewidth=1.2)
        ax.plot(tz, series(rz, f"ee_act_{axn}"), label="VSD only", alpha=0.85)
        ax.plot(ts, series(rs, f"ee_act_{axn}"), label="VSD+SAC", alpha=0.85)
        ax.set_xlabel("time (s)")
        ax.set_ylabel(f"EE {lab} (m)")
        ax.legend(loc="best", fontsize=8)
        ax.set_title(f"EE {lab}: desired vs zero vs SAC (seed {seed})")
        fig.tight_layout()
        fp = plot_dir / f"ee_{axn}_desired_zero_sac_seed_{seed}.png"
        fig.savefig(fp, dpi=140)
        plt.close(fig)
        written.append(str(fp))

    fig, ax = plt.subplots(figsize=(8, 3.5))
    ax.plot(tz, series(rz, "ee_err_norm"), label="zero", linewidth=1.4)
    ax.plot(ts, series(rs, "ee_err_norm"), label="SAC", linewidth=1.4)
    ax.set_title(f"EE error norm (seed {seed})")
    ax.legend()
    ax.set_xlabel("time (s)")
    ax.set_ylabel("|e| (m)")
    fp = plot_dir / f"ee_err_norm_seed_{seed}.png"
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    written.append(str(fp))

    # Time-domain path dashboard: desired / VSD-only / VSD+SAC on shared time axis (3 rows)
    fig_d, axs_d = plt.subplots(3, 1, figsize=(9, 7.2), sharex=True)
    for ax_, axn, lab in zip(axs_d, "xyz", ("X", "Y", "Z")):
        ax_.plot(tz, series(rz, f"ee_des_{axn}"), "k--", label="desired", linewidth=1.25)
        ax_.plot(tz, series(rz, f"ee_act_{axn}"), color="#4477aa", linewidth=1.15, alpha=0.92, label="VSD only")
        ax_.plot(ts, series(rs, f"ee_act_{axn}"), color="#cc7733", linewidth=1.15, alpha=0.92, label="VSD+SAC (RL)")
        ax_.set_ylabel(f"EE {lab} (m)")
        ax_.legend(loc="upper right", fontsize=7, ncol=3)
        ax_.grid(True, alpha=0.22)
    axs_d[-1].set_xlabel("time (s)")
    fig_d.suptitle(f"EE path vs time: VSD only vs VSD+SAC (seed {seed})", fontsize=11, y=1.01)
    fig_d.tight_layout()
    fp_d = plot_dir / f"ee_path_xyz_dashboard_seed_{seed}.png"
    fig_d.savefig(fp_d, dpi=140, bbox_inches="tight")
    plt.close(fig_d)
    written.append(str(fp_d))

    # XY spatial overlay (equal aspect) — path shape at a glance
    fig_xy, ax_xy = plt.subplots(figsize=(6.5, 6.2))
    ax_xy.plot(series(rz, "ee_des_x"), series(rz, "ee_des_y"), "k--", linewidth=2.0, label="desired", alpha=0.85)
    ax_xy.plot(series(rz, "ee_act_x"), series(rz, "ee_act_y"), color="#4477aa", linewidth=1.7, alpha=0.9, label="VSD only")
    ax_xy.plot(series(rs, "ee_act_x"), series(rs, "ee_act_y"), color="#cc7733", linewidth=1.7, alpha=0.9, label="VSD+SAC (RL)")
    ax_xy.set_xlabel("x (m)")
    ax_xy.set_ylabel("y (m)")
    ax_xy.set_title(f"EE path (XY projection, seed {seed})")
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.grid(True, alpha=0.25)
    ax_xy.legend(loc="best", fontsize=8)
    fig_xy.tight_layout()
    fp_xy = plot_dir / f"ee_path_overlay_xy_seed_{seed}.png"
    fig_xy.savefig(fp_xy, dpi=140)
    plt.close(fig_xy)
    written.append(str(fp_xy))

    # Tracking error: components + norm (VSD only vs VSD+SAC)
    fig_e, axs_e = plt.subplots(4, 1, figsize=(8, 8.5), sharex=True)
    for ax_, axn in zip(axs_e[:3], "xyz"):
        ax_.plot(tz, series(rz, f"ee_err_{axn}"), color="#4477aa", linewidth=1.15, label="VSD only")
        ax_.plot(ts, series(rs, f"ee_err_{axn}"), color="#cc7733", linewidth=1.15, label="VSD+SAC (RL)")
        ax_.axhline(0.0, color="gray", linewidth=0.6, linestyle=":")
        ax_.set_ylabel(f"e_{axn} (m)")
        ax_.legend(loc="upper right", fontsize=7)
        ax_.grid(True, alpha=0.22)
    axs_e[3].plot(tz, series(rz, "ee_err_norm"), color="#4477aa", linewidth=1.25, label="VSD only")
    axs_e[3].plot(ts, series(rs, "ee_err_norm"), color="#cc7733", linewidth=1.25, label="VSD+SAC (RL)")
    axs_e[3].set_ylabel("|e| (m)")
    axs_e[3].set_xlabel("time (s)")
    axs_e[3].legend(loc="upper right", fontsize=7)
    axs_e[3].grid(True, alpha=0.22)
    fig_e.suptitle(f"EE tracking error: VSD only vs VSD+SAC (seed {seed})", fontsize=11, y=1.01)
    fig_e.tight_layout()
    fp_e = plot_dir / f"ee_tracking_error_dashboard_seed_{seed}.png"
    fig_e.savefig(fp_e, dpi=140, bbox_inches="tight")
    plt.close(fig_e)
    written.append(str(fp_e))

    if _matplotlib_projection_3d_available():
        ax3 = plt.figure(figsize=(6.5, 5.8)).add_subplot(111, projection="3d")
        ax3.plot(series(rz, "ee_des_x"), series(rz, "ee_des_y"), series(rz, "ee_des_z"), "k--", label="desired", linewidth=1.2)
        ax3.plot(series(rz, "ee_act_x"), series(rz, "ee_act_y"), series(rz, "ee_act_z"), label="zero", linewidth=1.0)
        ax3.plot(series(rs, "ee_act_x"), series(rs, "ee_act_y"), series(rs, "ee_act_z"), label="SAC", linewidth=1.0)
        ax3.set_xlabel("x")
        ax3.set_ylabel("y")
        ax3.set_zlabel("z")
        ax3.set_title(f"EE path (seed {seed})")
        ax3.legend(loc="upper left", fontsize=7)
        ax3.tick_params(axis="both", labelsize=8)
        fig = plt.gcf()
        fp = plot_dir / f"ee_path_3d_seed_{seed}.png"
        fig.tight_layout()
        fig.savefig(fp, dpi=140)
        plt.close(fig)
        written.append(str(fp))
    else:
        combos = [("x", "y"), ("x", "z"), ("y", "z")]
        titles = ["XY", "XZ", "YZ"]
        fig, axs = plt.subplots(1, 3, figsize=(11, 3.9))
        for ax_, (a, b), tit in zip(axs, combos, titles):
            ax_.plot(series(rz, f"ee_des_{a}"), series(rz, f"ee_des_{b}"), "k--", label="desired", linewidth=1.2)
            ax_.plot(series(rz, f"ee_act_{a}"), series(rz, f"ee_act_{b}"), linewidth=1.0, alpha=0.9, label="zero")
            ax_.plot(series(rs, f"ee_act_{a}"), series(rs, f"ee_act_{b}"), linewidth=1.0, alpha=0.9, label="SAC")
            ax_.set_xlabel(f"{a} (m)")
            ax_.set_ylabel(f"{b} (m)")
            ax_.set_title(f"{tit} EE path (seed {seed})")
            ax_.legend(fontsize=6, loc="best")
            ax_.tick_params(axis="both", labelsize=8)
        fig.suptitle("EE path projections (3D unavailable — fix matplotlib / mpl_toolkits versions)", fontsize=9)
        fig.tight_layout()
        fp = plot_dir / f"ee_path_projection_seed_{seed}.png"
        fig.savefig(fp, dpi=140)
        plt.close(fig)
        written.append(str(fp))

    fig, ax = plt.subplots(figsize=(8, 3.8))
    for j, ln in enumerate("xyz"):
        ax.plot(ts, series(rs, f"F_res_{ln}"), label=f"F{ln}")
    ax.set_title(f"Residual force (SAC) seed {seed}")
    ax.legend()
    ax.set_xlabel("time (s)")
    ax.set_ylabel("F (scaled → N equiv.)")
    fp = plot_dir / f"residual_force_seed_{seed}.png"
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    written.append(str(fp))

    fig, ax = plt.subplots(figsize=(8, 3.8))
    for k in range(4):
        ax.plot(ts, series(rs, f"tau_residual_{k+1}"), label=f"τ_res {k+1}")
    ax.set_title(f"Residual joint torque seed {seed}")
    ax.legend(fontsize=7, ncol=2)
    fp = plot_dir / f"tau_residual_seed_{seed}.png"
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    written.append(str(fp))

    fig, ax = plt.subplots(figsize=(8, 3.8))
    ax.plot(ts, series(rs, "tau_vsd_norm"), label="‖τ_vsd‖")
    ax.plot(ts, series(rs, "tau_res_norm"), label="‖τ_residual‖")
    ax.plot(ts, series(rs, "tau_total_norm"), label="‖τ_total‖")
    ax.legend()
    ax.set_xlabel("time (s)")
    ax.set_title(f"Torque norms (SAC) seed {seed}")
    fp = plot_dir / f"tau_vsd_residual_total_seed_{seed}.png"
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    written.append(str(fp))

    fig, ax = plt.subplots(figsize=(8, 3.8))
    colors = plt.cm.tab10(np.linspace(0, 1, 4))
    for k in range(4):
        ax.plot(ts, series(rs, f"tau_act_ideal_{k+1}"), color=colors[k], linestyle="-", linewidth=1.4, alpha=0.95, label=f"ideal τ{k+1}")
        ax.plot(ts, series(rs, f"tau_act_out_{k+1}"), color=colors[k], linestyle=":", linewidth=2.0, alpha=0.8, label=f"out τ{k+1}")
    ax.set_title(f"tau_act_ideal vs tau_act_out (seed {seed})")
    ax.legend(fontsize=5, ncol=4, loc="upper right")
    fp = plot_dir / f"tau_act_ideal_minus_out_seed_{seed}.png"
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    written.append(str(fp))

    fig, ax = plt.subplots(figsize=(8, 3.8))
    for k in range(3):
        ax.plot(ts, series(rs, f"hys_z_q{k+2}"), linestyle="-", marker="", label=f"hys_z q{k+2}")
        ax.plot(ts, series(rs, f"tau_hys_q{k+2}"), linestyle="--", marker="", label=f"tau_hys q{k+2}")
    ax.legend(fontsize=7, ncol=2)
    ax.set_title(f"hys_z / tau_hys cable (SAC seed {seed})")
    fp = plot_dir / f"cable_hysteresis_seed_{seed}.png"
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    written.append(str(fp))

    labs = ["RMS EE", "final EE", "sat_frac", "lim_frac"]
    z_vals = [
        metric_row["rms_ee_zero"],
        metric_row["final_ee_zero"],
        metric_row["sat_frac_zero"],
        metric_row["lim_frac_zero"],
    ]
    s_vals = [
        metric_row["rms_ee_sac"],
        metric_row["final_ee_sac"],
        metric_row["sat_frac_sac"],
        metric_row["lim_frac_sac"],
    ]
    xv = np.arange(len(labs))
    w = 0.38
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.bar(xv - w / 2, z_vals, w, label="VSD zero", color="#4477aa")
    ax.bar(xv + w / 2, s_vals, w, label="VSD+SAC", color="#cc8844")
    ax.set_xticks(xv)
    ax.set_xticklabels(labs, fontsize=9)
    ax.legend()
    ax.set_title(f"Metrics bar seed {seed}")
    fp = plot_dir / f"metrics_bars_seed_{seed}.png"
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    written.append(str(fp))

    written.extend(_plot_oscillation_pack(plot_dir / "oscillation", seed, rz, rs, metric_row))
    return written


def _plot_aggregate(plot_dir: Path, agg_rows: list[dict[str, Any]]) -> list[str]:
    written: list[str] = []
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    deltas = np.array([float(r["delta_rms_ee"]) for r in agg_rows], dtype=float)
    fig, ax = plt.subplots(figsize=(6, 3.8))
    ax.hist(deltas, bins=min(15, max(4, len(deltas))), color="#557799", edgecolor="white")
    ax.axvline(0.0, color="crimson", linestyle="--")
    ax.set_title("Histogram: Δ RMS EE per seed")
    fp = plot_dir / "histogram_delta_rms_ee_across_seeds.png"
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    written.append(str(fp))

    m_rz = float(np.mean([float(r["rms_ee_zero"]) for r in agg_rows]))
    m_rs = float(np.mean([float(r["rms_ee_sac"]) for r in agg_rows]))
    fz = float(np.mean([float(r["final_ee_zero"]) for r in agg_rows]))
    fs = float(np.mean([float(r["final_ee_sac"]) for r in agg_rows]))
    sz = float(np.mean([float(r["sat_frac_zero"]) for r in agg_rows]))
    ss = float(np.mean([float(r["sat_frac_sac"]) for r in agg_rows]))
    lz = float(np.mean([float(r["lim_frac_zero"]) for r in agg_rows]))
    ls_s = float(np.mean([float(r["lim_frac_sac"]) for r in agg_rows]))

    xv = np.arange(4)
    w = 0.38
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    ax.bar(xv - w / 2, [m_rz, fz, sz, lz], w, label="VSD zero mean", color="#4477aa")
    ax.bar(xv + w / 2, [m_rs, fs, ss, ls_s], w, label="SAC mean", color="#cc8844")
    ax.set_xticks(xv)
    ax.set_xticklabels(["mean RMS EE", "mean final EE", "mean sat", "mean lim"])
    ax.legend()
    ax.set_title("Aggregate across seeds")
    fp = plot_dir / "metrics_bars_aggregate.png"
    fig.savefig(fp, dpi=140)
    plt.close(fig)
    written.append(str(fp))

    oscill_agg = plot_dir / "oscillation"
    oscill_agg.mkdir(parents=True, exist_ok=True)
    if agg_rows and all("delta_smooth_tracking_score" in r for r in agg_rows):
        dss = np.array([float(r["delta_smooth_tracking_score"]) for r in agg_rows], dtype=float)
        dss_ok = dss[np.isfinite(dss)]
        if dss_ok.size:
            fig, ax = plt.subplots(figsize=(6, 3.8))
            ax.hist(dss_ok, bins=min(15, max(4, dss_ok.size)), color="#884477", edgecolor="white")
            ax.axvline(0.0, color="crimson", linestyle="--")
            ax.set_title("Histogram: Δ smooth_tracking_score (SAC − VSD)")
            fp = oscill_agg / "histogram_delta_smooth_tracking_across_seeds.png"
            fig.savefig(fp, dpi=140)
            plt.close(fig)
            written.append(str(fp))

    if agg_rows and all("zero_smooth_tracking_score" in r and "sac_smooth_tracking_score" in r for r in agg_rows):
        m_sz = float(np.nanmean([float(r["zero_smooth_tracking_score"]) for r in agg_rows]))
        m_ss = float(np.nanmean([float(r["sac_smooth_tracking_score"]) for r in agg_rows]))
        fig, ax = plt.subplots(figsize=(4.5, 4.0))
        ax.bar([0, 1], [m_sz, m_ss], color=["#4477aa", "#cc7733"])
        ax.set_xticks([0, 1])
        ax.set_xticklabels(["VSD only mean", "VSD+SAC mean"])
        ax.set_ylabel("smooth_tracking_score (lower better)")
        ax.set_title("Mean smooth tracking score across seeds")
        fp = oscill_agg / "smooth_tracking_score_mean_bars_aggregate.png"
        fig.savefig(fp, dpi=140)
        plt.close(fig)
        written.append(str(fp))

    return written


def _write_video_dual(
    *,
    rz: list[dict],
    rs: list[dict],
    out_mp4: Path,
    fps: float,
    frame_stride: int = 1,
) -> bool:
    """Matplotlib EE path side-by-side animation (3D when available, else XY projection)."""

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib import animation as mpl_animation
    except Exception as exc:
        print(f"[compare_vsd_vs_sac] 영상 불가 matplotlib: {exc}", file=sys.stderr)
        return False

    xz = np.array([[float(r[f"ee_act_{x}"]) for x in "xyz"] for r in rz], dtype=float)
    xs_path = np.array([[float(r[f"ee_act_{x}"]) for x in "xyz"] for r in rs], dtype=float)
    xd = np.array([[float(r[f"ee_des_{x}"]) for x in "xyz"] for r in rz], dtype=float)

    n = min(len(xz), len(xs_path), len(xd))
    xz = xz[:n]
    xs_path = xs_path[:n]
    xd = xd[:n]

    def fr_n(idx: int) -> float:
        return float(
            np.linalg.norm([float(rs[idx]["F_res_x"]), float(rs[idx]["F_res_y"]), float(rs[idx]["F_res_z"])])
        )

    use_3d = _matplotlib_projection_3d_available()
    fig = plt.figure(figsize=(10.5, 5.0))
    if use_3d:
        ax1 = fig.add_subplot(1, 2, 1, projection="3d")
        ax2 = fig.add_subplot(1, 2, 2, projection="3d")
    else:
        ax1 = fig.add_subplot(1, 2, 1)
        ax2 = fig.add_subplot(1, 2, 2)
    ttl = fig.suptitle("")

    def init():
        return []

    k = max(1, int(frame_stride))
    frame_indices = list(range(0, n, k))

    def update(frame_idx: int):
        ax1.cla()
        ax2.cla()
        base_msg = (
            f"t={float(rz[min(frame_idx, n - 1)]['time']):.3f}s  EEerr zero={float(rz[frame_idx]['ee_err_norm']):.5f}  "
            f"SAC={float(rs[frame_idx]['ee_err_norm']):.5f}  ‖F_sac‖={fr_n(frame_idx):.4f}"
        )
        ends = slice(0, max(2, frame_idx + 1))
        axes_spec = [(ax1, xz, "#4477aa", "zero"), (ax2, xs_path, "#dd9944", "SAC")]
        if use_3d:
            for ax_, traj, clr, lbl in axes_spec:
                ax_.plot(xd[:, 0], xd[:, 1], xd[:, 2], "k--", linewidth=1.4, alpha=0.7, label="desired")
                seg = traj[ends]
                ax_.plot(seg[:, 0], seg[:, 1], seg[:, 2], color=clr, linewidth=1.6, label=lbl + " EE")
                p = traj[frame_idx]
                ax_.scatter([p[0]], [p[1]], [p[2]], color=clr, s=28)
                ax_.set_title(lbl.upper())
                ax_.legend(fontsize=6, loc="upper left")
            ttl.set_text(base_msg)
        else:
            for ax_, traj, clr, lbl in axes_spec:
                ax_.plot(xd[:, 0], xd[:, 1], "k--", linewidth=1.4, alpha=0.7, label="desired")
                seg = traj[ends]
                ax_.plot(seg[:, 0], seg[:, 1], color=clr, linewidth=1.6, label=lbl + " EE (XY)")
                p = traj[frame_idx]
                ax_.scatter([p[0]], [p[1]], color=clr, s=28)
                ax_.set_title(f"{lbl.upper()} (XY)")
                ax_.set_xlabel("x")
                ax_.set_ylabel("y")
                ax_.set_aspect("equal", adjustable="box")
                ax_.legend(fontsize=6, loc="upper left")
            ttl.set_text(base_msg + "  [side-by-side EE path XY projection]")
        plt.tight_layout(rect=[0, 0.04, 1, 0.92])
        return []

    ani = mpl_animation.FuncAnimation(fig, update, frames=frame_indices, init_func=init, interval=1000 / max(1e-6, fps))
    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    try:
        writer = mpl_animation.FFMpegWriter(fps=fps, metadata=dict(artist="PMI_compare"), bitrate=1800)
        ani.save(str(out_mp4), writer=writer)
        plt.close(fig)
        return True
    except Exception as exc:
        print(f"[compare_vsd_vs_sac] FFmpeg writer 실패: {exc}; PNG 시퀀스만 사용", file=sys.stderr)
        plt.close(fig)
        return False


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    cfg_def = _ROOT / "configs" / "rl_sac.yaml"
    out_def = COMPARE_ROOT_DEF
    ap.add_argument("--config", type=Path, default=cfg_def)
    ap.add_argument("--model-path", type=Path, required=True)
    ap.add_argument("--vecnormalize-path", type=Path, required=True)
    ap.add_argument("--profile", type=str, default="medium_train")
    ap.add_argument("--seed-start", type=int, default=10000)
    ap.add_argument("--num-episodes", type=int, default=5)
    ap.add_argument("--out-root", type=Path, default=out_def)
    ap.add_argument("--save-video", action="store_true")
    ap.add_argument("--video-fps", type=float, default=20)
    ap.add_argument(
        "--video-frame-stride",
        type=int,
        default=1,
        help="Use every k-th rollout step as an animation frame (try 3–5 for faster FFmpeg encodes).",
    )
    args = ap.parse_args()

    out_root = Path(args.out_root).resolve()
    out_root.mkdir(parents=True, exist_ok=True)
    plot_dir = out_root / "plots"
    videos_dir = out_root / "videos"
    videos_dir.mkdir(parents=True, exist_ok=True)

    run_dir = infer_run_dir(args.model_path.resolve())
    ov = build_overrides(str(args.profile), run_dir)

    cfg = Path(args.config).resolve()
    model = load_sac(Path(args.model_path).resolve())

    agg_metrics: list[dict[str, Any]] = []
    all_plot_paths: list[str] = []
    video_paths: list[str] = []

    for k in range(int(args.num_episodes)):
        ep_seed = int(args.seed_start) + k

        vn_z = build_vec_normalize(cfg, Path(args.vecnormalize_path).resolve(), ov)
        rows_z, sts_z = rollout_logged(
            vn_z, profile=str(args.profile), mode="zero", cable_seed=ep_seed, model=None, capture_states=args.save_video
        )
        vn_s = build_vec_normalize(cfg, Path(args.vecnormalize_path).resolve(), ov)
        rows_s, sts_s = rollout_logged(
            vn_s,
            profile=str(args.profile),
            mode="sac",
            cable_seed=ep_seed,
            model=model,
            capture_states=args.save_video,
        )

        ts_path = out_root / f"timeseries_seed_{ep_seed}.csv"
        _write_timeseries_csv(ts_path, ep_seed, rows_z, rows_s)

        mz = episode_metrics(rows_z)
        ms = episode_metrics(rows_s)

        agg_row: dict[str, Any] = {"seed": ep_seed}
        for k, v in mz.items():
            agg_row[f"zero_{k}"] = v
        for k, v in ms.items():
            agg_row[f"sac_{k}"] = v
        agg_row.update(
            {
                "rms_ee_zero": mz["rms_ee_error"],
                "rms_ee_sac": ms["rms_ee_error"],
                "delta_rms_ee": ms["rms_ee_error"] - mz["rms_ee_error"],
                "final_ee_zero": mz["final_ee_error"],
                "final_ee_sac": ms["final_ee_error"],
                "delta_final_ee": ms["final_ee_error"] - mz["final_ee_error"],
                "reward_zero": mz["mean_reward"],
                "reward_sac": ms["mean_reward"],
                "delta_reward": ms["mean_reward"] - mz["mean_reward"],
                "sat_frac_zero": mz["saturation_fraction"],
                "sat_frac_sac": ms["saturation_fraction"],
                "delta_saturation": ms["saturation_fraction"] - mz["saturation_fraction"],
                "lim_frac_zero": mz["limit_violation_fraction"],
                "lim_frac_sac": ms["limit_violation_fraction"],
                "delta_limit": ms["limit_violation_fraction"] - mz["limit_violation_fraction"],
            }
        )
        agg_row["zero_p2p_error_norm"] = float(mz.get("p2p_error_norm", float("nan")))
        agg_row["sac_p2p_error_norm"] = float(ms.get("p2p_error_norm", float("nan")))
        agg_row["zero_smooth_tracking_score"] = float(mz["smooth_tracking_score"])
        agg_row["sac_smooth_tracking_score"] = float(ms["smooth_tracking_score"])
        agg_row["delta_smooth_tracking_score"] = float(ms["smooth_tracking_score"] - mz["smooth_tracking_score"])
        for osc_name in (
            "rms_ee_error_velocity",
            "rms_ee_error_highfreq",
            "p2p_error_norm",
            "rms_residual_force_rate",
            "rms_tau_total_rate",
        ):
            if osc_name in mz and osc_name in ms:
                agg_row[f"delta_{osc_name}"] = float(ms[osc_name] - mz[osc_name])
        agg_metrics.append(agg_row)

        plot_paths_seed = _plot_suite(plot_dir, ep_seed, rows_z, rows_s, agg_row)
        all_plot_paths.extend(plot_paths_seed)

        if args.save_video:
            vf = videos_dir / f"vsd_vs_sac_seed_{ep_seed}.mp4"
            ok = _write_video_dual(
                rz=rows_z, rs=rows_s, out_mp4=vf, fps=float(args.video_fps), frame_stride=max(1, int(args.video_frame_stride))
            )
            if ok:
                video_paths.append(str(vf))

    met_csv = out_root / "comparison_metrics.csv"
    if agg_metrics:
        keys = sorted({kk for row in agg_metrics for kk in row})
        met_csv.parent.mkdir(parents=True, exist_ok=True)
        with met_csv.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for r in agg_metrics:
                w.writerow({k: r.get(k, "") for k in keys})

    agg_plots = _plot_aggregate(plot_dir, agg_metrics)
    all_plot_paths.extend(agg_plots)

    mean_d_rms = float(np.mean([float(r["delta_rms_ee"]) for r in agg_metrics])) if agg_metrics else float("nan")
    mean_d_fin = float(np.mean([float(r["delta_final_ee"]) for r in agg_metrics])) if agg_metrics else float("nan")

    def _mean_delta_col(name: str) -> float:
        if not agg_metrics:
            return float("nan")
        vals = [float(r[name]) for r in agg_metrics if name in r]
        return float(np.mean(vals)) if vals else float("nan")

    mean_d_smooth = _mean_delta_col("delta_smooth_tracking_score")
    mean_d_hf = _mean_delta_col("delta_rms_ee_error_highfreq")
    mean_d_evel = _mean_delta_col("delta_rms_ee_error_velocity")
    mean_d_p2p = _mean_delta_col("delta_p2p_error_norm")
    mean_d_frate = _mean_delta_col("delta_rms_residual_force_rate")
    mean_d_taur = _mean_delta_col("delta_rms_tau_total_rate")
    mean_d_sat = _mean_delta_col("delta_saturation")
    mean_d_lim = _mean_delta_col("delta_limit")

    report = [
        "# VSD-only vs VSD + SAC residual (paired comparison)",
        "",
        "## Configuration",
        f"- SAC checkpoint: `{args.model_path.resolve()}`",
        f"- VecNormalize: `{args.vecnormalize_path.resolve()}`",
        f"- Profile: `{args.profile}`",
        f"- Seeds: `{args.seed_start}` … `{args.seed_start + int(args.num_episodes) - 1}`",
        f"- RL config: `{cfg}`",
        f"- Overrides merged from `{run_dir / 'logs' / 'training_args.yaml'}` (if exists)"
        if run_dir is not None
        else "- Overrides: infer run dir from `--model-path` under `runs/<...>/checkpoints/` to merge ``training_args.yaml``.",
        "",
        "## Dynamics / control",
        "- Plant: hybrid cable-arm model path from RL YAML (`configs/rl_sac.yaml` unless overridden via `--config`).",
        "- Controller: nominal joint-space VSD (`Kq`,`Dq` in config) computes base joint torque.",
        "- **SAC action**: bounded task-space residual force `F_residual` scaled by `residual_force_scale`;",
        "  **`tau_residual = J.T @ F`** (cable joint columns); total joint torque clipped by `tau_jnt_limit`; mapped through hybrid transmission.",
        "",
        "## Metrics summary (paired)",
        "",
        "- In `comparison_metrics.csv`, `reward_zero` / `reward_sac` (and deltas) use **`mean_reward`**: the **mean per-step** reward over the episode (not undiscounted return).",
        "",
    ]
    if agg_metrics:
        report.append("| seed | RMS EE Δ | final EE Δ | reward Δ |")
        report.append("| --- | --- | --- | --- |")
        for r in agg_metrics:
            report.append(
                f"| {r['seed']} | {float(r['delta_rms_ee']):+.6g} | {float(r['delta_final_ee']):+.6g} | {float(r['delta_reward']):+.6g} |"
            )
        report.extend(
            [
                "",
                f"- **Across seeds**: mean Δ RMS EE = `{mean_d_rms:.6g}` m;",
                f"  mean Δ final EE = `{mean_d_fin:.6g}` m",
                "",
            ]
        )
        if mean_d_rms < 0:
            report.append(
                "- **This batch**: mean Δ RMS EE < 0 → average **path-tracking RMS** favors SAC versus zero residual."
            )
        if mean_d_fin > 0:
            report.append(
                "- **This batch**: mean Δ final EE > 0 → average **endpoint error** can be **larger** with SAC; "
                "**no** conclusion of improved final convergence."
            )
        if mean_d_rms < 0 or mean_d_fin > 0:
            report.append("")
        report.extend(
            [
                "## Oscillation & smoothness (mean Δ, VSD+SAC − VSD only)",
                "",
                f"- mean Δ `smooth_tracking_score` (lower better): `{mean_d_smooth:.6g}`",
                f"- mean Δ high-freq EE RMS: `{mean_d_hf:.6g}`",
                f"- mean Δ EE error velocity RMS: `{mean_d_evel:.6g}`",
                f"- mean Δ peak-to-peak ‖e‖: `{mean_d_p2p:.6g}`",
                f"- mean Δ residual force rate RMS: `{mean_d_frate:.6g}`",
                f"- mean Δ total torque rate RMS: `{mean_d_taur:.6g}`",
                f"- mean Δ saturation fraction: `{mean_d_sat:+.6g}`",
                f"- mean Δ limit-violation fraction: `{mean_d_lim:+.6g}`",
                "",
                "Per-seed diagnostics are under `plots/oscillation/` (EE error xyz+norm, logged high-frequency error, ",
                "`ee_dot` norm, residual force raw vs filtered, SAC force/tau rate norms, Welch PSD of ‖e‖, p2p and smooth-score bars).",
                "Aggregate Δ smooth-score histogram / mean-score bars live in `plots/oscillation/` as well.",
                "",
            ]
        )
    existing_videos = sorted(videos_dir.glob("vsd_vs_sac_seed_*.mp4"))
    if video_paths:
        video_report_lines = [f"- `{p}`" for p in video_paths]
    elif existing_videos:
        video_report_lines = [
            "- *(no new MP4 in this run; files already under `videos/`)*",
            *[f"- `{p.resolve()}`" for p in existing_videos],
        ]
    else:
        video_report_lines = ["- *(none)*"]

    report.extend(
        [
            "## Interpretation (do not over-claim)",
            "- Reference (larger paired study, 30 episodes): mean EE RMS improved (zero ≈ 0.0206 m → SAC ≈ 0.0191 m), whereas **final EE error was slightly worse** with SAC; **contacts** `ncon = 0`; **22/30** seeds showed RMS improvement. This script’s 5 seeds may differ numerically but illustrates the same qualitative pattern when it appears.",
            "- **If mean Δ RMS EE < 0**: SAC achieves **better RMS path-tracking** versus zero residual (emphasize this when it holds).",
            "- **If mean Δ final EE > 0**: report honestly that **endpoint error can be larger**; do **not** claim final convergence improvement.",
            "- EE path figures: if matplotlib 3D projection is broken (mixed system/pip installs), the script writes **XY/XZ/YZ projections** instead of a single 3D panel; video uses **side-by-side XY** in that case.",
            "",
            "## Outputs",
            f"- Metrics CSV: `{met_csv.resolve()}`",
            f"- Timeseries per seed: `{out_root}/timeseries_seed_<seed>.csv`",
            "### Plots",
        ]
        + [f"- `{p}`" for p in sorted(set(all_plot_paths))]
        + ["", "### Videos", ""]
        + video_report_lines
        + ["", "Generated by `scripts/compare_vsd_vs_sac_residual.py`."]
    )

    (out_root / "vsd_vs_sac_report.md").write_text("\n".join(report) + "\n", encoding="utf-8")

    print(f"[compare_vsd_vs_sac] Done. Report: {out_root / 'vsd_vs_sac_report.md'}")


if __name__ == "__main__":
    main()

