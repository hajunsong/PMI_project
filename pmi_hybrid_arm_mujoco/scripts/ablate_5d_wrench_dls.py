#!/usr/bin/env python3
"""Deterministic ablations: VSD vs 3D-SAC vs 5D wrench + DLS (no training).

Outputs:
  ``debug_outputs/sac_residual_task_force/ablation_5d_wrench_dls/ablation_results.csv``
  ``.../ablation_report.md``
  ``.../plots/...``
"""

from __future__ import annotations

import argparse
import copy
import csv
import itertools
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from stable_baselines3 import SAC
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import VecNormalize

_SCRIPTS = Path(__file__).resolve().parent
_PKG = _SCRIPTS.parent
for _p in (str(_PKG), str(_SCRIPTS)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from compare_vsd_vs_sac_residual import (  # noqa: E402
    ORIENTATION_INFO_KEYS,
    _safe_pearson,
    build_overrides,
    build_vec_normalize,
    episode_metrics,
    infer_run_dir,
    load_sac,
    unwrap_pmi,
    vec_options,
)
from envs.pmi_cable_residual_env import PMICableResidualEnv, _deep_merge  # noqa: E402
from final_compare_vsd_vs_sac_postprocessed import POSTPROCESS_OVERRIDES  # noqa: E402

CFG_DEF = _PKG / "configs" / "rl_sac.yaml"
OUT_DEF = _PKG / "debug_outputs" / "sac_residual_task_force" / "ablation_5d_wrench_dls"
CKPT_DEF = (
    _PKG
    / "debug_outputs"
    / "sac_residual_task_force"
    / "runs"
    / "sac_tf_tracking_reward_rs2_100k_s2"
    / "checkpoints"
    / "best_model_by_ee_rms.zip"
)
VN_DEF = CKPT_DEF.parent.parent / "vecnormalize" / "vecnormalize.pkl"

POSTPROC_5D = _deep_merge(
    copy.deepcopy(POSTPROCESS_OVERRIDES),
    {
        "residual_filter": {"enabled": True, "force_filter_tau": 0.03, "moment_filter_tau": 0.05},
        "action_smoothing": {"enabled": True, "max_delta_force_per_step": 0.2, "max_delta_moment_per_step": 0.01},
        "residual_postprocess": {"final_fade_duration": 0.5},
    },
)


def _unwrap_monitor(e: Any) -> PMICableResidualEnv:
    cur = e
    while hasattr(cur, "env") and not isinstance(cur, PMICableResidualEnv):
        cur = cur.env
    if not isinstance(cur, PMICableResidualEnv):
        raise TypeError(f"Expected PMICableResidualEnv under Monitor, got {type(cur)}")
    return cur


def _wrench_cols(prefix: str, w5: np.ndarray) -> dict[str, float]:
    v = np.asarray(w5, dtype=np.float64).reshape(5)
    return {
        f"{prefix}_x": float(v[0]),
        f"{prefix}_y": float(v[1]),
        f"{prefix}_z": float(v[2]),
        f"{prefix}_mr": float(v[3]),
        f"{prefix}_mp": float(v[4]),
    }


def _append_log_row(
    rows: list[dict[str, Any]],
    *,
    scenario_label: str,
    cable_seed: int,
    config_id: str,
    inf: dict[str, Any],
    pmi: PMICableResidualEnv,
    act: np.ndarray,
    rew_step: float,
) -> None:
    sac_full = np.asarray(act, dtype=np.float64).reshape(-1)
    sac_xyz = sac_full[:3] if sac_full.shape[0] >= 3 else np.pad(sac_full, (0, 3 - sac_full.shape[0]))
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

    Wraw = np.asarray(inf.get("W_residual_raw_5d", np.zeros(5)), dtype=np.float64).reshape(5)
    Wlim = np.asarray(inf.get("W_residual_rate_limited_5d", Wraw), dtype=np.float64).reshape(5)
    Wfilt = np.asarray(inf.get("W_residual_filtered_5d", Wlim), dtype=np.float64).reshape(5)
    Wused = np.asarray(inf.get("W_residual_used_5d", Wfilt), dtype=np.float64).reshape(5)
    gate = float(inf.get("residual_gate", 1.0))

    row: dict[str, Any] = {
        "scenario": scenario_label,
        "config_id": config_id,
        "cable_seed": int(cable_seed),
        "time": tn,
        "residual_gate": gate,
        **{f"ee_des_{axis}": float(ee_d[i]) for i, axis in enumerate("xyz")},
        **{f"ee_act_{axis}": float(ee_a[i]) for i, axis in enumerate("xyz")},
        **{f"ee_err_{axis}": float(ee_e[i]) for i, axis in enumerate("xyz")},
        "ee_err_norm": float(inf["ee_error_norm"]),
        **{f"q_jnt_{k+1}": float(q_j[k]) for k in range(4)},
        **{f"qdot_jnt_{k+1}": float(qd_j[k]) for k in range(4)},
        **{f"q_des_{k+1}": float(q_des[k]) for k in range(4)},
        **{f"qdot_des_{k+1}": float(qd_des[k]) for k in range(4)},
        "q_err_norm": float(inf["q_error_norm"]),
        **{f"sac_action_{axis}": float(sac_xyz[i]) for i, axis in enumerate("xyz")},
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
        "reward_step": float(rew_step),
        **_wrench_cols("W_raw", Wraw),
        **_wrench_cols("W_lim", Wlim),
        **_wrench_cols("W_filt", Wfilt),
        **_wrench_cols("W_used", Wused),
    }
    for ok in ORIENTATION_INFO_KEYS:
        if ok in inf:
            row[str(ok)] = inf[ok]
    rows.append(row)


def rollout_monitor(
    cfg: Path,
    overrides: dict[str, Any],
    *,
    profile: str,
    cable_seed: int,
    scenario_label: str,
    config_id: str,
) -> list[dict[str, Any]]:
    env = Monitor(PMICableResidualEnv(config_path=str(cfg), overrides=overrides))
    pmi = _unwrap_monitor(env)
    rows: list[dict[str, Any]] = []
    env.reset(options=vec_options(profile, cable_seed))
    while True:
        adim = int(env.action_space.shape[0])
        act = np.zeros((adim,), dtype=np.float32)
        obs, rew, terminated, truncated, inf = env.step(act)
        _append_log_row(
            rows,
            scenario_label=scenario_label,
            cable_seed=cable_seed,
            config_id=config_id,
            inf=inf,
            pmi=pmi,
            act=act,
            rew_step=float(rew),
        )
        if bool(terminated) or bool(truncated):
            break
    env.close()
    return rows


def rollout_vecnorm(
    cfg: Path,
    vn_path: Path,
    overrides: dict[str, Any],
    *,
    profile: str,
    cable_seed: int,
    scenario_label: str,
    config_id: str,
    model: SAC | None,
) -> list[dict[str, Any]]:
    vn = build_vec_normalize(cfg, vn_path, overrides)
    vn.seed(int(cable_seed))
    vn.set_options(vec_options(profile, cable_seed))
    pmi = unwrap_pmi(vn)
    rows: list[dict[str, Any]] = []
    obs = vn.reset()
    while True:
        if model is None:
            adim = int(vn.action_space.shape[0])
            act = np.zeros((vn.num_envs, adim), dtype=np.float32)
        else:
            act, _ = model.predict(obs, deterministic=True)
        tup = vn.step(act)
        if len(tup) == 5:
            obs, rew, terminated, truncated, infos = tup
            done = bool(np.asarray(terminated).reshape(-1)[0]) or bool(np.asarray(truncated).reshape(-1)[0])
        elif len(tup) == 4:
            obs, rew, dones, infos = tup
            done = bool(np.asarray(dones).reshape(-1)[0])
        else:
            vn.close()
            raise RuntimeError(f"Unexpected VecEnv step arity {len(tup)}")
        rew_f = float(np.asarray(rew).reshape(-1)[0])
        inf = infos[0]
        _append_log_row(
            rows,
            scenario_label=scenario_label,
            cable_seed=cable_seed,
            config_id=config_id,
            inf=inf,
            pmi=pmi,
            act=np.asarray(act).reshape(-1),
            rew_step=rew_f,
        )
        if done:
            break
    vn.close()
    return rows


def append_wrench_rate_metrics(m: dict[str, float], rows: list[dict[str, Any]]) -> None:
    if not rows or "W_used_x" not in rows[0]:
        return
    wn = np.array(
        [
            float(
                np.linalg.norm(
                    [
                        float(r["W_used_x"]),
                        float(r["W_used_y"]),
                        float(r["W_used_z"]),
                        float(r["W_used_mr"]),
                        float(r["W_used_mp"]),
                    ]
                )
            )
            for r in rows
        ],
        dtype=np.float64,
    )
    ts = np.array([float(r["time"]) for r in rows], dtype=np.float64)
    dt = float(np.median(np.diff(ts))) if len(ts) > 1 else 0.01
    if not (dt > 0.0):
        dt = 0.01
    if len(wn) > 2:
        d = np.diff(wn, prepend=wn[0:1])
        m["rms_residual_wrench_norm"] = float(np.sqrt(np.mean(wn * wn)))
        m["rms_residual_wrench_rate"] = float(np.sqrt(np.mean((d / dt) ** 2)))
    else:
        m["rms_residual_wrench_norm"] = float("nan")
        m["rms_residual_wrench_rate"] = float("nan")


def full_episode_analysis(rows: list[dict[str, Any]]) -> dict[str, float]:
    m = dict(episode_metrics(rows))
    append_wrench_rate_metrics(m, rows)
    return m


def aggregate_mean(metric_by_seed: dict[int, dict[str, float]], key: str) -> float:
    vals = []
    for _s, mx in metric_by_seed.items():
        v = mx.get(key, float("nan"))
        if v == v:
            vals.append(float(v))
    return float(sum(vals) / len(vals)) if vals else float("nan")


DELTA_PAIRS = [
    ("rms_ee_error", "delta_rms_ee"),
    ("final_ee_error", "delta_final_ee"),
    ("rms_ee_error_highfreq", "delta_highfreq"),
    ("p2p_error_norm", "delta_p2p"),
    ("rms_roll_error", "delta_roll_rms"),
    ("rms_pitch_error", "delta_pitch_rms"),
    ("rms_ee_error_velocity", "delta_vel"),
    ("rms_ee_error_acceleration", "delta_acc"),
    ("saturation_fraction", "delta_saturation"),
    ("limit_violation_fraction", "delta_limit"),
]


def deltas_one_seed(ma: dict[str, float], mx: dict[str, float]) -> dict[str, float]:
    out: dict[str, float] = {}
    for sk, dk in DELTA_PAIRS:
        av = ma.get(sk, float("nan"))
        xv = mx.get(sk, float("nan"))
        out[dk] = float(xv - av) if av == av and xv == xv else float("nan")
    return out


def mean_delta_over_seeds(delta_rows: list[dict[str, float]]) -> dict[str, float]:
    if not delta_rows:
        return {}
    keys = [dk for _, dk in DELTA_PAIRS]
    return {
        dk: float(np.nanmean(np.array([float(r.get(dk, np.nan)) for r in delta_rows], dtype=np.float64)))
        for dk in keys
    }


METRIC_COLUMNS = [
    "rms_ee_error",
    "final_ee_error",
    "max_ee_error",
    "rms_roll_error",
    "rms_pitch_error",
    "max_abs_roll_error",
    "max_abs_pitch_error",
    "rms_ee_error_velocity",
    "rms_ee_error_acceleration",
    "rms_ee_error_highfreq",
    "p2p_error_norm",
    "saturation_fraction",
    "limit_violation_fraction",
    "rms_residual_wrench_norm",
    "rms_residual_wrench_rate",
    "rms_residual_tau_rate",
    "rms_tau_total_rate",
]


def selection_score(dm: dict[str, float]) -> float:
    def _fv(key: str, default: float = 0.0) -> float:
        xf = float(dm.get(key, default))
        return default if xf != xf else float(xf)

    dr = _fv("delta_rms_ee")

    def pospart(key: str) -> float:
        return max(0.0, _fv(key))

    return float(
        dr
        + 1.0 * pospart("delta_final_ee")
        + 0.5 * pospart("delta_highfreq")
        + 0.2 * pospart("delta_p2p")
        + 0.2 * pospart("delta_roll_rms")
        + 0.2 * pospart("delta_pitch_rms")
        + 0.1 * pospart("delta_saturation")
        + 0.1 * pospart("delta_limit")
    )


@dataclass(frozen=True)
class ExpCfg:
    family: str
    config_id: str
    overrides: dict[str, Any]
    uses_vecnorm_sac: bool
    use_sac_model: bool


def baseline_a_overrides(profile: str, run_dir: Path | None) -> dict[str, Any]:
    root = copy.deepcopy(build_overrides(profile, run_dir))
    return _deep_merge(
        root,
        {
            "residual": {"type": "task_space_force", "action_dim": 3},
            "residual_filter": {"enabled": False, "tau": 0.03},
            "action_smoothing": {"enabled": False, "max_delta_force_per_step": 0.2},
            "residual_postprocess": {"final_fade_duration": 0.0},
        },
    )


def baseline_b_overrides(profile: str, run_dir: Path | None) -> dict[str, Any]:
    root = copy.deepcopy(build_overrides(profile, run_dir))
    m1 = _deep_merge(root, copy.deepcopy(POSTPROC_5D))
    return _deep_merge(m1, {"residual": {"type": "task_space_force", "action_dim": 3}})


def heuristic_5d_root(profile: str, run_dir: Path | None) -> dict[str, Any]:
    m0 = copy.deepcopy(build_overrides(profile, run_dir))
    m1 = _deep_merge(m0, copy.deepcopy(POSTPROC_5D))
    return _deep_merge(
        m1,
        {
            "residual": {
                "type": "task_space_wrench_5d",
                "wrench_command_mode": "heuristic_rp_zeros_force",
                "action_dim": 5,
                "residual_force_scale": 1.0,
                "residual_moment_scale": 0.05,
                "lambda_residual": 1.0,
            },
        },
    )


def build_experiment_list(
    *,
    profile: str,
    run_dir: Path | None,
    quick: bool,
    include_e: bool,
) -> list[ExpCfg]:
    exps: list[ExpCfg] = []

    exps.append(
        ExpCfg(
            family="A",
            config_id="A_vsd_only",
            overrides=baseline_a_overrides(profile, run_dir),
            uses_vecnorm_sac=True,
            use_sac_model=False,
        )
    )
    exps.append(
        ExpCfg(
            family="B",
            config_id="B_sac3d_postproc",
            overrides=baseline_b_overrides(profile, run_dir),
            uses_vecnorm_sac=True,
            use_sac_model=True,
        )
    )

    rp_w_vals = [0.05, 0.1, 0.2, 0.4]
    lambdas = [0.01, 0.03, 0.05, 0.1, 0.2]
    kp_vals = [(0.02, 0.002), (0.05, 0.005), (0.1, 0.01)]
    scopes = ["cable_joints_only", "all_joints"]

    if quick:
        rp_w_vals = [0.1, 0.2]
        lambdas = [0.03, 0.1]
        kp_vals = [(0.05, 0.005)]
        scopes = ["cable_joints_only"]

    def add_heuristic_grid(wmap: str, fam: str) -> None:
        for w_rp, lam, (kp, dp), scope in itertools.product(rp_w_vals, lambdas, kp_vals, scopes):
            cid = (
                f"{fam}_tw{w_rp:g}_ld{lam:g}_kp{kp:g}_{scope}_{wmap[:4]}"
            )
            ovr = heuristic_5d_root(profile, run_dir)
            ovr = _deep_merge(
                ovr,
                {
                    "residual": {
                        "wrench_map_mode": str(wmap),
                        "lambda_dls": float(lam),
                        "task_weights_5d": [1.0, 1.0, 1.0, float(w_rp), float(w_rp)],
                        "residual_joint_scope": str(scope),
                        "heuristic_residual_rp": {
                            "Kp_roll": float(kp),
                            "Dp_roll": float(dp),
                            "Kp_pitch": float(kp),
                            "Dp_pitch": float(dp),
                        },
                    },
                },
            )
            exps.append(
                ExpCfg(
                    family=fam,
                    config_id=cid,
                    overrides=ovr,
                    uses_vecnorm_sac=False,
                    use_sac_model=False,
                )
            )

    add_heuristic_grid("wrench_jacobian_transpose", "C")
    for wm in ("dls_task_correction", "dls_weighted_wrench"):
        add_heuristic_grid(wm, "D")

    if include_e:
        m0 = copy.deepcopy(build_overrides(profile, run_dir))
        m1 = _deep_merge(m0, copy.deepcopy(POSTPROC_5D))
        ovr = _deep_merge(
            m1,
            {
                "residual": {
                    "type": "task_space_wrench_5d",
                    "wrench_command_mode": "sac_xyz_plus_heuristic_rp",
                    "wrench_map_mode": "dls_task_correction",
                    "action_dim": 3,
                    "task_weights_5d": [1.0, 1.0, 1.0, 0.2, 0.2],
                    "lambda_dls": 0.05,
                    "residual_joint_scope": "cable_joints_only",
                    "heuristic_residual_rp": {
                        "Kp_roll": 0.05,
                        "Dp_roll": 0.005,
                        "Kp_pitch": 0.05,
                        "Dp_pitch": 0.005,
                    },
                },
            },
        )
        # 학습 스케일은 training_args.rl_overrides / 기본값 유지(Fx,Fy,Fz는 체크포인트 학습 분포와 맞춤)
        exps.append(
            ExpCfg(
                family="E",
                config_id="E_sac3d_plus_heuristic_dls_tw0.2_ld0.05",
                overrides=ovr,
                uses_vecnorm_sac=True,
                use_sac_model=True,
            )
        )

    return exps


def run_rollout(exp: ExpCfg, *, cfg: Path, vn_path: Path, ckpt: Path | None, profile: str, seed: int):
    ck = ckpt if exp.use_sac_model else None
    if exp.uses_vecnorm_sac:
        return rollout_vecnorm(
            cfg,
            vn_path,
            exp.overrides,
            profile=profile,
            cable_seed=seed,
            scenario_label=exp.family,
            config_id=exp.config_id,
            model=(load_sac(ck) if ck is not None and exp.use_sac_model else None),
        )
    return rollout_monitor(
        cfg,
        exp.overrides,
        profile=profile,
        cable_seed=seed,
        scenario_label=exp.family,
        config_id=exp.config_id,
    )


def _plot_ee_and_tau(
    out_dir: Path,
    *,
    scenario: str,
    config_id: str,
    seed: int,
    rows: list[dict[str, Any]],
    tag: str,
) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    out_dir.mkdir(parents=True, exist_ok=True)
    t = np.array([float(r["time"]) for r in rows], dtype=np.float64)
    base = f"{scenario}_{config_id}_s{seed}_{tag}".replace("/", "_")

    fig, axs = plt.subplots(5, 1, figsize=(9, 10.0), sharex=True)
    for i, a in enumerate("xyz"):
        axs[i].plot(t, [float(r[f"ee_err_{a}"]) for r in rows], lw=1.0)
        axs[i].set_ylabel(f"e_{a}")
    axs[3].plot(t, [float(r["ee_err_norm"]) for r in rows], color="k", lw=1.0)
    axs[3].set_ylabel("|e|")
    for k in range(1, 5):
        axs[4].plot(t, [float(r[f"tau_residual_{k}"]) for r in rows], lw=1.0, label=f"τ_res j{k}")
    axs[4].legend(fontsize=6, ncol=4)
    axs[4].set_ylabel("tau_res")
    axs[-1].set_xlabel("time [s]")
    fig.suptitle(f"top: EE + τ_residual ({scenario} {config_id} s={seed})")
    fig.tight_layout()
    fig.savefig(out_dir / f"{base}.png", dpi=120)
    plt.close(fig)


def _plot_path3d(
    out_dir: Path,
    *,
    scenario: str,
    config_id: str,
    seed: int,
    rows: list[dict[str, Any]],
    tag: str,
) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    out_dir.mkdir(parents=True, exist_ok=True)
    base = f"{scenario}_{config_id}_s{seed}_{tag}".replace("/", "_")
    xd = [float(r["ee_des_x"]) for r in rows]
    yd = [float(r["ee_des_y"]) for r in rows]
    zd = [float(r["ee_des_z"]) for r in rows]
    xa = [float(r["ee_act_x"]) for r in rows]
    ya = [float(r["ee_act_y"]) for r in rows]
    za = [float(r["ee_act_z"]) for r in rows]
    try:
        fig = plt.figure(figsize=(6.5, 6.0))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(xd, yd, zd, label="des", lw=2.0)
        ax.plot(xa, ya, za, label="act", lw=1.2, alpha=0.85)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.legend(loc="upper right", fontsize=8)
        ax.set_title("EE path 3D")
        fig.tight_layout()
        fig.savefig(out_dir / f"{base}.png", dpi=120)
        plt.close(fig)
    except (ValueError, KeyError, ImportError):
        fig, ax = plt.subplots(figsize=(6.0, 5.5))
        ax.plot(xd, yd, label="des xy", lw=2.0)
        ax.plot(xa, ya, label="act xy", lw=1.2, alpha=0.85)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.legend()
        ax.set_title("EE path (xy fallback; 3d unavailable)")
        fig.tight_layout()
        fig.savefig(out_dir / f"{base}_xy_fallback.png", dpi=120)
        plt.close(fig)


def _plot_orientation(
    out_dir: Path,
    *,
    scenario: str,
    config_id: str,
    seed: int,
    rows: list[dict[str, Any]],
    tag: str,
) -> None:
    if not rows or "ee_roll_rad" not in rows[0]:
        return
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    out_dir.mkdir(parents=True, exist_ok=True)
    t = np.array([float(r["time"]) for r in rows], dtype=np.float64)
    base = f"{scenario}_{config_id}_s{seed}_{tag}".replace("/", "_")

    fig, axs = plt.subplots(5, 1, figsize=(9, 9.5), sharex=True)
    axs[0].plot(t, [float(r["ee_roll_rad"]) for r in rows], label="roll act")
    axs[0].plot(t, [float(r["desired_roll_rad"]) for r in rows], "--", label="roll des")
    axs[1].plot(t, [float(r["ee_pitch_rad"]) for r in rows], label="pitch act")
    axs[1].plot(t, [float(r["desired_pitch_rad"]) for r in rows], "--", label="pitch des")
    axs[2].plot(t, [float(r["roll_error_rad"]) for r in rows], lw=1.0)
    axs[2].plot(t, [float(r["pitch_error_rad"]) for r in rows], lw=1.0)
    axs[2].set_ylabel("r/p err")
    axs[3].plot(t, [float(r["ee_hf_norm"]) for r in rows], lw=1.0, color="C0")
    axs[3].set_ylabel("HF ee")
    axs[4].plot(t, [float(r["ee_dot_norm"]) for r in rows], lw=1.0, color="C1")
    axs[4].set_ylabel("ee err vel norm")
    for ax in axs[:2]:
        ax.legend(fontsize=7, ncol=2)
        ax.grid(True, alpha=0.25)
    axs[-1].set_xlabel("time [s]")
    fig.suptitle(f"orientation + HF/vel ({scenario} s={seed})")
    fig.tight_layout()
    fig.savefig(out_dir / f"{base}.png", dpi=120)
    plt.close(fig)


def _plot_oscillation(
    out_dir: Path,
    *,
    scenario: str,
    config_id: str,
    seed: int,
    rows: list[dict[str, Any]],
    tag: str,
) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    out_dir.mkdir(parents=True, exist_ok=True)
    t = np.array([float(r["time"]) for r in rows], dtype=np.float64)
    base = f"{scenario}_{config_id}_s{seed}_{tag}".replace("/", "_")

    comps = ["W_used_x", "W_used_y", "W_used_z", "W_used_mr", "W_used_mp"]
    fig, axs = plt.subplots(5, 1, figsize=(9, 9.5), sharex=True)
    for ax, ck in zip(axs, comps):
        if ck in rows[0]:
            ax.plot(t, [float(r[ck]) for r in rows], lw=1.0)
        ax.set_ylabel(ck.replace("W_used_", "")[:8])
    axs[-1].set_xlabel("time [s]")
    fig.suptitle(f"residual wrench W_used ({scenario} s={seed})")
    fig.tight_layout()
    fig.savefig(out_dir / f"{base}_wrench.png", dpi=120)
    plt.close(fig)

    fig2, ax2 = plt.subplots(3, 1, figsize=(9, 6.8), sharex=True)
    for k in range(1, 5):
        ax2[0].plot(t, [float(r[f"tau_total_{k}"]) for r in rows], lw=1.0, label=f"j{k}")
    ax2[0].legend(fontsize=6, ncol=4)
    ax2[0].set_ylabel("tau_total")
    sat = np.array([float(r["saturation_flag"]) for r in rows])
    lim = np.array([max(int(r["joint_limit_violation"]), int(r["actuator_limit_violation"])) for r in rows])
    ax2[1].plot(t, sat, lw=1.0)
    ax2[2].plot(t, lim, lw=1.0)
    ax2[1].set_ylabel("sat")
    ax2[2].set_ylabel("lim")
    ax2[-1].set_xlabel("time [s]")
    fig2.suptitle("tau_total / flags")
    fig2.tight_layout()
    fig2.savefig(out_dir / f"{base}_tau_sat.png", dpi=120)
    plt.close(fig2)


def plot_best_config_figures(
    *,
    dirs_top: Path,
    dirs_orient: Path,
    dirs_osc: Path,
    dirs_p3d: Path,
    scenario: str,
    config_id: str,
    seed: int,
    rows: list[dict[str, Any]],
) -> None:
    _plot_ee_and_tau(dirs_top, scenario=scenario, config_id=config_id, seed=seed, rows=rows, tag="top")
    _plot_orientation(dirs_orient, scenario=scenario, config_id=config_id, seed=seed, rows=rows, tag="ori")
    _plot_oscillation(dirs_osc, scenario=scenario, config_id=config_id, seed=seed, rows=rows, tag="osc")
    _plot_path3d(dirs_p3d, scenario=scenario, config_id=config_id, seed=seed, rows=rows, tag="p3d")


def generate_report_md(
    path: Path,
    *,
    a_mean: dict[str, float],
    rows_summary: list[dict[str, Any]],
    correlations: dict[str, float],
) -> None:
    lines: list[str] = []
    lines.append("# 5D task-space wrench + DLS ablation report")
    lines.append("")
    lines.append("이 리포트는 결정론적 롤아웃(학습 없음) 기준입니다.")
    lines.append("")
    lines.append("## 명목 IK / 과제 차원 요약 답변")
    lines.append("")
    lines.append("1. **현재 명목 IK는 무엇을 제약하는가?** "
                 "설정 및 코드상 `solve_ik_task_mode` 호출 시 `task_feas_mode=\"xyz\"` 로 "
                 "IK는 **카테시안 위치(xyz)** 위주로 풀리며 자세(roll/pitch)는 IK 하드 과제 변수로는 "
                 "**같이 맞추지 않으며**, IK 시드·보간으로 생성된 `q_des`와 "
                 "**명목 소프트 자세**(controller `orientation_soft_mode` 등이 꺼져 있으면)에 따라 EE의 roll/pitch는 "
                 "경로를 따라 오차가 존재할 수 있습니다.")
    lines.append("")
    lines.append(f"2. **roll/pitch 오차와 EE 경로 진동 상관관계(에피소드 평균):** ")
    lines.append(
        "   Pearson (시드별 VSD 단독 궤적에서 샘플한 시점들): "
        f"|e|↔|roll|≈{correlations.get('ee_vs_abs_roll', float('nan')):.4f}, "
        f"|e|↔|pitch|≈{correlations.get('ee_vs_abs_pitch', float('nan')):.4f}, "
        f"HF↔|roll|≈{correlations.get('hf_vs_abs_roll', float('nan')):.4f}, "
        f"HF↔|pitch|≈{correlations.get('hf_vs_abs_pitch', float('nan')):.4f}."
    )
    lines.append("   실제 결과는 각 시드별 궤적에 따라 바뀌며, 아래 표의 집계를 우선 참고합니다.")
    lines.append("")
    lines.append("## 주요 결과 (scenario 평균, VSD 단독 대비)")
    lines.append("")
    if rows_summary:
        r0 = rows_summary[0]
        lines.append(
            f"- **평균 시드 기준 selection_score 최소:** `{r0['config_id']}` "
            f"(family **{r0['family']}**), score={float(r0['selection_score']):.6f}. "
            "λ·joint scope는 `config_id` 문자열(`_ld…`, `cable_joints_only` / `all_joints`)에서 읽습니다."
        )
        lines.append("")
    lines.append("| config_id | family | score↓ | Δrms EE | Δfinal | ΔHF | Δp2p | Δroll | Δpitch | Δsat |")
    lines.append("|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|")
    for r in rows_summary[:40]:
        lines.append(
            f"| `{r['config_id']}` | {r['family']} | {r['selection_score']:.4f} | "
            f"{r['delta_rms_ee']:.6f} | {r['delta_final_ee']:.6f} | {r['delta_highfreq']:.6f} | "
            f"{r['delta_p2p']:.6f} | {r['delta_roll_rms']:.6f} | {r['delta_pitch_rms']:.6f} | {r['delta_saturation']:.6f} |"
        )
    if len(rows_summary) > 40:
        lines.append("")
        lines.append(f"(표는 상위 {40}개만 표시 — 전체는 CSV 참조)")
    lines.append("")
    lines.append("## VSD-only baseline 평균 지표 일부")
    lines.append("")
    lines.append("| key | mean |")
    lines.append("|:---|---:|")
    for kk in sorted(a_mean.keys())[:25]:
        lines.append(f"| {kk} | {a_mean[kk]:.6f} |")
    lines.append("")
    lines.append("## 질문 3–11 (정성 + 표 기준)")
    lines.append("")
    lines.append("3–4. 소프트 roll/pitch 잔차와 DLS는 **설정별로** EE 고주파 에러 및 p2p를 줄일 수 있습니다. 최적은 위 표에서 **score 및 ΔHF/Δp2p** 확인.")
    lines.append("5. **최적 lambda_dls**는 그리드의 **상위 score 행**(보통 λ≈0.03–0.1 구간에서 안정되는 경우 많음 — 이번 CSV에서 가장 좋은 `ld*` 확인).")
    lines.append("6. **cable_joints_only vs all_joints**는 scope 비교 행을 필터링해 Δrms/Δsat을 비교하면 됩니다.")
    lines.append("7–9. 같은 방식으로 `delta_rms_ee`, `delta_final_ee`, `delta_highfreq`, `delta_p2p` 열 확인.")
    lines.append("10. `delta_saturation`, `delta_limit` 열로 한계 접촉/위반 증가 여부를 평가.")
    lines.append("11. **향후 SAC 5D 렌치 확장 여부**: 이번 휴리스틱 + DLS가 tracking/진동 개선 중 하나라면 **학습 차원 확장**(5D 또는 3D+F + 휴리스틱)을 검토할 가치가 있습니다. 학습 코드는 사용자 요청에 따라 본 과제에서는 추가하지 않았습니다.")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg", type=Path, default=CFG_DEF)
    ap.add_argument("--out-root", type=Path, default=OUT_DEF)
    ap.add_argument("--model-path", type=Path, default=CKPT_DEF)
    ap.add_argument("--vecnormalize-path", type=Path, default=VN_DEF)
    ap.add_argument("--profile", type=str, default="medium_train")
    ap.add_argument("--seed-start", type=int, default=10000)
    ap.add_argument("--num-seeds", type=int, default=30)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--include-e", action="store_true")
    ap.add_argument(
        "--skip-learned-sac",
        action="store_true",
        help="B/E(학습된 SAC) 제외 — 체크포인트 없이 A/C/D만 실행",
    )
    ap.add_argument("--plot-top-k", type=int, default=3)
    args = ap.parse_args()

    cfg_path = Path(args.cfg).resolve()
    run_dir = infer_run_dir(args.model_path)

    seeds = list(range(int(args.seed_start), int(args.seed_start) + int(args.num_seeds)))
    experiments = build_experiment_list(
        profile=str(args.profile),
        run_dir=run_dir,
        quick=bool(args.quick),
        include_e=bool(args.include_e),
    )
    if args.skip_learned_sac:
        experiments = [e for e in experiments if not e.use_sac_model]

    out_root = Path(args.out_root).resolve()
    plots_top = out_root / "plots" / "top_configs"
    plots_orch = out_root / "plots" / "orientation"
    plots_osc = out_root / "plots" / "oscillation"
    plots_p3 = out_root / "plots" / "3d_paths"
    plots_top.mkdir(parents=True, exist_ok=True)
    plots_orch.mkdir(parents=True, exist_ok=True)
    plots_osc.mkdir(parents=True, exist_ok=True)
    plots_p3.mkdir(parents=True, exist_ok=True)

    csv_path = out_root / "ablation_results.csv"

    by_key: dict[tuple[str, int], dict[str, float]] = {}
    traj_cache: dict[tuple[str, int], list[dict[str, Any]]] = {}
    a_by_seed: dict[int, dict[str, float]] = {}

    for exp in experiments:
        for sd in seeds:
            rows = run_rollout(
                exp,
                cfg=cfg_path,
                vn_path=Path(args.vecnormalize_path),
                ckpt=args.model_path,
                profile=args.profile,
                seed=int(sd),
            )
            m = full_episode_analysis(rows)
            key = (exp.config_id, int(sd))
            by_key[key] = m
            traj_cache[key] = rows
            if exp.family == "A":
                a_by_seed[int(sd)] = m

    corr_sum_ee_roll = corr_sum_ee_pitch = 0.0
    corr_hf_roll = corr_hf_pitch = 0.0
    n_corr = 0
    for sd in seeds:
        k = ("A_vsd_only", sd)
        if k not in traj_cache:
            continue
        rs = traj_cache[k]
        if not rs or "roll_error_rad" not in rs[0]:
            continue
        ee_n = np.array([float(r["ee_err_norm"]) for r in rs])
        hf = np.array([float(r["ee_hf_norm"]) for r in rs])
        are = np.abs(np.array([float(r["roll_error_rad"]) for r in rs]))
        ape = np.abs(np.array([float(r["pitch_error_rad"]) for r in rs]))
        corr_sum_ee_roll += _safe_pearson(ee_n, are)
        corr_sum_ee_pitch += _safe_pearson(ee_n, ape)
        corr_hf_roll += _safe_pearson(hf, are)
        corr_hf_pitch += _safe_pearson(hf, ape)
        n_corr += 1
    correlations = {
        "ee_vs_abs_roll": corr_sum_ee_roll / max(1, n_corr),
        "ee_vs_abs_pitch": corr_sum_ee_pitch / max(1, n_corr),
        "hf_vs_abs_roll": corr_hf_roll / max(1, n_corr),
        "hf_vs_abs_pitch": corr_hf_pitch / max(1, n_corr),
    }

    agg_rows: list[dict[str, Any]] = []
    csv_lines: list[dict[str, Any]] = []

    delta_keys_ordered = [dk for _, dk in DELTA_PAIRS]
    metric_prefixed_keys = [f"m_{mk}" for mk in METRIC_COLUMNS]

    for exp in experiments:
        for sd in seeds:
            mx = by_key[(exp.config_id, int(sd))]
            if exp.family == "A":
                drow = {dk: float("nan") for dk in delta_keys_ordered}
            else:
                drow = deltas_one_seed(a_by_seed[int(sd)], mx)
            csv_rec: dict[str, Any] = {
                "family": exp.family,
                "config_id": exp.config_id,
                "cable_seed": int(sd),
                **{n: mx.get(k, "") for k, n in zip(METRIC_COLUMNS, metric_prefixed_keys, strict=True)},
                **drow,
            }
            csv_lines.append(csv_rec)
        if exp.family == "A":
            continue
        dlist = [deltas_one_seed(a_by_seed[int(sd)], by_key[(exp.config_id, int(sd))]) for sd in seeds]
        dm_mean = mean_delta_over_seeds(dlist)
        sc = selection_score(dm_mean)
        agg_rows.append(
            {
                "family": exp.family,
                "config_id": exp.config_id,
                "selection_score": sc,
                **dm_mean,
                **{
                    "mean_" + k: aggregate_mean({s: by_key[(exp.config_id, s)] for s in seeds}, k)
                    for k in ("rms_ee_error", "final_ee_error", "rms_ee_error_highfreq", "p2p_error_norm")
                },
            }
        )
    agg_rows.sort(key=lambda z: float(z["selection_score"]))

    csv_fieldnames = ["family", "config_id", "cable_seed", *metric_prefixed_keys, *delta_keys_ordered]
    out_root.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=csv_fieldnames, extrasaction="ignore")
        w.writeheader()
        for rec in csv_lines:
            safe = {}
            for k in csv_fieldnames:
                val = rec.get(k, "")
                if isinstance(val, float) and val != val:
                    safe[k] = ""
                elif val == val or isinstance(val, (int, str)):
                    safe[k] = val
                else:
                    safe[k] = ""
            w.writerow(safe)

    plot_seed_hi = min(5, len(seeds))
    plot_seed_list = seeds[:plot_seed_hi]
    for ranked in agg_rows[: int(args.plot_top_k)]:
        cid = str(ranked["config_id"])
        for sd in plot_seed_list:
            k = (cid, sd)
            if k not in traj_cache:
                continue
            rs = traj_cache[k]
            plot_best_config_figures(
                dirs_top=plots_top,
                dirs_orient=plots_orch,
                dirs_osc=plots_osc,
                dirs_p3d=plots_p3,
                scenario=str(ranked["family"]),
                config_id=cid,
                seed=int(sd),
                rows=rs,
            )

    a_mean = {ak: aggregate_mean({s: a_by_seed[int(s)] for s in seeds}, ak) for ak in METRIC_COLUMNS}
    generate_report_md(
        out_root / "ablation_report.md",
        a_mean=a_mean,
        rows_summary=agg_rows,
        correlations=correlations,
    )

    print(f"Wrote {csv_path}")
    print(f"Wrote {out_root/'ablation_report.md'}")


if __name__ == "__main__":
    main()
