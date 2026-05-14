#!/usr/bin/env python3
"""Paired evaluation: workspace 5D VSD vs VSD+SAC under configurable measurement/actuator noise (eval-only)."""

from __future__ import annotations

import argparse
import copy
import csv
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

_ROOT = Path(__file__).resolve().parents[1]
import sys

if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

matplotlib = __import__("matplotlib")
matplotlib.use("Agg")

from utils.mujoco_helpers import load_yaml
from utils.workspace_5d_rl_metrics import rollout_one_episode_metrics, workspace_smooth_score


def _deep_merge(base: dict[str, Any], over: dict[str, Any]) -> dict[str, Any]:
    out = copy.deepcopy(base)
    for k, v in over.items():
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def _effective_cable_randomize(cfg_eval: dict[str, Any]) -> tuple[bool, str]:
    env_cfg = cfg_eval.get("env") or {}
    randomize = bool(env_cfg.get("randomize_cable", False))
    profile = str(env_cfg.get("randomization_profile", "medium_train"))
    cur = cfg_eval.get("curriculum") or {}
    stage = str(cur.get("stage", "deterministic"))
    stages = cur.get("stages") or {}
    if stage in stages and isinstance(stages[stage], dict):
        st = stages[stage]
        if "randomize_cable" in st:
            randomize = bool(st["randomize_cable"])
        if "randomization_profile" in st:
            profile = str(st["randomization_profile"])
    return randomize, profile


NOISE_PRESET_DELTAS: dict[str, dict[str, Any]] = {
    "clean": {"noise": {"enabled": False}},
    "sensor_noise_only": {
        "noise": {
            "enabled": True,
            "sensor": {"enabled": True},
            "measurement_delay": {"enabled": False},
            "measurement_filter": {"enabled": False},
            "actuator": {"enabled": False},
            "random_walk_bias": {"enabled": False},
        }
    },
    "delay_only": {
        "noise": {
            "enabled": True,
            "sensor": {"enabled": False},
            "measurement_delay": {"enabled": True, "delay_steps": 1},
            "measurement_filter": {"enabled": False},
            "actuator": {"enabled": False},
            "random_walk_bias": {"enabled": False},
        }
    },
    "sensor_delay_filter": {
        "noise": {
            "enabled": True,
            "sensor": {"enabled": True},
            "measurement_delay": {"enabled": True, "delay_steps": 1},
            "measurement_filter": {"enabled": True, "cutoff_hz": 20.0},
            "actuator": {"enabled": False},
            "random_walk_bias": {"enabled": False},
        }
    },
    "actuator_noise_only": {
        "noise": {
            "enabled": True,
            "sensor": {"enabled": False},
            "measurement_delay": {"enabled": False},
            "measurement_filter": {"enabled": False},
            "actuator": {"enabled": True, "torque_std_nm": 0.02, "gain_std": 0.01},
            "random_walk_bias": {"enabled": False},
        }
    },
    "full_noise_light": {
        "noise": {
            "enabled": True,
            "sensor": {"enabled": True},
            "measurement_delay": {"enabled": True, "delay_steps": 1},
            "measurement_filter": {"enabled": True, "cutoff_hz": 20.0},
            "actuator": {"enabled": True, "torque_std_nm": 0.02, "gain_std": 0.01},
            "random_walk_bias": {"enabled": False},
        }
    },
    "full_noise_medium": {
        "noise": {
            "enabled": True,
            "sensor": {"enabled": True},
            "measurement_delay": {"enabled": True, "delay_steps": 2},
            "measurement_filter": {"enabled": True, "cutoff_hz": 15.0},
            "actuator": {"enabled": True, "torque_std_nm": 0.05, "gain_std": 0.02},
            "random_walk_bias": {"enabled": False},
        }
    },
}


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=_ROOT / "configs" / "rl_workspace_5d_sac.yaml")
    ap.add_argument(
        "--model-path",
        type=Path,
        default=_ROOT
        / "debug_outputs"
        / "workspace_5d_residual_rl"
        / "runs"
        / "ws5d_residual_medium_train_rs05_30k_s4"
        / "checkpoints"
        / "best_model_by_smooth_score.zip",
    )
    ap.add_argument(
        "--vecnormalize-path",
        type=Path,
        default=_ROOT
        / "debug_outputs"
        / "workspace_5d_residual_rl"
        / "runs"
        / "ws5d_residual_medium_train_rs05_30k_s4"
        / "vecnormalize"
        / "vecnormalize.pkl",
    )
    ap.add_argument("--curriculum-stage", type=str, default="medium_train")
    ap.add_argument("--seed-start", type=int, default=30000)
    ap.add_argument("--num-episodes", type=int, default=50)
    ap.add_argument("--detail-seed", type=int, default=30000)
    ap.add_argument(
        "--out-dir",
        type=Path,
        default=_ROOT / "debug_outputs" / "workspace_5d_residual_rl" / "noise_robustness",
    )
    ap.add_argument(
        "--cases",
        nargs="*",
        default=list(NOISE_PRESET_DELTAS.keys()),
        help="Subset of noise cases to run.",
    )
    return ap.parse_args()


def _pct(z: float, s: float) -> float:
    if not np.isfinite(z) or abs(z) < 1e-18:
        return float("nan")
    return float((z - s) / z * 100.0)


def _mean_rollout_field(rows: list[dict[str, Any]], prefix: str, field: str) -> float:
    vals = [float(r[f"{prefix}{field}"]) for r in rows]
    return float(np.mean(vals))


def _plot_detail(mz: dict[str, Any], ms: dict[str, Any], plots_dir: Path, case: str, seed: int) -> None:
    t = np.asarray(mz["time"], dtype=np.float64)
    en = np.asarray(mz["e_norm"], dtype=np.float64)
    en_s = np.asarray(ms["e_norm"], dtype=np.float64)
    enm = np.linalg.norm(np.asarray(mz["ee_err_xyz_measured"], dtype=np.float64), axis=1)
    ens = np.linalg.norm(np.asarray(ms["ee_err_xyz_measured"], dtype=np.float64), axis=1)
    n = int(min(t.shape[0], en.shape[0], en_s.shape[0], enm.shape[0], ens.shape[0]))
    t = t[:n]
    en = en[:n]
    en_s = en_s[:n]
    enm = enm[:n]
    ens = ens[:n]
    plt.figure(figsize=(8, 4))
    plt.plot(t, en, label="true EE error ‖·‖ (zero res.)")
    plt.plot(t, enm, "--", label="measured EE error ‖·‖ (zero res.)")
    plt.plot(t, en_s, label="true EE ‖·‖ (SAC)")
    plt.plot(t, ens, "--", label="measured EE ‖·‖ (SAC)")
    plt.legend()
    plt.xlabel("time (s)")
    plt.title(f"EE error true vs measured — {case} seed={seed}")
    plt.tight_layout()
    plt.savefig(plots_dir / f"detail_{case}_seed{seed}_ee_errors.png", dpi=160)
    plt.close()

    xd = np.asarray(mz["x_des"], dtype=np.float64)[:n]
    xa_z = np.asarray(mz["x_act"], dtype=np.float64)[:n]
    xa_s = np.asarray(ms["x_act"], dtype=np.float64)[:n]
    plt.figure(figsize=(8, 5))
    for i, lab in enumerate(["x", "y", "z"]):
        plt.subplot(3, 1, i + 1)
        plt.plot(t, xd[:, i], "k--", lw=1.0, label="des")
        plt.plot(t, xa_z[:, i], label=f"act zero {lab}")
        plt.plot(t, xa_s[:, i], label=f"act SAC {lab}")
        plt.ylabel(lab)
        if i == 0:
            plt.legend(fontsize=8)
    plt.xlabel("time (s)")
    plt.suptitle(f"XYZ desired vs actual (sim true) — {case}")
    plt.tight_layout()
    plt.savefig(plots_dir / f"detail_{case}_seed{seed}_xyz.png", dpi=160)
    plt.close()

    rde = np.asarray(mz["roll_des"], dtype=np.float64)[:n]
    r_az = np.asarray(mz["roll_act"], dtype=np.float64)[:n]
    r_as = np.asarray(ms["roll_act"], dtype=np.float64)[:n]
    pde = np.asarray(mz["pitch_des"], dtype=np.float64)[:n]
    p_az = np.asarray(mz["pitch_act"], dtype=np.float64)[:n]
    p_as = np.asarray(ms["pitch_act"], dtype=np.float64)[:n]
    plt.figure(figsize=(8, 4))
    plt.subplot(1, 2, 1)
    plt.plot(t, rde, "k--", label="roll des")
    plt.plot(t, r_az, label="roll act zero")
    plt.plot(t, r_as, label="roll act SAC")
    plt.legend(fontsize=8)
    plt.subplot(1, 2, 2)
    plt.plot(t, pde, "k--", label="pitch des")
    plt.plot(t, p_az, label="pitch act zero")
    plt.plot(t, p_as, label="pitch act SAC")
    plt.legend(fontsize=8)
    plt.suptitle(f"Roll/pitch — {case}")
    plt.tight_layout()
    plt.savefig(plots_dir / f"detail_{case}_seed{seed}_roll_pitch.png", dpi=160)
    plt.close()

    Wz = np.asarray(mz["W_residual_used"], dtype=np.float64)[:n]
    Ws = np.asarray(ms["W_residual_used"], dtype=np.float64)[:n]
    plt.figure(figsize=(8, 4))
    for k in range(5):
        plt.plot(t, Ws[:, k], label=f"SAC W{k}")
    plt.plot(t, Wz[:, 0], "k:", lw=0.8, alpha=0.5, label="zero (0)")
    plt.legend(ncol=3, fontsize=8)
    plt.title("Residual wrench (SAC)")
    plt.tight_layout()
    plt.savefig(plots_dir / f"detail_{case}_seed{seed}_residual_wrench.png", dpi=160)
    plt.close()

    tz = np.asarray(mz["tau_jnt_cmd"], dtype=np.float64)[:n]
    ts = np.asarray(ms["tau_jnt_cmd"], dtype=np.float64)[:n]
    plt.figure(figsize=(8, 5))
    for j in range(4):
        plt.subplot(4, 1, j + 1)
        plt.plot(t, tz[:, j], label="tau joint cmd zero")
        plt.plot(t, ts[:, j], label="tau joint cmd SAC")
        plt.ylabel(f"τ{j+1}")
        if j == 0:
            plt.legend(fontsize=7)
    plt.xlabel("time (s)")
    plt.suptitle(f"Joint torque command — {case}")
    plt.tight_layout()
    plt.savefig(plots_dir / f"detail_{case}_seed{seed}_tau_jnt_cmd.png", dpi=160)
    plt.close()

    tai_z = np.asarray(mz.get("tau_act_ideal_full", np.zeros((0, 4))), dtype=np.float64)
    tno_z = np.asarray(mz.get("tau_act_noisy_full", np.zeros((0, 4))), dtype=np.float64)
    tai_s = np.asarray(ms.get("tau_act_ideal_full", np.zeros((0, 4))), dtype=np.float64)
    tno_s = np.asarray(ms.get("tau_act_noisy_full", np.zeros((0, 4))), dtype=np.float64)
    if min(tai_z.shape[0], tno_z.shape[0], tai_s.shape[0], tno_s.shape[0]) >= n:
        tai_z = tai_z[:n]
        tno_z = tno_z[:n]
        tai_s = tai_s[:n]
        tno_s = tno_s[:n]
        plt.figure(figsize=(8, 5))
        for j in range(4):
            plt.subplot(4, 1, j + 1)
            plt.plot(t, tai_z[:, j], label="act τ clean (zero pol.)")
            plt.plot(t, tno_z[:, j], "--", label="act τ noisy cmd (zero)")
            plt.plot(t, tai_s[:, j], alpha=0.7, label="act τ clean (SAC)")
            plt.plot(t, tno_s[:, j], ":", alpha=0.7, label="act τ noisy (SAC)")
            plt.ylabel(f"motor{j+1}")
            if j == 0:
                plt.legend(fontsize=6, ncol=2)
        plt.xlabel("time (s)")
        plt.suptitle(f"Actuator torque clean vs noisy — {case}")
        plt.tight_layout()
        plt.savefig(plots_dir / f"detail_{case}_seed{seed}_tau_act_clean_vs_noisy.png", dpi=160)
        plt.close()


def main() -> None:
    args = parse_args()
    cfg = load_yaml(args.config)
    out = Path(args.out_dir)
    plots_dir = out / "plots"
    out.mkdir(parents=True, exist_ok=True)
    plots_dir.mkdir(parents=True, exist_ok=True)

    model = SAC.load(str(args.model_path), device="auto")
    vec: VecNormalize | None = None
    if args.vecnormalize_path.is_file():

        def _make_base() -> Any:
            from envs.pmi_workspace_5d_residual_env import PMIWorkspace5DResidualEnv

            c = copy.deepcopy(cfg)
            c.setdefault("curriculum", {})["stage"] = str(args.curriculum_stage)
            return PMIWorkspace5DResidualEnv(config=c)

        dv = DummyVecEnv([_make_base])
        vec = VecNormalize.load(str(args.vecnormalize_path), dv)
        vec.training = False
        vec.norm_reward = False

    def pol_zero(obs: np.ndarray, _i: int) -> np.ndarray:
        return np.zeros(5, dtype=np.float32)

    def pol_sac(obs: np.ndarray, _i: int) -> np.ndarray:
        o = np.asarray(obs, dtype=np.float32).reshape(1, -1)
        if vec is not None:
            o = vec.normalize_obs(o)
        a, _ = model.predict(o, deterministic=True)
        return np.asarray(a, dtype=np.float32).reshape(-1)

    all_rows: list[dict[str, Any]] = []
    cases = [c for c in args.cases if c in NOISE_PRESET_DELTAS]
    if not cases:
        raise SystemExit("No valid noise cases.")

    for case in cases:
        cfg_case = copy.deepcopy(cfg)
        cfg_case.setdefault("curriculum", {})["stage"] = str(args.curriculum_stage)
        cfg_case = _deep_merge(cfg_case, NOISE_PRESET_DELTAS[case])
        do_rnd, rnd_profile = _effective_cable_randomize(cfg_case)

        case_rows: list[dict[str, Any]] = []
        for ep in range(int(args.num_episodes)):
            seed = int(args.seed_start + ep)
            opts: dict[str, Any] = {}
            if do_rnd:
                opts["randomize_cable"] = True
                opts["cable_seed"] = seed
                opts["randomization_profile"] = rnd_profile

            z = rollout_one_episode_metrics(policy_fn=pol_zero, config=cfg_case, seed=seed, options=opts)
            s = rollout_one_episode_metrics(policy_fn=pol_sac, config=cfg_case, seed=seed, options=opts)
            z["smooth_score"] = float(workspace_smooth_score(z))
            s["smooth_score"] = float(workspace_smooth_score(s))

            def fmag(d: dict[str, Any], k: str) -> float:
                v = d.get(k, 0.0)
                return float(v) if np.isscalar(v) else float("nan")

            row: dict[str, Any] = {
                "noise_case": case,
                "episode": ep,
                "seed": seed,
                "zero_rms_ee": fmag(z, "rms_ee_error"),
                "sac_rms_ee": fmag(s, "rms_ee_error"),
                "zero_rms_ee_measured": fmag(z, "rms_ee_error_measured"),
                "sac_rms_ee_measured": fmag(s, "rms_ee_error_measured"),
                "zero_final_ee": fmag(z, "final_ee_error"),
                "sac_final_ee": fmag(s, "final_ee_error"),
                "zero_final_ee_measured": fmag(z, "final_ee_error_measured"),
                "sac_final_ee_measured": fmag(s, "final_ee_error_measured"),
                "zero_rms_hf": fmag(z, "rms_highfreq"),
                "sac_rms_hf": fmag(s, "rms_highfreq"),
                "zero_smooth": float(z["smooth_score"]),
                "sac_smooth": float(s["smooth_score"]),
                "zero_sat": fmag(z, "saturation_fraction"),
                "sac_sat": fmag(s, "saturation_fraction"),
                "zero_lim": fmag(z, "limit_fraction"),
                "sac_lim": fmag(s, "limit_fraction"),
                "zero_ncon": fmag(z, "ncon_max"),
                "sac_ncon": fmag(s, "ncon_max"),
            }
            row["delta_rms_ee"] = row["sac_rms_ee"] - row["zero_rms_ee"]
            row["delta_final_ee"] = row["sac_final_ee"] - row["zero_final_ee"]
            row["delta_rms_hf"] = row["sac_rms_hf"] - row["zero_rms_hf"]
            row["delta_sat"] = row["sac_sat"] - row["zero_sat"]
            row["delta_lim"] = row["sac_lim"] - row["zero_lim"]
            case_rows.append(row)
            all_rows.append(row)

        # Detail plots for selected seed (SAC + zero rollouts)
        if int(args.detail_seed) in [int(args.seed_start + i) for i in range(int(args.num_episodes))]:
            ds = int(args.detail_seed)
            opts_d: dict[str, Any] = {}
            if do_rnd:
                opts_d["randomize_cable"] = True
                opts_d["cable_seed"] = ds
                opts_d["randomization_profile"] = rnd_profile
            mz = rollout_one_episode_metrics(policy_fn=pol_zero, config=cfg_case, seed=ds, options=opts_d)
            ms = rollout_one_episode_metrics(policy_fn=pol_sac, config=cfg_case, seed=ds, options=opts_d)
            _plot_detail(mz, ms, plots_dir, case, ds)

    csv_path = out / "noise_robustness_summary.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()) if all_rows else [])
        if all_rows:
            w.writeheader()
            w.writerows(all_rows)

    # Aggregate per case
    agg_lines: list[str] = ["# Noise robustness — paired zero vs SAC\n\n"]
    for case in cases:
        sub = [r for r in all_rows if r["noise_case"] == case]
        if not sub:
            continue
        agg_lines.append(f"## {case}\n\n")
        agg_lines.append(
            "| metric | zero | SAC | Δ (SAC−zero) |\n| --- | --- | --- | --- |\n"
            f"| RMS EE (true) | {_mean_rollout_field(sub, 'zero_', 'rms_ee'):.6f} | {_mean_rollout_field(sub, 'sac_', 'rms_ee'):.6f} | {_mean_rollout_field(sub, '', 'delta_rms_ee'):.6f} |\n"
            f"| final EE (true) | {_mean_rollout_field(sub, 'zero_', 'final_ee'):.6f} | {_mean_rollout_field(sub, 'sac_', 'final_ee'):.6f} | {_mean_rollout_field(sub, '', 'delta_final_ee'):.6f} |\n"
            f"| RMS EE (measured) | {_mean_rollout_field(sub, 'zero_', 'rms_ee_measured'):.6f} | {_mean_rollout_field(sub, 'sac_', 'rms_ee_measured'):.6f} | — |\n"
            f"| RMS HF | {_mean_rollout_field(sub, 'zero_', 'rms_hf'):.6f} | {_mean_rollout_field(sub, 'sac_', 'rms_hf'):.6f} | {_mean_rollout_field(sub, '', 'delta_rms_hf'):.6f} |\n"
            f"| smooth | {_mean_rollout_field(sub, 'zero_', 'smooth'):.6f} | {_mean_rollout_field(sub, 'sac_', 'smooth'):.6f} | — |\n"
            f"| sat frac | {_mean_rollout_field(sub, 'zero_', 'sat'):.6f} | {_mean_rollout_field(sub, 'sac_', 'sat'):.6f} | {_mean_rollout_field(sub, '', 'delta_sat'):.6f} |\n"
            f"| lim frac | {_mean_rollout_field(sub, 'zero_', 'lim'):.6f} | {_mean_rollout_field(sub, 'sac_', 'lim'):.6f} | {_mean_rollout_field(sub, '', 'delta_lim'):.6f} |\n"
            f"| ncon max | {_mean_rollout_field(sub, 'zero_', 'ncon'):.6f} | {_mean_rollout_field(sub, 'sac_', 'ncon'):.6f} | — |\n\n"
        )
        imp = _pct(_mean_rollout_field(sub, "zero_", "rms_ee"), _mean_rollout_field(sub, "sac_", "rms_ee"))
        agg_lines.append(f"- Mean RMS EE improvement (true): **{imp:.2f} %**\n\n")

    agg_lines.append(
        "## Notes\n\n"
        "- Primary metrics use **true** EE error from simulation.\n"
        "- **Measured** columns use the noisy/delayed/filtered observation pipeline (see `configs/rl_workspace_5d_sac.yaml` → `noise`).\n"
        "- If noisy evaluation degrades vs clean, consider lowering residual authority, stronger filtering, or future noise-aware training.\n"
    )
    (out / "noise_robustness_report.md").write_text("".join(agg_lines), encoding="utf-8")

    # Summary bar charts (per case means)
    def case_means(field_z: str, field_s: str) -> tuple[list[float], list[float]]:
        zv, sv = [], []
        for case in cases:
            sub = [r for r in all_rows if r["noise_case"] == case]
            zv.append(_mean_rollout_field(sub, "", field_z) if sub else float("nan"))
            sv.append(_mean_rollout_field(sub, "", field_s) if sub else float("nan"))
        return zv, sv

    x = np.arange(len(cases))
    w = 0.35
    for fname, fz, fs, ylab in (
        ("noise_case_rms_ee.png", "zero_rms_ee", "sac_rms_ee", "RMS EE (true)"),
        ("noise_case_final_ee.png", "zero_final_ee", "sac_final_ee", "final EE (true)"),
        ("noise_case_hf.png", "zero_rms_hf", "sac_rms_hf", "RMS HF"),
    ):
        zv, sv = case_means(fz, fs)
        plt.figure(figsize=(9, 4))
        plt.bar(x - w / 2, zv, w, label="VSD zero residual")
        plt.bar(x + w / 2, sv, w, label="VSD + SAC")
        plt.xticks(x, cases, rotation=25, ha="right")
        plt.ylabel(ylab)
        plt.legend()
        plt.title("Noise robustness — paired means")
        plt.tight_layout()
        plt.savefig(plots_dir / fname, dpi=160)
        plt.close()

    zsat, ssat = case_means("zero_sat", "sac_sat")
    zlim, slim = case_means("zero_lim", "sac_lim")
    plt.figure(figsize=(9, 4))
    plt.bar(x - w / 2, zsat, w, label="sat zero")
    plt.bar(x + w / 2, ssat, w, label="sat SAC")
    plt.plot(x, zlim, "C2o-", label="lim zero")
    plt.plot(x, slim, "C3s--", label="lim SAC")
    plt.xticks(x, cases, rotation=25, ha="right")
    plt.ylabel("fraction")
    plt.title("Saturation / limit fractions")
    plt.legend()
    plt.tight_layout()
    plt.savefig(plots_dir / "noise_case_saturation_limit.png", dpi=160)
    plt.close()

    d_rms = [_mean_rollout_field([r for r in all_rows if r["noise_case"] == c], "", "delta_rms_ee") for c in cases]
    d_fin = [_mean_rollout_field([r for r in all_rows if r["noise_case"] == c], "", "delta_final_ee") for c in cases]
    d_hf = [_mean_rollout_field([r for r in all_rows if r["noise_case"] == c], "", "delta_rms_hf") for c in cases]
    plt.figure(figsize=(9, 4))
    plt.bar(x - w / 2, d_rms, w, label="Δ RMS EE")
    plt.bar(x + w / 2, d_fin, w, label="Δ final EE")
    plt.axhline(0.0, color="k", lw=0.6)
    plt.xticks(x, cases, rotation=25, ha="right")
    plt.ylabel("SAC − zero")
    plt.title("Paired deltas (true errors)")
    plt.legend()
    plt.twinx()
    plt.plot(x, d_hf, "C3o-", label="Δ RMS HF")
    plt.ylabel("Δ HF", color="C3")
    plt.tight_layout()
    plt.savefig(plots_dir / "noise_case_delta_metrics.png", dpi=160)
    plt.close()

    print(f"Wrote {csv_path} and report under {out}")


if __name__ == "__main__":
    main()
