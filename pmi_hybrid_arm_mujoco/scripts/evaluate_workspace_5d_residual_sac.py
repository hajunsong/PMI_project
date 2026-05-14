#!/usr/bin/env python3
"""Paired evaluation: zero residual vs SAC (workspace 5D VSD + residual)."""

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
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from envs.pmi_workspace_5d_residual_env import PMIWorkspace5DResidualEnv
from utils.mujoco_helpers import load_yaml
from utils.workspace_5d_rl_metrics import rollout_one_episode_metrics, workspace_smooth_score

# Relaxed pass for `medium_v2_improved` (mild / medium_v2).
_MEDIUM_V2_EPS_FIN = 2e-4
_MEDIUM_V2_EPS_HF = 5e-5
_MEDIUM_V2_EPS_SAT = 1e-4
_MEDIUM_V2_EPS_LIM = 1e-4

# Pass for `medium_train_improved` (aligned with rollout spec; tunable independently).
_MEDIUM_TRAIN_EPS_FIN = 2e-4
_MEDIUM_TRAIN_EPS_HF = 5e-5
_MEDIUM_TRAIN_EPS_SAT = 1e-4
_MEDIUM_TRAIN_EPS_LIM = 1e-4


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=_ROOT / "configs" / "rl_workspace_5d_sac.yaml")
    ap.add_argument("--model-path", type=Path, required=True, help="SAC .zip checkpoint")
    ap.add_argument("--vecnormalize-path", type=Path, default=None, help="vecnormalize.pkl (optional)")
    ap.add_argument("--out-dir", type=Path, default=None, help="default: debug_outputs/workspace_5d_residual_rl/evaluation")
    ap.add_argument("--num-episodes", type=int, default=10)
    ap.add_argument("--seed-start", type=int, default=0)
    ap.add_argument("--randomize-cable", action="store_true", help="Force cable randomization (paired cable_seed)")
    ap.add_argument("--cable-seed-start", type=int, default=200000)
    ap.add_argument(
        "--curriculum-stage",
        type=str,
        default=None,
        choices=("deterministic", "mild", "medium_v2", "medium_train", "stress"),
        help="Overrides curriculum.stage (default: YAML). Match the training stage.",
    )
    ap.add_argument("--learning-rate", type=float, default=None, help="Override sac.learning_rate (parity with train CLI)")
    ap.add_argument("--residual-force-scale", type=float, default=None, help="Override residual.residual_force_scale")
    ap.add_argument("--residual-moment-scale", type=float, default=None, help="Override residual.residual_moment_scale")
    return ap.parse_args()


def _effective_cable_randomize(cfg_eval: dict[str, Any], force_rnd: bool) -> tuple[bool, str]:
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
    if force_rnd:
        randomize = True
    return randomize, profile


def _pct_improvement(zero_val: float, sac_val: float) -> float:
    """Percent reduction in error metric when SAC is better: (zero - sac) / zero * 100."""
    z = float(zero_val)
    if not np.isfinite(z) or abs(z) < 1e-15:
        return float("nan")
    return float((z - float(sac_val)) / z * 100.0)


def main() -> None:
    args = parse_args()
    cfg = load_yaml(args.config)
    cfg_eval = copy.deepcopy(cfg)
    if args.curriculum_stage is not None:
        cfg_eval.setdefault("curriculum", {})["stage"] = str(args.curriculum_stage)
    if args.learning_rate is not None:
        cfg_eval.setdefault("sac", {})["learning_rate"] = float(args.learning_rate)
    if args.residual_force_scale is not None:
        cfg_eval.setdefault("residual", {})["residual_force_scale"] = float(args.residual_force_scale)
    if args.residual_moment_scale is not None:
        cfg_eval.setdefault("residual", {})["residual_moment_scale"] = float(args.residual_moment_scale)

    out = args.out_dir or (_ROOT / "debug_outputs" / "workspace_5d_residual_rl" / "evaluation")
    out.mkdir(parents=True, exist_ok=True)

    model = SAC.load(str(args.model_path), device="auto")

    vec: VecNormalize | None = None
    if args.vecnormalize_path is not None and Path(args.vecnormalize_path).is_file():

        def _make() -> Any:
            return PMIWorkspace5DResidualEnv(config=cfg_eval)

        dv = DummyVecEnv([_make])
        vec = VecNormalize.load(str(args.vecnormalize_path), dv)
        vec.training = False
        vec.norm_reward = False

    mkeys = [
        "rms_ee_error",
        "final_ee_error",
        "max_ee_error",
        "rms_roll_error",
        "rms_pitch_error",
        "rms_highfreq",
        "p2p_error_norm",
        "saturation_fraction",
        "limit_fraction",
        "ncon_max",
        "mean_residual_wrench_norm",
        "mean_tau_res_norm",
        "mean_reward",
    ]

    rows: list[dict[str, Any]] = []

    do_rnd, rnd_profile = _effective_cable_randomize(cfg_eval, bool(args.randomize_cable))

    for ep in range(int(args.num_episodes)):
        seed = int(args.seed_start + ep)
        opts: dict[str, Any] = {}
        if do_rnd:
            opts["randomize_cable"] = True
            opts["cable_seed"] = int(args.cable_seed_start + ep)
            opts["randomization_profile"] = rnd_profile

        def pol_zero(obs: np.ndarray, _i: int) -> np.ndarray:
            return np.zeros(5, dtype=np.float32)

        def pol_sac(obs: np.ndarray, _i: int) -> np.ndarray:
            o = np.asarray(obs, dtype=np.float32).reshape(1, -1)
            if vec is not None:
                o = vec.normalize_obs(o)
            a, _ = model.predict(o, deterministic=True)
            return np.asarray(a, dtype=np.float32).reshape(-1)

        z = rollout_one_episode_metrics(policy_fn=pol_zero, config=cfg_eval, seed=seed, options=opts)
        s = rollout_one_episode_metrics(policy_fn=pol_sac, config=cfg_eval, seed=seed, options=opts)

        z["smooth_score"] = workspace_smooth_score(z)
        s["smooth_score"] = workspace_smooth_score(s)

        row: dict[str, Any] = {"episode": ep, "seed": seed, "cable_seed": int(opts.get("cable_seed", -1))}
        for k in mkeys:
            row[f"zero_{k}"] = float(z[k])
            row[f"sac_{k}"] = float(s[k])
            row[f"delta_{k}"] = float(s[k]) - float(z[k])
        row["delta_smooth_score"] = float(s["smooth_score"]) - float(z["smooth_score"])
        row["zero_smooth_score"] = float(z["smooth_score"])
        row["sac_smooth_score"] = float(s["smooth_score"])
        rows.append(row)

    csv_path = out / "evaluation_summary.csv"
    keys = list(rows[0].keys()) if rows else []
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(r)

    def col_mean(k: str) -> float:
        xs = [float(r[k]) for r in rows if k in r]
        return float(np.mean(xs)) if xs else float("nan")

    z_rms = col_mean("zero_rms_ee_error")
    s_rms = col_mean("sac_rms_ee_error")
    z_fin = col_mean("zero_final_ee_error")
    s_fin = col_mean("sac_final_ee_error")
    z_hf = col_mean("zero_rms_highfreq")
    s_hf = col_mean("sac_rms_highfreq")
    d_rms = col_mean("delta_rms_ee_error")
    d_fin = col_mean("delta_final_ee_error")
    d_hf = col_mean("delta_rms_highfreq")
    d_sat = col_mean("delta_saturation_fraction")
    d_lim = col_mean("delta_limit_fraction")
    sac_ncon_max_ep = max((float(r["sac_ncon_max"]) for r in rows), default=float("nan"))
    zero_ncon_max_ep = max((float(r["zero_ncon_max"]) for r in rows), default=float("nan"))

    pct_rms = _pct_improvement(z_rms, s_rms)
    pct_fin = _pct_improvement(z_fin, s_fin)
    pct_hf = _pct_improvement(z_hf, s_hf)

    deterministic_improved = bool(
        np.isfinite(d_rms)
        and d_rms < 0.0
        and np.isfinite(d_fin)
        and d_fin <= 0.0
        and np.isfinite(d_hf)
        and d_hf <= 0.0
        and np.isfinite(d_sat)
        and d_sat <= 1e-12
        and np.isfinite(d_lim)
        and d_lim <= 1e-12
        and np.isfinite(sac_ncon_max_ep)
        and sac_ncon_max_ep <= 1e-12
        and np.isfinite(zero_ncon_max_ep)
        and zero_ncon_max_ep <= 1e-12
    )

    medium_v2_improved = bool(
        np.isfinite(d_rms)
        and d_rms < 0.0
        and np.isfinite(d_fin)
        and d_fin <= _MEDIUM_V2_EPS_FIN
        and np.isfinite(d_hf)
        and d_hf <= _MEDIUM_V2_EPS_HF
        and np.isfinite(d_sat)
        and d_sat <= _MEDIUM_V2_EPS_SAT
        and np.isfinite(d_lim)
        and d_lim <= _MEDIUM_V2_EPS_LIM
        and np.isfinite(sac_ncon_max_ep)
        and sac_ncon_max_ep <= 1e-12
        and np.isfinite(zero_ncon_max_ep)
        and zero_ncon_max_ep <= 1e-12
    )

    medium_train_improved = bool(
        np.isfinite(d_rms)
        and d_rms < 0.0
        and np.isfinite(d_fin)
        and d_fin <= _MEDIUM_TRAIN_EPS_FIN
        and np.isfinite(d_hf)
        and d_hf <= _MEDIUM_TRAIN_EPS_HF
        and np.isfinite(d_sat)
        and d_sat <= _MEDIUM_TRAIN_EPS_SAT
        and np.isfinite(d_lim)
        and d_lim <= _MEDIUM_TRAIN_EPS_LIM
        and np.isfinite(sac_ncon_max_ep)
        and sac_ncon_max_ep <= 1e-12
        and np.isfinite(zero_ncon_max_ep)
        and zero_ncon_max_ep <= 1e-12
    )

    stage = str((cfg_eval.get("curriculum") or {}).get("stage", "deterministic"))
    next_rec: str
    if not np.isfinite(d_rms) or d_rms >= 0.0:
        next_rec = (
            "SAC RMS가 악화되었거나 비교 불가입니다. 잔차 스케일을 낮추거나 이전 커리큘럼 단계로 되돌려 "
            "재학습하는 방안을 검토하세요."
        )
    elif stage == "stress":
        next_rec = (
            "`stress`는 **평가 전용** 케이블 랜덤화 프로파일입니다. SAC **학습**에 사용하지 마세요. "
            "지표를 기록하고, 필요 시 보수적 하이퍼 재튜닝·추가 평가만 검토하세요."
        )
    elif stage == "medium_train":
        if medium_train_improved:
            next_rec = (
                "`medium_train` 통과로 **첫 견고 학습 체크포인트** 후보입니다. 결정론적·mild·medium_v2·medium_train을 "
                "한데 묶어 비교·최종 리포트·영상을 준비하세요. **Stress/평가 전용 프로파일**은 통과 확인 뒤에만 사용합니다."
            )
        elif d_rms < 0.0 and (d_fin > _MEDIUM_TRAIN_EPS_FIN or d_hf > _MEDIUM_TRAIN_EPS_HF):
            next_rec = (
                "RMS는 개선되었으나 final 또는 HF가 `medium_train` 허용을 넘겼습니다. `residual_force_scale=0.5` 유지, "
                "`w_final_xyz`·HF 보상 가중을 키우고 맹목적으로 스텝만 늘리지 마세요."
            )
        elif d_sat > _MEDIUM_TRAIN_EPS_SAT or d_lim > _MEDIUM_TRAIN_EPS_LIM:
            next_rec = (
                "포화·limit 악화입니다. `residual_force_scale≈0.3`, `residual_moment_scale≈0.01`, LR `1e-4` 또는 `5e-5`로 "
                "`medium_train` 재시도를 검토하세요."
            )
        else:
            next_rec = (
                "`medium_train` 기준 미달입니다. 잔차 스케일을 줄이거나 `medium_v2`에서 재안정화 후 다시 시도하세요."
            )
    elif stage == "medium_v2":
        if medium_v2_improved:
            next_rec = (
                "`medium_v2` 완화 기준을 통과했습니다. 다음으로 `--curriculum-stage medium_train` **준비**가 가능합니다. "
                "스크립트/문서만 갱신하고, 학습은 별도 명시 후에 진행하세요."
            )
        elif d_rms < 0.0 and (d_fin > _MEDIUM_V2_EPS_FIN or d_hf > _MEDIUM_V2_EPS_HF):
            next_rec = (
                "RMS는 개선되었으나 final 또는 RMS HF가 허용 범위를 넘겼습니다. `residual_force_scale=0.5`를 유지한 채 "
                "보상에서 `w_final_xyz`·HF 관련 항을 키우는 방향을 검토하고, `medium_train`으로는 아직 가지 마세요."
            )
        elif d_sat > _MEDIUM_V2_EPS_SAT or d_lim > _MEDIUM_V2_EPS_LIM:
            next_rec = (
                "포화 또는 limit 위반이 늘었습니다. `residual_force_scale≈0.3`, `residual_moment_scale≈0.01`, "
                "학습률 `1e-4` 또는 `5e-5`로 `medium_v2`를 재시도하세요."
            )
        else:
            next_rec = (
                "`medium_v2`가 완화 기준에 미달입니다. 잔차 스케일을 줄이거나 `mild`에서 추가 안정화 후 "
                "`medium_v2`를 반복하세요."
            )
    elif stage == "mild":
        if medium_v2_improved:
            next_rec = (
                "Mild 단계에서 로버스트 개선이 확인되었습니다. 다음은 `--curriculum-stage medium_v2` 학습·평가입니다. "
                "`medium_train`은 건너뛰지 마세요."
            )
        else:
            next_rec = (
                "Mild에서 완화 통과에 실패했습니다. 잔차 스케일·보상 스무딩을 조정하거나 결정론적 단계로 되돌리세요."
            )
    elif s_rms > 0.005:
        next_rec = (
            "RMS EE는 개선되었으나 아직 0.005 m보다 큽니다. 결정론적 설정에서 학습을 이어가거나 "
            "짧은 스케일 애블레이션을 검토하세요."
        )
    elif np.isfinite(d_fin) and d_fin <= 0.0 and np.isfinite(d_hf) and d_hf <= 0.0:
        next_rec = (
            "결정론적 기준에서 RMS·final·HF가 안정적입니다. 다음은 `--curriculum-stage mild` 랜덤화 단계입니다."
        )
    else:
        next_rec = (
            "RMS는 개선되나 final 또는 HF가 엄격 기준에서 악화했습니다. 잔차 스케일·최종구간 패널티를 조정하세요."
        )

    md = "# Workspace 5D VSD vs SAC residual (paired)\n\n"
    md += f"- Model: `{args.model_path}`\n"
    md += f"- Config: `{args.config}` (eval merge에 `--curriculum-stage` 등 반영)\n"
    md += f"- Curriculum stage (effective): `{stage}`\n"
    md += f"- Cable randomize (effective): `{do_rnd}` (profile `{rnd_profile}`)\n"
    md += f"- Episodes: {args.num_episodes}, seed_start={args.seed_start}\n\n"
    md += "## Mean metrics\n\n"
    md += "| Metric | Zero | SAC | Δ (SAC−zero) |\n"
    md += "| --- | --- | --- | --- |\n"
    for name, zk, sk, dk in (
        ("RMS EE", "zero_rms_ee_error", "sac_rms_ee_error", "delta_rms_ee_error"),
        ("Final EE", "zero_final_ee_error", "sac_final_ee_error", "delta_final_ee_error"),
        ("RMS HF", "zero_rms_highfreq", "sac_rms_highfreq", "delta_rms_highfreq"),
        ("Saturation frac.", "zero_saturation_fraction", "sac_saturation_fraction", "delta_saturation_fraction"),
        ("ncon max", "zero_ncon_max", "sac_ncon_max", "delta_ncon_max"),
        ("Limit frac.", "zero_limit_fraction", "sac_limit_fraction", "delta_limit_fraction"),
        ("Smooth", "zero_smooth_score", "sac_smooth_score", "delta_smooth_score"),
    ):
        md += f"| {name} | {col_mean(zk):.6f} | {col_mean(sk):.6f} | {col_mean(dk):.6f} |\n"

    md += "\n## 포화 / limit / 접촉 요약\n\n"
    md += f"- 평균 Δ 포화율: {d_sat:.6f}, 평균 Δ limit 비율: {d_lim:.6f}\n"
    md += f"- 에피소드별 ncon max의 최댓값 — zero: {zero_ncon_max_ep:.0f}, SAC: {sac_ncon_max_ep:.0f}\n"

    md += "\n## Error reduction vs zero baseline (%)\n\n"
    md += "(정의: (zero − SAC) / zero × 100%. SAC가 오차가 더 작으면 양수.)\n\n"
    md += f"- RMS EE: {pct_rms:.4f} %\n"
    md += f"- Final EE: {pct_fin:.4f} %\n"
    md += f"- RMS HF: {pct_hf:.4f} %\n"

    md += "\n## Pass / fail: `deterministic_improved` (strict)\n\n"
    md += "조건: ΔRMS EE < 0, Δfinal EE ≤ 0, ΔRMS HF ≤ 0, Δsat ≤ 0, Δlimit ≤ 0, "
    md += "모든 에피소드에서 zero/SAC의 ncon max가 0.\n\n"
    md += f"- **`deterministic_improved`**: `{'true' if deterministic_improved else 'false'}`\n"

    md += "\n## Pass / fail: `medium_v2_improved` (relaxed, mild / medium_v2 / …)\n\n"
    md += f"허용: Δfinal EE ≤ {_MEDIUM_V2_EPS_FIN}, ΔRMS HF ≤ {_MEDIUM_V2_EPS_HF}, "
    md += f"Δsat ≤ {_MEDIUM_V2_EPS_SAT}, Δlimit ≤ {_MEDIUM_V2_EPS_LIM}, "
    md += "ΔRMS EE < 0, 에피소드 전체 ncon max = 0.\n\n"
    md += f"- **`medium_v2_improved`**: `{'true' if medium_v2_improved else 'false'}`\n"

    md += "\n## Pass / fail: `medium_train_improved`\n\n"
    md += f"허용: Δfinal EE ≤ {_MEDIUM_TRAIN_EPS_FIN}, ΔRMS HF ≤ {_MEDIUM_TRAIN_EPS_HF}, "
    md += f"Δsat ≤ {_MEDIUM_TRAIN_EPS_SAT}, Δlimit ≤ {_MEDIUM_TRAIN_EPS_LIM}, "
    md += "ΔRMS EE < 0, 에피소드 전체 ncon max = 0.\n\n"
    md += f"- **`medium_train_improved`**: `{'true' if medium_train_improved else 'false'}`\n"

    md += "\n## Next-step recommendation (heuristic)\n\n"
    md += f"{next_rec}\n"

    md += f"\nCSV: `{csv_path}`\n"

    (out / "evaluation_report.md").write_text(md, encoding="utf-8")
    (out / "eval_config_snapshot.yaml").write_text(
        yaml.safe_dump(cfg_eval, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    print(f"Wrote {csv_path} and {out / 'evaluation_report.md'}")


if __name__ == "__main__":
    main()
