#!/usr/bin/env python3
"""Emit ``logs/next_run_recommendation.md`` and print baseline-only analysis (no stale eval rows)."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import yaml

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))


SUGGESTED_NEXT_RUN_COMMAND = r"""python scripts/train_sac_residual.py \
  --timesteps 50000 \
  --profile medium_train \
  --run-name sac_tf_medium_train_rs3_50k_s1 \
  --seed 1 \
  --progress \
  --early-stop \
  --eval-freq 5000 \
  --eval-episodes 20 \
  --checkpoint-freq 10000 \
  --min-train-steps-before-stop 30000 \
  --patience-evals 8 \
  --min-improvement 0.01 \
  --learning-rate 0.0003 \
  --batch-size 256 \
  --buffer-size 200000 \
  --learning-starts 1000 \
  --residual-force-scale 3.0 \
  --tau-jnt-limit 30 \
  --use-vecnormalize"""


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def read_yaml(path: Path) -> dict | None:
    if not path.is_file():
        return None
    try:
        out = yaml.safe_load(path.read_text(encoding="utf-8"))
        return out if isinstance(out, dict) else None
    except Exception:
        return None


def fnum(d: dict[str, str], k: str) -> float | None:
    v = d.get(k)
    if v in ("", None):
        return None
    try:
        return float(v)
    except ValueError:
        return None


def int_gs(d: dict[str, str]) -> int:
    v = fnum(d, "global_step")
    return int(v) if v is not None else -1


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run-dir", type=Path, required=True)
    args = ap.parse_args()
    run_dir = Path(args.run_dir).resolve()
    logs = run_dir / "logs"
    out = logs / "next_run_recommendation.md"

    ta = read_yaml(logs / "training_args.yaml") or {}
    run_name_cfg = str(ta.get("run_name", "")).strip()
    timesteps_req = ta.get("timesteps")
    try:
        ts_int = int(timesteps_req) if timesteps_req is not None else None
    except (TypeError, ValueError):
        ts_int = None

    bl_yaml = logs / "baseline_eval.yaml"
    bl_data = read_yaml(bl_yaml)
    baseline_pack: dict[str, float] | None = None
    if bl_data is not None:
        m = bl_data.get("baseline_zero_policy_metrics")
        if isinstance(m, dict) and m.get("mean_episode_return") is not None:
            try:
                baseline_pack = {str(k): float(v) for k, v in m.items()}
            except (TypeError, ValueError):
                baseline_pack = None

    ev_all = read_csv(logs / "eval_log.csv")

    run_names_in_csv = {r.get("run_name", "").strip() for r in ev_all if r.get("run_name")}
    run_names_in_csv.discard("")
    mixed_run_names = bool(run_names_in_csv) and (
        (run_name_cfg and any(n != run_name_cfg for n in run_names_in_csv))
        or len(run_names_in_csv) > 1
    )

    def row_matches_run(r: dict[str, str]) -> bool:
        if not run_name_cfg:
            return True
        rn = (r.get("run_name") or "").strip()
        return not rn or rn == run_name_cfg

    ev_run = [r for r in ev_all if row_matches_run(r)]

    bl_rows = [r for r in ev_run if (r.get("eval_type") or "").strip() == "baseline_zero"]
    baseline_mismatch = False
    if baseline_pack is not None and bl_rows:
        yret = float(baseline_pack.get("mean_episode_return", float("nan")))
        for r in bl_rows:
            cr = fnum(r, "mean_episode_return")
            if cr is not None and abs(cr - yret) > 1e-4:
                baseline_mismatch = True
                break

    periodic = [r for r in ev_run if (r.get("eval_type") or "").strip() == "periodic_eval"]
    periodic.sort(key=int_gs)
    first_p = periodic[0] if periodic else None
    last_p = periodic[-1] if periodic else None

    finals = [r for r in ev_run if (r.get("eval_type") or "").strip() == "final_eval"]
    finals.sort(key=int_gs)
    last_f = finals[-1] if finals else None

    last_eval = last_f or last_p

    early_p = logs / "early_stop_reason.txt"
    early = early_p.read_text(encoding="utf-8").strip() if early_p.is_file() else ""

    ro_rows = read_csv(logs / "rollout_diagnostics.csv")

    lines: list[str] = []
    lines += ["# 다음 SAC 실행 제안", "", f"- Run: `{run_dir}`", ""]

    if early:
        lines += ["## 중단 원인", "", f"- `{early}`", ""]
    elif not ev_all:
        lines += ["## 메모", "", "- `logs/eval_log.csv` 없음 또는 비어 있음.", ""]

    baseline_ret = baseline_sat = baseline_lim = baseline_ee = None
    lines += ["## 기준선 (zero policy)", ""]
    if baseline_pack is None:
        lines += [
            "- **경고**: `logs/baseline_eval.yaml` 없거나 손상됨 — baseline 대비 해석·비교를 건너뜁니다.",
            "",
        ]
    else:
        baseline_ret = baseline_pack.get("mean_episode_return")
        baseline_sat = baseline_pack.get("mean_sat_frac")
        baseline_lim = baseline_pack.get("mean_lim_frac")
        baseline_ee = baseline_pack.get("mean_ee_rms")
        lines.append(f"- YAML (`baseline_eval.yaml`): mean_return={baseline_ret}, sat={baseline_sat}, lim={baseline_lim}, EE_RMS={baseline_ee}")
        lines.append("- (CSV의 오래된 행에서 baseline을 추정하지 **않음**.)")
        lines.append("")
        if baseline_mismatch or mixed_run_names:
            lines += [
                "## 경고: 로그 혼선 / baseline 불일치",
                "",
                "- `eval_log.csv`의 `baseline_zero` 행이 YAML과 맞지 않거나, 서로 다른 `run_name` 행이 섞여 있을 수 있습니다.",
                "- **결과 해석 전에** 런 디렉터리를 정리하고 새 런으로 로그를 다시 쌓는 것을 권장합니다.",
                "",
            ]

    fr_ret = fnum(last_f, "mean_episode_return") if last_f else None
    fr_ee = fnum(last_f, "mean_ee_rms") if last_f else None
    fr_sat = fnum(last_f, "mean_sat_frac") if last_f else None
    fr_lim = fnum(last_f, "mean_lim_frac") if last_f else None

    lines += ["## 종합 해석", ""]
    if baseline_pack is None or last_f is None:
        lines += ["- final eval 또는 baseline YAML 부족으로 자동 분류를 건너뜁니다.", ""]
    else:
        ret_imp = (
            fr_ret is not None
            and baseline_ret is not None
            and fr_ret > float(baseline_ret) + 1e-6
        )
        ee_imp = fr_ee is not None and baseline_ee is not None and fr_ee < float(baseline_ee) - 1e-9
        if ret_imp or ee_imp:
            lines += [
                "- **분류**: *유망하지만 결론 불충분(promising but inconclusive)* — zero baseline 대비 final eval에서 개선 신호가 있습니다.",
                "- **권장**: 스케일을 즉시 낮추기보다 **더 긴 학습(예: 50k+)**, 더 많은 eval 에피소드·고정 eval 시드로 재평가하는 것을 우선 검토합니다.",
                "",
            ]
        else:
            lines += ["- final eval이 baseline 대비 명확히 나아졌다고 보기 어렵습니다. 학습 설정·보상·환경 일관성을 점검하세요.", ""]

        if (
            fr_sat is not None
            and fr_lim is not None
            and baseline_sat is not None
            and baseline_lim is not None
            and fr_sat <= float(baseline_sat) + 1e-9
            and fr_lim <= float(baseline_lim) + 1e-9
        ):
            lines += [
                "- **포화·한계**: final에서 평균 포화·한계 비율이 **baseline 이하**입니다. 절대 포화율이 높아 보여도 baseline 대비 개선이면 **불건전으로 단정하지 않습니다**.",
                "",
            ]

    periodic_returns: list[float] = []
    for r in periodic:
        mr = fnum(r, "mean_episode_return")
        gs = int_gs(r)
        if mr is not None and gs >= 500:
            periodic_returns.append(float(mr))
    oscillating = False
    if len(periodic_returns) >= 4:
        pr_arr = np.asarray(periodic_returns, dtype=np.float64)
        if float(np.std(pr_arr)) > 35.0:
            oscillating = True
            lines += [
                "## Eval 변동",
                "",
                "- periodic eval의 mean_return 분산이 큽니다 — 노이즈 또는 불안정 학습 가능성.",
                "- **`eval_episodes` 증가** 및 **`eval_seed_start` / 고정 에피소드 시드** 사용으로 평가 분산을 줄이세요.",
                "",
            ]

    lines += ["## 최근 periodic eval", ""]
    if last_p is None:
        lines += ["- periodic eval 행 없음 (`eval_type=periodic_eval`).", ""]
    else:
        mr = fnum(last_p, "mean_episode_return")
        es = fnum(last_p, "mean_sat_frac")
        lf = fnum(last_p, "mean_lim_frac")
        ee = fnum(last_p, "mean_ee_rms")
        fq = fnum(last_p, "mean_finite_frac")
        ncon_frac = fnum(last_p, "frac_eps_any_ncon")
        man = fnum(last_p, "mean_action_norm")

        if mr is not None:
            suf = ""
            if baseline_pack is not None and baseline_ret is not None:
                suf = f" (YAML baseline ~ {float(baseline_ret):.4f})"
            lines.append(f"- global_step={last_p.get('global_step', '?')}: mean_return={mr:.4f}{suf}")
        if es is not None:
            lines.append(f"- 평균 포화 단계 비율: {es:.4f}")
        if lf is not None:
            lines.append(f"- 평균 한계 위반 단계 비율: {lf:.4f}")
        if ee is not None:
            lines.append(f"- 평균 EE RMS 오차(m): {ee:.6f}")
        if fq is not None:
            lines.append(f"- 에피소드 유한 비율: {fq:.4f}")
        if ncon_frac is not None:
            lines.append(f"- 접촉 에피소드 비율: {ncon_frac:.4f}")
        if man is not None:
            lines.append(f"- mean_action_norm (eval): {man:.6g}")
        lines.append("")

    if last_f is not None:
        lines += [
            "## 종료 시점 final eval",
            "",
            f"- global_step={last_f.get('global_step', '?')}",
        ]
        ee = fnum(last_f, "mean_ee_rms")
        if ee is not None:
            lines.append(f"- EE RMS(m): {ee:.6f}")
        lines.append("")

    if ts_int is not None and ts_int <= 1000:
        lines += [
            "## 스모크 테스트 (≤1000 스텝)",
            "",
            "- 매우 짧은 학습에서는 수렴 여부·학습 품질을 **판단하기 어렵습니다**.",
            "",
        ]

    action_columns = (
        "mean_action_norm",
        "max_action_norm",
        "mean_residual_force_norm",
        "max_residual_force_norm",
        "mean_residual_tau_norm",
        "max_residual_tau_norm",
        "mean_action_rate",
    )
    action_missing = False
    if last_eval is not None:
        for c in action_columns:
            if fnum(last_eval, c) is None:
                action_missing = True
                break
    else:
        action_missing = True

    if action_missing:
        lines += [
            "## 경고: 액션 norm 로그",
            "",
            "- `eval_log.csv`에 action/residual norm 또는 `mean_action_rate`가 비어 있거나 누락되었습니다.",
            "- 다음 실행 제안에 액션 규모 진단을 포함하려면 해당 컬럼이 **필요**합니다.",
            "",
        ]

    cat_all: dict[str, list[str]] = {
        "하이퍼파라미터·탐험": [],
        "포화·한계": [],
        "종단 오차·궤적": [],
        "안정성·접촉·수치": [],
        "다음 실행 명령 예시": [],
    }

    small_action_eval = False
    if last_eval is not None:
        ma = fnum(last_eval, "mean_action_norm")
        ee_ev = fnum(last_eval, "mean_ee_rms")
        if ma is not None and ma < 0.12 and ee_ev is not None and baseline_ee is not None and ee_ev < float(baseline_ee):
            small_action_eval = True
            cat_all["하이퍼파라미터·탐험"].append(
                "- eval에서 action norm이 작고 EE가 baseline 대비 개선: 우선 **`residual_force_scale` 유지** 후 더 긴 학습으로 관측하는 것을 권장."
            )

    if (
        baseline_pack is not None
        and last_p is not None
        and first_p is not None
        and baseline_ee is not None
        and baseline_sat is not None
        and baseline_lim is not None
    ):
        le = fnum(last_p, "mean_ee_rms")
        ls = fnum(last_p, "mean_sat_frac")
        ll = fnum(last_p, "mean_lim_frac")
        bs0 = fnum(first_p, "mean_sat_frac")
        bl0 = fnum(first_p, "mean_lim_frac")
        if (
            le is not None
            and ls is not None
            and ll is not None
            and le < float(baseline_ee) - 1e-6
            and ls > float(baseline_sat) + 0.02
            and ll > float(baseline_lim) + 0.005
            and (
                (bs0 is not None and ls > float(bs0) + 0.01)
                or (bl0 is not None and ll > float(bl0) + 0.005)
            )
        ):
            cat_all["포화·한계"].append(
                "- (마지막 periodic 기준) EE는 나아졌으나 포화·한계가 **baseline보다 악화**: `--residual-force-scale` 완만화를 검토."
            )

    if last_p is not None:
        mr = fnum(last_p, "mean_episode_return")
        es = fnum(last_p, "mean_sat_frac")
        lf = fnum(last_p, "mean_lim_frac")
        ee = fnum(last_p, "mean_ee_rms")
        fq = fnum(last_p, "mean_finite_frac")
        ncon_frac = fnum(last_p, "frac_eps_any_ncon")

        fin_beats_bl = False
        if last_f is not None and baseline_ret is not None:
            fm = fnum(last_f, "mean_episode_return")
            fin_beats_bl = fm is not None and fm > float(baseline_ret) + 1e-6

        if (
            baseline_pack is not None
            and mr is not None
            and baseline_ret is not None
            and mr < float(baseline_ret) - 1e-6
            and not fin_beats_bl
        ):
            cat_all["하이퍼파라미터·탐험"].append(
                "- (마지막 periodic 기준) mean_return < YAML baseline: learning_rate·learning_starts·보상 가중 검토."
            )

        if baseline_pack is None and es is not None and es > 0.08:
            cat_all["포화·한계"].append(
                "- (baseline 없음) 포화 비율 높음: YAML baseline 확보 후 상대 비교 권장."
            )
        elif (
            baseline_pack is not None
            and baseline_sat is not None
            and es is not None
            and es > float(baseline_sat) + 0.10
        ):
            cat_all["포화·한계"].append(
                "- baseline 대비 포화 비율 **과도 상승**: residual_force_scale·보상항 검토."
            )

        if baseline_pack is None and lf is not None and lf > 0.08:
            cat_all["포화·한계"].append("- (baseline 없음) 한계 위반 비율 높음: randomization·w_lim 검토.")
        elif (
            baseline_pack is not None
            and baseline_lim is not None
            and lf is not None
            and lf > float(baseline_lim) + 0.01
        ):
            cat_all["포화·한계"].append("- baseline 대비 한계 위반 상승: randomization 단계·w_lim 검토.")

        if ee is not None and ee > 0.05:
            cat_all["종단 오차·궤적"].append("- EE RMS 큼: w_ee, PD 게인, 궤적 난도 점진 상향.")

        if fq is not None and fq < 1.0 - 1e-6:
            cat_all["안정성·접촉·수치"].append("- 유한 상태 비율 감소: 학습률 완만화 및 로그 재현.")

        if ncon_frac is not None and ncon_frac > 0.05:
            cat_all["안정성·접촉·수치"].append("- 접촉 비율 높음: 잔차 크기·경로 회피 검토.")

    if oscillating and not small_action_eval:
        cat_all["하이퍼파라미터·탐험"].append(
            "- eval 지표가 크게 요동: `--eval-episodes` 상향 및 고정 eval 시드(`--eval-seed-start`) 활용을 권장."
        )

    if ro_rows:
        r_last = ro_rows[-1]
        ml2 = fnum(r_last, "mean_action_l2_ratio")
        mmx = fnum(r_last, "mean_max_abs_action")
        if ml2 is not None and ml2 < 0.05:
            cat_all["하이퍼파라미터·탐험"].append(
                f"- rollout action L2 비율 매우 작음(~{ml2:.4f}): ent_coef 상향 검토."
            )
        if mmx is not None and mmx > 1.005:
            cat_all["하이퍼파라미터·탐험"].append(
                f"- rollout 평균 max|a| ~{mmx:.4f}: 정책 포화 가능성 검토."
            )

    cat_all["하이퍼파라미터·탐험"].append(
        "- ent_coef·batch·TensorBoard 학습 지표(loss/entropy)를 함께 보고 미세 조정."
    )

    cat_all["다음 실행 명령 예시"].append(
        "다음은 **갱신된 기본 조기종료·평가 설정**을 반영한 50k 실행 예시입니다 (`cd pmi_hybrid_arm_mujoco` 후 실행)."
    )
    cat_all["다음 실행 명령 예시"].append("")
    cat_all["다음 실행 명령 예시"].append("```bash")
    for ln in SUGGESTED_NEXT_RUN_COMMAND.splitlines():
        cat_all["다음 실행 명령 예시"].append(ln)
    cat_all["다음 실행 명령 예시"].append("```")

    resume_hint = (
        "**선택: 체크포인트 재개** — `replay_buffer` 복원, VecNormalize(`vecnormalize.pkl`) 통계, "
        "`residual_force_scale`·`tau_jnt_limit` 등 학습 인자가 저장 시점과 **일치하는지 검증한 경우에만** 사용하세요."
    )
    cat_all["다음 실행 명령 예시"].append("")
    cat_all["다음 실행 명령 예시"].append(resume_hint)
    cat_all["다음 실행 명령 예시"].append("")
    cat_all["다음 실행 명령 예시"].append("```bash")
    cat_all["다음 실행 명령 예시"].append(
        "python scripts/train_sac_residual.py \\\n"
        f"  --resume-from {run_dir}/checkpoints/best_model.zip \\\n"
        "  # 위 50k 블록과 동일한 timesteps·seed·eval·학습 하이퍼파라미터 플래그 유지"
    )
    cat_all["다음 실행 명령 예시"].append("```")

    for title, items in cat_all.items():
        if not items:
            continue
        lines.append(f"### {title}")
        lines.extend(items)
        lines.append("")

    lines += ["---", "", f"파일 출력: `{out}`", ""]
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
