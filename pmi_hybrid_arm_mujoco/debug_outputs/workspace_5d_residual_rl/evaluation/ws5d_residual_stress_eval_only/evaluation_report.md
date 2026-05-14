# Workspace 5D VSD vs SAC residual (paired)

- Model: `debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_medium_train_rs05_30k_s4/checkpoints/best_model_by_smooth_score.zip`
- Config: `configs/rl_workspace_5d_sac.yaml` (eval merge에 `--curriculum-stage` 등 반영)
- Curriculum stage (effective): `stress`
- Cable randomize (effective): `True` (profile `stress`)
- Episodes: 50, seed_start=20000

## Mean metrics

| Metric | Zero | SAC | Δ (SAC−zero) |
| --- | --- | --- | --- |
| RMS EE | 0.053114 | 0.039678 | -0.013436 |
| Final EE | 0.145339 | 0.116655 | -0.028684 |
| RMS HF | 0.030338 | 0.022304 | -0.008034 |
| Saturation frac. | 0.010841 | 0.004613 | -0.006228 |
| ncon max | 0.000000 | 0.000000 | 0.000000 |
| Limit frac. | 0.012889 | 0.007834 | -0.005055 |
| Smooth | 0.282378 | 0.219938 | -0.062440 |

## 포화 / limit / 접촉 요약

- 평균 Δ 포화율: -0.006228, 평균 Δ limit 비율: -0.005055
- 에피소드별 ncon max의 최댓값 — zero: 0, SAC: 0

## Error reduction vs zero baseline (%)

(정의: (zero − SAC) / zero × 100%. SAC가 오차가 더 작으면 양수.)

- RMS EE: 25.2963 %
- Final EE: 19.7360 %
- RMS HF: 26.4813 %

## Pass / fail: `deterministic_improved` (strict)

조건: ΔRMS EE < 0, Δfinal EE ≤ 0, ΔRMS HF ≤ 0, Δsat ≤ 0, Δlimit ≤ 0, 모든 에피소드에서 zero/SAC의 ncon max가 0.

- **`deterministic_improved`**: `true`

## Pass / fail: `medium_v2_improved` (relaxed, mild / medium_v2 / …)

허용: Δfinal EE ≤ 0.0002, ΔRMS HF ≤ 5e-05, Δsat ≤ 0.0001, Δlimit ≤ 0.0001, ΔRMS EE < 0, 에피소드 전체 ncon max = 0.

- **`medium_v2_improved`**: `true`

## Pass / fail: `medium_train_improved`

허용: Δfinal EE ≤ 0.0002, ΔRMS HF ≤ 5e-05, Δsat ≤ 0.0001, Δlimit ≤ 0.0001, ΔRMS EE < 0, 에피소드 전체 ncon max = 0.

- **`medium_train_improved`**: `true`

## Next-step recommendation (heuristic)

`stress`는 **평가 전용** 케이블 랜덤화 프로파일입니다. SAC **학습**에 사용하지 마세요. 지표를 기록하고, 필요 시 보수적 하이퍼 재튜닝·추가 평가만 검토하세요.

CSV: `debug_outputs/workspace_5d_residual_rl/evaluation/ws5d_residual_stress_eval_only/evaluation_summary.csv`
