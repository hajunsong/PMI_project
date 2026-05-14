# Workspace 5D VSD vs SAC residual (paired)

- Model: `debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_medium_v2_rs05_30k_s3/checkpoints/best_model_by_smooth_score.zip`
- Config: `configs/rl_workspace_5d_sac.yaml` (eval merge에 `--curriculum-stage` 등 반영)
- Curriculum stage (effective): `medium_v2`
- Cable randomize (effective): `True` (profile `medium_v2`)
- Episodes: 30, seed_start=10000

## Mean metrics

| Metric | Zero | SAC | Δ (SAC−zero) |
| --- | --- | --- | --- |
| RMS EE | 0.006860 | 0.003819 | -0.003041 |
| Final EE | 0.001422 | 0.000611 | -0.000812 |
| RMS HF | 0.000643 | 0.000500 | -0.000143 |
| Saturation frac. | 0.000000 | 0.000000 | 0.000000 |
| ncon max | 0.000000 | 0.000000 | 0.000000 |
| Limit frac. | 0.000000 | 0.000000 | 0.000000 |
| Smooth | 0.015603 | 0.011086 | -0.004517 |

## Error reduction vs zero baseline (%)

(정의: (zero − SAC) / zero × 100%. SAC가 오차가 더 작으면 양수.)

- RMS EE: 44.3239 %
- Final EE: 57.0669 %
- RMS HF: 22.2486 %

## Pass / fail: `deterministic_improved` (strict)

조건: ΔRMS EE < 0, Δfinal EE ≤ 0, ΔRMS HF ≤ 0, Δsat ≤ 0, Δlimit ≤ 0, 모든 에피소드에서 zero/SAC의 ncon max가 0.

- **`deterministic_improved`**: `true`

## Pass / fail: `medium_v2_improved` (relaxed, mild / medium_v2 / …)

허용: Δfinal EE ≤ 0.0002, ΔRMS HF ≤ 5e-05, Δsat ≤ 0.0001, Δlimit ≤ 0.0001, ΔRMS EE < 0, 에피소드 전체 ncon max = 0.

- **`medium_v2_improved`**: `true`

## Next-step recommendation (heuristic)

`medium_v2` 완화 기준을 통과했습니다. 다음으로 `--curriculum-stage medium_train` **준비**가 가능합니다. 스크립트/문서만 갱신하고, 학습은 별도 명시 후에 진행하세요.

CSV: `debug_outputs/workspace_5d_residual_rl/evaluation/ws5d_residual_medium_v2_rs05_30k_s3/evaluation_summary.csv`
