# Workspace 5D VSD vs SAC residual (paired)

- Model: `debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_mild_rs05_30k_s2/checkpoints/best_model_by_smooth_score.zip`
- Config: `configs/rl_workspace_5d_sac.yaml` (eval merge에 `--curriculum-stage` 등 반영)
- Curriculum stage (effective): `mild`
- Cable randomize (effective): `True` (profile `mild`)
- Episodes: 30, seed_start=10000

## Mean metrics

| Metric | Zero | SAC | Δ (SAC−zero) |
| --- | --- | --- | --- |
| RMS EE | 0.006251 | 0.003064 | -0.003187 |
| Final EE | 0.001396 | 0.000419 | -0.000977 |
| RMS HF | 0.000593 | 0.000413 | -0.000180 |
| Saturation frac. | 0.000000 | 0.000000 | 0.000000 |
| ncon max | 0.000000 | 0.000000 | 0.000000 |
| Limit frac. | 0.000000 | 0.000000 | 0.000000 |
| Smooth | 0.014377 | 0.009516 | -0.004861 |

## Error reduction vs zero baseline (%)

(정의: (zero − SAC) / zero × 100%. SAC가 오차가 더 작으면 양수.)

- RMS EE: 50.9841 %
- Final EE: 69.9762 %
- RMS HF: 30.3317 %

## Pass / fail: `deterministic_improved`

조건: ΔRMS EE < 0, Δfinal EE ≤ 0, ΔRMS HF ≤ 0, Δsat ≤ 0, Δlimit ≤ 0, 모든 에피소드에서 zero/SAC의 ncon max가 0.

- **`deterministic_improved`**: `true`

## Next-step recommendation (heuristic)

RMS가 목표대로 내려갔고 final / HF가 기준 대비 안정적입니다. 다음 단계로 `--curriculum-stage mild` 같은 완만한 랜덤화 커리큘럼을 **준비·검토**할 수 있습니다 (바로 `medium_train`으로 가지 마세요).

CSV: `debug_outputs/workspace_5d_residual_rl/evaluation/ws5d_residual_mild_rs05_30k_s2/evaluation_summary.csv`
