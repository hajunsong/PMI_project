# Workspace 5D VSD vs SAC residual (paired)

- Model: `debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_det_rs05_resume_30k_s1/checkpoints/best_model_by_smooth_score.zip`
- Config: `configs/rl_workspace_5d_sac.yaml` (eval merge에 `--curriculum-stage` 등 반영)
- Curriculum stage (effective): `deterministic`
- Cable randomize (effective): `False` (profile `medium_train`)
- Episodes: 10, seed_start=0

## Mean metrics

| Metric | Zero | SAC | Δ (SAC−zero) |
| --- | --- | --- | --- |
| RMS EE | 0.006140 | 0.003215 | -0.002925 |
| Final EE | 0.001459 | 0.000792 | -0.000667 |
| RMS HF | 0.000584 | 0.000417 | -0.000166 |
| Saturation frac. | 0.000000 | 0.000000 | 0.000000 |
| ncon max | 0.000000 | 0.000000 | 0.000000 |
| Limit frac. | 0.000000 | 0.000000 | 0.000000 |
| Smooth | 0.014204 | 0.009916 | -0.004289 |

## Error reduction vs zero baseline (%)

(정의: (zero − SAC) / zero × 100%. SAC가 오차가 더 작으면 양수.)

- RMS EE: 47.6377 %
- Final EE: 45.7174 %
- RMS HF: 28.5266 %

## Pass / fail: `deterministic_improved`

조건: ΔRMS EE < 0, Δfinal EE ≤ 0, ΔRMS HF ≤ 0, Δsat ≤ 0, Δlimit ≤ 0, 모든 에피소드에서 zero/SAC의 ncon max가 0.

- **`deterministic_improved`**: `true`

## Next-step recommendation (heuristic)

RMS가 목표대로 내려갔고 final / HF가 기준 대비 안정적입니다. 다음 단계로 `--curriculum-stage mild` 같은 완만한 랜덤화 커리큘럼을 **준비·검토**할 수 있습니다 (바로 `medium_train`으로 가지 마세요).

CSV: `debug_outputs/workspace_5d_residual_rl/evaluation/ws5d_residual_det_rs05_resume_30k_s1/evaluation_summary.csv`
