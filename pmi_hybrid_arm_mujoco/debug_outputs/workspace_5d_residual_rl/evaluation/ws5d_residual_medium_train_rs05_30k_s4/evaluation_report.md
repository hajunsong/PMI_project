# Workspace 5D VSD vs SAC residual (paired)

- Model: `debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_medium_train_rs05_30k_s4/checkpoints/best_model_by_smooth_score.zip`
- Config: `configs/rl_workspace_5d_sac.yaml` (eval merge에 `--curriculum-stage` 등 반영)
- Curriculum stage (effective): `medium_train`
- Cable randomize (effective): `True` (profile `medium_train`)
- Episodes: 50, seed_start=10000

## Mean metrics

| Metric | Zero | SAC | Δ (SAC−zero) |
| --- | --- | --- | --- |
| RMS EE | 0.006744 | 0.003740 | -0.003004 |
| Final EE | 0.001297 | 0.000683 | -0.000614 |
| RMS HF | 0.000638 | 0.000543 | -0.000095 |
| Saturation frac. | 0.000000 | 0.000000 | 0.000000 |
| ncon max | 0.000000 | 0.000000 | 0.000000 |
| Limit frac. | 0.000000 | 0.000000 | 0.000000 |
| Smooth | 0.015257 | 0.010971 | -0.004286 |

## 포화 / limit / 접촉 요약

- 평균 Δ 포화율: 0.000000, 평균 Δ limit 비율: 0.000000
- 에피소드별 ncon max의 최댓값 — zero: 0, SAC: 0

## Error reduction vs zero baseline (%)

(정의: (zero − SAC) / zero × 100%. SAC가 오차가 더 작으면 양수.)

- RMS EE: 44.5443 %
- Final EE: 47.3492 %
- RMS HF: 14.9073 %

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

`medium_train` 통과로 **첫 견고 학습 체크포인트** 후보입니다. 결정론적·mild·medium_v2·medium_train을 한데 묶어 비교·최종 리포트·영상을 준비하세요. **Stress/평가 전용 프로파일**은 통과 확인 뒤에만 사용합니다.

CSV: `debug_outputs/workspace_5d_residual_rl/evaluation/ws5d_residual_medium_train_rs05_30k_s4/evaluation_summary.csv`
