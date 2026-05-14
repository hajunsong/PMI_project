# SAC residual training summary

- **profile**: medium_train
- **run_name**: sac_tf_medium_train_rs3_50k_s1
- **requested timesteps**: 50000
- **baseline mean episode return (zero policy)**: -262.250730179685
- **stop / completion reason**: completed
- **successful_tracking_improvement** (final mean EE RMS < baseline mean EE RMS): false

Artifacts: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_medium_train_rs3_50k_s1/checkpoints/`, `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_medium_train_rs3_50k_s1/logs/`, `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_medium_train_rs3_50k_s1/tensorboard/`.

## Best checkpoints (from periodic eval)

- *(이 런은 인프라 업데이트 이전에 생성되어 `logs/best_metrics.yaml` 및 `best_model_by_ee_rms.zip`이 없을 수 있습니다. 새 학습 런에서는 주기 평가마다 생성됩니다.)*

## Diagnostic table (baseline vs final eval)

| Metric | Baseline (zero policy) | Final eval |
| --- | --- | --- |
| mean episode return | -262.251 | -247.825 |
| mean EE RMS (m) | 0.0191044 | 0.0212584 |
| mean saturation fraction | 0.7528 | 0.754847 |
| mean limit violation fraction | 0.02 | 0.0196216 |

- **Percent improvement in EE RMS** (baseline 대비 감소): -11.28% (baseline 대비 EE RMS 감소율)
- **Percent change in mean return** (의미 있을 때만 참고): 5.50% (return 변화율; 음수 보상 스케일에서 해석 유의)

## Paired evaluation

같은 케이블/환경 시드에서 잔차 0 베이스라인과 SAC를 짝지어 평가하면, 랜덤 시드 변동에 가려진 개선 여부를 확인할 수 있습니다.

```bash
cd pmi_hybrid_arm_mujoco
python scripts/evaluate_sac_residual.py --paired-seeds --model-path debug_outputs/sac_residual_task_force/runs/sac_tf_medium_train_rs3_50k_s1/checkpoints/final_model.zip \
  --vecnormalize-path debug_outputs/sac_residual_task_force/runs/sac_tf_medium_train_rs3_50k_s1/vecnormalize/vecnormalize.pkl \
  --config configs/rl_sac.yaml --profile medium_train --seed-start 10000 --num-episodes 20
```
