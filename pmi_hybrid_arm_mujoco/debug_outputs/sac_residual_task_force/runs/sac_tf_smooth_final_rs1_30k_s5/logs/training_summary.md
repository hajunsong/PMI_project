# SAC residual training summary

- **profile**: medium_train
- **run_name**: sac_tf_smooth_final_rs1_30k_s5
- **requested timesteps**: 30000
- **baseline mean episode return (zero policy)**: -27654.835788247456
- **stop / completion reason**: completed
- **successful_tracking_improvement** (final mean EE RMS < baseline mean EE RMS): False

Artifacts: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_smooth_final_rs1_30k_s5/checkpoints/`, `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_smooth_final_rs1_30k_s5/logs/`, `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_smooth_final_rs1_30k_s5/tensorboard/`.

## Best checkpoints (from periodic eval)

- **best reward checkpoint** (`best_model_by_reward.zip`): reward=-46489.319193374846, step=25000
- **best EE RMS checkpoint** (`best_model_by_ee_rms.zip`): mean_ee_rms=0.020719210532231205, step=20000 (mean final EE error at that eval: 0.01849162008651211, sat=0.7445298429714285, lim=0.02195142636463867)
- **best combined tracking** (`best_model_by_combined_tracking.zip`): score=0.0297049141645345 (mean EE RMS + 0.5×mean final EE error), step=25000
- **best smooth tracking** (`best_model_by_smooth_tracking.zip`): score=3.317663930012385, step=25000 (lower = better RMS + weighted final / velocity / HF / torque rates; see `logs/best_metrics.yaml`)

## Diagnostic table (baseline vs final eval)

| Metric | Baseline (zero policy) | Final eval |
| --- | --- | --- |
| mean episode return | -27654.8 | -46774.6 |
| mean EE RMS (m) | 0.0191044 | 0.0207295 |
| mean saturation fraction | 0.7528 | 0.743659 |
| mean limit violation fraction | 0.02 | 0.0216183 |

- **Percent improvement in EE RMS** (baseline 대비 감소): -8.51% (baseline 대비 EE RMS 감소율)
- **Percent change in mean return** (의미 있을 때만 참고): -69.14% (return 변화율; 음수 보상 스케일에서 해석 유의)

## Paired evaluation

같은 케이블/환경 시드에서 잔차 0 베이스라인과 SAC를 짝지어 평가하면, 랜덤 시드 변동에 가려진 개선 여부를 확인할 수 있습니다.

```bash
python scripts/evaluate_sac_residual.py --paired-seeds --model-path <run_dir>/checkpoints/final_model.zip \
  --vecnormalize-path <run_dir>/vecnormalize/vecnormalize.pkl \
  --config configs/rl_sac.yaml --profile medium_train --seed-start 10000 --num-episodes 20
```

