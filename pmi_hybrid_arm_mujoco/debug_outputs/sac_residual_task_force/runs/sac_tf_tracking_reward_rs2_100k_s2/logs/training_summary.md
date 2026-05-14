# SAC residual training summary

- **profile**: medium_train
- **run_name**: sac_tf_tracking_reward_rs2_100k_s2
- **requested timesteps**: 100000
- **baseline mean episode return (zero policy)**: -299.1438827707665
- **stop / completion reason**: early_stop:eval_plateau_patience
- **successful_tracking_improvement** (final mean EE RMS < baseline mean EE RMS): False

Artifacts: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_100k_s2/checkpoints/`, `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_100k_s2/logs/`, `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_100k_s2/tensorboard/`.

## Best checkpoints (from periodic eval)

- **best reward checkpoint** (`best_model_by_reward.zip`): reward=-283.5631171889758, step=1
- **best EE RMS checkpoint** (`best_model_by_ee_rms.zip`): mean_ee_rms=0.0197053020542627, step=10000 (mean final EE error at that eval: 0.018070215134285698, sat=0.7551885985872687, lim=0.018897951120333694)
- **best combined tracking** (`best_model_by_combined_tracking.zip`): score=0.028740409621405547 (mean EE RMS + 0.5×mean final EE error), step=10000

## Diagnostic table (baseline vs final eval)

| Metric | Baseline (zero policy) | Final eval |
| --- | --- | --- |
| mean episode return | -299.144 | -332.046 |
| mean EE RMS (m) | 0.0191044 | 0.0211734 |
| mean saturation fraction | 0.7528 | 0.76774 |
| mean limit violation fraction | 0.02 | 0.0185946 |

- **Percent improvement in EE RMS** (baseline 대비 감소): -10.83% (baseline 대비 EE RMS 감소율)
- **Percent change in mean return** (의미 있을 때만 참고): -11.00% (return 변화율; 음수 보상 스케일에서 해석 유의)

## Paired evaluation

같은 케이블/환경 시드에서 잔차 0 베이스라인과 SAC를 짝지어 평가하면, 랜덤 시드 변동에 가려진 개선 여부를 확인할 수 있습니다.

```bash
python scripts/evaluate_sac_residual.py --paired-seeds --model-path <run_dir>/checkpoints/final_model.zip \
  --vecnormalize-path <run_dir>/vecnormalize/vecnormalize.pkl \
  --config configs/rl_sac.yaml --profile medium_train --seed-start 10000 --num-episodes 20
```

