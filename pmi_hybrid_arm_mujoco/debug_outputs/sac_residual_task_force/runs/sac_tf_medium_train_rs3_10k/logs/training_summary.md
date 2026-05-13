# SAC residual training summary

- **profile**: medium_train
- **run_name**: sac_tf_medium_train_rs3_10k
- **requested timesteps**: 10000
- **baseline mean episode return (zero policy)**: -203.74261043951554
- **stop / completion reason**: early_stop:eval_plateau_patience

Artifacts: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_medium_train_rs3_10k/checkpoints/`, `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_medium_train_rs3_10k/logs/`, `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_medium_train_rs3_10k/tensorboard/`.

## Diagnostic table (baseline vs final eval)

| Metric | Baseline (zero policy) | Final eval |
| --- | --- | --- |
| mean episode return | -203.743 | -169.402 |
| mean EE RMS (m) | 0.025063 | 0.020782 |
| mean saturation fraction | 0.8228 | 0.754 |
| mean limit violation fraction | 0.0104 | 0.0044 |

- **Percent improvement in EE RMS** (baseline 대비 감소): 17.08% (baseline 대비 EE RMS 감소율)
- **Percent change in mean return** (의미 있을 때만 참고): 16.85% (return 변화율; 음수 보상 스케일에서 해석 유의)
