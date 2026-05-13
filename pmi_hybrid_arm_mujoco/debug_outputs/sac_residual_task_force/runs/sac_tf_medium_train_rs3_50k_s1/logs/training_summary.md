# SAC residual training summary

- **profile**: medium_train
- **run_name**: sac_tf_medium_train_rs3_50k_s1
- **requested timesteps**: 50000
- **baseline mean episode return (zero policy)**: -262.250730179685
- **stop / completion reason**: completed

Artifacts: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_medium_train_rs3_50k_s1/checkpoints/`, `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_medium_train_rs3_50k_s1/logs/`, `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_medium_train_rs3_50k_s1/tensorboard/`.

## Diagnostic table (baseline vs final eval)

| Metric | Baseline (zero policy) | Final eval |
| --- | --- | --- |
| mean episode return | -262.251 | -247.825 |
| mean EE RMS (m) | 0.0191044 | 0.0212584 |
| mean saturation fraction | 0.7528 | 0.754847 |
| mean limit violation fraction | 0.02 | 0.0196216 |

- **Percent improvement in EE RMS** (baseline 대비 감소): -11.28% (baseline 대비 EE RMS 감소율)
- **Percent change in mean return** (의미 있을 때만 참고): 5.50% (return 변화율; 음수 보상 스케일에서 해석 유의)

