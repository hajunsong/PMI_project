# Paired SAC residual evaluation

## Setup

1. **Model path (SAC checkpoint)**: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_30k_s2/checkpoints/best_model_by_ee_rms.zip`
2. **Profile**: `medium_train`
3. **Model checkpoint path**: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_30k_s2/checkpoints/best_model_by_ee_rms.zip`
4. **VecNormalize path**: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_30k_s2/vecnormalize/vecnormalize.pkl`
5. **seed_start**: `10000`
6. **num_episodes**: `30`

- Config YAML: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/configs/rl_sac.yaml`
- Training ``rl_overrides`` merged from: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_30k_s2/logs/training_args.yaml` *(if file exists)*
- CSV: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_30k_s2/logs/paired_evaluation.csv`
- Report: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_30k_s2/logs/paired_evaluation_report.md`
- Plots: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_30k_s2/logs/paired_plots`

## Summary metrics

### Final EE error

- mean final EE zero: 0.0182189
- mean final EE SAC: 0.0198252
- mean delta final EE: 0.00160626

### EE RMS

- mean ee_rms_zero: 0.0207603
- mean ee_rms_sac: 0.0195542
- mean delta_ee_rms: -0.00120609
- median delta_ee_rms: -0.00183364
- std delta_ee_rms: 0.00540899

### Reward

- mean reward_zero: -305.735
- mean reward_sac: -285.694
- mean delta_reward: 20.0409

### Saturation / limit fractions

- mean saturation_frac_zero: 0.745049
- mean saturation_frac_sac: 0.741327
- mean delta_saturation_frac: -0.00372245
- mean limit_frac_zero: 0.021619
- mean limit_frac_sac: 0.0199717
- mean delta_limit_frac: -0.00164726

### Episode counts

- number of improved episodes (delta_ee_rms < 0): 22
- number of worsened episodes (delta_ee_rms > 0): 8
- improvement ratio (improved / num_episodes): 0.7333

### Contacts / collisions

- max ncon_zero (over episodes): 0
- max ncon_sac (over episodes): 0

## Classification

- **paired_result** (aggregate EE RMS): `tracking_improved`
- **paired_final_error_classification**: `tracking_improved_but_final_error_worse`

## Recommendation

EE RMS가 짝 평가에서 일관되게 개선되었습니다. 더 긴 학습 또는 `medium` 등 다른 프로파일에서 추가 평가를 검토할 수 있습니다.

**종료 시점 EE:** 평균 EE RMS는 좋아졌으나 마지막 스텝 EE 오차가 나빠졌습니다. `configs/rl_sac.yaml`의 `use_terminal_final_error_penalty`와 `w_final_ee`를 사용해 학습을 이어가세요.

