# Paired SAC residual evaluation

## Setup

1. **Model path (SAC checkpoint)**: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_baseline_relative_rs1_30k_s6/checkpoints/best_model_by_relative_smooth_score.zip`
2. **Profile**: `medium_train`
3. **Model checkpoint path**: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_baseline_relative_rs1_30k_s6/checkpoints/best_model_by_relative_smooth_score.zip`
4. **VecNormalize path**: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_baseline_relative_rs1_30k_s6/vecnormalize/vecnormalize.pkl`
5. **seed_start**: `10000`
6. **num_episodes**: `30`

- Config YAML: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/configs/rl_sac.yaml`
- Training ``rl_overrides`` merged from: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_baseline_relative_rs1_30k_s6/logs/training_args.yaml` *(if file exists)*
- CSV: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_baseline_relative_rs1_30k_s6/logs/paired_evaluation_relative_smooth.csv`
- Report: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_baseline_relative_rs1_30k_s6/logs/paired_evaluation_report.md`
- Plots: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_baseline_relative_rs1_30k_s6/logs/paired_plots`

## Summary metrics

### Final EE error

- mean final EE zero: 0.0172177
- mean final EE SAC: 0.0190923
- mean delta final EE: 0.00187457

### EE RMS

- mean ee_rms_zero: 0.0206511
- mean ee_rms_sac: 0.0207158
- mean delta_ee_rms: 6.4662e-05
- median delta_ee_rms: -2.99781e-05
- std delta_ee_rms: 0.000426056

### Reward

- mean reward_zero: -1057.07
- mean reward_sac: -1043.54
- mean delta_reward: 13.531

### Saturation / limit fractions

- mean saturation_frac_zero: 0.744382
- mean saturation_frac_sac: 0.741977
- mean delta_saturation_frac: -0.00240543
- mean limit_frac_zero: 0.021619
- mean limit_frac_sac: 0.0211446
- mean delta_limit_frac: -0.000474406

### Oscillation / smoothness (mean paired deltas = SAC − zero)

- mean delta RMS EE error velocity: -0.0010937
- mean delta RMS high-frequency EE error: 9.80137e-05
- mean delta peak-to-peak EE error norm: 0.000396835
- mean delta RMS residual force rate: 8.31705
- mean delta RMS tau_total rate: -11.0763

### Episode counts

- number of improved episodes (delta_ee_rms < 0): 16
- number of worsened episodes (delta_ee_rms > 0): 14
- improvement ratio (improved / num_episodes): 0.5333

### Contacts / collisions

- max ncon_zero (over episodes): 0
- max ncon_sac (over episodes): 0

## Classification

- **paired_result** (aggregate EE RMS): `no_clear_difference`
- **paired_final_error_classification**: *(none)*

### Multi-label (RMS-only classification is ambiguous without oscillation)

- *(no tag matched thresholds)*

### Joint smooth-tracking classification (paired means)

정의: RMS만이 아니라 종료 EE·진동 관련 평균 델타(SAC − zero)·포화/리밋 증가를 함께 본 결과입니다.
- **`ambiguous`** (`baseline_relative_success` / `rms_only_improved` / `failed` / `ambiguous` / `inconclusive`)

## Recommendation

평균 차이가 매우 작습니다. 보상 스케일·액션(`residual_force_scale`)·관측 정규화를 조정한 뒤 다시 짝 평가하세요.

