# Paired SAC residual evaluation

## Setup

1. **Model path (SAC checkpoint)**: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_smooth_final_rs1_30k_s5/checkpoints/best_model_by_smooth_tracking.zip`
2. **Profile**: `medium_train`
3. **Model checkpoint path**: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_smooth_final_rs1_30k_s5/checkpoints/best_model_by_smooth_tracking.zip`
4. **VecNormalize path**: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_smooth_final_rs1_30k_s5/vecnormalize/vecnormalize.pkl`
5. **seed_start**: `10000`
6. **num_episodes**: `30`

- Config YAML: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/configs/rl_sac.yaml`
- Training ``rl_overrides`` merged from: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_smooth_final_rs1_30k_s5/logs/training_args.yaml` *(if file exists)*
- CSV: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_smooth_final_rs1_30k_s5/logs/paired_evaluation_smooth.csv`
- Report: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_smooth_final_rs1_30k_s5/logs/paired_evaluation_report.md`
- Plots: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_smooth_final_rs1_30k_s5/logs/paired_plots`

## Summary metrics

### Final EE error

- mean final EE zero: 0.0179596
- mean final EE SAC: 0.0192774
- mean delta final EE: 0.00131776

### EE RMS

- mean ee_rms_zero: 0.0206239
- mean ee_rms_sac: 0.021496
- mean delta_ee_rms: 0.000872062
- median delta_ee_rms: 0.000165155
- std delta_ee_rms: 0.00323746

### Reward

- mean reward_zero: -47070.4
- mean reward_sac: -48061.2
- mean delta_reward: -990.742

### Saturation / limit fractions

- mean saturation_frac_zero: 0.743182
- mean saturation_frac_sac: 0.759234
- mean delta_saturation_frac: 0.0160512
- mean limit_frac_zero: 0.021619
- mean limit_frac_sac: 0.0217239
- mean delta_limit_frac: 0.000104938

### Oscillation / smoothness (mean paired deltas = SAC − zero)

- mean delta RMS EE error velocity: 0.0122257
- mean delta RMS high-frequency EE error: 0.000188861
- mean delta peak-to-peak EE error norm: 0.0011057
- mean delta RMS residual force rate: 27.2301
- mean delta RMS tau_total rate: 36.0685

### Episode counts

- number of improved episodes (delta_ee_rms < 0): 12
- number of worsened episodes (delta_ee_rms > 0): 18
- improvement ratio (improved / num_episodes): 0.4000

### Contacts / collisions

- max ncon_zero (over episodes): 0
- max ncon_sac (over episodes): 0

## Classification

- **paired_result** (aggregate EE RMS): `tracking_not_improved`
- **paired_final_error_classification**: *(none)*

### Multi-label (RMS-only classification is ambiguous without oscillation)

- *(no tag matched thresholds)*

## Recommendation

이 체크포인트로 학습 스텝만 늘리기보다 보상·잔차 스케일을 조정하고 다른 체크포인트를 검토하는 편이 낫습니다.

