# VSD-only vs VSD + SAC residual (paired comparison)

## Configuration
- SAC checkpoint: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_100k_s2/checkpoints/best_model_by_ee_rms.zip`
- VecNormalize: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_100k_s2/vecnormalize/vecnormalize.pkl`
- Profile: `medium_train`
- Seeds: `10000` … `10004`
- RL config: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/configs/rl_sac.yaml`
- Overrides merged from `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_100k_s2/logs/training_args.yaml` (if exists)

## Dynamics / control
- Plant: hybrid cable-arm model path from RL YAML (`configs/rl_sac.yaml` unless overridden via `--config`).
- Controller: nominal joint-space VSD (`Kq`,`Dq` in config) computes base joint torque.
- **SAC action**: bounded task-space residual force `F_residual` scaled by `residual_force_scale`;
  **`tau_residual = J.T @ F`** (cable joint columns); total joint torque clipped by `tau_jnt_limit`; mapped through hybrid transmission.

## Metrics summary (paired)

- In `comparison_metrics.csv`, `reward_zero` / `reward_sac` (and deltas) use **`mean_reward`**: the **mean per-step** reward over the episode (not undiscounted return).

| seed | RMS EE Δ | final EE Δ | reward Δ |
| --- | --- | --- | --- |
| 10000 | -0.00388493 | -0.000571509 | +0.201191 |
| 10001 | +7.94129e-05 | +0.000689232 | -0.0327367 |
| 10002 | -0.000683018 | +0.00546323 | +0.0413383 |
| 10003 | -0.000315588 | +0.00802752 | -0.0265066 |
| 10004 | -0.000878918 | +0.00260528 | -0.0177432 |

- **Across seeds**: mean Δ RMS EE = `-0.00113661` m;
  mean Δ final EE = `0.00324275` m

- **This batch**: mean Δ RMS EE < 0 → average **path-tracking RMS** favors SAC versus zero residual.
- **This batch**: mean Δ final EE > 0 → average **endpoint error** can be **larger** with SAC; **no** conclusion of improved final convergence.

## Interpretation (do not over-claim)
- Reference (larger paired study, 30 episodes): mean EE RMS improved (zero ≈ 0.0206 m → SAC ≈ 0.0191 m), whereas **final EE error was slightly worse** with SAC; **contacts** `ncon = 0`; **22/30** seeds showed RMS improvement. This script’s 5 seeds may differ numerically but illustrates the same qualitative pattern when it appears.
- **If mean Δ RMS EE < 0**: SAC achieves **better RMS path-tracking** versus zero residual (emphasize this when it holds).
- **If mean Δ final EE > 0**: report honestly that **endpoint error can be larger**; do **not** claim final convergence improvement.
- EE path figures: if matplotlib 3D projection is broken (mixed system/pip installs), the script writes **XY/XZ/YZ projections** instead of a single 3D panel; video uses **side-by-side XY** in that case.

## Outputs
- Metrics CSV: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/comparison_metrics.csv`
- Timeseries per seed: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/timeseries_seed_<seed>.csv`
### Plots
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/cable_hysteresis_seed_10000.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/cable_hysteresis_seed_10001.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/cable_hysteresis_seed_10002.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/cable_hysteresis_seed_10003.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/cable_hysteresis_seed_10004.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_err_norm_seed_10000.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_err_norm_seed_10001.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_err_norm_seed_10002.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_err_norm_seed_10003.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_err_norm_seed_10004.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_path_3d_seed_10000.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_path_3d_seed_10001.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_path_3d_seed_10002.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_path_3d_seed_10003.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_path_3d_seed_10004.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_path_overlay_xy_seed_10000.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_path_overlay_xy_seed_10001.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_path_overlay_xy_seed_10002.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_path_overlay_xy_seed_10003.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_path_overlay_xy_seed_10004.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_path_xyz_dashboard_seed_10000.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_path_xyz_dashboard_seed_10001.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_path_xyz_dashboard_seed_10002.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_path_xyz_dashboard_seed_10003.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_path_xyz_dashboard_seed_10004.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_tracking_error_dashboard_seed_10000.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_tracking_error_dashboard_seed_10001.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_tracking_error_dashboard_seed_10002.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_tracking_error_dashboard_seed_10003.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_tracking_error_dashboard_seed_10004.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_x_desired_zero_sac_seed_10000.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_x_desired_zero_sac_seed_10001.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_x_desired_zero_sac_seed_10002.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_x_desired_zero_sac_seed_10003.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_x_desired_zero_sac_seed_10004.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_y_desired_zero_sac_seed_10000.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_y_desired_zero_sac_seed_10001.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_y_desired_zero_sac_seed_10002.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_y_desired_zero_sac_seed_10003.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_y_desired_zero_sac_seed_10004.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_z_desired_zero_sac_seed_10000.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_z_desired_zero_sac_seed_10001.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_z_desired_zero_sac_seed_10002.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_z_desired_zero_sac_seed_10003.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/ee_z_desired_zero_sac_seed_10004.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/histogram_delta_rms_ee_across_seeds.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/metrics_bars_aggregate.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/metrics_bars_seed_10000.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/metrics_bars_seed_10001.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/metrics_bars_seed_10002.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/metrics_bars_seed_10003.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/metrics_bars_seed_10004.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/residual_force_seed_10000.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/residual_force_seed_10001.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/residual_force_seed_10002.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/residual_force_seed_10003.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/residual_force_seed_10004.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/tau_act_ideal_minus_out_seed_10000.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/tau_act_ideal_minus_out_seed_10001.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/tau_act_ideal_minus_out_seed_10002.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/tau_act_ideal_minus_out_seed_10003.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/tau_act_ideal_minus_out_seed_10004.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/tau_residual_seed_10000.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/tau_residual_seed_10001.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/tau_residual_seed_10002.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/tau_residual_seed_10003.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/tau_residual_seed_10004.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/tau_vsd_residual_total_seed_10000.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/tau_vsd_residual_total_seed_10001.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/tau_vsd_residual_total_seed_10002.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/tau_vsd_residual_total_seed_10003.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/plots/tau_vsd_residual_total_seed_10004.png`

### Videos

- *(no new MP4 in this run; files already under `videos/`)*
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/videos/vsd_vs_sac_seed_10000.mp4`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/videos/vsd_vs_sac_seed_10001.mp4`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/videos/vsd_vs_sac_seed_10002.mp4`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/videos/vsd_vs_sac_seed_10003.mp4`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/comparison_vsd_vs_sac/videos/vsd_vs_sac_seed_10004.mp4`

Generated by `scripts/compare_vsd_vs_sac_residual.py`.
