# Workspace 5D VSD vs SAC residual (paired)

- Model: `debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_det_rs05_30k_s0/checkpoints/best_model_by_smooth_score.zip`
- Config: `configs/rl_workspace_5d_sac.yaml`
- Episodes: 10, seed_start=0

## Mean metrics

| Metric | Zero | SAC | Δ (SAC−zero) |
| --- | --- | --- | --- |
| RMS EE | 0.006140 | 0.005596 | -0.000544 |
| Final EE | 0.001459 | 0.001442 | -0.000017 |
| RMS HF | 0.000584 | 0.000550 | -0.000034 |
| Smooth | 0.014204 | 0.013679 | -0.000525 |

CSV: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_5d_residual_rl/evaluation/evaluation_summary.csv`
