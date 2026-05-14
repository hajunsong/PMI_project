# Paired comparison — full_noise_light

## A. Setup

- **Checkpoint:** `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_medium_train_rs05_30k_s4/checkpoints/best_model_by_smooth_score.zip`
- **VecNormalize:** `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_medium_train_rs05_30k_s4/vecnormalize/vecnormalize.pkl`
- **Seed (selected):** 30000
- **Noise preset:** `full_noise_light` (sensor + delay=1 + LPF 20 Hz + actuator torque/gain noise)
- **Controller:** workspace 5D VSD nominal; task $x,y,z$, roll, pitch; **yaw free**; roll/pitch ref = initial EE orientation.
- **Cable:** transmission on; **collision off.**
- **control_dt:** 0.0100 s · **video fps:** 30.0 · **RGB stride:** 3
- **Camera:** MuJoCo `Renderer` rgb_array (640×480).
- **3D path plot:** skipped (see XY/XZ fallbacks)

## B. Scalar metrics

| Metric | VSD only | VSD + SAC |
| --- | --- | --- |
| RMS EE (true) | 0.005839 | 0.004229 |
| Final EE (true) | 0.003023 | 0.002403 |
| RMS HF | 0.001186 | 0.001055 |
| Smooth score | 0.015926 | 0.013232 |
| Saturation frac. | 0.000000 | 0.000000 |
| Limit frac. | 0.000000 | 0.000000 |
| ncon max | 0 | 0 |

## C. Improvement (SAC vs zero)

| | Δ (SAC−zero) | % improvement vs zero |
| --- | --- | --- |
| RMS EE | -0.001610 | 27.58% |
| Final EE | -0.000620 | 20.50% |
| RMS HF | -0.000131 | 11.01% |

## D. Interpretation

- SAC changes **RMS EE (true)** by 27.58% vs zero residual (negative Δ means SAC lower).
- **Final EE** improvement: 20.50%.
- **HF** metric: 11.01%.
- **Contacts:** max ncon zero=0, SAC=0.
- **Saturation:** SAC − zero = 0.0000; **limits:** 0.0000.

## Outputs

- Plots: `plots/` (includes `ee_error_components_compare.png`: e_x, e_y, e_z, ||e|| vs time)
- Videos: `videos/full_noise_light_side_by_side.mp4`, `full_noise_light_overlay_path.mp4`, `full_noise_light_zoomed_terminal_compare.mp4`
- Time series: `paired_timeseries_zero.csv`, `paired_timeseries_sac.csv`
