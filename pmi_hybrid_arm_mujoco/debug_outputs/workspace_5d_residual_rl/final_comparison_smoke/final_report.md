# Final report: workspace 5D VSD + SAC residual (medium_train)

## 1. Model

- `models/pmi_hybrid_no_collision.xml` — **collision disabled** in this pipeline.

## 2. Controller

- Nominal **workspace 5D VSD** (not replaced).
- Task: **x, y, z, roll, pitch**; **yaw free**.
- Desired roll/pitch track initial attitude at episode start.

## 3. Transmission

- q1 belt/gear; q2–q4 cable (`HybridTransmission`).
- Torque applied **actuator-side** via `qfrc_applied` (not `data.ctrl`).

## 4. SAC residual

- **5D task-space wrench** on top of VSD: action = [Fx, Fy, Fz, Mroll, Mpitch].
- VSD nominal torques are **not** removed; residual is added.

## 5. Curriculum

`deterministic` → `mild` → `medium_v2` → `medium_train`.

## 6. Final medium_train checkpoint (user-reported reference)

- RMS EE reduction ≈ **44.54 %**
- Final EE reduction ≈ **47.35 %**
- RMS HF reduction ≈ **14.91 %**
- Saturation / limit: **0 → 0** in reference eval; **ncon max 0**.

## 7. This run aggregate (mean over 1 paired episodes)

- Percent RMS improvement: 47.9982 %
- Percent final EE improvement: 59.8038 %
- Percent RMS HF improvement: 11.7968 %

## 8. Outputs

- Metrics: `debug_outputs/workspace_5d_residual_rl/final_comparison_smoke/metrics/paired_metrics_50episodes.csv`, `debug_outputs/workspace_5d_residual_rl/final_comparison_smoke/metrics/aggregate_metrics.md`
- Plots: `debug_outputs/workspace_5d_residual_rl/final_comparison_smoke/plots` (including `seed_10000_*.png` for detail seed 10000)
- Videos: `debug_outputs/workspace_5d_residual_rl/final_comparison_smoke/videos` (`seed_*_vsd_only.mp4`, `_sac_residual.mp4`, `_side_by_side.mp4`)

## 9. Limitations

- Simulation-only; real hardware may differ.
- **Stress** randomization profile is **evaluation-only** (no training on stress in this project phase).
- Cable parameters follow configured **randomization_profiles** in `cable_layer.yaml`.
- Yaw is intentionally **uncontrolled**.

## 10. Next optional step

- Run **stress evaluation only**, e.g. `evaluate_workspace_5d_residual_sac.py --curriculum-stage stress ...`.
- **Do not** train SAC on the stress profile unless explicitly planned.
