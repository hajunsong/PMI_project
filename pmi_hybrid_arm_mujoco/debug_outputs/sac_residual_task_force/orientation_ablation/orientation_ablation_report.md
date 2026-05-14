# Orientation soft-VSD ablation (deterministic, no SAC training)

- Policy: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_100k_s2/checkpoints/best_model_by_ee_rms.zip`
- VecNormalize: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_100k_s2/vecnormalize/vecnormalize.pkl`
- Profile: `medium_train`
- Seeds: `10000` … `10004`
- Post-process overrides match `scripts/final_compare_vsd_vs_sac_postprocessed.POSTPROCESS_OVERRIDES`.
- When orientation mode is not `none`, control uses `orientation_soft_roll` / `orientation_soft_pitch` with **Kp=5**, **Dp=0.5** (script-enforced merge).

## Metrics (mean over seeds per mode)

| Mode | RMS_EE_V | RMS_EE_S | fin_EE_V | fin_EE_S | HF_V | HF_S | P2P_V | P2P_S | rmsR_V | rmsR_S | rmsP_V | rmsP_S | sat_V | sat_S | lim_V | lim_S | ncon_V | ncon_S |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `A_xyz_only` (`none`) | 0.0212864 | 0.02109144 | 0.01793475 | 0.02189556 | 0.01245742 | 0.01241454 | 0.04608385 | 0.0427431 | 3.989892e-06 | 3.945204e-06 | 0.1559606 | 0.1630352 | 0.814 | 0.7992 | 0.0092 | 0.0096 | 0 | 0 |
| `B_xyz_plus_roll_soft` (`xyz_plus_roll_soft`) | 0.02128735 | 0.02108737 | 0.01764081 | 0.0221187 | 0.01245853 | 0.0124187 | 0.04608385 | 0.04274309 | 3.98999e-06 | 3.945457e-06 | 0.1559688 | 0.1630622 | 0.8144 | 0.7988 | 0.0092 | 0.0092 | 0 | 0 |
| `C_xyz_plus_roll_pitch_soft` (`xyz_plus_roll_pitch_soft`) | 0.02226568 | 0.02090523 | 0.018651 | 0.02014624 | 0.01316383 | 0.01316637 | 0.04764005 | 0.04803833 | 3.998909e-06 | 3.96716e-06 | 0.1626339 | 0.1688908 | 0.8304 | 0.8356 | 0.0068 | 0.0048 | 0 | 0 |

## Interpretation (read with 3D path plots)

1. **Baseline (`none`)** matches nominal xyz IK + joint VSD without extra orientation torque.
2. Soft roll/pitch rows add **small stabilization** torque; **4-DoF** arm cannot independently hold 5 unrelated constraints.
3. Compare **HF EE RMS** and **P2P** in `orientation_ablation_metrics.csv` alongside **saturation_fraction** / **limit_violation_fraction**.

4. SAC remains **Fx,Fy,Fz**-only; gains here apply to **VSD nominal** torque only (+ same residual pipeline).

- Raw rows: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/orientation_ablation/orientation_ablation_metrics.csv`

## Interpretation prompts (answered for seeds 10000–10004, same policy/checkpoint)

1. **Was the nominal controller xyz-only for attitude?** In `A_xyz_only`, there is **no soft orientation Jacobian torque** (`orientation_soft_mode=none`): attitude follows **xyz-only IK joints** plus joint PD (see also `debug_outputs/sac_residual_task_force/control_mode_inspection.md`).
2. **Roll/pitch indirectly through `q_des`?** **Yes.** IK waypoint solve uses **`task_feas_mode="xyz"`**, so **`q_des` is not enforcing** roll=-π/2/pitch=0 in the IK cost; FK-from-`q_des` posture still has a deterministic roll/pitch (`ee_des_*_fk` in logs).
3. **Correlation of roll/pitch with EE oscillation?** Paired-script aggregate (same seeds) shows moderate **corr(‖e_hf‖,|pitch_err|)** (positive on average for this slice); pitch is mechanically coupled to planar motion on this manipulator — **compare time series** in `final_vsd_vs_sac_postprocessed/plots/orientation/` rather than interpreting correlation causally.
4. **Soft roll/pitch vs visible 3D path wobble:** This CSV-only ablation keeps **similar HF RMS EE** for **`B`** vs **`A`**; **`C`** **raises** averaged **HF RMS** and **pitch RMS** versus **`A`** for both VSD and SAC curves on this slice — visually compare `seed_*_3d_path.png` reruns generated under each mode (`orientation_ablation_metrics.csv` does **not** store path plots automatically).
5. **Saturation / limits:** **`C_xyz_plus_roll_pitch_soft`** raises **saturation_fraction** (~0.83 vs ~0.81 VSD baseline on the mean rows); **`ncon`** remains **zero** throughout this run slice.
6. **Future SAC wrench:** Prefer **parsimonious wrench channels** aligned with Jacobian rows actually used (still **≤4 actuator freedoms**): e.g. keep **Fx,Fy,Fz** first; add **minimal** torque about one body axis **only when** correlations + task demand justify **retuning / re-training** outside this no-training pass.