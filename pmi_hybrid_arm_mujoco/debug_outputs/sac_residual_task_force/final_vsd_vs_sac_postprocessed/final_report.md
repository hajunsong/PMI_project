# Final comparison: VSD-only vs VSD + SAC (best EE RMS) + post-processing

## 1. Model and controller
- **Plant / controller**: `configs/rl_sac.yaml` (VSD joint torque baseline + hybrid cable transmission).
- SAC는 작업공간 잔차 힘을 출력하고, 본 비교에서만 저역통과·속도제한·종료 페이드를 적용합니다 (학습 시 정의한 액션 스케일은 그대로).

## 2. SAC checkpoint
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_100k_s2/checkpoints/best_model_by_ee_rms.zip`

## 3. VecNormalize
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_100k_s2/vecnormalize/vecnormalize.pkl`

## 4. Cable profile
- `medium_train`

## 5. Post-processing parameters
- `residual_gain`: 1.0
- `residual_filter_tau`: 0.03
- `final_fade_duration`: 0.5 s
- `max_delta_force_per_step`: 0.2

## 6. Paired rollout summary

- Episodes averaged: **`5`** (seed band `10000`–`10004`).
- **Mean Δ RMS EE** (SAC+pp − VSD): **`-0.0003876664`** — VSD+SAC+후처리가 평균 EE RMS 추적 측면에서 유리합니다.
- **Mean Δ final EE**: **`-0.0003249602`** — 종단 EE 오차도 평균적으로 개선됩니다.
- **Contacts**: `ncon_max` (VSD / SAC+pp) = `0` / `0`.

**주의 (과잉 주장 방지):**
- 모든 고주파·진동 성분이 제거되었다고 단정할 수 없습니다.
평균 고주파 EE RMS 델타는 `7.415365e-05` 입니다. 일부 시드에서만 악화되면 위 시드 목록을 참고하세요.
- 원래 best RMS 모델 단독 평가에서 관찰된 **종단 오차 악화** 완화에는 **후처리(특히 종료 페이드)** 가 실질적으로 필요했습니다.

## 7. End-effector orientation logging & visuals

- 각 스텝의 `desired_roll_rad`/`desired_pitch_rad`/`desired_yaw_rad`(NaN) 및 FK 기반 `ee_des_*_fk`, 실측 `ee_roll_rad`/`ee_pitch_rad`/`ee_yaw_rad`, `roll_error_rad`/`pitch_error_rad`(wrap), `ee_omega_world_*`가 detail 시드 `10000`…`10004` CSV에 포함됩니다 (`timeseries/seed_*_*.csv`).
- **Mean RMS roll/pitch error (VSD / SAC+pp / Δ)** — roll: `3.989892e-06` / `3.974081e-06` / `-1.581145e-08`, pitch: `0.1559606` / `0.1606391` / `0.004678539`.
- **Mean Pearson corr** — corr(‖e‖,|roll_err|): VSD `-0.2457156`, SAC `-0.2388864`; corr(‖e‖,|pitch_err|): VSD `0.2281086`, SAC `0.4609646`; corr(‖e_hf‖,|roll_err|): VSD `-0.2064044`, SAC `-0.1980344`; corr(‖e_hf‖,|pitch_err|): VSD `0.4273945`, SAC `0.4023945`.
- Orientation figures: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation`.

- `paired_metrics_30seeds.csv`: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/metrics/paired_metrics_30seeds.csv`
- `aggregate_metrics.md`: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/metrics/aggregate_metrics.md`

## 8. Detailed seeds (table)

| seed | Δ RMS EE | Δ final EE | Δ HF | Δ ee_vel | ncon sac |
| --- | --- | --- | --- | --- | --- |
| 10000 | -0.00240629 | 0.0013311 | 1.0949425584232467e-05 | 0.0005557081493161586 | 0 |
| 10001 | 0.000728429 | -0.00360033 | 0.00017375519930061187 | 0.0009457282374349951 | 0 |
| 10002 | -0.000466699 | -0.000274687 | -5.000778935363055e-05 | -0.0015328437754501545 | 0 |
| 10003 | 0.000871668 | 0.00192383 | 0.00023810546385121846 | 0.0007572559442071691 | 0 |
| 10004 | -0.000665439 | -0.00100471 | -2.034067175860077e-06 | -0.00010128157936939441 | 0 |

## 9. Plot outputs

- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/aggregate_bar_metrics.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/aggregate_bar_sat_limit.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/delta_final_histogram.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/delta_rms_histogram.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/delta_smooth_metrics.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10000_orientation_dashboard.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10000_pitch_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10000_roll_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10000_roll_pitch_error.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10000_yaw_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10001_orientation_dashboard.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10001_pitch_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10001_roll_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10001_roll_pitch_error.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10001_yaw_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10002_orientation_dashboard.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10002_pitch_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10002_roll_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10002_roll_pitch_error.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10002_yaw_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10003_orientation_dashboard.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10003_pitch_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10003_roll_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10003_roll_pitch_error.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10003_yaw_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10004_orientation_dashboard.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10004_pitch_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10004_roll_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10004_roll_pitch_error.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation/seed_10004_yaw_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10000_3d_path.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10000_ee_desired_vs_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10000_ee_error_dashboard.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10000_residual_force.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10000_residual_gate.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10000_smoothness.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10000_tau_residual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10000_torque_total.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10001_3d_path.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10001_ee_desired_vs_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10001_ee_error_dashboard.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10001_residual_force.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10001_residual_gate.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10001_smoothness.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10001_tau_residual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10001_torque_total.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10002_3d_path.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10002_ee_desired_vs_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10002_ee_error_dashboard.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10002_residual_force.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10002_residual_gate.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10002_smoothness.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10002_tau_residual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10002_torque_total.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10003_3d_path.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10003_ee_desired_vs_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10003_ee_error_dashboard.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10003_residual_force.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10003_residual_gate.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10003_smoothness.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10003_tau_residual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10003_torque_total.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10004_3d_path.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10004_ee_desired_vs_actual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10004_ee_error_dashboard.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10004_residual_force.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10004_residual_gate.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10004_smoothness.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10004_tau_residual.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/seed_10004_torque_total.png`

## 10. Video outputs

*(none — use --skip-video 또는 FFmpeg 확인)*

## Recommendation

- 동영상·시각화·추가 보고에는 **`sac_tf_tracking_reward_rs2_100k_s2` / `best_model_by_ee_rms.zip`** 과 위 후처리 설정을 사용하세요.
- **`sac_tf_smooth_final_rs1_30k_s5`** 등 추적을 악화시킨 런은 최종 비교에 사용하지 마세요.

## 11. Interpretation (control modes & oscillation)

1. **Was the nominal stack xyz-only?** Joint references come from IK with ``task_feas_mode="xyz"`` (position rows only): roll/pitch targets are **not** in the IK residual, so **`q_des` does not impose** roll=-π/2 / pitch=0 as a soft/hard IK objective. Joint-space VSD tracks that `q_des` with PD gains.
2. **Roll/pitch via `q_des`?** **Only implicitly** via whatever orientation the XYZ IK solution induces; FK targets in logs (`ee_des_roll_rad_fk`) show the orientation consistent with **`q_des`**, while `roll_error_rad` compares actual EE orientation to fixed roll=-π/2 and pitch=0.
3. **Correlation with EE oscillation.** See §Orientation in `aggregate_metrics.md` for corr(‖e_xyz‖,|roll_err|) and corr(‖e_hf‖,|pitch_err|) means over seeds (not causal; shared dynamics / coupling).
4. **SAC residual.** `action_dim=3`: **Fx, Fy, Fz only** mapped through the **XYZ position Jacobian rows** (`joint_torque_from_task_force`): no learned roll/pitch moment channel.
5. **Soft orientation VSD.** `orientation_soft_mode=none` in default config matches **baseline xyz-only nominal torque** aside from IK path; optional soft roll/pitch uses extra Jacobian rows (`xyz_roll_pitch`) — run `orientation_soft_vsd_ablation.py` (no training) to compare visually.
6. **Future SAC action.** If orientation errors correlate with HF EE error but cannot be trimmed by XYZ force alone on a 4-DoF arm, consider extending Jacobian / action toward a **sparse wrench** rather than blindly adding 5 full hard constraints.

- Control stack review: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/control_mode_inspection.md` (generated with this analysis).

---
- Output root: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed`