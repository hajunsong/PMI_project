# Workspace 5D VSD demo (hybrid cable)

- **모델**: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/models/pmi_hybrid_no_collision.xml` (STL 충돌 비활성, 검증 무충돌 트리; 액추에이터 제거 후 torque-only)
| 항목 | 값 |
|:---|:---|
| Jacobian roll/pitch rows | Task Jacobian rows 4–5 (1-based indexing: rows 5–6) of the analytic 5×4 matrix from ``compute_task_jacobian_mode(..., task_mode='xyz_roll_pitch', mode='mujoco_analytic')`` map joint rates to **roll rate** and **pitch rate** consistent with ``fk_ee_rp`` (SciPy extrinsic xyz Euler); **yaw is omitted** from the task. |
| 케이블 파라미터 | deterministic (delay/friction/elasticity/backlash+hysteresis) — 코드 상단 `CABLE_SCALAR` 참고 |
| 궤적 | 카테시안 Quintic waypoint (0→0.5→1)×duration, 사용자 duration=5.0s |
| 초기 roll/pitch (명목 자세 유지 목표) | roll0=-1.570803 rad, pitch0=-0.000005 rad |
| 제어 모드 | `jt` |
| λ_DLS | 0.12 |
| roll/pitch task weight | 0.065 |
| τ joint limit | 30 N·m |
| 시간 CSV | `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_vsd_5d_cable/workspace_vsd_5d_cable_timeseries.csv` |
| Video | `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_vsd_5d_cable/workspace_vsd_5d_cable.mp4` |

## 메트릭
| key | value |
|:---|---:|
| act_frac | 0.0 |
| act_steps | 0 |
| final_ee_error | 0.0014301680906310024 |
| final_pitch_error | -0.008474516162056744 |
| final_roll_error | -1.142859910974181e-05 |
| jl_frac | 0.0 |
| jl_steps | 0 |
| max_abs_pitch_error | 0.08258911157775195 |
| max_abs_roll_error | 1.1429181728139781e-05 |
| max_cond_J | 19.24425084024638 |
| max_ee_error | 0.011378114278361873 |
| max_tau_act | 1.1244584497726078 |
| max_tau_act_error_q234 | 0.006422878093200561 |
| max_tau_joint | 7.520204607800445 |
| mean_cond_approx | 16.337480473643865 |
| min_sigma | 0.09532112777683568 |
| ncon_max | 0 |
| p2p_error_norm | 0.011345036289765065 |
| rms_ee_acc | 0.04716699246460413 |
| rms_ee_error | 0.006106161460515709 |
| rms_ee_vel | 0.012927630489245613 |
| rms_highfreq | 0.0005785618779857787 |
| rms_pitch_error | 0.039834576607151694 |
| rms_roll_error | 6.7461476680489435e-06 |
| rms_tau_act_error_q234 | 0.00462145137105907 |
| rms_x | 0.003069321209432485 |
| rms_y | 0.0045969700962738775 |
| rms_z | 0.002594675515200115 |
| saturation_fraction | 0.0 |
| saturation_steps | 0 |
| yaw_range | 1.7975763089824586 |

## 생성 플롯
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_vsd_5d_cable/plots/01_xyz_des_act.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_vsd_5d_cable/plots/02_ee_errors.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_vsd_5d_cable/plots/03_roll_pitch_des_act.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_vsd_5d_cable/plots/04_roll_pitch_error.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_vsd_5d_cable/plots/05_yaw_free.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_vsd_5d_cable/plots/06_path_3d.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_vsd_5d_cable/plots/07_q_jnt.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_vsd_5d_cable/plots/08_tau_jnt_cmd.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_vsd_5d_cable/plots/09_tau_act_ideal_vs_out_q234.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_vsd_5d_cable/plots/10_cable_states.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_vsd_5d_cable/plots/11_dls_singular_values.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_vsd_5d_cable/plots/12_flags.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_vsd_5d_cable/plots/13_hf_ee_err.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_vsd_5d_cable/plots/14_summary_dashboard.png`
- `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/workspace_vsd_5d_cable/plots/15_path_vs_ee_pose.png`

## 질문 (요약 답변)
1. **workspace VSD가 xyz 경로를 케이블 포함으로 추종하는가?** RMS EE 오차 약 **0.006106 m**, 최종 **0.001430 m** (게인·토크 한계에 민감).
2. **roll/pitch가 초기값 근처로 유지되는가?** RMS roll/pitch 오차 각각 **6.74615e-06**, **0.0398346 rad** (소프트 가중·4DOF 때문에 큰 편향 가능).
3. **yaw는 자유인가?** 과제 벡터에 미포함; 기록 범위 **1.79758 rad**.
4. **DLS가 과제 과제제약 불안정을 완화하는가?** `max_cond_J≈19.24`, `sigma_min≈0.09532`; 포화 많으면 **λ·w_rp·τ_lim** 조정 필요.
5. **진폭 큰 진동?** RMS HF(|e|)~**0.000578562**, P2P(||e||)~**0.011345**.
6. **포화/한계 위반?** sat_frac≈**0**, joint/actuator 근처 위반 평균 합≈**0**.
7. **충돌 접촉(ncon)?** 시뮬 중 최대 `ncon` = **0**.
8. **기존 IK 관절 VSD와 비교** — 과제표현이 다르므로 숫자 직비는 무의미; 동일 시간·케이블·한계선에서 교차 검증 권장.
9. **다른 모드/튜닝** — `dls`는 `τ_task = K_q·Jp·v_w − D_q·q̇`라 동일 과제 게인에서도 토크가 크게 증폭됨(`K_xyz`·`K_dls_joint` 연동). `jt`는 `Jᵀ` 저게인 토크라 동일 과제 게인이면 추종이 둔해질 수 있음.