# 5D task-space wrench + DLS ablation report

이 리포트는 결정론적 롤아웃(학습 없음) 기준입니다.

## 명목 IK / 과제 차원 요약 답변

1. **현재 명목 IK는 무엇을 제약하는가?** 설정 및 코드상 `solve_ik_task_mode` 호출 시 `task_feas_mode="xyz"` 로 IK는 **카테시안 위치(xyz)** 위주로 풀리며 자세(roll/pitch)는 IK 하드 과제 변수로는 **같이 맞추지 않으며**, IK 시드·보간으로 생성된 `q_des`와 **명목 소프트 자세**(controller `orientation_soft_mode` 등이 꺼져 있으면)에 따라 EE의 roll/pitch는 경로를 따라 오차가 존재할 수 있습니다.

2. **roll/pitch 오차와 EE 경로 진동 상관관계(에피소드 평균):** 
   Pearson (시드별 VSD 단독 궤적에서 샘플한 시점들): |e|↔|roll|≈-0.5494, |e|↔|pitch|≈0.3592, HF↔|roll|≈-0.2394, HF↔|pitch|≈0.6739.
   실제 결과는 각 시드별 궤적에 따라 바뀌며, 아래 표의 집계를 우선 참고합니다.

## 주요 결과 (scenario 평균, VSD 단독 대비)

- **평균 시드 기준 selection_score 최소:** `C_tw0.2_ld0.03_kp0.05_cable_joints_only_wren` (family **C**), score=-0.000013. λ·joint scope는 `config_id` 문자열(`_ld…`, `cable_joints_only` / `all_joints`)에서 읽습니다.

| config_id | family | score↓ | Δrms EE | Δfinal | ΔHF | Δp2p | Δroll | Δpitch | Δsat |
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| `C_tw0.2_ld0.03_kp0.05_cable_joints_only_wren` | C | -0.0000 | -0.000013 | -0.000132 | 0.000001 | -0.000022 | 0.000000 | -0.000032 | -0.002000 |
| `C_tw0.2_ld0.1_kp0.05_cable_joints_only_wren` | C | -0.0000 | -0.000013 | -0.000132 | 0.000001 | -0.000022 | 0.000000 | -0.000032 | -0.002000 |
| `D_tw0.2_ld0.03_kp0.05_cable_joints_only_dls_` | D | -0.0000 | -0.000011 | -0.000110 | 0.000001 | -0.000019 | 0.000000 | -0.000027 | -0.002000 |
| `D_tw0.2_ld0.03_kp0.05_cable_joints_only_dls_` | D | -0.0000 | -0.000011 | -0.000110 | 0.000001 | -0.000019 | 0.000000 | -0.000027 | -0.002000 |
| `C_tw0.1_ld0.03_kp0.05_cable_joints_only_wren` | C | -0.0000 | -0.000006 | -0.000056 | 0.000001 | -0.000011 | 0.000000 | -0.000015 | -0.002000 |
| `C_tw0.1_ld0.1_kp0.05_cable_joints_only_wren` | C | -0.0000 | -0.000006 | -0.000056 | 0.000001 | -0.000011 | 0.000000 | -0.000015 | -0.002000 |
| `D_tw0.2_ld0.1_kp0.05_cable_joints_only_dls_` | D | -0.0000 | -0.000004 | -0.000038 | 0.000000 | -0.000008 | 0.000000 | -0.000010 | -0.002000 |
| `D_tw0.2_ld0.1_kp0.05_cable_joints_only_dls_` | D | -0.0000 | -0.000004 | -0.000038 | 0.000000 | -0.000008 | 0.000000 | -0.000010 | -0.002000 |
| `D_tw0.1_ld0.03_kp0.05_cable_joints_only_dls_` | D | -0.0000 | -0.000004 | -0.000035 | 0.000000 | -0.000007 | 0.000000 | -0.000009 | -0.002000 |
| `D_tw0.1_ld0.03_kp0.05_cable_joints_only_dls_` | D | -0.0000 | -0.000004 | -0.000035 | 0.000000 | -0.000007 | 0.000000 | -0.000009 | -0.002000 |
| `D_tw0.1_ld0.1_kp0.05_cable_joints_only_dls_` | D | -0.0000 | -0.000001 | -0.000008 | 0.000000 | -0.000002 | 0.000000 | -0.000002 | 0.000000 |
| `D_tw0.1_ld0.1_kp0.05_cable_joints_only_dls_` | D | -0.0000 | -0.000001 | -0.000008 | 0.000000 | -0.000002 | 0.000000 | -0.000002 | 0.000000 |

## VSD-only baseline 평균 지표 일부

| key | mean |
|:---|---:|
| final_ee_error | 0.040976 |
| limit_violation_fraction | 0.032000 |
| max_abs_pitch_error | 0.264285 |
| max_abs_roll_error | 0.000007 |
| max_ee_error | 0.057164 |
| p2p_error_norm | 0.056733 |
| rms_ee_error | 0.036078 |
| rms_ee_error_acceleration | 51.712132 |
| rms_ee_error_highfreq | 0.010893 |
| rms_ee_error_velocity | 0.761486 |
| rms_pitch_error | 0.124878 |
| rms_residual_tau_rate | 0.000000 |
| rms_residual_wrench_norm | 0.000000 |
| rms_residual_wrench_rate | 0.000000 |
| rms_roll_error | 0.000004 |
| rms_tau_total_rate | 3361.253523 |
| saturation_fraction | 0.846000 |

## 질문 3–11 (정성 + 표 기준)

3–4. 소프트 roll/pitch 잔차와 DLS는 **설정별로** EE 고주파 에러 및 p2p를 줄일 수 있습니다. 최적은 위 표에서 **score 및 ΔHF/Δp2p** 확인.
5. **최적 lambda_dls**는 그리드의 **상위 score 행**(보통 λ≈0.03–0.1 구간에서 안정되는 경우 많음 — 이번 CSV에서 가장 좋은 `ld*` 확인).
6. **cable_joints_only vs all_joints**는 scope 비교 행을 필터링해 Δrms/Δsat을 비교하면 됩니다.
7–9. 같은 방식으로 `delta_rms_ee`, `delta_final_ee`, `delta_highfreq`, `delta_p2p` 열 확인.
10. `delta_saturation`, `delta_limit` 열로 한계 접촉/위반 증가 여부를 평가.
11. **향후 SAC 5D 렌치 확장 여부**: 이번 휴리스틱 + DLS가 tracking/진동 개선 중 하나라면 **학습 차원 확장**(5D 또는 3D+F + 휴리스틱)을 검토할 가치가 있습니다. 학습 코드는 사용자 요청에 따라 본 과제에서는 추가하지 않았습니다.
