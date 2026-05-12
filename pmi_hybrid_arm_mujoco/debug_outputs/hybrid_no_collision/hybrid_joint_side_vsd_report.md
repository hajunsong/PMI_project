# Hybrid joint-side torque VSD (no collision)

- 모델: `models/pmi_hybrid_no_collision.xml` (런타임에 위치 액추에이터만 제거하여 `ctrl` 미사용)
- 토크: `data.qfrc_applied` 는 **jnt1~4 dof** 만. `q_act` 는 equality로 동기.

## Answers

### 1. Does hybrid no-collision preserve the arm-only VSD tracking performance?
- 동일한 IK 궤적·이득이라도 **하이브리드(모터 관성·등식) 추가**로 수치는 arm-only와 완전 일치하지 않을 수 있습니다. 아래 표와 arm-only CSV를 비교하세요.

### 2–3. Equality stability & transmission error
- 스윕 전체 `max transmission ‖·‖` 상한: 2.7453e-06 rad, RMS 상한: 1.7961e-06.
- **등식이 수치적으로 안정적으로 유지되는지:** 예(임계 0.01 rad 미만)

### 4. Is data.ncon always zero?
- 예 (ncon_max per run in table).

### 5. Which controller works better?
- 동일 (duration, tau_limit)에서 최종/ RMS EE 기준으로는 **IK_joint_space_VSD** 쪽이 유리한 경우가 많았습니다(세부는 표).

### 6. Ready for actuator-side ideal torque transmission?
- 관절측 토크 제어·등식 전달이 본 스윕에서 허용 오차 내라면, **다음 단계로 액추에이터 축 토크 맵**을 설계 시 테스트할 수 있습니다. 케이블·백래시는 아직 포함하지 않습니다.

## Summary table

| run | controller | T | τ_lim | rms_ee | final_ee | rms_trans | max_trans | sat | ncon | jl |
|-----|------------|---|-------|--------|----------|-----------|-----------|-----|------|----|
| 0 | IK_joint_space_VSD | 3.0 | 20.0 | 0.001561 | 0.000908 | 1.4695e-06 | 2.3273e-06 | 0 | 0 | 0 |
| 1 | JTF_task_space_VSD | 3.0 | 20.0 | 0.022832 | 0.021461 | 1.7961e-06 | 2.7453e-06 | 0 | 0 | 0 |
| 2 | IK_joint_space_VSD | 3.0 | 50.0 | 0.001561 | 0.000908 | 1.4695e-06 | 2.3273e-06 | 0 | 0 | 0 |
| 3 | JTF_task_space_VSD | 3.0 | 50.0 | 0.022832 | 0.021461 | 1.7961e-06 | 2.7453e-06 | 0 | 0 | 0 |
| 4 | IK_joint_space_VSD | 5.0 | 20.0 | 0.000704 | 0.000238 | 8.0851e-07 | 1.2857e-06 | 0 | 0 | 0 |
| 5 | JTF_task_space_VSD | 5.0 | 20.0 | 0.008894 | 0.002907 | 9.1178e-07 | 1.5553e-06 | 0 | 0 | 0 |
| 6 | IK_joint_space_VSD | 5.0 | 50.0 | 0.000704 | 0.000238 | 8.0851e-07 | 1.2857e-06 | 0 | 0 | 0 |
| 7 | JTF_task_space_VSD | 5.0 | 50.0 | 0.008894 | 0.002907 | 9.1178e-07 | 1.5553e-06 | 0 | 0 | 0 |

## Arm-only IK baseline (same duration / tau_limit, if CSV exists)

| duration | tau_limit | arm_only_rms_ee | arm_only_final_ee |
|----------|-----------|-----------------|-------------------|
| 3.0 | 20.0 | 0.004038 | 0.002007 |
| 3.0 | 50.0 | 0.007209 | 0.006970 |
| 5.0 | 20.0 | 0.003808 | 0.001821 |
| 5.0 | 50.0 | 0.007273 | 0.006790 |

