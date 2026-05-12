# Actuator-side ideal torque transmission (hybrid no-collision)

- 관절 VSD: `tau_jnt = tau_bias + Kq e + Dq edot`, **관절 토크 먼저** `±tau_jnt_limit` 클립.
- 액추에이터 누력: `tau_act = ratio ⊙ tau_jnt_clipped` (전력 일관: τ_joint·q̇_joint ≈ τ_act·q̇_act).
- `data.qfrc_applied` 는 **q_act dof 만** (jnt dof 0). 위치 액추에이터는 런타임 제거.

## Answers

### 1. Does actuator-side ideal torque reproduce joint-side tracking?
- 동일 (duration, τ_lim)에서 EE·관절 오차를 아래 표로 비교. 수치가 근접하면 **동일 제어 목표가 액추에이터 경유로도 재현**됩니다.

### 2. Is τ_act = ratio ⊙ τ_jnt correct in this equality model?
- 등식 `q_jnt = ratio ⊙ q_act` 에서 순간 전력 일치를 쓰면 τ_act = ratio ⊙ τ_jnt 가 됩니다. 전달 잔차가 통계적으로 0에 가깝지 않으면 매핑·제약 해석을 재점검합니다.

### 3. Is transmission error still below 1e-4 rad?
- 스윕 전체 `max‖q_jnt−ratio⊙q_act‖` 최댓값: **5.8929e-05** rad → 예

### 4. Is data.ncon always zero?
- 예

### 5. Are equality constraint forces reasonable?
- 액추에이터 런별 `max|qfrc_constraint|` (jnt / act) 상한: 7.32 / 1.1 (구속·중력·관성에 따른 정상 범위인지 로그로 확인).

### 6. Does actuator-side torque require different gain scaling?
- 본 스크립트는 **같은 Kq,Dq** 를 유지. EE/관절 오차가 관절측 대비 유의미하게 악화되면 이득·토크 한계·클립 순서를 조정할 수 있습니다.

### 7. Ready for ideal q1 gear + q2~q4 cable layer?
- 이상 액추에이터 토크 전달이 위 표에서 검증되면, **q1 벨트/기어 및 q2~q4 케이블 층**을 별도 제약·손실 모델로 얹을 준비가 됩니다. 케이블 탄성/감쇠/백래시는 **아직 미포함**.

## Comparison table (joint-side vs actuator-side)

| mode | duration | τ_lim | rms_ee | final_ee | rms_q_err | final_q_err | max_trans | sat | ncon | jl | al | max|qfc| jnt/act |
|------|----------|-------|--------|----------|-----------|-------------|-----------|-----|------|----|----|----------------------|
| joint_side_IK_VSD | 3.0 | 20.0 | 0.001561 | 0.000908 | 0.004635 | 0.002767 | 2.3273e-06 | 0 | 0 | 0 | 0 | nan/nan |
| actuator_side_ideal | 3.0 | 20.0 | 0.001561 | 0.000908 | 0.004635 | 0.002767 | 5.8929e-05 | 0 | 0 | 0 | 0 | 7.32/1.1 |
| joint_side_IK_VSD | 3.0 | 50.0 | 0.001561 | 0.000908 | 0.004635 | 0.002767 | 2.3273e-06 | 0 | 0 | 0 | 0 | nan/nan |
| actuator_side_ideal | 3.0 | 50.0 | 0.001561 | 0.000908 | 0.004635 | 0.002767 | 5.8929e-05 | 0 | 0 | 0 | 0 | 7.32/1.1 |
| joint_side_IK_VSD | 5.0 | 20.0 | 0.000704 | 0.000238 | 0.002130 | 0.000726 | 1.2857e-06 | 0 | 0 | 0 | 0 | nan/nan |
| actuator_side_ideal | 5.0 | 20.0 | 0.000704 | 0.000238 | 0.002130 | 0.000726 | 5.8286e-05 | 0 | 0 | 0 | 0 | 7.27/1.09 |
| joint_side_IK_VSD | 5.0 | 50.0 | 0.000704 | 0.000238 | 0.002130 | 0.000726 | 1.2857e-06 | 0 | 0 | 0 | 0 | nan/nan |
| actuator_side_ideal | 5.0 | 50.0 | 0.000704 | 0.000238 | 0.002130 | 0.000726 | 5.8286e-05 | 0 | 0 | 0 | 0 | 7.27/1.09 |

