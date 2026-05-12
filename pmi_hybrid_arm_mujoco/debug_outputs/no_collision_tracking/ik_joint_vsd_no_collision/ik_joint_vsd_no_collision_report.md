# IK Joint VSD — No-Collision Arm-Only

- Model: `models/pmi_arm_only_no_collision.xml`
- IK joint targets (wp0→wp1→wp2) jnt1: [0.67474, 1.56280, 2.45728] (rad)
- |Δjnt1| sum across segments ≈ 1.78254 rad (large-motion indicator)

## Report answers

### 1. Does the no-collision model solve the IK joint trajectory?
- IK는 기구학만 사용하므로 무충돌 MJCF에서도 동일하게 관절 목표 해가 구해집니다. 
- 웨이포인트별 IK 관절값은 위 jnt1 나열과 CSV 초기 `q_des_*` 구간과 일치합니다.

### 2. What is the best run?
- **포화 스텝 최소 우선(가능한 success 제약 만족):** run_id=0, duration=1.0, tau_limit=20.0, final_ee=0.009621, rms_ee=0.008422, final_q=0.027931, rms_q=0.026397, sat_steps=232, jl_steps=0, ncon_max=0
- **느린 궤적 기준 (duration≥3 s, RMS EE<0.05):** run_id=7, duration=3.0, tau_limit=50.0, final_ee=0.006970, rms_ee=0.007209, final_q=0.036644, rms_q=0.065449, sat_steps=670, jl_steps=0, ncon_max=0

### 3. What are RMS and final EE errors?
- Per-run values are in the summary table below (all sweeps). Best run quoted above.

### 4. What are RMS and final joint errors?
- See `rms_joint_err` / `final_joint_err` in the summary table.

### 5. Is `data.ncon` always zero?
- All runs: **yes**.

### 6. Is `qfrc_constraint` near zero?
- Mesh가 비접촉(contype/conaffinity=0)이면 등식/접촉 제약이 없어 관절 DOF의 `qfrc_constraint`는 0에 가깝습니다. 
- 시계열은 CSV의 `qfrc_constraint_jnt*` 열을 확인하세요.

### 7. Does jnt1 now follow the large required motion?
- Quintic 경로상 jnt1은 약 1.783 rad 규모 웨이포인트 간 변화를 포함합니다.
- 실제 추종은 `q_act_jnt1` vs `q_des_jnt1` 및 joint error 열로 확인할 수 있습니다.

### 8. Baseline duration and `tau_limit`
- 성공 기준: final EE < 0.03 m, 느린 duration에서 RMS EE < 0.05 m, 접촉 없음, 관절 한계 위반 없음, 토크 포화 스텝 적음.
- **전체 스윕 중 포화 최소 추천:** duration=1.0, tau_limit=20.0.
- **느린 궤적 RMS 기준 베이스라인:** duration=3.0, tau_limit=50.0.
- Slow-duration RMS<0.05 만족 후보: 6 / 6 (duration≥3s in sweep).

## Summary (all sweep combinations)

| run_id | duration | tau_limit | rms_ee | final_ee | rms_q | final_q | sat | jl | ncon_max |
|--------|----------|-----------|--------|----------|-------|---------|-----|----|----------|
| 0 | 1.0 | 20.0 | 0.008422 | 0.009621 | 0.026397 | 0.027931 | 232 | 0 | 0 |
| 1 | 1.0 | 50.0 | 0.010745 | 0.016255 | 0.040974 | 0.037738 | 268 | 0 | 0 |
| 2 | 1.0 | 100.0 | 0.021911 | 0.036544 | 0.110543 | 0.105523 | 229 | 0 | 0 |
| 3 | 2.0 | 20.0 | 0.004646 | 0.002878 | 0.026310 | 0.015714 | 498 | 0 | 0 |
| 4 | 2.0 | 50.0 | 0.007390 | 0.007560 | 0.063822 | 0.038459 | 423 | 0 | 0 |
| 5 | 2.0 | 100.0 | 0.025554 | 0.034285 | 0.131052 | 0.096775 | 418 | 0 | 0 |
| 6 | 3.0 | 20.0 | 0.004038 | 0.002007 | 0.027109 | 0.014430 | 676 | 0 | 0 |
| 7 | 3.0 | 50.0 | 0.007209 | 0.006970 | 0.065449 | 0.036644 | 670 | 0 | 0 |
| 8 | 3.0 | 100.0 | 0.027146 | 0.033886 | 0.135696 | 0.095141 | 664 | 0 | 0 |
| 9 | 5.0 | 20.0 | 0.003808 | 0.001821 | 0.027340 | 0.013868 | 1176 | 0 | 0 |
| 10 | 5.0 | 50.0 | 0.007273 | 0.006790 | 0.066657 | 0.035987 | 1169 | 0 | 0 |
| 11 | 5.0 | 100.0 | 0.028456 | 0.033723 | 0.138975 | 0.094491 | 1164 | 0 | 0 |

