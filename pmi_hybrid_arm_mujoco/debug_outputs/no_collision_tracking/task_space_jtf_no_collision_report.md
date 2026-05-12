# Pure task-space JTF VSD (xyz) — No-collision arm-only

- Model: `models/pmi_arm_only_no_collision.xml`
- Control: `F = Kx*x_err + Dx*xdot_err`, `tau = bias + J^T F`, clip, `qfrc_applied`.
- Trajectory: Cartesian quintic through 3 waypoints (t_norm 0, 0.5, 1 → 0, T/2, T).

## Answers

### 1. Does pure task-space JTF VSD work after collision is removed?
- 무충돌 모델에서는 접촉 없이 폐루프 작업공간 토크가 적용되므로 알고리즘 자체는 동작합니다. 
- 성능은 이득·한계·포화에 따라 달라지며, 아래 수치(특히 최적 런)로 판단합니다.

### 2. What is the best run?
- **JTF (feasible 우선, 포화·오차·osc 비율 순):** run_id=87, duration=5.0, tau_limit=20.0, Kx=80.0, Dx=5.0, rms_ee=0.007058, final_ee=0.002554, sat=0, jl=0, ncon_max=0, peak/RMS=1.78

### 3. What are RMS and final EE errors?
- 스윕별 값은 CSV 집계 또는 요약 테이블(아래) 참고.

### 4. How does it compare to IK joint-space VSD baseline?
- IK baseline best row: run_id=0, duration=1.0, tau_limit=20.0, rms_ee=0.008422, final_ee=0.009621, sat=232, jl=0, ncon_max=0
- JTF best row: run_id=87, duration=5.0, tau_limit=20.0, Kx=80.0, Dx=5.0, rms_ee=0.007058, final_ee=0.002554, sat=0, jl=0, ncon_max=0, peak/RMS=1.78

### 5. Is `data.ncon` always zero?
- JTF 스윕: **yes**.

### 6. Is `qfrc_constraint` near zero?
- 메시 비접촉 MJCF이므로 관절 DOF 제약력은 0 근처입니다 (`qfrc_constraint_jnt*` CSV).

### 7. Does increasing Kx help or create oscillation?
- 동일 조건에서 peak/RMS 비율(클수록 진동·오버슈트 경향): Kx=30→peak/RMS≈1.73; Kx=50→peak/RMS≈1.65; Kx=80→peak/RMS≈1.64
- 일반적으로 Kx 상승은 강한 복원력과 함께 `F_xyz`·토크 변동을 키워 포화/진동을 유발할 수 있습니다.

### 8. Main controller recommendation
- 본 스윕에서 JTF 최적 런은 **포화 없이** 최종 EE를 수 mm 수준으로 낮출 수 있었고, IK 대표 런(짧은 duration·포화 많음)과는 지표가 다릅니다.
- 다만 JTF는 **Kx/Dx·경로 시간**에 민감하고, 여기서는 roll/pitch를 제어하지 않습니다.
- **작업공간 xyz만** 추적하고 이득 탐색 여유가 있으면 JTF를, **관절 궤적·전체 보수적 운전**이 우선이면 IK+관절공간 VSD를 기본으로 두는 편이 안전합니다.

## Controller comparison (best-effort rows)

| controller | best_duration | best_tau_limit | best_gains | rms_ee | final_ee | saturation_steps | ncon_max |
|------------|---------------|----------------|------------|--------|----------|------------------|-----------|
| IK joint-space VSD baseline | 1.0 | 20.0 | Kq=[80,80,60,40], Dq=[10,10,8,5] | 0.008422 | 0.009621 | 232 | 0 |
| pure task-space JTF VSD | 5.0 | 20.0 | Kx=80, Dx=5 (xyz) | 0.007058 | 0.002554 | 0 | 0 |

- 참고 (IK 느린 궤적 동일 조건 스윕): duration=3, tau_limit=50 → rms_ee=0.007209, final_ee=0.006970, sat=670, run_id=7.

