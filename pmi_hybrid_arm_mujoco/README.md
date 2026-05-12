# PMI 하이브리드 암 MuJoCo 패키지 (`pmi_hybrid_arm_mujoco`)

`ros_ws/pmi_description3` URDF를 원본으로 하여, 모터 축 `q*_act` 와 팔 관절 `jnt*` 의 전동비 검증(Phase 1) → VSD 토크 추종(Phase 2) → 비선형 와이어 전송(Phase 3) 순으로 모듈화한 예제입니다.

## 사전 조건

- Python 3.10+
- STL 메시 경로: 저장소의 `ros_ws/pmi_description3/meshes/` (MJCF `meshdir` 상대경로로 연결됨)
- 패키지 디렉터리에서 `PYTHONPATH=.` 로 스크립트를 실행합니다.

## 의존성 설치

```bash
cd pmi_hybrid_arm_mujoco
pip install -r requirements.txt
```

## URDF 재컴파일 (선택)

URDF/MuJoCo 버전에 맞춰 `models/pmi_hybrid_arm.xml` 을 다시 만들 때:

```bash
cd pmi_hybrid_arm_mujoco
PYTHONPATH=. python scripts/regenerate_mjcf.py
```

## 실행

```bash
cd pmi_hybrid_arm_mujoco
PYTHONPATH=. python scripts/inspect_urdf.py
PYTHONPATH=. python scripts/test_phase1_motion_input.py
PYTHONPATH=. python scripts/test_phase2_vsd_torque.py
PYTHONPATH=. python scripts/test_phase3_cable_transmission.py
```

- Phase 1 결과: `figures/phase1_motion.png`, `figures/phase1_joint_error.png`
- Phase 2: `figures/phase2_vsd.png`
- Phase 3: `figures/phase3_tracking_torque_bundle.png`

## IK 경로 추종 (Phase A / B)

`configs/path_tracking.yaml` — 웨이포인트, 자세(roll/pitch, yaw 자유), IK 가중치·감쇠, 전동비, Phase A 위치 제어 · Phase B 관절 공간 VSD.

```bash
cd pmi_hybrid_arm_mujoco
PYTHONPATH=. python scripts/test_phaseA_ik_position_tracking.py
PYTHONPATH=. python scripts/test_phaseB_ik_vsd_tracking.py
```

- Phase A 그림: `figures/phaseA_ik_tracking_*.png`
- Phase B 그림: `figures/phaseB_ik_vsd_*.png`

**토크 매핑(명세)**: `tau_act = ratio * tau_joint` (`controllers/vsd_joint_controller.py` 참고).

## 설정

- `configs/robot_transmission.yaml` — 관절별 ratio, `r_motor` / `r_joint` (케이블 신율에 사용)
- `configs/vsd_params.yaml` — `Kp`, `Kd`, 위치 궤적(사인/스텝/정지)
- `configs/cable_params.yaml` — antagonistic + Bouc–Wen 등 (jnt2~4)
- `configs/simulation.yaml` — 적분 스텝·시뮬 시간·Phase1 허용 오차 등
- `configs/path_tracking.yaml` — 웨이포인트 IK + Phase A 위치 추종 · Phase B 조인트 VSD

## 노트

- **Phase 2** 는 MJCF equality 없이, 스텝 후 `apply_kinematic_transmission` 으로 모터 축과 관절을 기하적으로 맞추고, 관절 dof에 `tau_act/ratio` 형태의 이상 토크를 가합니다.
- **Phase 3** 에서 jnt1 만 MJCF equality 로 벨트를 구속하고, jnt2~4 는 antagonistic 케이블 모델이 관절·모터에 토크를 나눠 넣습니다.
