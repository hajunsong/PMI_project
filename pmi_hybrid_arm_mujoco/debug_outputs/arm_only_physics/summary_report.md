# Arm-only torque 모델 물리 진단 요약

출력 디렉터리: `pmi_hybrid_arm_mujoco/debug_outputs/arm_only_physics/`  
(요청한 `model_physics` 대신 arm-only 결과만 모은 **별도 폴더**입니다.)

## 1. 접촉 / 충돌

**결론: jnt1 이상의 주원인 후보로 접촉·제약이 가장 유력합니다.**

근거 (`contact_diagnostics_report.md`, `diagnose_contacts_arm_only.py`):

- 기본 MJCF (`pmi_arm_only_torque_debug.xml`)에서 **ncon = 1**.
- 접촉 쌍: **베이스 메시(world 고정 geom) ↔ link1 메시**, 침투 거리 **dist ≈ −0.074** (음수 = 관통).
- **jnt1**에만 비정상적으로 큰 **`qfrc_constraint ≈ −1.62` N·m** (bias·passive는 미미).
- 다른 관절은 `qfrc_constraint = 0`.

즉, 단순 중력 바이어스 문제가 아니라 **메시 간 접촉 제약이 jnt1 축에 큰 역할을 넣고** 있습니다.

## 2. 무충돌 메시 모델과 단일 관절 추종

**결론: 접촉을 끄면 jnt1 추종 오차가 다른 관절과 동일 스케일로 돌아옵니다.**

근거 (`single_joint_tracking_collision_compare_report.md`, 동일 폴더 CSV):

| 설정 | jnt1 (예: Δq=+0.5) max ‖q_des−q‖₂ |
|------|-------------------------------------|
| 충돌 ON (기본) | ≈ **0.527** |
| 메시 무충돌 (`pmi_arm_only_no_collision.xml`) | **0.5** (목표 클램프 한계와 동일한 양만큼만 남는 수준; jnt2~4와 동일 패턴) |

→ **동일 PD+bias 법칙**에서 차이는 거의 전적으로 **충돌 모델 유무**에서 옵니다.

## 3. 중력 ON/OFF

**결론: 이번 설정에서는 jnt1 실패를 중력만으로 설명하기 어렵습니다.**

근거 (`single_joint_tracking_gravity_compare_report.md`):

- 중력 ON/OFF 모두 **jnt1 행이 동일** (예: Δq=+0.5 시 max 오차 ≈ 0.527).
- 접촉 제약이 그대로일 때는 중력 유무와 무관하게 같은 접촉 힘이 지배적일 수 있습니다.

## 4. qfrc_applied → qacc 부호 (`audit_qacc_response_corrected.py`)

**결론: 라우팅 오류보다는 “접촉 하에서 역학이 망가진 상태”에 가깝습니다.**

- **충돌 ON**: jnt1에서 **+τ와 −τ 모두 qacc가 같은 부호로 크게 치우침** → 단순 부호 반전으로 고칠 문제가 아님.
- **무충돌 MJCF**: jnt1~jnt4 **모두 τ 부호와 qacc 부호 일치** (`qacc_response_audit.csv`).

## 5. 총합 — 무엇이 jnt1 실패를 설명하는가?

| 후보 | 설명력 |
|------|--------|
| **접촉/충돌 (메시 침투·제약)** | **매우 높음** — 제약 토크, 침투 접촉, 무충돌 시 즉시 정상화 |
| **중력 단독** | 낮음 — ON/OFF에서 jnt1 표가 동일 |
| **qfrc 적용/ dof 매핑 단순 오류** | 낮음 — 무충돌 모델에서 부호 일관성 양호 |

## 생성된 파일

| 파일 | 내용 |
|------|------|
| `contact_diagnostics_report.md` | ncon, 접촉 쌍, dof별 bias/constraint/passive |
| `single_joint_tracking_collision_compare.csv` | 충돌 vs 무충돌 시계열 |
| `single_joint_tracking_collision_compare_report.md` | 관절·Δq별 max 오차 요약 |
| `single_joint_tracking_gravity_compare.csv` | 중력 on/off 시계열 |
| `single_joint_tracking_gravity_compare_report.md` | 요약 표 |
| `qacc_response_audit.csv` | 두 MJCF에 대한 τ vs qacc |
| `qacc_response_audit_report.md` | 해석 노트 |

## 권장 다음 조치 (진단 범위 밖 참고)

- 기본 디버그 MJCF에서 **베이스–링크1 메시 간 간격** 또는 베이스만 plane 등 **충돌 단순화**.
- 또는 작업 시 **`pmi_arm_only_no_collision.xml`** 로 제어 검증 후, 시각 메시만 유지한 채 충돌 geom 분리.
