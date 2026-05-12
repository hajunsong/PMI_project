# Hybrid no-collision — summary readiness

## 1. Does hybrid no-collision model have zero contacts?
- 초기 자세 `ncon=0` (예). 
- 위치 매핑 롤아웃 중 `ncon_max=0` (예). 
- 메시 비접촉 플래그: 예

## 2. Does q_act position mapping work?
- 전달 오차 벡터 노름: RMS=1.3327e-04, max=2.1243e-04. 
- (손실 기준 예시: max‖·‖ < 0.005 rad → 통과)

## 3. Does EE path tracking still work with ideal mapping?
- FK 기준 RMS EE 오차 ≈ 0.0274 m, 최대 ≈ 0.0415 m. 
- (추적 양호)

## 4. Is the model ready for ideal q_act torque transmission?
- 등식으로 `jnt = ratio * q_act` 가 이미 물리 제약으로 걸려 있으므로, **다음 단계**는 
  액추에이터 측 토크/힘을 정의한 뒤 관절측 `qfrc_applied` 또는 토크 액추에이터로 
  동일 비율을 역동역학적으로 맞추는 설계가 됩니다. 
- 현재 검증(접촉 없음·전달 소오차): 진행 가능.

## 5. Is the model ready for later cable transmission q2~q4?
- **구조적으로** q2~q4 는 별도 equality/케이블 모델로 `jnt2..4` 와 `q2..4_act` 결합을 바꿀 수 있는 상태입니다. 
- 본 MJCF에는 **케이블 동역학 미포함**. 케이블 도입 시 새 제약·마찰·프리스트레스를 추가해야 합니다.

## Controller baselines (참고, 미혼합)
- **A.** IK joint-space VSD: `tau = qfrc_bias + Kq(q_des−q)+Dq(qdot_des−qdot)` → `jnt1..4` `qfrc_applied`. 
- **B.** 작업공간 JTF VSD: `F=Kx e + Dx edot`, `tau = qfrc_bias + J^T F` → `jnt` `qfrc_applied`. 
- 위치 서보 XML은 **검증/동기용**이며, 토크 명령은 `data.ctrl` 로 직접 보내지 않는 것을 권장(토크 액추에이터 정의 시까지).
