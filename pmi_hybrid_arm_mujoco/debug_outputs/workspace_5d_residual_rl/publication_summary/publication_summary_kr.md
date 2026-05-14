# 출판/보고용 요약 — 워크스페이스 5D VSD + SAC 잔차 렌치

**시뮬레이션 연구.** 주요 학습 정책 체크포인트: `ws5d_residual_medium_train_rs05_30k_s4` / `best_model_by_smooth_score.zip`.

## 1. 시스템

- **모델:** `models/pmi_hybrid_no_collision.xml`
- **충돌:** 보고된 모든 실행에서 **비활성**
- **명목 제어기:** 워크스페이스 **5D VSD** (과제: $x,y,z$, roll, pitch)
- **Yaw:** **자유** (규제하지 않음)
- **전달:** q1 벨트/기어; q2–q4 케이블; **`qfrc_applied`** 로 **액추측** 토크 (`data.ctrl` 아님)
- **학습 잔차:** **5D 태스크 렌치**를 VSD에 **가산**; VSD **대체 아님**

## 2. SAC 잔차

행동은 환경에서 정규화·스케일링 후 게인·레이트 리밋·필터를 거쳐 관절 토크로 맵핑되고 $\bm{\tau}_{\rm vsd}$ 와 합산됨.

## 3. 커리큘럼 (학습 분포)

**deterministic** $\to$ **mild** $\to$ **medium_v2** $\to$ **medium_train**.  
케이블 **stress** 프로파일은 본 작업에서 **평가 전용**(학습 분포 아님).

## 4. 커리큘럼 성능 (zero 잔차 대비 개선율 %)

| 단계 | RMS EE | Final EE | RMS HF |
| --- | --- | --- | --- |
| deterministic | 47.64% | 45.72% | 28.53% |
| mild | 50.98% | 69.98% | 30.33% |
| medium_v2 | 44.32% | 57.07% | 22.25% |
| medium_train | 44.54% | 47.35% | 14.91% |

(전체 수치: `key_results_table.md`, `curriculum_summary.csv`.)

## 5. 최종 `medium_train` 집계 (에피소드 50, seed 10000–)

- RMS EE 개선: **44.49%**
- Final EE 개선: **52.95%**
- RMS HF 개선: **15.81%**

## 6. Stress 평가 전용

`cable_layer.yaml` 의 **stress** 랜덤화. **학습 단계 아님.**

| 지표 | Zero | SAC | 개선율 |
| --- | --- | --- | --- |
| RMS EE | 0.053114 | 0.039678 | 25.30% |
| Final EE | 0.145339 | 0.116655 | 19.74% |
| RMS HF | 0.030338 | 0.022304 | 26.48% |
| Saturation frac. | 0.010841 | 0.004613 | 57.45% |
| Limit frac. | 0.012889 | 0.007834 | 39.22% |
| ncon max | 0.0 | 0.0 | — |

## 7. 그림·영상

- 그림: `debug_outputs/workspace_5d_residual_rl/publication_summary/figures`
- 영상 목록: `video_index.md`

## 8. 후속 제안

- 동일 명목 VSD 구조·보수적 잔차 권한 하 **하드웨어** 검증.
- 접촉이 중요하면 **충돌 활성** 및 안전 제약 재설계(본 보고 범위 밖).
- stress 를 **학습**에 포함하려면 새 커리큘럼과 **명시적 재학습** 필요.

## 주장 가능 범위

- 시뮬레이션에서 SAC 잔차가 워크스페이스 5D VSD 추적을 개선함.
- deterministic·mild·medium_v2·medium_train 에서 일관된 개선 경향.
- Stress **평가**에서도 개선; 본 스트레스는 **학습 분포가 아님**.
- Yaw 는 의도적으로 자유; 충돌 비활성; VSD 는 명목으로 유지.

## 한계

- **시뮬레이션 전용**; **실기 검증 없음** 주장 안 함.
- 충돌 비활성; 접촉·다접촉 거동은 평가하지 않음.
- Stress 프로파일은 **평가 전용**이며, 새 커리큘럼 설계 없이는 학습 분포가 아님.
- 케이블 층은 `cable_layer.yaml`의 지연·마찰·탄성·히스테리시스·랜덤화 프로파일에 따르며, 결과는 이러한 모델링 선택에 의존함.

---

*생성: `scripts/package_workspace_5d_publication.py` — 학습/XML/제어기 코드 변경 없음.*
