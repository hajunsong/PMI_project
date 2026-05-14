# Workspace 5D VSD + SAC residual — 다음 실행 권장

## 상태 요약

- **전체 커리큘럼 완료 시**: 추가 SAC 학습 없이 **`summarize_workspace_5d_curriculum.py`**, **`final_compare_workspace_5d_vsd_vs_sac.py`** 로 표·플롯·영상·`final_report.md` 를 생성합니다.
- **Stress** (`cable_layer.yaml` 의 `stress` 프로파일): **`--curriculum-stage stress`** 는 **평가 전용**이며 학습 스크립트에는 포함하지 않았습니다.

고정: 워크스페이스 5D VSD nominal, SAC는 5D 태스크 렌치 잔차, yaw-free, XML·충돌 비활성, `data.ctrl`·관절 DOF 직접 토크 금지, `qfrc_applied`만.

---

## 0. 커리큘럼 종합 표·최종 비교 (학습 없음)

```bash
cd pmi_hybrid_arm_mujoco

# 네 단계 evaluation 폴더를 모아 curriculum_summary.csv / .md + plots
python scripts/summarize_workspace_5d_curriculum.py

# medium_train 체크포인트로 50회 짝 평가 + 집계 플롯 + 상세 PNG(시드 10000) + 영상(시드 10000–10004)
python scripts/final_compare_workspace_5d_vsd_vs_sac.py

# 영상 생략(빠른 재생성) 시:
# python scripts/final_compare_workspace_5d_vsd_vs_sac.py --no-videos
```

Stress **평가만** (기본 실행 안 함):

```bash
python scripts/evaluate_workspace_5d_residual_sac.py \
  --config configs/rl_workspace_5d_sac.yaml \
  --model-path debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_medium_train_rs05_30k_s4/checkpoints/best_model_by_smooth_score.zip \
  --vecnormalize-path debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_medium_train_rs05_30k_s4/vecnormalize/vecnormalize.pkl \
  --curriculum-stage stress \
  --num-episodes 50 \
  --seed-start 20000 \
  --out-dir debug_outputs/workspace_5d_residual_rl/evaluation/ws5d_residual_stress_eval_only
```

---

## 1. `medium_train` 학습 (참고 — 이미 통과한 경우 생략)

`medium_v2` 베스트에서 이어가며 VecNormalize 매칭. **eval 에피소드 30**으로 랜덤화 분산을 줄입니다.

```bash
cd pmi_hybrid_arm_mujoco

python scripts/train_workspace_5d_residual_sac.py \
  --config configs/rl_workspace_5d_sac.yaml \
  --run-name ws5d_residual_medium_train_rs05_30k_s4 \
  --timesteps 30000 \
  --seed 4 \
  --resume-from debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_medium_v2_rs05_30k_s3/checkpoints/best_model_by_smooth_score.zip \
  --vecnormalize-path debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_medium_v2_rs05_30k_s3/vecnormalize/vecnormalize.pkl \
  --curriculum-stage medium_train \
  --progress \
  --early-stop \
  --eval-freq 5000 \
  --eval-episodes 30 \
  --checkpoint-freq 5000 \
  --learning-rate 0.0001 \
  --residual-force-scale 0.5 \
  --residual-moment-scale 0.02
```

- **첫 러닝에서는 `residual_force_scale`을 올리지 마세요.**
- 리플레이 버퍼가 불안하면 동일 하이퍼·`medium_train`·VecNormalize로 **처음부터** 학습 가능.

---

## 2. `medium_train` 성공 기준 (평가)

평가 리포트의 **`medium_train_improved`** 및 (참고) `deterministic_improved`, `medium_v2_improved`.

목표(평균 짝 지표):

- **Δ RMS EE < 0**
- **Δ final EE ≤ 0.0002** (또는 더 좋음)
- **Δ RMS HF ≤ 0.00005** (또는 더 좋음)
- 포화·limit이 **의미 있게** 증가하지 않음 (리포트·스크립트 임계값 참고)
- **모든 에피소드 ncon max = 0**
- 잔차가 **완만하고 작게** 유지 (플롯·`mean_residual_wrench_norm`)

---

## 3. 학습 후 평가 (`best_model_by_smooth_score.zip`)

에피소드 **50**으로 분산 완화.

```bash
python scripts/evaluate_workspace_5d_residual_sac.py \
  --config configs/rl_workspace_5d_sac.yaml \
  --model-path debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_medium_train_rs05_30k_s4/checkpoints/best_model_by_smooth_score.zip \
  --vecnormalize-path debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_medium_train_rs05_30k_s4/vecnormalize/vecnormalize.pkl \
  --curriculum-stage medium_train \
  --num-episodes 50 \
  --seed-start 10000 \
  --out-dir debug_outputs/workspace_5d_residual_rl/evaluation/ws5d_residual_medium_train_rs05_30k_s4
```

리포트: zero / SAC / Δ, 개선율(%), 포화·limit·ncon 요약, **`medium_train_improved`**, 다음 단계 권장.

---

## 4. 비교 플롯

```bash
python scripts/compare_workspace_5d_vsd_vs_sac.py \
  --config configs/rl_workspace_5d_sac.yaml \
  --model-path debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_medium_train_rs05_30k_s4/checkpoints/best_model_by_smooth_score.zip \
  --vecnormalize-path debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_medium_train_rs05_30k_s4/vecnormalize/vecnormalize.pkl \
  --curriculum-stage medium_train \
  --seed 10000 \
  --out-dir debug_outputs/workspace_5d_residual_rl/comparison/ws5d_residual_medium_train_rs05_30k_s4
```

(기존 스크립트가 XYZ·EE·roll/pitch·yaw·3D·렌치·토크·VSD 분해·케이블·HF·smooth 대시보드 등을 생성합니다.)

---

## 5. `medium_train` 이후 로직 (수동)

| 결과 | 권장 |
| --- | --- |
| `medium_train_improved` 통과 | **첫 견고 체크포인트**로 보관. deterministic·mild·medium_v2·medium_train **통합 비교**, 최종 리포트·영상 |
| RMS만 좋고 final/HF 악화 | `residual_force_scale=0.5` 유지, `w_final_xyz`·HF 보상 강화, 맹목적 연장 금지 |
| 포화·limit 악화 | `≈0.3` / `≈0.01` 스케일, LR `1e-4` 또는 `5e-5`로 재시도 |
| 전반 실패 | 잔차 축소 또는 `medium_v2` 복귀 후 재진입 |

---

## 6. 참고: `medium_v2` 단계 (통과함)

```bash
python scripts/train_workspace_5d_residual_sac.py \
  --config configs/rl_workspace_5d_sac.yaml \
  --run-name ws5d_residual_medium_v2_rs05_30k_s3 \
  --timesteps 30000 --seed 3 \
  --resume-from debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_mild_rs05_30k_s2/checkpoints/best_model_by_smooth_score.zip \
  --vecnormalize-path debug_outputs/workspace_5d_residual_rl/runs/ws5d_residual_mild_rs05_30k_s2/vecnormalize/vecnormalize.pkl \
  --curriculum-stage medium_v2 \
  --progress --early-stop --eval-freq 5000 --eval-episodes 20 --checkpoint-freq 5000 \
  --learning-rate 0.0001 --residual-force-scale 0.5 --residual-moment-scale 0.02
```

---

## 커리큘럼 순서 (건너뛰기 금지)

`deterministic` → `mild` → `medium_v2` → `medium_train` → (통과 후) stress / 최종 평가.
