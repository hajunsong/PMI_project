# 다음 SAC 실행 제안

- Run: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/smoke_progress_test`

## 기준선 (zero policy)

- YAML (`baseline_eval.yaml`): mean_return=-203.74261043951554, sat=0.8228, lim=0.010400000000000001, EE_RMS=0.02506300780254417
- (CSV의 오래된 행에서 baseline을 추정하지 **않음**.)

## 최근 periodic eval

- periodic eval 행 없음 (`eval_type=periodic_eval`).

## 스모크 테스트 (≤1000 스텝)

- 매우 짧은 학습에서는 수렴 여부·학습 품질을 **판단하기 어렵습니다**.

## 경고: 액션 norm 로그

- `eval_log.csv`에 action/residual norm 또는 `mean_action_rate`가 비어 있거나 누락되었습니다.
- 다음 실행 제안에 액션 규모 진단을 포함하려면 해당 컬럼이 **필요**합니다.

### 하이퍼파라미터·탐험
- ent_coef·batch·TensorBoard 학습 지표(loss/entropy)를 함께 보고 미세 조정.

### 다음 실행 명령 예시
다음은 **갱신된 인프라** 가정 하의 10k 실행 예시입니다 (필요 시 경로에서 `cd pmi_hybrid_arm_mujoco` 후 실행).

```bash
python scripts/train_sac_residual.py \
  --timesteps 10000 \
  --profile medium_train \
  --run-name sac_tf_medium_train_rs3_10k \
  --seed 1 \
  --progress \
  --early-stop \
  --eval-freq 2000 \
  --eval-episodes 10 \
  --checkpoint-freq 5000 \
  --min-train-steps-before-stop 8000 \
  --patience-evals 4 \
  --learning-rate 0.0003 \
  --batch-size 256 \
  --buffer-size 200000 \
  --learning-starts 1000 \
  --residual-force-scale 3.0 \
  --tau-jnt-limit 30 \
  --use-vecnormalize
```

---

파일 출력: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/smoke_progress_test/logs/next_run_recommendation.md`
