# SAC residual training summary

- **profile**: medium_train
- **run_name**: sac_tf_baseline_relative_rs1_30k_s6
- **requested timesteps**: 30000
- **baseline mean episode return (zero policy)**: -1163.6444424754363
- **stop / completion reason**: completed
- **successful_tracking_improvement** (final mean EE RMS < baseline mean EE RMS): False

Artifacts: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_baseline_relative_rs1_30k_s6/checkpoints/`, `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_baseline_relative_rs1_30k_s6/logs/`, `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/runs/sac_tf_baseline_relative_rs1_30k_s6/tensorboard/`.

## Best checkpoints (from periodic eval)

- **best reward checkpoint** (`best_model_by_reward.zip`): reward=-1048.2126588042972, step=30000
- **best EE RMS checkpoint** (`best_model_by_ee_rms.zip`): mean_ee_rms=0.02068739808135961, step=1 (mean final EE error at that eval: 0.019394828809532142, sat=0.7437461074030334, lim=0.02038551682346749)
- **best combined tracking** (`best_model_by_combined_tracking.zip`): score=0.029787302142060715 (mean EE RMS + 0.5×mean final EE error), step=15000
- **best smooth tracking** (`best_model_by_smooth_tracking.zip`): score=3.3251773131089446, step=10000 (lower = better RMS + weighted final / velocity / HF / torque rates; see `logs/best_metrics.yaml`)
- **best baseline-relative (proxy) smooth** (`best_model_by_relative_smooth_score.zip`): score=3.3251773131089446, step=10000 (periodic eval uses `mean_smooth_tracking_score` as proxy; 짝 평가로 상대 지표 확정)

## Diagnostic table (baseline vs final eval)

| Metric | Baseline (zero policy) | Final eval |
| --- | --- | --- |
| mean episode return | -1163.64 | -1043.97 |
| mean EE RMS (m) | 0.0190614 | 0.0207597 |
| mean saturation fraction | 0.7532 | 0.74265 |
| mean limit violation fraction | 0.02 | 0.02078 |

- **Percent improvement in EE RMS** (baseline 대비 감소): -8.91% (baseline 대비 EE RMS 감소율)
- **Percent change in mean return** (의미 있을 때만 참고): 10.28% (return 변화율; 음수 보상 스케일에서 해석 유의)

## Paired evaluation

같은 케이블/환경 시드에서 잔차 0 베이스라인과 SAC를 짝지어 평가하면, 랜덤 시드 변동에 가려진 개선 여부를 확인할 수 있습니다.

### 권장: baseline-relative 학습 (다음 실험 후보)

베이스라인 VSD(동일 케이블 시드에서 잔차 0) 대비 개선을 보상에 직접 넣을 때 사용합니다. 주기적 평가에서는 `best_model_by_relative_smooth_score.zip` 선택에 `mean_smooth_tracking_score` 프록시가 쓰이며, 학습 후에는 아래 짝 평가로 최종 판정합니다.

```bash
python scripts/train_sac_residual.py \
  --timesteps 30000 --profile medium_train --run-name sac_tf_baseline_relative_rs1_30k_s6 --seed 6 \
  --progress --early-stop \
  --eval-freq 5000 --eval-episodes 30 --checkpoint-freq 5000 \
  --min-train-steps-before-stop 20000 --patience-evals 6 --min-improvement 0.003 \
  --learning-rate 0.0001 --batch-size 256 --buffer-size 200000 --learning-starts 2000 \
  --residual-force-scale 1.0 --tau-jnt-limit 30 \
  --reward-preset baseline_relative_smooth \
  --baseline-relative-reward --include-baseline-reference-in-obs \
  --residual-filter --residual-filter-tau 0.08 \
  --action-smoothing --max-delta-force-per-step 0.2 \
  --use-vecnormalize
```

```bash
python scripts/evaluate_sac_residual.py --paired-seeds --model-path <run_dir>/checkpoints/final_model.zip \
  --vecnormalize-path <run_dir>/vecnormalize/vecnormalize.pkl \
  --config configs/rl_sac.yaml --profile medium_train --seed-start 10000 --num-episodes 20
```

