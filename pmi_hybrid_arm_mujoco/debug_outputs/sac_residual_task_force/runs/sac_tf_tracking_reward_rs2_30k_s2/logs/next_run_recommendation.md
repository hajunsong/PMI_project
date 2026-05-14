# Next run recommendation

## Recommended command (100k continuation)

`pmi_hybrid_arm_mujoco` 디렉터리에서 실행하세요.

```bash
python scripts/train_sac_residual.py \
  --timesteps 100000 \
  --profile medium_train \
  --run-name sac_tf_tracking_reward_rs2_100k_s2 \
  --seed 2 \
  --resume-from debug_outputs/sac_residual_task_force/runs/sac_tf_tracking_reward_rs2_30k_s2/checkpoints/best_model_by_ee_rms.zip \
  --progress \
  --early-stop \
  --eval-freq 10000 \
  --eval-episodes 30 \
  --checkpoint-freq 20000 \
  --min-train-steps-before-stop 60000 \
  --patience-evals 8 \
  --min-improvement 0.003 \
  --learning-rate 0.0003 \
  --batch-size 256 \
  --buffer-size 300000 \
  --learning-starts 2000 \
  --residual-force-scale 2.0 \
  --tau-jnt-limit 30 \
  --reward-preset tracking_focused \
  --use-vecnormalize
```

## Notes

- **`--resume-from`** 사용 전에 리플레이 버퍼와 VecNormalize 통계 복원이 의도대로 되는지 확인하세요. SB3에서 체크포인트만 로드하면 버퍼가 비어 있을 수 있습니다.
- 복원이 불확실하면 **`--resume-from` 없이** 동일 하이퍼파라미터로 새로 학습하는 편이 안전할 수 있습니다.

짝 평가에서 EE RMS는 개선되었으나 종료 시점 final EE가 다소 악화된 적이 있으므로, **`tracking_focused` 프리셋의 `use_terminal_final_error_penalty` / `w_final_ee`**(종료 시점 패널티)를 켠 상태로 이어 학습하는 것을 권장합니다.
