# Next run recommendation

- Return은 개선되었으나 EE RMS가 악화되었습니다.
- `tracking_focused` 보상(`--reward-preset tracking_focused`) 전환을 검토하세요.
- `--residual-force-scale`을 낮추는 것을 검토하세요.
- 베스트 모델은 EE RMS 기준(`best_model_by_ee_rms.zip`)을 사용하세요.
- `scripts/evaluate_sac_residual.py --paired-seeds`로 짝 평가를 먼저 수행하세요.
- 동일 설정으로 타임스텝만 늘리는 것은 권장하지 않습니다.

## Recommended command

다음 실험은 `pmi_hybrid_arm_mujoco` 디렉터리에서 실행하세요.

```bash
python scripts/train_sac_residual.py \
  --timesteps 30000 \
  --profile medium_train \
  --run-name sac_tf_tracking_reward_rs2_30k_s2 \
  --seed 2 \
  --progress \
  --early-stop \
  --eval-freq 5000 \
  --eval-episodes 20 \
  --checkpoint-freq 10000 \
  --min-train-steps-before-stop 25000 \
  --patience-evals 6 \
  --min-improvement 0.005 \
  --learning-rate 0.0003 \
  --batch-size 256 \
  --buffer-size 200000 \
  --learning-starts 1000 \
  --residual-force-scale 2.0 \
  --tau-jnt-limit 30 \
  --reward-preset tracking_focused \
  --use-vecnormalize
```

**Note:** 짝 평가에서 SAC가 EE RMS를 실제로 개선하는 것이 확인되기 전에는 100k 스텝 학습으로 바로 늘리지 마세요.
