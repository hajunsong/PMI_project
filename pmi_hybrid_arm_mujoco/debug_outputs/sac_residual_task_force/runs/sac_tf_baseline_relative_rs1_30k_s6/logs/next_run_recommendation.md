# Next run recommendation

- Return은 개선되었으나 EE RMS가 악화되었습니다.
- `tracking_focused` 보상(`--reward-preset tracking_focused`) 전환을 검토하세요.
- `--residual-force-scale`을 낮추는 것을 검토하세요.
- 베스트 모델은 EE RMS 기준(`best_model_by_ee_rms.zip`)을 사용하세요.
- `scripts/evaluate_sac_residual.py --paired-seeds`로 짝 평가를 먼저 수행하세요.
- 동일 설정으로 타임스텝만 늘리는 것은 권장하지 않습니다.

