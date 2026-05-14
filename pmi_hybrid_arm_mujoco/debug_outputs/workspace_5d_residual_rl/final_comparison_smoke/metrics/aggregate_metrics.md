# Aggregate metrics (paired, mean over episodes)

- Episodes: 1, seed_start=10000
- Curriculum stage: `medium_train`

| Metric | Zero (VSD) | SAC residual | Δ |
| --- | --- | --- | --- |
| RMS EE | 0.006416 | 0.003336 | -0.003079 |
| Final EE | 0.001152 | 0.000463 | -0.000689 |
| RMS HF | 0.000616 | 0.000543 | -0.000073 |
| Smooth | 0.014535 | 0.010043 | -0.004492 |
| Saturation | 0.000000 | 0.000000 | 0.000000 |
| Limit frac. | 0.000000 | 0.000000 | 0.000000 |

- Percent RMS improvement: 47.9982 %
- Percent final EE improvement: 59.8038 %
- Percent RMS HF improvement: 11.7968 %
