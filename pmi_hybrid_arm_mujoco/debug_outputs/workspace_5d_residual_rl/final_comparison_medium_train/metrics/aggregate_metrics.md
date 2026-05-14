# Aggregate metrics (paired, mean over episodes)

- Episodes: 50, seed_start=10000
- Curriculum stage: `medium_train`

| Metric | Zero (VSD) | SAC residual | Δ |
| --- | --- | --- | --- |
| RMS EE | 0.006770 | 0.003758 | -0.003012 |
| Final EE | 0.001287 | 0.000606 | -0.000681 |
| RMS HF | 0.000639 | 0.000538 | -0.000101 |
| Smooth | 0.015297 | 0.010931 | -0.004366 |
| Saturation | 0.000000 | 0.000000 | 0.000000 |
| Limit frac. | 0.000000 | 0.000000 | 0.000000 |

- Percent RMS improvement: 44.4882 %
- Percent final EE improvement: 52.9482 %
- Percent RMS HF improvement: 15.8148 %
