# Noise robustness — paired zero vs SAC

## clean

| metric | zero | SAC | Δ (SAC−zero) |
| --- | --- | --- | --- |
| RMS EE (true) | 0.006738 | 0.003747 | -0.002991 |
| final EE (true) | 0.001295 | 0.000699 | -0.000596 |
| RMS EE (measured) | 0.006738 | 0.003747 | — |
| RMS HF | 0.000639 | 0.000548 | -0.000091 |
| smooth | 0.015236 | 0.010994 | — |
| sat frac | 0.000000 | 0.000000 | 0.000000 |
| lim frac | 0.000000 | 0.000000 | 0.000000 |
| ncon max | 0.000000 | 0.000000 | — |

- Mean RMS EE improvement (true): **44.39 %**

## sensor_noise_only

| metric | zero | SAC | Δ (SAC−zero) |
| --- | --- | --- | --- |
| RMS EE (true) | 0.006740 | 0.004275 | -0.002465 |
| final EE (true) | 0.001330 | 0.001058 | -0.000272 |
| RMS EE (measured) | 0.006796 | 0.004361 | — |
| RMS HF | 0.000659 | 0.000621 | -0.000038 |
| smooth | 0.015287 | 0.012110 | — |
| sat frac | 0.000000 | 0.000000 | 0.000000 |
| lim frac | 0.000000 | 0.000000 | 0.000000 |
| ncon max | 0.000000 | 0.000000 | — |

- Mean RMS EE improvement (true): **36.57 %**

## delay_only

| metric | zero | SAC | Δ (SAC−zero) |
| --- | --- | --- | --- |
| RMS EE (true) | 0.006138 | 0.003621 | -0.002517 |
| final EE (true) | 0.001173 | 0.000835 | -0.000339 |
| RMS EE (measured) | 0.006771 | 0.003900 | — |
| RMS HF | 0.000650 | 0.000706 | 0.000056 |
| smooth | 0.014292 | 0.011129 | — |
| sat frac | 0.000000 | 0.000000 | 0.000000 |
| lim frac | 0.000000 | 0.000000 | 0.000000 |
| ncon max | 0.000000 | 0.000000 | — |

- Mean RMS EE improvement (true): **41.01 %**

## sensor_delay_filter

| metric | zero | SAC | Δ (SAC−zero) |
| --- | --- | --- | --- |
| RMS EE (true) | 0.012816 | 0.010554 | -0.002262 |
| final EE (true) | 0.031181 | 0.032443 | 0.001262 |
| RMS EE (measured) | 0.013113 | 0.010474 | — |
| RMS HF | 0.007363 | 0.006701 | -0.000662 |
| smooth | 0.064577 | 0.062207 | — |
| sat frac | 0.000500 | 0.000331 | -0.000170 |
| lim frac | 0.001296 | 0.001234 | -0.000063 |
| ncon max | 0.000000 | 0.000000 | — |

- Mean RMS EE improvement (true): **17.65 %**

## actuator_noise_only

| metric | zero | SAC | Δ (SAC−zero) |
| --- | --- | --- | --- |
| RMS EE (true) | 0.006818 | 0.004058 | -0.002761 |
| final EE (true) | 0.001738 | 0.001393 | -0.000346 |
| RMS EE (measured) | 0.006818 | 0.004058 | — |
| RMS HF | 0.000776 | 0.000699 | -0.000077 |
| smooth | 0.015837 | 0.012084 | — |
| sat frac | 0.000000 | 0.000000 | 0.000000 |
| lim frac | 0.000000 | 0.000000 | 0.000000 |
| ncon max | 0.000000 | 0.000000 | — |

- Mean RMS EE improvement (true): **40.49 %**

## full_noise_light

| metric | zero | SAC | Δ (SAC−zero) |
| --- | --- | --- | --- |
| RMS EE (true) | 0.022786 | 0.018538 | -0.004248 |
| final EE (true) | 0.074272 | 0.057408 | -0.016864 |
| RMS EE (measured) | 0.022276 | 0.018113 | — |
| RMS HF | 0.016353 | 0.013453 | -0.002900 |
| smooth | 0.135669 | 0.108367 | — |
| sat frac | 0.001569 | 0.001714 | 0.000145 |
| lim frac | 0.003345 | 0.002500 | -0.000844 |
| ncon max | 0.000000 | 0.000000 | — |

- Mean RMS EE improvement (true): **18.64 %**

## full_noise_medium

| metric | zero | SAC | Δ (SAC−zero) |
| --- | --- | --- | --- |
| RMS EE (true) | 0.086176 | 0.084429 | -0.001748 |
| final EE (true) | 0.282569 | 0.271022 | -0.011547 |
| RMS EE (measured) | 0.078333 | 0.076688 | — |
| RMS HF | 0.065245 | 0.063955 | -0.001291 |
| smooth | 0.522190 | 0.502383 | — |
| sat frac | 0.026451 | 0.023659 | -0.002792 |
| lim frac | 0.018602 | 0.019327 | 0.000725 |
| ncon max | 0.000000 | 0.000000 | — |

- Mean RMS EE improvement (true): **2.03 %**

## Notes

- Primary metrics use **true** EE error from simulation.
- **Measured** columns use the noisy/delayed/filtered observation pipeline (see `configs/rl_workspace_5d_sac.yaml` → `noise`).
- If noisy evaluation degrades vs clean, consider lowering residual authority, stronger filtering, or future noise-aware training.
