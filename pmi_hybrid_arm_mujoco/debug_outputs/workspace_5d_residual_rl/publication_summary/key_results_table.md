# Key results tables

## A. Curriculum performance

| stage | zero RMS EE | SAC RMS EE | RMS imp % | zero final | SAC final | final imp % | zero HF | SAC HF | HF imp % | Δ sat | Δ limit | ncon |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| deterministic | 0.006140130555392373 | 0.003215114283518132 | 47.63768857171024 | 0.0014587039527353283 | 0.0007918224614098231 | 45.71739797338482 | 0.0005836432881462491 | 0.0004171495325084363 | 28.52662902483217 | 0.0 | 0.0 | z 0.0/ s 0.0 |
| mild | 0.0062508248165651075 | 0.003063897319369249 | 50.98411154877171 | 0.001396435921622576 | 0.00041926289123332526 | 69.97621697197773 | 0.0005929281143302726 | 0.0004130827847655377 | 30.33172575530084 | 0.0 | 0.0 | z 0.0/ s 0.0 |
| medium_v2 | 0.006860213427138209 | 0.0038194993374947505 | 44.32389927716747 | 0.0014223653335704344 | 0.0006106650506591981 | 57.066933772471714 | 0.0006426572393601665 | 0.000499675220131342 | 22.248565871782336 | 0.0 | 0.0 | z 0.0/ s 0.0 |
| medium_train | 0.006743868562519157 | 0.0037398611557831114 | 44.54427572078756 | 0.0012970633801863563 | 0.0006829137818215552 | 47.34923580037876 | 0.0006380806988064748 | 0.0005429598960713816 | 14.907331143696394 | 0.0 | 0.0 | z 0.0/ s 0.0 |

## B. Final `medium_train` aggregate (50 paired episodes)

Per-episode data: `debug_outputs/workspace_5d_residual_rl/final_comparison_medium_train/metrics/paired_metrics_50episodes.csv`. Aggregate: `final_comparison_medium_train/metrics/aggregate_metrics.md`.

| metric | zero | SAC | Δ | improvement % |
| --- | --- | --- | --- | --- |
| RMS EE | 0.006770 | 0.003758 | -0.003012 | 44.4882 |
| Final EE | 0.001287 | 0.000606 | -0.000681 | 52.9482 |
| RMS HF | 0.000639 | 0.000538 | -0.000101 | 15.8148 |

## C. Stress (evaluation-only)

| metric | zero | SAC | Δ | improvement % |
| --- | --- | --- | --- | --- |
| RMS EE | 0.053114 | 0.039678 | -0.013436000000000003 | 25.2965 |
| Final EE | 0.145339 | 0.116655 | -0.028684 | 19.7359 |
| RMS HF | 0.030338 | 0.022304 | -0.008034 | 26.4816 |
| Saturation frac. | 0.010841 | 0.004613 | -0.006228 | 57.4486 |
| Limit frac. | 0.012889 | 0.007834 | -0.005054999999999999 | 39.2195 |
| ncon max | 0.0 | 0.0 | 0.0 | — |

## D. Controller configuration (YAML snapshot)

| Item | Value |
| --- | --- |
| VSD control_mode | jt |
| VSD K_xyz | [150.0, 150.0, 230.0] |
| VSD D_xyz | [5.28, 5.28, 5.28] |
| VSD K_rp | [51.5, 51.5] |
| VSD D_rp | [1.15, 1.15] |
| VSD lambda_dls | 0.12 |
| Residual force scale | 0.5 |
| Residual moment scale | 0.02 |
| Residual mapping | jt |
| Task | x, y, z, roll, pitch (5D); **yaw free** |
| Training cable profile (final stage) | `medium_train` (see `cable_layer.yaml`) |
