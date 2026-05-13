# RL environment rollout (`PMICableResidualEnv`)

- Profile: `medium_train` (`randomize_cable=True`).
- Episodes per policy: 10.

## Report answers

### 1. Zero residual × medium_train — VSD baseline?
- Mean total reward: **-182.1465**, mean EE error: **0.016890**.
- Max `ncon_max` over zero-residual runs: **0**.

### 2. Random residual — stable?
- Mean EE error: **0.017587**, mean terminated: **0.200**, mean NaN flag: **0.000**.

### 3. Sinusoidal residual — finite / interpretable?
- Mean EE error: **0.021055**, mean ‖F‖: **0.419308**.

### 4–5. Finite obs / rewards?
- Total NaN-flagged rollouts: **0 / 30**.

### 6. Limits / contacts
- Zero policy soft-limit steps (mean jl / act): **3.3** / **4.4**.

### 7. SAC smoke readiness
- If Items 1–5 pass (stable, finite, `ncon_max=0`, rare limits), run `python scripts/train_sac_residual.py --timesteps 1000 --profile medium_train`.

## Per-policy means

| policy | mean total R | mean EE err | max EE err | sat | jl | al | nan | term |
|--------|-------------|-------------|------------|-----|----|----|-----|-----|
| zero_force | -182.1465 | 0.016890 | 0.040564 | 315.5 | 3.3 | 4.4 | 0.000 | 0.300 |
| random_force | -178.2438 | 0.017587 | 0.047065 | 330.4 | 1.2 | 1.5 | 0.000 | 0.200 |
| sinusoid_force | -216.0830 | 0.021055 | 0.044713 | 396.9 | 2.8 | 3.3 | 0.000 | 0.100 |

CSV: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/rl_env/rollout_summary.csv`
