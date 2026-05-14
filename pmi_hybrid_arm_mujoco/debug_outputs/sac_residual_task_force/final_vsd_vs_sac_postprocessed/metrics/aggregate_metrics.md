# Aggregate metrics (paired)

- Seeds: `10000` … `10004` (`num_episodes=5`)
- Mean Δ RMS EE: `-0.0003876664`
- Mean Δ final EE: `-0.0003249602`
- Max ncon (VSD / SAC+pp): `0` / `0`
- Mean Δ HF EE error RMS: `7.415365e-05`
- Seeds with HF worse (Δ>0): `[10000, 10001, 10003]`

## Orientation & vibration linkage (paired)

### RMS / peak orientation error (wrapped vs roll=-π/2, pitch=0)
- Mean RMS roll error — VSD-only: `3.989892e-06`, VSD+SAC+pp: `3.974081e-06`, Δ(SAC−VSD): `-1.581145e-08`
- Mean RMS pitch error — VSD-only: `0.1559606`, VSD+SAC+pp: `0.1606391`, Δ(SAC−VSD): `0.004678539`
- Mean max |roll error| — VSD: `7.342852e-06`, SAC+pp: `7.345296e-06`
- Mean max |pitch error| — VSD: `0.3219255`, SAC+pp: `0.3411967`

### Pearson correlation (per episode; means over seeds)
Columns: VSD-only vs VSD+SAC+pp.
- corr(‖e_xyz‖, |roll_err|): `-0.2457156` / `-0.2388864`
- corr(‖e_xyz‖, |pitch_err|): `0.2281086` / `0.4609646`
- corr(‖e_hf‖, |roll_err|): `-0.2064044` / `-0.1980344`
- corr(‖e_hf‖, |pitch_err|): `0.4273945` / `0.4023945`

- Orientation plots (detail seeds): `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/debug_outputs/sac_residual_task_force/final_vsd_vs_sac_postprocessed/plots/orientation`
