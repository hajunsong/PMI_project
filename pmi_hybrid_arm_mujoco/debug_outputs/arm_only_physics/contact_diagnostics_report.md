# Arm-only contact / physics diagnostics

- MJCF: `/home/keti/Project/12_FieldSensor/PMI_Project/pmi_hybrid_arm_mujoco/models/pmi_arm_only_torque_debug.xml`
- Initial q (rad): [0.0, 0.0, 0.0, 0.0]
- **ncon**: 1

## Contact pairs

| idx | geom1 | geom2 | dist |
|-----|-------|-------|------|
| 0 | world:geom[0] | link1:geom[1] | -0.0742541 |

## Per-joint generalized forces (after mj_forward)

| joint | dof | qfrc_bias | qfrc_constraint | qfrc_passive | sum(excl applied) |
|-------|-----|-----------|-----------------|--------------|-------------------|
| jnt1 | 0 | 4.4374754e-08 | -1.6237783 | 0 | -1.6237783 |
| jnt2 | 1 | 0.038993027 | 0 | 0 | 0.038993027 |
| jnt3 | 2 | 0.039729718 | 0 | 0 | 0.039729718 |
| jnt4 | 3 | -0.038945466 | 0 | 0 | -0.038945466 |

Notes: `qfrc_applied` was zero during this diagnostic.
