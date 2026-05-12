# Joint Order Verification

- controller joint order: ['jnt1', 'jnt2', 'jnt3', 'jnt4']
- q_actual order (constructed): [0.0, 0.0, 0.0, 0.0]
- qdot order (constructed): [0.0, 0.0, 0.0, 0.0]
- q_des order (constructed): [0.0, 0.0, 0.0, 0.0]
- jnt1: joint_id=0, qpos_adr=0, dof_adr=0
- jnt2: joint_id=1, qpos_adr=1, dof_adr=1
- jnt3: joint_id=2, qpos_adr=2, dof_adr=2
- jnt4: joint_id=3, qpos_adr=3, dof_adr=3

## Per-joint torque routing check
- target jnt1: tau_pd=[3.0, 0.0, 0.0, 0.0], nonzero_qfrc_dof=[0]
- target jnt2: tau_pd=[0.0, 3.0, 0.0, 0.0], nonzero_qfrc_dof=[1]
- target jnt3: tau_pd=[0.0, 0.0, 3.0, 0.0], nonzero_qfrc_dof=[2]
- target jnt4: tau_pd=[0.0, 0.0, 0.0, 3.0], nonzero_qfrc_dof=[3]

Answers:
- q_des[jnt1..4] is applied to the same-order dof list by construction.
- q_des and q_actual are generated with identical J order.
