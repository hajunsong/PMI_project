# Controller Comparison Report

Controllers:
- A. pure JTF task-space VSD
- B. IK + joint-space VSD
- C. DLS resolved-rate + joint torque servo

- pure_task_space_jtf_vsd: rms=0.294156, max=0.462561, final=0.462180, max_tau=10.0000, jl=0, sat=302
- ik_joint_space_vsd: rms=0.322011, max=0.512764, final=0.512549, max_tau=10.0000, jl=0, sat=487
- dls_resolved_rate_joint_torque: rms=0.295194, max=0.451228, final=0.450763, max_tau=10.0000, jl=0, sat=475

Success criteria pass/fail:
- pure_task_space_jtf_vsd: False
- ik_joint_space_vsd: False
- dls_resolved_rate_joint_torque: False

No controller reached target criteria. Likely dominant limit: torque limit issue.
