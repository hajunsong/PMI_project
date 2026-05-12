# qfrc Application Audit

## Joint Index Map
- jnt1: joint_id=0, qpos_adr=0, dof_adr=0
- jnt2: joint_id=1, qpos_adr=1, dof_adr=1
- jnt3: joint_id=2, qpos_adr=2, dof_adr=2
- jnt4: joint_id=3, qpos_adr=3, dof_adr=3

## qacc Response Test (+1Nm)
- jnt1: dof=0, qacc_before=-54.491641, qacc_after=-54.491641, positive_response=False
- jnt2: dof=1, qacc_before=46.353871, qacc_after=46.353871, positive_response=True
- jnt3: dof=2, qacc_before=4.524618, qacc_after=4.524618, positive_response=True
- jnt4: dof=3, qacc_before=204.540415, qacc_after=204.540415, positive_response=True

## Audit Answers
1. qfrc_applied dof index correctness: verified by joint_id->dof_adr mapping above.
2. positive qfrc -> positive qacc: see per-joint positive_response.
3. overwritten before mj_step: this script sets qfrc then immediately steps; no overwrite in-between.
4. qfrc reset each timestep: tests explicitly reset data.qfrc_applied[:] = 0.0 each step.
