# Arm-only Debug Summary

1. Does the arm-only model track better than hybrid?
- Yes, but only partially. Best arm-only final error is about `0.428` (pure JTF), better than hybrid best around `0.450~0.462`, but still far from good tracking.

2. Does removing q_act and Python sync fix the issue?
- It improves consistency and removes mimic-sync ambiguity, but it does not fully fix tracking.

3. Is tau_bias sign correct?
- Yes. Bias sign test shows `sign=+1` has much smaller drift than `sign=-1` (`0.0184` vs `0.0913`), so compensation should use `tau = +qfrc_bias`.

4. Are torque impulse signs correct after bias compensation?
- Mostly yes for jnt2~jnt4. jnt1 shows unexpected sign behavior for `+delta` (delta_q_target stays negative), so jnt1 axis/sign convention should be treated carefully.

5. Which controller works best in arm-only mode?
- By RMS and final error in this run: `pure_jtf_task_vsd`.

6. Should we continue with task-space VSD, DLS, or IK+joint-space VSD?
- Continue with task-space VSD first on arm-only debug model, because it is currently best among tested controllers.

7. Should hybrid transmission be modified only after arm-only works?
- Yes. Arm-only is cleaner and should be stabilized first; then reintroduce hybrid transmission/sync effects step-by-step.

## Key Arm-only Results
- pure_jtf_task_vsd: `rms=0.2888`, `max=0.4385`, `final=0.4281`
- ik_joint_space_vsd: `rms=0.3220`, `max=0.5128`, `final=0.5123`
- dls_resolved_rate_joint_torque: `rms=0.2952`, `max=0.4518`, `final=0.4508`

## Files
- controller comparison: `debug_outputs/arm_only_debug/controller_comparison.csv`
- bias impulse: `debug_outputs/arm_only_debug/bias_compensated_impulse.csv`
- bias pose hold: `debug_outputs/arm_only_debug/bias_pose_hold.csv`
- bias sign check: `debug_outputs/arm_only_debug/bias_sign_check.csv`
