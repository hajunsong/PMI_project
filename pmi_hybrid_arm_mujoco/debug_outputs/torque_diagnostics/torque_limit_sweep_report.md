# Torque Limit Sweep Report

## dls_resolved_rate_joint_torque
- tau=5: rms=0.294862, max=0.451504, final=0.450785, sat=501
- tau=10: rms=0.295194, max=0.451228, final=0.450763, sat=475
- tau=20: rms=0.295098, max=0.450798, final=0.450134, sat=440
- tau=50: rms=0.295224, max=0.451375, final=0.450315, sat=380
- tau=100: rms=0.295410, max=0.451554, final=0.450329, sat=0
- tau=200: rms=0.295410, max=0.451554, final=0.450329, sat=0

## ik_joint_space_vsd
- tau=5: rms=0.306796, max=0.481801, final=0.478199, sat=501
- tau=10: rms=0.322011, max=0.512764, final=0.512549, sat=487
- tau=20: rms=0.322021, max=0.512775, final=0.512114, sat=478
- tau=50: rms=0.322031, max=0.512584, final=0.512280, sat=446
- tau=100: rms=0.322015, max=0.512526, final=0.512222, sat=415
- tau=200: rms=0.322006, max=0.512563, final=0.512309, sat=369

## pure_jtf_task_vsd
- tau=5: rms=0.293768, max=0.462272, final=0.462053, sat=501
- tau=10: rms=0.294156, max=0.462561, final=0.462180, sat=302
- tau=20: rms=0.294143, max=0.462544, final=0.462285, sat=0
- tau=50: rms=0.294143, max=0.462544, final=0.462285, sat=0
- tau=100: rms=0.294143, max=0.462544, final=0.462285, sat=0
- tau=200: rms=0.294143, max=0.462544, final=0.462285, sat=0

## Answers
1. Does increasing tau_limit reduce RMS position error?
- Mixed; inspect per-controller trends above.
2. Does increasing tau_limit reduce final position error?
- Mixed; inspect per-controller trends above.
3. At what tau_limit does performance stop improving?
- Around each controller's best-rms tau shown below.
4. Is 10 Nm clearly too low?
- Check tau=10 vs higher limits for each controller in table.
5. Which controller benefits most from higher torque limit?
- Controller with largest rms drop from tau=10 to its best tau.
- best(dls_resolved_rate_joint_torque) at tau=5 with rms=0.294862
- best(ik_joint_space_vsd) at tau=5 with rms=0.306796
- best(pure_jtf_task_vsd) at tau=5 with rms=0.293768
