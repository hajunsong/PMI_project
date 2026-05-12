# IK Joint Tracking After Sign Fix

- torque_sign used: [-1, 1, 1, 1]
- best before: {'controller_mode': 'computed_torque', 'lambda_task': '0.3', 'duration': '1.0', 'tau_limit': '20.0', 'final_ee_pos_err': 0.48837942478470087, 'rms_ee_pos_err': 0.3253128432719195, 'rms_joint_err': 1.1003771630417682, 'saturation_steps': 470.0}
- best after: {'controller_mode': 'computed_torque', 'lambda_task': '0.3', 'duration': '1.0', 'tau_limit': '20.0', 'final_ee_pos_err': 0.48817334421454567, 'rms_ee_pos_err': 0.32521877466436916, 'rms_joint_err': 1.1002271194707327, 'saturation_steps': 501.0}

## Comparison
- final EE error: 0.488379 -> 0.488173
- RMS EE error: 0.325313 -> 0.325219
- RMS joint error: 1.100377 -> 1.100227
- saturation steps: 470 -> 501

- Result: sign fix does not resolve large tracking error; baseline remains unstable for path tracking criteria.
