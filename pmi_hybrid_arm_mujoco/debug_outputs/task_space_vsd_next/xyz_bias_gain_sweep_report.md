# XYZ + Bias Gain Sweep Report

- best gain by rms_pos: {'K_xyz': 120, 'D_xyz': 12, 'rms_pos': 0.2924499249545449, 'max_pos': 0.4584219195577765, 'final_pos_err': 0.022173028582688232, 'max_tau': 10.0, 'joint_limit_steps': 0, 'torque_saturation_steps': 20, 'stable': True}
- best gain by final_pos_err: {'K_xyz': 50, 'D_xyz': 8, 'rms_pos': 0.29309082091736455, 'max_pos': 0.458254671461075, 'final_pos_err': 0.020169745565909752, 'max_tau': 8.119725799363298, 'joint_limit_steps': 12, 'torque_saturation_steps': 0, 'stable': True}
- unstable gains: 0 cases
- whether increasing K helps: True (mean rms at K=10: 0.31203, K=120: 0.29264)
- damping too low/high hint: min mean-rms at D=2 (edge minima may indicate too low/high damping regime).
