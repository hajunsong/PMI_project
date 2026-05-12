# Task-space VSD Diagnostic Report

## 1. Model structure
- Raw XML nu=4 (position servos on q*_act if nu>0).
- Simulation model nu=0 (strip_position_actuators → torque via qfrc only).
- See `model_structure.txt` for joint/sensor/site listing.

## 2. ACTUATOR MODE CHECK
- **Raw XML:** `general` actuators with affine bias are **PD-style position actuators**; `ctrl` is typically desired joint coordinate (not torque).
- **Task-space VSD in scripts** applies torque through `data.qfrc_applied` on joint or actuator dofs, with `ctrl=0` on stripped model.
- If someone wrote torque into `data.ctrl` on the **raw** model, it would **not** behave like torque — mismatch guaranteed.
- `data.qfrc_actuator` is zero when nu=0; applied wrench shows up in `qfrc_applied`.

## 3. Initial state & transmission
- Expected q_jnt from ratio×q_act err norm: 0.000e+00 after Python sync.
- Transmission perturbation test: max |Δjnt|/Δqa ~ 0.000e+00  static err 0.000e+00
- q_act를 변동해도 jnt 연쇄가 없습니다(MuJoCo XML은 mimic/전달 제약 없음). 컨트롤러는 q_act가 jnt를 구동한다고 가정할 수 없으며 Python 동기화가 필요합니다.

## 4. Jacobian
- Shape (5, 4), rank 4, cond ~ 15.3
- Singular values: [1.82352136 0.32019616 0.24017765 0.11934078]
- Position Jacobian num vs MuJoCo hybrid xyz diff norm: 7.257e-11
**5D task / 4 joints:** J is 5×4; cannot span all R^5 — residual orientation/position tradeoff is expected.

## 5. Virtual work & sign
- Fx: |τ|=0.462  Δp=[ 0.00480439  0.00190495 -0.00540333]  inst_slack=0.00e+00
- Fy: |τ|=0.417  Δp=[ 0.00130574  0.00462488 -0.00518312]  inst_slack=0.00e+00
- Fz: |τ|=0.336  Δp=[ 0.00272726  0.0035188  -0.00503824]  inst_slack=0.00e+00
- Mr: |τ|=0.000  Δp=[ 0.00285843  0.00341684 -0.00529737]  inst_slack=0.00e+00
- Mp: |τ|=1.732  Δp=[ 0.01950957 -0.0095269  -0.00508394]  inst_slack=0.00e+00

## 6. VSD run (Part 6 CSV)
- Log: `diagnostic_log.csv`
- RMS pos err: 0.300662606795517, max: 0.4666778258101104
- Monotonic position error growth max consecutive steps (threshold 1e-5): 455

## 7. Control modes A/B/C
- Table: `mode_compare.csv`
  - {'mode': 'A_joint', 'torque_mode': 'joint_direct_debug', 'rms_pos': 0.21391741950268597, 'max_pos': 0.30118780268373413, 'rms_roll': 8.271297816328969e-06, 'rms_pitch': 0.0332767266358597, 'max_tau_j': 4.17927597189135, 'sat_j': 0, 'sat_a': 0, 'nan_stop': False}
  - {'mode': 'B_actuator', 'torque_mode': 'actuator', 'rms_pos': 0.5976775809807428, 'max_pos': 0.8471803521109259, 'rms_roll': 1.755594390328027, 'rms_pitch': 1.1641613161457665, 'max_tau_j': 10.0, 'sat_j': 279, 'sat_a': 0, 'nan_stop': False}
  - {'mode': 'C_current', 'torque_mode': 'current_ctrl', 'rms_pos': 0.21391741950268597, 'max_pos': 0.30118780268373413, 'rms_roll': 8.271297816328969e-06, 'rms_pitch': 0.0332767266358597, 'max_tau_j': 4.17927597189135, 'sat_j': 0, 'sat_a': 0, 'nan_stop': False}

## 8. Gain sweep
- `gain_sweep.csv`
  - {'gain_set': 'low', 'rms_pos': 0.19577870917728946, 'max_pos': 0.29951165188738604, 'rms_roll': 8.221861844361177e-06, 'rms_pitch': 0.06794496462637299, 'max_tau_j': 1.3023746506342924, 'sat_j': 0, 'stable': True}
  - {'gain_set': 'medium', 'rms_pos': 0.19309263291635834, 'max_pos': 0.29605831234210955, 'rms_roll': 8.191744056393617e-06, 'rms_pitch': 0.029256413568143312, 'max_tau_j': 4.17927597189135, 'sat_j': 0, 'stable': True}
  - {'gain_set': 'high', 'rms_pos': 0.19258043346073694, 'max_pos': 0.29541494264133433, 'rms_roll': 8.128564582818875e-06, 'rms_pitch': 0.03608289264657259, 'max_tau_j': 7.2980703107968905, 'sat_j': 0, 'stable': True}

## 9. Most likely failure causes (checklist)
- **XML suitable for torque in ctrl?** No — position actuators; use stripped model + qfrc (current workflow) or replace with torque actuators.
- **q_act mechanically coupled in MuJoCo?** No equality in MJCF; coupling is Python `apply_ideal_*`.
- **q_jnt = ratio×q_act enforced in physics?** Not automatically; only after sync step.
- **Jacobian correct?** Compare xyz rows vs MuJoCo; orientation rows use numerical hybrid in this codebase.
- **τ = JᵀF direction?** See Part 5 short impulse tests.
- **5D task vs 4DOF?** Yes, over-constrained subspace; expect non-zero residual unless gains/tasks aligned.
- **Orientation gains too high?** Can saturate or fight position — reduce K_rp / D_rp or task weights.
- **Clipping?** Check F_task and tau limits in config.
- **Torque into position actuators?** If nu>0 and ctrl used incorrectly — yes, fatal; use qfrc or strip actuators.

## 10. Recommended next fixes
- Keep simulation on **stripped** model for torque; never send torque through `ctrl` on position-actuator XML.
- Verify Python transmission sync each step matches deployed controller.
- Tune medium gains first; watch singular values — near rank loss directions explode force demand.