# PMI hybrid transmission simulation (belt q1 + cable q2–q4)

Kinematics and inertias follow **`ros_ws/pmi_description2/urdf/pmi_description2.urdf`** (`models/pmi_cable_arm.xml`): **q1_act↔q1_jnt** timing belt/gear; **q2–4_act↔q2–4_jnt** wire rope + pulleys (rigid mimic = PMI_Server `kGear` in MJCF `<equality>`).

MuJoCo arm joint names **jnt1..jnt4** match URDF **q1_jnt..q4_jnt**. Python adds parasitic dynamics **in series** with the torque command (cable path does **not** add a separate `tau_cable` on top of VSD torque):

| URDF / MuJoCo | Mechanism | Nonlinear model |
|---------------|------------|-------------------|
| **q1_jnt / jnt1** | Belt / gear | Efficiency, damping, Coulomb, backlash in joint space (`transmission/belt_model.py`). **No** cable Bouc–Wen. |
| **q2–4_jnt / jnt2–4** | Antagonistic cable | Motor-side torque command → two tendon tensions → each passes `CableTendonTransmission` (friction, stretch elasticity, Bouc–Wen, tension dead-zone, clip, optional lag, **stretch backlash**). Net joint torque `r_joint * (T_plus_out - T_minus_out)` (`transmission/antagonistic_cable_joint.py`, `transmission/cable_transmission.py`). |

Torque flow (conceptual):

- **Control:** `tau_joint_cmd = tau_vsd + lambda ⊙ tau_residual` (4-vector, joint space for the command intent).
- **jnt1:** `tau_joint_delivered[0] = tau_drive + tau_belt_effect` with `tau_drive = tau_joint_cmd[0]` (`belt_model`).
- **jnt2–4:** `tau_act_cmd = tau_joint_cmd[1:4] ⊙ gear` (MuJoCo motor shaft), then `tau_joint_delivered[1:4] = AntagonisticCableStack.transmit(tau_act_cmd, q, qdot)` — cable **filters** torque to the joint; no additive `tau_cable_effect`.

Observation Bouc–Wen states: **`z_cable` length 3** (plus-cable `z` for jnt2, jnt3, jnt4).

Legacy additive cable helper: `transmission/cable_model.py` (not used in the default env path).

## PMI_Server 동일 기구 / 경로 (Python)

- ``kinematics/pmi_chain.py`` — ``pmi_kinematics`` FK + 5×4 task Jacobian (EE xyz + roll/pitch).
- ``planning/task_path_planner.py`` — ``PMI_Server`` ``PathPlanner``: 퀸틱 세그먼트, 사전 IK로 경로 스냅, 재생 시 샘플마다 IK 체인.
- ``controllers/waypoint_trajectory.py`` — 위 플래너로 ``(q_des, qdot_des)`` 생성; ``configs/control_params_waypoints.yaml`` 참고.
- 환경에서 ``trajectory.mode: waypoints_xyz`` 이면 구동축 초기각 ``initial_actuator_rad`` + 웨이포인트 ``t,x,y,z`` 로 관절 참조 추종 (관절 공간 VSD는 기존과 동일).

## Layout

```
kinematics/       # PMI arm FK / Jacobian (matches pmi_kinematics + server IK)
planning/         # TaskPathPlanner (PathPlanner port)
transmission/     # belt_model, cable_transmission, antagonistic_cable_joint, hysteresis, backlash, hybrid_joint_torque, randomization; cable_model (legacy)
configs/
  belt_params.yaml    # jnt1 + belt randomization ranges
  cable_params.yaml   # jnt2–4 antagonistic + tendon params + randomization
  control_params.yaml # Kp/Kd, tau_saturation, lambda_residual[4], trajectory
  control_params_waypoints.yaml  # example: waypoints_xyz + initial_actuator_rad
models/pmi_cable_arm.xml
envs/pmi_cable_arm_env.py
scripts/
```

## Install

```bash
cd pmi_cable_mujoco
pip install -r requirements.txt
```

## Scripts

| Script | Role |
|--------|------|
| `python scripts/demo_position_path_follow.py` | Position actuator demo for `control_params_waypoints.yaml`; shows requested waypoints, reachable IK path, desired EE, and actual EE in MuJoCo viewer. |
| `python scripts/test_load_model.py` | Load MJCF, print structure, uncontrolled rollout. |
| `python scripts/test_belt_joint.py` | **jnt1 only**: open-loop sine on `tau_drive` + belt effect; plots `tau_belt_jnt1` components. |
| `python scripts/test_cable_nonlinearity.py` | **jnt2–4 only**: antagonistic stack + MuJoCo; hysteresis-style plots (`T_plus_out` vs `x_plus`, `z_plus`). |
| `python scripts/test_vsd_control.py` | Full arm VSD + hybrid transmission; plots joint cmd vs delivered, tendon tensions, cable diagnostics. |
| `python scripts/test_hybrid_transmission_env.py` | Asserts belt on axis 1, cable stack on 2–4, `z_cable.shape==(3,)`, per-joint `tau_joint_transmitted` matches `info["tau_joint_delivered"]`. |
| `python scripts/test_env_randomization.py` | Prints independent belt + cable randomized parameters. |
| `python scripts/plot_vsd_torque_path_desired_vs_ee.py` | EE desired vs actual PNG; with transmission on, also saves `<output stem>_cable.png` (torque, tensions, stretch, z, flags). |

## Gymnasium API

- **Action:** 4-D normalized residual torque → scaled by `tau_saturation` (same as before).
- **Obs:** `[q, qdot, q_des, qdot_des, err, prev_action]` + optional **`z_cable` (3,)**.
- **`step` info keys:** `tau_vsd`, `tau_residual` (raw scaled-from-action), `lambda_tau_residual`, `tau_belt_jnt1`, `tau_joint_delivered`, `cable_transmission` (nested `per_joint` diagnostics: `T_plus_cmd`, `T_minus_cmd`, `T_plus_out`, `T_minus_out`, `tau_joint_transmitted`, `torque_loss_joint`, `x_plus` / tendon `plus` dict with `x`, `x_elastic`, `xdot`, `z`, `deadzone_active`, `backlash_active`, …), `z_cable`.

Environment kwargs:

- `randomize_transmission=True|False` — toggles **both** YAML `randomization.enabled` flags.
- `belt_config_path`, `cable_config_path`, `control_config_path`.
- Legacy: `randomize_cable` only toggles cable YAML randomization.

## SAC / SB3

Same pattern as before: wrap `PMICableArmEnv`, optional `VecNormalize`, tune `lambda_residual` vector in `control_params.yaml`.
