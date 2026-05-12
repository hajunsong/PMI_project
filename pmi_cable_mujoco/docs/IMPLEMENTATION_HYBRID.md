# Hybrid belt / cable implementation (serial transmission)

Canonical robot / mimic layout: **`ros_ws/pmi_description2/urdf/pmi_description2.urdf`** — **q1_jnt** mimics **q1_act** (belt); **q2–4_jnt** mimic **q2–4_act** (cable + pulley). MJCF uses short names **jnt1..jnt4** for the arm hinges.

## Transmission split

- `belt_joint_names = ["jnt1"]`  (URDF **q1_jnt**)
- `cable_joint_names = ["jnt2", "jnt3", "jnt4"]`  (URDF **q2_jnt..q4_jnt**)

## jnt1 — belt / gear (`transmission/belt_model.py`)

Parasitic effect **added to the same-axis drive torque** (joint 1 only):

- Efficiency loss as `(efficiency - 1) * tau_drive` with `tau_drive = tau_joint_cmd[0]` from the hybrid applier.
- Viscous + linear damping on `qdot` with gear-ratio scaling on reflected term.
- Coulomb friction `tanh(qdot / v_eps)`.
- Backlash path: `DeadzoneBacklash` + elastic term in joint space (belt-style gap — **not** the cable stretch stack).

No Bouc–Wen on jnt1.

Randomization: `configs/belt_params.yaml` → `randomization` block.

## jnt2–jnt4 — antagonistic cable (`transmission/antagonistic_cable_joint.py` + `cable_transmission.py`)

1. Joint-space command from controller: `tau_joint_cmd[i] = tau_vsd[i] + lambda_i * tau_residual[i]` (same assembly as before for the **command** vector).
2. MuJoCo motors act on **actuator** coordinates: `tau_act_cmd[i] = tau_joint_cmd[i] * gear_i` for `i ∈ {2,3,4}` (`kinematics.pmi_chain.MOTOR_TO_JOINT_GEAR`).
3. Per joint, **two** commanded tendon tensions (preload + positive/negative torque split), each processed by `CableTendonTransmission.transmit(T_in, x, xdot, dt)`:
   - Optional first-order lag on `T_in`
   - Coulomb / viscous on `xdot`
   - Elastic term `k_stretch * x_elastic` where `x_elastic` comes from `StretchBacklash` on geometric stretch `x` (`transmission/backlash.py`)
   - Bouc–Wen internal force on `xdot`
   - Tension dead-zone and `[0, T_max]` clip
4. Joint torque: `tau_joint = r_joint * (T_plus_out - T_minus_out)`.

There is **no** separate `tau_cable` vector added to `tau_vsd + lambda*tau_res` for jnt2–4; the stack **replaces** the torque path from command to delivered torque for those axes.

`z_cable` in observations: plus-cable hysteresis state per joint (length 3).

Randomization: `configs/cable_params.yaml` → `randomization` block (shared with legacy keys where applicable). Antagonistic defaults / overrides under YAML `antagonistic:` (e.g. `T_preload`, `stretch_backlash_slack_rate`).

## Environment (`envs/pmi_cable_arm_env.py`)

Each inner step:

```text
tau_joint_cmd = tau_vsd + lambda ⊙ tau_residual
(tau_joint_delivered, diag) = apply_transmission_joint_torque(
    tau_joint_cmd, q, qd, q_des, qdot_des, dt, belt, cable_stack)
tau_joint_delivered ← clip to tau_saturation
→ actuator_torque_from_joint_torque → MuJoCo ctrl
```

`diag["cable_transmission"]` is stored in `info` for logging/plots (single `transmit` per sub-step; no double integration of hysteresis).

## Deprecated / legacy

- `transmission/cable_model.py`: older **additive** cable torque on top of VSD; not wired into `PMICableArmEnv` after the serial antagonistic refactor.
- The old `cable/` package was removed; use **`transmission/`** for new work.
