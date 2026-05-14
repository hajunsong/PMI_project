# VSD vs SAC residual: control mode inspection

This document summarizes how **position**, **orientation**, and **SAC residual** interact in the current stack (`envs/pmi_cable_residual_env.py`, IK/VSD helpers). No training or model edits were performed for this inspection.

---

## 1. Nominal controller: joint-space VSD or pure task-space xyz VSD?

- **Nominal control is joint-space PD (VSD-style) on the arm joints**, not a closed-loop operational-space wrench controller on Cartesian error.
- Per control step the environment computes **`q_des(t), q_dot_des(t)`** from a **quintic joint path** (`trajectory/joint_quintic.py`) built off **IK waypoint joint targets**.
- Nominal torque (before clipping / transmission):

  `tau_vsd = tau_bias + Kq * (q_des - q) + Dq * (qd_des - qd) + tau_orientation_soft`

  where `tau_orientation_soft` is **zero** when `controller.orientation_soft_mode` resolves to `none` (default / “xyz_only” behavior in config naming).

So: **primary regulation is in joint space**; there is **no** separate full **6-DoF task-space** VSD on `(x,y,z,roll,pitch,yaw)`.

---

## 2. IK: does `q_des` enforce roll = −π/2 and pitch = 0?

- Waypoint IK calls `solve_ik_task_mode(..., roll_des=ROLL_DES_IK=-π/2, pitch_des=PITCH_DES_IK=0, task_feas_mode="xyz", ...)`.
- In `kinematics/inverse_kinematics.py`, mode **`"xyz"`** builds a residual vector with **only the weighted position rows** (3 rows). **Roll and pitch targets are not included in the IK cost** for this mode.
- Therefore **`q_des` is the solution of position-only IK**. Any roll/pitch of the EE that follows is whatever the **4-DoF** arm happens to produce for that `q_des`, **not** an explicit roll/pitch IK solve.
- **Yaw** is **not** specified in IK (same as roll/pitch in `"xyz"` mode: orientation is not in the objective).

**Logging note:** `desired_roll_rad` / `desired_pitch_rad` in `info` are the **constant design references** (−π/2, 0). `ee_des_roll_rad_fk` / `ee_des_pitch_rad_fk` are the **FK orientation of `q_des`** (what the nominal joint path “expects” in attitude if the plant matched `q_des` exactly).

---

## 3. Task Jacobian for residual: xyz-only? Roll/pitch terms disabled for SAC?

- SAC action is **3-D** (`action_dim=3`), clamped in env init to **only 3**.
- Residual torque uses `compute_task_jacobian_mode(..., task_mode="xyz", ...)` and `joint_torque_from_task_force` → **`tau_res ∝ J_pos^T F` with 3 rows only**.
- **No residual roll/pitch moment** is applied by SAC in the current code path.

---

## 4. Optional soft orientation on the nominal path

- If `orientation_soft_mode` is `xyz_plus_roll_soft` / `soft_roll` or `xyz_plus_roll_pitch_soft` / `soft_roll_pitch`, the env adds **`tau_orientation_soft`** from Jacobian rows for **roll and optionally pitch** (`task_mode="xyz_roll_pitch"`), with small **P/D gains** (`orientation_soft_roll`, `orientation_soft_pitch`). This is **soft stabilization**, not a hard constraint.
- Default comparison runs typically keep this **off** (`none`), i.e. **nominal torque is not actively steering roll/pitch** beyond what joint tracking of `q_des` does.

---

## 5. Answers to the checklist

| Question | Answer |
| -------- | ------ |
| Nominal IK joint-space VSD or pure task-space xyz VSD? | **Joint-space VSD on `q_des`**; IK is **xyz-only** (`task_feas_mode="xyz"`). |
| If IK joint-space: roll=−π/2, pitch=0? | **Not in the IK objective** for `"xyz"` mode; constants are passed but **not** in the residual. |
| Yaw free? | **Yes** (not in IK objective for `"xyz"`). |
| Task-space VSD on EE xyz? | **No** separate Cartesian VSD; **position error** is formed in task space for observation/reward, **torque is joint PD (+ optional soft orientation)**. |
| SAC residual action | **`[Fx, Fy, Fz]` only**. |
| Residual roll/pitch moment? | **None** (only if future extension increases action/J rows). |
| Logged orientation? | **`ee_roll_rad`, `ee_pitch_rad`, `ee_yaw_rad`, errors vs fixed roll/pitch, `ee_omega_world_*`, FK-from-`q_des` rpy`** in `info` and comparison CSVs. |

---

## 6. Implication for “3D path oscillation”

Path plots are **xyz** of the actual site. **Oscillation there is not equivalent to “xyz-only control” alone** — the plant is **4-DoF**, IK is **xyz-only**, and **attitude can drift** without contradicting the nominal design. Diagnose coupling using the new **orientation time series and correlations** in `aggregate_metrics.md` / `plots/orientation/`.
