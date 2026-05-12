# URDF inspection (legacy): `pmi_description.urdf`

**Current hybrid sim:** use **`ros_ws/pmi_description2/urdf/pmi_description2.urdf`** with mimic **q*_jnt ← q*_act** (belt q1; cable q2–4). MJCF `pmi_cable_mujoco/models/pmi_cable_arm.xml` mirrors that file; MuJoCo arm joints are named `jnt1..jnt4` (= URDF `q1_jnt..q4_jnt`).

---

## Older file — `pmi_description.urdf`

## Parsed structure

| Link       | Parent (implicit / joint) | Mass [kg] | Joint child |
|-----------|---------------------------|-----------|-------------|
| base_link | world                     | 2.4727    | —           |
| link1     | base_link ← `jnt1`        | 10.01235  | ✓           |
| link2     | link1 ← `jnt2`          | 10.43914  | ✓           |
| link3     | link2 ← `jnt3`          | 10.34065  | ✓           |
| link4     | link3 ← `jnt4`          | 7.01417   | ✓           |

## Revolute joints

| Joint | Parent → Child | Origin xyz | Origin rpy | Axis | Limits [rad] | Effort | Vel limit |
|-------|----------------|------------|------------|------|--------------|--------|-----------|
| jnt1  | base_link → link1 | 0 0 0 | π 0 0 | 0 0 1 | [-3.1416, 3.1416] | 10 | 100 |
| jnt2  | link1 → link2 | 0 0 -0.22 | 1.5708 0 0 | 0 0 1 | [-1.5709, 1.5709] | 10 | 100 |
| jnt3  | link2 → link3 | 0 -0.23 0 | 0 0 -1.5708 | 0 0 1 | [-1.5709, 1.5709] | 10 | 100 |
| jnt4  | link3 → link4 | 0.23 0 0 | -π 0 0 | 0 0 1 | [-1.5709, 1.5709] | 10 | 100 |

All joints use axis **z** (`0 0 1`) expressed in the joint frame defined by the joint origin.

## Inertial parameters

Each link has `<inertial>` with COM `xyz`, `mass`, and full inertia tensor `ixx, ixy, ixz, iyy, iyz, izz` at that COM — suitable for MuJoCo `fullinertia` after reordering to MuJoCo’s `ixx, iyy, izz, ixy, ixz, iyz`.

## Mesh references

URDF references:

- `package://pmi_description/meshes/base_link.STL`
- `package://pmi_description/meshes/link1.STL`
- … `link4.STL`

### Issues for MuJoCo / this repo

1. **STL files are not present** under `ros_ws/pmi_description/` in this workspace (no `meshes/` directory). Loading MJCF with those paths would fail unless meshes are added or paths are fixed.
2. **`package://` URIs** are ROS-specific. MuJoCo expects filesystem paths or assets relative to `meshdir`.
3. **SolidWorks exporter** sometimes produces degenerate collision meshes; MuJoCo’s convex decomposition may warn — mitigated here by using primitive collision geoms in `pmi_cable_arm.xml`.

The MJCF models (`models/pmi_cable_arm.xml`, `pmi_cable_arm_position.xml`) set **`meshdir="../../ros_ws/pmi_description3/meshes"`** (CAD STLs from `pmi_description3`; arm `link*.STL`, actuators `q*_motor.STL`). Joint origins follow **`pmi_description3.urdf`**; link masses/inertias used in simulation still match the heavier **`pmi_description2` / PMI_Server** lumped model unless you intentionally retune.

## Minor notes

- Joint `effort="10"` is quite low vs link masses (~10 kg); simulation may need larger actuator limits for aggressive tracking — overridden in MJCF actuators with conservative torque bounds and clipping in Python.
- Blank `<material name="">` in URDF is harmless.
