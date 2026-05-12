#!/usr/bin/env python3
"""Load MJCF, print model structure, simulate 1000 steps with zero control."""

from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import mujoco

MODEL = ROOT / "models" / "pmi_cable_arm.xml"


def main() -> None:
    print("Loading:", MODEL)
    m = mujoco.MjModel.from_xml_path(os.fspath(MODEL))
    d = mujoco.MjData(m)

    print("\n--- Sizes ---")
    print(f"nq={m.nq} nv={m.nv} nu={m.nu} njnt={m.njnt} nbody={m.nbody} ngeom={m.ngeom}")

    print("\n--- Joints ---")
    for i in range(m.njnt):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
        print(f"  [{i}] {name} type={m.jnt_type[i]} qposadr={m.jnt_qposadr[i]} dofadr={m.jnt_dofadr[i]}")

    print("\n--- Bodies ---")
    for i in range(m.nbody):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)
        print(f"  [{i}] {name}")

    print("\n--- Actuators ---")
    for i in range(m.nu):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  [{i}] {name} trntype={m.actuator_trntype[i]} dyntype={m.actuator_dyntype[i]}")

    print("\n--- Simulate 1000 steps (ctrl=0) ---")
    mujoco.mj_resetData(m, d)
    for k in range(1000):
        mujoco.mj_step(m, d)
        if k == 0:
            print(f"first step qpos[:nq]={d.qpos.copy()}")

    print(f"final qpos={d.qpos.copy()}")
    print("done.")


if __name__ == "__main__":
    main()
