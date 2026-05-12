#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import mujoco as mj
import numpy as np

from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

MODEL = PKG_ROOT / "models" / "pmi_arm_only_torque_debug.xml"
OUT = PKG_ROOT / "debug_outputs" / "joint_servo_debug"
J = ["jnt1", "jnt2", "jnt3", "jnt4"]


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    model = load_mjmodel(MODEL, strip_position_actuators=True)
    data = mj.MjData(model)

    lines = ["# Joint Order Verification", "", f"- controller joint order: {J}"]
    q = np.zeros(4)
    qdot = np.zeros(4)
    qdes = np.zeros(4)
    lines.append(f"- q_actual order (constructed): {list(q)}")
    lines.append(f"- qdot order (constructed): {list(qdot)}")
    lines.append(f"- q_des order (constructed): {list(qdes)}")

    for jn in J:
        jid = joint_id(model, jn)
        lines.append(f"- {jn}: joint_id={jid}, qpos_adr={int(model.jnt_qposadr[jid])}, dof_adr={int(model.jnt_dofadr[jid])}")

    lines += ["", "## Per-joint torque routing check"]
    Kp = 30.0
    for i, jn in enumerate(J):
        e = np.zeros(4); e[i] = 0.1
        tau_pd = Kp * e
        qfrc = np.zeros(model.nv)
        dof = [int(model.jnt_dofadr[joint_id(model, n)]) for n in J]
        for k, d in enumerate(dof):
            qfrc[d] = tau_pd[k]
        nz = np.where(np.abs(qfrc) > 1e-12)[0].tolist()
        lines.append(f"- target {jn}: tau_pd={tau_pd.tolist()}, nonzero_qfrc_dof={nz}")

    lines += [
        "",
        "Answers:",
        "- q_des[jnt1..4] is applied to the same-order dof list by construction.",
        "- q_des and q_actual are generated with identical J order.",
    ]

    (OUT / "joint_order_verification.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
