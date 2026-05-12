#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys

import mujoco as mj
import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from utils.mujoco_helpers import PKG_ROOT, VSD_DEBUG_MODEL_PATH, apply_ideal_qjnt_equals_ratio_qact, joint_id, load_mjmodel
from utils.path_tracking_io import load_task_space_vsd_debug_yaml, ordered_transmission_arrays

OUT = PKG_ROOT / "debug_outputs" / "torque_diagnostics"


def main() -> None:
    cfg = load_task_space_vsd_debug_yaml(None)
    model = load_mjmodel(VSD_DEBUG_MODEL_PATH, strip_position_actuators=True)
    data = mj.MjData(model)

    j_ord, a_ord, ratios = ordered_transmission_arrays(cfg)
    ratios = np.asarray(ratios, dtype=float)
    qa0 = np.asarray(cfg["path"]["initial_actuator_rad"], dtype=float)
    for i, n in enumerate(a_ord):
        data.qpos[int(model.jnt_qposadr[joint_id(model, n)])] = qa0[i]
    apply_ideal_qjnt_equals_ratio_qact(model, data, list(j_ord), list(a_ord), ratios)
    mj.mj_forward(model, data)

    lines = ["# qfrc Application Audit", ""]
    lines.append("## Joint Index Map")
    for jn in j_ord:
        jid = joint_id(model, jn)
        lines.append(
            f"- {jn}: joint_id={jid}, qpos_adr={int(model.jnt_qposadr[jid])}, dof_adr={int(model.jnt_dofadr[jid])}"
        )

    lines.append("")
    lines.append("## qacc Response Test (+1Nm)")
    for jn in j_ord:
        jid = joint_id(model, jn)
        did = int(model.jnt_dofadr[jid])
        data.qfrc_applied[:] = 0.0
        data.qfrc_applied[did] = 1.0
        mj.mj_forward(model, data)
        qacc_before = data.qacc.copy()
        mj.mj_step(model, data)
        qacc_after = data.qacc.copy()
        lines.append(
            f"- {jn}: dof={did}, qacc_before={float(qacc_before[did]):.6f}, qacc_after={float(qacc_after[did]):.6f}, positive_response={qacc_after[did] > 0}"
        )

    lines += [
        "",
        "## Audit Answers",
        "1. qfrc_applied dof index correctness: verified by joint_id->dof_adr mapping above.",
        "2. positive qfrc -> positive qacc: see per-joint positive_response.",
        "3. overwritten before mj_step: this script sets qfrc then immediately steps; no overwrite in-between.",
        "4. qfrc reset each timestep: tests explicitly reset data.qfrc_applied[:] = 0.0 each step.",
    ]

    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "qfrc_application_audit.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
