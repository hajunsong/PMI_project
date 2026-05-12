#!/usr/bin/env python3
"""Arm-only torque 디버그 MJCF 접촉·바이어스·패시브·제약 역량 진단."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import mujoco as mj
import numpy as np

from utils.mujoco_helpers import PKG_ROOT

MODEL_REL = Path("models/pmi_arm_only_torque_debug.xml")
JOINT_NAMES = ["jnt1", "jnt2", "jnt3", "jnt4"]
OUT_REPORT = PKG_ROOT / "debug_outputs" / "arm_only_physics" / "contact_diagnostics_report.md"


def _geom_name(model: mj.MjModel, gid: int) -> str:
    if gid < 0:
        return "(world)"
    nm = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, gid)
    bid = int(model.geom_bodyid[gid])
    bnm = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, bid) or f"body{bid}"
    return nm if nm else f"{bnm}:geom[{gid}]"


def _joint_dof_addrs(model: mj.MjModel, names: list[str]) -> np.ndarray:
    adr = []
    for n in names:
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)
        if jid < 0:
            raise RuntimeError(f"joint not found: {n}")
        adr.append(int(model.jnt_dofadr[jid]))
    return np.array(adr, dtype=np.int32)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", type=Path, default=None, help="MJCF path (default: models/pmi_arm_only_torque_debug.xml)")
    ap.add_argument("--q", type=float, nargs=4, default=[0.0, 0.0, 0.0, 0.0], metavar=("jnt1", "jnt2", "jnt3", "jnt4"))
    args = ap.parse_args()

    mp = Path(args.model) if args.model else PKG_ROOT / MODEL_REL
    if not mp.is_absolute():
        mp = PKG_ROOT / mp

    OUT_REPORT.parent.mkdir(parents=True, exist_ok=True)

    model = mj.MjModel.from_xml_path(str(mp))
    data = mj.MjData(model)

    dof = _joint_dof_addrs(model, JOINT_NAMES)
    for i, name in enumerate(JOINT_NAMES):
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
        adr_q = int(model.jnt_qposadr[jid])
        data.qpos[adr_q] = float(args.q[i])

    data.qvel[:] = 0.0
    mj.mj_forward(model, data)

    lines: list[str] = []
    lines.append("# Arm-only contact / physics diagnostics")
    lines.append("")
    lines.append(f"- MJCF: `{mp.resolve()}`")
    lines.append(f"- Initial q (rad): {list(args.q)}")
    lines.append(f"- **ncon**: {data.ncon}")
    lines.append("")

    lines.append("## Contact pairs")
    lines.append("")
    if data.ncon == 0:
        lines.append("(No contacts)")
    else:
        lines.append("| idx | geom1 | geom2 | dist |")
        lines.append("|-----|-------|-------|------|")
        for i in range(data.ncon):
            c = data.contact[i]
            g1 = _geom_name(model, int(c.geom1))
            g2 = _geom_name(model, int(c.geom2))
            dist = float(c.dist)
            lines.append(f"| {i} | {g1} | {g2} | {dist:.6g} |")
    lines.append("")

    lines.append("## Per-joint generalized forces (after mj_forward)")
    lines.append("")
    lines.append("| joint | dof | qfrc_bias | qfrc_constraint | qfrc_passive | sum(excl applied) |")
    lines.append("|-------|-----|-----------|-----------------|--------------|-------------------|")
    for k, name in enumerate(JOINT_NAMES):
        d = int(dof[k])
        qb = float(data.qfrc_bias[d])
        qc = float(data.qfrc_constraint[d])
        qp = float(data.qfrc_passive[d])
        s = qb + qc + qp
        lines.append(f"| {name} | {d} | {qb:.8g} | {qc:.8g} | {qp:.8g} | {s:.8g} |")
    lines.append("")
    lines.append("Notes: `qfrc_applied` was zero during this diagnostic.")

    report = "\n".join(lines) + "\n"
    OUT_REPORT.write_text(report, encoding="utf-8")

    print(report)
    print(f"[saved] {OUT_REPORT.resolve()}")


if __name__ == "__main__":
    main()
