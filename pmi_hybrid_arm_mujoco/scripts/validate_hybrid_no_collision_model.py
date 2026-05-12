#!/usr/bin/env python3
"""pmi_hybrid_no_collision.xml: 접촉·메시 시각화·관절/사이트 점검."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import mujoco as mj
import numpy as np

from utils.mujoco_helpers import PKG_ROOT, joint_id

HYBRID_MODEL_XML = PKG_ROOT / "models" / "pmi_hybrid_no_collision.xml"
OUT_DIR = PKG_ROOT / "debug_outputs" / "hybrid_no_collision"

JNT_NAMES = ["jnt1", "jnt2", "jnt3", "jnt4"]
ACT_NAMES = ["q1_act", "q2_act", "q3_act", "q4_act"]
RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=float)
INITIAL_ACTUATOR_RAD = np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=float)
Q_JNT_INIT = RATIOS * INITIAL_ACTUATOR_RAD


def _set_consistent_pose(model: mj.MjModel, data: mj.MjData) -> None:
    """등식 만족: q_jnt_i = ratio_i * q_act_i."""
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    for i, an in enumerate(ACT_NAMES):
        aid = joint_id(model, an)
        jid = joint_id(model, JNT_NAMES[i])
        qa = float(INITIAL_ACTUATOR_RAD[i])
        data.qpos[int(model.jnt_qposadr[aid])] = qa
        data.qpos[int(model.jnt_qposadr[jid])] = float(Q_JNT_INIT[i])
    mj.mj_forward(model, data)


def _check_mesh_geoms_xml(xml_text: str) -> tuple[bool, list[str]]:
    lines = []
    ok = True
    for m in re.finditer(r"<geom\b[^>]*>", xml_text, re.I):
        tag = m.group(0)
        if 'type="mesh"' not in tag and "type='mesh'" not in tag:
            continue
        c0 = "contype=" in tag and re.search(r'contype\s*=\s*"0"', tag)
        ca0 = "conaffinity=" in tag and re.search(r'conaffinity\s*=\s*"0"', tag)
        if not (c0 and ca0):
            ok = False
            lines.append(f"mesh geom missing contype=0/conaffinity=0: {tag[:120]}...")
    return ok, lines


def run_model_validation(verbose: bool = False) -> dict:
    model = mj.MjModel.from_xml_path(str(HYBRID_MODEL_XML))
    data = mj.MjData(model)
    xml_text = HYBRID_MODEL_XML.read_text(encoding="utf-8")

    mesh_xml_ok, mesh_xml_msgs = _check_mesh_geoms_xml(xml_text)

    mesh_model_ok = True
    mesh_model_msgs: list[str] = []
    for gi in range(model.ngeom):
        if int(model.geom_type[gi]) != int(mj.mjtGeom.mjGEOM_MESH):
            continue
        ct = int(model.geom_contype[gi])
        ca = int(model.geom_conaffinity[gi])
        if ct != 0 or ca != 0:
            mesh_model_ok = False
            mesh_model_msgs.append(
                f"geom id={gi} name={mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, gi)!r} contype={ct} conaffinity={ca}"
            )

    _set_consistent_pose(model, data)
    ncon0 = int(data.ncon)
    qfc = np.array(data.qfrc_constraint, dtype=float)
    qfc_max = float(np.max(np.abs(qfc))) if qfc.size else 0.0

    jid_exist = []
    for n in JNT_NAMES + ACT_NAMES:
        jid_exist.append((n, mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)))

    site_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "end_effector")

    dof_rows = []
    for n in JNT_NAMES + ACT_NAMES:
        jid = joint_id(model, n)
        dof_rows.append(f"| {n} | jid={jid} | qposadr={int(model.jnt_qposadr[jid])} | dofadr={int(model.jnt_dofadr[jid])} |")

    tau_consistency = []
    for i in range(4):
        jid = joint_id(model, JNT_NAMES[i])
        aid = joint_id(model, ACT_NAMES[i])
        qj = float(data.qpos[int(model.jnt_qposadr[jid])])
        qa = float(data.qpos[int(model.jnt_qposadr[aid])])
        exp = float(RATIOS[i]) * qa
        tau_consistency.append(float(abs(qj - exp)))

    tau_max_err = max(tau_consistency) if tau_consistency else 0.0

    result = {
        "mesh_xml_ok": mesh_xml_ok,
        "mesh_xml_msgs": mesh_xml_msgs,
        "mesh_model_ok": mesh_model_ok,
        "mesh_model_msgs": mesh_model_msgs,
        "ncon_initial": ncon0,
        "qfrc_constraint_max_abs": qfc_max,
        "joint_ids_ok": all(i >= 0 for _, i in jid_exist),
        "site_end_effector_ok": site_id >= 0,
        "equality_pose_max_abs_err_jnt_vs_ratio_act": tau_max_err,
        "dof_table_md": "\n".join(dof_rows),
        "nv": int(model.nv),
        "nu": int(model.nu),
        "ngeom": int(model.ngeom),
    }
    if verbose:
        print(result)
    return result


def write_validation_report(path: Path, r: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    mesh_ok = r["mesh_xml_ok"] and r["mesh_model_ok"]
    ncon_ok = r["ncon_initial"] == 0
    # 등식이 있으면 정지 상태에서도 qfrc_constraint 가 0이 아닐 수 있음(반력 분담)
    qfc_note = (
        "이 모델은 `equality` 관절 4개가 있어, 중력·관성 없이도 "
        "제약 반력 때문에 `qfrc_constraint` 벡터가 완전히 0이 아닐 수 있습니다. "
        "접촉 부재 여부는 `ncon` 으로 판단합니다."
    )
    lines = [
        "# Hybrid no-collision model validation",
        "",
        f"- MJCF: `{HYBRID_MODEL_XML.relative_to(PKG_ROOT)}`",
        f"- `nv={r['nv']}`, `nu={r['nu']}`, `ngeom={r['ngeom']}`",
        "",
        "## Checks",
        "",
        f"| Check | Result |",
        f"|------|--------|",
        f"| All STL mesh geoms contype/conaffinity=0 (XML scan) | {'PASS' if r['mesh_xml_ok'] else 'FAIL'} |",
        f"| All mesh geoms contype/conaffinity=0 (compiled model) | {'PASS' if r['mesh_model_ok'] else 'FAIL'} |",
        f"| `data.ncon` at initial consistent pose | {r['ncon_initial']} ({'PASS' if ncon_ok else 'FAIL'}) |",
        f"| Equality `jnt ≈ ratio*q_act` (forward, abs err) | max={r['equality_pose_max_abs_err_jnt_vs_ratio_act']:.3e} ({'PASS' if r['equality_pose_max_abs_err_jnt_vs_ratio_act'] < 1e-5 else 'WARN'}) |",
        f"| Joints jnt*, q*_act exist | {'PASS' if r['joint_ids_ok'] else 'FAIL'} |",
        f"| Site `end_effector` | {'PASS' if r['site_end_effector_ok'] else 'FAIL'} |",
        f"| `max abs qfrc_constraint` after forward | {r['qfrc_constraint_max_abs']:.6g} |",
        "",
        "### Note: qfrc_constraint",
        qfc_note,
        "",
        "### DOF table",
        "",
        "| joint | ids |",
        "|-------|-----|",
    ]
    for row in r["dof_table_md"].split("\n"):
        lines.append(row)
    if r["mesh_xml_msgs"]:
        lines.extend(["", "### XML mesh warnings", *[f"- {m}" for m in r["mesh_xml_msgs"]]])
    if r["mesh_model_msgs"]:
        lines.extend(["", "### Compiled model mesh warnings", *[f"- {m}" for m in r["mesh_model_msgs"]]])
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR)
    args = ap.parse_args()
    out = Path(args.out_dir)
    r = run_model_validation()
    write_validation_report(out / "model_validation_report.md", r)
    print(f"Wrote {out / 'model_validation_report.md'}")


if __name__ == "__main__":
    main()
