#!/usr/bin/env python3
"""MJCF 재생성: ros_ws 원본 URDF → models/pmi_hybrid_arm.xml (meshdir 상대경로 포함)."""

from __future__ import annotations

import math
import re
from pathlib import Path

import mujoco as mj

PKG = Path(__file__).resolve().parents[1]
PROJECT = Path(__file__).resolve().parents[2]
URDF_SRC = PROJECT / "ros_ws" / "pmi_description3" / "urdf" / "pmi_description3.urdf"
MESH_ABS = PROJECT / "ros_ws" / "pmi_description3" / "meshes"
OUT_XML = PKG / "models" / "pmi_hybrid_arm.xml"


def preprocess_urdf_text(raw: str) -> str:
    text = raw.replace("package://pmi_description3/meshes/", "")
    text = re.sub(r"<mimic[^/]*/>\s*\n?", "", text)
    text = text.replace("<mass\n        value=\"0\" />", "<mass\n        value=\"0.031263\" />")
    text = text.replace(
        """<inertia
        ixx="0"
        ixy="0"
        ixz="0"
        iyy="0"
        iyz="0"
        izz="0" />""",
        """<inertia
        ixx="7.8696E-06"
        ixy="0"
        ixz="0"
        iyy="7.8696E-06"
        iyz="0"
        izz="7.8696E-06" />""",
    )
    return text


def main() -> None:
    if not URDF_SRC.is_file():
        raise SystemExit(f"원본 URDF 없음: {URDF_SRC}")
    spec = mj.MjSpec.from_string(preprocess_urdf_text(URDF_SRC.read_text(encoding="utf-8")))
    spec.meshdir = MESH_ABS.as_posix() + "/"
    spec.modelname = "pmi_hybrid_arm"
    spec.option.timestep = 0.002

    body = next(b for b in spec.bodies if b.name == "link4")
    quat_wxyz = [math.cos(-1.5709 / 2), 0.0, 0.0, math.sin(-1.5709 / 2)]
    body.add_site(name="end_effector", pos=[0.18, 0.0, 0.0], quat=quat_wxyz)

    for name in ["q1_act", "q2_act", "q3_act", "q4_act", "jnt1", "jnt2", "jnt3", "jnt4"]:
        spec.add_sensor(
            name=f"sens_{name}",
            type=mj.mjtSensor.mjSENS_JOINTPOS,
            objtype=mj.mjtObj.mjOBJ_JOINT,
            objname=name,
        )

    spec.add_sensor(
        name="ee_pos",
        type=mj.mjtSensor.mjSENS_FRAMEPOS,
        objtype=mj.mjtObj.mjOBJ_SITE,
        objname="end_effector",
    )

    for i in range(1, 5):
        act = spec.add_actuator(name=f"pos_q{i}", target=f"q{i}_act", trntype=mj.mjtTrn.mjTRN_JOINT)
        act.set_to_position(kp=200.0, kv=20.0, dampratio=-1)

    xml = spec.to_xml()
    rel = "../../ros_ws/pmi_description3/meshes/"
    xml = xml.replace(f'meshdir="{MESH_ABS.as_posix()}/"', f'meshdir="{rel}"')
    OUT_XML.parent.mkdir(parents=True, exist_ok=True)
    OUT_XML.write_text(xml, encoding="utf-8")
    mj.MjModel.from_xml_path(str(OUT_XML))
    print(f"[OK] wrote {OUT_XML.relative_to(PROJECT)}")


if __name__ == "__main__":
    main()
