"""URDF를 MuJoCo로 로드하고 RL용 옵션(equality, 모터, 접촉 비활성)을 적용."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from pmi_mujoco_rl.paths import pmi_description_dir, pmi_urdf_path


ROS_PACKAGE_PREFIX = "package://pmi_description/"

# URDF mimic: q*_jnt 가 q*_act 에 연동 (polycoef: q_jnt - mult*q_act = 0)
MIMIC_PAIRS: tuple[tuple[str, str, float], ...] = (
    ("q1_act", "q1_jnt", 0.53333),
    ("q2_act", "q2_jnt", 6.66667),
    ("q3_act", "q3_jnt", 3.33333),
    ("q4_act", "q4_jnt", 3.33333),
)

ACTUATED_JOINTS: tuple[str, ...] = ("q1_act", "q2_act", "q3_act", "q4_act")


@dataclass(frozen=True)
class LoadOptions:
    timestep: float = 0.001
    integrator: str = "RK4"
    """STL 메시 기본 자세에서 자기 충돌이 나기 쉬우므로, 첫 RL 실험은 꺼 두는 것을 권장."""
    disable_collision: bool = True
    mimic_equalities: bool = True
    add_actuators: bool = True
    motor_gear: float = 1.0
    ctrl_range: tuple[float, float] = (-1.0, 1.0)
    # 초기 RL 튜닝용 기본값; 실제 역학은 예: (0.0, 0.0, -9.81).
    gravity: tuple[float, float, float] = (0.0, 0.0, 0.0)


def _read_resolved_urdf_text() -> str:
    text = pmi_urdf_path().read_text(encoding="utf-8")
    desc = pmi_description_dir().resolve().as_posix()
    return text.replace(ROS_PACKAGE_PREFIX, desc + "/")


def _inject_before_worldbody(xml: str, fragment: str) -> str:
    return xml.replace("<worldbody>", fragment + "\n  <worldbody>", 1)


def _build_mjcf_xml(base_xml: str, options: LoadOptions) -> str:
    xml = base_xml
    opt_line = (
        f'  <option timestep="{options.timestep}" '
        f'integrator="{options.integrator}" '
        f'gravity="{options.gravity[0]} {options.gravity[1]} {options.gravity[2]}"/>\n'
    )
    xml = xml.replace("<compiler angle=\"radian\"/>", "<compiler angle=\"radian\"/>\n" + opt_line, 1)

    if options.disable_collision:
        xml = _inject_before_worldbody(
            xml,
            '  <default>\n    <geom contype="0" conaffinity="0"/>\n  </default>',
        )

    if options.mimic_equalities:
        lines = [
            "  <equality>",
        ]
        for act, jnt, mult in MIMIC_PAIRS:
            lines.append(
                f'    <joint joint1="{act}" joint2="{jnt}" '
                f'solref="0.005 1" solimp="0.99 0.999 0.0001" '
                f'polycoef="0 {-mult} 1"/>'
            )
        lines.append("  </equality>")
        xml = xml.replace("</mujoco>", "\n".join(lines) + "\n</mujoco>", 1)

    return xml


def load_pmi_model(options: LoadOptions | None = None) -> mujoco.MjModel:
    """URDF에서 `MjModel` 생성 (모터 4개: q1_act … q4_act)."""
    opts = options or LoadOptions()
    urdf_resolved = _read_resolved_urdf_text()
    base = mujoco.MjSpec.from_string(urdf_resolved)
    xml = _build_mjcf_xml(base.to_xml(), opts)
    spec = mujoco.MjSpec.from_string(xml)

    if opts.add_actuators:
        lo, hi = opts.ctrl_range
        for jname in ACTUATED_JOINTS:
            act = spec.add_actuator()
            act.name = f"motor_{jname}"
            act.trntype = mujoco.mjtTrn.mjTRN_JOINT
            act.target = jname
            act.set_to_motor()
            act.ctrlrange[:] = [lo, hi]
            act.gear[:] = [opts.motor_gear]

    return spec.compile()


def joint_qpos_indices(model: mujoco.MjModel, joint_name: str) -> np.ndarray:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if jid < 0:
        raise ValueError(f"joint not found: {joint_name}")
    adr = model.jnt_qposadr[jid]
    width = 1 if model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_FREE else 7
    return np.arange(adr, adr + width)


def ctrl_indices_for_actuators(model: mujoco.MjModel) -> np.ndarray:
    """추가한 모터 액추에이터의 ctrl 인덱스 순서 (ACTUATED_JOINTS 순)."""
    idx = []
    for jname in ACTUATED_JOINTS:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"motor_{jname}")
        if aid < 0:
            raise RuntimeError("모터 액추에이터가 없습니다. add_actuators=True 인지 확인하세요.")
        idx.append(aid)
    return np.array(idx, dtype=np.int32)


def format_model_summary(model: mujoco.MjModel) -> str:
    """URDF→MuJoCo 반영 여부를 빠르게 볼 때 사용 (메시·관절 이름 등)."""
    lines: list[str] = [
        f"nbody={model.nbody} njnt={model.njnt} ngeom={model.ngeom} "
        f"nmesh={model.nmesh} nu={model.nu} neq={model.neq}",
        "joints:",
    ]
    for i in range(model.njnt):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        lines.append(f"  [{i}] {nm}")
    lines.append("meshes:")
    for i in range(model.nmesh):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, i)
        lines.append(f"  [{i}] {nm}")
    return "\n".join(lines)
