"""URDF를 MuJoCo로 로드하고 RL용 옵션(equality, 모터, 접촉 비활성)을 적용."""

from __future__ import annotations

import os
from contextlib import contextmanager
from dataclasses import dataclass

import mujoco
import numpy as np

from pmi_mujoco_rl.paths import pmi_description_dir, pmi_urdf_path


ROS_PACKAGE_PREFIX = "package://pmi_description/"

# 토크·VSD·RL 액추 목표(우선순위). URDF 변환 결과에 따라 실제 이름이 달라질 수 있음(예: jnt1..jnt4).
ACTUATED_JOINTS: tuple[str, ...] = ("q1_jnt", "q2_jnt", "q3_jnt", "q4_jnt")
FALLBACK_ACTUATED_JOINTS: tuple[str, ...] = ("jnt1", "jnt2", "jnt3", "jnt4")


@dataclass(frozen=True)
class LoadOptions:
    timestep: float = 0.001
    integrator: str = "RK4"
    """STL 메시 기본 자세에서 자기 충돌이 나기 쉬우므로, 첫 RL 실험은 꺼 두는 것을 권장."""
    disable_collision: bool = True
    add_actuators: bool = True
    motor_gear: float = 1.0
    ctrl_range: tuple[float, float] = (-1.0, 1.0)
    # 초기 RL 튜닝용 기본값; 실제 역학은 예: (0.0, 0.0, -9.81).
    gravity: tuple[float, float, float] = (0.0, 0.0, 0.0)


@contextmanager
def _chdir(path: str | os.PathLike[str]):
    """MuJoCo `MjSpec.to_xml()`가 메시를 basename으로만 열 때 cwd에 의존하므로, 메시 폴더로 잠시 이동."""
    prev = os.getcwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(prev)


def _read_resolved_urdf_text() -> str:
    text = pmi_urdf_path().read_text(encoding="utf-8")
    desc = pmi_description_dir().resolve().as_posix()
    return text.replace(ROS_PACKAGE_PREFIX, desc + "/")


def _inject_before_worldbody(xml: str, fragment: str) -> str:
    return xml.replace("<worldbody>", fragment + "\n  <worldbody>", 1)


def _build_mjcf_xml(base_xml: str, options: LoadOptions) -> str:
    xml = base_xml
    # URDF→MjSpec→to_xml() 시 메시가 basename만 남으므로, 작업 디렉터리와 무관하게 로드되도록 meshdir 고정.
    mesh_dir = (pmi_description_dir() / "meshes").resolve().as_posix()
    opt_line = (
        f'  <option timestep="{options.timestep}" '
        f'integrator="{options.integrator}" '
        f'gravity="{options.gravity[0]} {options.gravity[1]} {options.gravity[2]}"/>\n'
    )
    xml = xml.replace(
        '<compiler angle="radian"/>',
        f'<compiler angle="radian" meshdir="{mesh_dir}"/>\n' + opt_line,
        1,
    )

    if options.disable_collision:
        xml = _inject_before_worldbody(
            xml,
            '  <default>\n    <geom contype="0" conaffinity="0"/>\n  </default>',
        )

    return xml


def load_pmi_model(options: LoadOptions | None = None) -> mujoco.MjModel:
    """URDF에서 `MjModel` 생성 (모터 4개: q1_jnt … q4_jnt)."""
    opts = options or LoadOptions()
    urdf_resolved = _read_resolved_urdf_text()
    base = mujoco.MjSpec.from_string(urdf_resolved)
    mesh_dir = (pmi_description_dir() / "meshes").resolve()
    with _chdir(mesh_dir):
        roundtrip_xml = base.to_xml()
    xml = _build_mjcf_xml(roundtrip_xml, opts)
    spec = mujoco.MjSpec.from_string(xml)

    if opts.add_actuators:
        spec_joint_names = {str(j.name) for j in spec.joints}
        joint_targets = (
            ACTUATED_JOINTS
            if all(name in spec_joint_names for name in ACTUATED_JOINTS)
            else FALLBACK_ACTUATED_JOINTS
        )
        lo, hi = opts.ctrl_range
        for jname in joint_targets:
            act = spec.add_actuator()
            act.name = f"motor_{jname}"
            act.trntype = mujoco.mjtTrn.mjTRN_JOINT
            act.target = jname
            act.set_to_motor()
            act.ctrlrange[:] = [lo, hi]
            act.gear[:] = [opts.motor_gear]

    return spec.compile()


def actuated_joints_for_model(model: mujoco.MjModel) -> tuple[str, ...]:
    names = tuple(
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or ""
        for i in range(model.njnt)
    )
    if all(name in names for name in ACTUATED_JOINTS):
        return ACTUATED_JOINTS
    if all(name in names for name in FALLBACK_ACTUATED_JOINTS):
        return FALLBACK_ACTUATED_JOINTS
    raise RuntimeError(f"지원되지 않는 조인트 이름 구성: {names}")


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
    for jname in actuated_joints_for_model(model):
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
