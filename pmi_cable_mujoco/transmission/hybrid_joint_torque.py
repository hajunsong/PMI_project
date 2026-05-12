"""하이브리드 전달: jnt1 벨트(기존), jnt2–4 길항 케이블(모터 토크 명령 → 관절 전달 토크).

관절 공간에서의 **명령** ``tau_joint_cmd`` (VSD+잔차+바이어스 등)에 대해:

- **jnt1**: ``tau_drive = tau_joint_cmd[0]`` 를 벨트 모델에 넣어 ``tau_joint_delivered[0] = tau_drive + tau_belt``.
- **jnt2–4**: ``tau_act_cmd = tau_joint_cmd[i] * gear_i`` (구동축 토크 명령) → 길항 케이블 스택으로
  ``tau_joint_delivered[i] = AntagonisticCableJoint.transmit(tau_act_cmd, q, qdot)``.

MuJoCo ``motor`` 는 구동축이므로 최종 ``tau_joint_delivered`` 를 ``actuator_torque_from_joint_torque`` 로 ``ctrl`` 에 넣는다.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np
import yaml

from kinematics.pmi_chain import MOTOR_TO_JOINT_GEAR

from .antagonistic_cable_joint import AntagonisticCableStack, build_antagonistic_stack_from_config
from .belt_model import BeltTransmissionModel, build_belt_model_from_config


def _load_yaml(path: Path) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def build_belt_cable_models(
    belt_cfg_path: Path,
    cable_cfg_path: Path,
    *,
    randomize: bool = False,
) -> Tuple[BeltTransmissionModel, AntagonisticCableStack]:
    """YAML에서 벨트 + 길항 케이블 스택 생성."""
    bcfg = _load_yaml(belt_cfg_path)
    ccfg = _load_yaml(cable_cfg_path)
    if not randomize:
        bcfg.setdefault("randomization", {})["enabled"] = False
        ccfg.setdefault("randomization", {})["enabled"] = False
    belt = build_belt_model_from_config(bcfg)
    cable = build_antagonistic_stack_from_config(ccfg)
    belt.reset()
    cable.reset()
    return belt, cable


def apply_transmission_joint_torque(
    tau_joint_cmd: np.ndarray,
    q: np.ndarray,
    qd: np.ndarray,
    q_des: np.ndarray,
    qdot_des: np.ndarray,
    dt: float,
    belt: Optional[BeltTransmissionModel],
    cable: Optional[AntagonisticCableStack],
) -> tuple[np.ndarray, Dict[str, Any]]:
    """
    ``tau_joint_cmd``: 관절 공간 명령 [N·m] (케이블 **가산 잔차 없음**).

    ``q_des``, ``qdot_des`` 는 jnt1 벨트 블록에만 사용 (기존 belt API).

    Returns
    -------
    tau_joint_delivered
        shape (4,)
    diag
        ``belt_diag`` (optional), ``cable_transmission`` (optional).
    """
    diag: Dict[str, Any] = {}
    out = np.asarray(tau_joint_cmd, dtype=np.float64).reshape(4).copy()
    q = np.asarray(q, dtype=np.float64).reshape(4)
    qd = np.asarray(qd, dtype=np.float64).reshape(4)
    q_des = np.asarray(q_des, dtype=np.float64).reshape(4)
    _ = qdot_des  # cable path does not use joint reference velocity

    if belt is not None:
        tau_drive_1 = float(out[0])
        tau_belt, belt_diag = belt.compute_effect(
            float(q[0]), float(qd[0]), float(q_des[0]), float(dt), tau_drive_1
        )
        out[0] = tau_drive_1 + float(tau_belt)
        diag["belt_diag"] = belt_diag

    if cable is not None:
        gear234 = MOTOR_TO_JOINT_GEAR[1:4].astype(np.float64)
        tau_act_cmd = out[1:4] * gear234
        tau_del, cdiag = cable.transmit(tau_act_cmd, q[1:4], qd[1:4], float(dt))
        out[1:4] = tau_del
        diag["cable_transmission"] = cdiag

    return out, diag
