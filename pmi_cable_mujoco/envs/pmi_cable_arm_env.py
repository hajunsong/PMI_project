"""Gymnasium env: MuJoCo rigid arm + hybrid transmission (ros_ws/pmi_description2/urdf/pmi_description2.urdf).

Rigid kinematics in MJCF: q1_jnt↔q1_act belt/gear; q2–4_jnt↔q2–4_act cable+pulley (equality mimic = PMI_Server kGear).
MuJoCo ``motor`` 는 구동축 ``q1_act``…``q4_act`` 에만 걸리며, ``_set_ctrl_torque`` 에 넣는 벡터는 **관절** 전달 토크(``jnt1``…``jnt4``)이며 ``τ_act = τ_joint ⊙ gear`` 로 변환한다.
jnt2–4 는 ``transmission.antagonistic_cable_joint`` 가 **모터 토크 명령**을 길항 케이블로 왜곡한 뒤의 관절 토크를 낸다 (가산 잔차 아님).
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import gymnasium as gym
import numpy as np
import yaml
from gymnasium import spaces

import mujoco

from transmission import BELT_JOINT_NAMES, CABLE_JOINT_NAMES
from transmission.antagonistic_cable_joint import AntagonisticCableStack, build_antagonistic_stack_from_config
from transmission.belt_model import BeltTransmissionModel, build_belt_model_from_config
from transmission.hybrid_joint_torque import apply_transmission_joint_torque
from transmission.randomization import HybridTransmissionSampler
from controllers.vsd_controller import ReferenceTrajectory, VSDController
from controllers.waypoint_trajectory import WaypointTrajectory
from kinematics.pmi_chain import actuator_torque_from_joint_torque

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = PROJECT_ROOT / "models" / "pmi_cable_arm.xml"

# MuJoCo joint names (URDF: q1_jnt..q4_jnt)
JOINT_NAMES = ("jnt1", "jnt2", "jnt3", "jnt4")
ACTUATOR_NAMES = ("tau_q1_act", "tau_q2_act", "tau_q3_act", "tau_q4_act")


def _load_yaml(path: Path) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _lambda_vec(control_cfg: Dict[str, Any]) -> np.ndarray:
    lr = control_cfg.get("lambda_residual", 0.08)
    if np.isscalar(lr):
        return np.full(4, float(lr), dtype=float)
    return np.asarray(lr, dtype=float).reshape(4)


class PMICableArmEnv(gym.Env):
    """Residual torque action shape (4,).

    Torque composition::

        tau_joint_cmd = tau_vsd + lambda * tau_res
        q1: tau_joint = belt(tau_joint_cmd[0], ...)
        q2–4: tau_act_cmd = tau_joint_cmd[i] * gear_i → 길항 케이블 → tau_joint[i]

    Bouc–Wen ``z`` (obs): plus-cable 축당 1개, 길이 3.
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(
        self,
        model_path: Optional[Union[str, Path]] = None,
        belt_config_path: Optional[Union[str, Path]] = None,
        cable_config_path: Optional[Union[str, Path]] = None,
        control_config_path: Optional[Union[str, Path]] = None,
        *,
        randomize_transmission: Optional[bool] = None,
        randomize_cable: Optional[bool] = None,
        include_hysteresis_state: bool = True,
        render_mode: Optional[str] = None,
        rng_seed: Optional[int] = None,
    ) -> None:
        super().__init__()
        self.render_mode = render_mode
        self._rng = np.random.default_rng(rng_seed)

        model_path = Path(model_path) if model_path else DEFAULT_MODEL
        belt_path = Path(
            belt_config_path or (PROJECT_ROOT / "configs" / "belt_params.yaml")
        )
        cable_path = Path(
            cable_config_path or (PROJECT_ROOT / "configs" / "cable_params.yaml")
        )
        ctrl_path = Path(
            control_config_path or (PROJECT_ROOT / "configs" / "control_params.yaml")
        )

        self._belt_base_cfg = _load_yaml(belt_path)
        self._cable_base_cfg = _load_yaml(cable_path)
        self._control_cfg = _load_yaml(ctrl_path)

        if randomize_transmission is not None:
            self._belt_base_cfg.setdefault("randomization", {})["enabled"] = bool(
                randomize_transmission
            )
            self._cable_base_cfg.setdefault("randomization", {})["enabled"] = bool(
                randomize_transmission
            )
        elif randomize_cable is not None:
            self._cable_base_cfg.setdefault("randomization", {})["enabled"] = bool(
                randomize_cable
            )

        self._sampler = HybridTransmissionSampler(
            self._belt_base_cfg, self._cable_base_cfg, seed=rng_seed
        )

        self.model = mujoco.MjModel.from_xml_path(os.fspath(model_path))
        self.data = mujoco.MjData(self.model)

        self.dt = float(self.model.opt.timestep)
        self.frame_skip = int(self._control_cfg.get("simulation", {}).get("frame_skip", 1))

        self._lambda = _lambda_vec(self._control_cfg)
        self._vsd = VSDController.from_config(self._control_cfg)
        traj_cfg = self._control_cfg.get("trajectory", {})
        ctrl_dt = self.dt * self.frame_skip
        traj_mode = str(traj_cfg.get("mode", "sine"))
        if traj_mode == "waypoints_xyz":
            self._traj: Union[ReferenceTrajectory, WaypointTrajectory] = WaypointTrajectory(
                traj_cfg, ctrl_dt
            )
            self._traj_is_waypoints = True
        else:
            self._traj = ReferenceTrajectory.from_config(traj_cfg, ctrl_dt)
            self._traj_is_waypoints = False

        bcfg, ccfg = self._sampler.sample()
        self._belt_model: BeltTransmissionModel = build_belt_model_from_config(bcfg)
        self._cable_stack: AntagonisticCableStack = build_antagonistic_stack_from_config(ccfg)

        self._joint_qpos_adr: List[int] = []
        self._joint_dof_adr: List[int] = []
        self._actuator_ids: List[int] = []
        for jn in JOINT_NAMES:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid < 0:
                raise RuntimeError(f"Joint not found in MJCF: {jn}")
            self._joint_qpos_adr.append(int(self.model.jnt_qposadr[jid]))
            self._joint_dof_adr.append(int(self.model.jnt_dofadr[jid]))
        for an in ACTUATOR_NAMES:
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, an)
            if aid < 0:
                raise RuntimeError(f"Actuator not found in MJCF: {an}")
            self._actuator_ids.append(int(aid))

        n = 4
        self._include_z = include_hysteresis_state
        obs_dim = 24 + (3 if self._include_z else 0)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(n,), dtype=np.float32
        )
        self._prev_action = np.zeros(n, dtype=np.float32)
        self._renderer: Optional[mujoco.Renderer] = None

    def _get_state(self) -> Tuple[np.ndarray, np.ndarray]:
        q = np.array(
            [float(self.data.qpos[adr]) for adr in self._joint_qpos_adr], dtype=float
        )
        qd = np.array(
            [float(self.data.qvel[adr]) for adr in self._joint_dof_adr], dtype=float
        )
        return q, qd

    def _set_ctrl_torque(self, tau_joint: np.ndarray) -> None:
        tau_joint = np.asarray(tau_joint, dtype=float).reshape(-1)
        assert tau_joint.size == len(self._actuator_ids)
        tau_act = actuator_torque_from_joint_torque(tau_joint)
        for i, aid in enumerate(self._actuator_ids):
            self.data.ctrl[aid] = float(tau_act[i])

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)
            self._sampler.set_seed(seed)

        mujoco.mj_resetData(self.model, self.data)
        bcfg, ccfg = self._sampler.sample()
        self._belt_model = build_belt_model_from_config(bcfg)
        self._cable_stack = build_antagonistic_stack_from_config(ccfg)
        self._belt_model.reset()
        self._cable_stack.reset()

        self._prev_action[:] = 0.0

        if self._traj_is_waypoints:
            q_seed, _ = self._get_state()
            self._traj.on_reset(
                q_seed,
                mujoco_data=self.data,
                joint_qpos_adr=self._joint_qpos_adr,
            )
        else:
            self._traj.reset()

        mujoco.mj_forward(self.model, self.data)
        q, qd = self._get_state()
        q_des, qdot_des = self._traj.peek()

        obs = self._make_obs(q, qd, q_des, qdot_des)
        info: Dict[str, Any] = {
            "belt_params_sample": bcfg,
            "cable_params_sample": ccfg,
            "belt_joint_names": list(BELT_JOINT_NAMES),
            "cable_joint_names": list(CABLE_JOINT_NAMES),
        }
        return obs, info

    def _make_obs(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        q_des: np.ndarray,
        qdot_des: np.ndarray,
    ) -> np.ndarray:
        err = q_des - q
        parts: List[np.ndarray] = [
            q.astype(np.float32),
            qd.astype(np.float32),
            q_des.astype(np.float32),
            qdot_des.astype(np.float32),
            err.astype(np.float32),
            self._prev_action.astype(np.float32),
        ]
        if self._include_z:
            z = self._cable_stack.z_states().astype(np.float32)
            parts.append(z)
        return np.concatenate(parts).astype(np.float32)

    def step(
        self, action: np.ndarray
    ) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        a = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        tau_scale = np.asarray(self._control_cfg["tau_saturation"], dtype=np.float64)
        tau_res_raw = (a * tau_scale.astype(np.float32)).astype(np.float64)

        q_des, qdot_des = self._traj.step()
        total_reward = 0.0

        tau_vsd_last = np.zeros(4)
        tau_belt_last = 0.0
        lam_tau_res_last = np.zeros(4)
        cable_diag_last: Dict[str, Any] = {}
        tau_cmd_last = np.zeros(4)

        for _ in range(self.frame_skip):
            q, qd = self._get_state()

            tau_vsd = self._vsd.compute(q, qd, q_des, qdot_des)
            lam_tau_res = self._lambda * tau_res_raw
            tau_joint_cmd = tau_vsd + lam_tau_res

            tau_cmd, trans_diag = apply_transmission_joint_torque(
                tau_joint_cmd,
                q,
                qd,
                q_des,
                qdot_des,
                self.dt,
                self._belt_model,
                self._cable_stack,
            )
            bd = trans_diag.get("belt_diag") or {}
            tau_belt_last = float(bd.get("tau_belt_total", 0.0))
            cable_diag_last = trans_diag.get("cable_transmission", {})

            tau_cmd = np.clip(
                tau_cmd,
                -np.asarray(self._control_cfg["tau_saturation"]),
                np.asarray(self._control_cfg["tau_saturation"]),
            )

            tau_vsd_last = tau_vsd
            lam_tau_res_last = lam_tau_res
            tau_cmd_last = tau_cmd.copy()

            self._set_ctrl_torque(tau_cmd)
            mujoco.mj_step(self.model, self.data)

            qn, _ = self._get_state()
            err = q_des - qn
            total_reward += -float(np.dot(err, err))

        self._prev_action = a
        q, qd = self._get_state()
        obs = self._make_obs(q, qd, q_des, qdot_des)

        z_cable = np.zeros(3, dtype=np.float64)
        if cable_diag_last:
            for ii, dj in enumerate(cable_diag_last.get("per_joint", [])):
                if ii < 3:
                    z_cable[ii] = float(dj.get("z_plus", 0.0))

        info = {
            "tau_vsd": tau_vsd_last.astype(np.float64),
            "tau_residual": tau_res_raw.astype(np.float64),
            "lambda_tau_residual": lam_tau_res_last.astype(np.float64),
            "tau_belt_jnt1": float(tau_belt_last),
            "tau_joint_delivered": tau_cmd_last.astype(np.float64),
            "cable_transmission": cable_diag_last,
            "z_cable": z_cable,
        }
        return (
            obs,
            total_reward / self.frame_skip,
            False,
            False,
            info,
        )

    def render(self) -> Optional[np.ndarray]:
        if self.render_mode is None:
            return None
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=480, width=640)
        self._renderer.update_scene(self.data, camera="fixed")
        return self._renderer.render()

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
