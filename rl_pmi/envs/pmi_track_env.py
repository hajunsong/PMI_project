"""
MuJoCo PMI 팔 — ``run_vsd`` 와 동일한 목표 EE 궤적·오차 정의에 대해,
관절 토크를 출력하고 오차(위치·자세·속도) 기반 보상을 받는 Gymnasium 환경.

제어·차원·기본 파라미터는 ``rl_pmi/docs/pmi_track_env.md`` 에 정리해 두었으며,
해당 내용을 바꿀 때 문서도 함께 갱신합니다.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Optional

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces

_RL_ROOT = Path(__file__).resolve().parents[1]
if str(_RL_ROOT) not in sys.path:
    sys.path.insert(0, str(_RL_ROOT))

_AP = _RL_ROOT.parent / "analysis/python"
if str(_AP) not in sys.path:
    sys.path.insert(0, str(_AP))

from trajectory import build_run_vsd_trajectories  # noqa: E402
from utils import mat2rpy, wrap_to_pi  # noqa: E402
from main import ControlMain  # noqa: E402


class PMITrackEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 60}

    def __init__(
        self,
        mjcf_path: Optional[Path] = None,
        h: float = 0.001,
        t_end: float = 3.0,
        tau_limit: float = 600.0,
        delta_f_scale: Optional[np.ndarray] = None,
        ks: Optional[np.ndarray] = None,
        kd: Optional[np.ndarray] = None,
        use_gravity_feedforward: bool = True,
        w_pos: float = 1.0,
        w_rp: float = 0.1,
        w_vel: float = 0.01,
        w_omega: float = 0.01,
        action_penalty: float = 1e-4,
        penalize_residual_torque: bool = False,
        reset_noise: float = 0.05,
        render_mode: Optional[str] = None,
    ):
        super().__init__()
        self.h = float(h)
        self.t_end = float(t_end)
        self.tau_limit = float(tau_limit)
        self.use_gravity_feedforward = use_gravity_feedforward
        # ``run_vsd`` 와 동일 PD 게인 (작업공간 성분별)
        self._ks = (
            np.asarray(ks, dtype=float)
            if ks is not None
            else np.array([15000.0, 15000.0, 15000.0, 1500.0, 1500.0], dtype=float)
        )
        self._kd = (
            np.asarray(kd, dtype=float)
            if kd is not None
            else np.array([1000.0, 1000.0, 1000.0, 10.0, 10.0], dtype=float)
        )
        # ΔF = action ⊙ delta_f_scale (action ∈ [-1,1]^5)
        self._delta_f_scale = (
            np.asarray(delta_f_scale, dtype=float)
            if delta_f_scale is not None
            else np.array([80.0, 80.0, 80.0, 40.0, 40.0], dtype=float)
        )
        if self._delta_f_scale.shape != (5,):
            raise ValueError("delta_f_scale 는 (5,) 형태여야 합니다.")
        self.w_pos = float(w_pos)
        self.w_rp = float(w_rp)
        self.w_vel = float(w_vel)
        self.w_omega = float(w_omega)
        self.action_penalty = float(action_penalty)
        # True: 보상에 ``w_u ||Δτ_RL||^2`` (Δτ_RL = J^T ΔF), False: ``w_u ||a||^2``
        self.penalize_residual_torque = bool(penalize_residual_torque)
        self.reset_noise = float(reset_noise)
        self.render_mode = render_mode
        self._delta_tau_rl = np.zeros(4, dtype=float)

        if mjcf_path is None:
            mjcf_path = (
                _RL_ROOT.parent / "mujoco_pmi_viz/models/pmi_arm_primitive_actuated.xml"
            )
        self.mjcf_path = Path(mjcf_path).resolve()

        sx, sy, sz = build_run_vsd_trajectories(h=self.h)
        self._path_x = sx[:, 0]
        self._path_vx = sx[:, 1]
        self._path_y = sy[:, 0]
        self._path_vy = sy[:, 1]
        self._path_z = sz[:, 0]
        self._path_vz = sz[:, 1]
        self._n_path = len(self._path_x)
        if self._n_path < 2:
            raise ValueError("궤적 길이가 부족합니다. h·웨이포인트를 확인하세요.")
        n_transition = max(1, int(round(self.t_end / self.h)))
        self._max_steps = min(self._n_path - 1, n_transition)

        self.des_roll = -np.pi / 2.0
        self.des_pitch = 0.0

        self.model = mujoco.MjModel.from_xml_path(str(self.mjcf_path))
        self.model.opt.timestep = self.h
        self.data = mujoco.MjData(self.model)
        if self.model.nu != 4 or self.model.nv != 4:
            raise ValueError(f"모델 nu/nv 가 4가 아님: nu={self.model.nu}, nv={self.model.nv}")

        self._ctrl = ControlMain()

        self._ee_site = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "ee"
        )
        self._jac_lin = np.zeros((3, self.model.nv), dtype=float)
        self._jac_rot = np.zeros((3, self.model.nv), dtype=float)

        # 관측: q(4), qdot(4), pos_err(3), roll_err, pitch_err, v_err(3), ω_err(2) → 18
        high = np.inf * np.ones(18, dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        # ΔF ∈ R^5 (작업공간 힘·모멘트 보정; run_vsd PD와 동일 차원)
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(5,), dtype=np.float32
        )

        self._step_idx = 0
        self._renderer: Optional[mujoco.Renderer] = None

    def _idx(self) -> int:
        return int(min(self._step_idx, self._n_path - 1))

    def _analytical_task_errors(self) -> tuple[np.ndarray, np.ndarray]:
        """해석 모델 EE 기준 ``run_vsd`` 와 동일한 e, e_v (각 5,)."""
        ix = self._idx()
        des_pos = np.array(
            [
                self._path_x[ix],
                self._path_y[ix],
                self._path_z[ix],
            ],
            dtype=float,
        )
        des_vel = np.array(
            [
                self._path_vx[ix],
                self._path_vy[ix],
                self._path_vz[ix],
                0.0,
                0.0,
            ],
            dtype=float,
        )
        ee = self._ctrl.body[3]
        err_pos = des_pos - ee.re
        err_roll = wrap_to_pi(self.des_roll - float(ee.rpy[0]))
        err_pitch = wrap_to_pi(self.des_pitch - float(ee.rpy[1]))
        err = np.concatenate(
            (err_pos, np.array([err_roll, err_pitch], dtype=float))
        )
        ev = np.zeros(5, dtype=float)
        ev[0] = des_vel[0] - ee.dre[0]
        ev[1] = des_vel[1] - ee.dre[1]
        ev[2] = des_vel[2] - ee.dre[2]
        ev[3] = des_vel[3] - ee.wi[0]
        ev[4] = des_vel[4] - ee.wi[1]
        return err, ev

    def _sync_kin_from_mujoco(self) -> None:
        for i in range(4):
            self._ctrl.body[i].qi = float(self.data.qpos[i])
            self._ctrl.body[i].dqi = float(self.data.qvel[i])
        self._ctrl.position_calculation()
        self._ctrl.velocity_calculation()

    def _get_obs(self) -> np.ndarray:
        d = self.data
        site_mat = d.site_xmat[self._ee_site].reshape(3, 3)
        rpy = mat2rpy(site_mat)
        ee_pos = d.site_xpos[self._ee_site].copy()

        ix = self._idx()
        des_pos = np.array(
            [
                self._path_x[ix],
                self._path_y[ix],
                self._path_z[ix],
            ],
            dtype=float,
        )
        des_vel = np.array(
            [
                self._path_vx[ix],
                self._path_vy[ix],
                self._path_vz[ix],
                0.0,
                0.0,
            ],
            dtype=float,
        )

        err_pos = des_pos - ee_pos
        err_roll = float(wrap_to_pi(self.des_roll - float(rpy[0])))
        err_pitch = float(wrap_to_pi(self.des_pitch - float(rpy[1])))

        mujoco.mj_jacSite(
            self.model,
            self.data,
            self._jac_lin,
            self._jac_rot,
            self._ee_site,
        )
        qv = d.qvel.copy()
        ee_lv = (self._jac_lin @ qv).ravel()
        ee_w = (self._jac_rot @ qv).ravel()
        ev = np.zeros(5, dtype=float)
        ev[0:3] = des_vel[0:3] - ee_lv
        ev[3] = des_vel[3] - ee_w[0]
        ev[4] = des_vel[4] - ee_w[1]

        obs = np.concatenate(
            [
                d.qpos[:4].astype(float),
                d.qvel[:4].astype(float),
                err_pos,
                np.array([err_roll, err_pitch], dtype=float),
                ev[0:3],
                ev[3:5],
            ]
        ).astype(np.float32)
        return obs

    def get_observation(self, path_step_index: int) -> np.ndarray:
        """현재 ``self.data`` 상태와 경로 샘플 인덱스로 SAC 관측 벡터(18,)를 만든다."""
        self._step_idx = int(path_step_index)
        return np.asarray(self._get_obs(), dtype=np.float32)

    def compute_actuator_torque(self, action: np.ndarray) -> np.ndarray:
        """
        학습 환경 ``step`` 과 동일: PD 작업공간 힘 + ΔF → 관절 토크 + (옵션) 중력 보상.
        ``self.data`` 가 현재 물리 상태와 일치해야 한다. ``mj_step`` 은 호출하지 않는다.
        """
        a = np.asarray(action, dtype=np.float64).reshape(5).clip(-1.0, 1.0)

        self._sync_kin_from_mujoco()
        err, ev = self._analytical_task_errors()
        J = self._ctrl.jacobian_calculation()

        F_vsd = self._ks * err + self._kd * ev
        delta_F = a * self._delta_f_scale
        self._delta_tau_rl = np.asarray(J.T @ delta_F, dtype=float).ravel()
        F = F_vsd + delta_F
        tau = J.T @ F
        tau = np.clip(tau, -self.tau_limit, self.tau_limit)

        if self.use_gravity_feedforward:
            v_save = self.data.qvel.copy()
            self.data.qvel[:] = 0.0
            mujoco.mj_forward(self.model, self.data)
            tau_g = np.asarray(
                self.data.qfrc_bias[: self.model.nv], dtype=float
            ).copy()
            self.data.qvel[:] = v_save
            mujoco.mj_forward(self.model, self.data)
            return tau + tau_g
        return tau

    def tracking_metrics(self) -> dict[str, float]:
        """현재 ``self.data`` 기준 추적 비용·오차 (보상의 cost 항과 동일 정의)."""
        d = self.data
        ee_pos = d.site_xpos[self._ee_site].copy()
        site_mat = d.site_xmat[self._ee_site].reshape(3, 3)
        rpy = mat2rpy(site_mat)

        ix = self._idx()
        des_pos = np.array(
            [
                self._path_x[ix],
                self._path_y[ix],
                self._path_z[ix],
            ],
            dtype=float,
        )
        des_vel = np.array(
            [
                self._path_vx[ix],
                self._path_vy[ix],
                self._path_vz[ix],
                0.0,
                0.0,
            ],
            dtype=float,
        )

        err_pos = des_pos - ee_pos
        err_roll = float(wrap_to_pi(self.des_roll - float(rpy[0])))
        err_pitch = float(wrap_to_pi(self.des_pitch - float(rpy[1])))

        mujoco.mj_jacSite(
            self.model,
            self.data,
            self._jac_lin,
            self._jac_rot,
            self._ee_site,
        )
        qv = d.qvel.copy()
        ee_lv = (self._jac_lin @ qv).ravel()
        ee_w = (self._jac_rot @ qv).ravel()
        ev = np.zeros(5, dtype=float)
        ev[0:3] = des_vel[0:3] - ee_lv
        ev[3] = des_vel[3] - ee_w[0]
        ev[4] = des_vel[4] - ee_w[1]

        cost = (
            self.w_pos * float(np.dot(err_pos, err_pos))
            + self.w_rp * (err_roll**2 + err_pitch**2)
            + self.w_vel * float(np.dot(ev[0:3], ev[0:3]))
            + self.w_omega * float(ev[3] ** 2 + ev[4] ** 2)
        )
        return {
            "err_pos_norm": float(np.linalg.norm(err_pos)),
            "err_angle_norm": float(np.sqrt(err_roll**2 + err_pitch**2)),
            "tracking_cost": float(cost),
        }

    def _reward_and_info(self, action: np.ndarray) -> tuple[float, dict[str, Any]]:
        m = self.tracking_metrics()
        cost = m["tracking_cost"]
        a = np.asarray(action, dtype=np.float64).reshape(5).clip(-1.0, 1.0)
        if self.penalize_residual_torque:
            u = float(np.dot(self._delta_tau_rl, self._delta_tau_rl))
        else:
            u = float(np.dot(a, a))
        reward = float(-cost - self.action_penalty * u)
        info = {
            "err_pos_norm": m["err_pos_norm"],
            "err_angle_norm": m["err_angle_norm"],
            "tracking_cost": m["tracking_cost"],
        }
        return reward, info

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ):
        super().reset(seed=seed)
        self._step_idx = 0
        mujoco.mj_resetData(self.model, self.data)
        rng = np.random.default_rng(seed)
        q0 = rng.uniform(-self.reset_noise, self.reset_noise, size=4)
        self.data.qpos[:] = q0
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self._delta_tau_rl[:] = 0.0
        obs = self._get_obs()
        return obs, {}

    def step(self, action: np.ndarray):
        a = np.asarray(action, dtype=np.float64).reshape(5).clip(-1.0, 1.0)

        self._sync_kin_from_mujoco()
        err, ev = self._analytical_task_errors()
        J = self._ctrl.jacobian_calculation()

        F_vsd = self._ks * err + self._kd * ev
        delta_F = a * self._delta_f_scale
        self._delta_tau_rl = np.asarray(J.T @ delta_F, dtype=float).ravel()
        F = F_vsd + delta_F
        tau = J.T @ F
        tau = np.clip(tau, -self.tau_limit, self.tau_limit)

        if self.use_gravity_feedforward:
            v_save = self.data.qvel.copy()
            self.data.qvel[:] = 0.0
            mujoco.mj_forward(self.model, self.data)
            tau_g = np.asarray(
                self.data.qfrc_bias[: self.model.nv], dtype=float
            ).copy()
            self.data.qvel[:] = v_save
            mujoco.mj_forward(self.model, self.data)
            tau_cmd = tau + tau_g
        else:
            tau_cmd = tau

        self.data.ctrl[:] = tau_cmd
        mujoco.mj_step(self.model, self.data)

        self._step_idx += 1
        reward, info = self._reward_and_info(a)

        terminated = not (
            np.all(np.isfinite(self.data.qpos))
            and np.all(np.isfinite(self.data.qvel))
        )
        truncated = self._step_idx >= self._max_steps

        obs = self._get_obs()
        return obs, reward, terminated, truncated, info

    def render(self):
        if self.render_mode == "rgb_array":
            if self._renderer is None:
                self._renderer = mujoco.Renderer(self.model, height=480, width=640)
            cam = mujoco.MjvCamera()
            mujoco.mjv_defaultFreeCamera(self.model, cam)
            cam.lookat[:] = [0.0, 0.0, 0.15]
            cam.distance = 2.2
            cam.elevation = -20.0
            cam.azimuth = 135.0
            self._renderer.update_scene(self.data, camera=cam)
            return self._renderer.render()
        return None

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
