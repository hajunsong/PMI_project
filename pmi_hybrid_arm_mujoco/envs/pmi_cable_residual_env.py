"""PMI hybrid arm: VSD nominal control + SAC residual task-space force (Jᵀ F → joint torque), actuator-side torque."""

from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import gymnasium as gym
import mujoco as mj
import numpy as np
from gymnasium import spaces

from kinematics.forward_kinematics import fk_ee_rp
from kinematics.inverse_kinematics import IKConfig, solve_ik_task_mode
from kinematics.task_jacobian import compute_task_jacobian_mode
from trajectory.joint_quintic import scaled_joint_quintic
from transmission.hybrid_transmission import HybridTransmission
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel, load_yaml

J_ARM = ["jnt1", "jnt2", "jnt3", "jnt4"]
ACT_NAMES = ["q1_act", "q2_act", "q3_act", "q4_act"]
SITE = "end_effector"
ARM_ONLY_XML = PKG_ROOT / "models" / "pmi_arm_only_no_collision.xml"
CABLE_LAYER_YAML = PKG_ROOT / "configs" / "cable_layer.yaml"

RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=np.float64)
Q_ACT_INITIAL_DEFAULT = np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=np.float64)

OBS_DIM = 52

# Termination / reward thresholds
JOINT_LIMIT_REWARD_MARGIN = -0.01
ACTUATOR_LIMIT_REWARD_MARGIN = -0.01
# 종료 검사용: 등식 및 케이블 무작위화 시 과도 종료 줄이도록 관절·액추에이터·속도 허용을 분리한다.
QVEL_INSTABILITY_THRESH = 500.0
LIMIT_BREACH_SEVERE_RAD_JOINT = 0.10
LIMIT_BREACH_SEVERE_RAD_ACTUATOR = 0.15


def joint_or_actuator_severe_breach(
    model: mj.MjModel, names: list[str], q: np.ndarray, *, is_actuator_side: bool
) -> bool:
    thr = LIMIT_BREACH_SEVERE_RAD_ACTUATOR if is_actuator_side else LIMIT_BREACH_SEVERE_RAD_JOINT
    for nm, qi in zip(names, np.asarray(q).flat):
        ji = joint_id(model, nm)
        lo = float(model.jnt_range[ji, 0])
        hi = float(model.jnt_range[ji, 1])
        if float(qi) < lo - thr or float(qi) > hi + thr:
            return True
    return False


def _margins(model: mj.MjModel, names: list[str], q: np.ndarray) -> float:
    lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in names])
    hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in names])
    return float(np.min(np.minimum(hi - q, q - lo)))


ROLL_DES_IK = -np.pi / 2.0
PITCH_DES_IK = 0.0


@dataclass
class JacobianResidualExtension:
    """
    Hooks for extending residual from 3-D force → 5-D wrench (Fx,Fy,Fz,M_roll,M_pitch).
    Override :meth:`action_dim_allowed` / :meth:`build_task_jacobian_slice` later.
    """

    def action_dim_allowed(self) -> tuple[int, ...]:
        return (3,)

    def build_task_jacobian_slice(self, J_pos_rp: np.ndarray) -> np.ndarray:
        """Return J such that tau = J.T @ W with current action layout; default: position rows only."""
        return np.asarray(J_pos_rp, dtype=np.float64)


def waypoint_arrays_from_config(waypoints_yaml: list[dict[str, Any]]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """First / middle / last waypoint by ascending ``t`` (expects three entries like 0 / 0.5 / 1)."""
    wps = sorted(waypoints_yaml, key=lambda d: float(d["t"]))
    if len(wps) < 3:
        raise ValueError("trajectory.waypoints requires at least 3 samples")
    w0, wm, wend = wps[0], wps[len(wps) // 2], wps[-1]
    def _xyz(w: dict[str, Any]) -> np.ndarray:
        return np.array([float(w["x"]), float(w["y"]), float(w["z"])], dtype=np.float64)

    return _xyz(w0), _xyz(wm), _xyz(wend)


def joint_torque_from_task_force(
    J_pos: np.ndarray,
    F: np.ndarray,
    residual_mode: str,
    *,
    _extension: JacobianResidualExtension | None = None,
) -> np.ndarray:
    """``F`` aligned with rows of ``J_pos`` (3-D force for xyz-only)."""
    J = np.asarray(J_pos, dtype=np.float64)
    Fv = np.asarray(F, dtype=np.float64).reshape(-1)
    if J.shape[0] != Fv.shape[0]:
        raise ValueError(f"J rows {J.shape[0]} != dim(F) {Fv.shape[0]}")
    if residual_mode == "all_joints_from_task_force":
        return J.T @ Fv
    if residual_mode == "cable_joints_only_from_task_force":
        Jc = J[:, 1:4]
        tau_c = Jc.T @ Fv
        out = np.zeros(4, dtype=np.float64)
        out[1:4] = tau_c
        return out
    raise ValueError(f"unknown residual_mode: {residual_mode}")


def load_hybrid_torque_only(xml_path: Path) -> mj.MjModel:
    spec = mj.MjSpec.from_file(str(xml_path))
    for a in list(spec.actuators)[::-1]:
        spec.delete(a)
    return spec.compile()


def _margins(model: mj.MjModel, names: list[str], q: np.ndarray) -> float:
    lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in names])
    hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in names])
    return float(np.min(np.minimum(hi - q, q - lo)))


def _deep_merge(base: dict[str, Any], over: dict[str, Any]) -> dict[str, Any]:
    out = copy.deepcopy(base)
    for k, v in over.items():
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def default_rl_sac_config() -> dict[str, Any]:
    p = PKG_ROOT / "configs" / "rl_sac.yaml"
    if not p.is_file():
        raise FileNotFoundError(f"missing {p}")
    return load_yaml(p)


class PMICableResidualEnv(gym.Env):
    """
    Nominal joint-space VSD on the hybrid (no position actuators, no ``data.ctrl`` torque).
    SAC adds task-space residual force mapped by ``tau_res = J^T F``, then ``tau_act = ratio ⊙ tau_jnt``
    via :class:`HybridTransmission` to ``q_act`` generalized forces.
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 100}

    def __init__(
        self,
        config: dict[str, Any] | None = None,
        *,
        config_path: str | Path | None = None,
        overrides: dict[str, Any] | None = None,
        rng: np.random.Generator | None = None,
    ):
        super().__init__()
        base = dict(config) if config is not None else default_rl_sac_config()
        cfg = base if config_path is None else _deep_merge(load_yaml(Path(config_path)), base)
        if overrides:
            cfg = _deep_merge(cfg, overrides)

        self._cfg = cfg
        self.rng = np.random.default_rng() if rng is None else rng

        env_cfg = cfg["env"]
        res_cfg = cfg["residual"]
        traj_cfg = cfg["trajectory"]
        ctl_cfg = cfg["controller"]
        rew_cfg = cfg["reward"]

        _mp = Path(str(env_cfg.get("model_path", "models/pmi_hybrid_no_collision.xml")))
        self._model_xml = _mp if _mp.is_absolute() else PKG_ROOT / _mp

        self.randomize_cable = bool(env_cfg.get("randomize_cable", True))
        self.randomization_profile = str(env_cfg.get("randomization_profile", "medium_train"))
        self.episode_duration = float(env_cfg.get("episode_duration", 5.0))
        self.control_dt = float(env_cfg.get("control_dt", 0.01))
        self.tau_jnt_limit = float(env_cfg.get("tau_jnt_limit", 30.0))
        self.normalize_observation = bool(env_cfg.get("normalize_observation", True))

        self.residual_force_scale = float(res_cfg.get("residual_force_scale", 5.0))
        self.lambda_residual = float(res_cfg.get("lambda_residual", 1.0))
        self.residual_mode = str(res_cfg.get("residual_mode", "cable_joints_only_from_task_force"))
        self._jacobian_ext = JacobianResidualExtension()

        adim = int(res_cfg.get("action_dim", 3))
        if adim != 3:
            raise ValueError(f"Only action_dim=3 supported in this revision; got {adim}")

        self.Kq = np.asarray(ctl_cfg["Kq"], dtype=np.float64).reshape(4)
        self.Dq = np.asarray(ctl_cfg["Dq"], dtype=np.float64).reshape(4)

        self._w_ee = float(rew_cfg["w_ee"])
        self._w_q = float(rew_cfg["w_q"])
        self._w_f = float(rew_cfg["w_f"])
        self._w_df = float(rew_cfg["w_df"])
        self._w_tau = float(rew_cfg["w_tau"])
        self._w_sat = float(rew_cfg["w_sat"])
        self._w_lim = float(rew_cfg["w_lim"])
        self._use_improvement = bool(rew_cfg.get("use_improvement_reward", False))
        self._w_improve = float(rew_cfg.get("w_improvement", rew_cfg.get("w_improve", 10.0)))

        traj_dur = float(traj_cfg.get("duration", self.episode_duration))
        wp0, wp1, wp2 = waypoint_arrays_from_config(list(traj_cfg["waypoints"]))
        self._traj_duration = traj_dur
        self._wp_xyz = (wp0, wp1, wp2)

        self.model = load_hybrid_torque_only(self._model_xml)
        self.data = mj.MjData(self.model)
        self._sim_dt = float(self.model.opt.timestep)
        if self._sim_dt <= 0.0:
            raise ValueError("model.opt.timestep must be positive")
        self._n_internal = max(1, int(round(self.control_dt / self._sim_dt)))
        err = abs(self._n_internal * self._sim_dt - self.control_dt)
        if err > 1e-6 + 1e-9 * self.control_dt:
            raise ValueError(
                f"control_dt={self.control_dt} not commensurate with sim_dt={self._sim_dt} "
                f"(n={self._n_internal}, error={err})"
            )

        self._qadr_j = np.array([int(self.model.jnt_qposadr[joint_id(self.model, n)]) for n in J_ARM], dtype=int)
        self._dadr_j = np.array([int(self.model.jnt_dofadr[joint_id(self.model, n)]) for n in J_ARM], dtype=int)
        self._qadr_a = np.array([int(self.model.jnt_qposadr[joint_id(self.model, n)]) for n in ACT_NAMES], dtype=int)
        self._dadr_a = np.array([int(self.model.jnt_dofadr[joint_id(self.model, n)]) for n in ACT_NAMES], dtype=int)

        self._jac_data = mj.MjData(self.model)
        self._scratch = mj.MjData(self.model)

        cable_full = load_yaml(CABLE_LAYER_YAML)
        self._transmission = HybridTransmission(cable_config=cable_full)

        self._jpath: Any = None
        self._q_act0 = Q_ACT_INITIAL_DEFAULT.copy()
        self._q_jnt0 = RATIOS * self._q_act0

        self._time = 0.0
        self._step_count = 0
        self._max_steps = int(round(self.episode_duration / self.control_dt))

        self._prev_action = np.zeros(3, dtype=np.float64)
        self._prev_F = np.zeros(3, dtype=np.float64)
        self._prev_tau_res = np.zeros(4, dtype=np.float64)
        self._prev_ee_err_norm = 0.0

        self._renderer: mj.Renderer | None = None
        self.render_mode: str | None = None

        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(OBS_DIM,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)

        self._arm_only_model: mj.MjModel | None = None
        self._arm_only_scratch: mj.MjData | None = None
        self._ik_wp_cache: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None

    # --- IK cache (joint quintic waypoint joints) ---
    def _ensure_ik(self) -> None:
        if self._ik_wp_cache is not None:
            return
        self._arm_only_model = load_mjmodel(ARM_ONLY_XML, strip_position_actuators=True)
        self._arm_only_scratch = mj.MjData(self._arm_only_model)
        m, d = self._arm_only_model, self._arm_only_scratch
        q_lo = np.array([float(m.jnt_range[joint_id(m, n), 0]) for n in J_ARM])
        q_hi = np.array([float(m.jnt_range[joint_id(m, n), 1]) for n in J_ARM])
        ik = IKConfig((1, 1, 1), 1.0, 1.0, 1e-3, 1e-4, 80, 1e-9, tuple(J_ARM))
        qs: list[np.ndarray] = []
        q_seed = self._q_jnt0.copy()
        for wp in self._wp_xyz:
            q, _ = solve_ik_task_mode(
                m,
                d,
                np.asarray(wp, dtype=np.float64),
                roll_des=ROLL_DES_IK,
                pitch_des=PITCH_DES_IK,
                task_feas_mode="xyz",
                ik=ik,
                q_seed=q_seed,
                bounds_lo=q_lo,
                bounds_hi=q_hi,
            )
            qs.append(q.copy())
            q_seed = q.copy()
        self._ik_wp_cache = (qs[0], qs[1], qs[2])

    def _rebuild_joint_path(self) -> None:
        self._ensure_ik()
        assert self._ik_wp_cache is not None
        self._jpath = scaled_joint_quintic(self._ik_wp_cache[0], self._ik_wp_cache[1], self._ik_wp_cache[2], self._traj_duration)

    # --- MuJoCo / control ---
    def _read_arm_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        q_j = np.array([float(self.data.qpos[int(a)]) for a in self._qadr_j], dtype=np.float64)
        qd_j = np.array([float(self.data.qvel[int(a)]) for a in self._dadr_j], dtype=np.float64)
        q_a = np.array([float(self.data.qpos[int(a)]) for a in self._qadr_a], dtype=np.float64)
        qd_a = np.array([float(self.data.qvel[int(a)]) for a in self._dadr_a], dtype=np.float64)
        return q_j, qd_j, q_a, qd_a

    def _qact_snapshot(self) -> tuple[np.ndarray, np.ndarray]:
        qa = np.array([float(self.data.qpos[int(a)]) for a in self._qadr_a], dtype=np.float64)
        qda = np.array([float(self.data.qvel[int(a)]) for a in self._dadr_a], dtype=np.float64)
        return qa, qda

    def _site_xyz(self) -> np.ndarray:
        sid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_SITE, SITE)
        return np.array(self.data.site_xpos[sid], dtype=np.float64).copy()

    def _ee_desired_kinematics(self, t_path: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        assert self._jpath is not None
        q_des, qd_des, _ = self._jpath.sample(float(t_path))
        self._jac_data.qpos[:] = self.data.qpos
        self._jac_data.qvel[:] = self.data.qvel
        for k in range(4):
            self._jac_data.qpos[int(self._qadr_j[k])] = float(q_des[k])
        mj.mj_forward(self.model, self._jac_data)
        J_des = compute_task_jacobian_mode(
            self.model,
            self._jac_data,
            joint_names=J_ARM,
            task_mode="xyz",
            ee_site_name=SITE,
            mode="mujoco_analytic",
            epsilon=1e-6,
        )
        xd_des = J_des @ qd_des
        x_des, *_ = fk_ee_rp(self.model, self._scratch, q_des, J_ARM)
        x_des = np.asarray(x_des, dtype=np.float64).reshape(3)
        return q_des, qd_des, x_des, xd_des

    def _jacobian_pos_at(self) -> np.ndarray:
        self._jac_data.qpos[:] = self.data.qpos
        self._jac_data.qvel[:] = self.data.qvel
        mj.mj_forward(self.model, self._jac_data)
        return compute_task_jacobian_mode(
            self.model,
            self._jac_data,
            joint_names=J_ARM,
            task_mode="xyz",
            ee_site_name=SITE,
            mode="mujoco_analytic",
            epsilon=1e-6,
        )

    def _normalize_obs(self, obs: np.ndarray) -> np.ndarray:
        if not self.normalize_observation:
            return obs
        out = obs.copy()
        i = 0
        # q_jnt, qdot_jnt, q_des, qdot_des, q_err (20)
        out[i : i + 20] /= np.array([np.pi] * 4 + [10.0] * 4 + [np.pi] * 4 + [10.0] * 4 + [np.pi] * 4, dtype=np.float64)
        i += 20
        out[i : i + 3] /= 0.1
        i += 3
        out[i : i + 3] /= 2.0
        i += 3
        out[i : i + 4] /= self.tau_jnt_limit
        i += 4
        out[i : i + 3] /= 1.0
        i += 3
        out[i : i + 3] /= self.residual_force_scale
        i += 3
        out[i : i + 4] /= self.tau_jnt_limit
        i += 4
        act_scale = self.tau_jnt_limit * float(np.max(RATIOS))
        out[i : i + 9] /= act_scale
        i += 9
        out[i : i + 3] /= 1.0
        i += 3
        assert i == OBS_DIM
        return out

    def _build_observation(
        self,
        q_j: np.ndarray,
        qd_j: np.ndarray,
        q_des: np.ndarray,
        qd_des: np.ndarray,
        tau_vsd: np.ndarray,
        ee_e: np.ndarray,
        ee_ve: np.ndarray,
        tau_act_ideal: np.ndarray,
        tau_act_out: np.ndarray,
        hys_z_cable: np.ndarray,
    ) -> np.ndarray:
        q_err = q_des - q_j
        terr = tau_act_ideal[1:4] - tau_act_out[1:4]
        parts = [
            q_j,
            qd_j,
            q_des,
            qd_des,
            q_err,
            ee_e,
            ee_ve,
            tau_vsd,
            self._prev_action,
            self._prev_F,
            self._prev_tau_res,
            tau_act_ideal[1:4],
            tau_act_out[1:4],
            terr,
            hys_z_cable,
        ]
        obs = np.concatenate(parts, dtype=np.float64)
        assert obs.shape[0] == OBS_DIM
        return self._normalize_obs(obs).astype(np.float32)

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(int(seed))

        opts = options or {}
        if "config_overrides" in opts:
            self._cfg = _deep_merge(self._cfg, opts["config_overrides"])
        if "q_act_initial" in opts:
            self._q_act0 = np.asarray(opts["q_act_initial"], dtype=np.float64).reshape(4)
            self._q_jnt0 = RATIOS * self._q_act0

        self._ik_wp_cache = None
        self._rebuild_joint_path()

        mj.mj_resetData(self.model, self.data)
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        for k in range(4):
            self.data.qpos[int(self._qadr_a[k])] = float(self._q_act0[k])
            self.data.qpos[int(self._qadr_j[k])] = float(self._q_jnt0[k])
        mj.mj_forward(self.model, self.data)
        self.data.qfrc_applied[:] = 0.0

        self._time = 0.0
        self._step_count = 0

        self._prev_action[:] = 0.0
        self._prev_F[:] = 0.0
        self._prev_tau_res[:] = 0.0
        x_a = self._site_xyz()
        t_path = min(self._time, self._traj_duration)
        q_des, qd_des, x_des, xd_des = self._ee_desired_kinematics(t_path)
        q_j, qd_j, _, _ = self._read_arm_state()
        mj.mj_forward(self.model, self.data)
        tau_bias = np.array([float(self.data.qfrc_bias[int(d)]) for d in self._dadr_j], dtype=np.float64)
        tau_vsd = tau_bias + self.Kq * (q_des - q_j) + self.Dq * (qd_des - qd_j)
        J = self._jacobian_pos_at()
        xd_act = J @ qd_j
        ee_e = x_des - x_a
        ee_ve = xd_des - xd_act
        self._prev_ee_err_norm = float(np.linalg.norm(ee_e))

        prof = str(opts.get("randomization_profile", self.randomization_profile))
        do_rnd = bool(opts.get("randomize_cable", self.randomize_cable))
        rseed = int(opts["cable_seed"]) if opts.get("cable_seed") is not None else int(self.rng.integers(0, 2**31 - 1))
        self._transmission.reset(randomize=do_rnd, seed=rseed if do_rnd else None, profile_name=prof if do_rnd else None)

        tau0 = np.zeros(4)
        tout0 = self._transmission.transmit(np.zeros(4), self._sim_dt, *self._qact_snapshot())

        hz0 = np.zeros(3)
        cr = self._transmission.last_cable_result
        if cr is not None:
            hz0 = np.array(cr.hys_z, dtype=np.float64)

        obs = self._build_observation(
            q_j,
            qd_j,
            q_des,
            qd_des,
            tau_vsd,
            ee_e,
            ee_ve,
            np.zeros(4),
            tout0,
            hz0,
        )
        inf0: dict[str, Any] = {
            "time": self._time,
            "ee_des_xyz": np.asarray(x_des, dtype=np.float64).reshape(3).copy(),
            "ee_act_xyz": np.asarray(x_a, dtype=np.float64).reshape(3).copy(),
            "F_residual_xyz": np.zeros(3),
            "tau_residual_jnt": np.zeros(4),
            "tau_vsd": tau_vsd,
            "tau_jnt_cmd": tau_vsd.copy(),
            "tau_act_ideal": np.zeros(4),
            "tau_act_out": tout0.copy(),
            "sampled_cable_params": self._transmission.get_current_params(),
            "reward_total": 0.0,
        }
        return obs, inf0

    def step(self, action: np.ndarray | list[float]):
        """One control step of duration ``control_dt`` (possibly multiple physics substeps)."""
        a = np.clip(np.asarray(action, dtype=np.float64).reshape(3), -1.0, 1.0)
        F = a * float(self.residual_force_scale)
        F_prev = self._prev_F.copy()

        t_ctrl = min(self._time, self._traj_duration)
        q_j0, qd_j0, _, _ = self._read_arm_state()
        mj.mj_forward(self.model, self.data)
        tau_bias0 = np.array([float(self.data.qfrc_bias[int(d)]) for d in self._dadr_j], dtype=np.float64)
        q_des, qd_des, _, _ = self._ee_desired_kinematics(t_ctrl)
        tau_vsd_nominal = tau_bias0 + self.Kq * (q_des - q_j0) + self.Dq * (qd_des - qd_j0)

        J = self._jacobian_pos_at()
        tau_geom = joint_torque_from_task_force(J, F, self.residual_mode, _extension=self._jacobian_ext)
        tau_res_cmd = float(self.lambda_residual) * tau_geom
        tau_unclip = tau_vsd_nominal + tau_res_cmd
        tau_jnt_clip = np.clip(tau_unclip, -self.tau_jnt_limit, self.tau_jnt_limit)
        sat_count_step = int(np.any(np.abs(tau_jnt_clip - tau_unclip) > 1e-8))

        tau_act_ideal = RATIOS * tau_jnt_clip
        tout_last = tau_act_ideal.copy()
        hz_last = np.zeros(3, dtype=np.float64)

        cr: Any = None
        ncon_peak = 0
        for _ in range(self._n_internal):
            qa, qda = self._qact_snapshot()
            tout_last = self._transmission.transmit(tau_act_ideal, self._sim_dt, qa, qda).copy()

            cr = self._transmission.last_cable_result
            if cr is not None:
                hz_last = np.array(cr.hys_z, dtype=np.float64).copy()

            self.data.qfrc_applied[:] = 0.0
            for kk in range(4):
                self.data.qfrc_applied[int(self._dadr_a[kk])] = float(tout_last[kk])
            mj.mj_step(self.model, self.data)
            ncon_peak = max(ncon_peak, int(self.data.ncon))

        self._time += self.control_dt
        self._step_count += 1

        mj.mj_forward(self.model, self.data)
        q_j, qd_j, _, _ = self._read_arm_state()

        t_new = min(self._time, self._traj_duration)
        q_des_r, qd_des_r, x_des_r, xd_des_r = self._ee_desired_kinematics(t_new)

        jac_after = self._jacobian_pos_at()
        xd_act = jac_after @ qd_j
        x_actual = self._site_xyz()
        ee_e = x_des_r - x_actual
        ee_err_norm = float(np.linalg.norm(ee_e))
        q_a_vec = np.array([float(self.data.qpos[int(adr)]) for adr in self._qadr_a], dtype=np.float64)
        jl_marg = _margins(self.model, J_ARM, q_j)
        al_marg = _margins(self.model, ACT_NAMES, q_a_vec)
        limit_count_step = int((jl_marg < JOINT_LIMIT_REWARD_MARGIN) or (al_marg < ACTUATOR_LIMIT_REWARD_MARGIN))

        breached = joint_or_actuator_severe_breach(self.model, J_ARM, q_j, is_actuator_side=False) or joint_or_actuator_severe_breach(
            self.model,
            ACT_NAMES,
            q_a_vec,
            is_actuator_side=True,
        )

        rew_ee = -self._w_ee * float(np.dot(ee_e, ee_e))
        q_err = q_des_r - q_j
        rew_q = -self._w_q * float(np.dot(q_err, q_err))
        rew_f = -self._w_f * float(np.dot(F, F))
        dF = F - F_prev
        rew_df = -self._w_df * float(np.dot(dF, dF))
        rew_tau = -self._w_tau * float(np.dot(tau_res_cmd, tau_res_cmd))
        rew_sat = -self._w_sat * float(sat_count_step)
        rew_lim = -self._w_lim * float(limit_count_step)
        reward = rew_ee + rew_q + rew_f + rew_df + rew_tau + rew_sat + rew_lim

        rew_improve = 0.0
        if self._use_improvement:
            rew_improve = float(self._w_improve * (self._prev_ee_err_norm - ee_err_norm))
            reward += rew_improve

        terminated = False
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all() or not np.isfinite(self.data.qacc).all():
            terminated = True
        elif ncon_peak > 0:
            terminated = True
        elif breached:
            terminated = True
        elif float(np.max(np.abs(self.data.qvel))) > QVEL_INSTABILITY_THRESH:
            terminated = True

        truncated = self._step_count >= self._max_steps or self._time >= self.episode_duration - 1e-9

        tau_loss_c = tau_hys_c = np.zeros(3, dtype=np.float64)
        trans_err_q = tau_act_ideal[1:4] - tout_last[1:4]

        self._prev_action = a.astype(np.float64).copy()
        self._prev_F = F.astype(np.float64).copy()
        self._prev_tau_res = tau_res_cmd.astype(np.float64).copy()

        mj.mj_forward(self.model, self.data)
        tau_bias_post = np.array([float(self.data.qfrc_bias[int(d)]) for d in self._dadr_j], dtype=np.float64)
        tau_vsd_obs = tau_bias_post + self.Kq * (q_des_r - q_j) + self.Dq * (qd_des_r - qd_j)
        J_obs = self._jacobian_pos_at()
        xd_act_obs = J_obs @ qd_j
        ee_e_obs = x_des_r - self._site_xyz()
        ee_ve_obs = xd_des_r - xd_act_obs

        if cr is not None:
            tau_loss_c = np.array(cr.tau_loss, dtype=np.float64).copy()
            tau_hys_c = np.array(cr.tau_hys, dtype=np.float64).copy()

        obs = self._build_observation(
            q_j,
            qd_j,
            q_des_r,
            qd_des_r,
            tau_vsd_obs,
            ee_e_obs,
            ee_ve_obs,
            tau_act_ideal,
            tout_last,
            hz_last,
        )

        self._prev_ee_err_norm = ee_err_norm

        info: dict[str, Any] = {
            "ee_des_xyz": x_des_r.astype(np.float64).copy(),
            "ee_act_xyz": self._site_xyz().astype(np.float64).copy(),
            "time": float(self._time),
            "ee_error_norm": ee_err_norm,
            "q_error_norm": float(np.linalg.norm(q_err)),
            "F_residual_xyz": F.copy(),
            "tau_residual_jnt": tau_res_cmd.copy(),
            "reward_ee": rew_ee,
            "reward_q": rew_q,
            "reward_force": rew_f,
            "reward_force_rate": rew_df,
            "reward_tau_residual": rew_tau,
            "reward_saturation": rew_sat,
            "reward_limit": rew_lim,
            "reward_improvement": rew_improve,
            "reward_total": float(reward),
            "saturation_count": sat_count_step,
            "joint_limit_violation": int(jl_marg < JOINT_LIMIT_REWARD_MARGIN),
            "actuator_limit_violation": int(al_marg < ACTUATOR_LIMIT_REWARD_MARGIN),
            "joint_limit_margin": jl_marg,
            "actuator_limit_margin": al_marg,
            "ncon": int(ncon_peak),
            "tau_vsd": tau_vsd_nominal.copy(),
            "tau_jnt_cmd": tau_jnt_clip.copy(),
            "tau_act_ideal": tau_act_ideal.copy(),
            "tau_act_out": tout_last.copy(),
            "sampled_cable_params": self._transmission.get_current_params(),
            "hys_z_q2q4": hz_last.astype(np.float64).copy(),
            "tau_loss_q2q4": tau_loss_c.astype(np.float64).copy(),
            "tau_hys_q2q4": tau_hys_c.astype(np.float64).copy(),
            "tau_transmission_error_q2q4": trans_err_q.astype(np.float64).copy(),
            "termination_reason": "",
        }
        if terminated:
            if not np.isfinite(self.data.qpos).all():
                info["termination_reason"] = "nan"
            elif ncon_peak > 0:
                info["termination_reason"] = "ncon"
            elif breached:
                info["termination_reason"] = "limit_breach"
            else:
                info["termination_reason"] = "instability"

        return obs, float(reward), bool(terminated), bool(truncated), info

    def render(self) -> np.ndarray | None:
        if self.render_mode is None:
            return None
        h, w = 480, 640
        if self._renderer is None:
            self._renderer = mj.Renderer(self.model, height=h, width=w)
        self._renderer.update_scene(self.data)
        return np.asarray(self._renderer.render(), dtype=np.uint8)

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
