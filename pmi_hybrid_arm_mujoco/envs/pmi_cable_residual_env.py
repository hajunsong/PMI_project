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
from kinematics.orientation_utils import angle_error
from kinematics.task_jacobian import compute_task_jacobian_mode
from trajectory.joint_quintic import scaled_joint_quintic
from transmission.hybrid_transmission import HybridTransmission
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel, load_yaml

from control.task_space_5d import (
    analytic_jacobian_5dof,
    heuristic_wrench_rp_zeros_force,
    residual_torque_from_wrench_5d,
    slice_joint_columns,
)

J_ARM = ["jnt1", "jnt2", "jnt3", "jnt4"]
ACT_NAMES = ["q1_act", "q2_act", "q3_act", "q4_act"]
SITE = "end_effector"
ARM_ONLY_XML = PKG_ROOT / "models" / "pmi_arm_only_no_collision.xml"
CABLE_LAYER_YAML = PKG_ROOT / "configs" / "cable_layer.yaml"

RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=np.float64)
Q_ACT_INITIAL_DEFAULT = np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=np.float64)

OBS_DIM_BASE = 52
OBS_EXTRA_BASELINE_REF = 8
# Back-compat alias (base observation without baseline-ref extension)
OBS_DIM = OBS_DIM_BASE

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
    """Backward-compat stub — env selects Jacobians explicitly."""

    def action_dim_allowed(self) -> tuple[int, ...]:
        return (3, 5)

    def build_task_jacobian_slice(self, J_pos_rp: np.ndarray) -> np.ndarray:
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
        self.residual_moment_scale = float(res_cfg.get("residual_moment_scale", 0.05))
        self._residual_gain = float(res_cfg.get("residual_gain", 1.0))
        self.lambda_residual = float(res_cfg.get("lambda_residual", 1.0))
        self.residual_mode = str(res_cfg.get("residual_mode", "cable_joints_only_from_task_force"))
        self._jacobian_ext = JacobianResidualExtension()
        self._residual_type_tag = str(res_cfg.get("type", "task_space_force"))
        if self._residual_type_tag not in ("task_space_force", "task_space_wrench_5d"):
            raise ValueError(f"unknown residual.type: {self._residual_type_tag}")

        joint_scope_ov = res_cfg.get("residual_joint_scope")
        if isinstance(joint_scope_ov, str) and joint_scope_ov == "all_joints":
            self._wrench_joint_cable_only = False
        elif isinstance(joint_scope_ov, str) and joint_scope_ov == "cable_joints_only":
            self._wrench_joint_cable_only = True
        elif joint_scope_ov is None:
            self._wrench_joint_cable_only = "cable_joints_only" in self.residual_mode
        else:
            raise ValueError(f"unknown residual_joint_scope: {joint_scope_ov}")

        if self._residual_type_tag == "task_space_force":
            self._action_dim = int(res_cfg.get("action_dim", 3))
            if self._action_dim != 3:
                raise ValueError("task_space_force requires action_dim=3")
            self._wrench_map_mode = str(res_cfg.get("wrench_map_mode", "wrench_jacobian_transpose"))
            self._wrench_command_mode = "scaled_action_xyz"
            self._lambda_dls = float(res_cfg.get("lambda_dls", 0.05))
            self._K_dls = float(res_cfg.get("K_dls_joint", res_cfg.get("K_dls", 20.0)))
            self._D_dls = float(res_cfg.get("D_dls_joint", res_cfg.get("D_dls", 5.0)))
            self._task_weight_5d = np.ones(5, dtype=np.float64)
        else:
            self._wrench_command_mode = str(res_cfg.get("wrench_command_mode", "scaled_action_full5"))
            self._wrench_map_mode = str(res_cfg.get("wrench_map_mode", "wrench_jacobian_transpose"))
            wm = self._wrench_map_mode
            if wm not in (
                "wrench_jacobian_transpose",
                "dls_task_correction",
                "dls_weighted_wrench",
            ):
                raise ValueError(f"unknown wrench_map_mode: {wm}")

            wc = self._wrench_command_mode
            if wc == "sac_xyz_plus_heuristic_rp":
                self._action_dim = int(res_cfg.get("action_dim", 3))
                if self._action_dim != 3:
                    raise ValueError("sac_xyz_plus_heuristic_rp requires action_dim=3")
            elif wc == "scaled_action_full5":
                self._action_dim = int(res_cfg.get("action_dim", 5))
                if self._action_dim != 5:
                    raise ValueError("scaled_action_full5 requires action_dim=5")
            elif wc == "heuristic_rp_zeros_force":
                self._action_dim = int(res_cfg.get("action_dim", 5))
            else:
                raise ValueError(f"unknown wrench_command_mode: {wc}")

            tw = res_cfg.get("task_weights_5d")
            if tw is None:
                self._task_weight_5d = np.array([1.0, 1.0, 1.0, 0.2, 0.2], dtype=np.float64)
            else:
                self._task_weight_5d = np.asarray(tw, dtype=np.float64).reshape(5)
            self._lambda_dls = float(res_cfg.get("lambda_dls", 0.05))
            self._K_dls = float(res_cfg.get("K_dls_joint", res_cfg.get("K_dls", 20.0)))
            self._D_dls = float(res_cfg.get("D_dls_joint", res_cfg.get("D_dls", 5.0)))
            hrp = res_cfg.get("heuristic_residual_rp") if isinstance(res_cfg.get("heuristic_residual_rp"), dict) else {}
            self._h_kp_roll = float(hrp.get("Kp_roll", 0.05))
            self._h_dp_roll = float(hrp.get("Dp_roll", 0.005))
            self._h_kp_pitch = float(hrp.get("Kp_pitch", 0.05))
            self._h_dp_pitch = float(hrp.get("Dp_pitch", 0.005))
        self.Kq = np.asarray(ctl_cfg["Kq"], dtype=np.float64).reshape(4)
        self.Dq = np.asarray(ctl_cfg["Dq"], dtype=np.float64).reshape(4)

        orient_mode_raw = str(ctl_cfg.get("orientation_soft_mode", "none"))
        legacy_map = {
            "none": "none",
            "joint_only": "none",
            "xyz_only": "none",
            "soft_roll": "soft_roll",
            "xyz_plus_roll_soft": "soft_roll",
            "soft_roll_pitch": "soft_roll_pitch",
            "xyz_plus_roll_pitch_soft": "soft_roll_pitch",
        }
        self._orient_soft_mode = legacy_map.get(orient_mode_raw, orient_mode_raw)
        if self._orient_soft_mode not in ("none", "soft_roll", "soft_roll_pitch"):
            raise ValueError(f"unknown orientation_soft_mode: {orient_mode_raw}")

        oroll = ctl_cfg.get("orientation_soft_roll") if isinstance(ctl_cfg.get("orientation_soft_roll"), dict) else {}
        opitch = ctl_cfg.get("orientation_soft_pitch") if isinstance(ctl_cfg.get("orientation_soft_pitch"), dict) else {}
        self._Kp_roll_soft = float(oroll.get("Kp", 5.0))
        self._Dp_roll_soft = float(oroll.get("Dp", 0.5))
        self._Kp_pitch_soft = float(opitch.get("Kp", 5.0))
        self._Dp_pitch_soft = float(opitch.get("Dp", 0.5))

        self._w_ee = float(rew_cfg["w_ee"])
        self._w_q = float(rew_cfg["w_q"])
        self._w_f = float(rew_cfg["w_f"])
        self._w_df = float(rew_cfg["w_df"])
        self._w_tau = float(rew_cfg["w_tau"])
        self._w_sat = float(rew_cfg["w_sat"])
        self._w_lim = float(rew_cfg["w_lim"])
        self._use_ee_l1 = bool(rew_cfg.get("use_ee_l1", False))
        self._w_ee_l1 = float(rew_cfg.get("w_ee_l1", 0.0))
        self._use_improvement = bool(rew_cfg.get("use_improvement_reward", False))
        self._w_improve = float(rew_cfg.get("w_improvement", rew_cfg.get("w_improve", 10.0)))
        self._use_terminal_final_penalty = bool(rew_cfg.get("use_terminal_final_error_penalty", False))
        self._w_final_ee = float(rew_cfg.get("w_final_ee", 0.0))
        self._w_tau_rate = float(rew_cfg.get("w_tau_rate", 0.0))
        self._w_ee_vel = float(rew_cfg.get("w_ee_vel", 0.0))
        self._w_ee_acc = float(rew_cfg.get("w_ee_acc", 0.0))
        self._w_ee_highfreq = float(rew_cfg.get("w_ee_highfreq", 0.0))
        self._highfreq_lp_hz = float(rew_cfg.get("highfreq_lp_hz", 8.0))
        self._use_terminal_final_vel_penalty = bool(rew_cfg.get("use_terminal_final_velocity_penalty", False))
        self._w_final_ee_vel = float(rew_cfg.get("w_final_ee_vel", 0.0))

        rf_cfg = cfg.get("residual_filter") or {}
        self._residual_filter_enabled = bool(rf_cfg.get("enabled", False))
        self._residual_filter_tau_xyz = float(rf_cfg.get("force_filter_tau", rf_cfg.get("tau", 0.03)))
        self._residual_filter_tau_rp = float(rf_cfg.get("moment_filter_tau", rf_cfg.get("tau", self._residual_filter_tau_xyz)))

        rsp_cfg = cfg.get("residual_postprocess") or {}
        self._final_fade_duration = float(rsp_cfg.get("final_fade_duration", 0.0))

        asm_cfg = cfg.get("action_smoothing") or {}
        self._action_smooth_enabled = bool(asm_cfg.get("enabled", False))
        self._max_delta_force = float(asm_cfg.get("max_delta_force_per_step", 0.5))
        self._max_delta_moment = float(asm_cfg.get("max_delta_moment_per_step", 0.01))

        br_cfg = cfg.get("baseline_relative_reward") or {}
        self._baseline_rel_enabled = bool(br_cfg.get("enabled", False))
        self._baseline_cache_rollout = bool(br_cfg.get("cache_zero_rollout_on_reset", True))
        self._include_baseline_obs_cfg = bool(br_cfg.get("include_baseline_reference_in_obs", False))
        if self._baseline_rel_enabled and not self._baseline_cache_rollout:
            raise ValueError("baseline_relative_reward.enabled requires cache_zero_rollout_on_reset: true in this revision")

        self._w_rel_ee = float(rew_cfg.get("w_rel_ee", 0.0))
        self._w_rel_ee_l1 = float(rew_cfg.get("w_rel_ee_l1", 0.0))
        self._w_rel_vel = float(rew_cfg.get("w_rel_vel", 0.0))
        self._w_rel_hf = float(rew_cfg.get("w_rel_hf", 0.0))
        self._w_sat_extra = float(rew_cfg.get("w_sat_extra", 0.0))
        self._w_lim_extra = float(rew_cfg.get("w_lim_extra", 0.0))
        self._use_terminal_rel_final = bool(rew_cfg.get("use_terminal_relative_final_error", False))
        self._w_rel_final = float(rew_cfg.get("w_rel_final", 0.0))

        self._rel_weight_sum = abs(self._w_rel_ee) + abs(self._w_rel_ee_l1) + abs(self._w_rel_vel) + abs(self._w_rel_hf)
        self._use_baseline_relative_reward = bool(self._baseline_rel_enabled and self._rel_weight_sum > 1e-18)

        self._include_baseline_obs = bool(self._include_baseline_obs_cfg)

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

        self._prev_action = np.zeros(self._action_dim, dtype=np.float64)
        self._prev_F = np.zeros(3, dtype=np.float64)
        self._prev_tau_res = np.zeros(4, dtype=np.float64)
        self._Wr_raw = np.zeros(5, dtype=np.float64)
        self._Wr_lim = np.zeros(5, dtype=np.float64)
        self._Wr_filt = np.zeros(5, dtype=np.float64)
        self._Wr_used_full = np.zeros(5, dtype=np.float64)
        self._prev_ee_err_norm = 0.0
        self._prev_ee_err_xyz = np.zeros(3, dtype=np.float64)
        self._prev_ee_dot = np.zeros(3, dtype=np.float64)
        self._e_err_lp = np.zeros(3, dtype=np.float64)
        self._prev_wr_lim_full = np.zeros(5, dtype=np.float64)
        self._W_lp_state = np.zeros(5, dtype=np.float64)
        self._prev_wr_used_full = np.zeros(5, dtype=np.float64)
        self._prev_tau_total = np.zeros(4, dtype=np.float64)

        hz = max(self._highfreq_lp_hz, 1e-3)
        tau_r = 1.0 / (2.0 * np.pi * hz)
        self._e_err_lp_alpha = float(self.control_dt / (tau_r + self.control_dt))

        self._renderer: mj.Renderer | None = None
        self.render_mode: str | None = None

        self._obs_dim = OBS_DIM_BASE + (OBS_EXTRA_BASELINE_REF if self._include_baseline_obs else 0)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self._obs_dim,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(self._action_dim,), dtype=np.float32)

        self._baseline_ok = False
        self._baseline_ee_xyz = np.zeros((max(2, self._max_steps + 1), 3), dtype=np.float64)
        self._baseline_edot = np.zeros_like(self._baseline_ee_xyz)
        self._baseline_ehf = np.zeros_like(self._baseline_ee_xyz)
        self._baseline_sat = np.zeros(self._baseline_ee_xyz.shape[0], dtype=np.int32)
        self._baseline_lim = np.zeros_like(self._baseline_sat)
        self._baseline_tau_tot = np.zeros((self._baseline_ee_xyz.shape[0], 4), dtype=np.float64)
        self._baseline_final_norm = 0.0

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

    def _eef_rpy(self, q_arm: np.ndarray) -> tuple[float, float, float]:
        qv = np.asarray(q_arm, dtype=np.float64).reshape(-1)
        _, r_a, p_a, y_a = fk_ee_rp(self.model, self._scratch, qv, J_ARM, site_name=SITE)
        return float(r_a), float(p_a), float(y_a)

    def _site_omega_world(self) -> np.ndarray:
        mj.mj_forward(self.model, self.data)
        sid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_SITE, SITE)
        jacp = np.zeros((3, self.model.nv), dtype=np.float64)
        jacr = np.zeros((3, self.model.nv), dtype=np.float64)
        mj.mj_jacSite(self.model, self.data, jacp, jacr, sid)
        qd_j = np.array([float(self.data.qvel[int(d)]) for d in self._dadr_j], dtype=np.float64).reshape(-1)
        G = np.asarray(jacr[:, np.asarray(self._dadr_j, dtype=int)], dtype=np.float64)
        out = G @ qd_j
        return np.asarray(out, dtype=np.float64).reshape(3)

    def _orientation_soft_torque(
        self,
        q_j: np.ndarray,
        qd_j: np.ndarray,
        q_des: np.ndarray,
        qd_des: np.ndarray,
    ) -> np.ndarray:
        if self._orient_soft_mode == "none":
            return np.zeros(4, dtype=np.float64)
        self._jac_data.qpos[:] = self.data.qpos
        self._jac_data.qvel[:] = self.data.qvel
        mj.mj_forward(self.model, self._jac_data)
        J5 = compute_task_jacobian_mode(
            self.model,
            self._jac_data,
            joint_names=J_ARM,
            task_mode="xyz_roll_pitch",
            ee_site_name=SITE,
            mode="mujoco_analytic",
            epsilon=1e-6,
        )
        jr = np.asarray(J5[3, :], dtype=np.float64).reshape(4)
        jp = np.asarray(J5[4, :], dtype=np.float64).reshape(4)
        r_act, p_act, _ = self._eef_rpy(q_j)
        r_des, p_des, _ = self._eef_rpy(q_des)
        er = float(angle_error(r_des, r_act))
        ep = float(angle_error(p_des, p_act))
        qv = np.asarray(qd_j, dtype=np.float64).reshape(-1)
        qvd = np.asarray(qd_des, dtype=np.float64).reshape(-1)
        tau_soft = np.zeros(4, dtype=np.float64)
        if self._orient_soft_mode in ("soft_roll", "soft_roll_pitch"):
            rd_m = float(jr @ qv)
            rd_d = float(jr @ qvd)
            fr = float(self._Kp_roll_soft) * er + float(self._Dp_roll_soft) * (rd_d - rd_m)
            tau_soft += jr * fr
        if self._orient_soft_mode == "soft_roll_pitch":
            pd_m = float(jp @ qv)
            pd_d = float(jp @ qvd)
            fp = float(self._Kp_pitch_soft) * ep + float(self._Dp_pitch_soft) * (pd_d - pd_m)
            tau_soft += jp * fp
        return tau_soft

    def _orientation_info_bundle(self, q_arm: np.ndarray, q_des_arm: np.ndarray, omega_world: np.ndarray) -> dict[str, Any]:
        er, ep, ey = self._eef_rpy(q_arm)
        fk_dr, fk_dp, fk_dy = self._eef_rpy(q_des_arm)
        ow = np.asarray(omega_world, dtype=np.float64).reshape(3)
        return {
            "desired_roll_rad": float(ROLL_DES_IK),
            "desired_pitch_rad": float(PITCH_DES_IK),
            "desired_yaw_rad": float("nan"),
            "ee_des_roll_rad_fk": float(fk_dr),
            "ee_des_pitch_rad_fk": float(fk_dp),
            "ee_des_yaw_rad_fk": float(fk_dy),
            "ee_roll_rad": float(er),
            "ee_pitch_rad": float(ep),
            "ee_yaw_rad": float(ey),
            "roll_error_rad": float(angle_error(float(ROLL_DES_IK), float(er))),
            "pitch_error_rad": float(angle_error(float(PITCH_DES_IK), float(ep))),
            "roll_error_rad_vs_qdes_fk": float(angle_error(float(fk_dr), float(er))),
            "pitch_error_rad_vs_qdes_fk": float(angle_error(float(fk_dp), float(ep))),
            "ee_omega_world_x": float(ow[0]),
            "ee_omega_world_y": float(ow[1]),
            "ee_omega_world_z": float(ow[2]),
        }

    def _residual_time_gate(self, t_elapsed: float) -> float:
        """Smoothstep fade of residual force over the last ``final_fade_duration`` of the episode."""
        fade = float(self._final_fade_duration)
        if fade <= 0.0:
            return 1.0
        T = float(self.episode_duration)
        t = float(t_elapsed)
        if t < T - fade:
            return 1.0
        s = float(np.clip((T - t) / fade, 0.0, 1.0))
        return float(3.0 * s * s - 2.0 * s * s * s)

    def _jacobian_5dof_at_current(self) -> np.ndarray:
        self._jac_data.qpos[:] = self.data.qpos
        self._jac_data.qvel[:] = self.data.qvel
        mj.mj_forward(self.model, self._jac_data)
        return analytic_jacobian_5dof(
            self.model,
            self._jac_data,
            joint_names=list(J_ARM),
            site_name=SITE,
        )

    def _compose_residual_wrench_unscaled_gain(
        self,
        a_clip: np.ndarray,
        q_j: np.ndarray,
        qd_j: np.ndarray,
        q_des: np.ndarray,
        qd_des: np.ndarray,
    ) -> np.ndarray:
        """Residual wrench before multiplying ``residual_gain`` (linear in policy action/heuristic)."""
        a_clip = np.clip(np.asarray(a_clip, dtype=np.float64).reshape(-1), -1.0, 1.0)
        W = np.zeros(5, dtype=np.float64)
        if self._residual_type_tag == "task_space_force":
            W[:3] = a_clip[:3] * float(self.residual_force_scale)
            return W
        wm = self._wrench_command_mode
        if wm == "scaled_action_full5":
            if a_clip.shape[0] < 5:
                raise ValueError("scaled_action_full5 needs action length ≥5")
            W[:3] = a_clip[:3] * float(self.residual_force_scale)
            W[3:5] = a_clip[3:5] * float(self.residual_moment_scale)
            return W
        if wm == "sac_xyz_plus_heuristic_rp":
            J5 = self._jacobian_5dof_at_current()
            W[:3] = a_clip[:3] * float(self.residual_force_scale)
            Wh = heuristic_wrench_rp_zeros_force(
                J5,
                roll_des=float(ROLL_DES_IK),
                pitch_des=float(PITCH_DES_IK),
                q_j_arm=q_j,
                qd_j_arm=qd_j,
                q_des_arm=q_des,
                qd_des_arm=qd_des,
                model=self.model,
                fk_data=self._scratch,
                joint_names=list(J_ARM),
                site_name=SITE,
                Kp_roll=self._h_kp_roll,
                Dp_roll=self._h_dp_roll,
                Kp_pitch=self._h_kp_pitch,
                Dp_pitch=self._h_dp_pitch,
            )
            W[3:5] = Wh[3:5]
            return W
        if wm == "heuristic_rp_zeros_force":
            J5 = self._jacobian_5dof_at_current()
            return heuristic_wrench_rp_zeros_force(
                J5,
                roll_des=float(ROLL_DES_IK),
                pitch_des=float(PITCH_DES_IK),
                q_j_arm=q_j,
                qd_j_arm=qd_j,
                q_des_arm=q_des,
                qd_des_arm=qd_des,
                model=self.model,
                fk_data=self._scratch,
                joint_names=list(J_ARM),
                site_name=SITE,
                Kp_roll=self._h_kp_roll,
                Dp_roll=self._h_dp_roll,
                Kp_pitch=self._h_kp_pitch,
                Dp_pitch=self._h_dp_pitch,
            )
        raise RuntimeError(f"unhandled wrench_command_mode {wm}")

    def _postprocess_wr_with_gain_then_slew_lp_gate(self, Wr_pre_gain_linear: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        """Combine linear compose with ``residual_gain``, slew, LP, temporal gate."""

        Wr_pre = np.asarray(Wr_pre_gain_linear, dtype=np.float64).reshape(5)
        Wr_target = (float(self._residual_gain) * Wr_pre).astype(np.float64, copy=False)
        dt = float(self.control_dt)

        if self._action_smooth_enabled:
            delta = Wr_target - self._prev_wr_lim_full
            d_xyz = delta[:3].copy()
            n_xyz = float(np.linalg.norm(d_xyz))
            mx_xyz = float(self._max_delta_force)
            if mx_xyz > 0.0 and n_xyz > mx_xyz and n_xyz > 0.0:
                d_xyz *= mx_xyz / n_xyz
            d_rp = delta[3:].copy()
            n_rp = float(np.linalg.norm(d_rp))
            mx_rp = float(self._max_delta_moment)
            if mx_rp > 0.0 and n_rp > mx_rp and n_rp > 0.0:
                d_rp *= mx_rp / n_rp
            wr_lim = (self._prev_wr_lim_full + np.concatenate([d_xyz, d_rp])).astype(np.float64)
        else:
            wr_lim = Wr_target.astype(np.float64)

        if self._residual_filter_enabled:
            tau_xyz = max(float(self._residual_filter_tau_xyz), 1e-9)
            tau_rp = max(float(self._residual_filter_tau_rp), 1e-9)
            ax = float(np.clip(dt / tau_xyz, 0.0, 1.0))
            ar = float(np.clip(dt / tau_rp, 0.0, 1.0))
            self._W_lp_state[:3] += ax * (wr_lim[:3] - self._W_lp_state[:3])
            self._W_lp_state[3:5] += ar * (wr_lim[3:5] - self._W_lp_state[3:5])
            wr_filt = self._W_lp_state.copy()
        else:
            self._W_lp_state[:] = wr_lim
            wr_filt = wr_lim.astype(np.float64, copy=False)

        self._prev_wr_lim_full[:] = wr_lim
        gate_b = float(self._residual_time_gate(float(self._time)))
        wr_used = (gate_b * wr_filt).astype(np.float64, copy=False)
        return wr_lim, wr_filt, wr_used, gate_b

    def _tau_residual_from_wr_used(self, wr_used: np.ndarray, q_j0: np.ndarray, qd_j0: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return torque (4-vector) and optional aux from DLS internals."""
        Wr = np.asarray(wr_used, dtype=np.float64).reshape(5)
        aux = np.zeros(8, dtype=np.float64)
        if self._residual_type_tag == "task_space_force":
            J = self._jacobian_pos_at()
            tau_geom = joint_torque_from_task_force(J, Wr[:3], self.residual_mode, _extension=self._jacobian_ext)
            return tau_geom.astype(np.float64), aux

        J5 = self._jacobian_5dof_at_current()

        slice_cable = bool(self._wrench_joint_cable_only)
        Jc = slice_joint_columns(J5, cable_only=slice_cable)

        qd_np = np.asarray(qd_j0, dtype=np.float64).reshape(4)
        if slice_cable:
            qd_cols = qd_np[1:4].astype(np.float64)
        else:
            qd_cols = qd_np.copy()

        return residual_torque_from_wrench_5d(
            Jc,
            Wr,
            wrench_map_mode=self._wrench_map_mode,  # type: ignore[arg-type]
            task_weight_diag=np.asarray(self._task_weight_5d, dtype=np.float64),
            cable_only=slice_cable,
            lambda_dls=float(self._lambda_dls),
            K_dls=float(self._K_dls),
            D_dls=float(self._D_dls),
            qd_joint_cols=qd_cols,
        )

    def _episode_snapshot_for_baseline(self) -> dict[str, Any]:
        return {
            "qpos": np.asarray(self.data.qpos, dtype=np.float64).copy(),
            "qvel": np.asarray(self.data.qvel, dtype=np.float64).copy(),
            "time_mu": float(self.data.time),
            "qfrc_applied": np.asarray(self.data.qfrc_applied, dtype=np.float64).copy(),
            "transmission": copy.deepcopy(self._transmission),
            "_time": float(self._time),
            "_step_count": int(self._step_count),
            "_prev_action": self._prev_action.copy(),
            "_prev_F": self._prev_F.copy(),
            "_prev_tau_res": self._prev_tau_res.copy(),
            "_prev_ee_err_norm": float(self._prev_ee_err_norm),
            "_prev_ee_err_xyz": self._prev_ee_err_xyz.copy(),
            "_prev_ee_dot": self._prev_ee_dot.copy(),
            "_e_err_lp": self._e_err_lp.copy(),
            "_prev_wr_lim_full": self._prev_wr_lim_full.copy(),
            "_W_lp_state": self._W_lp_state.copy(),
            "_prev_wr_used_full": self._prev_wr_used_full.copy(),
            "_prev_tau_total": self._prev_tau_total.copy(),
        }

    def _restore_episode_snapshot(self, snap: dict[str, Any]) -> None:
        self.data.qpos[:] = snap["qpos"]
        self.data.qvel[:] = snap["qvel"]
        self.data.time = float(snap["time_mu"])
        self.data.qfrc_applied[:] = snap["qfrc_applied"]
        self._transmission = snap["transmission"]
        self._time = float(snap["_time"])
        self._step_count = int(snap["_step_count"])
        pad_a = np.zeros(self._action_dim, dtype=np.float64)
        pvec = np.asarray(snap["_prev_action"], dtype=np.float64).reshape(-1)
        pad_a[: min(self._action_dim, pvec.shape[0])] = pvec[: min(self._action_dim, pvec.shape[0])]
        self._prev_action[:] = pad_a
        self._prev_F[:] = snap["_prev_F"]
        self._prev_tau_res[:] = snap["_prev_tau_res"]
        self._prev_ee_err_norm = float(snap["_prev_ee_err_norm"])
        self._prev_ee_err_xyz[:] = snap["_prev_ee_err_xyz"]
        self._prev_ee_dot[:] = snap["_prev_ee_dot"]
        self._e_err_lp[:] = snap["_e_err_lp"]
        if "_prev_wr_lim_full" in snap:
            self._prev_wr_lim_full[:] = np.asarray(snap["_prev_wr_lim_full"], dtype=np.float64).reshape(5)
        else:
            self._prev_wr_lim_full[:] = 0.0
            lf = snap.get("_prev_F_limited")
            if lf is not None:
                self._prev_wr_lim_full[:3] = np.asarray(lf, dtype=np.float64).reshape(3)
        if "_W_lp_state" in snap:
            self._W_lp_state[:] = np.asarray(snap["_W_lp_state"], dtype=np.float64).reshape(5)
        elif "_F_lp_state" in snap:
            self._W_lp_state[:] = 0.0
            fl = snap["_F_lp_state"]
            self._W_lp_state[:3] = np.asarray(fl, dtype=np.float64).reshape(3)
        else:
            self._W_lp_state[:] = 0.0
        if "_prev_wr_used_full" in snap:
            self._prev_wr_used_full[:] = np.asarray(snap["_prev_wr_used_full"], dtype=np.float64).reshape(5)
        else:
            self._prev_wr_used_full[:] = 0.0
        self._prev_tau_total[:] = snap["_prev_tau_total"]
        mj.mj_forward(self.model, self.data)

    def _simulate_zero_residual_interval_once(self, loc: dict[str, Any]) -> dict[str, Any]:
        dt = float(self.control_dt)
        Wr_pre_lin = np.zeros(5, dtype=np.float64)
        Wr_target = (Wr_pre_lin * float(self._residual_gain)).astype(np.float64, copy=False)

        prev_lim = np.asarray(loc["Wr_prev_lim"], dtype=np.float64).reshape(5)
        lp_st = np.asarray(loc["Wr_lp_state"], dtype=np.float64).reshape(5)

        if self._action_smooth_enabled:
            delta = Wr_target - prev_lim
            d_xyz = delta[:3].copy()
            n_xyz = float(np.linalg.norm(d_xyz))
            mx_xyz = float(self._max_delta_force)
            if mx_xyz > 0.0 and n_xyz > mx_xyz and n_xyz > 0.0:
                d_xyz *= mx_xyz / n_xyz
            d_rp = delta[3:].copy()
            n_rp = float(np.linalg.norm(d_rp))
            mx_rp = float(self._max_delta_moment)
            if mx_rp > 0.0 and n_rp > mx_rp and n_rp > 0.0:
                d_rp *= mx_rp / n_rp
            wr_lim = (prev_lim + np.concatenate([d_xyz, d_rp])).astype(np.float64)
        else:
            wr_lim = Wr_target.astype(np.float64, copy=False)

        if self._residual_filter_enabled:
            tau_xyz = max(float(self._residual_filter_tau_xyz), 1e-9)
            tau_rp = max(float(self._residual_filter_tau_rp), 1e-9)
            ax = float(np.clip(dt / tau_xyz, 0.0, 1.0))
            ar = float(np.clip(dt / tau_rp, 0.0, 1.0))
            lp_st[:3] += ax * (wr_lim[:3] - lp_st[:3])
            lp_st[3:5] += ar * (wr_lim[3:5] - lp_st[3:5])
            wr_filt = lp_st.copy()
        else:
            lp_st[:] = wr_lim
            wr_filt = lp_st.astype(np.float64, copy=False)

        loc["Wr_prev_lim"][:] = wr_lim
        loc["Wr_lp_state"][:] = lp_st
        gate_b = float(self._residual_time_gate(float(loc["time"])))
        wr_used = (gate_b * wr_filt).astype(np.float64, copy=False)

        t_ctrl = min(float(loc["time"]), self._traj_duration)
        mj.mj_forward(self.model, self.data)
        q_j0, qd_j0, _, _ = self._read_arm_state()
        tau_bias0 = np.array([float(self.data.qfrc_bias[int(d)]) for d in self._dadr_j], dtype=np.float64)
        q_des, qd_des, _, _ = self._ee_desired_kinematics(t_ctrl)
        tau_vsd_nominal = tau_bias0 + self.Kq * (q_des - q_j0) + self.Dq * (qd_des - qd_j0)
        tau_vsd_nominal = tau_vsd_nominal.astype(np.float64, copy=False) + self._orientation_soft_torque(
            q_j0, qd_j0, q_des, qd_des
        )

        tau_geom, _aux = self._tau_residual_from_wr_used(wr_used, q_j0, qd_j0)
        tau_res_cmd = float(self.lambda_residual) * tau_geom
        tau_unclip = tau_vsd_nominal + tau_res_cmd
        tau_jnt_clip = np.clip(tau_unclip, -self.tau_jnt_limit, self.tau_jnt_limit)
        sat_count_step = int(np.any(np.abs(tau_jnt_clip - tau_unclip) > 1e-8))

        tau_act_ideal = RATIOS * tau_jnt_clip
        tout_last = tau_act_ideal.copy()
        hz_last = np.zeros(3, dtype=np.float64)

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

        loc["time"] += self.control_dt
        loc["step_count"] += 1

        mj.mj_forward(self.model, self.data)
        q_j, qd_j, _, _ = self._read_arm_state()
        t_new = min(float(loc["time"]), self._traj_duration)
        q_des_r, qd_des_r, x_des_r, _ = self._ee_desired_kinematics(t_new)
        jac_after = self._jacobian_pos_at()
        xd_act = jac_after @ qd_j
        x_actual = self._site_xyz()
        ee_e = x_des_r - x_actual
        q_a_vec = np.array([float(self.data.qpos[int(adr)]) for adr in self._qadr_a], dtype=np.float64)
        jl_marg = _margins(self.model, J_ARM, q_j)
        al_marg = _margins(self.model, ACT_NAMES, q_a_vec)
        jlv = int(jl_marg < JOINT_LIMIT_REWARD_MARGIN)
        alv = int(al_marg < ACTUATOR_LIMIT_REWARD_MARGIN)
        limit_count_step = int(jlv or alv)

        prev_ee = np.asarray(loc["prev_ee"], dtype=np.float64).reshape(3)
        e_dot = (ee_e - prev_ee) / dt
        e_lp = np.asarray(loc["e_lp"], dtype=np.float64).reshape(3).copy()
        e_lp += self._e_err_lp_alpha * (ee_e - e_lp)
        e_hf = ee_e - e_lp
        loc["e_lp"][:] = e_lp

        breached = joint_or_actuator_severe_breach(self.model, J_ARM, q_j, is_actuator_side=False) or joint_or_actuator_severe_breach(
            self.model,
            ACT_NAMES,
            q_a_vec,
            is_actuator_side=True,
        )
        terminated = False
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            terminated = True
        elif ncon_peak > 0:
            terminated = True
        elif breached:
            terminated = True
        elif float(np.max(np.abs(self.data.qvel))) > QVEL_INSTABILITY_THRESH:
            terminated = True

        loc["prev_ee"][:] = ee_e
        loc["prev_edot"][:] = e_dot
        loc["prev_F"][:] = wr_used[:3]
        loc["prev_tau_res"][:] = tau_res_cmd.copy()
        loc["prev_tau_total"][:] = tau_jnt_clip.copy()

        return {
            "ee_e": ee_e.copy(),
            "e_dot": e_dot.copy(),
            "e_hf": e_hf.copy(),
            "sat": int(sat_count_step),
            "lim": int(limit_count_step),
            "tau_total": tau_jnt_clip.copy(),
            "terminated": bool(terminated),
        }

    def _populate_baseline_from_zero_rollout(self, ee_initial: np.ndarray, tau_total_initial: np.ndarray) -> None:
        if not (self._baseline_rel_enabled and self._baseline_cache_rollout):
            self._baseline_ok = False
            return

        snap = self._episode_snapshot_for_baseline()
        n_rows = self._max_steps + 1
        if self._baseline_ee_xyz.shape[0] != n_rows:
            self._baseline_ee_xyz = np.zeros((n_rows, 3), dtype=np.float64)
            self._baseline_edot = np.zeros((n_rows, 3), dtype=np.float64)
            self._baseline_ehf = np.zeros((n_rows, 3), dtype=np.float64)
            self._baseline_sat = np.zeros(n_rows, dtype=np.int32)
            self._baseline_lim = np.zeros(n_rows, dtype=np.int32)
            self._baseline_tau_tot = np.zeros((n_rows, 4), dtype=np.float64)

        self._baseline_ee_xyz[0] = np.asarray(ee_initial, dtype=np.float64).reshape(3)
        self._baseline_edot[0] = 0.0
        self._baseline_ehf[0] = 0.0
        self._baseline_sat[0] = 0
        self._baseline_lim[0] = 0
        self._baseline_tau_tot[0] = np.asarray(tau_total_initial, dtype=np.float64).reshape(4).copy()

        loc: dict[str, Any] = {
            "time": float(snap["_time"]),
            "step_count": int(snap["_step_count"]),
            "prev_action": snap["_prev_action"].copy(),
            "prev_F": snap["_prev_F"].copy(),
            "prev_tau_res": snap["_prev_tau_res"].copy(),
            "prev_ee": snap["_prev_ee_err_xyz"].copy(),
            "prev_edot": snap["_prev_ee_dot"].copy(),
            "e_lp": snap["_e_err_lp"].copy(),
            "Wr_prev_lim": snap["_prev_wr_lim_full"].copy(),
            "Wr_lp_state": snap["_W_lp_state"].copy(),
            "prev_tau_total": snap["_prev_tau_total"].copy(),
        }

        ok = True
        for _k in range(self._max_steps):
            diag = self._simulate_zero_residual_interval_once(loc)
            idx = int(loc["step_count"])
            if not (1 <= idx < n_rows):
                ok = False
                break
            self._baseline_ee_xyz[idx] = diag["ee_e"]
            self._baseline_edot[idx] = diag["e_dot"]
            self._baseline_ehf[idx] = diag["e_hf"]
            self._baseline_sat[idx] = int(diag["sat"])
            self._baseline_lim[idx] = int(diag["lim"])
            self._baseline_tau_tot[idx] = diag["tau_total"]
            if diag["terminated"]:
                ok = False
                break

        self._restore_episode_snapshot(snap)
        self._baseline_final_norm = float(np.linalg.norm(self._baseline_ee_xyz[n_rows - 1]))
        self._baseline_ok = bool(ok)

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
        assert i == OBS_DIM_BASE
        if obs.shape[0] > OBS_DIM_BASE:
            out[i:] /= 0.1
            assert obs.shape[0] == self._obs_dim
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
        *,
        baseline_row_idx: int,
    ) -> np.ndarray:
        q_err = q_des - q_j
        terr = tau_act_ideal[1:4] - tau_act_out[1:4]
        pa3 = np.zeros(3, dtype=np.float64)
        nk = min(3, int(self._prev_action.shape[0]))
        pa3[:nk] = self._prev_action[:nk]
        parts = [
            q_j,
            qd_j,
            q_des,
            qd_des,
            q_err,
            ee_e,
            ee_ve,
            tau_vsd,
            pa3,
            self._prev_F,
            self._prev_tau_res,
            tau_act_ideal[1:4],
            tau_act_out[1:4],
            terr,
            hys_z_cable,
        ]
        obs = np.concatenate(parts, dtype=np.float64)
        assert obs.shape[0] == OBS_DIM_BASE
        if self._include_baseline_obs:
            if self._baseline_ok:
                kk = min(int(baseline_row_idx), int(self._baseline_ee_xyz.shape[0]) - 1)
                be = np.asarray(self._baseline_ee_xyz[kk], dtype=np.float64).reshape(3)
                d_xyz = ee_e.reshape(3) - be
                n_be = float(np.linalg.norm(be))
                n_dd = float(np.linalg.norm(d_xyz))
                obs = np.concatenate([obs, be, d_xyz, np.array([n_be, n_dd], dtype=np.float64)], dtype=np.float64)
            else:
                obs = np.concatenate([obs, np.zeros(OBS_EXTRA_BASELINE_REF, dtype=np.float64)], dtype=np.float64)
        assert obs.shape[0] == self._obs_dim
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
        self._Wr_raw[:] = 0.0
        self._Wr_lim[:] = 0.0
        self._Wr_filt[:] = 0.0
        self._Wr_used_full[:] = 0.0
        self._prev_wr_lim_full[:] = 0.0
        self._W_lp_state[:] = 0.0
        self._prev_wr_used_full[:] = 0.0
        x_a = self._site_xyz()
        t_path = min(self._time, self._traj_duration)
        q_des, qd_des, x_des, xd_des = self._ee_desired_kinematics(t_path)
        q_j, qd_j, _, _ = self._read_arm_state()
        mj.mj_forward(self.model, self.data)
        tau_bias = np.array([float(self.data.qfrc_bias[int(d)]) for d in self._dadr_j], dtype=np.float64)
        tau_vsd = tau_bias + self.Kq * (q_des - q_j) + self.Dq * (qd_des - qd_j)
        tau_vsd = tau_vsd.astype(np.float64, copy=False) + self._orientation_soft_torque(q_j, qd_j, q_des, qd_des)
        J = self._jacobian_pos_at()
        xd_act = J @ qd_j
        ee_e = x_des - x_a
        ee_ve = xd_des - xd_act
        self._prev_ee_err_norm = float(np.linalg.norm(ee_e))
        self._prev_ee_err_xyz = ee_e.copy()
        self._prev_ee_dot[:] = 0.0
        self._e_err_lp[:] = 0.0
        self._prev_tau_total = tau_vsd.copy()

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

        self._populate_baseline_from_zero_rollout(ee_e, tau_vsd)

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
            baseline_row_idx=0,
        )
        inf0: dict[str, Any] = {
            "time": self._time,
            "ee_des_xyz": np.asarray(x_des, dtype=np.float64).reshape(3).copy(),
            "ee_act_xyz": np.asarray(x_a, dtype=np.float64).reshape(3).copy(),
            "ee_err_xyz": ee_e.copy(),
            "F_residual_raw_xyz": np.zeros(3),
            "F_residual_rate_limited_xyz": np.zeros(3),
            "F_residual_filtered_xyz": np.zeros(3),
            "F_residual_used_xyz": np.zeros(3),
            "residual_gate": 1.0,
            "F_residual_xyz": np.zeros(3),
            "W_residual_raw_5d": np.zeros(5),
            "W_residual_rate_limited_5d": np.zeros(5),
            "W_residual_filtered_5d": np.zeros(5),
            "W_residual_used_5d": np.zeros(5),
            "M_residual_rp": np.zeros(2),
            "ee_err_highfreq_xyz": np.zeros(3),
            "ee_dot": np.zeros(3),
            "tau_residual_jnt": np.zeros(4),
            "tau_vsd": tau_vsd,
            "tau_jnt_cmd": tau_vsd.copy(),
            "tau_act_ideal": np.zeros(4),
            "tau_act_out": tout0.copy(),
            "sampled_cable_params": self._transmission.get_current_params(),
            "reward_total": 0.0,
            "baseline_relative_ok": bool(self._baseline_ok),
        }
        inf0.update(self._orientation_info_bundle(q_j, q_des, self._site_omega_world()))
        return obs, inf0

    def step(self, action: np.ndarray | list[float]):
        """One control step of duration ``control_dt`` (possibly multiple physics substeps)."""
        a = np.clip(np.asarray(action, dtype=np.float64).reshape(self._action_dim), -1.0, 1.0)
        t_ctrl = min(self._time, self._traj_duration)
        q_j0, qd_j0, _, _ = self._read_arm_state()
        mj.mj_forward(self.model, self.data)
        q_des0, qd_des0, _, _ = self._ee_desired_kinematics(t_ctrl)

        Wr_pre_lin = self._compose_residual_wrench_unscaled_gain(a, q_j0, qd_j0, q_des0, qd_des0)
        self._Wr_raw[:] = (float(self._residual_gain) * np.asarray(Wr_pre_lin, dtype=np.float64).reshape(5)).astype(
            np.float64, copy=False
        )
        wr_lim, wr_filt, wr_used, gate = self._postprocess_wr_with_gain_then_slew_lp_gate(Wr_pre_lin)
        self._Wr_lim[:] = wr_lim
        self._Wr_filt[:] = wr_filt
        self._Wr_used_full[:] = wr_used
        W_prev_applied = self._prev_wr_used_full.copy()

        tau_bias0 = np.array([float(self.data.qfrc_bias[int(d)]) for d in self._dadr_j], dtype=np.float64)
        tau_vsd_nominal = tau_bias0 + self.Kq * (q_des0 - q_j0) + self.Dq * (qd_des0 - qd_j0)
        tau_vsd_nominal = tau_vsd_nominal.astype(np.float64, copy=False) + self._orientation_soft_torque(
            q_j0, qd_j0, q_des0, qd_des0
        )

        tau_geom, _aux_tr = self._tau_residual_from_wr_used(wr_used, q_j0, qd_j0)
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

        dt = float(self.control_dt)
        e_dot = (ee_e - self._prev_ee_err_xyz) / dt
        e_ddot = (e_dot - self._prev_ee_dot) / dt
        self._e_err_lp += self._e_err_lp_alpha * (ee_e - self._e_err_lp)
        e_hf = ee_e - self._e_err_lp

        rew_rel_ee = rew_rel_l1 = rew_rel_vel = rew_rel_hf = 0.0
        rew_sat_extra = rew_lim_extra = 0.0
        rew_ee_vel = 0.0
        rew_ee_acc = 0.0
        rew_ee_hf = 0.0
        rew_tau_rate = 0.0
        if self._step_count >= 1:
            rew_ee_vel = -self._w_ee_vel * float(np.dot(e_dot, e_dot))
            rew_ee_hf = -self._w_ee_highfreq * float(np.dot(e_hf, e_hf))
            d_tau_tot = tau_jnt_clip - self._prev_tau_total
            rew_tau_rate = -self._w_tau_rate * float(np.dot(d_tau_tot, d_tau_tot))
        if self._step_count >= 2:
            rew_ee_acc = -self._w_ee_acc * float(np.dot(e_ddot, e_ddot))

        ee_sq = float(np.dot(ee_e, ee_e))
        if self._use_ee_l1:
            rew_ee = -self._w_ee * ee_sq - self._w_ee_l1 * ee_err_norm
        else:
            rew_ee = -self._w_ee * ee_sq
        q_err = q_des_r - q_j
        rew_q = -self._w_q * float(np.dot(q_err, q_err))
        rew_f = -self._w_f * float(np.dot(wr_used, wr_used))
        d_w = wr_used - W_prev_applied
        rew_df = -self._w_df * float(np.dot(d_w, d_w))
        rew_tau = -self._w_tau * float(np.dot(tau_res_cmd, tau_res_cmd))
        rew_sat = -self._w_sat * float(sat_count_step)
        rew_lim = -self._w_lim * float(limit_count_step)
        rew_improve = 0.0

        use_rel = bool(self._use_baseline_relative_reward and self._baseline_ok)
        if use_rel:
            b_idx = min(int(self._step_count), int(self._baseline_ee_xyz.shape[0]) - 1)
            e0 = self._baseline_ee_xyz[b_idx]
            ed0 = self._baseline_edot[b_idx]
            eh0 = self._baseline_ehf[b_idx]
            n0 = float(np.linalg.norm(e0))
            n0_sq = float(np.dot(e0, e0))
            rew_rel_ee = float(self._w_rel_ee) * (n0_sq - ee_sq)
            rew_rel_l1 = float(self._w_rel_ee_l1) * (n0 - ee_err_norm)
            ned0 = float(np.dot(ed0, ed0))
            ned = float(np.dot(e_dot, e_dot))
            rew_rel_vel = float(self._w_rel_vel) * (ned0 - ned)
            neh0 = float(np.dot(eh0, eh0))
            neh = float(np.dot(e_hf, e_hf))
            rew_rel_hf = float(self._w_rel_hf) * (neh0 - neh)
            bsat = int(self._baseline_sat[b_idx])
            blim = int(self._baseline_lim[b_idx])
            rew_sat_extra = -float(self._w_sat_extra) * float(max(0, sat_count_step - bsat))
            rew_lim_extra = -float(self._w_lim_extra) * float(max(0, limit_count_step - blim))
            rew_ee = 0.0
            rew_q = 0.0
            rew_sat = rew_lim = rew_ee_vel = rew_ee_acc = rew_ee_hf = 0.0
            reward = (
                rew_rel_ee
                + rew_rel_l1
                + rew_rel_vel
                + rew_rel_hf
                + rew_f
                + rew_df
                + rew_tau
                + rew_tau_rate
                + rew_sat_extra
                + rew_lim_extra
            )
        else:
            reward = (
                rew_ee + rew_q + rew_f + rew_df + rew_tau + rew_sat + rew_lim
                + rew_ee_vel + rew_ee_acc + rew_ee_hf + rew_tau_rate
            )
            rew_rel_ee = rew_rel_l1 = rew_rel_vel = rew_rel_hf = 0.0
            rew_sat_extra = rew_lim_extra = 0.0
            if self._baseline_rel_enabled and not self._baseline_ok and abs(self._w_ee) < 1e-18:
                rew_ee_fallback = -100.0 * ee_sq
                reward += rew_ee_fallback
                rew_ee = rew_ee_fallback

        if self._use_improvement and not use_rel:
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

        rew_final_ee = 0.0
        rew_final_ee_vel = 0.0
        if truncated and not terminated:
            if use_rel and self._use_terminal_rel_final:
                bf = float(self._baseline_final_norm)
                rew_final_ee = float(self._w_rel_final) * (bf * bf - ee_sq)
                reward += rew_final_ee
            elif self._use_terminal_final_penalty:
                rew_final_ee = -self._w_final_ee * ee_sq
                reward += rew_final_ee
            if (
                self._use_terminal_final_vel_penalty
                and not (use_rel and self._use_terminal_rel_final)
                and self._step_count >= 1
            ):
                rew_final_ee_vel = -self._w_final_ee_vel * float(np.dot(e_dot, e_dot))
                reward += rew_final_ee_vel

        tau_loss_c = tau_hys_c = np.zeros(3, dtype=np.float64)
        trans_err_q = tau_act_ideal[1:4] - tout_last[1:4]

        self._prev_action[:] = np.asarray(a, dtype=np.float64).reshape(self._action_dim)
        self._prev_F[:] = wr_used[:3]
        self._prev_wr_used_full[:] = wr_used
        self._prev_tau_res = tau_res_cmd.astype(np.float64).copy()
        self._prev_ee_err_xyz = ee_e.copy()
        self._prev_ee_dot = e_dot.copy()
        self._prev_tau_total = tau_jnt_clip.astype(np.float64).copy()

        mj.mj_forward(self.model, self.data)
        tau_bias_post = np.array([float(self.data.qfrc_bias[int(d)]) for d in self._dadr_j], dtype=np.float64)
        tau_vsd_obs = tau_bias_post + self.Kq * (q_des_r - q_j) + self.Dq * (qd_des_r - qd_j)
        tau_vsd_obs = tau_vsd_obs.astype(np.float64, copy=False) + self._orientation_soft_torque(
            q_j, qd_j, q_des_r, qd_des_r
        )
        J_obs = self._jacobian_pos_at()
        xd_act_obs = J_obs @ qd_j
        ee_e_obs = x_des_r - self._site_xyz()
        ee_ve_obs = xd_des_r - xd_act_obs

        if cr is not None:
            tau_loss_c = np.array(cr.tau_loss, dtype=np.float64).copy()
            tau_hys_c = np.array(cr.tau_hys, dtype=np.float64).copy()

        b_obs_idx = min(int(self._step_count), max(0, int(self._baseline_ee_xyz.shape[0]) - 1))
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
            baseline_row_idx=b_obs_idx,
        )

        self._prev_ee_err_norm = ee_err_norm

        info: dict[str, Any] = {
            "ee_des_xyz": x_des_r.astype(np.float64).copy(),
            "ee_act_xyz": self._site_xyz().astype(np.float64).copy(),
            "time": float(self._time),
            "ee_error_norm": ee_err_norm,
            "ee_err_xyz": ee_e.copy(),
            "F_residual_raw_xyz": self._Wr_raw[:3].copy(),
            "F_residual_rate_limited_xyz": wr_lim[:3].copy(),
            "F_residual_filtered_xyz": wr_filt[:3].copy(),
            "F_residual_used_xyz": wr_used[:3].copy(),
            "residual_gate": float(gate),
            "F_residual_xyz": wr_used[:3].copy(),
            "W_residual_raw_5d": self._Wr_raw.copy(),
            "W_residual_rate_limited_5d": wr_lim.copy(),
            "W_residual_filtered_5d": wr_filt.copy(),
            "W_residual_used_5d": wr_used.copy(),
            "M_residual_rp": wr_used[3:5].copy(),
            "ee_err_highfreq_xyz": e_hf.copy(),
            "ee_dot": e_dot.copy(),
            "q_error_norm": float(np.linalg.norm(q_err)),
            "tau_residual_jnt": tau_res_cmd.copy(),
            "reward_ee": rew_ee,
            "reward_q": rew_q,
            "reward_force": rew_f,
            "reward_force_rate": rew_df,
            "reward_tau_residual": rew_tau,
            "reward_tau_rate": rew_tau_rate,
            "reward_ee_velocity": rew_ee_vel,
            "reward_ee_acceleration": rew_ee_acc,
            "reward_ee_highfreq": rew_ee_hf,
            "reward_saturation": rew_sat,
            "reward_limit": rew_lim,
            "reward_improvement": rew_improve,
            "reward_rel_ee": float(rew_rel_ee),
            "reward_rel_ee_l1": float(rew_rel_l1),
            "reward_rel_velocity": float(rew_rel_vel),
            "reward_rel_highfreq": float(rew_rel_hf),
            "reward_sat_extra": float(rew_sat_extra),
            "reward_lim_extra": float(rew_lim_extra),
            "reward_final_ee": float(rew_final_ee),
            "reward_final_ee_velocity": float(rew_final_ee_vel),
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
            "baseline_relative_ok": bool(self._baseline_ok),
            "baseline_obs_row": int(b_obs_idx),
        }
        info.update(self._orientation_info_bundle(q_j, q_des_r, self._site_omega_world()))
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
