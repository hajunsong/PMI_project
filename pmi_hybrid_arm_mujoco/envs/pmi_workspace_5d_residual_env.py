"""Workspace 5D VSD (nominal) + SAC residual wrench (5D task), hybrid cable, torque via qfrc_applied."""

from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import gymnasium as gym
import mujoco as mj
import numpy as np
from gymnasium import spaces

from control.task_space_5d import analytic_jacobian_5dof, dls_inverse_right_square
from kinematics.forward_kinematics import fk_ee_rp
from kinematics.orientation_utils import angle_error
from trajectory.joint_quintic import scaled_joint_quintic
from transmission.cable_transmission import (
    CableLayerBacklashDeadzoneConfig,
    CableLayerDelayConfig,
    CableLayerElasticityConfig,
    CableLayerFrictionConfig,
    CableLayerHysteresisConfig,
    CableTransmission,
)
from transmission.hybrid_transmission import HybridTransmission
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_yaml
from envs.workspace_5d_measurement_noise import MeasPacket, MeasurementNoiseRuntime

J_ARM = ["jnt1", "jnt2", "jnt3", "jnt4"]
ACT_NAMES = ["q1_act", "q2_act", "q3_act", "q4_act"]
SITE = "end_effector"
RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=np.float64)
Q_ACT_INITIAL = np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=np.float64)
WPS = (
    np.array([0.25, -0.20, -0.10], dtype=np.float64),
    np.array([0.00, -0.35, -0.15], dtype=np.float64),
    np.array([-0.25, -0.20, -0.10], dtype=np.float64),
)

CABLE_SCALAR: dict[str, float] = {
    "tau_delay": 0.005,
    "viscous_b": 0.001,
    "coulomb_fc": 0.002,
    "tau_elastic": 0.005,
    "tau_deadzone": 0.001,
    "backlash_width": 0.0,
    "backlash_slope": 1.0,
    "hys_alpha": 0.001,
    "hys_A": 1.0,
    "hys_beta": 0.5,
    "hys_gamma": 0.5,
    "hys_n": 1.0,
}

ControlMode = Literal["dls", "jt"]
ResidualMap = Literal["jt", "dls"]

JOINT_LIMIT_REWARD_MARGIN = -0.01
ACTUATOR_LIMIT_REWARD_MARGIN = -0.01
QVEL_INSTABILITY_THRESH = 500.0
LIMIT_BREACH_SEVERE_RAD_JOINT = 0.10
LIMIT_BREACH_SEVERE_RAD_ACTUATOR = 0.15

CABLE_LAYER_YAML = PKG_ROOT / "configs" / "cable_layer.yaml"
DEFAULT_CONFIG = PKG_ROOT / "configs" / "rl_workspace_5d_sac.yaml"


def np3(scalar: float) -> np.ndarray:
    return np.array([float(scalar)] * 3, dtype=np.float64)


def _deep_merge(base: dict[str, Any], over: dict[str, Any]) -> dict[str, Any]:
    out = copy.deepcopy(base)
    for k, v in over.items():
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def make_demo_cable_transmission(full_yaml: dict[str, Any]) -> CableTransmission:
    s = CABLE_SCALAR
    bd = full_yaml["cable_backlash_deadzone"]
    cd = full_yaml["cable_delay"]
    cf = full_yaml["cable_friction"]
    el = full_yaml["cable_elasticity"]
    hy = full_yaml["cable_hysteresis"]
    tau_eps = float(bd["tau_eps"])
    v_eps = float(cf["v_eps"])
    z_max = float(hy["z_max"])
    tau_hys_max = float(hy["tau_hys_max"])
    init_d = bool(cd["initialize_state_to_input"])
    init_e = bool(el["initialize_state_to_input"])
    init_z = bool(hy["initialize_z_to_zero"])
    deadzone_mode = str(bd.get("deadzone_mode", "hard"))
    tau_clip = cf.get("tau_out_clip")
    dcfg = CableLayerDelayConfig(enabled=True, tau_delay_s=np3(s["tau_delay"]), initialize_state_to_input=init_d)
    fcfg = CableLayerFrictionConfig(
        enabled=True,
        viscous_b=np3(s["viscous_b"]),
        coulomb_fc=np3(s["coulomb_fc"]),
        v_eps=float(v_eps),
        tau_out_clip=float(tau_clip) if tau_clip is not None else None,
    )
    ecfg = CableLayerElasticityConfig(
        enabled=True,
        mode="torque_compliance",
        tau_elastic_s=np3(s["tau_elastic"]),
        initialize_state_to_input=init_e,
    )
    bzcfg = CableLayerBacklashDeadzoneConfig(
        enabled=True,
        tau_deadzone=np3(s["tau_deadzone"]),
        backlash_width=np3(s["backlash_width"]),
        backlash_slope=float(s["backlash_slope"]),
        tau_eps=float(tau_eps),
        deadzone_mode=deadzone_mode,
    )
    hcfg = CableLayerHysteresisConfig(
        enabled=True,
        mode="torque_bouc_wen",
        alpha=np3(s["hys_alpha"]),
        A=np3(s["hys_A"]),
        beta=np3(s["hys_beta"]),
        gamma=np3(s["hys_gamma"]),
        n=np3(s["hys_n"]),
        z_max=z_max,
        tau_hys_max=tau_hys_max,
        initialize_z_to_zero=init_z,
    )
    return CableTransmission(dcfg, fcfg, ecfg, bzcfg, hcfg)


def load_hybrid_torque_only(xml: Path) -> mj.MjModel:
    spec = mj.MjSpec.from_file(str(xml))
    for a in list(spec.actuators)[::-1]:
        spec.delete(a)
    return spec.compile()


@dataclass
class AddrBundle:
    qadr_j: np.ndarray
    dadr_j: np.ndarray
    qadr_a: np.ndarray
    dadr_a: np.ndarray


def _addrs(model: mj.MjModel) -> AddrBundle:
    q_j = np.array([int(model.jnt_qposadr[joint_id(model, n)]) for n in J_ARM], dtype=int)
    d_j = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in J_ARM], dtype=int)
    d_a = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in ACT_NAMES], dtype=int)
    q_a = np.array([int(model.jnt_qposadr[joint_id(model, n)]) for n in ACT_NAMES], dtype=int)
    return AddrBundle(q_j, d_j, q_a, d_a)


def jnt_margin(model: mj.MjModel, q: np.ndarray, names: list[str]) -> float:
    lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in names])
    hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in names])
    return float(np.min(np.minimum(hi - q, q - lo)))


def joint_or_actuator_severe_breach(model: mj.MjModel, names: list[str], q: np.ndarray, *, is_actuator_side: bool) -> bool:
    thr = LIMIT_BREACH_SEVERE_RAD_ACTUATOR if is_actuator_side else LIMIT_BREACH_SEVERE_RAD_JOINT
    for nm, qi in zip(names, np.asarray(q).flat):
        ji = joint_id(model, nm)
        lo = float(model.jnt_range[ji, 0])
        hi = float(model.jnt_range[ji, 1])
        if float(qi) < lo - thr or float(qi) > hi + thr:
            return True
    return False


def tau_workspace_vsd(
    J5: np.ndarray,
    *,
    e5: np.ndarray,
    ed5: np.ndarray,
    K_xyz: np.ndarray,
    D_xyz: np.ndarray,
    K_rp: np.ndarray,
    D_rp: np.ndarray,
    rp_weight: float,
    control_mode: ControlMode,
    lambda_dls: float,
    K_dls_joint: float,
    D_dls_joint: float,
    qdot_jnt: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, dict[str, float]]:
    J5 = np.asarray(J5, dtype=np.float64).reshape(5, 4)
    K_vec = np.concatenate([np.asarray(K_xyz).reshape(3), np.asarray(K_rp).reshape(2)])
    D_vec = np.concatenate([np.asarray(D_xyz).reshape(3), np.asarray(D_rp).reshape(2)])
    w_diag = np.array([1.0, 1.0, 1.0, float(rp_weight), float(rp_weight)], dtype=np.float64)

    v_cmd = K_vec * np.asarray(e5).reshape(5) + D_vec * np.asarray(ed5).reshape(5)
    v_w = w_diag * v_cmd

    diag: dict[str, float] = {}
    sv = np.linalg.svd(J5, compute_uv=False)
    diag["sigma_min_J"] = float(sv[-1]) if sv.size else float("nan")
    diag["cond_J"] = float(sv[0] / sv[-1]) if sv.size >= 2 and sv[-1] > 1e-15 else float("nan")
    if control_mode == "jt":
        F_w = v_w.copy()
        tau_t = J5.T @ F_w
        return tau_t, np.zeros(4), diag

    if control_mode == "dls":
        lam2 = float(max(float(lambda_dls), 1e-9)) ** 2
        jp, _ = dls_inverse_right_square(J5, task_weight_row=w_diag, damping_sq=lam2)
        q_dot_cmd = jp @ v_w.reshape(5)
        tau_t = float(K_dls_joint) * q_dot_cmd - float(D_dls_joint) * np.asarray(qdot_jnt).reshape(4)
        return tau_t, q_dot_cmd, diag

    raise ValueError(control_mode)


def residual_torque_from_wrench(
    J5: np.ndarray,
    W_res: np.ndarray,
    w_task: np.ndarray,
    *,
    mapping: ResidualMap,
    lambda_res_dls: float,
    K_res_dls: float,
    D_res_dls: float,
    qdot_jnt: np.ndarray,
) -> np.ndarray:
    J5 = np.asarray(J5, dtype=np.float64).reshape(5, 4)
    W = np.asarray(W_res, dtype=np.float64).reshape(5)
    wt = np.asarray(w_task, dtype=np.float64).reshape(5)
    if mapping == "jt":
        return (J5.T @ (wt * W)).astype(np.float64)
    if mapping == "dls":
        Jw = wt.reshape(5, 1) * J5
        rhs = wt * W
        lam2 = float(max(float(lambda_res_dls), 1e-9)) ** 2
        g = Jw @ Jw.T + lam2 * np.eye(5, dtype=np.float64)
        qdot_res_cmd = Jw.T @ np.linalg.inv(g) @ rhs
        return (float(K_res_dls) * qdot_res_cmd - float(D_res_dls) * np.asarray(qdot_jnt).reshape(4)).astype(
            np.float64
        )
    raise ValueError(mapping)


def compute_observation_dim() -> int:
    task = 3 + 2 + 3 + 2 + 1 + 1
    pose = 3 + 3 + 2 + 2 + 1
    qblk = 4 + 4 + 4 + 4
    ctrl = 4 + 5 + 4
    cable = 3 + 3 + 3 + 3
    jac = 4 + 1
    return task + pose + qblk + ctrl + cable + jac


class PMIWorkspace5DResidualEnv(gym.Env):
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
        base = dict(config) if config is not None else load_yaml(DEFAULT_CONFIG)
        cfg = base if config_path is None else _deep_merge(load_yaml(Path(config_path)), base)
        if overrides:
            cfg = _deep_merge(cfg, overrides)

        self._cfg = cfg
        self.rng = np.random.default_rng() if rng is None else rng

        env_c = cfg["env"]
        vsd_c = cfg["nominal_vsd"]
        res_c = cfg["residual"]
        rew_c = cfg["reward"]

        mp = Path(str(env_c.get("model_path", "models/pmi_hybrid_no_collision.xml")))
        self._model_xml = mp if mp.is_absolute() else PKG_ROOT / mp

        self.randomize_cable = bool(env_c.get("randomize_cable", False))
        self.randomization_profile = str(env_c.get("randomization_profile", "medium_train"))
        self.episode_duration = float(env_c.get("episode_duration", 5.0))
        self.control_dt = float(env_c.get("control_dt", 0.01))
        self.tau_jnt_limit = float(env_c.get("tau_jnt_limit", 30.0))
        self.normalize_observation = bool(env_c.get("normalize_observation", True))

        cur = cfg.get("curriculum") or {}
        stage = str(cur.get("stage", "deterministic"))
        stages = cur.get("stages") or {}
        if stage in stages and isinstance(stages[stage], dict):
            st = stages[stage]
            if "randomize_cable" in st:
                self.randomize_cable = bool(st["randomize_cable"])
            if "randomization_profile" in st:
                self.randomization_profile = str(st["randomization_profile"])
        self._curriculum_stage = stage

        self.control_mode = str(vsd_c.get("control_mode", "jt"))
        if self.control_mode not in ("jt", "dls"):
            raise ValueError(self.control_mode)
        self.lambda_dls = float(vsd_c.get("lambda_dls", 0.12))
        self.rp_weight = float(vsd_c.get("rp_weight", 0.065))
        self.K_xyz = np.asarray(vsd_c.get("K_xyz", [150.0, 150.0, 230.0]), dtype=np.float64).reshape(3)
        self.D_xyz = np.asarray(vsd_c.get("D_xyz", [5.28, 5.28, 5.28]), dtype=np.float64).reshape(3)
        self.K_rp = np.asarray(vsd_c.get("K_rp", [51.5, 51.5]), dtype=np.float64).reshape(2)
        self.D_rp = np.asarray(vsd_c.get("D_rp", [1.15, 1.15]), dtype=np.float64).reshape(2)
        self.K_dls_joint = float(vsd_c.get("K_dls_joint", 4.0))
        self.D_dls_joint = float(vsd_c.get("D_dls_joint", 2.0))

        rtype = str(res_c.get("type", "workspace_5d_wrench"))
        if rtype != "workspace_5d_wrench":
            raise ValueError(f"residual.type must be workspace_5d_wrench, got {rtype}")
        self._residual_mapping = str(res_c.get("residual_mapping", "jt"))
        if self._residual_mapping not in ("jt", "dls"):
            raise ValueError(self._residual_mapping)

        self.residual_force_scale = float(res_c.get("residual_force_scale", 0.5))
        self.residual_moment_scale = float(res_c.get("residual_moment_scale", 0.02))
        self.residual_gain = float(res_c.get("residual_gain", 1.0))
        rp_w = float(res_c.get("residual_rp_weight", 0.1))
        self._W_task_res = np.array([1.0, 1.0, 1.0, rp_w, rp_w], dtype=np.float64)
        self.lambda_res_dls = float(res_c.get("lambda_res_dls", 0.05))
        self.K_res_dls = float(res_c.get("K_res_dls", 2.0))
        self.D_res_dls = float(res_c.get("D_res_dls", 0.5))

        self.residual_filter_enabled = bool(res_c.get("residual_filter_enabled", True))
        self.force_filter_tau = float(res_c.get("force_filter_tau", 0.03))
        self.moment_filter_tau = float(res_c.get("moment_filter_tau", 0.05))
        self.rate_limit_enabled = bool(res_c.get("rate_limit_enabled", True))
        self.max_delta_force_per_step = float(res_c.get("max_delta_force_per_step", 0.1))
        self.max_delta_moment_per_step = float(res_c.get("max_delta_moment_per_step", 0.005))
        self.terminal_fade_enabled = bool(res_c.get("terminal_fade_enabled", True))
        self.final_fade_duration = float(res_c.get("final_fade_duration", 0.3))

        self._w_xyz = float(rew_c.get("w_xyz", 1000.0))
        self._w_xyz_l1 = float(rew_c.get("w_xyz_l1", 20.0))
        self._w_rp = float(rew_c.get("w_rp", 20.0))
        self._w_edot = float(rew_c.get("w_edot", 1.0))
        self._w_res = float(rew_c.get("w_res", 0.01))
        self._w_dres = float(rew_c.get("w_dres", 0.02))
        self._w_tau_res = float(rew_c.get("w_tau_res", 0.005))
        self._w_tau_rate = float(rew_c.get("w_tau_rate", 0.001))
        self._w_sat = float(rew_c.get("w_sat", 1.0))
        self._w_lim = float(rew_c.get("w_lim", 20.0))
        self._w_final_xyz = float(rew_c.get("w_final_xyz", 200.0))
        self._w_final_rp = float(rew_c.get("w_final_rp", 50.0))

        n_raw = cfg.get("noise") or {}
        self._noise = MeasurementNoiseRuntime(
            dict(n_raw),
            self.rng,
            control_dt=float(self.control_dt),
            cutoff_hz=float((n_raw.get("measurement_filter") or {}).get("cutoff_hz", 20.0)),
            actuator_ratios=RATIOS.copy(),
        )

        self._cable_yaml = load_yaml(CABLE_LAYER_YAML)
        self._hybrid_det = HybridTransmission(cable=make_demo_cable_transmission(self._cable_yaml))
        self._hybrid_cfg = HybridTransmission(cable_config=self._cable_yaml)

        self.model = load_hybrid_torque_only(self._model_xml)
        self.data = mj.MjData(self.model)
        self.add = _addrs(self.model)
        self._jac_data = mj.MjData(self.model)
        self._fk_data = mj.MjData(self.model)

        self._sim_dt = float(self.model.opt.timestep)
        self._n_internal = max(1, int(round(self.control_dt / self._sim_dt)))
        err = abs(self._n_internal * self._sim_dt - self.control_dt)
        if err > 1e-6 + 1e-9 * self.control_dt:
            raise ValueError(f"control_dt {self.control_dt} not commensurate with sim_dt {self._sim_dt}")

        self._cart = scaled_joint_quintic(WPS[0], WPS[1], WPS[2], float(self.episode_duration))
        self._max_steps = int(round(self.episode_duration / self.control_dt))

        tau_lp_hz = 1.0 / (2.0 * np.pi * 8.0)
        self._ee_lp_alpha = float(self.control_dt / (tau_lp_hz + self.control_dt))

        self._obs_dim = int(compute_observation_dim())
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self._obs_dim,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(5,), dtype=np.float32)

        self._transmission: HybridTransmission
        self._time = 0.0
        self._step_count = 0
        self._roll_des = 0.0
        self._pitch_des = 0.0
        self._ee_lp = np.zeros(3, dtype=np.float64)
        self._prev_W_used = np.zeros(5, dtype=np.float64)
        self._prev_wr_lim = np.zeros(5, dtype=np.float64)
        self._W_lp_state = np.zeros(5, dtype=np.float64)
        self._prev_tau_jnt_cmd = np.zeros(4, dtype=np.float64)
        self._prev_tau_res = np.zeros(4, dtype=np.float64)

        self._Wr_raw = np.zeros(5, dtype=np.float64)
        self._Wr_limited = np.zeros(5, dtype=np.float64)
        self._Wr_filtered = np.zeros(5, dtype=np.float64)
        self._Wr_used = np.zeros(5, dtype=np.float64)
        self._residual_gate = 1.0

        self._renderer: mj.Renderer | None = None
        self.render_mode: str | None = None

    def _residual_gate_fn(self, t_elapsed: float) -> float:
        if not self.terminal_fade_enabled:
            return 1.0
        fade = float(self.final_fade_duration)
        if fade <= 0.0:
            return 1.0
        T = float(self.episode_duration)
        t = float(t_elapsed)
        if t < T - fade:
            return 1.0
        s = float(np.clip((T - t) / fade, 0.0, 1.0))
        return float(3.0 * s * s - 2.0 * s * s * s)

    def _postprocess_residual(self, W_scaled: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        """W_scaled: physical wrench from action×scale (before residual_gain). Pipeline: gain → rate → LP → gate."""
        dt = float(self.control_dt)
        W_scaled = np.asarray(W_scaled, dtype=np.float64).reshape(5)
        self._Wr_raw[:] = W_scaled
        W_target = (float(self.residual_gain) * W_scaled).astype(np.float64)

        if self.rate_limit_enabled:
            delta = W_target - self._prev_wr_lim
            d_xyz = delta[:3].copy()
            n_xyz = float(np.linalg.norm(d_xyz))
            mx_xyz = float(self.max_delta_force_per_step)
            if mx_xyz > 0.0 and n_xyz > mx_xyz and n_xyz > 0.0:
                d_xyz *= mx_xyz / n_xyz
            d_rp = delta[3:].copy()
            n_rp = float(np.linalg.norm(d_rp))
            mx_rp = float(self.max_delta_moment_per_step)
            if mx_rp > 0.0 and n_rp > mx_rp and n_rp > 0.0:
                d_rp *= mx_rp / n_rp
            wr_lim = (self._prev_wr_lim + np.concatenate([d_xyz, d_rp])).astype(np.float64)
        else:
            wr_lim = W_target.astype(np.float64)

        self._Wr_limited[:] = wr_lim
        self._prev_wr_lim[:] = wr_lim

        if self.residual_filter_enabled:
            tx = max(float(self.force_filter_tau), 1e-9)
            tr = max(float(self.moment_filter_tau), 1e-9)
            ax = float(np.clip(dt / tx, 0.0, 1.0))
            ar = float(np.clip(dt / tr, 0.0, 1.0))
            self._W_lp_state[:3] += ax * (wr_lim[:3] - self._W_lp_state[:3])
            self._W_lp_state[3:5] += ar * (wr_lim[3:5] - self._W_lp_state[3:5])
            wr_filt = self._W_lp_state.copy()
        else:
            self._W_lp_state[:] = wr_lim
            wr_filt = wr_lim.copy()

        self._Wr_filtered[:] = wr_filt
        gate = float(self._residual_gate_fn(float(self._time)))
        self._residual_gate = gate
        wr_used = (gate * wr_filt).astype(np.float64)
        self._Wr_used[:] = wr_used
        return wr_lim, wr_filt, wr_used, gate

    def _read_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        add = self.add
        q_j = np.array([float(self.data.qpos[int(add.qadr_j[k])]) for k in range(4)])
        qd_j = np.array([float(self.data.qvel[int(add.dadr_j[k])]) for k in range(4)])
        q_a = np.array([float(self.data.qpos[int(add.qadr_a[k])]) for k in range(4)])
        qd_a = np.array([float(self.data.qvel[int(add.dadr_a[k])]) for k in range(4)])
        return q_j, qd_j, q_a, qd_a

    def _jacobian_5d_at_qj(self, q_j_arm: np.ndarray) -> np.ndarray:
        q_j_arm = np.asarray(q_j_arm, dtype=np.float64).reshape(4)
        self._jac_data.qpos[:] = self.data.qpos
        self._jac_data.qvel[:] = self.data.qvel
        for k in range(4):
            self._jac_data.qpos[int(self.add.qadr_j[k])] = float(q_j_arm[k])
        mj.mj_forward(self.model, self._jac_data)
        return analytic_jacobian_5dof(self.model, self._jac_data, joint_names=J_ARM, site_name=SITE)

    def _raw_meas_from_true(
        self,
        q_j: np.ndarray,
        qd_j: np.ndarray,
        q_a: np.ndarray,
        qd_a: np.ndarray,
    ) -> MeasPacket:
        del q_a, qd_a
        q_j = np.asarray(q_j, dtype=np.float64).reshape(4)
        qd_j = np.asarray(qd_j, dtype=np.float64).reshape(4)
        ee_xyz, roll_a, pitch_a, yaw_a = fk_ee_rp(self.model, self._fk_data, q_j, J_ARM, site_name=SITE)
        J5 = self._jacobian_5d_at_qj(q_j)
        xd_act = (J5 @ qd_j.reshape(4)).astype(np.float64)
        rr = float(J5[3, :].reshape(4) @ qd_j)
        pp = float(J5[4, :].reshape(4) @ qd_j)
        xd5 = np.array([float(xd_act[0]), float(xd_act[1]), float(xd_act[2]), rr, pp], dtype=np.float64)
        self._noise.update_random_walk()
        qmj, qmdj, ee, r, p, y, xd5n = self._noise.apply_sensor_white(q_j, qd_j, ee_xyz, roll_a, pitch_a, yaw_a, xd5)
        ee, r, p = self._noise.add_ee_bias(ee, r, p)
        return self._noise.assemble_packet(qmj, qmdj, RATIOS, ee, r, p, y, xd5n)

    def _vsd_from_filt_meas(
        self,
        fm: MeasPacket,
        xyz_des: np.ndarray,
        xyzd_des: np.ndarray,
    ) -> tuple[np.ndarray, float, float, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        J5 = self._jacobian_5d_at_qj(fm.q_j)
        e_xyz = np.asarray(xyz_des[:3], dtype=np.float64).reshape(3) - np.asarray(fm.ee_xyz[:3], dtype=np.float64).reshape(3)
        e_roll = angle_error(self._roll_des, float(fm.roll))
        e_pitch = angle_error(self._pitch_des, float(fm.pitch))
        e5 = np.array([float(e_xyz[0]), float(e_xyz[1]), float(e_xyz[2]), float(e_roll), float(e_pitch)], dtype=np.float64)
        xd_des5 = np.zeros(5, dtype=np.float64)
        xd_des5[:3] = np.asarray(xyzd_des[:3]).reshape(3)
        ed5 = xd_des5 - np.asarray(fm.xd5, dtype=np.float64).reshape(5)
        qd_j_m = np.asarray(fm.qd_j, dtype=np.float64).reshape(4)
        tau_bias = np.array([float(self.data.qfrc_bias[int(self.add.dadr_j[k])]) for k in range(4)], dtype=np.float64)
        tau_task, _, _ = tau_workspace_vsd(
            J5,
            e5=e5,
            ed5=ed5,
            K_xyz=self.K_xyz,
            D_xyz=self.D_xyz,
            K_rp=self.K_rp,
            D_rp=self.D_rp,
            rp_weight=self.rp_weight,
            control_mode=self.control_mode,  # type: ignore[arg-type]
            lambda_dls=self.lambda_dls,
            K_dls_joint=self.K_dls_joint,
            D_dls_joint=self.D_dls_joint,
            qdot_jnt=qd_j_m,
        )
        tau_vsd = (tau_bias + tau_task).astype(np.float64)
        return e_xyz, e_roll, e_pitch, ed5, J5, tau_vsd, tau_bias, tau_task, e5

    def _jacobian_5d(self) -> np.ndarray:
        self._jac_data.qpos[:] = self.data.qpos
        self._jac_data.qvel[:] = self.data.qvel
        mj.mj_forward(self.model, self._jac_data)
        return analytic_jacobian_5dof(self.model, self._jac_data, joint_names=J_ARM, site_name=SITE)

    def _build_observation(
        self,
        e_xyz: np.ndarray,
        e_roll: float,
        e_pitch: float,
        edot_xyz: np.ndarray,
        edot_rp: np.ndarray,
        ee_xyz: np.ndarray,
        xyz_des: np.ndarray,
        roll: float,
        pitch: float,
        yaw: float,
        q_j: np.ndarray,
        qd_j: np.ndarray,
        q_a: np.ndarray,
        qd_a: np.ndarray,
        tau_vsd: np.ndarray,
        sig4: np.ndarray,
        cond_j: float,
    ) -> np.ndarray:
        e_xyz = np.asarray(e_xyz, dtype=np.float64).reshape(3)
        erp = np.array([float(e_roll), float(e_pitch)], dtype=np.float64)
        ed_xyz = np.asarray(edot_xyz, dtype=np.float64).reshape(3)
        ed_rp = np.asarray(edot_rp, dtype=np.float64).reshape(2)
        e_norm = float(np.linalg.norm(e_xyz))
        e5 = np.array([e_xyz[0], e_xyz[1], e_xyz[2], float(e_roll), float(e_pitch)], dtype=np.float64)
        e_5d_norm = float(np.linalg.norm(e5))

        parts = [
            e_xyz,
            erp,
            np.clip(ed_xyz, -2.0, 2.0),
            np.clip(ed_rp, -2.0, 2.0),
            np.array([e_norm], dtype=np.float64),
            np.array([e_5d_norm], dtype=np.float64),
            np.asarray(ee_xyz, dtype=np.float64).reshape(3),
            np.asarray(xyz_des, dtype=np.float64).reshape(3),
            np.array([float(roll), float(pitch)], dtype=np.float64),
            np.array([float(self._roll_des), float(self._pitch_des)], dtype=np.float64),
            np.array([float(yaw)], dtype=np.float64),
            np.asarray(q_j, dtype=np.float64).reshape(4),
            np.asarray(qd_j, dtype=np.float64).reshape(4),
            np.asarray(q_a, dtype=np.float64).reshape(4),
            np.asarray(qd_a, dtype=np.float64).reshape(4),
            np.asarray(tau_vsd, dtype=np.float64).reshape(4),
            self._prev_W_used.copy(),
            self._prev_tau_res.copy(),
        ]
        tau_ideal = self._last_tau_act_ideal_q234.copy()
        tau_out = self._last_tau_act_out_q234.copy()
        terr = self._last_trans_err_q234.copy()
        hz = self._last_hys_z_q234.copy()
        parts += [tau_ideal, tau_out, terr, hz]
        s4 = np.asarray(sig4, dtype=np.float64).reshape(4)
        parts += [s4, np.array([float(cond_j)], dtype=np.float64)]

        obs = np.concatenate(parts, dtype=np.float64)
        assert obs.shape[0] == self._obs_dim

        if not self.normalize_observation:
            return obs.astype(np.float32)

        o = obs.copy()
        i = 0
        o[i : i + 3] /= 0.05
        i += 3
        o[i : i + 2] /= 0.2
        i += 2
        o[i : i + 3] /= 0.5
        i += 3
        o[i : i + 2] /= 0.5
        i += 2
        i += 1
        i += 1
        o[i : i + 3] /= 0.5
        i += 3
        o[i : i + 3] /= 0.5
        i += 3
        o[i : i + 2] /= float(np.pi)
        i += 2
        o[i : i + 2] /= float(np.pi)
        i += 2
        o[i : i + 1] /= float(np.pi)
        i += 1
        o[i : i + 4] /= float(np.pi)
        i += 4
        o[i : i + 4] /= 10.0
        i += 4
        o[i : i + 4] /= float(np.pi)
        i += 4
        o[i : i + 4] /= 10.0
        i += 4
        o[i : i + 4] /= self.tau_jnt_limit
        i += 4
        sc_w = np.array(
            [self.residual_force_scale] * 3 + [self.residual_moment_scale] * 2,
            dtype=np.float64,
        )
        o[i : i + 5] /= sc_w
        i += 5
        o[i : i + 4] /= self.tau_jnt_limit
        i += 4
        act_scale = self.tau_jnt_limit * float(np.max(RATIOS))
        o[i : i + 12] /= act_scale
        i += 12
        o[i : i + 4] /= 100.0
        i += 4
        o[i : i + 1] /= 1000.0
        i += 1
        assert i == self._obs_dim
        return o.astype(np.float32)

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
        do_rnd = bool(opts.get("randomize_cable", self.randomize_cable))

        if do_rnd:
            self._transmission = self._hybrid_cfg
            prof = str(opts.get("randomization_profile", self.randomization_profile))
            rseed = int(opts["cable_seed"]) if opts.get("cable_seed") is not None else int(self.rng.integers(0, 2**31 - 1))
            self._transmission.reset(randomize=True, seed=rseed, profile_name=prof)
        else:
            self._transmission = self._hybrid_det
            self._transmission.reset(randomize=False)

        mj.mj_resetData(self.model, self.data)
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        q_act0 = np.asarray(opts.get("q_act_initial", Q_ACT_INITIAL), dtype=np.float64).reshape(4)
        q_j0 = RATIOS * q_act0
        for k in range(4):
            self.data.qpos[int(self.add.qadr_a[k])] = float(q_act0[k])
            self.data.qpos[int(self.add.qadr_j[k])] = float(q_j0[k])
        mj.mj_forward(self.model, self.data)
        self.data.qfrc_applied[:] = 0.0

        _, r0, p0, y0 = fk_ee_rp(self.model, self.data, q_j0, J_ARM, site_name=SITE)
        self._roll_des = float(r0)
        self._pitch_des = float(p0)

        self._time = 0.0
        self._step_count = 0
        self._ee_lp[:] = 0.0
        self._prev_W_used[:] = 0.0
        self._prev_wr_lim[:] = 0.0
        self._W_lp_state[:] = 0.0
        self._prev_tau_res[:] = 0.0
        self._prev_tau_jnt_cmd[:] = 0.0
        self._Wr_raw[:] = 0.0
        self._Wr_limited[:] = 0.0
        self._Wr_filtered[:] = 0.0
        self._Wr_used[:] = 0.0
        self._last_tau_act_ideal_q234 = np.zeros(3, dtype=np.float64)
        self._last_tau_act_out_q234 = np.zeros(3, dtype=np.float64)
        self._last_trans_err_q234 = np.zeros(3, dtype=np.float64)
        self._last_hys_z_q234 = np.zeros(3, dtype=np.float64)

        q_j, qd_j, q_a, qd_a = self._read_state()
        self._noise.rng = self.rng
        self._noise.reset_episode()
        raw0 = self._raw_meas_from_true(q_j, qd_j, q_a, qd_a)
        self._noise.push_raw(raw0)
        fm0 = self._noise.filt_meas
        assert fm0 is not None

        t_cur = min(self._time, float(self.episode_duration))
        xyz_des, xyzd_des, _ = self._cart.sample(float(t_cur))
        ee_xyz, roll_a, pitch_a, yaw_a = fk_ee_rp(self.model, self._fk_data, q_j, J_ARM, site_name=SITE)
        e_xyz_true = np.asarray(xyz_des[:3], dtype=np.float64) - np.asarray(ee_xyz[:3], dtype=np.float64).reshape(3)
        er_true = angle_error(self._roll_des, float(roll_a))
        ep_true = angle_error(self._pitch_des, float(pitch_a))
        self._ee_lp = self._ee_lp + self._ee_lp_alpha * (e_xyz_true - self._ee_lp)
        mj.mj_forward(self.model, self.data)

        e_xyz_m, e_roll_m, e_pitch_m, ed5, J5, tau_vsd, tau_bias, tau_task, _e5 = self._vsd_from_filt_meas(fm0, xyz_des, xyzd_des)
        ed_xyz = ed5[:3].copy()
        ed_rp = ed5[3:5].copy()
        sv = np.linalg.svd(J5, compute_uv=False)
        sig4 = np.zeros(4, dtype=np.float64)
        for i in range(min(4, int(sv.size))):
            sig4[i] = float(sv[i])
        cond_j = float(sv[0] / sv[-1]) if sv.size >= 2 and sv[-1] > 1e-15 else float("nan")

        self._prev_tau_jnt_cmd = np.clip(tau_vsd, -self.tau_jnt_limit, self.tau_jnt_limit).copy()

        obs = self._build_observation(
            e_xyz_m,
            e_roll_m,
            e_pitch_m,
            ed_xyz,
            ed_rp,
            np.asarray(fm0.ee_xyz, dtype=np.float64).reshape(3),
            np.asarray(xyz_des, dtype=np.float64).reshape(3),
            float(fm0.roll),
            float(fm0.pitch),
            float(fm0.yaw),
            fm0.q_j,
            fm0.qd_j,
            fm0.q_a,
            fm0.qd_a,
            tau_vsd,
            sig4,
            cond_j,
        )
        inf = {
            "time": float(self._time),
            "ee_error_norm": float(np.linalg.norm(e_xyz_true)),
            "ee_err_xyz": e_xyz_true.copy(),
            "ee_error_norm_measured": float(np.linalg.norm(e_xyz_m)),
            "ee_err_xyz_measured": e_xyz_m.copy(),
            "ee_des_xyz": np.asarray(xyz_des, dtype=np.float64).reshape(3).copy(),
            "ee_act_xyz": np.asarray(ee_xyz, dtype=np.float64).reshape(3).copy(),
            "e_roll": float(er_true),
            "e_pitch": float(ep_true),
            "e_roll_measured": float(e_roll_m),
            "e_pitch_measured": float(e_pitch_m),
            "roll": float(roll_a),
            "pitch": float(pitch_a),
            "yaw": float(yaw_a),
            "roll_des": float(self._roll_des),
            "pitch_des": float(self._pitch_des),
            "W_residual_raw_5d": np.zeros(5),
            "W_residual_limited_5d": np.zeros(5),
            "W_residual_filtered_5d": np.zeros(5),
            "W_residual_used_5d": np.zeros(5),
            "residual_gate": 1.0,
            "tau_residual_jnt": np.zeros(4),
            "tau_vsd_jnt": tau_vsd.copy(),
            "tau_jnt_cmd": self._prev_tau_jnt_cmd.copy(),
            "tau_task_jnt": tau_task.copy(),
            "tau_bias_jnt": tau_bias.copy(),
            "edot_5d": ed5.copy(),
            "noise_delay_steps": int(self._noise.delay_steps if self._noise.delay_on else 0),
            "noise_log": self._noise.log_settings(),
        }
        return obs, inf

    def step(self, action: np.ndarray | list[float]):
        a = np.clip(np.asarray(action, dtype=np.float64).reshape(5), -1.0, 1.0)
        W_scaled = np.zeros(5, dtype=np.float64)
        W_scaled[:3] = a[:3] * float(self.residual_force_scale)
        W_scaled[3:5] = a[3:5] * float(self.residual_moment_scale)
        W_prev_applied = self._prev_W_used.copy()

        wr_lim, wr_filt, wr_used, gate = self._postprocess_residual(W_scaled)

        t_cur = min(self._time, float(self.episode_duration))
        xyz_des, xyzd_des, _ = self._cart.sample(float(t_cur))

        mj.mj_forward(self.model, self.data)
        q_j, qd_j, q_a, qd_a = self._read_state()

        fm_ctrl = self._noise.filt_meas
        if fm_ctrl is None:
            raise RuntimeError("measurement pipeline not initialized (call reset)")

        _e_xyz_m, _e_roll_m, _e_pitch_m, _ed5, J5, tau_vsd, tau_bias, tau_task, _e5_unused = self._vsd_from_filt_meas(
            fm_ctrl, xyz_des, xyzd_des
        )
        del _e_xyz_m, _e_roll_m, _e_pitch_m, _ed5, _e5_unused

        tau_res = residual_torque_from_wrench(
            J5,
            wr_used,
            self._W_task_res,
            mapping=self._residual_mapping,  # type: ignore[arg-type]
            lambda_res_dls=self.lambda_res_dls,
            K_res_dls=self.K_res_dls,
            D_res_dls=self.D_res_dls,
            qdot_jnt=np.asarray(fm_ctrl.qd_j, dtype=np.float64).reshape(4),
        )
        tau_unclip = tau_vsd + tau_res
        tau_jnt_cmd = np.clip(tau_unclip, -float(self.tau_jnt_limit), float(self.tau_jnt_limit))
        sat_count = int(np.any(np.abs(tau_unclip - tau_jnt_cmd) > 1e-8))

        jl_m = jnt_margin(self.model, q_j, J_ARM)
        al_m = jnt_margin(self.model, q_a, ACT_NAMES)
        jl_viol = int(jl_m < JOINT_LIMIT_REWARD_MARGIN)
        al_viol = int(al_m < ACTUATOR_LIMIT_REWARD_MARGIN)
        limit_count = int(bool(jl_viol or al_viol))

        tau_act_ideal = (RATIOS * tau_jnt_cmd).astype(np.float64)
        tau_ideal_clean, tau_act_noisy_cmd = self._noise.actuator_torques(tau_act_ideal)
        tout_last = tau_act_ideal.copy()
        ncon_peak = 0
        for _ in range(self._n_internal):
            qa, qda = self._read_state()[2], self._read_state()[3]
            tout_last = self._transmission.transmit(tau_act_noisy_cmd, self._sim_dt, qa, qda).copy()
            self.data.qfrc_applied[:] = 0.0
            for k in range(4):
                self.data.qfrc_applied[int(self.add.dadr_a[k])] = float(tout_last[k])
            mj.mj_step(self.model, self.data)
            ncon_peak = max(ncon_peak, int(self.data.ncon))

        self._time += float(self.control_dt)
        self._step_count += 1

        mj.mj_forward(self.model, self.data)
        q_jn, qd_jn, q_an, qd_an = self._read_state()

        t_next = min(self._time, float(self.episode_duration))
        xyz_dn, xyzd_dn, _ = self._cart.sample(float(t_next))
        ee_n, rn, pn, yn = fk_ee_rp(self.model, self._fk_data, q_jn, J_ARM, site_name=SITE)
        e_xyz_n = np.asarray(xyz_dn[:3], dtype=np.float64) - np.asarray(ee_n[:3], dtype=np.float64).reshape(3)
        er_n = angle_error(self._roll_des, float(rn))
        ep_n = angle_error(self._pitch_des, float(pn))

        raw_next = self._raw_meas_from_true(q_jn, qd_jn, q_an, qd_an)
        fm_next = self._noise.push_raw(raw_next)
        e_xyz_mn = np.asarray(xyz_dn[:3], dtype=np.float64).reshape(3) - np.asarray(fm_next.ee_xyz[:3], dtype=np.float64).reshape(3)
        erm_n = angle_error(self._roll_des, float(fm_next.roll))
        epm_mn = angle_error(self._pitch_des, float(fm_next.pitch))

        self._ee_lp = self._ee_lp + self._ee_lp_alpha * (e_xyz_n - self._ee_lp)
        ee_hf = e_xyz_n - self._ee_lp

        J5n = self._jacobian_5d()
        xd_act_n = (J5n @ qd_jn.reshape(4)).astype(np.float64)
        xd_des5_n = np.zeros(5, dtype=np.float64)
        xd_des5_n[:3] = np.asarray(xyzd_dn[:3]).reshape(3)
        rr_n = float(J5n[3, :].reshape(4) @ qd_jn)
        pp_n = float(J5n[4, :].reshape(4) @ qd_jn)
        ed5_n = xd_des5_n - np.array(
            [float(xd_act_n[0]), float(xd_act_n[1]), float(xd_act_n[2]), rr_n, pp_n],
            dtype=np.float64,
        )

        _, _, _, ed5_nm, J5m, tau_vsd_n, _, _, _ = self._vsd_from_filt_meas(
            fm_next, xyz_dn, xyzd_dn
        )
        sv = np.linalg.svd(J5m, compute_uv=False)
        sig4 = np.zeros(4, dtype=np.float64)
        for i in range(min(4, int(sv.size))):
            sig4[i] = float(sv[i])
        cond_j = float(sv[0] / sv[-1]) if sv.size >= 2 and sv[-1] > 1e-15 else float("nan")

        cr = self._transmission.last_cable_result
        if cr is not None:
            self._last_tau_act_ideal_q234 = np.array(tau_ideal_clean[1:4], dtype=np.float64).copy()
            self._last_tau_act_out_q234 = np.array(tout_last[1:4], dtype=np.float64).copy()
            self._last_trans_err_q234 = self._last_tau_act_ideal_q234 - self._last_tau_act_out_q234
            self._last_hys_z_q234 = np.array(cr.hys_z, dtype=np.float64).copy()
        else:
            self._last_tau_act_ideal_q234 = np.array(tau_ideal_clean[1:4], dtype=np.float64).copy()
            self._last_tau_act_out_q234 = np.array(tout_last[1:4], dtype=np.float64).copy()
            self._last_trans_err_q234 = self._last_tau_act_ideal_q234 - self._last_tau_act_out_q234
            self._last_hys_z_q234 = np.zeros(3, dtype=np.float64)

        e_rp = np.array([float(er_n), float(ep_n)], dtype=np.float64)
        rew = (
            -self._w_xyz * float(np.dot(e_xyz_n, e_xyz_n))
            - self._w_xyz_l1 * float(np.linalg.norm(e_xyz_n))
            - self._w_rp * float(np.dot(e_rp, e_rp))
            - self._w_edot * float(np.dot(ed5_n, ed5_n))
            - self._w_res * float(np.dot(wr_used, wr_used))
            - self._w_dres * float(np.dot(wr_used - W_prev_applied, wr_used - W_prev_applied))
            - self._w_tau_res * float(np.dot(tau_res, tau_res))
            - self._w_tau_rate * float(np.dot(tau_jnt_cmd - self._prev_tau_jnt_cmd, tau_jnt_cmd - self._prev_tau_jnt_cmd))
            - self._w_sat * float(sat_count)
            - self._w_lim * float(limit_count)
        )

        q_an_vec = q_an.copy()
        breached = joint_or_actuator_severe_breach(self.model, J_ARM, q_jn, is_actuator_side=False) or joint_or_actuator_severe_breach(
            self.model,
            ACT_NAMES,
            q_an_vec,
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

        truncated = self._step_count >= self._max_steps or self._time >= self.episode_duration - 1e-9

        if truncated and not terminated:
            rew += -self._w_final_xyz * float(np.dot(e_xyz_n, e_xyz_n))
            rew += -self._w_final_rp * float(np.dot(e_rp, e_rp))

        self._prev_W_used[:] = wr_used
        self._prev_tau_res[:] = tau_res.copy()
        self._prev_tau_jnt_cmd[:] = tau_jnt_cmd.copy()

        obs = self._build_observation(
            e_xyz_mn,
            erm_n,
            epm_mn,
            ed5_nm[:3],
            ed5_nm[3:5],
            np.asarray(fm_next.ee_xyz, dtype=np.float64).reshape(3),
            np.asarray(xyz_dn, dtype=np.float64).reshape(3),
            float(fm_next.roll),
            float(fm_next.pitch),
            float(fm_next.yaw),
            fm_next.q_j,
            fm_next.qd_j,
            fm_next.q_a,
            fm_next.qd_a,
            tau_vsd_n,
            sig4,
            cond_j,
        )

        inf: dict[str, Any] = {
            "time": float(self._time),
            "ee_error_norm": float(np.linalg.norm(e_xyz_n)),
            "ee_err_xyz": e_xyz_n.copy(),
            "ee_error_norm_measured": float(np.linalg.norm(e_xyz_mn)),
            "ee_err_xyz_measured": e_xyz_mn.copy(),
            "ee_des_xyz": np.asarray(xyz_dn, dtype=np.float64).reshape(3).copy(),
            "ee_act_xyz": np.asarray(ee_n, dtype=np.float64).reshape(3).copy(),
            "e_roll": float(er_n),
            "e_pitch": float(ep_n),
            "e_roll_measured": float(erm_n),
            "e_pitch_measured": float(epm_mn),
            "roll": float(rn),
            "pitch": float(pn),
            "yaw": float(yn),
            "roll_des": float(self._roll_des),
            "pitch_des": float(self._pitch_des),
            "ee_err_highfreq_xyz": ee_hf.copy(),
            "edot_5d": ed5_n.copy(),
            "W_residual_raw_5d": self._Wr_raw.copy(),
            "W_residual_limited_5d": wr_lim.copy(),
            "W_residual_filtered_5d": wr_filt.copy(),
            "W_residual_used_5d": wr_used.copy(),
            "residual_gate": float(gate),
            "tau_residual_jnt": tau_res.copy(),
            "tau_vsd_jnt": tau_vsd.copy(),
            "tau_jnt_cmd": tau_jnt_cmd.copy(),
            "tau_task_jnt": tau_task.copy(),
            "tau_bias_jnt": tau_bias.copy(),
            "tau_act_ideal": tau_ideal_clean.copy(),
            "tau_act_noisy_cmd": tau_act_noisy_cmd.copy(),
            "tau_act_out": tout_last.copy(),
            "saturation_count": sat_count,
            "joint_limit_violation": jl_viol,
            "actuator_limit_violation": al_viol,
            "ncon": int(ncon_peak),
            "reward_total": float(rew),
            "noise_delay_steps": int(self._noise.delay_steps if self._noise.delay_on else 0),
            "noise_log": self._noise.log_settings(),
        }
        return obs, float(rew), bool(terminated), bool(truncated), inf

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

