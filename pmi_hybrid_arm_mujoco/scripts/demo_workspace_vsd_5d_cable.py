#!/usr/bin/env python3
"""
Deterministic workspace 5D VSD (xyz + roll + pitch, yaw free) with hybrid cable transmission.

- No Gymnasium, no RL/SAC, no training.
- Torque on actuator DOFs only via ``data.qfrc_applied``.
- Model: ``models/pmi_hybrid_no_collision.xml`` (actuators stripped; validated no-collision tree).
- Default gains target ``control_mode=dls`` (joint torque scales with ``K_dls_joint`` and DLS ẋ_cmd);
  very high xyz gains + large ``K_dls_joint`` easily hit ``tau_jnt_limit``; JT mode may need separate tuning.
"""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
import yaml

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
from utils.mujoco_helpers import PKG_ROOT, joint_id, populate_ik_path_overlay_geoms
from utils.smooth_tracking_metrics import episode_smooth_tracking_metrics

HYBRID_XML = PKG_ROOT / "models" / "pmi_hybrid_no_collision.xml"
OUT_DIR = PKG_ROOT / "debug_outputs" / "workspace_vsd_5d_cable"
SWEEP_DIR = OUT_DIR / "sweep"
PLOTS_SUB = "plots"

SITE = "end_effector"
J_ARM = ["jnt1", "jnt2", "jnt3", "jnt4"]
ACT = ["q1_act", "q2_act", "q3_act", "q4_act"]
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

JACOBIAN_RP_DOCUMENTATION = (
    "Task Jacobian rows 4–5 (1-based indexing: rows 5–6) of the analytic 5×4 matrix from "
    "``compute_task_jacobian_mode(..., task_mode='xyz_roll_pitch', mode='mujoco_analytic')`` "
    "map joint rates to **roll rate** and **pitch rate** consistent with ``fk_ee_rp`` "
    "(SciPy extrinsic xyz Euler); **yaw is omitted** from the task."
)


def np3(scalar: float) -> np.ndarray:
    return np.array([float(scalar)] * 3, dtype=np.float64)


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


@dataclass
class AddrBundle:
    qadr_j: np.ndarray
    dadr_j: np.ndarray
    qadr_a: np.ndarray
    dadr_a: np.ndarray


def load_hybrid_torque_only(xml: Path = HYBRID_XML) -> mj.MjModel:
    spec = mj.MjSpec.from_file(str(xml))
    for a in list(spec.actuators)[::-1]:
        spec.delete(a)
    return spec.compile()


def cable_yaml_path() -> Path:
    return PKG_ROOT / "configs" / "cable_layer.yaml"


def addrs(model: mj.MjModel) -> AddrBundle:
    q_j = np.array([int(model.jnt_qposadr[joint_id(model, n)]) for n in J_ARM], dtype=int)
    d_j = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in J_ARM], dtype=int)
    d_a = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in ACT], dtype=int)
    q_a = np.array([int(model.jnt_qposadr[joint_id(model, n)]) for n in ACT], dtype=int)
    return AddrBundle(q_j, d_j, q_a, d_a)


def jnt_margin(model: mj.MjModel, q: np.ndarray, names: list[str]) -> float:
    lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in names])
    hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in names])
    return float(np.min(np.minimum(hi - q, q - lo)))


def pad4_from_cable(v3: np.ndarray, fill_q1: float) -> np.ndarray:
    out = np.zeros(4, dtype=np.float64)
    out[0] = float(fill_q1)
    out[1:4] = np.asarray(v3, dtype=np.float64).reshape(3).copy()
    return out


ControlMode = Literal["dls", "jt"]


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
    """Return ``tau_task (4,), qdot_cmd (4,), diag``."""
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
        diag["sigma_min_Jw"] = float("nan")  # optional refine via SVD of Jw
        tau_t = float(K_dls_joint) * q_dot_cmd - float(D_dls_joint) * np.asarray(qdot_jnt).reshape(4)
        return tau_t, q_dot_cmd, diag

    raise ValueError(control_mode)


def build_csv_headers() -> list[str]:
    h = ["time"]
    for a in ("x", "y", "z"):
        h.append(f"{a}_des")
        h.append(f"{a}_dot_des")
        h.append(f"{a}")
        h.append(f"{a}_dot")
    h += ["roll_des", "pitch_des", "roll", "pitch", "yaw"]
    for p in ("roll_rate", "pitch_rate"):
        h.append(p)
    h += ["e_x", "e_y", "e_z", "e_norm", "e_roll", "e_pitch", "e_5d_norm"]
    for k in ("ee_hf_norm",):
        h.append(k)
    for i in range(1, 5):
        h += [f"q_jnt{i}", f"qdot_jnt{i}", f"q_act{i}", f"qdot_act{i}"]
    for pref in ("tau_bias_jnt", "tau_task_jnt", "tau_jnt_cmd"):
        for i in range(1, 5):
            h.append(f"{pref}{i}")
    for pref in ("tau_act_ideal", "tau_act_out"):
        for i in range(1, 5):
            h.append(f"{pref}_{i}")
    for lab, base in (
        ("tau_after_delay", "tau_ad"),
        ("tau_compliant", "tau_cf"),
        ("tau_after_hysteresis", "tau_ah"),
        ("tau_after_deadzone", "tau_adz"),
        ("tau_after_backlash", "tau_abk"),
        ("tau_loss", "tau_ls"),
        ("tau_hys", "tau_hys"),
        ("hys_z", "hz"),
        ("tau_elastic_err", "tel"),
    ):
        for qi in ("q2", "q3", "q4"):
            h.append(f"{lab}_{qi}")
    h += ["lambda_dls", "rp_weight", "control_mode"]
    for k in ("cond_J", "sigma_min_J", "sigma1", "sigma2", "sigma3", "sigma4"):
        h.append(k)
    for i in range(1, 5):
        h.append(f"qdot_task_cmd{i}")
    h += ["saturation_flag", "joint_limit_viol_flag", "actuator_limit_viol_flag", "ncon"]
    return h


def simulate(
    *,
    duration: float,
    tau_jnt_limit: float,
    control_mode: ControlMode,
    lambda_dls: float,
    rp_weight: float,
    K_xyz: np.ndarray,
    D_xyz: np.ndarray,
    K_rp: np.ndarray,
    D_rp: np.ndarray,
    K_dls_joint: float,
    D_dls_joint: float,
    out_dir: Path,
    save_video: bool,
    video_fps_stride: int,
) -> tuple[list[dict[str, Any]], list[np.ndarray], dict[str, Any]]:
    out_dir.mkdir(parents=True, exist_ok=True)
    plots_dir = out_dir / PLOTS_SUB
    plots_dir.mkdir(parents=True, exist_ok=True)

    model = load_hybrid_torque_only()
    add = addrs(model)
    dt_ctrl = 0.01
    dt = float(model.opt.timestep)
    n_inner = max(1, int(round(dt_ctrl / dt)))
    assert abs(n_inner * dt - dt_ctrl) < 1e-6 + 1e-9 * dt_ctrl

    data = mj.MjData(model)
    jac_w = mj.MjData(model)
    yaml_full = yaml.safe_load(cable_yaml_path().read_text(encoding="utf-8"))
    hybrid = HybridTransmission(cable=make_demo_cable_transmission(yaml_full))

    q_j_init = RATIOS * Q_ACT_INITIAL
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    for i in range(4):
        data.qpos[int(add.qadr_a[i])] = float(Q_ACT_INITIAL[i])
        data.qpos[int(add.qadr_j[i])] = float(q_j_init[i])
    mj.mj_forward(model, data)

    xyz0, roll0_v, pitch0_v, yaw0 = fk_ee_rp(
        model,
        data,
        np.array([data.qpos[int(add.qadr_j[k])] for k in range(4)]),
        J_ARM,
        site_name=SITE,
    )
    roll0 = float(roll0_v)
    pitch0 = float(pitch0_v)

    cart = scaled_joint_quintic(WPS[0], WPS[1], WPS[2], float(duration))

    n_steps = int(np.round(duration / dt_ctrl)) + 1
    rows: list[dict[str, Any]] = []
    frames: list[np.ndarray] = []
    renderer = None
    scene_camera: int | mj.MjvCamera = -1
    if save_video:
        try:
            w, h = 640, 480
            renderer = mj.Renderer(model, width=w, height=h)
            scene_camera = mj.MjvCamera()
            scene_camera.type = mj.mjtCamera.mjCAMERA_FREE
            mj.mjv_defaultFreeCamera(model, scene_camera)
            scene_camera.distance = 1.5
            scene_camera.azimuth = 135
            scene_camera.elevation = -25
        except Exception:
            renderer = None
            scene_camera = -1

    hybrid.reset_cable_state()
    ncon_max = 0
    ee_lp = np.zeros(3)
    tau_lp_hz = 1.0 / (2.0 * np.pi * 8.0)
    alp_ee = float(dt_ctrl / (tau_lp_hz + dt_ctrl))

    sigma_hist: list[np.ndarray] = []

    render_every = max(1, video_fps_stride)

    for step_i in range(n_steps):
        t_cur = min(step_i * dt_ctrl, float(duration))

        xyz_des, xyzd_des, _ = cart.sample(float(t_cur))
        roll_des = roll0
        pitch_des = pitch0

        q_j = np.array([float(data.qpos[int(add.qadr_j[k])]) for k in range(4)])
        qd_j = np.array([float(data.qvel[int(add.dadr_j[k])]) for k in range(4)])
        q_a = np.array([float(data.qpos[int(add.qadr_a[k])]) for k in range(4)])
        qd_a = np.array([float(data.qvel[int(add.dadr_a[k])]) for k in range(4)])

        mj.mj_forward(model, data)
        tau_bias = np.array([float(data.qfrc_bias[int(add.dadr_j[k])]) for k in range(4)])

        jac_w.qpos[:] = data.qpos
        jac_w.qvel[:] = data.qvel
        mj.mj_forward(model, jac_w)
        J5 = analytic_jacobian_5dof(model, jac_w, joint_names=J_ARM, site_name=SITE)
        xd_act = (J5 @ qd_j.reshape(4)).astype(np.float64)

        ee_xyz_act, roll_act, pitch_act, yaw_act = fk_ee_rp(model, data, q_j, J_ARM, site_name=SITE)
        e_xyz = np.asarray(xyz_des[:3]) - np.asarray(ee_xyz_act[:3]).reshape(3)
        ee_lp = ee_lp + alp_ee * (e_xyz - ee_lp)
        ee_hf = e_xyz - ee_lp
        e_roll = angle_error(float(roll_des), float(roll_act))
        e_pitch = angle_error(float(pitch_des), float(pitch_act))
        e5 = np.array([float(e_xyz[0]), float(e_xyz[1]), float(e_xyz[2]), float(e_roll), float(e_pitch)], dtype=np.float64)

        xd_des5 = np.zeros(5, dtype=np.float64)
        xd_des5[:3] = np.asarray(xyzd_des[:3]).reshape(3)
        rr = float(J5[3, :].reshape(4) @ qd_j)
        pp = float(J5[4, :].reshape(4) @ qd_j)
        ed5 = xd_des5 - np.array([float(xd_act[0]), float(xd_act[1]), float(xd_act[2]), rr, pp], dtype=np.float64)

        tau_task, qdot_tc, jd = tau_workspace_vsd(
            J5,
            e5=e5,
            ed5=ed5,
            K_xyz=K_xyz,
            D_xyz=D_xyz,
            K_rp=K_rp,
            D_rp=D_rp,
            rp_weight=rp_weight,
            control_mode=control_mode,
            lambda_dls=lambda_dls,
            K_dls_joint=K_dls_joint,
            D_dls_joint=D_dls_joint,
            qdot_jnt=qd_j,
        )
        tau_bc = tau_bias + tau_task
        tau_cmd = np.clip(tau_bc, -float(tau_jnt_limit), float(tau_jnt_limit))
        sat = bool(np.any(np.abs(tau_bc - tau_cmd) > 1e-9))

        jl_m = jnt_margin(model, q_j, J_ARM)
        al_m = jnt_margin(model, q_a, ACT)
        jl_viol = jl_m < -1e-9
        al_viol = al_m < -1e-9

        tau_act_ideal = RATIOS * tau_cmd
        tau_act_out = hybrid.transmit(tau_act_ideal, dt_ctrl, q_a, qd_a)
        cr = hybrid.last_cable_result

        sig = np.linalg.svd(J5, compute_uv=False)
        sigma_hist.append(sig.copy())

        if cr is not None:
            pad = lambda v3, f: pad4_from_cable(v3, f)
            tau_ad_full = pad(cr.tau_after_delay, tau_act_ideal[0])
            tau_cf_full = pad(cr.tau_compliant, tau_act_ideal[0])
            tau_ah_full = pad(cr.tau_after_hysteresis, tau_act_ideal[0])
            tau_adz_full = pad(cr.tau_after_deadzone, tau_act_ideal[0])
            tau_abk_full = pad(cr.tau_after_backlash, tau_act_ideal[0])
            tau_ls_full = pad(cr.tau_loss, 0.0)
            tau_h_full = pad(cr.tau_hys, 0.0)
            hz_full = pad(cr.hys_z, 0.0)
            tel_full = pad(cr.tau_elastic_error, 0.0)
        else:
            z = np.zeros(4)
            tau_ad_full = tau_act_ideal.copy()
            tau_cf_full = tau_act_ideal.copy()
            tau_ah_full = tau_act_ideal.copy()
            tau_adz_full = tau_act_ideal.copy()
            tau_abk_full = tau_act_ideal.copy()
            tau_ls_full = z.copy()
            tau_h_full = z.copy()
            hz_full = z.copy()
            tel_full = z.copy()

        e_norm = float(np.linalg.norm(e_xyz))
        en5 = float(np.linalg.norm(e5))

        row: dict[str, Any] = {
            "time": float(t_cur),
            **{f"{['x','y','z'][i]}_des": float(xyz_des[i]) for i in range(3)},
            **{f"{['x','y','z'][i]}_dot_des": float(xyzd_des[i]) for i in range(3)},
            "x": float(ee_xyz_act[0]),
            "y": float(ee_xyz_act[1]),
            "z": float(ee_xyz_act[2]),
            **{
                "x_dot": float(xd_act[0]),
                "y_dot": float(xd_act[1]),
                "z_dot": float(xd_act[2]),
            },
            "roll_des": float(roll_des),
            "pitch_des": float(pitch_des),
            "roll": float(roll_act),
            "pitch": float(pitch_act),
            "yaw": float(yaw_act),
            "roll_rate": rr,
            "pitch_rate": pp,
            "e_x": float(e_xyz[0]),
            "e_y": float(e_xyz[1]),
            "e_z": float(e_xyz[2]),
            "e_norm": e_norm,
            "e_roll": float(e_roll),
            "e_pitch": float(e_pitch),
            "e_5d_norm": en5,
            "ee_hf_norm": float(np.linalg.norm(ee_hf)),
            **{f"q_jnt{k+1}": float(q_j[k]) for k in range(4)},
            **{f"qdot_jnt{k+1}": float(qd_j[k]) for k in range(4)},
            **{f"q_act{k+1}": float(q_a[k]) for k in range(4)},
            **{f"qdot_act{k+1}": float(qd_a[k]) for k in range(4)},
            **{f"tau_bias_jnt{k+1}": float(tau_bias[k]) for k in range(4)},
            **{f"tau_task_jnt{k+1}": float(tau_task[k]) for k in range(4)},
            **{f"tau_jnt_cmd{k+1}": float(tau_cmd[k]) for k in range(4)},
            **{f"tau_act_ideal_{k+1}": float(tau_act_ideal[k]) for k in range(4)},
            **{f"tau_act_out_{k+1}": float(tau_act_out[k]) for k in range(4)},
            **{f"tau_after_delay_{qj}": float(tau_ad_full[i]) for qj, i in zip(("q2", "q3", "q4"), (1, 2, 3))},
            **{f"tau_compliant_{qj}": float(tau_cf_full[i]) for qj, i in zip(("q2", "q3", "q4"), (1, 2, 3))},
            **{f"tau_after_hysteresis_{qj}": float(tau_ah_full[i]) for qj, i in zip(("q2", "q3", "q4"), (1, 2, 3))},
            **{f"tau_after_deadzone_{qj}": float(tau_adz_full[i]) for qj, i in zip(("q2", "q3", "q4"), (1, 2, 3))},
            **{f"tau_after_backlash_{qj}": float(tau_abk_full[i]) for qj, i in zip(("q2", "q3", "q4"), (1, 2, 3))},
            **{f"tau_loss_{qj}": float(tau_ls_full[i]) for qj, i in zip(("q2", "q3", "q4"), (1, 2, 3))},
            **{f"tau_hys_{qj}": float(tau_h_full[i]) for qj, i in zip(("q2", "q3", "q4"), (1, 2, 3))},
            **{f"hys_z_{qj}": float(hz_full[i]) for qj, i in zip(("q2", "q3", "q4"), (1, 2, 3))},
            **{f"tau_elastic_err_{qj}": float(tel_full[i]) for qj, i in zip(("q2", "q3", "q4"), (1, 2, 3))},
            "lambda_dls": float(lambda_dls),
            "rp_weight": float(rp_weight),
            "control_mode": str(control_mode),
            "cond_J": jd["cond_J"],
            "sigma_min_J": jd["sigma_min_J"],
            "sigma1": float(sig[0]) if sig.size > 0 else float("nan"),
            "sigma2": float(sig[1]) if sig.size > 1 else float("nan"),
            "sigma3": float(sig[2]) if sig.size > 2 else float("nan"),
            "sigma4": float(sig[3]) if sig.size > 3 else float("nan"),
            **{f"qdot_task_cmd{k+1}": float(qdot_tc[k]) for k in range(4)},
            "saturation_flag": int(sat),
            "joint_limit_viol_flag": int(jl_viol),
            "actuator_limit_viol_flag": int(al_viol),
            "ncon": int(data.ncon),
        }
        rows.append(row)

        if renderer is not None and step_i % render_every == 0:
            renderer.update_scene(data, camera=scene_camera)
            frames.append(np.asarray(renderer.render(), dtype=np.uint8))

        data.qfrc_applied[:] = 0.0
        for k in range(4):
            data.qfrc_applied[int(add.dadr_a[k])] = float(tau_act_out[k])

        for _ in range(n_inner):
            mj.mj_step(model, data)
        ncon_max = max(ncon_max, int(data.ncon))

    meta = {
        "roll0": roll0,
        "pitch0": pitch0,
        "yaw0": float(yaw0),
        "xyz0": xyz0.reshape(3).copy(),
        "ncon_max": int(ncon_max),
        "jacobian_doc": JACOBIAN_RP_DOCUMENTATION,
    }
    return rows, frames, meta


def write_csv(rows: list[dict[str, Any]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = build_csv_headers()
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in keys})


def metrics_pack(rows: list[dict[str, Any]], dt: float) -> dict[str, Any]:
    if not rows:
        return {}
    ee = np.array([[float(r["e_x"]), float(r["e_y"]), float(r["e_z"])] for r in rows])
    en = np.array([float(r["e_norm"]) for r in rows])
    re = np.array([float(r["e_roll"]) for r in rows])
    pe = np.array([float(r["e_pitch"]) for r in rows])
    yaw = np.array([float(r["yaw"]) for r in rows])
    osc = episode_smooth_tracking_metrics(ee_err_xyz=ee, ee_err_norm=en, dt=float(dt))

    tres = [
        float(
            np.linalg.norm(
                [
                    float(r["tau_act_ideal_2"]) - float(r["tau_act_out_2"]),
                    float(r["tau_act_ideal_3"]) - float(r["tau_act_out_3"]),
                    float(r["tau_act_ideal_4"]) - float(r["tau_act_out_4"]),
                ]
            )
        )
        for r in rows
    ]
    tout_arr = np.array([[float(r[f"tau_act_out_{i + 1}"]) for i in range(4)] for r in rows], dtype=np.float64)
    tij = np.array([tuple(float(r[f"tau_jnt_cmd{k}"]) for k in range(1, 5)) for r in rows])

    sg = []
    for r in rows:
        sg.append(
            np.array(
                [float(r["sigma1"]), float(r["sigma2"]), float(r["sigma3"]), float(r["sigma4"])],
                dtype=np.float64,
            )
        )
    SG = np.array(sg)
    return {
        "rms_ee_error": osc.get("rms_ee", float(np.sqrt(np.mean(en**2)))),
        "final_ee_error": float(en[-1]),
        "max_ee_error": float(np.max(en)),
        "rms_x": float(np.sqrt(np.mean(ee[:, 0] ** 2))),
        "rms_y": float(np.sqrt(np.mean(ee[:, 1] ** 2))),
        "rms_z": float(np.sqrt(np.mean(ee[:, 2] ** 2))),
        "rms_roll_error": float(np.sqrt(np.mean(re**2))),
        "rms_pitch_error": float(np.sqrt(np.mean(pe**2))),
        "final_roll_error": float(re[-1]),
        "final_pitch_error": float(pe[-1]),
        "max_abs_roll_error": float(np.max(np.abs(re))),
        "max_abs_pitch_error": float(np.max(np.abs(pe))),
        "yaw_range": float(np.max(yaw) - np.min(yaw)),
        "rms_ee_vel": osc.get("rms_ee_error_velocity", float("nan")),
        "rms_ee_acc": osc.get("rms_ee_error_acceleration", float("nan")),
        "rms_highfreq": osc.get("rms_ee_error_highfreq", float("nan")),
        "p2p_error_norm": osc.get("p2p_error_norm", float("nan")),
        "max_tau_joint": float(np.max(np.abs(tij))),
        "max_tau_act": float(np.max(np.abs(tout_arr))),
        "saturation_steps": int(sum(int(r["saturation_flag"]) for r in rows)),
        "saturation_fraction": float(np.mean([int(r["saturation_flag"]) for r in rows])),
        "jl_steps": int(sum(int(r["joint_limit_viol_flag"]) for r in rows)),
        "act_steps": int(sum(int(r["actuator_limit_viol_flag"]) for r in rows)),
        "jl_frac": float(np.mean([int(r["joint_limit_viol_flag"]) for r in rows])),
        "act_frac": float(np.mean([int(r["actuator_limit_viol_flag"]) for r in rows])),
        "ncon_max": int(max(int(r["ncon"]) for r in rows)),
        "rms_tau_act_error_q234": float(np.sqrt(np.mean(np.array(tres) ** 2))),
        "max_tau_act_error_q234": float(np.max(np.array(tres))) if tres else float("nan"),
        "min_sigma": float(np.min(SG[:, -1])) if SG.size else float("nan"),
        "mean_cond_approx": float(np.nanmean(np.array([float(r["cond_J"]) for r in rows], dtype=np.float64))),
        "max_cond_J": float(np.nanmax(np.array([float(r["cond_J"]) for r in rows], dtype=np.float64))),
    }


def sweep_score(mp: dict[str, Any]) -> float:
    return float(
        float(mp["rms_ee_error"])
        + float(mp["final_ee_error"])
        + 0.5 * float(mp.get("rms_highfreq") or 0.0)
        + 0.2 * float(mp.get("p2p_error_norm") or 0.0)
        + 0.2 * float(mp["saturation_fraction"])
        + 0.2 * float(mp["jl_frac"] + mp["act_frac"])
        + 0.1 * float(mp["rms_roll_error"])
        + 0.1 * float(mp["rms_pitch_error"])
    )


def write_mp4(frames: list[np.ndarray], path: Path, fps: float) -> bool:
    if not frames:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    arr = np.stack(frames, axis=0)
    h_, w_, _ = arr.shape[1:]
    cmd = [
        "ffmpeg", "-y", "-f", "rawvideo", "-vcodec", "rawvideo",
        "-s", f"{w_}x{h_}", "-pix_fmt", "rgb24", "-r", str(fps), "-i", "-",
        "-an", "-vcodec", "libx264", "-pix_fmt", "yuv420p", str(path),
    ]
    try:
        p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
        assert p.stdin is not None
        p.stdin.write(arr.astype(np.uint8).tobytes())
        p.stdin.close()
        p.wait(timeout=600)
        return p.returncode == 0 and path.is_file()
    except (FileNotFoundError, OSError):
        return False


def make_plots(rows: list[dict[str, Any]], plots_dir: Path, meta: dict[str, Any]) -> list[str]:
    plots_dir.mkdir(parents=True, exist_ok=True)
    written: list[str] = []
    if not rows:
        return written
    t = np.array([float(r["time"]) for r in rows])
    try:
        plt.style.use("seaborn-v0_8-whitegrid")
    except OSError:
        pass

    def save(fig: Any, name: str) -> None:
        p = plots_dir / name
        fig.savefig(p, dpi=140)
        plt.close(fig)
        written.append(str(p))

    fig, axes = plt.subplots(3, 1, figsize=(9, 6.5), sharex=True)
    for ax, kk, lab in zip(axes, ("x", "y", "z"), ("x [m]", "y [m]", "z [m]")):
        ax.plot(t, [float(r[f"{kk}_des"]) for r in rows], label="des")
        ax.plot(t, [float(r[kk]) for r in rows], "--", label="act")
        ax.set_ylabel(lab)
        ax.legend(fontsize=7)
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("EE position desired vs actual")
    save(fig, "01_xyz_des_act.png")

    fig, axes = plt.subplots(4, 1, figsize=(9, 7.5), sharex=True)
    for i, kk in enumerate(("e_x", "e_y", "e_z")):
        axes[i].plot(t, [float(r[kk]) for r in rows])
        axes[i].set_ylabel(kk)
    axes[3].plot(t, [float(r["e_norm"]) for r in rows], color="k")
    axes[3].set_ylabel("||e||")
    axes[-1].set_xlabel("time [s]")
    save(fig, "02_ee_errors.png")

    fig, axes = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    axes[0].plot(t, [float(r["roll_des"]) for r in rows], label="des")
    axes[0].plot(t, [float(r["roll"]) for r in rows], "--", label="act")
    axes[0].legend()
    axes[0].set_ylabel("roll [rad]")
    axes[1].plot(t, [float(r["pitch_des"]) for r in rows], label="des")
    axes[1].plot(t, [float(r["pitch"]) for r in rows], "--", label="act")
    axes[1].legend()
    axes[1].set_ylabel("pitch [rad]")
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Roll/pitch (des = initial pose)")
    save(fig, "03_roll_pitch_des_act.png")

    fig, ax = plt.subplots(figsize=(9, 2.8))
    ax.plot(t, [float(r["e_roll"]) for r in rows], label="e_roll")
    ax.plot(t, [float(r["e_pitch"]) for r in rows], label="e_pitch")
    ax.legend()
    ax.set_xlabel("time [s]")
    save(fig, "04_roll_pitch_error.png")

    fig, ax = plt.subplots(figsize=(9, 2.8))
    ax.plot(t, [float(r["yaw"]) for r in rows], color="C2")
    ax.set_title("yaw (not in task)")
    ax.set_xlabel("time [s]")
    ax.text(0.02, 0.95, "yaw not controlled", transform=ax.transAxes, va="top", fontsize=10)
    save(fig, "05_yaw_free.png")

    fig = plt.figure(figsize=(7, 6))
    try:
        ax = fig.add_subplot(111, projection="3d")
        xd = [float(r["x_des"]) for r in rows]
        yd = [float(r["y_des"]) for r in rows]
        zd = [float(r["z_des"]) for r in rows]
        xa = [float(r["x"]) for r in rows]
        ya = [float(r["y"]) for r in rows]
        za = [float(r["z"]) for r in rows]
        ax.plot(xd, yd, zd, label="des", lw=2)
        ax.plot(xa, ya, za, label="act", lw=1.3, alpha=0.85)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.legend()
    except Exception:
        ax = fig.add_subplot(111)
        ax.plot([float(r["x_des"]) for r in rows], [float(r["y_des"]) for r in rows], label="des")
        ax.plot([float(r["x"]) for r in rows], [float(r["y"]) for r in rows], label="act")
        ax.legend()
        ax.set_title("path (xy fallback)")
    fig.tight_layout()
    save(fig, "06_path_3d.png")

    fig, axes = plt.subplots(4, 1, figsize=(9, 8), sharex=True)
    for i in range(4):
        axes[i].plot(t, [float(r[f"q_jnt{i+1}"]) for r in rows])
        axes[i].set_ylabel(f"q{i+1}")
    axes[-1].set_xlabel("time [s]")
    save(fig, "07_q_jnt.png")

    fig, axes = plt.subplots(4, 1, figsize=(9, 8), sharex=True)
    for i in range(4):
        axes[i].plot(t, [float(r[f"tau_jnt_cmd{i+1}"]) for r in rows])
        axes[i].set_ylabel(f"τ{i+1}")
    axes[-1].set_xlabel("time [s]")
    save(fig, "08_tau_jnt_cmd.png")

    fig, axes = plt.subplots(3, 1, figsize=(9, 6), sharex=True)
    for ii, iq in enumerate((2, 3, 4)):
        axes[ii].plot(t, [float(r[f"tau_act_ideal_{iq}"]) for r in rows], label="ideal")
        axes[ii].plot(t, [float(r[f"tau_act_out_{iq}"]) for r in rows], "--", label="out")
        axes[ii].legend(fontsize=7)
        axes[ii].set_ylabel(f"tau q{iq}_act")
    axes[-1].set_xlabel("time [s]")
    save(fig, "09_tau_act_ideal_vs_out_q234.png")

    fig, axes = plt.subplots(3, 1, figsize=(9, 6.5), sharex=True)
    for ii, suf in enumerate(("q2", "q3", "q4")):
        axes[ii].plot(t, [float(r[f"hys_z_{suf}"]) for r in rows], label="hys_z")
        axes[ii].plot(t, [float(r[f"tau_loss_{suf}"]) for r in rows], label="tau_loss")
        axes[ii].plot(t, [float(r[f"tau_hys_{suf}"]) for r in rows], label="tau_hys")
        axes[ii].legend(fontsize=6, ncol=3)
        axes[ii].set_ylabel(suf)
    axes[-1].set_xlabel("time [s]")
    save(fig, "10_cable_states.png")

    fig, axes = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    axes[0].plot(t, [float(r["cond_J"]) for r in rows], label="cond(J)")
    axes[0].set_ylabel("cond")
    axes[0].legend()
    for k in ("sigma1", "sigma2", "sigma3", "sigma4"):
        axes[1].plot(t, [float(r[k]) for r in rows], label=k)
    axes[1].legend(fontsize=7, ncol=2)
    axes[1].set_ylabel("σ")
    axes[-1].set_xlabel("time [s]")
    save(fig, "11_dls_singular_values.png")

    fig, ax = plt.subplots(3, 1, figsize=(9, 6), sharex=True)
    ax0 = ax[0]
    ax0.plot(t, [float(r["saturation_flag"]) for r in rows], drawstyle="steps-post")
    ax0.set_ylabel("sat")
    ax[1].plot(t, [float(r["joint_limit_viol_flag"]) for r in rows], drawstyle="steps-post")
    ax[1].set_ylabel("j lim")
    ax[2].plot(t, [float(r["actuator_limit_viol_flag"]) for r in rows], drawstyle="steps-post")
    ax[2].set_ylabel("a lim")
    ax[2].set_xlabel("time [s]")
    save(fig, "12_flags.png")

    fig, ax = plt.subplots(figsize=(9, 2.8))
    ax.plot(t, [float(r["ee_hf_norm"]) for r in rows])
    ax.set_ylabel("||e_hf||")
    ax.set_xlabel("time [s]")
    ax.set_title("High-frequency EE error (8 Hz LP residue)")
    save(fig, "13_hf_ee_err.png")

    fig, axes = plt.subplots(2, 2, figsize=(9, 5))
    ee_n = np.array([float(r["e_norm"]) for r in rows])
    axes[0, 0].plot(t, ee_n)
    axes[0, 0].set_title("||e||")
    axes[0, 1].plot(t, np.abs(np.array([float(r["e_roll"]) for r in rows])))
    axes[0, 1].set_title("|e_roll|")
    axes[1, 0].plot(t, np.abs(np.array([float(r["e_pitch"]) for r in rows])))
    axes[1, 0].set_title("|e_pitch|")
    dv = np.diff(ee_n) / float(np.median(np.diff(t)))
    axes[1, 1].plot(t[1:], np.abs(dv))
    axes[1, 1].set_title("|d||e||/dt| approx")
    for ax in axes.ravel():
        ax.set_xlabel("t")
    fig.tight_layout()
    save(fig, "14_summary_dashboard.png")

    # Path (2D projections) + EE pose + errors on one page
    fig, axes2 = plt.subplots(3, 2, figsize=(9, 10.2))
    xd = np.array([float(r["x_des"]) for r in rows])
    yd = np.array([float(r["y_des"]) for r in rows])
    zd = np.array([float(r["z_des"]) for r in rows])
    xa = np.array([float(r["x"]) for r in rows])
    ya = np.array([float(r["y"]) for r in rows])
    za = np.array([float(r["z"]) for r in rows])
    ax_xy = axes2[0, 0]
    ax_xy.plot(xd, yd, label="des", lw=2)
    ax_xy.plot(xa, ya, "--", label="act", lw=1.3, alpha=0.9)
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("Path XY (des vs act)")
    ax_xy.legend(fontsize=8)
    ax_xy.grid(True, alpha=0.35)
    ax_xz = axes2[0, 1]
    ax_xz.plot(xd, zd, label="des", lw=2)
    ax_xz.plot(xa, za, "--", label="act", lw=1.3, alpha=0.9)
    ax_xz.set_aspect("equal", adjustable="box")
    ax_xz.set_xlabel("x [m]")
    ax_xz.set_ylabel("z [m]")
    ax_xz.set_title("Path XZ (des vs act)")
    ax_xz.legend(fontsize=8)
    ax_xz.grid(True, alpha=0.35)
    ax_r = axes2[1, 0]
    ax_r.plot(t, [float(r["roll_des"]) for r in rows], label="roll des")
    ax_r.plot(t, [float(r["roll"]) for r in rows], "--", label="roll act")
    ax_r.set_ylabel("roll [rad]")
    ax_r.set_title("EE pose: roll (des = initial)")
    ax_r.legend(fontsize=7)
    ax_r.grid(True, alpha=0.25)
    ax_p = axes2[1, 1]
    ax_p.plot(t, [float(r["pitch_des"]) for r in rows], label="pitch des")
    ax_p.plot(t, [float(r["pitch"]) for r in rows], "--", label="pitch act")
    ax_p.set_ylabel("pitch [rad]")
    ax_p.set_title("EE pose: pitch (des = initial)")
    ax_p.legend(fontsize=7)
    ax_p.grid(True, alpha=0.25)
    ex = np.array([float(r["e_x"]) for r in rows])
    ey = np.array([float(r["e_y"]) for r in rows])
    ez = np.array([float(r["e_z"]) for r in rows])
    en = np.array([float(r["e_norm"]) for r in rows])
    ax_ex = axes2[2, 0]
    ax_ex.plot(t, ex, label="e_x", lw=1.0)
    ax_ex.plot(t, ey, label="e_y", lw=1.0)
    ax_ex.plot(t, ez, label="e_z", lw=1.0)
    ax_ex.plot(t, en, color="k", lw=1.3, label="||e||")
    ax_ex.set_ylabel("[m]")
    ax_ex.set_xlabel("time [s]")
    ax_ex.set_title("Position error (XYZ + norm)")
    ax_ex.legend(fontsize=7, ncol=2)
    ax_ex.grid(True, alpha=0.35)
    ax_eo = axes2[2, 1]
    ax_eo.plot(t, [float(r["e_roll"]) for r in rows], label="e_roll", lw=1.0)
    ax_eo.plot(t, [float(r["e_pitch"]) for r in rows], label="e_pitch", lw=1.0)
    ax_eo.set_ylabel("[rad]")
    ax_eo.set_xlabel("time [s]")
    ax_eo.set_title("Orientation error (roll / pitch)")
    ax_eo.legend(fontsize=7)
    ax_eo.grid(True, alpha=0.35)
    axes2[1, 0].sharex(axes2[2, 0])
    axes2[1, 1].sharex(axes2[2, 1])
    plt.setp(axes2[1, 0].get_xticklabels(), visible=False)
    plt.setp(axes2[1, 1].get_xticklabels(), visible=False)
    fig.suptitle("Path, EE pose, and tracking errors", y=1.01, fontsize=11)
    fig.tight_layout()
    save(fig, "15_path_vs_ee_pose.png")
    return written


def write_report(
    path: Path,
    *,
    args_ns: argparse.Namespace,
    metrics: dict[str, Any],
    meta: dict[str, Any],
    csv_path: Path,
    plot_paths: list[str],
    video_path: Path | None,
) -> None:
    vid_line = (f"| Video | `{video_path}` |" if video_path and video_path.is_file() else "| Video | (없음) |")
    lines = [
        "# Workspace 5D VSD demo (hybrid cable)",
        "",
        f"- **모델**: `{HYBRID_XML}` (STL 충돌 비활성, 검증 무충돌 트리; 액추에이터 제거 후 torque-only)",
        "| 항목 | 값 |",
        "|:---|:---|",
        f"| Jacobian roll/pitch rows | {JACOBIAN_RP_DOCUMENTATION} |",
        f"| 케이블 파라미터 | deterministic (delay/friction/elasticity/backlash+hysteresis) — 코드 상단 `CABLE_SCALAR` 참고 |",
        f"| 궤적 | 카테시안 Quintic waypoint (0→0.5→1)×duration, 사용자 duration={args_ns.duration}s |",
        f"| 초기 roll/pitch (명목 자세 유지 목표) | roll0={meta['roll0']:.6f} rad, pitch0={meta['pitch0']:.6f} rad |",
        f"| 제어 모드 | `{args_ns.control_mode}` |",
        f"| λ_DLS | {args_ns.lambda_dls:g} |",
        f"| roll/pitch task weight | {args_ns.rp_weight:g} |",
        f"| τ joint limit | {args_ns.tau_jnt_limit:g} N·m |",
        f"| 시간 CSV | `{csv_path}` |",
        vid_line,
        "",
        "## 메트릭",
        "| key | value |",
        "|:---|---:|",
    ]
    for k in sorted(metrics):
        lines.append(f"| {k} | {metrics[k]} |")
    lines += ["", "## 생성 플롯", *[f"- `{p}`" for p in plot_paths], "", "## 질문 (요약 답변)", ]
    lines += [
        f"1. **workspace VSD가 xyz 경로를 케이블 포함으로 추종하는가?** RMS EE 오차 약 **{metrics['rms_ee_error']:.6f} m**, 최종 **{metrics['final_ee_error']:.6f} m** (게인·토크 한계에 민감).",
        f"2. **roll/pitch가 초기값 근처로 유지되는가?** RMS roll/pitch 오차 각각 **{metrics['rms_roll_error']:.6g}**, **{metrics['rms_pitch_error']:.6g} rad** (소프트 가중·4DOF 때문에 큰 편향 가능).",
        f"3. **yaw는 자유인가?** 과제 벡터에 미포함; 기록 범위 **{metrics['yaw_range']:.6g} rad**.",
        f"4. **DLS가 과제 과제제약 불안정을 완화하는가?** `max_cond_J≈{metrics['max_cond_J']:.4g}`, `sigma_min≈{metrics['min_sigma']:.4g}`; 포화 많으면 **λ·w_rp·τ_lim** 조정 필요.",
        f"5. **진폭 큰 진동?** RMS HF(|e|)~**{metrics.get('rms_highfreq', float('nan')):.6g}**, P2P(||e||)~**{metrics.get('p2p_error_norm', float('nan')):.6g}**.",
        f"6. **포화/한계 위반?** sat_frac≈**{metrics['saturation_fraction']:.6g}**, joint/actuator 근처 위반 평균 합≈**{metrics['jl_frac'] + metrics['act_frac']:.6g}**.",
        f"7. **충돌 접촉(ncon)?** 시뮬 중 최대 `ncon` = **{metrics['ncon_max']}**.",
        "8. **기존 IK 관절 VSD와 비교** — 과제표현이 다르므로 숫자 직비는 무의미; 동일 시간·케이블·한계선에서 교차 검증 권장.",
        "9. **다른 모드/튜닝** — `dls`는 `τ_task = K_q·Jp·v_w − D_q·q̇`라 동일 과제 게인에서도 토크가 크게 증폭됨(`K_xyz`·`K_dls_joint` 연동). `jt`는 `Jᵀ` 저게인 토크라 동일 과제 게인이면 추종이 둔해질 수 있음.",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        epilog="`--sweep` 는 많은 조합(수백 회)을 연속 실행하므로 매우 오래 걸릴 수 있습니다. 기본은 단일 데모입니다.",
    )
    p.add_argument("--duration", type=float, default=5.0, help="궤적 지속 시간 [s]")
    p.add_argument("--control-mode", choices=("dls", "jt"), default="jt")
    p.add_argument("--lambda-dls", type=float, default=0.12)
    p.add_argument("--rp-weight", type=float, default=0.065)
    p.add_argument("--tau-jnt-limit", type=float, default=30.0)
    p.add_argument("--save-video", action="store_true", default=True)
    p.add_argument("--video-fps", type=float, default=30.0)
    p.add_argument("--video-stride", type=int, default=3)
    p.add_argument("--out-dir", type=Path, default=None)
    p.add_argument("--sweep", action="store_true")
    # gains
    p.add_argument("--K-xyz", type=float, nargs=3, default=(150.0, 150.0, 230.0))
    p.add_argument("--D-xyz", type=float, nargs=3, default=(5.28, 5.28, 5.28))
    p.add_argument("--K-rp", type=float, nargs=2, default=(51.5, 51.5))
    p.add_argument("--D-rp", type=float, nargs=2, default=(1.15, 1.15))
    p.add_argument("--K-dls-joint", type=float, default=4.0)
    p.add_argument("--D-dls-joint", type=float, default=2.0)
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.sweep:
        return sweep_main(args)

    out = Path(args.out_dir) if args.out_dir else OUT_DIR
    out.mkdir(parents=True, exist_ok=True)

    rows, frames, meta = simulate(
        duration=float(args.duration),
        tau_jnt_limit=float(args.tau_jnt_limit),
        control_mode=args.control_mode,
        lambda_dls=float(args.lambda_dls),
        rp_weight=float(args.rp_weight),
        K_xyz=np.asarray(args.K_xyz, dtype=np.float64),
        D_xyz=np.asarray(args.D_xyz, dtype=np.float64),
        K_rp=np.asarray(args.K_rp, dtype=np.float64),
        D_rp=np.asarray(args.D_rp, dtype=np.float64),
        K_dls_joint=float(args.K_dls_joint),
        D_dls_joint=float(args.D_dls_joint),
        out_dir=out,
        save_video=bool(args.save_video),
        video_fps_stride=max(1, int(args.video_stride)),
    )

    csv_p = out / "workspace_vsd_5d_cable_timeseries.csv"
    write_csv(rows, csv_p)
    mp = metrics_pack(rows, 0.01)
    plots = make_plots(rows, out / PLOTS_SUB, meta)
    vp: Path | None = None
    if args.save_video and frames:
        vp = out / "workspace_vsd_5d_cable.mp4"
        if not write_mp4(frames, vp, fps=float(args.video_fps)):
            vp = None
    write_report(out / "workspace_vsd_5d_cable_report.md", args_ns=args, metrics=mp, meta=meta, csv_path=csv_p, plot_paths=plots, video_path=vp)
    print(csv_p)
    print(out / "workspace_vsd_5d_cable_report.md")
    return 0


def sweep_main(args: argparse.Namespace) -> int:
    SWEEP_DIR.mkdir(parents=True, exist_ok=True)
    combos: list[tuple[float, float, float, float, str]] = []
    for dur in (5.0, 7.0, 10.0):
        for tau_lim in (30.0, 40.0, 50.0):
            for lam in (0.03, 0.05, 0.1):
                for w_rp in (0.05, 0.1, 0.2):
                    for mode in ("dls", "jt"):
                        combos.append((dur, tau_lim, lam, w_rp, mode))

    csv_out = SWEEP_DIR / "workspace_vsd_5d_sweep_results.csv"
    rep_out = SWEEP_DIR / "workspace_vsd_5d_sweep_report.md"

    agg: list[dict[str, Any]] = []
    for dur, tau_lim, lam, w_rp, mode in combos:
        sub = SWEEP_DIR / f"c_{dur}_{tau_lim}_{lam}_{w_rp}_{mode}".replace(".", "p")
        rows, _, meta = simulate(
            duration=float(dur),
            tau_jnt_limit=float(tau_lim),
            control_mode=mode,
            lambda_dls=float(lam),
            rp_weight=float(w_rp),
            K_xyz=np.asarray(args.K_xyz, dtype=np.float64),
            D_xyz=np.asarray(args.D_xyz, dtype=np.float64),
            K_rp=np.asarray(args.K_rp, dtype=np.float64),
            D_rp=np.asarray(args.D_rp, dtype=np.float64),
            K_dls_joint=float(args.K_dls_joint),
            D_dls_joint=float(args.D_dls_joint),
            out_dir=sub,
            save_video=False,
            video_fps_stride=999,
        )
        mp = metrics_pack(rows, 0.01)
        mp["score"] = sweep_score(mp)
        mp.update(
            {
                "duration": dur,
                "tau_jnt_limit": tau_lim,
                "lambda_dls": lam,
                "rp_weight": w_rp,
                "control_mode": mode,
                "roll0": meta["roll0"],
            }
        )
        agg.append(mp)

    agg_sorted = sorted(agg, key=lambda d: float(d["score"]))[:10]
    keys = sorted(agg[0].keys()) if agg else []
    with csv_out.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for r in agg:
            w.writerow({k: r.get(k, "") for k in keys})

    lines = ["# Sweep (top 10 by score)", "", "| rank | dur | τlim | λ | w_rp | mode | score | RMS EE |", "|---:|---:|---:|---:|---:|:---|---:|---:|"]
    for i, r in enumerate(agg_sorted, start=1):
        lines.append(
            f"| {i} | {r['duration']} | {r['tau_jnt_limit']} | {r['lambda_dls']} | "
            f"{r['rp_weight']} | {r['control_mode']} | {r['score']:.6f} | {r['rms_ee_error']:.6g} |"
        )
    rep_out.write_text("\n".join(lines), encoding="utf-8")
    print(csv_out, rep_out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
