#!/usr/bin/env python3
"""
Deterministic task-space VSD trajectory tracking (Jacobian transpose) on the hybrid arm.
Model: collision-excluded MJCF + belt (q1) + cable pipeline (q2–q4). Torque enters actuator DOFs.
No RL/SAC/training — simulation + CSV + PNG plots + optional MP4/GIF + optional MuJoCo passive viewer.
"""

from __future__ import annotations

import argparse
import csv
import shutil
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

from kinematics.orientation_utils import angle_error
from kinematics.task_jacobian import compute_task_jacobian_mode, fk_task_y
from trajectory.joint_quintic import scaled_joint_quintic
from transmission.cable_transmission import (
    CableLayerBacklashDeadzoneConfig,
    CableLayerDelayConfig,
    CableLayerElasticityConfig,
    CableLayerFrictionConfig,
    CableLayerHysteresisConfig,
    CableTransmission,
)
from transmission.cable_transmission import cable_transmission_identity as make_identity_cable
from transmission.hybrid_transmission import HybridTransmission
from utils.mujoco_helpers import PKG_ROOT, joint_id, passive_mujoco_viewer, populate_ik_path_overlay_geoms

HYBRID_XML = PKG_ROOT / "models" / "pmi_hybrid_no_collision.xml"
OUT_DIR = PKG_ROOT / "debug_outputs" / "task_space_demo"
PLOTS_DIR = OUT_DIR / "plots"

SITE = "end_effector"
J_ARM = ["jnt1", "jnt2", "jnt3", "jnt4"]
ACT = ["q1_act", "q2_act", "q3_act", "q4_act"]
RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=np.float64)

Q_ACT_INITIAL = np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=np.float64)
Q_JNT_INITIAL = RATIOS * Q_ACT_INITIAL

WPS = [
    np.array([0.25, -0.20, -0.10], dtype=np.float64),
    np.array([0.00, -0.35, -0.15], dtype=np.float64),
    np.array([-0.25, -0.20, -0.10], dtype=np.float64),
]

ROLL_DES = -np.pi / 2.0
PITCH_DES = 0.0

JacobianModePick = Literal["numerical", "mujoco_analytic"]

CABLE_SCALAR: dict[str, float] = dict(
    tau_delay=0.005,
    viscous_b=0.001,
    coulomb_fc=0.002,
    tau_elastic=0.005,
    tau_deadzone=0.001,
    backlash_width=0.0,
    backlash_slope=1.0,
    hys_alpha=0.001,
    hys_A=1.0,
    hys_beta=0.5,
    hys_gamma=0.5,
    hys_n=1.0,
)

DEFAULT_KP_XYZ = np.array([80.0, 80.0, 80.0], dtype=np.float64)
DEFAULT_DP_XYZ = np.array([12.0, 12.0, 8.0], dtype=np.float64)
DEFAULT_KP_RP = np.array([20.0, 20.0], dtype=np.float64)
DEFAULT_DP_RP = np.array([2.0, 2.0], dtype=np.float64)


def np3(scalar: float) -> np.ndarray:
    return np.array([float(scalar)] * 3, dtype=np.float64)


def cable_yaml_path() -> Path:
    return PKG_ROOT / "configs" / "cable_layer.yaml"


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


def load_hybrid_torque_only() -> mj.MjModel:
    spec = mj.MjSpec.from_file(str(HYBRID_XML))
    for a in list(spec.actuators)[::-1]:
        spec.delete(a)
    return spec.compile()


def _addrs(model: mj.MjModel) -> AddrBundle:
    q_j = np.array([int(model.jnt_qposadr[joint_id(model, n)]) for n in J_ARM], dtype=int)
    d_j = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in J_ARM], dtype=int)
    d_a = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in ACT], dtype=int)
    q_a = np.array([int(model.jnt_qposadr[joint_id(model, n)]) for n in ACT], dtype=int)
    return AddrBundle(q_j, d_j, q_a, d_a)


def _jnt_margin(model: mj.MjModel, q: np.ndarray, names: list[str]) -> float:
    lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in names])
    hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in names])
    return float(np.min(np.minimum(hi - q, q - lo)))


def ee_xyz(model: mj.MjModel, data: mj.MjData) -> np.ndarray:
    sid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, SITE)
    return np.array(data.site_xpos[sid], dtype=np.float64).copy()


def pad4_from_cable(v3: np.ndarray, q1_fill: float) -> np.ndarray:
    out = np.zeros(4, dtype=np.float64)
    out[0] = float(q1_fill)
    out[1:4] = np.asarray(v3, dtype=np.float64).reshape(3).copy()
    return out


def fieldnames_flat() -> list[str]:
    base = ["time"]
    base += ["ee_des_x", "ee_des_y", "ee_des_z", "ee_act_x", "ee_act_y", "ee_act_z"]
    base += ["ee_err_x", "ee_err_y", "ee_err_z", "ee_err_norm"]
    base += ["ee_vel_des_x", "ee_vel_des_y", "ee_vel_des_z"]
    base += ["ee_vel_act_x", "ee_vel_act_y", "ee_vel_act_z"]
    base += ["ee_vel_err_x", "ee_vel_err_y", "ee_vel_err_z"]
    base += ["roll_des", "pitch_des", "roll_act", "pitch_act", "roll_err", "pitch_err"]
    for i in range(4):
        base.append(f"q_jnt_{i + 1}")
    for i in range(4):
        base.append(f"qdot_jnt_{i + 1}")
    for i in range(4):
        base.append(f"q_act_{i + 1}")
    for i in range(4):
        base.append(f"qdot_act_{i + 1}")
    for i in range(4):
        base.append(f"tau_bias_jnt_{i + 1}")
    for i in range(4):
        base.append(f"tau_task_jnt_{i + 1}")
    for i in range(4):
        base.append(f"tau_jnt_cmd_{i + 1}")
    for i in range(4):
        base.append(f"tau_act_ideal_{i + 1}")
    for i in range(4):
        base.append(f"tau_act_out_{i + 1}")
    for i in range(4):
        base.append(f"tau_after_delay_{i + 1}")
    for i in range(4):
        base.append(f"tau_after_hysteresis_{i + 1}")
    for i in range(4):
        base.append(f"tau_after_deadzone_{i + 1}")
    for i in range(4):
        base.append(f"tau_after_backlash_{i + 1}")
    for i in range(4):
        base.append(f"tau_loss_{i + 1}")
    for i in range(4):
        base.append(f"hys_z_{i + 1}")
    for i in range(4):
        base.append(f"trans_pos_err_{i + 1}")
    base += ["saturation_flag", "joint_limit_viol_flag", "actuator_limit_viol_flag", "ncon", "dataset_label"]
    return base


def run_rollout(
    *,
    model: mj.MjModel,
    hybrid: HybridTransmission,
    duration: float,
    tau_jnt_limit: float,
    kp_xyz: np.ndarray,
    dp_xyz: np.ndarray,
    kp_rp: np.ndarray,
    dp_rp: np.ndarray,
    jacobian_mode: JacobianModePick,
    dataset_label: str,
    collect_frames_every: int,
    renderer: mj.Renderer | None,
    camera_id: int,
    sim_data: mj.MjData | None = None,
    viewer: Any | None = None,
    viewer_realtime_scale: float = 1.0,
    viewer_overlay: bool = True,
) -> tuple[list[dict[str, Any]], list[np.ndarray], int]:
    data = mj.MjData(model) if sim_data is None else sim_data
    jac_w = mj.MjData(model)
    add = _addrs(model)
    dt = float(model.opt.timestep)

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    for i in range(4):
        data.qpos[int(add.qadr_a[i])] = float(Q_ACT_INITIAL[i])
        data.qpos[int(add.qadr_j[i])] = float(Q_JNT_INITIAL[i])
    mj.mj_forward(model, data)
    data.qfrc_applied[:] = 0.0

    hybrid.reset_cable_state()
    cart = scaled_joint_quintic(WPS[0], WPS[1], WPS[2], float(duration))

    n_steps = int(np.round(duration / dt)) + 1
    rows: list[dict[str, Any]] = []
    frames: list[np.ndarray] = []

    kp_xyz = np.asarray(kp_xyz, dtype=np.float64).reshape(3)
    dp_xyz = np.asarray(dp_xyz, dtype=np.float64).reshape(3)
    kp_rp = np.asarray(kp_rp, dtype=np.float64).reshape(2)
    dp_rp = np.asarray(dp_rp, dtype=np.float64).reshape(2)

    ncon_peak_run = 0
    render_stride = max(1, int(collect_frames_every))
    capture = renderer is not None and collect_frames_every > 0

    for step_i in range(n_steps):
        t_cur = min(step_i * dt, float(duration))

        xyz_des, xyzd_des, _ = cart.sample(float(t_cur))

        q_j = np.array([float(data.qpos[int(add.qadr_j[k])]) for k in range(4)])
        qd_j = np.array([float(data.qvel[int(add.dadr_j[k])]) for k in range(4)])
        q_a = np.array([float(data.qpos[int(add.qadr_a[k])]) for k in range(4)])
        qd_a = np.array([float(data.qvel[int(add.dadr_a[k])]) for k in range(4)])

        mj.mj_forward(model, data)
        tau_bias = np.array([float(data.qfrc_bias[int(add.dadr_j[k])]) for k in range(4)])

        jac_w.qpos[:] = data.qpos
        jac_w.qvel[:] = data.qvel
        mj.mj_forward(model, jac_w)
        J = compute_task_jacobian_mode(
            model,
            jac_w,
            joint_names=J_ARM,
            task_mode="xyz_roll_pitch",
            ee_site_name=SITE,
            mode=jacobian_mode,
            epsilon=1e-6,
        )

        y_act = fk_task_y(model, data, q_j, J_ARM, "xyz_roll_pitch", site_name=SITE)
        ee_act = ee_xyz(model, data)
        xd_act = J @ qd_j

        e_xyz = np.asarray(xyz_des[:3]) - ee_act[:3]
        xd_des5 = np.zeros(5, dtype=np.float64)
        xd_des5[:3] = xyzd_des[:3]
        e_vel_5 = xd_des5 - xd_act

        roll_act = float(y_act[3])
        pitch_act = float(y_act[4])
        e_roll = angle_error(float(ROLL_DES), roll_act)
        e_pitch = angle_error(float(PITCH_DES), pitch_act)
        e_rp = np.array([e_roll, e_pitch], dtype=np.float64)

        F = np.zeros(5, dtype=np.float64)
        F[:3] = kp_xyz * e_xyz + dp_xyz * e_vel_5[:3]
        F[3:5] = kp_rp * e_rp + dp_rp * e_vel_5[3:5]

        tau_task = J.T @ F
        tau_bc = tau_bias + tau_task
        tau_cmd = np.clip(tau_bc, -float(tau_jnt_limit), float(tau_jnt_limit))
        sat = bool(np.any(np.abs(tau_bc - tau_cmd) > 1e-9))

        tau_act_ideal = RATIOS * tau_cmd
        tau_act_out = hybrid.transmit(tau_act_ideal, dt, q_a, qd_a)
        cr = hybrid.last_cable_result

        jl_m = _jnt_margin(model, q_j, J_ARM)
        al_m = _jnt_margin(model, q_a, ACT)
        jl_viol = jl_m < -1e-9
        al_viol = al_m < -1e-9
        rq = RATIOS * q_a
        trans_pos = q_j - rq

        if capture and (step_i % render_stride == 0):
            assert renderer is not None
            renderer.update_scene(data, camera=camera_id)
            frames.append(np.asarray(renderer.render(), dtype=np.uint8))

        if cr is not None:
            t_ad = pad4_from_cable(cr.tau_after_delay, tau_act_ideal[0])
            t_ah = pad4_from_cable(cr.tau_after_hysteresis, tau_act_ideal[0])
            t_adz = pad4_from_cable(cr.tau_after_deadzone, tau_act_ideal[0])
            t_abk = pad4_from_cable(cr.tau_after_backlash, tau_act_ideal[0])
            t_ls = pad4_from_cable(cr.tau_loss, 0.0)
            hz4 = pad4_from_cable(cr.hys_z, 0.0)
        else:
            t_ad = tau_act_ideal.copy()
            t_ah = tau_act_ideal.copy()
            t_adz = tau_act_ideal.copy()
            t_abk = tau_act_ideal.copy()
            t_ls = np.zeros(4)
            hz4 = np.zeros(4)

        row: dict[str, Any] = {
            "time": float(t_cur),
            "ee_des_x": float(xyz_des[0]),
            "ee_des_y": float(xyz_des[1]),
            "ee_des_z": float(xyz_des[2]),
            "ee_act_x": float(ee_act[0]),
            "ee_act_y": float(ee_act[1]),
            "ee_act_z": float(ee_act[2]),
            "ee_err_x": float(e_xyz[0]),
            "ee_err_y": float(e_xyz[1]),
            "ee_err_z": float(e_xyz[2]),
            "ee_err_norm": float(np.linalg.norm(e_xyz)),
            "ee_vel_des_x": float(xyzd_des[0]),
            "ee_vel_des_y": float(xyzd_des[1]),
            "ee_vel_des_z": float(xyzd_des[2]),
            "ee_vel_act_x": float(xd_act[0]),
            "ee_vel_act_y": float(xd_act[1]),
            "ee_vel_act_z": float(xd_act[2]),
            "ee_vel_err_x": float(e_vel_5[0]),
            "ee_vel_err_y": float(e_vel_5[1]),
            "ee_vel_err_z": float(e_vel_5[2]),
            "roll_des": float(ROLL_DES),
            "pitch_des": float(PITCH_DES),
            "roll_act": float(roll_act),
            "pitch_act": float(pitch_act),
            "roll_err": float(e_roll),
            "pitch_err": float(e_pitch),
            **{f"q_jnt_{i + 1}": float(q_j[i]) for i in range(4)},
            **{f"qdot_jnt_{i + 1}": float(qd_j[i]) for i in range(4)},
            **{f"q_act_{i + 1}": float(q_a[i]) for i in range(4)},
            **{f"qdot_act_{i + 1}": float(qd_a[i]) for i in range(4)},
            **{f"tau_bias_jnt_{i + 1}": float(tau_bias[i]) for i in range(4)},
            **{f"tau_task_jnt_{i + 1}": float(tau_task[i]) for i in range(4)},
            **{f"tau_jnt_cmd_{i + 1}": float(tau_cmd[i]) for i in range(4)},
            **{f"tau_act_ideal_{i + 1}": float(tau_act_ideal[i]) for i in range(4)},
            **{f"tau_act_out_{i + 1}": float(tau_act_out[i]) for i in range(4)},
            **{f"tau_after_delay_{i + 1}": float(t_ad[i]) for i in range(4)},
            **{f"tau_after_hysteresis_{i + 1}": float(t_ah[i]) for i in range(4)},
            **{f"tau_after_deadzone_{i + 1}": float(t_adz[i]) for i in range(4)},
            **{f"tau_after_backlash_{i + 1}": float(t_abk[i]) for i in range(4)},
            **{f"tau_loss_{i + 1}": float(t_ls[i]) for i in range(4)},
            **{f"hys_z_{i + 1}": float(hz4[i]) for i in range(4)},
            **{f"trans_pos_err_{i + 1}": float(trans_pos[i]) for i in range(4)},
            "saturation_flag": int(sat),
            "joint_limit_viol_flag": int(jl_viol),
            "actuator_limit_viol_flag": int(al_viol),
            "ncon": int(data.ncon),
            "dataset_label": dataset_label,
        }
        rows.append(row)

        data.qfrc_applied[:] = 0.0
        for k in range(4):
            data.qfrc_applied[int(add.dadr_a[k])] = float(tau_act_out[k])

        mj.mj_step(model, data)
        ncon_peak_run = max(ncon_peak_run, int(data.ncon))

        if viewer is not None:
            if not viewer.is_running():
                break
            ee_viz = ee_xyz(model, data)
            if viewer_overlay:
                with viewer.lock():
                    populate_ik_path_overlay_geoms(
                        viewer.user_scn,
                        np.vstack(WPS),
                        np.asarray(xyz_des[:3], dtype=np.float64),
                        ee_viz,
                    )
            viewer.sync()
            if viewer_realtime_scale > 0.0:
                time.sleep(float(viewer_realtime_scale) * dt)

    return rows, frames, ncon_peak_run


def metrics_from_rows(rows: list[dict[str, Any]]) -> dict[str, float | int]:
    if not rows:
        return {}

    ee_n = np.array([float(r["ee_err_norm"]) for r in rows])
    roll_e = np.array([float(r["roll_err"]) for r in rows])
    pitch_e = np.array([float(r["pitch_err"]) for r in rows])
    tj = np.array([[float(r[f"tau_jnt_cmd_{i + 1}"]) for i in range(4)] for r in rows])
    ta = np.array([[float(r[f"tau_act_out_{i + 1}"]) for i in range(4)] for r in rows])
    tr = np.array([[float(r[f"trans_pos_err_{i + 1}"]) for i in range(4)] for r in rows])

    return {
        "rms_ee_pos": float(np.sqrt(np.mean(ee_n**2))),
        "final_ee_pos_norm": float(ee_n[-1]),
        "max_ee_pos_norm": float(np.max(ee_n)),
        "rms_roll_err": float(np.sqrt(np.mean(roll_e**2))),
        "rms_pitch_err": float(np.sqrt(np.mean(pitch_e**2))),
        "max_torque_joint": float(np.max(np.abs(tj))),
        "max_torque_act": float(np.max(np.abs(ta))),
        "max_trans_pos_err_component": float(np.max(np.abs(tr))),
        "saturation_steps": int(sum(int(r["saturation_flag"]) for r in rows)),
        "jl_viol_steps": int(sum(int(r["joint_limit_viol_flag"]) for r in rows)),
        "al_viol_steps": int(sum(int(r["actuator_limit_viol_flag"]) for r in rows)),
    }


def write_mp4_via_ffmpeg(frames: list[np.ndarray], path: Path, fps: float) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not frames:
        return False
    arr = np.stack(frames, axis=0)
    h, w, _ = arr.shape[1:]
    cmd = [
        "ffmpeg",
        "-y",
        "-f",
        "rawvideo",
        "-vcodec",
        "rawvideo",
        "-s",
        f"{w}x{h}",
        "-pix_fmt",
        "rgb24",
        "-r",
        str(fps),
        "-i",
        "-",
        "-an",
        "-vcodec",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(path),
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


def maybe_write_gif(frames: list[np.ndarray], path: Path, fps: float) -> bool:
    try:
        from PIL import Image  # noqa: PLC0415
    except ImportError:
        return False
    if not frames:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    ims = [Image.fromarray(f) for f in frames]
    duration_ms = max(10, int(1000.0 / max(fps, 1e-6)))
    ims[0].save(path, save_all=True, append_images=ims[1:], duration=duration_ms, loop=0)
    return path.is_file()


def plots_from_csv(rows: list[dict[str, Any]], plot_dir: Path = PLOTS_DIR) -> None:
    plot_dir.mkdir(parents=True, exist_ok=True)
    if not rows:
        return

    try:
        plt.style.use("seaborn-v0_8-whitegrid")
    except OSError:
        pass

    t = np.array([float(r["time"]) for r in rows], dtype=float)

    def xy_lines(fname: str, labels: tuple[str, ...], keys_des: tuple[str, ...], keys_act: tuple[str, ...]) -> None:
        fig, axes = plt.subplots(len(labels), 1, figsize=(8, 2.8 * len(labels)), sharex=True)
        if len(labels) == 1:
            axes = np.array([axes])
        assert isinstance(axes, np.ndarray)
        for ax, lab, kd, ka in zip(axes, labels, keys_des, keys_act):
            ax.plot(t, [float(r[kd]) for r in rows], label="desired")
            ax.plot(t, [float(r[ka]) for r in rows], "--", label="actual")
            ax.set_ylabel(lab)
            ax.legend(fontsize=8)
        axes[-1].set_xlabel("time [s]")
        fig.tight_layout()
        fig.savefig(plot_dir / fname, dpi=150)
        plt.close(fig)

    xy_lines(
        "01_ee_xyz.png",
        ("x [m]", "y [m]", "z [m]"),
        ("ee_des_x", "ee_des_y", "ee_des_z"),
        ("ee_act_x", "ee_act_y", "ee_act_z"),
    )

    fig, ax = plt.subplots(figsize=(8, 3))
    ax.plot(t, [float(r["ee_err_norm"]) for r in rows], color="tab:red")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("‖e_pos‖ [m]")
    ax.set_title("EE position error norm")
    fig.tight_layout()
    fig.savefig(plot_dir / "02_ee_err_norm.png", dpi=150)
    plt.close(fig)

    xy_lines(
        "03_roll_pitch.png",
        ("roll [rad]", "pitch [rad]"),
        ("roll_des", "pitch_des"),
        ("roll_act", "pitch_act"),
    )

    fig, axes = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    axes[0].plot(t, [float(r["roll_err"]) for r in rows])
    axes[0].set_ylabel("roll_err [rad]")
    axes[1].plot(t, [float(r["pitch_err"]) for r in rows])
    axes[1].set_ylabel("pitch_err [rad]")
    axes[1].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(plot_dir / "04_rp_err.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(8, 8), sharex=True)
    for i in range(4):
        axes[i].plot(t, [float(r[f"q_jnt_{i + 1}"]) for r in rows])
        axes[i].set_ylabel(f"q_jnt_{i + 1}")
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Actual joint positions (Cartesian trajectory reference)")
    fig.tight_layout()
    fig.savefig(plot_dir / "05_q_jnt_actual.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(8, 8), sharex=True)
    for i in range(4):
        axes[i].plot(t, [float(r[f"tau_jnt_cmd_{i + 1}"]) for r in rows])
        axes[i].set_ylabel(f"tau_cmd_{i + 1} [Nm]")
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Joint torque command (after clip)")
    fig.tight_layout()
    fig.savefig(plot_dir / "06_tau_jnt_cmd.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    for row_idx, iq in enumerate([2, 3, 4]):
        axes[row_idx].plot(t, [float(r[f"tau_act_ideal_{iq}"]) for r in rows], label="ideal")
        axes[row_idx].plot(t, [float(r[f"tau_act_out_{iq}"]) for r in rows], "--", label="out")
        axes[row_idx].legend(fontsize=7)
        axes[row_idx].set_ylabel(f"act q{iq}")
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Actuator torque ideal vs cable output (q2–q4)")
    fig.tight_layout()
    fig.savefig(plot_dir / "07_tau_act_ideal_vs_out_q234.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(8, 8), sharex=True)
    for i in range(4):
        axes[i].plot(t, [float(r[f"trans_pos_err_{i + 1}"]) for r in rows])
        axes[i].set_ylabel(f"trans_{i + 1} [rad]")
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Transmission position error: q_jnt - ratio*q_act")
    fig.tight_layout()
    fig.savefig(plot_dir / "08_transmission_pos_err.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    for row_idx, iq in enumerate([2, 3, 4]):
        axes[row_idx].plot(t, [float(r[f"hys_z_{iq}"]) for r in rows], label="hys_z")
        axes[row_idx].plot(t, [float(r[f"tau_after_delay_{iq}"]) for r in rows], alpha=0.7, label="tau_ad")
        axes[row_idx].plot(t, [float(r[f"tau_after_hysteresis_{iq}"]) for r in rows], alpha=0.7, label="tau_ah")
        axes[row_idx].plot(t, [float(r[f"tau_loss_{iq}"]) for r in rows], alpha=0.7, label="tau_loss")
        axes[row_idx].legend(fontsize=6)
        axes[row_idx].set_ylabel(f"q{iq}")
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Cable diagnostics q2–q4")
    fig.tight_layout()
    fig.savefig(plot_dir / "09_cable_internals_q234.png", dpi=150)
    plt.close(fig)

    xd = np.array([[float(r["ee_des_x"]), float(r["ee_des_y"]), float(r["ee_des_z"])] for r in rows])
    xa = np.array([[float(r["ee_act_x"]), float(r["ee_act_y"]), float(r["ee_act_z"])] for r in rows])

    fig = plt.figure(figsize=(6, 5))
    ax3 = fig.add_subplot(111, projection="3d")
    ax3.plot(xd[:, 0], xd[:, 1], xd[:, 2], label="desired")
    ax3.plot(xa[:, 0], xa[:, 1], xa[:, 2], "--", label="actual")
    wp = np.vstack(WPS)
    ax3.scatter(wp[:, 0], wp[:, 1], wp[:, 2], marker="o", s=40, label="wps")
    ax3.set_xlabel("x")
    ax3.set_ylabel("y")
    ax3.set_zlabel("z")
    ax3.legend(fontsize=8)
    ax3.set_title("EE paths")
    fig.tight_layout()
    fig.savefig(plot_dir / "10_ee_path_3d.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    ee_norm = np.array([float(r["ee_err_norm"]) for r in rows])
    axes[0, 0].plot(t, ee_norm)
    axes[0, 0].set_title("EE position error norm")
    axes[0, 0].set_xlabel("time [s]")
    axes[0, 1].plot(t, [float(r["roll_err"]) for r in rows], label="roll")
    axes[0, 1].plot(t, [float(r["pitch_err"]) for r in rows], label="pitch")
    axes[0, 1].legend(fontsize=7)
    axes[0, 1].set_title("Roll/pitch error")
    axes[0, 1].set_xlabel("time [s]")
    tau_stack = np.array([[float(r[f"tau_jnt_cmd_{k + 1}"]) for k in range(4)] for r in rows])
    axes[1, 0].plot(t, np.linalg.norm(tau_stack, axis=1))
    axes[1, 0].set_title("Joint torque command L2 norm")
    axes[1, 0].set_xlabel("time [s]")
    axes[1, 1].plot(xd[:, 0], xd[:, 1], label="des")
    axes[1, 1].plot(xa[:, 0], xa[:, 1], "--", label="act")
    axes[1, 1].set_aspect("equal", adjustable="box")
    axes[1, 1].set_title("XY projection (EE)")
    axes[1, 1].set_xlabel("x [m]")
    axes[1, 1].set_ylabel("y [m]")
    axes[1, 1].legend(fontsize=7)
    fig.suptitle("Dashboard (task-space VSD demo)")
    fig.tight_layout()
    fig.savefig(plot_dir / "dashboard.png", dpi=120)
    plt.close(fig)


def write_csv(rows: list[dict[str, Any]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fn = fieldnames_flat()
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fn)
        w.writeheader()
        for r in rows:
            w.writerow({k: r[k] for k in fn})


def read_csv_rows(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    rows: list[dict[str, Any]] = []
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            row: dict[str, Any] = {}
            for k, v in r.items():
                if k == "dataset_label":
                    row[k] = str(v)
                else:
                    row[k] = float(v)
            rows.append(row)
    return rows


def write_report_md(
    path: Path,
    *,
    label: str,
    metrics: dict[str, float | int],
    video_rel: str,
    csv_rel: str,
    plots_rel: str,
    ncon_max: int,
    duration: float,
    tau_lim: float,
    cable_yaml_rel: str,
    jacobian_mode: str,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        f"# Task-space VSD trajectory demo ({label})",
        "",
        "## 1. Model",
        f"- `{HYBRID_XML.relative_to(PKG_ROOT)}` — collision meshes remain disabled in MJCF.",
        "- Runtime compile removes position actuators; torques applied on **actuator joint DOFs** (`q*_act`).",
        "",
        "## 2. Controller",
        "- Jacobian transpose task-space VSD (`xyz_roll_pitch`), yaw unconstrained.",
        "",
        "## 3. Fixed cable scalars (`CABLE_SCALAR` in script)",
    ]
    lines += [f"- `{k}`: **{v}**" for k, v in sorted(CABLE_SCALAR.items())]
    lines.append(f"- Auxiliary limits from yaml: `{cable_yaml_rel}`.")
    lines += [
        "",
        "## 4. Trajectory",
        "- Cartesian quintic segments (normalized quintic `_g()` over [0,T/2], [T/2,T]) through the three EE waypoints.",
        f"- `total_duration` = **{duration}** s.",
        "- Desired Euler (extrinsic XYZ): roll = −π/2, pitch = 0.",
        "",
        "## 5. Gains",
        f"- `Kp_xyz`: **{DEFAULT_KP_XYZ.tolist()}**, `Dp_xyz`: **{DEFAULT_DP_XYZ.tolist()}**.",
        f"- `Kp_rp`: **{DEFAULT_KP_RP.tolist()}**, `Dp_rp`: **{DEFAULT_DP_RP.tolist()}**.",
        f"- Build mode for Jacobian rows: **`{jacobian_mode}`**.",
        f"- tau_jnt_limit (clip): **{tau_lim:.1f} N·m**.",
        "",
        "## 6–9. Outputs",
        f"- Video: `{video_rel}`",
        f"- CSV: `{csv_rel}`",
        f"- Plots: `{plots_rel}`",
        "",
        "## Computed metrics",
        f"- RMS EE position error [m]: **{metrics.get('rms_ee_pos', float('nan')):.6f}**",
        f"- Final EE ‖e‖ [m]: **{metrics.get('final_ee_pos_norm', float('nan')):.6f}**",
        f"- Max EE ‖e‖ [m]: **{metrics.get('max_ee_pos_norm', float('nan')):.6f}**",
        f"- RMS roll error [rad]: **{metrics.get('rms_roll_err', float('nan')):.6f}**",
        f"- RMS pitch error [rad]: **{metrics.get('rms_pitch_err', float('nan')):.6f}**",
        f"- Max |τ_joint| [Nm]: **{metrics.get('max_torque_joint', float('nan')):.6f}**",
        f"- Max |τ_actuator| [Nm]: **{metrics.get('max_torque_act', float('nan')):.6f}**",
        f"- Max transmission position component [rad]: **{metrics.get('max_trans_pos_err_component', float('nan')):.6e}**",
        f"- Saturation steps: **{metrics.get('saturation_steps', 0)}**",
        f"- Joint limit violation steps: **{metrics.get('jl_viol_steps', 0)}**",
        f"- Actuator limit violation steps: **{metrics.get('al_viol_steps', 0)}**",
        f"- Peak `data.ncon`: **{ncon_max}**",
        "",
        "## Interpretation",
        "",
        "- Task-space VSD maintains the Cartesian path reference while gravity is cancelled at joint level before mapping to actuators.",
        "- Mild cable dynamics on q2–q4 should appear as divergence between tau_act_ideal and tau_act_out plus internal cable traces.",
        "- This isolated demo is intended as baseline for comparisons (identity cable vs residual torque / RL layers) without SAC or Gymnasium.",
        "",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_compare(paths: Path = PLOTS_DIR) -> None:
    rc = read_csv_rows(OUT_DIR / "task_space_vsd_with_cable_timeseries.csv")
    ri = read_csv_rows(OUT_DIR / "task_space_vsd_ideal_timeseries.csv")
    if not rc or not ri:
        return
    t = np.array([float(r["time"]) for r in rc], dtype=float)
    fig, ax = plt.subplots(figsize=(8, 3.5))
    ax.plot(t, [float(r["ee_err_norm"]) for r in rc], label="cable q2–q4")
    ax.plot(np.array([float(r["time"]) for r in ri]), [float(r["ee_err_norm"]) for r in ri], "--", label="ideal")
    ax.legend()
    ax.set_xlabel("time [s]")
    ax.set_ylabel("‖e_pos‖ [m]")
    ax.set_title("EE error: cable vs ideal transmission")
    fig.tight_layout()
    paths.mkdir(parents=True, exist_ok=True)
    fig.savefig(paths / "compare_ee_err_norm.png", dpi=150)
    plt.close(fig)

    xd = np.array([[float(r["ee_act_x"]), float(r["ee_act_y"]), float(r["ee_act_z"])] for r in rc])
    xa = np.array([[float(r["ee_act_x"]), float(r["ee_act_y"]), float(r["ee_act_z"])] for r in ri])
    fig = plt.figure(figsize=(5.5, 4.8))
    ax3 = fig.add_subplot(111, projection="3d")
    ax3.plot(xd[:, 0], xd[:, 1], xd[:, 2], label="cable actual EE")
    ax3.plot(xa[:, 0], xa[:, 1], xa[:, 2], "--", label="ideal actual EE")
    vp = np.vstack(WPS)
    ax3.scatter(vp[:, 0], vp[:, 1], vp[:, 2], s=36, marker="x", label="wps")
    ax3.legend(fontsize=7)
    ax3.set_title("Actual EE path — cable vs ideal")
    fig.tight_layout()
    fig.savefig(paths / "compare_paths_3d.png", dpi=150)
    plt.close(fig)


def write_compare_md(path: Path, m_cable: dict[str, float | int], m_ideal: dict[str, float | int]) -> None:
    def dv(name: str, k: str) -> str:
        a, b = m_cable.get(k), m_ideal.get(k)
        if a is None or b is None:
            return f"| {name} | — | — | — |"

        da = str(a)
        db = str(b)

        if isinstance(a, bool) or isinstance(b, bool):
            return f"| {name} | {da} | {db} | n/a |"
        if isinstance(a, (int, np.integer)) and isinstance(b, (int, np.integer)):
            return f"| {name} | {int(a)} | {int(b)} | {int(a) - int(b)} |"
        if isinstance(a, (float, np.floating)) and isinstance(b, (float, np.floating)):
            fx = lambda v: float(v)  # noqa: E731
            return f"| {name} | {fx(a):.6g} | {fx(b):.6g} | {fx(a) - fx(b):+.6g} |"

        return f"| {name} | {da} | {db} | n/a |"

    body = "\n".join(
        [
            "# Task-space VSD — cable vs ideal transmission",
            "",
            "| Metric | Cable | Ideal | Δ (c−i) |",
            "| --- | ---:| ---:| ---:|",
            dv("RMS EE pos error norm [m]", "rms_ee_pos"),
            dv("Final EE pos error norm [m]", "final_ee_pos_norm"),
            dv("Max EE pos error norm [m]", "max_ee_pos_norm"),
            dv("Max |joint torque| [Nm]", "max_torque_joint"),
            dv("Max |actuator torque| [Nm]", "max_torque_act"),
            dv("Saturation steps", "saturation_steps"),
            dv("JL violation steps", "jl_viol_steps"),
            dv("ACT violation steps", "al_viol_steps"),
            dv("Max transmission pos component [rad]", "max_trans_pos_err_component"),
            "",
            "## Plots",
            "- `plots/compare_ee_err_norm.png`",
            "- `plots/compare_paths_3d.png`",
            "",
            "Same controller gains and trajectory as the main demo; only `HybridTransmission.cable` differs (identity cable for ideal column).",
            "",
        ]
    )
    path.write_text(body, encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--tau-jnt-limit", type=float, default=30.0)
    ap.add_argument("--save-video", action="store_true", help="Needs ffmpeg on PATH.")
    ap.add_argument("--save-gif", action="store_true")
    ap.add_argument("--compare-ideal", action="store_true")
    ap.add_argument("--frame-stride", type=int, default=4)
    ap.add_argument("--jacobian-mode", choices=["numerical", "mujoco_analytic"], default="mujoco_analytic")
    ap.add_argument(
        "--viewer",
        action="store_true",
        help="MuJoCo GLFW passive viewer (needs display / WSLg). Yellow: waypoint polyline; red: desired EE; green: actual EE.",
    )
    ap.add_argument(
        "--viewer-realtime-scale",
        type=float,
        default=1.0,
        help="Sleep scale vs sim dt each step (1≈real-time). 0 = run as fast as possible.",
    )
    ap.add_argument("--no-viewer-overlay", action="store_true", help="Disable path / EE marker overlay in the viewer.")
    args = ap.parse_args()

    with open(cable_yaml_path(), encoding="utf-8") as fy:
        full_yaml = yaml.safe_load(fy)
    demo_cable = make_demo_cable_transmission(full_yaml)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)

    model = load_hybrid_torque_only()
    dt_sim = float(model.opt.timestep)
    stride = max(1, int(args.frame_stride))
    fps_vid = float(1.0 / (dt_sim * stride))

    save_video_flag = args.save_video
    cam_id = -1

    hybrid_cable = HybridTransmission(cable=demo_cable)
    renderer: mj.Renderer | None = None
    if save_video_flag:
        renderer = mj.Renderer(model, width=640, height=480)

    data_shared: mj.MjData | None = mj.MjData(model) if args.viewer else None
    vw_scale = float(args.viewer_realtime_scale)
    overlay = not args.no_viewer_overlay

    def run_cable(viewer_h: Any | None) -> tuple[list[dict[str, Any]], list[np.ndarray], int]:
        return run_rollout(
            model=model,
            hybrid=hybrid_cable,
            duration=float(args.duration),
            tau_jnt_limit=float(args.tau_jnt_limit),
            kp_xyz=DEFAULT_KP_XYZ,
            dp_xyz=DEFAULT_DP_XYZ,
            kp_rp=DEFAULT_KP_RP,
            dp_rp=DEFAULT_DP_RP,
            jacobian_mode=args.jacobian_mode,  # type: ignore[arg-type]
            dataset_label="cable",
            collect_frames_every=stride if save_video_flag and renderer is not None else 0,
            renderer=renderer,
            camera_id=cam_id,
            sim_data=data_shared,
            viewer=viewer_h,
            viewer_realtime_scale=vw_scale,
            viewer_overlay=overlay,
        )

    _stub_for_ctx = mj.MjData(model)
    with passive_mujoco_viewer(model, data_shared if args.viewer else _stub_for_ctx, args.viewer) as vw:
        rows_cable, frames, ncon_peak = run_cable(vw)

    n_expect = int(np.round(float(args.duration) / dt_sim)) + 1
    if args.viewer and len(rows_cable) < n_expect:
        print(f"[viewer] 창을 닫아 조기 종료: {len(rows_cable)} / 약 {n_expect} 스텝만 기록되었습니다.")

    csv_main = OUT_DIR / "task_space_vsd_with_cable_timeseries.csv"
    write_csv(rows_cable, csv_main)

    plots_from_csv(rows_cable)

    vid_rel = "*(not requested — pass --save-video)*"
    gif_rel = "*(skipped)*"

    mp4_path = OUT_DIR / "demo_task_space_vsd_with_cable.mp4"
    gif_path = OUT_DIR / "demo_task_space_vsd_with_cable.gif"
    ffmpeg_ok = shutil.which("ffmpeg") is not None

    if save_video_flag:
        if not ffmpeg_ok:
            print("Skipping MP4: ffmpeg not found on PATH.")
        elif not frames:
            print("Skipping video: no frames captured.")
        else:
            ok = write_mp4_via_ffmpeg(frames, mp4_path, fps=fps_vid)
            vid_rel = str(mp4_path.relative_to(PKG_ROOT)) if ok else "*(ffmpeg failed)*"
            if ok and args.save_gif:
                if maybe_write_gif(frames, gif_path, fps=fps_vid):
                    gif_rel = str(gif_path.relative_to(PKG_ROOT))
                else:
                    gif_rel = "*(GIF: install Pillow)*"
            elif ok:
                gif_rel = "*(use --save-gif)*"

    mc = metrics_from_rows(rows_cable)
    yaml_rel = str(cable_yaml_path().relative_to(PKG_ROOT))

    write_report_md(
        OUT_DIR / "task_space_vsd_with_cable_report.md",
        label="cable dynamics q2–q4",
        metrics=mc,
        video_rel=vid_rel + (f"; GIF `{gif_rel}`" if gif_rel != "*(skipped)*" else ""),
        csv_rel=str(csv_main.relative_to(PKG_ROOT)),
        plots_rel=str(PLOTS_DIR.relative_to(PKG_ROOT)),
        ncon_max=ncon_peak,
        duration=float(args.duration),
        tau_lim=float(args.tau_jnt_limit),
        cable_yaml_rel=yaml_rel,
        jacobian_mode=str(args.jacobian_mode),
    )

    if args.compare_ideal:
        ideal_h = HybridTransmission(cable=make_identity_cable())
        rows_i, _, ncon_ideal = run_rollout(
            model=model,
            hybrid=ideal_h,
            duration=float(args.duration),
            tau_jnt_limit=float(args.tau_jnt_limit),
            kp_xyz=DEFAULT_KP_XYZ,
            dp_xyz=DEFAULT_DP_XYZ,
            kp_rp=DEFAULT_KP_RP,
            dp_rp=DEFAULT_DP_RP,
            jacobian_mode=args.jacobian_mode,  # type: ignore[arg-type]
            dataset_label="ideal_identity",
            collect_frames_every=0,
            renderer=None,
            camera_id=cam_id,
            sim_data=None,
            viewer=None,
        )
        csv_ideal = OUT_DIR / "task_space_vsd_ideal_timeseries.csv"
        write_csv(rows_i, csv_ideal)
        mi = metrics_from_rows(rows_i)
        plot_compare(PLOTS_DIR)
        write_compare_md(OUT_DIR / "task_space_vsd_cable_vs_ideal_report.md", mc, mi)
        print(f"Compared ncon_peak ideal={ncon_ideal} cable={ncon_peak}")

    print(f"Wrote CSV: {csv_main}")
    print(f"Wrote plots under: {PLOTS_DIR}")
    print(f"Report: {OUT_DIR / 'task_space_vsd_with_cable_report.md'}")


if __name__ == "__main__":
    main()

