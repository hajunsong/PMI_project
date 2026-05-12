#!/usr/bin/env python3
"""
Deterministic comparison: hybrid arm task-space VSD with ideal transmission vs cable (q2–q4).

Same model (`pmi_hybrid_no_collision.xml`), trajectory, gains, initialization; transmission differs only.
Outputs: per-case CSV, comparison metrics CSV, plots, MP4 videos, Markdown report.

No SAC, Gymnasium, training, or cable parameter randomization.
"""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
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

from kinematics.forward_kinematics import fk_ee_rp
from kinematics.orientation_utils import angle_error
from kinematics.task_jacobian import compute_task_jacobian_mode
from trajectory.joint_quintic import scaled_joint_quintic
from transmission.cable_transmission import (
    CableLayerBacklashDeadzoneConfig,
    CableLayerDelayConfig,
    CableLayerElasticityConfig,
    CableLayerFrictionConfig,
    CableLayerHysteresisConfig,
    CableTransmission,
    CableTransmitResult,
)
from transmission.cable_transmission import cable_transmission_identity as make_identity_cable
from transmission.hybrid_transmission import HybridTransmission
from utils.mujoco_helpers import PKG_ROOT, joint_id

HYBRID_XML = PKG_ROOT / "models" / "pmi_hybrid_no_collision.xml"
OUT_DIR = PKG_ROOT / "debug_outputs" / "task_space_comparison"
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

CASE_IDEAL = "ideal_no_cable"
CASE_CABLE = "cable_enabled"

JacobPick = Literal["numerical", "mujoco_analytic"]

KP_XYZ = np.array([80.0, 80.0, 80.0], dtype=np.float64)
DP_XYZ = np.array([8.0, 8.0, 8.0], dtype=np.float64)
KP_RP = np.array([20.0, 20.0], dtype=np.float64)
DP_RP = np.array([2.0, 2.0], dtype=np.float64)

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

VIDEO_CAM_USE_MUJOCO_DEFAULT = False
VIDEO_CAM_DISTANCE = 2.5
VIDEO_CAM_AZIMUTH = 135.0
VIDEO_CAM_ELEVATION = -25.0
VIDEO_CAM_LOOKAT: np.ndarray | None = None


def np3(s: float) -> np.ndarray:
    return np.array([float(s)] * 3, dtype=np.float64)


def cable_yaml_path() -> Path:
    return PKG_ROOT / "configs" / "cable_layer.yaml"


def make_cable_stack(full_yaml: dict[str, Any]) -> CableTransmission:
    cfg = CABLE_SCALAR
    bd = full_yaml["cable_backlash_deadzone"]
    cd = full_yaml["cable_delay"]
    cf = full_yaml["cable_friction"]
    el = full_yaml["cable_elasticity"]
    hy = full_yaml["cable_hysteresis"]
    tau_clip = cf.get("tau_out_clip")

    return CableTransmission(
        CableLayerDelayConfig(
            enabled=True, tau_delay_s=np3(cfg["tau_delay"]), initialize_state_to_input=bool(cd["initialize_state_to_input"])
        ),
        CableLayerFrictionConfig(
            enabled=True,
            viscous_b=np3(cfg["viscous_b"]),
            coulomb_fc=np3(cfg["coulomb_fc"]),
            v_eps=float(cf["v_eps"]),
            tau_out_clip=float(tau_clip) if tau_clip is not None else None,
        ),
        CableLayerElasticityConfig(
            enabled=True,
            mode="torque_compliance",
            tau_elastic_s=np3(cfg["tau_elastic"]),
            initialize_state_to_input=bool(el["initialize_state_to_input"]),
        ),
        CableLayerBacklashDeadzoneConfig(
            enabled=True,
            tau_deadzone=np3(cfg["tau_deadzone"]),
            backlash_width=np3(cfg["backlash_width"]),
            backlash_slope=float(cfg["backlash_slope"]),
            tau_eps=float(bd["tau_eps"]),
            deadzone_mode=str(bd.get("deadzone_mode", "hard")),
        ),
        CableLayerHysteresisConfig(
            enabled=True,
            mode="torque_bouc_wen",
            alpha=np3(cfg["hys_alpha"]),
            A=np3(cfg["hys_A"]),
            beta=np3(cfg["hys_beta"]),
            gamma=np3(cfg["hys_gamma"]),
            n=np3(cfg["hys_n"]),
            z_max=float(hy["z_max"]),
            tau_hys_max=float(hy["tau_hys_max"]),
            initialize_z_to_zero=bool(hy["initialize_z_to_zero"]),
        ),
    )


def scene_camera_for_mp4(model: mj.MjModel) -> int | mj.MjvCamera:
    if VIDEO_CAM_USE_MUJOCO_DEFAULT:
        return -1
    cam = mj.MjvCamera()
    cam.type = mj.mjtCamera.mjCAMERA_FREE
    mj.mjv_defaultFreeCamera(model, cam)
    cam.distance = float(VIDEO_CAM_DISTANCE)
    cam.azimuth = float(VIDEO_CAM_AZIMUTH)
    cam.elevation = float(VIDEO_CAM_ELEVATION)
    if VIDEO_CAM_LOOKAT is not None:
        cam.lookat[:] = np.asarray(VIDEO_CAM_LOOKAT, dtype=np.float64).reshape(3)
    return cam


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


def ee_xyz(model: mj.MjModel, data: mj.MjData) -> np.ndarray:
    sid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, SITE)
    return np.array(data.site_xpos[sid], dtype=np.float64).copy()


def pad_q1(v3: np.ndarray, fill: float) -> np.ndarray:
    o = np.zeros(4)
    o[0] = float(fill)
    o[1:4] = np.asarray(v3, dtype=np.float64).reshape(3)
    return o


def diagnostics_from_cr(cr: CableTransmitResult | None, tau_act_ideal: np.ndarray) -> dict[str, np.ndarray]:
    if cr is None:
        z = np.zeros(4)
        return dict(
            t_delay=tau_act_ideal.copy(),
            t_cmp=tau_act_ideal.copy(),
            t_ah=tau_act_ideal.copy(),
            t_adz=tau_act_ideal.copy(),
            t_abk=tau_act_ideal.copy(),
            tv=z.copy(),
            tc=z.copy(),
            tl=z.copy(),
            th=z.copy(),
            hz=z.copy(),
        )
    return dict(
        t_delay=pad_q1(cr.tau_after_delay, tau_act_ideal[0]),
        t_cmp=pad_q1(cr.tau_compliant, tau_act_ideal[0]),
        t_ah=pad_q1(cr.tau_after_hysteresis, tau_act_ideal[0]),
        t_adz=pad_q1(cr.tau_after_deadzone, tau_act_ideal[0]),
        t_abk=pad_q1(cr.tau_after_backlash, tau_act_ideal[0]),
        tv=pad_q1(cr.tau_viscous, 0.0),
        tc=pad_q1(cr.tau_coulomb, 0.0),
        tl=pad_q1(cr.tau_loss, 0.0),
        th=pad_q1(cr.tau_hys, 0.0),
        hz=pad_q1(cr.hys_z, 0.0),
    )


def csv_fieldnames() -> list[str]:
    fn = ["time", "case_name"]
    fn += ["ee_des_x", "ee_des_y", "ee_des_z", "ee_act_x", "ee_act_y", "ee_act_z"]
    fn += ["ee_err_x", "ee_err_y", "ee_err_z", "ee_err_norm"]
    fn += ["ee_vel_des_x", "ee_vel_des_y", "ee_vel_des_z"]
    fn += ["ee_vel_act_x", "ee_vel_act_y", "ee_vel_act_z"]
    fn += ["ee_vel_err_x", "ee_vel_err_y", "ee_vel_err_z"]
    fn += ["roll_des", "pitch_des", "roll_act", "pitch_act", "roll_err", "pitch_err", "orientation_controlled_flag"]
    for i in range(4):
        fn.append(f"q_jnt_{i + 1}")
    for i in range(4):
        fn.append(f"qdot_jnt_{i + 1}")
    for i in range(4):
        fn.append(f"q_act_{i + 1}")
    for i in range(4):
        fn.append(f"qdot_act_{i + 1}")
    for i in range(4):
        fn.append(f"tau_bias_jnt_{i + 1}")
    for i in range(4):
        fn.append(f"tau_task_jnt_{i + 1}")
    for i in range(4):
        fn.append(f"tau_jnt_cmd_{i + 1}")
    for i in range(4):
        fn.append(f"tau_act_ideal_{i + 1}")
    for i in range(4):
        fn.append(f"tau_act_out_{i + 1}")
    for i in range(4):
        fn.append(f"tau_transmission_error_{i + 1}")
    for i in range(4):
        fn.append(f"tau_after_delay_{i + 1}")
    for i in range(4):
        fn.append(f"tau_compliant_{i + 1}")
    for i in range(4):
        fn.append(f"tau_after_hysteresis_{i + 1}")
    for i in range(4):
        fn.append(f"tau_after_deadzone_{i + 1}")
    for i in range(4):
        fn.append(f"tau_after_backlash_{i + 1}")
    for i in range(4):
        fn.append(f"tau_loss_{i + 1}")
    for i in range(4):
        fn.append(f"tau_viscous_{i + 1}")
    for i in range(4):
        fn.append(f"tau_coulomb_{i + 1}")
    for i in range(4):
        fn.append(f"tau_hys_{i + 1}")
    for i in range(4):
        fn.append(f"hys_z_{i + 1}")
    for i in range(4):
        fn.append(f"trans_pos_err_{i + 1}")
    fn += ["saturation_flag", "joint_limit_viol_flag", "actuator_limit_viol_flag", "ncon"]
    return fn


def append_frame_overlay(
    img: np.ndarray,
    *,
    title: str,
    ee_des: np.ndarray,
    ee_act: np.ndarray,
) -> np.ndarray:
    try:
        from PIL import Image, ImageDraw  # noqa: PLC0415

        pil = Image.fromarray(img.copy())
        d = ImageDraw.Draw(pil)
        lines = (
            title,
            f"des: ({ee_des[0]:.3f},{ee_des[1]:.3f},{ee_des[2]:.3f})",
            f"act: ({ee_act[0]:.3f},{ee_act[1]:.3f},{ee_act[2]:.3f})",
        )
        y = 4
        for ln in lines:
            d.rectangle([0, y - 2, pil.width, y + 14], fill=(20, 20, 20, 200))
            d.text((4, y), ln, fill=(255, 255, 240))
            y += 16
        return np.asarray(pil)
    except Exception:
        return img


def write_mp4(frames: list[np.ndarray], path: Path, fps: float) -> bool:
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


def run_single_case(
    *,
    model: mj.MjModel,
    hybrid: HybridTransmission,
    case_name: str,
    duration: float,
    tau_jnt_limit: float,
    xyz_only: bool,
    jacobian_mode: JacobPick,
    collect_frames_every: int,
    renderer_template: mj.Renderer | None,
    scene_camera: int | mj.MjvCamera,
    video_overlay_label: str | None,
) -> tuple[list[dict[str, Any]], list[np.ndarray], int]:
    data = mj.MjData(model)
    jac_w = mj.MjData(model)
    add = addrs(model)
    dt = float(model.opt.timestep)

    hybrid.reset_cable_state()
    cart = scaled_joint_quintic(WPS[0], WPS[1], WPS[2], float(duration))
    orientation_flag = int(not xyz_only)
    tk = "xyz" if xyz_only else "xyz_roll_pitch"

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    for i in range(4):
        data.qpos[int(add.qadr_a[i])] = float(Q_ACT_INITIAL[i])
        data.qpos[int(add.qadr_j[i])] = float(Q_JNT_INITIAL[i])
    mj.mj_forward(model, data)
    data.qfrc_applied[:] = 0.0

    n_steps = int(np.round(duration / dt)) + 1
    rows: list[dict[str, Any]] = []
    frames: list[np.ndarray] = []
    stride = max(1, int(collect_frames_every))
    cap = renderer_template is not None and collect_frames_every > 0
    ncon_peak = 0

    for step_i in range(n_steps):
        t_cur = min(step_i * dt, float(duration))
        p_des, pd_des, _ = cart.sample(float(t_cur))

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
            task_mode=tk,
            ee_site_name=SITE,
            mode=jacobian_mode,
            epsilon=1e-6,
        )
        ee_act = ee_xyz(model, data)

        xyz_des = np.asarray(p_des[:3], dtype=np.float64).reshape(3)
        e_xyz = xyz_des - ee_act

        xd_act_full = J @ qd_j
        if xyz_only:
            v_act = xd_act_full[:3]
            e_vel_xyz = pd_des[:3] - v_act
            F_task = KP_XYZ * e_xyz + DP_XYZ * e_vel_xyz
            tau_task = J.T @ F_task
            roll_act, pitch_act, _ = fk_ee_rp(model, data, q_j, J_ARM, site_name=SITE)[1:4]
            e_roll = angle_error(float(ROLL_DES), float(roll_act))
            e_pitch = angle_error(float(PITCH_DES), float(pitch_act))
        else:
            v_act_xyz = xd_act_full[:3]
            e_vel_xyz = pd_des[:3] - v_act_xyz
            e_rp_vel = np.array([0.0, 0.0]) - xd_act_full[3:5]
            roll_act, pitch_act, _ = fk_ee_rp(model, data, q_j, J_ARM, site_name=SITE)[1:4]
            e_roll = angle_error(float(ROLL_DES), float(roll_act))
            e_pitch = angle_error(float(PITCH_DES), float(pitch_act))
            e_rp = np.array([e_roll, e_pitch])
            F_full = np.zeros(5)
            F_full[:3] = KP_XYZ * e_xyz + DP_XYZ * e_vel_xyz
            F_full[3:5] = KP_RP * e_rp + DP_RP * e_rp_vel
            tau_task = J.T @ F_full

        tau_bc = tau_bias + tau_task
        tau_cmd = np.clip(tau_bc, -float(tau_jnt_limit), float(tau_jnt_limit))
        sat = bool(np.any(np.abs(tau_bc - tau_cmd) > 1e-9))

        tau_act_ideal = RATIOS * tau_cmd
        tau_act_out = hybrid.transmit(tau_act_ideal, dt, q_a, qd_a)
        cr = hybrid.last_cable_result
        tau_tr = tau_act_out - tau_act_ideal

        jl_m = jnt_margin(model, q_j, J_ARM)
        al_m = jnt_margin(model, q_a, ACT)
        jl_v = jl_m < -1e-9
        al_v = al_m < -1e-9

        xd_des5 = np.zeros(5)
        xd_des5[:3] = pd_des[:3]
        xd_act_logged = xd_act_full.copy()
        ee_vel_act = xd_act_logged[:3]
        ee_vel_err = xd_des5[:3] - ee_vel_act

        dq = diagnostics_from_cr(cr, tau_act_ideal)
        rq = RATIOS * q_a

        row: dict[str, Any] = {
            "time": float(t_cur),
            "case_name": case_name,
            **{f"ee_des_{ax}": float(xyz_des[i]) for i, ax in enumerate(["x", "y", "z"])},
            **{f"ee_act_{ax}": float(ee_act[i]) for i, ax in enumerate(["x", "y", "z"])},
            **{f"ee_err_{ax}": float(e_xyz[i]) for i, ax in enumerate(["x", "y", "z"])},
            "ee_err_norm": float(np.linalg.norm(e_xyz)),
            **{f"ee_vel_des_{ax}": float(pd_des[i]) for i, ax in enumerate(["x", "y", "z"])},
            **{f"ee_vel_act_{ax}": float(ee_vel_act[i]) for i, ax in enumerate(["x", "y", "z"])},
            **{f"ee_vel_err_{ax}": float(ee_vel_err[i]) for i, ax in enumerate(["x", "y", "z"])},
            "roll_des": float(ROLL_DES),
            "pitch_des": float(PITCH_DES),
            "roll_act": float(roll_act),
            "pitch_act": float(pitch_act),
            "roll_err": float(e_roll),
            "pitch_err": float(e_pitch),
            "orientation_controlled_flag": orientation_flag,
            **{f"q_jnt_{i + 1}": float(q_j[i]) for i in range(4)},
            **{f"qdot_jnt_{i + 1}": float(qd_j[i]) for i in range(4)},
            **{f"q_act_{i + 1}": float(q_a[i]) for i in range(4)},
            **{f"qdot_act_{i + 1}": float(qd_a[i]) for i in range(4)},
            **{f"tau_bias_jnt_{i + 1}": float(tau_bias[i]) for i in range(4)},
            **{f"tau_task_jnt_{i + 1}": float(tau_task[i]) for i in range(4)},
            **{f"tau_jnt_cmd_{i + 1}": float(tau_cmd[i]) for i in range(4)},
            **{f"tau_act_ideal_{i + 1}": float(tau_act_ideal[i]) for i in range(4)},
            **{f"tau_act_out_{i + 1}": float(tau_act_out[i]) for i in range(4)},
            **{f"tau_transmission_error_{i + 1}": float(tau_tr[i]) for i in range(4)},
            **{f"tau_after_delay_{i + 1}": float(dq["t_delay"][i]) for i in range(4)},
            **{f"tau_compliant_{i + 1}": float(dq["t_cmp"][i]) for i in range(4)},
            **{f"tau_after_hysteresis_{i + 1}": float(dq["t_ah"][i]) for i in range(4)},
            **{f"tau_after_deadzone_{i + 1}": float(dq["t_adz"][i]) for i in range(4)},
            **{f"tau_after_backlash_{i + 1}": float(dq["t_abk"][i]) for i in range(4)},
            **{f"tau_loss_{i + 1}": float(dq["tl"][i]) for i in range(4)},
            **{f"tau_viscous_{i + 1}": float(dq["tv"][i]) for i in range(4)},
            **{f"tau_coulomb_{i + 1}": float(dq["tc"][i]) for i in range(4)},
            **{f"tau_hys_{i + 1}": float(dq["th"][i]) for i in range(4)},
            **{f"hys_z_{i + 1}": float(dq["hz"][i]) for i in range(4)},
            **{f"trans_pos_err_{i + 1}": float(q_j[i] - rq[i]) for i in range(4)},
            "saturation_flag": int(sat),
            "joint_limit_viol_flag": int(jl_v),
            "actuator_limit_viol_flag": int(al_v),
            "ncon": int(data.ncon),
        }
        rows.append(row)

        if cap and renderer_template is not None and step_i % stride == 0:
            renderer_template.update_scene(data, camera=scene_camera)
            fr = np.asarray(renderer_template.render(), dtype=np.uint8)
            if video_overlay_label:
                fr = append_frame_overlay(fr, title=video_overlay_label, ee_des=xyz_des.copy(), ee_act=ee_act.copy())
            frames.append(fr)

        data.qfrc_applied[:] = 0.0
        for k in range(4):
            data.qfrc_applied[int(add.dadr_a[k])] = float(tau_act_out[k])
        mj.mj_step(model, data)
        ncon_peak = max(ncon_peak, int(data.ncon))

    return rows, frames, ncon_peak


def per_case_metrics(rows: list[dict[str, Any]], case_label: str) -> dict[str, Any]:
    ex = np.array([float(r["ee_err_x"]) for r in rows])
    ey = np.array([float(r["ee_err_y"]) for r in rows])
    ez = np.array([float(r["ee_err_z"]) for r in rows])
    en = np.array([float(r["ee_err_norm"]) for r in rows])
    tjq = np.array([[float(r[f"tau_jnt_cmd_{k + 1}"]) for k in range(4)] for r in rows])
    tao = np.array([[float(r[f"tau_act_out_{k + 1}"]) for k in range(4)] for r in rows])
    tte_q234 = np.array([[float(r[f"tau_transmission_error_{k + 1}"]) for k in range(1, 4)] for r in rows])
    tr_all = np.array([[float(r[f"trans_pos_err_{k + 1}"]) for k in range(4)] for r in rows])

    return {
        "case_name": case_label,
        "rms_ee_position_error": float(np.sqrt(np.mean(en**2))),
        "final_ee_position_error": float(en[-1]),
        "max_ee_position_error": float(np.max(en)),
        "rms_ex": float(np.sqrt(np.mean(ex**2))),
        "rms_ey": float(np.sqrt(np.mean(ey**2))),
        "rms_ez": float(np.sqrt(np.mean(ez**2))),
        "max_joint_torque_mag": float(np.max(np.abs(tjq))),
        "max_actuator_torque_mag": float(np.max(np.abs(tao))),
        "rms_tau_transmission_error_q234": float(np.sqrt(np.mean(np.sum(tte_q234**2, axis=1)))),
        "max_tau_transmission_error_q234": float(np.max(np.abs(tte_q234))),
        "max_transmission_pos_err_component": float(np.max(np.abs(tr_all))),
        "saturation_steps": int(sum(int(r["saturation_flag"]) for r in rows)),
        "joint_limit_viol_steps": int(sum(int(r["joint_limit_viol_flag"]) for r in rows)),
        "actuator_limit_viol_steps": int(sum(int(r["actuator_limit_viol_flag"]) for r in rows)),
        "max_ncon": max(int(r["ncon"]) for r in rows) if rows else 0,
        "orientation_controlled_flag": rows[0].get("orientation_controlled_flag", 0) if rows else 0,
    }


def path_diff_metrics(ideal_rows: list[dict[str, Any]], cable_rows: list[dict[str, Any]]) -> dict[str, float]:
    n = min(len(ideal_rows), len(cable_rows))
    ai = np.array([[float(ideal_rows[i][f"ee_act_{ax}"]) for ax in ["x", "y", "z"]] for i in range(n)])
    ac = np.array([[float(cable_rows[i][f"ee_act_{ax}"]) for ax in ["x", "y", "z"]] for i in range(n)])
    d = np.linalg.norm(ac - ai, axis=1)
    return {
        "rms_actual_ee_path_difference": float(np.sqrt(np.mean(d**2))),
        "max_actual_ee_path_difference": float(np.max(d)),
        "final_actual_ee_position_difference": float(d[-1] if len(d) else 0.0),
    }


def write_csv(rows: list[dict[str, Any]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fn = csv_fieldnames()
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fn)
        w.writeheader()
        for r in rows:
            w.writerow({k: r[k] for k in fn})


def make_plots(ri: list[dict[str, Any]], rc: list[dict[str, Any]]) -> None:
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    n = min(len(ri), len(rc))
    ri = ri[:n]
    rc = rc[:n]
    t = np.array([float(r["time"]) for r in ri])

    try:
        plt.style.use("seaborn-v0_8-whitegrid")
    except OSError:
        pass

    des_x = np.array([float(r["ee_des_x"]) for r in ri])
    ix = np.array([float(r["ee_act_x"]) for r in ri])
    cx = np.array([float(r["ee_act_x"]) for r in rc])

    fig, ax = plt.subplots(figsize=(8, 3.5))
    ax.plot(t, des_x, label="desired")
    ax.plot(t, ix, "--", label=CASE_IDEAL)
    ax.plot(t, cx, ":", label=CASE_CABLE)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("x [m]")
    ax.legend(fontsize=8)
    ax.set_title("EE x")
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "01_ee_x_des_ideal_cable.png", dpi=140)
    plt.close(fig)

    for yi, ylab in enumerate(["y", "z"]):
        ides = np.array([float(r[f"ee_des_{ylab}"]) for r in ri])
        iact = np.array([float(r[f"ee_act_{ylab}"]) for r in ri])
        cact = np.array([float(r[f"ee_act_{ylab}"]) for r in rc])
        fig, ax = plt.subplots(figsize=(8, 3.5))
        ax.plot(t, ides, label="desired")
        ax.plot(t, iact, "--", label=CASE_IDEAL)
        ax.plot(t, cact, ":", label=CASE_CABLE)
        ax.set_xlabel("time [s]")
        ax.set_ylabel(f"{ylab} [m]")
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"0{yi + 2}_ee_{ylab}_des_ideal_cable.png", dpi=140)
        plt.close(fig)

    en_i = np.array([float(r["ee_err_norm"]) for r in ri])
    en_c = np.array([float(r["ee_err_norm"]) for r in rc])
    fig, ax = plt.subplots(figsize=(8, 3.5))
    ax.plot(t, en_i, label=CASE_IDEAL)
    ax.plot(t, en_c, label=CASE_CABLE)
    ax.set_xlabel("time [s]")
    ax.set_ylabel(r"$\| e_{xyz} \|$ [m]")
    ax.legend()
    fig.savefig(PLOTS_DIR / "04_ee_err_norm_ideal_vs_cable.png", dpi=140)
    plt.close(fig)

    wp = np.vstack(WPS)
    fig = plt.figure(figsize=(5.5, 5))
    ax3 = fig.add_subplot(111, projection="3d")
    ax3.plot(wp[:, 0], wp[:, 1], wp[:, 2], "ko-", markersize=4, label="waypoints/desired shape")
    ax3.plot(des_x[:n], np.array([float(r["ee_des_y"]) for r in ri][:n]), np.array([float(r["ee_des_z"]) for r in ri][:n]), alpha=0.5, label="sampled des")
    ax3.plot(ix, np.array([float(r["ee_act_y"]) for r in ri]), np.array([float(r["ee_act_z"]) for r in ri]), "--", label=CASE_IDEAL)
    ax3.plot(cx, np.array([float(r["ee_act_y"]) for r in rc]), np.array([float(r["ee_act_z"]) for r in rc]), label=CASE_CABLE)
    ax3.set_title("EE paths (world)")
    ax3.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "05_path_3d_des_ideal_cable.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(8, 7), sharex=True)
    for k in range(4):
        axes[k].plot(t, [float(r[f"q_jnt_{k + 1}"]) for r in ri], "--", label=CASE_IDEAL if k == 0 else None)
        axes[k].plot(t, [float(r[f"q_jnt_{k + 1}"]) for r in rc], label=CASE_CABLE if k == 0 else None)
        axes[k].set_ylabel(f"q_jnt_{k + 1}")
    axes[0].legend(fontsize=7)
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Joint positions (FK — no Cartesian q_des in log)")
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "06_q_jnt_ideal_vs_cable.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(8, 7), sharex=True)
    for k in range(4):
        axes[k].plot(t, [float(r[f"tau_jnt_cmd_{k + 1}"]) for r in ri], "--", label=f"{CASE_IDEAL}" if k == 0 else None)
        axes[k].plot(t, [float(r[f"tau_jnt_cmd_{k + 1}"]) for r in rc], label=f"{CASE_CABLE}" if k == 0 else None)
        axes[k].set_ylabel(fr"tau_cmd_{k + 1}")
    axes[0].legend(fontsize=7)
    axes[-1].set_xlabel("time [s]")
    fig.savefig(PLOTS_DIR / "07_tau_jnt_cmd_ideal_vs_cable.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(8, 5.5), sharex=True)
    for idx, iq in enumerate([2, 3, 4]):
        k = iq - 1
        axes[idx].plot(t, [float(r[f"tau_act_ideal_{iq}"]) for r in rc], label="ideal (cmd map)")
        axes[idx].plot(t, [float(r[f"tau_act_out_{iq}"]) for r in rc], "--", label="out (cable)")
        axes[idx].set_title(f"q{iq} actuator torque (cable case)")
        axes[idx].legend(fontsize=7)
    axes[-1].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "08_tau_act_ideal_vs_out_q234_cable.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(8, 5.5), sharex=True)
    for idx, iq in enumerate([2, 3, 4]):
        axes[idx].plot(t, [float(r[f"tau_loss_{iq}"]) for r in rc], label="tau_loss")
        axes[idx].plot(t, [float(r[f"tau_hys_{iq}"]) for r in rc], label="tau_hys")
        axes[idx].plot(t, [float(r[f"hys_z_{iq}"]) for r in rc], alpha=0.7, label="hys_z")
        axes[idx].set_ylabel(f"q{iq}")
        axes[idx].legend(fontsize=6)
    axes[-1].set_xlabel("time [s]")
    fig.suptitle("Cable loss / hysteresis diagnostics (enabled case)")
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "09_cable_loss_terms_q234.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(8, 6), sharex=True)
    for k in range(4):
        axes[k].plot(t, [float(r[f"tau_transmission_error_{k + 1}"]) for r in ri], "--", linewidth=1, label="ideal (≈0)" if k == 0 else None)
        axes[k].plot(t, [float(r[f"tau_transmission_error_{k + 1}"]) for r in rc], label="cable")
        axes[k].set_ylabel(f"Δτ_{k + 1}")
    axes[0].legend(fontsize=7)
    axes[-1].set_xlabel("time [s]")
    fig.savefig(PLOTS_DIR / "10_tau_transmission_error.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(8, 6), sharex=True)
    for k in range(4):
        axes[k].plot(t, [float(r[f"trans_pos_err_{k + 1}"]) for r in ri], "--")
        axes[k].plot(t, [float(r[f"trans_pos_err_{k + 1}"]) for r in rc])
        axes[k].set_ylabel(f"Δq_{k + 1}")
    axes[-1].set_xlabel("time [s]")
    fig.suptitle(r"$q_{\mathrm{jnt}} - r \odot q_{\mathrm{act}}$")
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "11_transmission_position_error.png", dpi=140)
    plt.close(fig)

    # 12 metrics bar chart
    mi = per_case_metrics(ri[:n], CASE_IDEAL)
    mc = per_case_metrics(rc[:n], CASE_CABLE)
    keys_plot = ["rms_ee_position_error", "final_ee_position_error", "max_ee_position_error"]
    xv = np.arange(len(keys_plot))
    w = 0.35
    fig, ax = plt.subplots(figsize=(7.5, 4))
    vals_i = [float(mi[k]) for k in keys_plot]
    vals_c = [float(mc[k]) for k in keys_plot]
    ax.bar(xv - w / 2, vals_i, w, label=CASE_IDEAL)
    ax.bar(xv + w / 2, vals_c, w, label=CASE_CABLE)
    ax.set_xticks(xv)
    ax.set_xticklabels(["RMS EE\nerr", "final EE\nerr", "max EE\nerr"], fontsize=8)
    ax.set_ylabel("[m]")
    ax.legend()
    ax.set_title("Position error metrics (selected)")
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "12_metrics_bars_selected.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(5, 3.8))
    ax.bar([CASE_IDEAL, CASE_CABLE], [mi["saturation_steps"], mc["saturation_steps"]], color=["tab:blue", "tab:orange"])
    ax.set_ylabel("count")
    ax.set_title("Saturation steps")
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "13_saturation_steps_bar.png", dpi=140)
    plt.close(fig)


def comparison_md(
    path: Path,
    *,
    mi: dict[str, Any],
    mc: dict[str, Any],
    path_diff: dict[str, float],
    video_paths: dict[str, str],
    ffmpeg_ok: bool,
) -> None:
    pd = yaml.dump(CABLE_SCALAR, default_flow_style=True, sort_keys=True)
    lines = [
        "# Task-space VSD — ideal transmission vs deterministic cable",
        "",
        "## 1. Model",
        f"- MJCF (no mesh collision validated): `{HYBRID_XML.relative_to(PKG_ROOT)}` compiled with torque-only (position actuators removed).",
        "- Torques on actuator DOFs: `tau_act_ideal = ratios ⊙ tau_jnt_cmd` → HybridTransmission (`qfrc_applied` on `q*_act`).",
        f"- Collision contacts: **`max(ncon)_ideal={mi['max_ncon']}`, `max(ncon)_cable={mc['max_ncon']}`**.",
        "",
        "## 2. Controller",
        f"- Task-space Jacobian-transpose VSD; orientation subset flag in CSV **`orientation_controlled_flag={mi['orientation_controlled_flag']}`** (0 = xyz-only, 1 = xyz+roll+pitch).",
        f"- Default gains used: **Kp_xyz={KP_XYZ.tolist()}**, **Dp_xyz={DP_XYZ.tolist()}**; roll/pitch (if enabled) **Kp_rp={KP_RP.tolist()}**, **Dp_rp={DP_RP.tolist()}**.",
        "",
        "## 3. Trajectory",
        "- Quintic Cartesian path through three waypoints scaled to `[0, T/2, T]` (same as other demos); `duration=5` s nominal.",
        f"- Desired roll/pitch (reporting): −π/2, 0; yaw free.",
        "",
        "## 4. Cable parameters (deterministic q2–q4, no randomization)",
        "```yaml",
        pd.strip(),
        "```",
        "",
        "## 5. Metrics summary",
        "",
        "| Metric | ideal_no_cable | cable_enabled |",
        "| --- | ---:| ---:|",
        f"| RMS EE ‖e‖ | {mi['rms_ee_position_error']:.6f} | {mc['rms_ee_position_error']:.6f} |",
        f"| Final EE ‖e‖ | {mi['final_ee_position_error']:.6f} | {mc['final_ee_position_error']:.6f} |",
        f"| Max EE ‖e‖ | {mi['max_ee_position_error']:.6f} | {mc['max_ee_position_error']:.6f} |",
        f"| RMS τ_trans err q2–q4 | {mi['rms_tau_transmission_error_q234']:.4e} | {mc['rms_tau_transmission_error_q234']:.4e} |",
        f"| Saturation steps | {mi['saturation_steps']} | {mc['saturation_steps']} |",
        "",
        "**Actual EE path difference (ideal vs cable, same timestep index):**",
        f"- RMS ‖p_c−p_i‖: **{path_diff['rms_actual_ee_path_difference']:.6f}** m",
        f"- Max ‖·‖: **{path_diff['max_actual_ee_path_difference']:.6f}** m",
        f"- Final ‖·‖: **{path_diff['final_actual_ee_position_difference']:.6f}** m",
        "",
        "## 6. Interpretation (checklist)",
        "",
        "- **Still tracks:** Compare RMS/max EE error cable vs ideal; trajectory remains close if RMS is small relative to workspace.",
        "- **Degradation:** Δ RMS EE = cable RMS − ideal RMS (see table above).",
        "- **Torque:** `plots/08_*` / `plots/09_*` show τ_out vs τ_ideal and hysteresis diagnostics on cable case.",
        "- **Hysteresis state:** `hys_z_*` bounded by model z_max yaml; plotted in Fig. 9.",
        "- **Saturation / limits:** see bar plot and CSV flags.",
        "- **ncon:** Must stay zero under no-mesh-contact assumption (table).",
        "- **Residual RL:** This deterministic baseline separates transmission effects before adding RL residual torque.",
        "",
        "## 7. Plots (`debug_outputs/task_space_comparison/plots/`)",
        "- See PNG files `01_*.png` … `13_*.png`.",
        "",
        "## 8. Videos",
        f"- FFmpeg available: **`{ffmpeg_ok}`**.",
        f"- Intended paths: `{video_paths['ideal_no_cable.mp4']}`, `{video_paths['cable_enabled.mp4']}`, `{video_paths['ideal_vs_cable_side_by_side.mp4']}`",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    import shutil as sh  # noqa: PLC0415

    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--tau-jnt-limit", type=float, default=30.0)
    ap.add_argument("--no-save-video", action="store_true", help="Disable MP4 export (defaults to saving videos).")
    ap.add_argument("--no-side-by-side", action="store_true", help="Skip horizontal concat video.")
    ap.add_argument(
        "--include-roll-pitch",
        action="store_true",
        help="Include roll+pitch in the task Jacobian (default: xyz position only — more stable for first comparison).",
    )
    ap.add_argument("--frame-stride", type=int, default=6)
    ap.add_argument("--jacobian-mode", choices=["numerical", "mujoco_analytic"], default="mujoco_analytic")
    args = ap.parse_args()

    xyz_only_run = not args.include_roll_pitch

    save_video = not args.no_save_video
    do_side_by_side = save_video and not args.no_side_by_side

    with open(cable_yaml_path(), encoding="utf-8") as fy:
        full_yaml = yaml.safe_load(fy)
    cable_stack = make_cable_stack(full_yaml)

    hybrid_ideal = HybridTransmission(cable=make_identity_cable())
    hybrid_cable = HybridTransmission(cable=cable_stack)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)

    model = load_hybrid_torque_only()
    dt_sim = float(model.opt.timestep)
    stride = max(1, int(args.frame_stride))
    fps_vid = float(1.0 / (dt_sim * stride))
    sc = scene_camera_for_mp4(model)
    ffmpeg_ok = sh.which("ffmpeg") is not None

    renderer: mj.Renderer | None = None
    collect_every = stride if save_video and ffmpeg_ok else 0
    if collect_every > 0:
        renderer = mj.Renderer(model, width=640, height=480)

    vlabel_ideal = CASE_IDEAL if (collect_every > 0) else None
    vlabel_cable = CASE_CABLE if (collect_every > 0) else None

    ri, frames_i, _ncon_i = run_single_case(
        model=model,
        hybrid=hybrid_ideal,
        case_name=CASE_IDEAL,
        duration=float(args.duration),
        tau_jnt_limit=float(args.tau_jnt_limit),
        xyz_only=xyz_only_run,
        jacobian_mode=args.jacobian_mode,  # type: ignore[arg-type]
        collect_frames_every=collect_every,
        renderer_template=renderer,
        scene_camera=sc,
        video_overlay_label=vlabel_ideal,
    )
    rc, frames_c, _ncon_c = run_single_case(
        model=model,
        hybrid=hybrid_cable,
        case_name=CASE_CABLE,
        duration=float(args.duration),
        tau_jnt_limit=float(args.tau_jnt_limit),
        xyz_only=xyz_only_run,
        jacobian_mode=args.jacobian_mode,  # type: ignore[arg-type]
        collect_frames_every=collect_every,
        renderer_template=renderer,
        scene_camera=sc,
        video_overlay_label=vlabel_cable,
    )
    csv_i = OUT_DIR / "ideal_no_cable_timeseries.csv"
    csv_c = OUT_DIR / "cable_enabled_timeseries.csv"
    write_csv(ri, csv_i)
    write_csv(rc, csv_c)

    mi = per_case_metrics(ri, CASE_IDEAL)
    mc = per_case_metrics(rc, CASE_CABLE)
    pdf = path_diff_metrics(ri, rc)
    task_label = "xyz_only" if xyz_only_run else "xyz_roll_pitch"

    csv_cmp = OUT_DIR / "comparison_metrics.csv"
    fieldnames_m = [
        "case_name",
        "task_control",
        "rms_ee_position_error",
        "final_ee_position_error",
        "max_ee_position_error",
        "rms_ex",
        "rms_ey",
        "rms_ez",
        "max_joint_torque_mag",
        "max_actuator_torque_mag",
        "rms_tau_transmission_error_q234",
        "max_tau_transmission_error_q234",
        "max_transmission_pos_err_component",
        "saturation_steps",
        "joint_limit_viol_steps",
        "actuator_limit_viol_steps",
        "max_ncon",
        "rms_actual_ee_path_difference_vs_other_case",
        "max_actual_ee_path_difference_vs_other_case",
        "final_actual_ee_position_difference_vs_other_case",
    ]

    pdf_cols = (
        ("rms_actual_ee_path_difference", "rms_actual_ee_path_difference_vs_other_case"),
        ("max_actual_ee_path_difference", "max_actual_ee_path_difference_vs_other_case"),
        ("final_actual_ee_position_difference", "final_actual_ee_position_difference_vs_other_case"),
    )

    with open(csv_cmp, "w", newline="", encoding="utf-8") as f:
        wf = csv.DictWriter(f, fieldnames=fieldnames_m)
        wf.writeheader()
        for src in (mi, mc):
            row = {k: "" for k in fieldnames_m}
            for k in fieldnames_m:
                if k in src:
                    row[k] = src[k]
                elif k == "task_control":
                    row[k] = task_label
            for pk, ck in pdf_cols:
                row[ck] = pdf[pk]
            wf.writerow(row)

    make_plots(ri, rc)

    vid = {
        "ideal_no_cable.mp4": str((OUT_DIR / "ideal_no_cable.mp4").relative_to(PKG_ROOT)),
        "cable_enabled.mp4": str((OUT_DIR / "cable_enabled.mp4").relative_to(PKG_ROOT)),
        "ideal_vs_cable_side_by_side.mp4": str((OUT_DIR / "ideal_vs_cable_side_by_side.mp4").relative_to(PKG_ROOT)),
    }

    if save_video and ffmpeg_ok and frames_i and frames_c:
        write_mp4(frames_i, OUT_DIR / "ideal_no_cable.mp4", fps_vid)
        write_mp4(frames_c, OUT_DIR / "cable_enabled.mp4", fps_vid)
        if do_side_by_side:
            nn = min(len(frames_i), len(frames_c))
            fused = []
            for j in range(nn):
                a, b = frames_i[j], frames_c[j]
                if a.shape != b.shape:
                    h = min(a.shape[0], b.shape[0])
                    w = min(a.shape[1], b.shape[1])
                    a, b = a[:h, :w], b[:h, :w]
                fused.append(np.concatenate([a, b], axis=1))
            write_mp4(fused, OUT_DIR / "ideal_vs_cable_side_by_side.mp4", fps_vid)
    elif save_video and not ffmpeg_ok:
        print("Skipping videos: ffmpeg not on PATH.")

    comparison_md(
        OUT_DIR / "task_space_vsd_ideal_vs_cable_report.md",
        mi=mi,
        mc=mc,
        path_diff=pdf,
        video_paths=vid,
        ffmpeg_ok=ffmpeg_ok and save_video,
    )

    print(f"Wrote {csv_i.name}, {csv_c.name}, comparison_metrics.csv, report, plots/ under {OUT_DIR}")


if __name__ == "__main__":
    main()
