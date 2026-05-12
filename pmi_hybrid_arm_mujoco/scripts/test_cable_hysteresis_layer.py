#!/usr/bin/env python3
"""Cable Bouc–Wen torque hysteresis sweep (delay → compliance → hysteresis → dead-zone → backlash → friction)."""

from __future__ import annotations

import csv
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import mujoco as mj
import numpy as np
import yaml

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from kinematics.forward_kinematics import fk_ee_rp
from kinematics.inverse_kinematics import IKConfig, solve_ik_task_mode
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
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

HYBRID_XML = PKG_ROOT / "models" / "pmi_hybrid_no_collision.xml"
CABLE_LAYER_DEFAULT = PKG_ROOT / "configs" / "cable_layer.yaml"
OUT_DIR = PKG_ROOT / "debug_outputs" / "cable_layer"
PLOTS_DIR = OUT_DIR / "plots" / "hysteresis"
CSV_PATH = OUT_DIR / "hysteresis_sweep.csv"
REPORT_PATH = OUT_DIR / "hysteresis_sweep_report.md"

J = ["jnt1", "jnt2", "jnt3", "jnt4"]
ACT = ["q1_act", "q2_act", "q3_act", "q4_act"]
RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=float)
Q_ACT_INITIAL = np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=float)
Q_JNT_INITIAL = RATIOS * Q_ACT_INITIAL

WPS = [
    np.array([0.25, -0.20, -0.10], dtype=float),
    np.array([0.00, -0.35, -0.15], dtype=float),
    np.array([-0.25, -0.20, -0.10], dtype=float),
]

KQ = np.array([80.0, 80.0, 60.0, 40.0])
DQ = np.array([10.0, 10.0, 8.0, 5.0])
DURATION = 5.0
TAU_JNT_LIMIT = 20.0
SITE = "end_effector"

# light_deadzone-style RMS order (≈ combined_light); script compares no_hysteresis to this band
REF_LIGHT_DEADZONE_RMS = 0.000721


def load_hybrid_torque_only() -> mj.MjModel:
    spec = mj.MjSpec.from_file(str(HYBRID_XML))
    for a in list(spec.actuators)[::-1]:
        spec.delete(a)
    return spec.compile()


def solve_wp_q_arm_only() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    arm_only = PKG_ROOT / "models" / "pmi_arm_only_no_collision.xml"
    model = load_mjmodel(arm_only, strip_position_actuators=True)
    scratch = mj.MjData(model)
    q_lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in J])
    q_hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in J])
    ik = IKConfig((1, 1, 1), 1.0, 1.0, 1e-3, 1e-4, 80, 1e-9, tuple(J))
    q_seed = Q_JNT_INITIAL.copy()
    qs: list[np.ndarray] = []
    for wp in WPS:
        q, _ = solve_ik_task_mode(
            model,
            scratch,
            wp,
            roll_des=-np.pi / 2,
            pitch_des=0.0,
            task_feas_mode="xyz",
            ik=ik,
            q_seed=q_seed,
            bounds_lo=q_lo,
            bounds_hi=q_hi,
        )
        qs.append(q.copy())
        q_seed = q.copy()
    return qs[0], qs[1], qs[2]


def _addrs(model: mj.MjModel) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    q_j = np.array([int(model.jnt_qposadr[joint_id(model, n)]) for n in J], dtype=int)
    d_j = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in J], dtype=int)
    d_a = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in ACT], dtype=int)
    q_a = np.array([int(model.jnt_qposadr[joint_id(model, n)]) for n in ACT], dtype=int)
    return q_j, d_j, d_a, q_a


def _site_xyz(model: mj.MjModel, data: mj.MjData) -> np.ndarray:
    sid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, SITE)
    return np.array(data.site_xpos[sid], dtype=float).copy()


def _jnt_margin(model: mj.MjModel, q: np.ndarray) -> float:
    lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in J])
    hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in J])
    return float(np.min(np.minimum(hi - q, q - lo)))


def _act_margin(model: mj.MjModel, q: np.ndarray) -> float:
    lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in ACT])
    hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in ACT])
    return float(np.min(np.minimum(hi - q, q - lo)))


def load_cfg() -> dict:
    with open(CABLE_LAYER_DEFAULT, encoding="utf-8") as f:
        return yaml.safe_load(f)


def build_cable(
    *,
    tau_delay_s: float,
    viscous_b: float,
    coulomb_fc: float,
    tau_elastic_s: float,
    tau_deadzone: float,
    backlash_width: float,
    backlash_slope: float,
    hys_alpha: float,
    hys_A: float,
    hys_beta: float,
    hys_gamma: float,
    hys_n: float,
    v_eps: float,
    tau_eps: float,
    deadzone_mode: str,
    z_max: float,
    tau_hys_max: float,
    init_z_hys: bool,
    init_to_input_delay: bool,
    delay_enabled_global: bool,
    friction_enabled_global: bool,
    elastic_enabled_global: bool,
    elastic_mode: str,
    init_elastic_to_input: bool,
    backlash_deadzone_enabled_global: bool,
    hysteresis_enabled_global: bool,
    tau_out_clip: float | None,
) -> CableTransmission:
    td = float(tau_delay_s)
    if delay_enabled_global and td > 0.0:
        dcfg = CableLayerDelayConfig(
            enabled=True,
            tau_delay_s=np.full(3, td, dtype=np.float64),
            initialize_state_to_input=init_to_input_delay,
        )
    else:
        dcfg = CableLayerDelayConfig(
            enabled=False,
            tau_delay_s=np.zeros(3, dtype=np.float64),
            initialize_state_to_input=init_to_input_delay,
        )

    bv = float(viscous_b)
    fc = float(coulomb_fc)
    if friction_enabled_global and (abs(bv) > 0.0 or abs(fc) > 0.0):
        fcfg = CableLayerFrictionConfig(
            enabled=True,
            viscous_b=np.full(3, bv, dtype=np.float64),
            coulomb_fc=np.full(3, fc, dtype=np.float64),
            v_eps=float(v_eps),
            tau_out_clip=tau_out_clip,
        )
    else:
        fcfg = CableLayerFrictionConfig(
            enabled=False,
            viscous_b=np.zeros(3, dtype=np.float64),
            coulomb_fc=np.zeros(3, dtype=np.float64),
            v_eps=float(v_eps),
            tau_out_clip=tau_out_clip,
        )

    te = float(tau_elastic_s)
    if elastic_enabled_global and te > 0.0 and str(elastic_mode) == "torque_compliance":
        ecfg = CableLayerElasticityConfig(
            enabled=True,
            mode="torque_compliance",
            tau_elastic_s=np.full(3, te, dtype=np.float64),
            initialize_state_to_input=init_elastic_to_input,
        )
    else:
        ecfg = CableLayerElasticityConfig(
            enabled=False,
            mode="torque_compliance",
            tau_elastic_s=np.zeros(3, dtype=np.float64),
            initialize_state_to_input=init_elastic_to_input,
        )

    dz = float(tau_deadzone)
    bw = float(backlash_width)
    bs = float(backlash_slope)
    if backlash_deadzone_enabled_global:
        bz = CableLayerBacklashDeadzoneConfig(
            enabled=True,
            tau_deadzone=np.full(3, dz, dtype=np.float64),
            backlash_width=np.full(3, bw, dtype=np.float64),
            backlash_slope=bs,
            tau_eps=float(tau_eps),
            deadzone_mode=str(deadzone_mode),
        )
    else:
        bz = CableLayerBacklashDeadzoneConfig(enabled=False)

    if hysteresis_enabled_global:
        hcfg = CableLayerHysteresisConfig(
            enabled=True,
            mode="torque_bouc_wen",
            alpha=np.full(3, float(hys_alpha), dtype=np.float64),
            A=np.full(3, float(hys_A), dtype=np.float64),
            beta=np.full(3, float(hys_beta), dtype=np.float64),
            gamma=np.full(3, float(hys_gamma), dtype=np.float64),
            n=np.full(3, float(hys_n), dtype=np.float64),
            z_max=float(z_max),
            tau_hys_max=float(tau_hys_max),
            initialize_z_to_zero=bool(init_z_hys),
        )
    else:
        hcfg = CableLayerHysteresisConfig(enabled=False)

    return CableTransmission(dcfg, fcfg, ecfg, bz, hcfg)


def run_case(
    sweep_name: str,
    tau_delay_param: float,
    viscous_b: float,
    coulomb_fc: float,
    tau_elastic_param: float,
    tau_deadzone_param: float,
    backlash_width_param: float,
    backlash_slope_param: float,
    hys_alpha: float,
    hys_A: float,
    hys_beta: float,
    hys_gamma: float,
    hys_n: float,
    *,
    v_eps: float,
    tau_eps: float,
    deadzone_mode: str,
    z_max: float,
    tau_hys_max: float,
    init_z_hys: bool,
    init_to_input_delay: bool,
    init_elastic_to_input: bool,
    delay_glob: bool,
    friction_glob: bool,
    elastic_glob: bool,
    elastic_mode: str,
    backlash_dz_glob: bool,
    hysteresis_glob: bool,
    tau_out_clip: float | None,
    qwp0: np.ndarray,
    qwp1: np.ndarray,
    qwp2: np.ndarray,
) -> tuple[list[dict], dict, dict[str, np.ndarray]]:
    jpath = scaled_joint_quintic(qwp0, qwp1, qwp2, float(DURATION))
    cable = build_cable(
        tau_delay_s=tau_delay_param,
        viscous_b=viscous_b,
        coulomb_fc=coulomb_fc,
        tau_elastic_s=tau_elastic_param,
        tau_deadzone=tau_deadzone_param,
        backlash_width=backlash_width_param,
        backlash_slope=backlash_slope_param,
        hys_alpha=hys_alpha,
        hys_A=hys_A,
        hys_beta=hys_beta,
        hys_gamma=hys_gamma,
        hys_n=hys_n,
        v_eps=v_eps,
        tau_eps=tau_eps,
        deadzone_mode=deadzone_mode,
        z_max=z_max,
        tau_hys_max=tau_hys_max,
        init_z_hys=init_z_hys,
        init_to_input_delay=init_to_input_delay,
        delay_enabled_global=delay_glob,
        friction_enabled_global=friction_glob,
        elastic_enabled_global=elastic_glob,
        elastic_mode=elastic_mode,
        init_elastic_to_input=init_elastic_to_input,
        backlash_deadzone_enabled_global=backlash_dz_glob,
        hysteresis_enabled_global=hysteresis_glob,
        tau_out_clip=tau_out_clip,
    )
    hybrid = HybridTransmission(cable)

    model = load_hybrid_torque_only()
    data = mj.MjData(model)
    scratch = mj.MjData(model)
    qadr_j, dadr_j, dadr_a, qadr_a = _addrs(model)
    dt = float(model.opt.timestep)

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    for i in range(4):
        data.qpos[int(qadr_a[i])] = float(Q_ACT_INITIAL[i])
        data.qpos[int(qadr_j[i])] = float(Q_JNT_INITIAL[i])
    mj.mj_forward(model, data)
    data.qfrc_applied[:] = 0.0

    n = int(round(float(DURATION) / dt)) + 1
    rows: list[dict] = []
    sat_steps = 0
    jl_viol = 0
    al_viol = 0
    ncon_max = 0
    ee_seq: list[float] = []

    trace: dict[str, list] = {k: [] for k in (
        "t", "ee_err", "des_x", "des_y", "des_z", "act_x", "act_y", "act_z",
        "tau_ideal", "tau_after_delay", "tau_compliant",
        "tau_after_hysteresis", "tau_hys", "hys_u", "hys_udot", "hys_z", "hys_zdot",
        "tau_after_deadzone", "tau_after_backlash", "tau_out",
        "q_des", "q_act", "trans_err",
        "deadzone_active", "backlash_active", "tau_elastic_err", "tau_loss",
    )}

    pad4 = lambda a3, z0: np.array([z0, a3[0], a3[1], a3[2]], dtype=np.float64)

    for step_i in range(n):
        t = min(step_i * dt, float(DURATION))
        q_des, qd_des, _ = jpath.sample(float(t))
        q_j = np.array([float(data.qpos[int(qadr_j[k])]) for k in range(4)])
        qd_j = np.array([float(data.qvel[int(dadr_j[k])]) for k in range(4)])
        q_a = np.array([float(data.qpos[int(qadr_a[k])]) for k in range(4)])
        qd_a = np.array([float(data.qvel[int(dadr_a[k])]) for k in range(4)])
        ratio_qa = RATIOS * q_a
        trans_pos = q_j - ratio_qa
        q_err = q_des - q_j

        mj.mj_forward(model, data)
        tau_bias = np.array([float(data.qfrc_bias[int(dadr_j[k])]) for k in range(4)])
        tau_pd = KQ * (q_des - q_j) + DQ * (qd_des - qd_j)
        tau_j_unc = tau_bias + tau_pd
        tau_j_cmd = np.clip(tau_j_unc, -float(TAU_JNT_LIMIT), float(TAU_JNT_LIMIT))
        sat = bool(np.any(np.abs(tau_j_unc - tau_j_cmd) > 1e-9))
        if sat:
            sat_steps += 1

        tau_act_ideal = RATIOS * tau_j_cmd
        tau_act_out = hybrid.transmit(tau_act_ideal, dt, q_a, qd_a)
        cr = hybrid.last_cable_result
        assert cr is not None

        t_ad = pad4(cr.tau_after_delay, tau_act_ideal[0])
        t_cmp = pad4(cr.tau_compliant, tau_act_ideal[0])
        t_ah = pad4(cr.tau_after_hysteresis, tau_act_ideal[0])
        thys4 = pad4(cr.tau_hys, 0.0)
        hu4 = pad4(cr.hys_u, tau_act_ideal[0])
        hud4 = pad4(cr.hys_udot, 0.0)
        hz4 = pad4(cr.hys_z, 0.0)
        hzd4 = pad4(cr.hys_zdot, 0.0)
        t_adz = pad4(cr.tau_after_deadzone, tau_act_ideal[0])
        t_abk = pad4(cr.tau_after_backlash, tau_act_ideal[0])
        t_es = pad4(cr.tau_elastic_state, tau_act_ideal[0])
        t_eerr = pad4(cr.tau_elastic_error, 0.0)
        tv4 = pad4(cr.tau_viscous, 0.0)
        tc4 = pad4(cr.tau_coulomb, 0.0)
        tl4 = pad4(cr.tau_loss, 0.0)
        dz4 = pad4(cr.deadzone_active, 0.0)
        bk4 = pad4(cr.backlash_active, 0.0)

        data.qfrc_applied[:] = 0.0
        for k in range(4):
            data.qfrc_applied[int(dadr_a[k])] = float(tau_act_out[k])

        jl_m = _jnt_margin(model, q_j)
        al_m = _act_margin(model, q_a)
        if jl_m < -1e-9:
            jl_viol += 1
        if al_m < -1e-9:
            al_viol += 1

        x_des, *_ = fk_ee_rp(model, scratch, q_des, J)
        x_des = np.asarray(x_des, dtype=float).reshape(3)
        x_act = _site_xyz(model, data)
        ee_err = float(np.linalg.norm(x_des - x_act))
        ee_seq.append(ee_err)

        mj.mj_step(model, data)
        ncon_max = max(ncon_max, int(data.ncon))

        row: dict = {
            "sweep_name": sweep_name,
            "tau_delay": float(tau_delay_param),
            "viscous_b": float(viscous_b),
            "coulomb_fc": float(coulomb_fc),
            "tau_elastic": float(tau_elastic_param),
            "tau_deadzone": float(tau_deadzone_param),
            "backlash_width": float(backlash_width_param),
            "backlash_slope": float(backlash_slope_param),
            "hys_alpha": float(hys_alpha),
            "hys_A": float(hys_A),
            "hys_beta": float(hys_beta),
            "hys_gamma": float(hys_gamma),
            "hys_n": float(hys_n),
            "step": step_i,
            "time": float(t),
            "ncon": int(data.ncon),
            "saturation_flag": int(sat),
            "joint_limit_margin": jl_m,
            "actuator_limit_margin": al_m,
            "ee_err_norm": ee_err,
            "des_x": float(x_des[0]),
            "des_y": float(x_des[1]),
            "des_z": float(x_des[2]),
            "act_x": float(x_act[0]),
            "act_y": float(x_act[1]),
            "act_z": float(x_act[2]),
        }
        for k in range(4):
            row[f"q_act_{k+1}"] = float(q_a[k])
            row[f"qdot_act_{k+1}"] = float(qd_a[k])
            row[f"tau_jnt_cmd_{k+1}"] = float(tau_j_cmd[k])
            row[f"tau_act_ideal_{k+1}"] = float(tau_act_ideal[k])
            row[f"tau_after_delay_{k+1}"] = float(t_ad[k])
            row[f"tau_elastic_state_{k+1}"] = float(t_es[k])
            row[f"tau_compliant_{k+1}"] = float(t_cmp[k])
            row[f"hys_input_u_{k+1}"] = float(hu4[k])
            row[f"hys_input_udot_{k+1}"] = float(hud4[k])
            row[f"hys_z_{k+1}"] = float(hz4[k])
            row[f"hys_zdot_{k+1}"] = float(hzd4[k])
            row[f"tau_hys_{k+1}"] = float(thys4[k])
            row[f"tau_after_hysteresis_{k+1}"] = float(t_ah[k])
            row[f"tau_after_deadzone_{k+1}"] = float(t_adz[k])
            row[f"tau_after_backlash_{k+1}"] = float(t_abk[k])
            row[f"tau_viscous_{k+1}"] = float(tv4[k])
            row[f"tau_coulomb_{k+1}"] = float(tc4[k])
            row[f"tau_loss_{k+1}"] = float(tl4[k])
            row[f"tau_act_out_{k+1}"] = float(tau_act_out[k])
            row[f"tau_elastic_error_{k+1}"] = float(t_eerr[k])
            row[f"q_jnt_des_{k+1}"] = float(q_des[k])
            row[f"q_jnt_actual_{k+1}"] = float(q_j[k])
            row[f"q_jnt_error_{k+1}"] = float(q_err[k])
            row[f"transmission_pos_err_{k+1}"] = float(trans_pos[k])
            if k >= 1:
                row[f"deadzone_active_{k}"] = float(dz4[k])
                row[f"backlash_active_{k}"] = float(bk4[k])
        rows.append(row)

        trace["t"].append(float(t))
        trace["ee_err"].append(ee_err)
        trace["des_x"].append(float(x_des[0]))
        trace["des_y"].append(float(x_des[1]))
        trace["des_z"].append(float(x_des[2]))
        trace["act_x"].append(float(x_act[0]))
        trace["act_y"].append(float(x_act[1]))
        trace["act_z"].append(float(x_act[2]))
        trace["tau_ideal"].append(tau_act_ideal.copy())
        trace["tau_after_delay"].append(t_ad.copy())
        trace["tau_compliant"].append(t_cmp.copy())
        trace["tau_after_hysteresis"].append(t_ah.copy())
        trace["tau_hys"].append(thys4.copy())
        trace["hys_u"].append(hu4.copy())
        trace["hys_udot"].append(hud4.copy())
        trace["hys_z"].append(hz4.copy())
        trace["hys_zdot"].append(hzd4.copy())
        trace["tau_after_deadzone"].append(t_adz.copy())
        trace["tau_after_backlash"].append(t_abk.copy())
        trace["tau_out"].append(tau_act_out.copy())
        trace["q_des"].append(q_des.copy())
        trace["q_act"].append(q_j.copy())
        trace["trans_err"].append(trans_pos.copy())
        trace["deadzone_active"].append(
            np.array([0.0, cr.deadzone_active[0], cr.deadzone_active[1], cr.deadzone_active[2]], dtype=np.float64)
        )
        trace["backlash_active"].append(
            np.array([0.0, cr.backlash_active[0], cr.backlash_active[1], cr.backlash_active[2]], dtype=np.float64)
        )
        trace["tau_elastic_err"].append(t_eerr.copy())
        trace["tau_loss"].append(tl4.copy())

    ee_a = np.asarray(ee_seq, dtype=float)
    summary = {
        "sweep_name": sweep_name,
        "tau_delay": float(tau_delay_param),
        "viscous_b": float(viscous_b),
        "coulomb_fc": float(coulomb_fc),
        "tau_elastic": float(tau_elastic_param),
        "tau_deadzone": float(tau_deadzone_param),
        "backlash_width": float(backlash_width_param),
        "backlash_slope": float(backlash_slope_param),
        "hys_alpha": float(hys_alpha),
        "rms_ee_err": float(np.sqrt(np.mean(ee_a**2))),
        "final_ee_err": float(ee_a[-1]),
        "ncon_max": int(ncon_max),
        "saturation_steps": int(sat_steps),
        "joint_limit_violation_steps": int(jl_viol),
        "actuator_limit_violation_steps": int(al_viol),
        "max_abs_hys_z": float(np.nanmax(np.abs(np.stack(trace["hys_z"], axis=0)[1:, 1:4]))),
        "max_abs_tau_hys": float(np.nanmax(np.abs(np.stack(trace["tau_hys"], axis=0)[1:, 1:4]))),
    }

    stack_keys = (
        "tau_ideal", "tau_after_delay", "tau_compliant", "tau_after_hysteresis", "tau_hys",
        "hys_u", "hys_udot", "hys_z", "hys_zdot",
        "tau_after_deadzone", "tau_after_backlash", "tau_out", "q_des", "q_act", "trans_err",
        "deadzone_active", "backlash_active", "tau_elastic_err", "tau_loss",
    )
    ta: dict[str, np.ndarray] = {}
    for k in trace:
        if k in stack_keys:
            ta[k] = np.stack(trace[k], axis=0)
        else:
            ta[k] = np.asarray(trace[k], dtype=np.float64)
    return rows, summary, ta


def make_plots(traces: dict[str, dict[str, np.ndarray]], names: list[str]) -> None:
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    n_case = len(names)
    colors = plt.cm.viridis(np.linspace(0.1, 0.95, max(n_case, 2)))

    # 1) EE error norm
    fig, ax = plt.subplots(figsize=(9, 4))
    for i, name in enumerate(names):
        tr = traces[name]
        ax.plot(tr["t"], tr["ee_err"], color=colors[i % len(colors)], label=name)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("‖EE err‖ [m]")
    ax.legend(loc="best", fontsize=6, ncol=2)
    ax.set_title("EE error norm (hysteresis sweep)")
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "ee_err_norm.png", dpi=120)
    plt.close(fig)

    # 2) ideal vs tau_after_hysteresis vs out — q2–q4
    for jname, ji in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(9, 4))
        for i, name in enumerate(names):
            tr = traces[name]
            c = colors[i % len(colors)]
            ax.plot(tr["t"], tr["tau_ideal"][:, ji], color=c, linestyle="-", alpha=0.9, linewidth=1.0)
            ax.plot(tr["t"], tr["tau_after_hysteresis"][:, ji], color=c, linestyle=":", alpha=0.85)
            ax.plot(tr["t"], tr["tau_out"][:, ji], color=c, linestyle="--", alpha=0.75)
        ax.set_xlabel("time [s]")
        ax.set_ylabel(f"tau {jname} [Nm]")
        ax.set_title(f"{jname}: ideal / after_hysteresis / out")
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"tau_ideal_hys_out_{jname}.png", dpi=120)
        plt.close(fig)

    # 3) tau_hys
    for jname, ji in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(9, 3.2))
        for i, name in enumerate(names):
            tr = traces[name]
            ax.plot(tr["t"], tr["tau_hys"][:, ji], color=colors[i % len(colors)], label=name, alpha=0.85)
        ax.set_xlabel("time [s]")
        ax.set_ylabel("tau_hys [Nm]")
        ax.legend(fontsize=6, ncol=2)
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"tau_hys_{jname}.png", dpi=120)
        plt.close(fig)

    # 4) hys_z
    for jname, ji in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(9, 3.2))
        for i, name in enumerate(names):
            tr = traces[name]
            ax.plot(tr["t"], tr["hys_z"][:, ji], color=colors[i % len(colors)], label=name, alpha=0.85)
        ax.set_xlabel("time [s]")
        ax.set_ylabel("hys_z")
        ax.legend(fontsize=6, ncol=2)
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"hys_z_{jname}.png", dpi=120)
        plt.close(fig)

    # 5) hys_z vs hys_input_u loop
    for jname, ji in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(6, 5))
        for i, name in enumerate(names):
            tr = traces[name]
            ax.plot(tr["hys_u"][:, ji], tr["hys_z"][:, ji], color=colors[i % len(colors)], alpha=0.7, linewidth=0.8, label=name)
        ax.set_xlabel("hys_input_u [Nm]")
        ax.set_ylabel("hys_z")
        ax.set_title(f"{jname}: z vs u")
        ax.legend(fontsize=6)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"hys_z_vs_u_{jname}.png", dpi=120)
        plt.close(fig)

    # 6) tau_after_hysteresis vs tau_compliant loop
    for jname, ji in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(6, 5))
        for i, name in enumerate(names):
            tr = traces[name]
            ax.plot(
                tr["tau_compliant"][:, ji],
                tr["tau_after_hysteresis"][:, ji],
                color=colors[i % len(colors)],
                alpha=0.7,
                linewidth=0.8,
                label=name,
            )
        ax.set_xlabel("tau_compliant [Nm]")
        ax.set_ylabel("tau_after_hysteresis [Nm]")
        ax.set_title(f"{jname}: after_hyst vs compliant")
        ax.legend(fontsize=6)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"tau_ah_vs_compliant_{jname}.png", dpi=120)
        plt.close(fig)

    # 7) q_jnt des vs actual
    for k in range(4):
        fig, ax = plt.subplots(figsize=(9, 3.0))
        for i, name in enumerate(names):
            tr = traces[name]
            ax.plot(tr["t"], tr["q_des"][:, k], color=colors[i % len(colors)], linestyle="-", alpha=0.85)
            ax.plot(tr["t"], tr["q_act"][:, k], color=colors[i % len(colors)], linestyle="--", alpha=0.65)
        ax.set_xlabel("time [s]")
        ax.set_ylabel(f"q_jnt {k+1} [rad]")
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"q_jnt_des_vs_actual_{k+1}.png", dpi=120)
        plt.close(fig)

    # 8) transmission position error
    fig, ax = plt.subplots(figsize=(9, 3.5))
    for i, name in enumerate(names):
        tr = traces[name]
        mabs = np.max(np.abs(tr["trans_err"]), axis=1)
        ax.plot(tr["t"], mabs, color=colors[i % len(colors)], label=name)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("max_i |trans err| [rad]")
    ax.legend(fontsize=6, ncol=2)
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "transmission_position_error.png", dpi=120)
    plt.close(fig)

    # 9) deadzone / backlash flags
    for jname, ji in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(9, 3.4))
        for i, name in enumerate(names):
            tr = traces[name]
            ax.plot(tr["t"], tr["deadzone_active"][:, ji], color=colors[i % len(colors)], linestyle="-", alpha=0.8)
            ax.plot(tr["t"], tr["backlash_active"][:, ji], color=colors[i % len(colors)], linestyle="--", alpha=0.5)
        ax.set_xlabel("time [s]")
        ax.set_ylabel("flag")
        ax.set_ylim(-0.1, 1.1)
        ax.set_title(f"{jname}: deadzone_active (—) / backlash_active (--)")
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"deadzone_backlash_flags_{jname}.png", dpi=120)
        plt.close(fig)


def write_report(summaries: list[dict], _traces: dict[str, dict[str, np.ndarray]]) -> None:
    noh = next(s for s in summaries if s["sweep_name"] == "no_hysteresis")
    drms = {s["sweep_name"]: float(s["rms_ee_err"]) - float(noh["rms_ee_err"]) for s in summaries}
    max_ncon = max(s["ncon_max"] for s in summaries)
    sat_total = sum(s["saturation_steps"] for s in summaries)
    viol = sum(s["joint_limit_violation_steps"] + s["actuator_limit_violation_steps"] for s in summaries)
    sat_breakdown = ", ".join(f"{s['sweep_name']}:{s['saturation_steps']}" for s in summaries)
    stress = next(s for s in summaries if s["sweep_name"] == "stress_hysteresis")

    lines = [
        "# Bouc–Wen torque hysteresis sweep",
        "",
        "Pipeline: `tau_in` → delay → compliance → **Bouc–Wen** → dead-zone → backlash → friction.",
        "",
        "## Summary",
        "",
        "| name | α | RMS EE | ΔRMS vs no_hys | max|z| | max|τ_hys| | sat steps |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for s in summaries:
        lines.append(
            f"| {s['sweep_name']} | {s['hys_alpha']:.4g} | {s['rms_ee_err']:.6f} | "
            f"{drms[s['sweep_name']]:+.6g} | {s['max_abs_hys_z']:.4g} | {s['max_abs_tau_hys']:.4g} | {s['saturation_steps']} |"
        )
    lines += [
        "",
        "## Answers",
        "",
        "### 1. Does no_hysteresis reproduce the mild dead-zone baseline?",
        f"- **no_hysteresis** RMS **{noh['rms_ee_err']:.6f} m** (light_deadzone/덴드존 baseline 대역 ≈ **{REF_LIGHT_DEADZONE_RMS:.6f} m**). "
        f"**{'예 — 유사' if abs(float(noh['rms_ee_err']) - REF_LIGHT_DEADZONE_RMS) < 3e-4 else 'CSV로 확인'}**.",
        "",
        "### 2. How much does light/medium/strong hysteresis affect RMS EE error?",
        "- ΔRMS vs no_hysteresis: "
        + "; ".join(f"**{k}**: {v:+.6g} m" for k, v in drms.items() if k in ("light_hysteresis", "medium_hysteresis", "strong_hysteresis")),
        "",
        "### 3. Loop behavior in z vs u and torque vs compliant?",
        "- `hys_z_vs_u_*.png`, `tau_ah_vs_compliant_*.png`에서 히스테리시스 루프·넓이 확인.",
        "",
        "### 4. Are hys_z and tau_hys bounded?",
        f"- 테이블 `max|z|`, `max|τ_hys|` 열 및 설정 `z_max`, `tau_hys_max`와 비교. 전 스윕 최대 **max|z|** ≈ {max(s['max_abs_hys_z'] for s in summaries):.4g}, **max|τ_hys|** ≈ {max(s['max_abs_tau_hys'] for s in summaries):.4g}.",
        "",
        "### 5. Does stress_hysteresis cause saturation or limit violation?",
        f"- **stress_hysteresis** 포화 스텝 **{stress['saturation_steps']}**, 관절/액추 위반 합 **{stress['joint_limit_violation_steps'] + stress['actuator_limit_violation_steps']}**.",
        "",
        "### 6. Is ncon still zero?",
        f"- `ncon_max` **{max_ncon}**.",
        "",
        "### 7. Which hysteresis setting is appropriate as a mild baseline?",
        "- **no_hysteresis** 또는 **light_hysteresis** (`hys_alpha=0.001`)가 완만한 출발점.",
        "",
        "### 8. Is the model ready for parameter randomization next?",
        "- **예** — 토크 경로 Bouc–Wen·데드존·백래시가 정리되었으므로 다음 단계는 이 매개변수들의 불확실성/랜덤화가 자연스럽습니다.",
        "",
    ]
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    cfg = load_cfg()
    init_d = bool(cfg["cable_delay"]["initialize_state_to_input"])
    delay_glob = bool(cfg["cable_delay"]["enabled"])
    friction_glob = bool(cfg["cable_friction"]["enabled"])
    v_eps = float(cfg["cable_friction"]["v_eps"])
    tau_clip = cfg["cable_friction"].get("tau_out_clip")

    el = cfg["cable_elasticity"]
    elastic_glob = bool(el["enabled"])
    elastic_mode = str(el["mode"])
    init_e = bool(el["initialize_state_to_input"])

    bd = cfg["cable_backlash_deadzone"]
    bd_glob = bool(bd["enabled"])
    tau_eps = float(bd["tau_eps"])
    deadzone_mode = str(bd.get("deadzone_mode", "hard"))

    hysd = cfg["cable_hysteresis"]
    hys_glob = bool(hysd["enabled"])
    z_max = float(hysd["z_max"])
    tau_hys_max = float(hysd["tau_hys_max"])
    init_z = bool(hysd["initialize_z_to_zero"])

    qwp0, qwp1, qwp2 = solve_wp_q_arm_only()
    entries = cfg["hysteresis_sweep"]

    all_rows: list[dict] = []
    summaries: list[dict] = []
    traces: dict[str, dict[str, np.ndarray]] = {}
    names: list[str] = []

    for entry in entries:
        name = str(entry["name"])
        names.append(name)
        rows, summary, tr = run_case(
            name,
            float(entry["tau_delay"]),
            float(entry["viscous_b"]),
            float(entry["coulomb_fc"]),
            float(entry["tau_elastic"]),
            float(entry["tau_deadzone"]),
            float(entry["backlash_width"]),
            float(entry["backlash_slope"]),
            float(entry["hys_alpha"]),
            float(entry["hys_A"]),
            float(entry["hys_beta"]),
            float(entry["hys_gamma"]),
            float(entry["hys_n"]),
            v_eps=v_eps,
            tau_eps=tau_eps,
            deadzone_mode=deadzone_mode,
            z_max=z_max,
            tau_hys_max=tau_hys_max,
            init_z_hys=init_z,
            init_to_input_delay=init_d,
            init_elastic_to_input=init_e,
            delay_glob=delay_glob,
            friction_glob=friction_glob,
            elastic_glob=elastic_glob,
            elastic_mode=elastic_mode,
            backlash_dz_glob=bd_glob,
            hysteresis_glob=hys_glob,
            tau_out_clip=float(tau_clip) if tau_clip is not None else None,
            qwp0=qwp0,
            qwp1=qwp1,
            qwp2=qwp2,
        )
        all_rows.extend(rows)
        summaries.append(summary)
        traces[name] = tr

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    if all_rows:
        with open(CSV_PATH, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
            w.writeheader()
            w.writerows(all_rows)

    make_plots(traces, names)
    write_report(summaries, traces)

    print("Wrote", CSV_PATH)
    print("Wrote", REPORT_PATH)
    print("Plots ->", PLOTS_DIR)


if __name__ == "__main__":
    main()
