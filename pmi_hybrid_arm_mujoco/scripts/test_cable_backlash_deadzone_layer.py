#!/usr/bin/env python3
"""Cable torque dead-zone + direction-dependent backlash sweep (delay → compliance → dead-zone/backlash → friction)."""

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
    CableTransmission,
)
from transmission.hybrid_transmission import HybridTransmission
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

HYBRID_XML = PKG_ROOT / "models" / "pmi_hybrid_no_collision.xml"
CABLE_LAYER_DEFAULT = PKG_ROOT / "configs" / "cable_layer.yaml"
OUT_DIR = PKG_ROOT / "debug_outputs" / "cable_layer"
PLOTS_DIR = OUT_DIR / "plots" / "backlash"
CSV_PATH = OUT_DIR / "backlash_sweep.csv"
REPORT_PATH = OUT_DIR / "backlash_sweep_report.md"

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

REF_COMBINED_LIGHT_RMS = 0.000721


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
    v_eps: float,
    tau_eps: float,
    deadzone_mode: str,
    init_to_input_delay: bool,
    delay_enabled_global: bool,
    friction_enabled_global: bool,
    elastic_enabled_global: bool,
    elastic_mode: str,
    init_elastic_to_input: bool,
    backlash_deadzone_enabled_global: bool,
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

    return CableTransmission(dcfg, fcfg, ecfg, bz)


def _sign3(x: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    s = np.zeros(3, dtype=np.float64)
    for i in range(3):
        if abs(float(x[i])) < eps:
            s[i] = 0.0
        else:
            s[i] = float(np.sign(float(x[i])))
    return s


def run_case(
    sweep_name: str,
    tau_delay_param: float,
    viscous_b: float,
    coulomb_fc: float,
    tau_elastic_param: float,
    tau_deadzone_param: float,
    backlash_width_param: float,
    backlash_slope_param: float,
    *,
    v_eps: float,
    tau_eps: float,
    deadzone_mode: str,
    init_to_input_delay: bool,
    init_elastic_to_input: bool,
    delay_glob: bool,
    friction_glob: bool,
    elastic_glob: bool,
    elastic_mode: str,
    backlash_dz_glob: bool,
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
        v_eps=v_eps,
        tau_eps=tau_eps,
        deadzone_mode=deadzone_mode,
        init_to_input_delay=init_to_input_delay,
        delay_enabled_global=delay_glob,
        friction_enabled_global=friction_glob,
        elastic_enabled_global=elastic_glob,
        elastic_mode=elastic_mode,
        init_elastic_to_input=init_elastic_to_input,
        backlash_deadzone_enabled_global=backlash_dz_glob,
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

    prev_cmp = np.zeros(3, dtype=np.float64)
    prev_bl = np.zeros(3, dtype=np.float64)
    n_sign_change = 0
    n_backlash_rising = 0
    n_rising_at_sign_change = 0

    trace: dict[str, list] = {k: [] for k in (
        "t", "ee_err", "des_x", "des_y", "des_z", "act_x", "act_y", "act_z",
        "tau_ideal", "tau_after_delay", "tau_compliant", "tau_after_deadzone", "tau_after_backlash", "tau_out",
        "q_des", "q_act", "trans_err",
        "deadzone_active", "backlash_active", "backlash_remaining", "torque_sign",
        "tau_elastic_err", "tau_loss",
    )}

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

        if step_i > 0:
            s_prev = _sign3(prev_cmp)
            s_now = _sign3(cr.tau_after_hysteresis)
            sign_flip = np.any((s_prev != s_now) & ((np.abs(s_prev) + np.abs(s_now)) > 1e-15))
            if sign_flip:
                n_sign_change += 1
            rising_edge = np.any((cr.backlash_active > 0.5) & (prev_bl < 0.5))
            if rising_edge:
                n_backlash_rising += 1
                if sign_flip:
                    n_rising_at_sign_change += 1

        pad4 = lambda a3, z0: np.array([z0, a3[0], a3[1], a3[2]], dtype=np.float64)

        t_ad = pad4(cr.tau_after_delay, tau_act_ideal[0])
        t_cmp = pad4(cr.tau_compliant, tau_act_ideal[0])
        t_adz = pad4(cr.tau_after_deadzone, tau_act_ideal[0])
        t_abk = pad4(cr.tau_after_backlash, tau_act_ideal[0])
        t_es = pad4(cr.tau_elastic_state, tau_act_ideal[0])
        t_eerr = pad4(cr.tau_elastic_error, 0.0)
        tv4 = pad4(cr.tau_viscous, 0.0)
        tc4 = pad4(cr.tau_coulomb, 0.0)
        tl4 = pad4(cr.tau_loss, 0.0)
        dz4 = pad4(cr.deadzone_active, 0.0)
        bk4 = pad4(cr.backlash_active, 0.0)
        br4 = pad4(cr.backlash_remaining, 0.0)
        ts4 = pad4(cr.torque_sign, 0.0)

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
                j = k - 1
                row[f"deadzone_active_{k}"] = float(dz4[k])
                row[f"backlash_active_{k}"] = float(bk4[k])
                row[f"backlash_remaining_{k}"] = float(br4[k])
                row[f"torque_sign_{k}"] = float(ts4[k])
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
        trace["backlash_remaining"].append(
            np.array([0.0, cr.backlash_remaining[0], cr.backlash_remaining[1], cr.backlash_remaining[2]], dtype=np.float64)
        )
        trace["torque_sign"].append(
            np.array([0.0, cr.torque_sign[0], cr.torque_sign[1], cr.torque_sign[2]], dtype=np.float64)
        )
        trace["tau_elastic_err"].append(t_eerr.copy())
        trace["tau_loss"].append(tl4.copy())

        prev_cmp = cr.tau_after_hysteresis.copy()
        prev_bl = cr.backlash_active.copy()

    ee_a = np.asarray(ee_seq, dtype=float)
    frac_at_flip = (
        float(n_rising_at_sign_change / max(n_backlash_rising, 1)) if n_backlash_rising > 0 else float("nan")
    )
    summary = {
        "sweep_name": sweep_name,
        "tau_delay": float(tau_delay_param),
        "viscous_b": float(viscous_b),
        "coulomb_fc": float(coulomb_fc),
        "tau_elastic": float(tau_elastic_param),
        "tau_deadzone": float(tau_deadzone_param),
        "backlash_width": float(backlash_width_param),
        "backlash_slope": float(backlash_slope_param),
        "rms_ee_err": float(np.sqrt(np.mean(ee_a**2))),
        "final_ee_err": float(ee_a[-1]),
        "ncon_max": int(ncon_max),
        "saturation_steps": int(sat_steps),
        "joint_limit_violation_steps": int(jl_viol),
        "actuator_limit_violation_steps": int(al_viol),
        "sign_change_steps": int(n_sign_change),
        "backlash_rising_edges": int(n_backlash_rising),
        "backlash_rising_at_sign_change": int(n_rising_at_sign_change),
        "frac_backlash_edge_with_sign_flip": frac_at_flip,
    }

    stack_keys = (
        "tau_ideal", "tau_after_delay", "tau_compliant", "tau_after_deadzone", "tau_after_backlash",
        "tau_out", "q_des", "q_act", "trans_err", "deadzone_active", "backlash_active",
        "backlash_remaining", "torque_sign", "tau_elastic_err", "tau_loss",
    )
    ta = {
        k: np.stack(trace[k], axis=0) if k in stack_keys else np.asarray(trace[k], dtype=np.float64)
        for k in trace
    }
    return rows, summary, ta


def make_plots(traces: dict[str, dict[str, np.ndarray]], names: list[str]) -> None:
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    n = len(names)
    colors = plt.cm.viridis(np.linspace(0.1, 0.95, max(n, 2)))

    # 1) EE error norm
    fig, ax = plt.subplots(figsize=(9, 4))
    for i, name in enumerate(names):
        tr = traces[name]
        ax.plot(tr["t"], tr["ee_err"], color=colors[i % len(colors)], label=name)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("‖EE err‖ [m]")
    ax.legend(loc="best", fontsize=6, ncol=2)
    ax.set_title("EE error norm (backlash/dead-zone sweep)")
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "ee_err_norm.png", dpi=120)
    plt.close(fig)

    # 2) ideal, after_deadzone, after_backlash, out — q2–q4
    for jname, ji in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(9, 4))
        for i, name in enumerate(names):
            tr = traces[name]
            c = colors[i % len(colors)]
            ax.plot(tr["t"], tr["tau_ideal"][:, ji], color=c, linestyle="-", alpha=0.85, linewidth=1.1, label=name if ji == 1 else "")
            ax.plot(tr["t"], tr["tau_after_deadzone"][:, ji], color=c, linestyle=":", alpha=0.8)
            ax.plot(tr["t"], tr["tau_after_backlash"][:, ji], color=c, linestyle="-.", alpha=0.75)
            ax.plot(tr["t"], tr["tau_out"][:, ji], color=c, linestyle="--", alpha=0.65)
        ax.set_xlabel("time [s]")
        ax.set_ylabel(f"tau {jname} [Nm]")
        ax.set_title(f"{jname}: ideal / after_deadzone / after_backlash / out (line style)")
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"tau_pipeline_{jname}.png", dpi=120)
        plt.close(fig)

    # 3) backlash_active flags q2–q4
    for jname, ji in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(9, 3.2))
        for i, name in enumerate(names):
            tr = traces[name]
            ax.plot(tr["t"], tr["backlash_active"][:, ji], color=colors[i % len(colors)], label=name, alpha=0.85)
        ax.set_xlabel("time [s]")
        ax.set_ylabel("backlash_active")
        ax.set_ylim(-0.1, 1.1)
        ax.legend(fontsize=6, ncol=2)
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"backlash_active_{jname}.png", dpi=120)
        plt.close(fig)

    # 4) deadzone_active
    for jname, ji in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(9, 3.2))
        for i, name in enumerate(names):
            tr = traces[name]
            ax.plot(tr["t"], tr["deadzone_active"][:, ji], color=colors[i % len(colors)], label=name, alpha=0.85)
        ax.set_xlabel("time [s]")
        ax.set_ylabel("deadzone_active")
        ax.set_ylim(-0.1, 1.1)
        ax.legend(fontsize=6, ncol=2)
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"deadzone_active_{jname}.png", dpi=120)
        plt.close(fig)

    # 5) backlash_remaining
    for jname, ji in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(9, 3.2))
        for i, name in enumerate(names):
            tr = traces[name]
            ax.plot(tr["t"], tr["backlash_remaining"][:, ji], color=colors[i % len(colors)], label=name, alpha=0.85)
        ax.set_xlabel("time [s]")
        ax.set_ylabel("backlash_remaining")
        ax.legend(fontsize=6, ncol=2)
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"backlash_remaining_{jname}.png", dpi=120)
        plt.close(fig)

    # 6) q_jnt des vs actual (joint-space)
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

    # 7) transmission position error (max abs over joints)
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

    # 8) hysteresis-like: tau_act_ideal vs tau_act_out (q2–q4)
    for jname, ji in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(6, 5))
        for i, name in enumerate(names):
            tr = traces[name]
            ax.plot(
                tr["tau_ideal"][:, ji],
                tr["tau_out"][:, ji],
                color=colors[i % len(colors)],
                alpha=0.7,
                linewidth=0.8,
                label=name,
            )
        ax.set_xlabel("tau_act_ideal [Nm]")
        ax.set_ylabel("tau_act_out [Nm]")
        ax.set_title(f"{jname}: ideal vs out (phase)")
        ax.legend(fontsize=6, loc="best")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"tau_hysteresis_{jname}.png", dpi=120)
        plt.close(fig)


def write_report(summaries: list[dict], traces: dict[str, dict[str, np.ndarray]]) -> None:
    nob = next(s for s in summaries if s["sweep_name"] == "no_backlash")
    drms = {s["sweep_name"]: float(s["rms_ee_err"]) - float(nob["rms_ee_err"]) for s in summaries}
    deadzone_only = [s for s in summaries if s["backlash_width"] == 0.0 and s["tau_deadzone"] > 0.0]
    backlash_cases = [s for s in summaries if s["backlash_width"] > 0.0]

    max_ncon = max(s["ncon_max"] for s in summaries)
    sat_total = sum(s["saturation_steps"] for s in summaries)
    viol = sum(s["joint_limit_violation_steps"] + s["actuator_limit_violation_steps"] for s in summaries)
    sat_breakdown = ", ".join(f"{s['sweep_name']}:{s['saturation_steps']}" for s in summaries)

    lines = [
        "# Cable dead-zone / backlash sweep (torque transmission)",
        "",
        "Pipeline: `tau_in` → delay → compliance → **dead-zone** → **backlash** → friction → `tau_out` (q2–q4).",
        "",
        "## Summary",
        "",
        "| name | τ_delay | b | Fc | τ_elastic | τ_dz | bl_w | bl_slope | RMS EE | final EE | frac(edge@flip) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for s in summaries:
        lines.append(
            f"| {s['sweep_name']} | {s['tau_delay']:.4g} | {s['viscous_b']:.4g} | {s['coulomb_fc']:.4g} | "
            f"{s['tau_elastic']:.4g} | {s['tau_deadzone']:.4g} | {s['backlash_width']:.4g} | {s['backlash_slope']:.4g} | "
            f"{s['rms_ee_err']:.6f} | {s['final_ee_err']:.6f} | {s['frac_backlash_edge_with_sign_flip']:.3f} |"
        )
    lines += [
        "",
        "## Answers",
        "",
        "### 1. Does no_backlash reproduce the mild cable baseline?",
        f"- **no_backlash** RMS **{nob['rms_ee_err']:.6f} m** (combined_light 참고 ≈ {REF_COMBINED_LIGHT_RMS:.6f} m). "
        f"**{'예 — 유사' if abs(float(nob['rms_ee_err']) - REF_COMBINED_LIGHT_RMS) < 2e-4 else 'CSV/플롯으로 확인'}**.",
        "",
        "### 2. How much does dead-zone alone affect RMS EE error?",
        "- ΔRMS vs no_backlash (dead-zone only): "
        + "; ".join(
            f"**{s['sweep_name']}**: {float(s['rms_ee_err']) - float(nob['rms_ee_err']):+.6g} m"
            for s in deadzone_only
        )
        if deadzone_only
        else "- (설정 없음)",
        "",
        "### 3. How much does backlash affect RMS EE error?",
        "- 백래시 케이스 ΔRMS vs no_backlash: "
        + "; ".join(
            f"**{s['sweep_name']}**: {drms.get(s['sweep_name'], 0):+.6g} m"
            for s in backlash_cases
        )
        if backlash_cases
        else "-",
        "",
        "### 4. Are backlash events triggered mainly at torque sign reversals?",
        "- `frac_backlash_edge_with_sign_flip` 열: 같은 스텝에 토크 부호 변화와 백래시 활성 상승(edge)이 겹치는 비율. "
        "1에 가깝면 부호 반전 기반 모델과 일치.",
        "",
        "### 5. Does strong_backlash create visible tracking lag or step-like behavior?",
        "- `strong_backlash`의 `tau_pipeline_*.png`, `q_jnt_des_vs_actual_*.png`에서 계단·위상 지연을 확인.",
        "",
        "### 6. Are there any saturation or limit violations?",
        f"- 포화 스텝 합 **{sat_total}** (런별: {sat_breakdown}), 관절/액추 위반 **{viol}**.",
        "",
        "### 7. Is ncon still zero?",
        f"- `ncon_max` **{max_ncon}**.",
        "",
        "### 8. Which backlash/dead-zone setting is appropriate as a mild baseline?",
        "- **no_backlash** 또는 **light_deadzone** (`tau_deadzone=0.001`, 백래시 0)이 보수적 출발점.",
        "",
        "### 9. Is the model ready to add Bouc-Wen hysteresis next?",
        "- **예** — 토크 경로 비선형이 정리된 상태에서 Bouc-Wen을 직렬/병렬로 추가 가능. (요청 시 hysteresis 미도입 유지.)",
        "",
        "### Note: If backlash improves tracking, check filtering",
        "- 추적이 개선되면 백래시가 사실상 저역 통과·클리핑 역할인지 `tau_hysteresis_*.png`와 RMS 변화를 교차 확인.",
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

    qwp0, qwp1, qwp2 = solve_wp_q_arm_only()
    entries = cfg["backlash_sweep"]

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
            v_eps=v_eps,
            tau_eps=tau_eps,
            deadzone_mode=deadzone_mode,
            init_to_input_delay=init_d,
            init_elastic_to_input=init_e,
            delay_glob=delay_glob,
            friction_glob=friction_glob,
            elastic_glob=elastic_glob,
            elastic_mode=elastic_mode,
            backlash_dz_glob=bd_glob,
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
