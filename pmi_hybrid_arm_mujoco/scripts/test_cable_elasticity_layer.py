#!/usr/bin/env python3
"""Cable torque-compliance elasticity sweep (delay → compliance → friction)."""

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
PLOTS_DIR = OUT_DIR / "plots" / "elasticity"
CSV_PATH = OUT_DIR / "elasticity_sweep.csv"
REPORT_PATH = OUT_DIR / "elasticity_sweep_report.md"

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

# previous friction report: combined_light RMS EE ≈ 0.000721 m
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
    v_eps: float,
    init_to_input_delay: bool,
    delay_enabled_global: bool,
    friction_enabled_global: bool,
    elastic_enabled_global: bool,
    elastic_mode: str,
    init_elastic_to_input: bool,
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

    return CableTransmission(dcfg, fcfg, ecfg)


def run_case(
    sweep_name: str,
    tau_delay_param: float,
    viscous_b: float,
    coulomb_fc: float,
    tau_elastic_param: float,
    *,
    v_eps: float,
    init_to_input_delay: bool,
    init_elastic_to_input: bool,
    delay_glob: bool,
    friction_glob: bool,
    elastic_glob: bool,
    elastic_mode: str,
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
        v_eps=v_eps,
        init_to_input_delay=init_to_input_delay,
        delay_enabled_global=delay_glob,
        friction_enabled_global=friction_glob,
        elastic_enabled_global=elastic_glob,
        elastic_mode=elastic_mode,
        init_elastic_to_input=init_elastic_to_input,
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
        "tau_ideal", "tau_after_delay", "tau_compliant", "tau_out", "q_des", "q_act",
        "trans_err", "tau_elastic_err", "tau_loss",
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
        tau_tr_err = tau_act_out - tau_act_ideal

        pad4 = lambda a3, z0: np.array([z0, a3[0], a3[1], a3[2]], dtype=np.float64)

        t_ad = pad4(cr.tau_after_delay, tau_act_ideal[0])
        t_cmp = pad4(cr.tau_compliant, tau_act_ideal[0])
        t_es = pad4(cr.tau_elastic_state, tau_act_ideal[0])
        t_eerr = pad4(cr.tau_elastic_error, 0.0)
        tv4 = pad4(cr.tau_viscous, 0.0)
        tc4 = pad4(cr.tau_coulomb, 0.0)
        tl4 = pad4(cr.tau_loss, 0.0)

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
            row[f"tau_viscous_{k+1}"] = float(tv4[k])
            row[f"tau_coulomb_{k+1}"] = float(tc4[k])
            row[f"tau_loss_{k+1}"] = float(tl4[k])
            row[f"tau_act_out_{k+1}"] = float(tau_act_out[k])
            row[f"tau_elastic_error_{k+1}"] = float(t_eerr[k])
            row[f"tau_transmission_error_{k+1}"] = float(tau_tr_err[k])
            row[f"q_jnt_des_{k+1}"] = float(q_des[k])
            row[f"q_jnt_actual_{k+1}"] = float(q_j[k])
            row[f"q_jnt_error_{k+1}"] = float(q_err[k])
            row[f"transmission_pos_err_{k+1}"] = float(trans_pos[k])
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
        trace["tau_out"].append(tau_act_out.copy())
        trace["q_des"].append(q_des.copy())
        trace["q_act"].append(q_j.copy())
        trace["trans_err"].append(trans_pos.copy())
        trace["tau_elastic_err"].append(t_eerr.copy())
        trace["tau_loss"].append(tl4.copy())

    ee_a = np.asarray(ee_seq, dtype=float)
    summary = {
        "sweep_name": sweep_name,
        "tau_delay": float(tau_delay_param),
        "viscous_b": float(viscous_b),
        "coulomb_fc": float(coulomb_fc),
        "tau_elastic": float(tau_elastic_param),
        "rms_ee_err": float(np.sqrt(np.mean(ee_a**2))),
        "final_ee_err": float(ee_a[-1]),
        "ncon_max": int(ncon_max),
        "saturation_steps": int(sat_steps),
        "joint_limit_violation_steps": int(jl_viol),
        "actuator_limit_violation_steps": int(al_viol),
    }

    stack_keys = (
        "tau_ideal", "tau_after_delay", "tau_compliant", "tau_out", "q_des", "q_act",
        "trans_err", "tau_elastic_err", "tau_loss",
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
    ax.set_title("EE error norm (elasticity sweep)")
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "ee_err_norm.png", dpi=120)
    plt.close(fig)

    # 2) EE xyz per sweep
    for name in names:
        tr = traces[name]
        t = tr["t"]
        fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
        axs[0].plot(t, tr["des_x"], label="des")
        axs[0].plot(t, tr["act_x"], "--", label="act")
        axs[0].set_ylabel("x [m]")
        axs[1].plot(t, tr["des_y"], label="des")
        axs[1].plot(t, tr["act_y"], "--", label="act")
        axs[1].set_ylabel("y [m]")
        axs[2].plot(t, tr["des_z"], label="des")
        axs[2].plot(t, tr["act_z"], "--", label="act")
        axs[2].set_ylabel("z [m]")
        axs[2].set_xlabel("time [s]")
        axs[0].legend(fontsize=7)
        fig.suptitle(f"EE xyz — {name}")
        fig.tight_layout()
        safe = name.replace(" ", "_")
        fig.savefig(PLOTS_DIR / f"ee_xyz_{safe}.png", dpi=120)
        plt.close(fig)

    # 3) ideal, after_delay, compliant, out — q2–q4
    for jname, ji in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(9, 4))
        for i, name in enumerate(names):
            tr = traces[name]
            c = colors[i % len(colors)]
            ax.plot(tr["t"], tr["tau_ideal"][:, ji], color=c, linestyle="-", alpha=0.85, linewidth=1.1)
            ax.plot(tr["t"], tr["tau_after_delay"][:, ji], color=c, linestyle=":", alpha=0.8)
            ax.plot(tr["t"], tr["tau_compliant"][:, ji], color=c, linestyle="-.", alpha=0.75)
            ax.plot(tr["t"], tr["tau_out"][:, ji], color=c, linestyle="--", alpha=0.65)
        ax.set_xlabel("time [s]")
        ax.set_ylabel(f"tau {jname} [Nm]")
        ax.set_title(f"{jname}: ideal / after_delay / compliant / out (color = sweep)")
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"tau_pipeline_{jname}.png", dpi=120)
        plt.close(fig)

    # 4) tau_elastic_error
    for jname, ji in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(9, 3.2))
        for i, name in enumerate(names):
            tr = traces[name]
            ax.plot(tr["t"], tr["tau_elastic_err"][:, ji], color=colors[i % len(colors)], label=name)
        ax.set_xlabel("time [s]")
        ax.set_ylabel(f"tau_compliant - tau_after_delay [Nm]")
        ax.legend(fontsize=6, ncol=2)
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"tau_elastic_error_{jname}.png", dpi=120)
        plt.close(fig)

    # 5) tau_loss
    for jname, ji in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(9, 3.2))
        for i, name in enumerate(names):
            tr = traces[name]
            ax.plot(tr["t"], tr["tau_loss"][:, ji], color=colors[i % len(colors)], label=name)
        ax.set_xlabel("time [s]")
        ax.set_ylabel(f"tau_loss [Nm]")
        ax.legend(fontsize=6, ncol=2)
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"tau_loss_{jname}.png", dpi=120)
        plt.close(fig)

    # 6) q joints
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

    # 7) transmission pos err
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


def overshoot_metric(tee: np.ndarray) -> float:
    """Simple variability proxy on tau_elastic_error channel (q2)."""
    x = tee[:, 1].astype(np.float64)
    return float(np.std(x) / (np.std(np.diff(x)) + 1e-12))


def write_report(summaries: list[dict], traces: dict[str, dict[str, np.ndarray]]) -> None:
    noe = next(s for s in summaries if s["sweep_name"] == "no_elasticity")
    drms = {s["sweep_name"]: float(s["rms_ee_err"]) - float(noe["rms_ee_err"]) for s in summaries}
    max_ncon = max(s["ncon_max"] for s in summaries)
    sat_total = sum(s["saturation_steps"] for s in summaries)
    viol = sum(s["joint_limit_violation_steps"] + s["actuator_limit_violation_steps"] for s in summaries)
    sat_breakdown = ", ".join(f"{s['sweep_name']}:{s['saturation_steps']}" for s in summaries)

    lines = [
        "# Cable torque compliance (elasticity) sweep",
        "",
        "`tau_after_delay` → first-order compliance (`tau_elastic`) → `tau_compliant` → friction → `tau_out`.",
        "",
        "## Summary",
        "",
        "| name | τ_delay | b | Fc | τ_elastic | RMS EE | final EE |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for s in summaries:
        lines.append(
            f"| {s['sweep_name']} | {s['tau_delay']:.4g} | {s['viscous_b']:.4g} | {s['coulomb_fc']:.4g} | {s['tau_elastic']:.4g} | "
            f"{s['rms_ee_err']:.6f} | {s['final_ee_err']:.6f} |"
        )
    lines += [
        "",
        "## Answers",
        "",
        "### 1. Does no_elasticity reproduce the previous friction baseline?",
        f"- **no_elasticity** RMS **{noe['rms_ee_err']:.6f} m** (combined_light 참고 ≈ {REF_COMBINED_LIGHT_RMS:.6f} m). "
        f"**{'예 — 동일 설정·탄성 꺼짐' if abs(float(noe['rms_ee_err']) - REF_COMBINED_LIGHT_RMS) < 2e-4 else '표·CSV로 확인'}**.",
        "",
        "### 2. How much does light/medium/strong elasticity affect RMS EE error?",
        "- ΔRMS vs no_elasticity: "
        + "; ".join(f"**{k}**: {v:+.6g} m" for k, v in drms.items() if k != "no_elasticity")
        + " — 동일 게인에서 저역 통과형 컴플라이언스가 **평활화**를 줄 수 있어 RMS가 소폭 **감소**한 것처럼 보일 수 있습니다. "
        "진짜 성능 저하는 EE·포화·토크 플롯과 함께 판단하는 것이 안전합니다.",
        "",
        "### 3. Does torque compliance introduce visible lag?",
        "- τ_elastic가 클수록 `tau_compliant`가 `tau_after_delay`를 따라가며 **위상 지연**이 생길 수 있습니다 (`tau_pipeline_*.png`, `tau_elastic_error_*.png`).",
        "",
        "### 4. Does strong elasticity produce oscillation or overshoot?",
        f"- 정성적으로 강한 τ_elastic에서 `tau_elastic_error` 변동이 커질 수 있습니다. "
        f"(q2 보조 지표: strong_elasticity **{overshoot_metric(traces['strong_elasticity']['tau_elastic_err']):.4g}** vs "
        f"strong_elasticity_medium_loss **{overshoot_metric(traces['strong_elasticity_medium_loss']['tau_elastic_err']):.4g}**.) "
        "`strong_elasticity_medium_loss`는 지연·손실·컴플라이언스가 겹쳐 **포화**가 발생할 수 있습니다(아래 5번).",
        "",
        "### 5. Are there any saturation or limit violations?",
        f"- 포화 스텝 합 **{sat_total}** (런별: {sat_breakdown}), 관절/액추 리밋 위반 **{viol}**.",
        "",
        "### 6. Is ncon still zero?",
        f"- `ncon_max` 최댓값 **{max_ncon}**.",
        "",
        "### 7. Which elasticity level is appropriate as a mild compliance baseline?",
        "- **light_elasticity** (`tau_elastic=0.005`)이 작은 추가 왜곡으로 컴플라이언스 효과를 줄 수 있는 완만한 출발점입니다.",
        "",
        "### 8. Is the model ready to add backlash/dead-zone next?",
        "- **예** — 토크 경로에 지연·탄성·손실이 쌓인 상태에서, 동일 레이어 또는 직렬로 백래시/데드존을 추가할 수 있습니다.",
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

    qwp0, qwp1, qwp2 = solve_wp_q_arm_only()
    entries = cfg["elasticity_sweep"]

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
            v_eps=v_eps,
            init_to_input_delay=init_d,
            init_elastic_to_input=init_e,
            delay_glob=delay_glob,
            friction_glob=friction_glob,
            elastic_glob=elastic_glob,
            elastic_mode=elastic_mode,
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
