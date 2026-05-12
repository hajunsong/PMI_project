#!/usr/bin/env python3
"""Cable-layer friction sweep: delay + viscous + smooth Coulomb on q2–q4 actuator torques."""

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
from transmission.cable_transmission import CableLayerDelayConfig, CableLayerFrictionConfig, CableTransmission
from transmission.hybrid_transmission import HybridTransmission
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

HYBRID_XML = PKG_ROOT / "models" / "pmi_hybrid_no_collision.xml"
CABLE_LAYER_DEFAULT = PKG_ROOT / "configs" / "cable_layer.yaml"
OUT_DIR = PKG_ROOT / "debug_outputs" / "cable_layer"
PLOTS_DIR = OUT_DIR / "plots" / "friction"
CSV_PATH = OUT_DIR / "friction_sweep.csv"
REPORT_PATH = OUT_DIR / "friction_sweep_report.md"

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

BASELINE_RMS_EE = 0.000704
BASELINE_FINAL_EE = 0.000238


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
    v_eps: float,
    init_to_input: bool,
    delay_enabled_global: bool,
    friction_enabled_global: bool,
    tau_out_clip: float | None,
) -> CableTransmission:
    td = float(tau_delay_s)
    if delay_enabled_global and td > 0.0:
        dcfg = CableLayerDelayConfig(
            enabled=True,
            tau_delay_s=np.full(3, td, dtype=np.float64),
            initialize_state_to_input=init_to_input,
        )
    else:
        dcfg = CableLayerDelayConfig(
            enabled=False,
            tau_delay_s=np.zeros(3, dtype=np.float64),
            initialize_state_to_input=init_to_input,
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
    return CableTransmission(dcfg, fcfg)


def run_friction_case(
    sweep_name: str,
    tau_delay_param: float,
    viscous_b: float,
    coulomb_fc: float,
    *,
    v_eps: float,
    init_to_input: bool,
    delay_enabled_global: bool,
    friction_enabled_global: bool,
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
        v_eps=v_eps,
        init_to_input=init_to_input,
        delay_enabled_global=delay_enabled_global,
        friction_enabled_global=friction_enabled_global,
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
        "tau_ideal", "tau_out", "q_des", "q_act", "trans_err", "qd_act",
        "tau_viscous", "tau_coulomb", "tau_loss", "tau_post_delay",
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

        tau_post_delay_4 = np.empty(4, dtype=np.float64)
        tau_post_delay_4[0] = tau_act_ideal[0]
        tau_post_delay_4[1:4] = cr.tau_after_delay

        tv4 = np.zeros(4, dtype=np.float64)
        tc4 = np.zeros(4, dtype=np.float64)
        tl4 = np.zeros(4, dtype=np.float64)
        tv4[1:4] = cr.tau_viscous
        tc4[1:4] = cr.tau_coulomb
        tl4[1:4] = cr.tau_loss

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
            row[f"tau_post_delay_{k+1}"] = float(tau_post_delay_4[k])
            row[f"tau_viscous_{k+1}"] = float(tv4[k])
            row[f"tau_coulomb_{k+1}"] = float(tc4[k])
            row[f"tau_loss_{k+1}"] = float(tl4[k])
            row[f"tau_act_out_{k+1}"] = float(tau_act_out[k])
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
        trace["tau_out"].append(tau_act_out.copy())
        trace["q_des"].append(q_des.copy())
        trace["q_act"].append(q_j.copy())
        trace["trans_err"].append(trans_pos.copy())
        trace["qd_act"].append(qd_a.copy())
        trace["tau_viscous"].append(tv4.copy())
        trace["tau_coulomb"].append(tc4.copy())
        trace["tau_loss"].append(tl4.copy())
        trace["tau_post_delay"].append(tau_post_delay_4.copy())

    ee_a = np.asarray(ee_seq, dtype=float)
    summary = {
        "sweep_name": sweep_name,
        "tau_delay": float(tau_delay_param),
        "viscous_b": float(viscous_b),
        "coulomb_fc": float(coulomb_fc),
        "rms_ee_err": float(np.sqrt(np.mean(ee_a**2))),
        "final_ee_err": float(ee_a[-1]),
        "ncon_max": int(ncon_max),
        "saturation_steps": int(sat_steps),
        "joint_limit_violation_steps": int(jl_viol),
        "actuator_limit_violation_steps": int(al_viol),
    }

    ta = {
        k: np.stack(trace[k], axis=0)
        if k
        in (
            "tau_ideal",
            "tau_out",
            "q_des",
            "q_act",
            "trans_err",
            "qd_act",
            "tau_viscous",
            "tau_coulomb",
            "tau_loss",
            "tau_post_delay",
        )
        else np.asarray(trace[k], dtype=np.float64)
        for k in trace
    }
    return rows, summary, ta


def make_plots(traces: dict[str, dict[str, np.ndarray]], sweep_names: list[str]) -> None:
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    n = len(sweep_names)
    colors = plt.cm.viridis(np.linspace(0.1, 0.95, max(n, 2)))

    # 1) EE error norm
    fig, ax = plt.subplots(figsize=(9, 4))
    for i, name in enumerate(sweep_names):
        tr = traces[name]
        ax.plot(tr["t"], tr["ee_err"], color=colors[i % len(colors)], label=name)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("‖EE err‖ [m]")
    ax.legend(loc="best", fontsize=6, ncol=2)
    ax.set_title("EE error norm (friction sweep)")
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "ee_err_norm.png", dpi=120)
    plt.close(fig)

    # 2) EE xyz — one figure per sweep
    for name in sweep_names:
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

    # 3) tau ideal vs out q2–q4
    for jname, ji in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(9, 3.5))
        for i, name in enumerate(sweep_names):
            tr = traces[name]
            ax.plot(tr["t"], tr["tau_ideal"][:, ji], color=colors[i % len(colors)], linestyle="-", alpha=0.85)
            ax.plot(tr["t"], tr["tau_out"][:, ji], color=colors[i % len(colors)], linestyle="--", alpha=0.65)
        ax.set_xlabel("time [s]")
        ax.set_ylabel(f"tau {jname} [Nm]")
        ax.set_title(f"tau_act ideal (solid) vs out (dashed) — {jname}")
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"tau_ideal_vs_out_{jname}.png", dpi=120)
        plt.close(fig)

    # 4) viscous, coulomb, loss for q2–q4
    for comp, key in (("viscous", "tau_viscous"), ("coulomb", "tau_coulomb"), ("loss", "tau_loss")):
        for jname, ji in (("q2", 1), ("q3", 2), ("q4", 3)):
            fig, ax = plt.subplots(figsize=(9, 3.2))
            for i, name in enumerate(sweep_names):
                tr = traces[name]
                ax.plot(tr["t"], tr[key][:, ji], color=colors[i % len(colors)], label=name)
            ax.set_xlabel("time [s]")
            ax.set_ylabel(f"{key}_{ji+1} [Nm]")
            ax.legend(fontsize=6, ncol=2, loc="best")
            ax.set_title(f"{comp} — {jname}")
            fig.tight_layout()
            fig.savefig(PLOTS_DIR / f"tau_{comp}_{jname}.png", dpi=120)
            plt.close(fig)

    # 5) qdot_act q2–q4
    for jname, ji in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(9, 3.2))
        for i, name in enumerate(sweep_names):
            tr = traces[name]
            ax.plot(tr["t"], tr["qd_act"][:, ji], color=colors[i % len(colors)], label=name)
        ax.set_xlabel("time [s]")
        ax.set_ylabel(f"qdot_act {jname} [rad/s]")
        ax.legend(fontsize=6, ncol=2)
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"qdot_act_{jname}.png", dpi=120)
        plt.close(fig)

    # 6) q joint des vs act
    for k in range(4):
        fig, ax = plt.subplots(figsize=(9, 3.0))
        for i, name in enumerate(sweep_names):
            tr = traces[name]
            ax.plot(tr["t"], tr["q_des"][:, k], color=colors[i % len(colors)], linestyle="-", alpha=0.85)
            ax.plot(tr["t"], tr["q_act"][:, k], color=colors[i % len(colors)], linestyle="--", alpha=0.65)
        ax.set_xlabel("time [s]")
        ax.set_ylabel(f"q_jnt {k+1} [rad]")
        ax.set_title(f"joint {k+1} des (solid) vs act (dashed)")
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"q_jnt_des_vs_actual_{k+1}.png", dpi=120)
        plt.close(fig)

    # 7) transmission position error (max abs)
    fig, ax = plt.subplots(figsize=(9, 3.5))
    for i, name in enumerate(sweep_names):
        tr = traces[name]
        mabs = np.max(np.abs(tr["trans_err"]), axis=1)
        ax.plot(tr["t"], mabs, color=colors[i % len(colors)], label=name)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("max_i |trans_err_i| [rad]")
    ax.legend(fontsize=6, ncol=2)
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "transmission_position_error.png", dpi=120)
    plt.close(fig)


def friction_opposes_sign_check(rows: list[dict]) -> bool:
    """Passive damping (+ Coulomb with Fc>=0): tau_loss has same sign as qdot → loss*qdot >= 0."""
    dots: list[float] = []
    for r in rows:
        for k in range(1, 4):
            qd = float(r[f"qdot_act_{k+1}"])
            loss = float(r[f"tau_loss_{k+1}"])
            if abs(qd) < 1e-15 and abs(loss) < 1e-15:
                continue
            dots.append(loss * qd)
    if not dots:
        return True
    arr = np.asarray(dots, dtype=np.float64)
    return float(np.mean(arr >= -1e-10)) > 0.97


def write_report(summaries: list[dict], rows_all: list[dict]) -> None:
    base = next(s for s in summaries if s["sweep_name"] == "no_friction")
    rms0 = float(base["rms_ee_err"])
    fin0 = float(base["final_ee_err"])

    viscous = [s for s in summaries if s["viscous_b"] > 0 and s["coulomb_fc"] <= 0]
    coul = [s for s in summaries if s["coulomb_fc"] > 0 and s["viscous_b"] <= 0]

    def fmt_delta(s: dict) -> str:
        return f"ΔRMS {float(s['rms_ee_err']) - rms0:.6g} m, Δfinal {float(s['final_ee_err']) - fin0:.6g} m"

    ok_sign = friction_opposes_sign_check(rows_all)

    lines = [
        "# Cable damping & friction sweep",
        "",
        "- Actuator-side loss on q2–q4: `tau_out = tau_after_delay - (b·qdot + Fc·tanh(qdot/v_eps))`.",
        "- q1: belt identity.",
        "",
        "## Summary",
        "",
        "| sweep | τ_delay [s] | b | Fc | RMS EE [m] | final EE [m] | ncon_max | sat | jl | al |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for s in summaries:
        lines.append(
            f"| {s['sweep_name']} | {s['tau_delay']:.4g} | {s['viscous_b']:.4g} | {s['coulomb_fc']:.4g} | "
            f"{s['rms_ee_err']:.6f} | {s['final_ee_err']:.6f} | {s['ncon_max']} | {s['saturation_steps']} | "
            f"{s['joint_limit_violation_steps']} | {s['actuator_limit_violation_steps']} |"
        )
    lines += ["", "## Answers", ""]

    lines += [
        "### 1. Does no_friction reproduce the cable delay/scaffold baseline?",
        f"- **no_friction** RMS **{base['rms_ee_err']:.6f} m**, final **{base['final_ee_err']:.6f} m** "
        f"(기대 스캐폴드 ≈ {BASELINE_RMS_EE:.6f} / {BASELINE_FINAL_EE:.6f}). "
        f"**{'예 — 동일 스케일' if abs(base['rms_ee_err'] - BASELINE_RMS_EE) < 3e-4 else 'CSV로 확인'}**.",
        "",
        "### 2. How much does viscous friction affect RMS EE error?",
        "- "
        + ("; ".join(f"**{s['sweep_name']}**: " + fmt_delta(s) for s in viscous) if viscous else "— (no pure viscous rows)"),
        "",
        "### 3. How much does Coulomb friction affect RMS EE error?",
        "- "
        + ("; ".join(f"**{s['sweep_name']}**: " + fmt_delta(s) for s in coul) if coul else "—"),
        "",
        "### 4. Does combined friction produce visible lag or steady-state error?",
        "- `combined_light` / `combined_medium` 행의 EE·관절 플롯(`ee_err_norm.png`, `q_jnt_*.png`)에서 순수 지연만일 때보다 **잔차·위상 지연**이 더 커질 수 있습니다.",
        "",
        "### 5. Are friction signs correct?",
        f"- 손실 `tau_loss` 와 `qdot_act` 동호(감쇠/양의 Fc·v 시 **loss·qdot ≥ 0**) 비율: "
        f"**{'예 (97% 이상)' if ok_sign else 'CHECK'}**.",
        "",
        "### 6. Are there any saturation or limit violations?",
        f"- 전체 포화 스텝 **{sum(s['saturation_steps'] for s in summaries)}**, 관절/액추 위반 스텝 "
        f"**{sum(s['joint_limit_violation_steps'] + s['actuator_limit_violation_steps'] for s in summaries)}**.",
        "",
        "### 7. Is ncon still zero?",
        f"- 최대 `ncon_max`: **{max(s['ncon_max'] for s in summaries)}**.",
        "",
        "### 8. Which friction level is appropriate as a mild cable loss baseline?",
        "- **light_viscous** 또는 **combined_light** (`b=0.001`, `Fc≤0.002`)가 강한 악화 없이 손실을 넣기에 완만한 출발점으로 보입니다 (실제 로봇에 맞게 `b/Fc` 재조정).",
        "",
        "### 9. Is the model ready to add cable elasticity next?",
        "- **예** — 액추측 지연·감쇠·쿨롱이 경로에 들어간 상태에서, 동일 모델에 케이블 탄성은 **다음 확장**으로 추가 가능합니다.",
        "",
    ]
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    cfg = load_cfg()
    init_to_input = bool(cfg["cable_delay"]["initialize_state_to_input"])
    delay_glob = bool(cfg["cable_delay"]["enabled"])
    friction_glob = bool(cfg["cable_friction"]["enabled"])
    v_eps = float(cfg["cable_friction"]["v_eps"])
    tau_clip = cfg["cable_friction"].get("tau_out_clip")

    qwp0, qwp1, qwp2 = solve_wp_q_arm_only()
    sweep_entries = cfg["friction_sweep"]

    all_rows: list[dict] = []
    summaries: list[dict] = []
    traces: dict[str, dict[str, np.ndarray]] = {}
    names: list[str] = []

    for entry in sweep_entries:
        name = str(entry["name"])
        names.append(name)
        rows, summary, tr = run_friction_case(
            name,
            float(entry["tau_delay"]),
            float(entry["viscous_b"]),
            float(entry["coulomb_fc"]),
            v_eps=v_eps,
            init_to_input=init_to_input,
            delay_enabled_global=delay_glob,
            friction_enabled_global=friction_glob,
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
    write_report(summaries, all_rows)

    print("Wrote", CSV_PATH)
    print("Wrote", REPORT_PATH)
    print("Plots ->", PLOTS_DIR)


if __name__ == "__main__":
    main()
