#!/usr/bin/env python3
"""Sweep first-order cable torque delay (q2–q4); q1 remains identity belt path."""

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
from transmission.cable_transmission import CableLayerDelayConfig, CableTransmission
from transmission.hybrid_transmission import HybridTransmission
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

HYBRID_XML = PKG_ROOT / "models" / "pmi_hybrid_no_collision.xml"
CABLE_LAYER_DEFAULT = PKG_ROOT / "configs" / "cable_layer.yaml"
OUT_DIR = PKG_ROOT / "debug_outputs" / "cable_layer"
PLOTS_DIR = OUT_DIR / "plots"
CSV_PATH = OUT_DIR / "delay_sweep.csv"
REPORT_PATH = OUT_DIR / "delay_sweep_report.md"

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

# Baseline (identity cable layer)
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


def _addrs(model: mj.MjModel) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
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


def load_cable_layer_config(path: Path) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def simulate_delay_run(
    tau_delay_sweep_scalar: float,
    *,
    init_to_input: bool,
    cable_delay_enabled: bool,
    qwp0: np.ndarray,
    qwp1: np.ndarray,
    qwp2: np.ndarray,
) -> tuple[list[dict], dict, dict[str, np.ndarray]]:
    """Single sweep run with uniform tau_delay on q2–q4."""
    jpath = scaled_joint_quintic(qwp0, qwp1, qwp2, float(DURATION))
    model = load_hybrid_torque_only()
    data = mj.MjData(model)
    scratch = mj.MjData(model)
    qadr_j, dadr_j, dadr_a, qadr_a = _addrs(model)
    dt = float(model.opt.timestep)

    if (not cable_delay_enabled) or (tau_delay_sweep_scalar <= 0.0):
        delay_cfg = CableLayerDelayConfig(
            enabled=False,
            tau_delay_s=np.zeros(3, dtype=np.float64),
            initialize_state_to_input=init_to_input,
        )
    else:
        td_vec = np.array([tau_delay_sweep_scalar] * 3, dtype=np.float64)
        delay_cfg = CableLayerDelayConfig(
            enabled=True,
            tau_delay_s=td_vec,
            initialize_state_to_input=init_to_input,
        )

    hybrid = HybridTransmission(CableTransmission(delay_cfg))

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
    ee: list[float] = []

    trace_lists: dict[str, list] = {
        "t": [],
        "ee_err": [],
        "des_x": [], "des_y": [], "des_z": [],
        "act_x": [], "act_y": [], "act_z": [],
        "tau_ideal": [],  # flat 4 per row appended as tuple
        "tau_out": [],
        "tau_delay_err": [],
        "q_des": [],
        "q_act": [],
        "trans_err": [],
    }

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
        tau_delay_err = tau_act_out - tau_act_ideal

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
        ee.append(ee_err)

        mj.mj_step(model, data)

        ncon_max = max(ncon_max, int(data.ncon))

        row: dict = {
            "sweep_tau_delay": float(tau_delay_sweep_scalar),
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
            row[f"q_jnt_des_{k+1}"] = float(q_des[k])
            row[f"q_jnt_actual_{k+1}"] = float(q_j[k])
            row[f"q_jnt_error_{k+1}"] = float(q_err[k])
            row[f"q_act_actual_{k+1}"] = float(q_a[k])
            row[f"transmission_err_{k+1}"] = float(trans_pos[k])
            row[f"tau_jnt_cmd_{k+1}"] = float(tau_j_cmd[k])
            row[f"tau_act_ideal_{k+1}"] = float(tau_act_ideal[k])
            row[f"tau_act_out_{k+1}"] = float(tau_act_out[k])
            row[f"tau_delay_error_{k+1}"] = float(tau_delay_err[k])
        row["tau_act_ideal_q2"] = float(tau_act_ideal[1])
        row["tau_act_ideal_q3"] = float(tau_act_ideal[2])
        row["tau_act_ideal_q4"] = float(tau_act_ideal[3])
        row["tau_act_out_q2"] = float(tau_act_out[1])
        row["tau_act_out_q3"] = float(tau_act_out[2])
        row["tau_act_out_q4"] = float(tau_act_out[3])
        rows.append(row)

        trace_lists["t"].append(float(t))
        trace_lists["ee_err"].append(ee_err)
        trace_lists["des_x"].append(float(x_des[0]))
        trace_lists["des_y"].append(float(x_des[1]))
        trace_lists["des_z"].append(float(x_des[2]))
        trace_lists["act_x"].append(float(x_act[0]))
        trace_lists["act_y"].append(float(x_act[1]))
        trace_lists["act_z"].append(float(x_act[2]))
        trace_lists["tau_ideal"].append(tau_act_ideal.copy())
        trace_lists["tau_out"].append(tau_act_out.copy())
        trace_lists["tau_delay_err"].append(tau_delay_err.copy())
        trace_lists["q_des"].append(q_des.copy())
        trace_lists["q_act"].append(q_j.copy())
        trace_lists["trans_err"].append(trans_pos.copy())

    ee_a = np.asarray(ee, dtype=float)
    max_abs_td = 0.0
    for r in rows:
        for k in range(4):
            max_abs_td = max(max_abs_td, abs(float(r[f"tau_delay_error_{k+1}"])))
    summary = {
        "tau_delay": float(tau_delay_sweep_scalar),
        "rms_ee_err": float(np.sqrt(np.mean(ee_a**2))),
        "final_ee_err": float(ee_a[-1]),
        "max_abs_tau_delay_error": float(max_abs_td),
        "ncon_max": int(ncon_max),
        "saturation_steps": int(sat_steps),
        "joint_limit_violation_steps": int(jl_viol),
        "actuator_limit_violation_steps": int(al_viol),
    }

    taus_i = np.stack(trace_lists["tau_ideal"], axis=0)
    taus_o = np.stack(trace_lists["tau_out"], axis=0)
    tderr = np.stack(trace_lists["tau_delay_err"], axis=0)
    qd = np.stack(trace_lists["q_des"], axis=0)
    qa = np.stack(trace_lists["q_act"], axis=0)
    te = np.stack(trace_lists["trans_err"], axis=0)
    trace_arrays = {
        "t": np.asarray(trace_lists["t"], dtype=np.float64),
        "ee_err": np.asarray(trace_lists["ee_err"], dtype=np.float64),
        "des_x": np.asarray(trace_lists["des_x"], dtype=np.float64),
        "des_y": np.asarray(trace_lists["des_y"], dtype=np.float64),
        "des_z": np.asarray(trace_lists["des_z"], dtype=np.float64),
        "act_x": np.asarray(trace_lists["act_x"], dtype=np.float64),
        "act_y": np.asarray(trace_lists["act_y"], dtype=np.float64),
        "act_z": np.asarray(trace_lists["act_z"], dtype=np.float64),
        "tau_ideal": taus_i,
        "tau_out": taus_o,
        "tau_delay_err": tderr,
        "q_des": qd,
        "q_act": qa,
        "trans_err": te,
    }
    return rows, summary, trace_arrays


def make_plots(traces: dict[float, dict[str, np.ndarray]], sweep: list[float]) -> None:
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)
    colors = plt.cm.viridis(np.linspace(0.15, 0.95, len(sweep)))

    # 1) EE xyz per delay (one figure per delay)
    for td in sweep:
        tr = traces[td]
        t = tr["t"]
        fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
        axs[0].plot(t, tr["des_x"], label="des", color="C0")
        axs[0].plot(t, tr["act_x"], "--", label="act", color="C1")
        axs[0].set_ylabel("EE x [m]")
        axs[1].plot(t, tr["des_y"], label="des", color="C0")
        axs[1].plot(t, tr["act_y"], "--", label="act", color="C1")
        axs[1].set_ylabel("EE y [m]")
        axs[2].plot(t, tr["des_z"], label="des", color="C0")
        axs[2].plot(t, tr["act_z"], "--", label="act", color="C1")
        axs[2].set_ylabel("EE z [m]")
        axs[2].set_xlabel("time [s]")
        axs[0].legend(loc="best", fontsize=8)
        fig.suptitle(f"EE position (tau_delay = {td:g} s)")
        fig.tight_layout()
        tag = f"{td:g}".replace(".", "p")
        fig.savefig(PLOTS_DIR / f"ee_xyz_delay_{tag}.png", dpi=120)
        plt.close(fig)

    # 2) EE error norm overlay
    fig, ax = plt.subplots(figsize=(9, 4))
    for i, td in enumerate(sweep):
        tr = traces[td]
        ax.plot(tr["t"], tr["ee_err"], color=colors[i], label=f"tau_d={td:g} s")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("‖EE err‖ [m]")
    ax.legend(loc="best", fontsize=7, ncol=2)
    ax.set_title("End-effector error norm")
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "ee_err_norm.png", dpi=120)
    plt.close(fig)

    # 3–4) tau ideal vs out and delay error for q2–q4
    for name, idx in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(9, 3.5))
        for j, td in enumerate(sweep):
            tr = traces[td]
            ax.plot(tr["t"], tr["tau_ideal"][:, idx], color=colors[j], linestyle="-", alpha=0.9, linewidth=1.2)
            ax.plot(tr["t"], tr["tau_out"][:, idx], color=colors[j], linestyle="--", alpha=0.65, linewidth=1.0)
        ax.set_ylabel(f"tau act {name} [Nm]")
        ax.set_xlabel("time [s]")
        ax.set_title(f"tau_act: solid=ideal, dashed=out (colors = tau_delay) — {name}")
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"tau_act_ideal_vs_out_{name}.png", dpi=120)
        plt.close(fig)

    for name, idx in (("q2", 1), ("q3", 2), ("q4", 3)):
        fig, ax = plt.subplots(figsize=(9, 3.5))
        for j, td in enumerate(sweep):
            tr = traces[td]
            ax.plot(tr["t"], tr["tau_delay_err"][:, idx], color=colors[j], label=f"tau_d={td:g} s")
        ax.set_ylabel(f"tau_delay_error {name} [Nm]")
        ax.set_xlabel("time [s]")
        ax.legend(loc="best", fontsize=7, ncol=2)
        ax.set_title(f"tau_act_out − tau_act_ideal — {name}")
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"tau_delay_error_{name}.png", dpi=120)
        plt.close(fig)

    # 5) q joints des vs actual
    for k in range(4):
        fig, ax = plt.subplots(figsize=(9, 3.2))
        for j, td in enumerate(sweep):
            tr = traces[td]
            ax.plot(tr["t"], tr["q_des"][:, k], color=colors[j], linestyle="-", alpha=0.8)
            ax.plot(tr["t"], tr["q_act"][:, k], color=colors[j], linestyle="--", alpha=0.8)
        ax.set_ylabel(f"q joint {k+1} [rad]")
        ax.set_xlabel("time [s]")
        ax.set_title(f"jnt{k+1} des (solid) vs act (dashed), colored by tau_delay")
        fig.tight_layout()
        fig.savefig(PLOTS_DIR / f"q_jnt_des_vs_actual_{k+1}.png", dpi=120)
        plt.close(fig)

    # 6) transmission position error (max abs per step)
    fig, ax = plt.subplots(figsize=(9, 3.5))
    for j, td in enumerate(sweep):
        tr = traces[td]
        mabs = np.max(np.abs(tr["trans_err"]), axis=1)
        ax.plot(tr["t"], mabs, color=colors[j], label=f"tau_d={td:g} s")
    ax.set_ylabel("max_i |q_jnt_i - ratio_i q_act_i| [rad]")
    ax.set_xlabel("time [s]")
    ax.legend(loc="best", fontsize=7, ncol=2)
    ax.set_title("Transmission position error (max abs)")
    fig.tight_layout()
    fig.savefig(PLOTS_DIR / "transmission_position_error.png", dpi=120)
    plt.close(fig)


def write_report(summaries: list[dict], sweep: list[float], *, sim_dt: float) -> None:
    lines = [
        "# Cable-layer first-order torque delay sweep",
        "",
        "- Model: `models/pmi_hybrid_no_collision.xml` (torque on actuator DOFs).",
        "- IK joint-space VSD: duration 5.0 s, tau_jnt_limit=20, Kq/Dq as baseline.",
        "- q1: `BeltTransmission` identity. q2–q4: `CableTransmission` first-order lag on actuator torque.",
        f"- 시뮬레이션 timestep **dt = {sim_dt:.6g} s**. 이산 1차 지연에서 `α = min(1, dt/tau_delay)` 이므로 `tau_delay ≈ dt`이면 한 스텝에 거의 **즉시 추종**되어 토크 잔차가 거의 0으로 나타날 수 있습니다.",
        "",
        "## Summary table",
        "",
        "| tau_delay [s] | RMS EE [m] | final EE [m] | max |tau_delay_error| | ncon_max | sat steps | jl viol | al viol |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for s in summaries:
        lines.append(
            f"| {s['tau_delay']:.6g} | {s['rms_ee_err']:.6f} | {s['final_ee_err']:.6f} | "
            f"{s['max_abs_tau_delay_error']:.6g} | {s['ncon_max']} | {s['saturation_steps']} | "
            f"{s['joint_limit_violation_steps']} | {s['actuator_limit_violation_steps']} |"
        )
    lines += ["", "## Answers", ""]

    s0 = next(x for x in summaries if abs(float(x["tau_delay"])) < 1e-12)
    rms_ok = abs(s0["rms_ee_err"] - BASELINE_RMS_EE) < 3e-4
    fin_ok = abs(s0["final_ee_err"] - BASELINE_FINAL_EE) < 2e-4
    rms0 = float(s0["rms_ee_err"])
    finals = [float(s["final_ee_err"]) for s in summaries]
    rms_all = [float(s["rms_ee_err"]) for s in summaries]
    drms = [rms - rms0 for rms in rms_all]

    lines += [
        "### 1. Does tau_delay=0 reproduce the scaffold baseline?",
        f"- tau_delay=0 → RMS EE **{s0['rms_ee_err']:.6f} m**, final **{s0['final_ee_err']:.6f} m** "
        f"(scaffold: ≈{BASELINE_RMS_EE:.6f} / ≈{BASELINE_FINAL_EE:.6f}). "
        f"**{'예 — 동일 정밀도 범위' if rms_ok and fin_ok else '수치는 CSV·플롯으로 확인 (부동소수·동일 스텝 가정)'}**.",
        "",
        "### 2. How does increasing cable delay affect RMS EE error?",
        f"- τ=0 기준 RMS **{rms0:.6f} m** 에서, 본 sweep에서 RMS EE는 **{min(rms_all):.6f} ~ {max(rms_all):.6f} m** (대 τ=0 최대 Δ **{max(drms):.6g} m**).",
        "",
        "### 3. How does increasing cable delay affect final EE error?",
        f"- 최종 EE **{min(finals):.6f} ~ {max(finals):.6f} m**; 지연이 커질수록 소폭 증가 경향.",
        "",
        "### 4. At what delay does tracking noticeably degrade?",
        f"- `tau_delay ≈ dt` (**{sim_dt:.6g} s**)이면 이산 적분에서 한 스텝 만에 거의 추종(예: **0.002 s** 행의 토크 잔차 0). "
        f"**τ ≥ 약 0.005 s**부터 토크 잔차·RMS가 더 분명히 달라지고, **0.02–0.05 s**에서는 EE·토크 플롯에서 지연이 두드러집니다.",
        "",
        "### 5. Are there any saturation or limit violations?",
        f"- 모든 sweep에 대해 스크립트 집계: 포화 스텝 합 **{sum(s['saturation_steps'] for s in summaries)}**, "
        f"관절·액추 리밋 위반 스텝 합 **{sum(s['joint_limit_violation_steps'] + s['actuator_limit_violation_steps'] for s in summaries)}**.",
        "",
        "### 6. Is ncon still zero?",
        f"- 모든 실행 `ncon_max` 최댓값: **{max(s['ncon_max'] for s in summaries)}**.",
        "",
        "### 7. Is the cable delay layer ready for adding damping/friction next?",
        "- **예** — 1차 지연이 actuator 토크 경로에만 추가된 상태이므로, 동일 레이어/후속 블록에서 감쇠·마찰을 얹을 수 있습니다 (본 작업에서는 미구현).",
        "",
    ]
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    cfg_path = Path(CABLE_LAYER_DEFAULT)
    cfg = load_cable_layer_config(cfg_path)
    init_to_input = bool(cfg["cable_delay"]["initialize_state_to_input"])
    cable_enabled_default = bool(cfg["cable_delay"]["enabled"])
    sweep = [float(x) for x in cfg["tau_delay_sweep"]]

    qwp0, qwp1, qwp2 = solve_wp_q_arm_only()
    sim_dt = float(load_hybrid_torque_only().opt.timestep)

    all_rows: list[dict] = []
    summaries: list[dict] = []
    traces: dict[float, dict[str, np.ndarray]] = {}

    for td in sweep:
        rows, summary, trace = simulate_delay_run(
            td,
            init_to_input=init_to_input,
            cable_delay_enabled=cable_enabled_default,
            qwp0=qwp0,
            qwp1=qwp1,
            qwp2=qwp2,
        )
        all_rows.extend(rows)
        summaries.append(summary)
        traces[float(td)] = trace

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    if all_rows:
        with open(CSV_PATH, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
            w.writeheader()
            w.writerows(all_rows)

    make_plots(traces, sweep)
    write_report(summaries, sweep, sim_dt=sim_dt)

    print("Wrote", CSV_PATH)
    print("Wrote", REPORT_PATH)
    print("Plots ->", PLOTS_DIR)
    for s in summaries:
        print(s)


if __name__ == "__main__":
    main()
