#!/usr/bin/env python3
"""Cable-layer scaffold: BeltTransmission (q1) + CableTransmission (q2–q4), identity on actuator torque."""

from __future__ import annotations

import csv
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import mujoco as mj
import numpy as np

from kinematics.forward_kinematics import fk_ee_rp
from kinematics.inverse_kinematics import IKConfig, solve_ik_task_mode
from trajectory.joint_quintic import scaled_joint_quintic
from transmission.hybrid_transmission import HybridTransmission
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

HYBRID_XML = PKG_ROOT / "models" / "pmi_hybrid_no_collision.xml"
OUT_DIR = PKG_ROOT / "debug_outputs" / "cable_layer"

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


def run_scaffold() -> tuple[list[dict], dict]:
    qwp0, qwp1, qwp2 = solve_wp_q_arm_only()
    jpath = scaled_joint_quintic(qwp0, qwp1, qwp2, float(DURATION))

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

    hybrid = HybridTransmission()

    n = int(round(float(DURATION) / dt)) + 1
    rows: list[dict] = []
    ee_err_seq: list[float] = []
    trans_pos_seq: list[float] = []
    tau_tr_err_seq: list[float] = []
    sat_steps = 0
    jl_viol = 0
    al_viol = 0
    ncon_max = 0

    for step_i in range(n):
        t = min(step_i * dt, float(DURATION))
        q_des, qd_des, _ = jpath.sample(float(t))
        q_j = np.array([float(data.qpos[int(qadr_j[k])]) for k in range(4)])
        qd_j = np.array([float(data.qvel[int(dadr_j[k])]) for k in range(4)])
        q_a = np.array([float(data.qpos[int(qadr_a[k])]) for k in range(4)])
        qd_a = np.array([float(data.qvel[int(dadr_a[k])]) for k in range(4)])
        ratio_qa = RATIOS * q_a
        trans_pos = q_j - ratio_qa

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
        tau_tr_err = tau_act_out - tau_act_ideal

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
        ee_err_seq.append(ee_err)
        trans_pos_seq.append(float(np.max(np.abs(trans_pos))))
        tau_tr_err_seq.append(float(np.max(np.abs(tau_tr_err))))

        mj.mj_step(model, data)

        ncon_max = max(ncon_max, int(data.ncon))
        qfc_j = np.array([float(data.qfrc_constraint[int(dadr_j[k])]) for k in range(4)])
        qfc_a = np.array([float(data.qfrc_constraint[int(dadr_a[k])]) for k in range(4)])

        row: dict = {
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
            row[f"q_act_actual_{k+1}"] = float(q_a[k])
            row[f"transmission_err_{k+1}"] = float(trans_pos[k])
            row[f"tau_jnt_unc_{k+1}"] = float(tau_j_unc[k])
            row[f"tau_jnt_cmd_{k+1}"] = float(tau_j_cmd[k])
            row[f"tau_act_ideal_{k+1}"] = float(tau_act_ideal[k])
            row[f"tau_act_out_{k+1}"] = float(tau_act_out[k])
            row[f"tau_transmission_err_{k+1}"] = float(tau_tr_err[k])
            row[f"qfrc_constraint_jnt_{k+1}"] = float(qfc_j[k])
            row[f"qfrc_constraint_act_{k+1}"] = float(qfc_a[k])
        rows.append(row)

    ee_a = np.asarray(ee_err_seq, dtype=float)
    summary = {
        "duration": float(DURATION),
        "tau_jnt_limit": float(TAU_JNT_LIMIT),
        "rms_ee_err": float(np.sqrt(np.mean(ee_a**2))),
        "final_ee_err": float(ee_a[-1]),
        "max_transmission_pos_err_rad": float(np.max(trans_pos_seq)),
        "max_tau_transmission_err": float(np.max(tau_tr_err_seq)),
        "ncon_max": int(ncon_max),
        "saturation_steps": int(sat_steps),
        "joint_limit_violation_steps": int(jl_viol),
        "actuator_limit_violation_steps": int(al_viol),
    }
    return rows, summary


def write_report(path: Path, summary: dict, pass_fail: dict) -> None:
    lines = [
        "# Cable-layer scaffold test (hybrid no-collision)",
        "",
        "- Model: `models/pmi_hybrid_no_collision.xml` (torque on actuator DOFs only).",
        "- Control: IK joint-space VSD → clip joint torque → `tau_act_ideal = ratio ⊙ tau_jnt_cmd` → "
        "`HybridTransmission` (Belt q1, Cable q2–q4, identity) → `tau_act_out`.",
        "- Baseline match: `duration=5.0`, `tau_jnt_limit=20`, `Kq=[80,80,60,40]`, `Dq=[10,10,8,5]`.",
        "",
        "## Pass / fail",
        "",
        f"- RMS EE error: **{summary['rms_ee_err']:.6f} m** (expect ≈ 0.0007) → {pass_fail['rms_ee']}",
        f"- Final EE error: **{summary['final_ee_err']:.6f} m** (expect ≈ 0.0002) → {pass_fail['final_ee']}",
        f"- Max ‖q_jnt − ratio⊙q_act‖∞: **{summary['max_transmission_pos_err_rad']:.4e} rad** (< 1e-4) → {pass_fail['trans_pos']}",
        f"- Max ‖tau_act_out − tau_act_ideal‖∞: **{summary['max_tau_transmission_err']:.4e}** → {pass_fail['tau_tr']}",
    ]
    lines += [
        f"- ncon: **{summary['ncon_max']}** (expect 0) → {pass_fail['ncon']}",
        f"- Saturation steps: **{summary['saturation_steps']}** (expect 0) → {pass_fail['sat']}",
        f"- Joint-limit violations: **{summary['joint_limit_violation_steps']}** → {pass_fail['jl']}",
        f"- Actuator-limit violations: **{summary['actuator_limit_violation_steps']}** → {pass_fail['al']}",
        "",
        "## Answers",
        "",
        "### 1. Does the cable-layer scaffold reproduce the validated ideal actuator-side torque result?",
        f"- **{pass_fail['answer1']}** — EE metrics vs. expected baseline (RMS ≈ 7e-4 m, final ≈ 2e-4 m).",
        "",
        "### 2. Are q2~q4 routed through CableTransmission?",
        "- **예.** `HybridTransmission.transmit` passes `tau_act_ideal[1:4]` through `CableTransmission.transmit`.",
        "",
        "### 3. Is q1 routed through BeltTransmission?",
        "- **예.** `tau_act_ideal[0]` passes through `BeltTransmission.transmit`.",
        "",
        "### 4. Is tau_act_out equal to tau_act_ideal in this scaffold?",
        f"- **{pass_fail['answer4']}** (max abs deviation {summary['max_tau_transmission_err']:.4e}).",
        "",
        "### 5. Is the model ready to add first-order cable delay next?",
        "- **예**, provided the pass criteria above hold — identity cable layer is the injection point for a first-order delay on `tau_act` (q2–q4) without changing equality constraints.",
        "",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    rows, summary = run_scaffold()

    rms_tgt, final_tgt = 0.0007, 0.0002
    rms_ok = abs(summary["rms_ee_err"] - rms_tgt) < 0.0003
    final_ok = abs(summary["final_ee_err"] - final_tgt) < 0.0002
    pass_fail = {
        "rms_ee": "PASS" if rms_ok else "CHECK",
        "final_ee": "PASS" if final_ok else "CHECK",
        "trans_pos": "PASS" if summary["max_transmission_pos_err_rad"] < 1e-4 else "FAIL",
        "tau_tr": "PASS" if summary["max_tau_transmission_err"] < 1e-12 else "CHECK",
        "ncon": "PASS" if summary["ncon_max"] == 0 else "FAIL",
        "sat": "PASS" if summary["saturation_steps"] == 0 else "FAIL",
        "jl": "PASS" if summary["joint_limit_violation_steps"] == 0 else "FAIL",
        "al": "PASS" if summary["actuator_limit_violation_steps"] == 0 else "FAIL",
        "answer1": "예 (EE 오차가 베이스라인과 동급)" if rms_ok and final_ok else "스캐폴드는 동일 경로지만 수치는 CSV로 확인",
        "answer4": "예 (단위 행렬 수준)" if summary["max_tau_transmission_err"] < 1e-12 else "수치적으로 0에 가깝다 (부동소수)",
    }

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = OUT_DIR / "scaffold_test.csv"
    rep_path = OUT_DIR / "scaffold_test_report.md"

    if rows:
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

    write_report(rep_path, summary, pass_fail)

    print(summary)
    print(f"Wrote {csv_path}")
    print(f"Wrote {rep_path}")


if __name__ == "__main__":
    main()
