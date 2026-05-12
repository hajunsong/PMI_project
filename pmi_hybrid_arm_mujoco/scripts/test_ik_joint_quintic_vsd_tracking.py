#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import itertools
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np

from kinematics.forward_kinematics import fk_ee_rp
from kinematics.inverse_kinematics import IKConfig, solve_ik_task_mode
from kinematics.task_jacobian import compute_task_jacobian_mode
from trajectory.joint_quintic import scaled_joint_quintic
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

MODEL = PKG_ROOT / "models" / "pmi_arm_only_torque_debug.xml"
OUT = PKG_ROOT / "debug_outputs" / "ik_joint_tracking"
J = ["jnt1", "jnt2", "jnt3", "jnt4"]
R = np.array([0.5333, 0.15, 0.3, 0.3], dtype=float)
Q0 = R * np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=float)
WPS = [np.array([0.25, -0.20, -0.10]), np.array([0.00, -0.35, -0.15]), np.array([-0.25, -0.20, -0.10])]
KQ = np.array([80.0, 80.0, 60.0, 40.0])
DQ = np.array([10.0, 10.0, 8.0, 5.0])
KX = np.array([60.0, 60.0, 60.0])
DX = np.array([8.0, 8.0, 8.0])


def solve_wp_q(model):
    scratch = mj.MjData(model)
    q_lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in J])
    q_hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in J])
    ik = IKConfig((1, 1, 1), 1.0, 1.0, 1e-3, 1e-4, 80, 1e-9, tuple(J))
    q_seed = Q0.copy()
    qs = []
    for wp in WPS:
        q, _ = solve_ik_task_mode(model, scratch, wp, roll_des=-np.pi/2, pitch_des=0.0, task_feas_mode="xyz", ik=ik, q_seed=q_seed, bounds_lo=q_lo, bounds_hi=q_hi)
        qs.append(q.copy())
        q_seed = q.copy()
    return qs


def run_once(duration: float, tau_lim: float, mode: str, lam_task: float, torque_sign: np.ndarray) -> dict:
    model = load_mjmodel(MODEL, strip_position_actuators=True)
    data = mj.MjData(model)
    scratch = mj.MjData(model)
    jac_s = mj.MjData(model)
    dt = float(model.opt.timestep)
    n = int(round(duration / dt)) + 1
    ts = np.array([min(i * dt, duration) for i in range(n)], dtype=float)

    q_lo = np.array([float(model.jnt_range[joint_id(model, n0), 0]) for n0 in J])
    q_hi = np.array([float(model.jnt_range[joint_id(model, n0), 1]) for n0 in J])
    dof = np.array([int(model.jnt_dofadr[joint_id(model, n0)]) for n0 in J], dtype=int)

    qwp0, qwp1, qwp2 = solve_wp_q(model)
    path = scaled_joint_quintic(qwp0, qwp1, qwp2, duration)

    for i, n0 in enumerate(J):
        data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])] = Q0[i]
    data.qvel[:] = 0.0
    mj.mj_forward(model, data)

    ee_err = []
    q_err = []
    sat_steps = 0
    jl_steps = 0
    max_tau = 0.0

    for t in ts:
        q = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])]) for n0 in J])
        qdot = np.array([float(data.qvel[int(model.jnt_dofadr[joint_id(model, n0)])]) for n0 in J])
        qd, qddot_des, qdd = path.sample(float(t))

        mj.mj_forward(model, data)
        tau_bias = np.array([float(data.qfrc_bias[d]) for d in dof])

        tau_pd = KQ * (qd - q) + DQ * (qddot_des - qdot)

        if mode == "computed_torque":
            Mfull = np.zeros((model.nv, model.nv), dtype=float)
            mj.mj_fullM(model, Mfull, data.qM)
            M = Mfull[np.ix_(dof, dof)]
            tau = tau_bias + M @ qdd + tau_pd
        else:
            tau = tau_bias + tau_pd

        p_des = np.array(WPS[0]) if t <= duration*0.5e-3 else None
        q_for_fk = qd
        p_des_fk, *_ = fk_ee_rp(model, scratch, q_for_fk, J)
        p_des_fk = np.asarray(p_des_fk)

        p_act, *_ = fk_ee_rp(model, scratch, q, J)
        p_act = np.asarray(p_act)

        if lam_task > 0.0:
            jac_s.qpos[:] = data.qpos
            jac_s.qvel[:] = data.qvel
            mj.mj_forward(model, jac_s)
            Jp = compute_task_jacobian_mode(model, jac_s, joint_names=J, task_mode="xyz", ee_site_name="end_effector", mode="numerical", epsilon=1e-6)
            xdot = Jp @ qdot
            xdot_des = Jp @ qddot_des
            F = KX * (p_des_fk - p_act) + DX * (xdot_des - xdot)
            tau = tau + lam_task * (Jp.T @ F)

        tau_applied = np.asarray(torque_sign, dtype=float).reshape(4) * tau
        tau_clip = np.clip(tau_applied, -tau_lim, tau_lim)
        sat_steps += int(np.any(np.abs(tau - tau_clip) > 1e-9))
        max_tau = max(max_tau, float(np.max(np.abs(tau_clip))))

        data.qfrc_applied[:] = 0.0
        for i, d in enumerate(dof):
            data.qfrc_applied[d] = tau_clip[i]
        mj.mj_step(model, data)

        qn = np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n0)])]) for n0 in J])
        p_now, *_ = fk_ee_rp(model, scratch, qn, J)
        p_now = np.asarray(p_now)
        p_des_now, *_ = fk_ee_rp(model, scratch, qd, J)
        p_des_now = np.asarray(p_des_now)

        ee_err.append(float(np.linalg.norm(p_des_now - p_now)))
        q_err.append(float(np.linalg.norm(qd - qn)))

        jm = np.minimum(q_hi - qn, qn - q_lo)
        jl_steps += int(np.min(jm) < 0.0)

    ee = np.asarray(ee_err)
    qe = np.asarray(q_err)
    return {
        "controller_mode": mode,
        "lambda_task": lam_task,
        "duration": duration,
        "tau_limit": tau_lim,
        "rms_ee_pos_err": float(np.sqrt(np.mean(ee**2))),
        "max_ee_pos_err": float(np.max(ee)),
        "final_ee_pos_err": float(ee[-1]),
        "rms_joint_err": float(np.sqrt(np.mean(qe**2))),
        "max_joint_err": float(np.max(qe)),
        "max_tau": float(max_tau),
        "saturation_steps": int(sat_steps),
        "joint_limit_steps": int(jl_steps),
        "stable": bool(np.isfinite(np.max(ee))),
    }


def plot_joint_quintic() -> None:
    model = load_mjmodel(MODEL, strip_position_actuators=True)
    q0, q1, q2 = solve_wp_q(model)
    OUT.mkdir(parents=True, exist_ok=True)
    for T in [1.0, 2.0, 3.0, 5.0]:
        path = scaled_joint_quintic(q0, q1, q2, T)
        ts = np.linspace(0.0, T, 500)
        Q = np.zeros((len(ts), 4)); Qd = np.zeros((len(ts), 4)); Qdd = np.zeros((len(ts), 4))
        for i, t in enumerate(ts):
            q, qd, qdd = path.sample(float(t))
            Q[i], Qd[i], Qdd[i] = q, qd, qdd
        fig, axs = plt.subplots(4, 3, figsize=(12, 8), sharex="col")
        for j in range(4):
            axs[j, 0].plot(ts, Q[:, j]); axs[j, 0].set_ylabel(f"j{j+1}")
            axs[j, 1].plot(ts, Qd[:, j])
            axs[j, 2].plot(ts, Qdd[:, j])
            for c in range(3): axs[j, c].grid(True, alpha=0.3)
        axs[0, 0].set_title("q_des"); axs[0, 1].set_title("qdot_des"); axs[0, 2].set_title("qddot_des")
        axs[-1, 0].set_xlabel("t"); axs[-1, 1].set_xlabel("t"); axs[-1, 2].set_xlabel("t")
        plt.tight_layout(); plt.savefig(OUT / f"joint_quintic_T{int(T)}.png", dpi=140); plt.close()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", type=Path, default=OUT)
    ap.add_argument("--torque-sign", type=str, default="1,1,1,1")
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    torque_sign = np.array([float(x.strip()) for x in str(args.torque_sign).split(",")], dtype=float)
    if torque_sign.shape[0] != 4:
        raise ValueError("torque-sign must have 4 comma-separated values")

    plot_joint_quintic()

    rows = []
    for duration, tau_lim, mode, lam in itertools.product([1.0, 2.0, 3.0, 5.0], [20.0, 50.0, 100.0], ["pd_bias", "computed_torque"], [0.0, 0.1, 0.3]):
        rows.append(run_once(duration, tau_lim, mode, lam, torque_sign))
        print(rows[-1])

    with open(out_dir / "joint_quintic_vsd_sweep.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)

    # reports
    best = min(rows, key=lambda r: r["final_ee_pos_err"])
    rep = ["# Joint Quintic VSD Sweep", "", f"best run: {best}"]
    rep.append("- success criterion: final<0.03, RMS<0.05(slow), no joint limit violation")
    succ = [r for r in rows if r["final_ee_pos_err"] < 0.03 and r["joint_limit_steps"] == 0]
    rep.append(f"- success runs: {len(succ)}")
    (out_dir / "joint_quintic_vsd_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")

    # summary report
    rows_pd = [r for r in rows if r["controller_mode"] == "pd_bias"]
    rows_ct = [r for r in rows if r["controller_mode"] == "computed_torque"]
    bpd = min(rows_pd, key=lambda r: r["final_ee_pos_err"])
    bct = min(rows_ct, key=lambda r: r["final_ee_pos_err"])
    rows_l0 = [r for r in rows if abs(r["lambda_task"] - 0.0) < 1e-12]
    rows_l1 = [r for r in rows if abs(r["lambda_task"] - 0.1) < 1e-12]
    rows_l3 = [r for r in rows if abs(r["lambda_task"] - 0.3) < 1e-12]

    def bestf(rr):
        return min(rr, key=lambda r: r["final_ee_pos_err"])

    summ = [
        "# IK Joint Tracking Summary",
        "",
        "1. Does joint-space quintic tracking solve the path?",
        f"- best final EE error: {best['final_ee_pos_err']:.6f}",
        "2. Does longer duration improve tracking?",
        f"- best per duration: {[bestf([r for r in rows if r['duration']==d])['final_ee_pos_err'] for d in [1.0,2.0,3.0,5.0]]}",
        "3. Are torque limits sufficient?",
        f"- best per tau limit: {[bestf([r for r in rows if r['tau_limit']==t])['final_ee_pos_err'] for t in [20.0,50.0,100.0]]}",
        "4. Does computed torque improve tracking?",
        f"- best pd_bias final: {bpd['final_ee_pos_err']:.6f}, best computed_torque final: {bct['final_ee_pos_err']:.6f}",
        "5. Does task-space residual correction help or hurt?",
        f"- best final by lambda: l0={bestf(rows_l0)['final_ee_pos_err']:.6f}, l0.1={bestf(rows_l1)['final_ee_pos_err']:.6f}, l0.3={bestf(rows_l3)['final_ee_pos_err']:.6f}",
        "6. Is this now ready to reintroduce q_act transmission?",
        "- Reintroduce only if success criterion is met in arm-only baseline.",
    ]
    summ.insert(1, f"- torque_sign used: {torque_sign.tolist()}")
    (out_dir / "summary_report.md").write_text("\n".join(summ) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
