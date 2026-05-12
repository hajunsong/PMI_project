#!/usr/bin/env python3
"""VSD baseline + hybrid transmission; logs belt vs cable torque components."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt
import numpy as np

from envs.pmi_cable_arm_env import PMICableArmEnv


def main() -> None:
    env = PMICableArmEnv(render_mode=None, randomize_transmission=False)
    env.reset(seed=0)

    n_steps = 2500
    qs, q_des_log = [], []
    tau_vsd, tau_belt = [], []
    tau_del234 = []
    tau_cmd234 = []
    tpc2, tmc2, tpo2, tmo2 = [], [], [], []
    loss2, loss3, loss4 = [], [], []
    z2, dz2, bz2 = [], [], []

    for _ in range(n_steps):
        obs, _, _, _, info = env.step(np.zeros(4, dtype=np.float32))
        qs.append(obs[:4].copy())
        q_des_log.append(obs[8:12].copy())
        tau_vsd.append(info["tau_vsd"].copy())
        tau_belt.append(info["tau_belt_jnt1"])
        tt = info["tau_joint_delivered"]
        tau_del234.append(tt[1:4].copy())
        tv = info["tau_vsd"] + info["lambda_tau_residual"]
        tau_cmd234.append(tv[1:4].copy())
        cd = info["cable_transmission"]
        pj = cd.get("per_joint", [{}, {}, {}])
        d0, d1, d2 = pj[0], pj[1], pj[2]
        tpc2.append(float(d0.get("T_plus_cmd", 0.0)))
        tmc2.append(float(d0.get("T_minus_cmd", 0.0)))
        tpo2.append(float(d0.get("T_plus_out", 0.0)))
        tmo2.append(float(d0.get("T_minus_out", 0.0)))
        loss2.append(float(d0.get("torque_loss_joint", 0.0)))
        loss3.append(float(d1.get("torque_loss_joint", 0.0)))
        loss4.append(float(d2.get("torque_loss_joint", 0.0)))
        z2.append(float(d0.get("z_plus", 0.0)))
        dz2.append(float(d0.get("plus", {}).get("deadzone_active", False)))
        bz2.append(float(d0.get("plus", {}).get("backlash_active", False)))

    qs = np.array(qs)
    q_des_log = np.array(q_des_log)
    tau_vsd = np.array(tau_vsd)
    tau_del234 = np.array(tau_del234)
    tau_cmd234 = np.array(tau_cmd234)
    time = np.arange(n_steps) * env.dt * env.frame_skip

    fig, ax = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    names = ["jnt1 (belt)", "jnt2 (cable)", "jnt3 (cable)", "jnt4 (cable)"]
    for i in range(4):
        ax[i].plot(time, qs[:, i], label="q")
        ax[i].plot(time, q_des_log[:, i], "--", label="q_des")
        ax[i].set_ylabel(names[i])
        ax[i].legend(loc="upper right", fontsize=8)
    ax[-1].set_xlabel("time [s]")
    plt.suptitle("Hybrid transmission — VSD only (zero residual)")
    plt.tight_layout()
    outp = ROOT / "scripts" / "_vsd_tracking.png"
    plt.savefig(outp, dpi=120)
    print("Saved", outp)

    fig2, ax2 = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    ax2[0].plot(time, tau_vsd[:, 0], label="tau_vsd_1")
    ax2[0].plot(time, tau_belt, label="tau_belt_jnt1")
    ax2[0].legend()
    ax2[0].set_ylabel("jnt1 torque [Nm]")
    ax2[1].plot(time, tau_cmd234[:, 0], label="tau_motor_cmd_j2 (joint)")
    ax2[1].plot(time, tau_del234[:, 0], label="tau_joint_transmitted_j2")
    ax2[1].plot(time, loss2, ":", label="torque_loss_j2")
    ax2[1].legend(fontsize=8)
    ax2[1].set_ylabel("jnt2 [Nm]")
    ax2[2].plot(time, tau_cmd234[:, 1], label="tau_cmd_j3")
    ax2[2].plot(time, tau_cmd234[:, 2], label="tau_cmd_j4")
    ax2[2].plot(time, tau_del234[:, 1], "--", label="tau_del_j3")
    ax2[2].plot(time, tau_del234[:, 2], "--", label="tau_del_j4")
    ax2[2].legend(ncol=2, fontsize=8)
    ax2[2].set_ylabel("jnt3/4 [Nm]")
    ax2[-1].set_xlabel("time [s]")
    plt.tight_layout()
    outp2 = ROOT / "scripts" / "_vsd_torque_components.png"
    plt.savefig(outp2, dpi=120)
    print("Saved", outp2)

    fig3, ax3 = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    ax3[0].plot(time, tpc2, label="T_plus_cmd j2")
    ax3[0].plot(time, tmc2, label="T_minus_cmd j2")
    ax3[0].plot(time, tpo2, "--", label="T_plus_out")
    ax3[0].plot(time, tmo2, "--", label="T_minus_out")
    ax3[0].legend(fontsize=8)
    ax3[0].set_ylabel("tension j2 [N]")
    ax3[1].plot(time, z2, label="z_plus j2")
    ax3[1].legend()
    ax3[1].set_ylabel("hyst z")
    ax3[2].plot(time, np.asarray(dz2, dtype=float), label="deadzone_active plus")
    ax3[2].plot(time, np.asarray(bz2, dtype=float), label="backlash_active plus")
    ax3[2].set_ylim(-0.1, 1.1)
    ax3[2].legend()
    ax3[2].set_ylabel("flags")
    ax3[-1].set_xlabel("time [s]")
    plt.suptitle("jnt2 cable diagnostics (T, z, flags)")
    plt.tight_layout()
    outp3 = ROOT / "scripts" / "_vsd_cable_diag_j2.png"
    plt.savefig(outp3, dpi=120)
    print("Saved", outp3)


if __name__ == "__main__":
    main()
