#!/usr/bin/env python3
"""Cable hysteresis / tension loops for jnt2–jnt4 (antagonistic stack, no belt on jnt1)."""

from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt
import numpy as np
import yaml

import mujoco

from kinematics.pmi_chain import MOTOR_TO_JOINT_GEAR, actuator_torque_from_joint_torque
from transmission.antagonistic_cable_joint import build_antagonistic_stack_from_config

MODEL = ROOT / "models" / "pmi_cable_arm.xml"
CFG_PATH = ROOT / "configs" / "cable_params.yaml"


def main() -> None:
    with open(CFG_PATH, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg.setdefault("randomization", {})["enabled"] = False
    stack = build_antagonistic_stack_from_config(cfg)
    stack.reset()

    m = mujoco.MjModel.from_xml_path(os.fspath(MODEL))
    d = mujoco.MjData(m)

    jids = [
        mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        for n in ("jnt2", "jnt3", "jnt4")
    ]

    dt = float(m.opt.timestep)
    T = 12.0
    steps = int(T / dt)

    tau_amp = np.array([5.0, 4.0, 3.0])
    freq = np.array([0.12, 0.14, 0.11])
    gears = MOTOR_TO_JOINT_GEAR[1:4].astype(np.float64)

    log_xp = []
    log_Tp = []
    log_z = []

    mujoco.mj_resetData(m, d)

    for k in range(steps):
        t = k * dt
        tau_joint_cmd = tau_amp * np.sin(2.0 * np.pi * freq * t)

        q234 = np.array([d.qpos[m.jnt_qposadr[j]] for j in jids])
        qd234 = np.array([d.qvel[m.jnt_dofadr[j]] for j in jids])

        tau_act_cmd = tau_joint_cmd * gears
        _tau_j234, cdiag = stack.transmit(tau_act_cmd, q234, qd234, dt)

        tau_joint = np.zeros(4, dtype=np.float64)
        tau_joint[1:4] = np.clip(_tau_j234, -30.0, 30.0)
        d.ctrl[: m.nu] = actuator_torque_from_joint_torque(tau_joint)

        mujoco.mj_step(m, d)

        row_xp = []
        row_Tp = []
        row_z = []
        for dj in cdiag["per_joint"]:
            row_xp.append(float(dj["x_plus"]))
            row_Tp.append(float(dj["T_plus_out"]))
            row_z.append(float(dj["z_plus"]))
        log_xp.append(row_xp)
        log_Tp.append(row_Tp)
        log_z.append(row_z)

    log_xp = np.array(log_xp)
    log_Tp = np.array(log_Tp)
    log_z = np.array(log_z)

    labels = ["jnt2", "jnt3", "jnt4"]
    fig, ax = plt.subplots(2, 2, figsize=(10, 9))
    ax = ax.ravel()
    for i in range(3):
        ax[i].plot(log_xp[:, i], log_Tp[:, i], lw=0.6)
        ax[i].set_title(f"{labels[i]}: T_plus_out vs x_plus")
        ax[i].set_xlabel("x_plus [m]")
        ax[i].set_ylabel("T_plus_out [N]")
    ax[3].axis("off")
    plt.tight_layout()
    out = ROOT / "scripts" / "_cable_hysteresis_force_extension.png"
    plt.savefig(out, dpi=120)
    print("Saved", out)

    fig2, ax2 = plt.subplots(1, 3, figsize=(12, 4))
    for i in range(3):
        ax2[i].plot(log_xp[:, i], log_z[:, i], lw=0.6)
        ax2[i].set_title(f"Bouc–Wen z_plus vs x_plus ({labels[i]})")
        ax2[i].set_xlabel("x_plus [m]")
        ax2[i].set_ylabel("z_plus")
    plt.tight_layout()
    out2 = ROOT / "scripts" / "_cable_hysteresis_z_extension.png"
    plt.savefig(out2, dpi=120)
    print("Saved", out2)


if __name__ == "__main__":
    main()
