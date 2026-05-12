#!/usr/bin/env python3
"""Exercise jnt1 belt-only effects (jnt2–4 torques zero); plot tau_belt diagnostics."""

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

from kinematics.pmi_chain import MOTOR_TO_JOINT_GEAR
from transmission.belt_model import build_belt_model_from_config

MODEL = ROOT / "models" / "pmi_cable_arm.xml"
CFG_PATH = ROOT / "configs" / "belt_params.yaml"


def main() -> None:
    with open(CFG_PATH, "r", encoding="utf-8") as f:
        belt_cfg = yaml.safe_load(f)
    belt_cfg.setdefault("randomization", {})["enabled"] = False
    belt = build_belt_model_from_config(belt_cfg)
    belt.reset()

    m = mujoco.MjModel.from_xml_path(os.fspath(MODEL))
    d = mujoco.MjData(m)

    j1 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "jnt1")
    a1 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "tau_q1_act")
    for name in ("tau_q2_act", "tau_q3_act", "tau_q4_act"):
        aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        d.ctrl[aid] = 0.0

    dt = float(m.opt.timestep)
    steps = int(10.0 / dt)

    tau_amp = 5.0
    freq_hz = 0.2

    log_tb = []
    log_visc = []
    log_eta = []
    q_log = []

    mujoco.mj_resetData(m, d)

    for k in range(steps):
        t = k * dt
        tau_drive = tau_amp * np.sin(2.0 * np.pi * freq_hz * t)
        q = float(d.qpos[m.jnt_qposadr[j1]])
        qd = float(d.qvel[m.jnt_dofadr[j1]])
        q_des = 0.0

        tau_belt, diag = belt.compute_effect(q, qd, q_des, dt, tau_drive)
        tau_cmd = tau_drive + tau_belt

        # 관절 1 일반화력 → 구동축 토크 (q_jnt = gear * q_act)
        d.ctrl[a1] = float(np.clip(tau_cmd * float(MOTOR_TO_JOINT_GEAR[0]), -22.0, 22.0))
        mujoco.mj_step(m, d)

        log_tb.append(tau_belt)
        log_visc.append(diag["tau_viscous"])
        log_eta.append(diag["tau_eta_loss"])
        q_log.append(q)

    time = np.arange(steps) * dt

    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    ax[0].plot(time, log_tb)
    ax[0].set_ylabel("tau_belt_jnt1 [Nm]")
    ax[1].plot(time, log_eta, label="efficiency loss term")
    ax[1].plot(time, log_visc, label="viscous term")
    ax[1].legend()
    ax[1].set_ylabel("components [Nm]")
    ax[2].plot(time, q_log)
    ax[2].set_ylabel("q1 [rad]")
    ax[2].set_xlabel("time [s]")
    plt.suptitle("Belt/gear transmission — jnt1 only (open-loop tau_drive sine)")
    plt.tight_layout()
    out = ROOT / "scripts" / "_belt_jnt1_only.png"
    plt.savefig(out, dpi=120)
    print("Saved", out)


if __name__ == "__main__":
    main()
