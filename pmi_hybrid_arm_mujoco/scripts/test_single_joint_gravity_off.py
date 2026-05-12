#!/usr/bin/env python3
"""중력 ON/OFF 단일 관절 추종 비교 (동일 PD+bias)."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import mujoco as mj
import numpy as np

from utils.mujoco_helpers import PKG_ROOT

MODEL_PATH = PKG_ROOT / "models" / "pmi_arm_only_torque_debug.xml"
JOINT_NAMES = ["jnt1", "jnt2", "jnt3", "jnt4"]
OUT_DIR = PKG_ROOT / "debug_outputs" / "arm_only_physics"


def dof_addrs(model: mj.MjModel) -> np.ndarray:
    return np.array(
        [int(model.jnt_dofadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)]) for n in JOINT_NAMES],
        dtype=np.int32,
    )


def qpos_addrs(model: mj.MjModel) -> np.ndarray:
    return np.array(
        [int(model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)]) for n in JOINT_NAMES],
        dtype=np.int32,
    )


def joint_ranges(model: mj.MjModel) -> tuple[np.ndarray, np.ndarray]:
    lo, hi = [], []
    for n in JOINT_NAMES:
        jid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)
        lo.append(float(model.jnt_range[jid, 0]))
        hi.append(float(model.jnt_range[jid, 1]))
    return np.array(lo), np.array(hi)


def simulate_batch(
    *,
    gravity_on: bool,
    joint_idx: int,
    delta_rad: float,
    q_init: np.ndarray,
    duration: float,
    Kp: float,
    Kd: float,
) -> tuple[list[dict], float]:
    model = mj.MjModel.from_xml_path(str(MODEL_PATH))
    data = mj.MjData(model)
    if not gravity_on:
        model.opt.gravity[:] = 0.0

    dt = float(model.opt.timestep)
    qadr = qpos_addrs(model)
    dadr = dof_addrs(model)
    q_lo, q_hi = joint_ranges(model)

    q_des = np.asarray(q_init, dtype=np.float64).copy()
    q_des[joint_idx] = np.clip(q_des[joint_idx] + float(delta_rad), q_lo[joint_idx], q_hi[joint_idx])

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    for i in range(4):
        data.qpos[int(qadr[i])] = float(q_init[i])
    mj.mj_forward(model, data)

    rows: list[dict] = []
    n_steps = int(np.ceil(duration / dt))
    max_err = 0.0
    gtag = "gravity_on" if gravity_on else "gravity_off"
    for k in range(n_steps):
        mj.mj_forward(model, data)
        q = np.array([float(data.qpos[int(a)]) for a in qadr])
        qd = np.array([float(data.qvel[int(a)]) for a in dadr])
        tau_bias = np.array([float(data.qfrc_bias[int(d)]) for d in dadr])
        qdot_des = np.zeros(4)
        tau_cmd = tau_bias + float(Kp) * (q_des - q) + float(Kd) * (qdot_des - qd)

        data.qfrc_applied[:] = 0.0
        for i in range(4):
            data.qfrc_applied[int(dadr[i])] += float(tau_cmd[i])

        err = float(np.linalg.norm(q_des - q))
        max_err = max(max_err, err)

        rows.append(
            {
                "gravity_mode": gtag,
                "joint_under_test": JOINT_NAMES[joint_idx],
                "delta_rad": float(delta_rad),
                "step": k,
                "t": float(k * dt),
                **{f"q_{JOINT_NAMES[j]}": float(q[j]) for j in range(4)},
                "q_err_norm": err,
                **{f"tau_{JOINT_NAMES[j]}": float(tau_cmd[j]) for j in range(4)},
            }
        )
        mj.mj_step(model, data)

    return rows, max_err


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=2.0)
    ap.add_argument("--Kp", type=float, default=40.0)
    ap.add_argument("--Kd", type=float, default=4.0)
    ap.add_argument("--q-init", type=float, nargs=4, default=[0.0, 0.0, 0.0, 0.0])
    args = ap.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    deltas = [0.2, -0.2, 0.5, -0.5]
    q0 = np.array(args.q_init, dtype=np.float64)

    all_rows: list[dict] = []
    summ = ["# Gravity on/off single-joint tracking\n", "", "| gravity | joint | Δq | max ‖q_des−q‖₂ |", "|---------|-------|-----|-----------------|"]

    for g_on in (True, False):
        for ji in range(4):
            for dlt in deltas:
                rows, mx = simulate_batch(
                    gravity_on=g_on,
                    joint_idx=ji,
                    delta_rad=dlt,
                    q_init=q0,
                    duration=float(args.duration),
                    Kp=float(args.Kp),
                    Kd=float(args.Kd),
                )
                all_rows.extend(rows)
                summ.append(f"| {'on' if g_on else 'off'} | {JOINT_NAMES[ji]} | {dlt:g} | {mx:.8g} |")

    csv_path = OUT_DIR / "single_joint_tracking_gravity_compare.csv"
    if all_rows:
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
            w.writeheader()
            w.writerows(all_rows)

    Path(OUT_DIR / "single_joint_tracking_gravity_compare_report.md").write_text("\n".join(summ) + "\n", encoding="utf-8")
    print(f"CSV → {csv_path.resolve()}")


if __name__ == "__main__":
    main()
