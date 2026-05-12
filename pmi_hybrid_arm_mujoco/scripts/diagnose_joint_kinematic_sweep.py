#!/usr/bin/env python3
from __future__ import annotations

import csv
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
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

MODEL = PKG_ROOT / "models" / "pmi_arm_only_torque_debug.xml"
OUT = PKG_ROOT / "debug_outputs" / "kinematic_sign"
J = ["jnt1", "jnt2", "jnt3", "jnt4"]
RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=float)
QJ0 = RATIOS * np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=float)


def main() -> None:
    model = load_mjmodel(MODEL, strip_position_actuators=True)
    data = mj.MjData(model)
    scratch = mj.MjData(model)
    rows = []

    OUT.mkdir(parents=True, exist_ok=True)

    for ji, jn in enumerate(J):
        dqs = np.linspace(-0.3, 0.3, 121)
        p_ref = None
        rpy_ref = None
        for dq in dqs:
            q = QJ0.copy()
            q[ji] += dq
            for k, n in enumerate(J):
                data.qpos[int(model.jnt_qposadr[joint_id(model, n)])] = q[k]
            data.qvel[:] = 0.0
            mj.mj_forward(model, data)
            p, r, pch, y = fk_ee_rp(model, scratch, q, J)
            p = np.asarray(p, dtype=float)
            rpy = np.array([r, pch, y], dtype=float)
            if p_ref is None:
                p_ref = p.copy()
                rpy_ref = rpy.copy()
            dp = p - p_ref
            rows.append(
                {
                    "joint_name": jn,
                    "q_value": float(q[ji]),
                    "delta_q": float(dq),
                    "ee_x": float(p[0]),
                    "ee_y": float(p[1]),
                    "ee_z": float(p[2]),
                    "delta_ee_x": float(dp[0]),
                    "delta_ee_y": float(dp[1]),
                    "delta_ee_z": float(dp[2]),
                    "roll": float(r),
                    "pitch": float(pch),
                    "yaw": float(y),
                }
            )

        rr = [x for x in rows if x["joint_name"] == jn]
        dqv = np.array([x["delta_q"] for x in rr])
        for key in ["delta_ee_x", "delta_ee_y", "delta_ee_z"]:
            yv = np.array([x[key] for x in rr])
            plt.figure(figsize=(6, 3.5))
            plt.plot(dqv, yv)
            plt.grid(True, alpha=0.3)
            plt.xlabel("delta_q")
            plt.ylabel(key)
            plt.title(f"{jn}: {key}")
            plt.tight_layout()
            plt.savefig(OUT / f"{jn}_{key}.png", dpi=140)
            plt.close()

        plt.figure(figsize=(7, 4))
        for key in ["roll", "pitch", "yaw"]:
            yv = np.array([x[key] for x in rr])
            plt.plot(dqv, yv, label=key)
        plt.grid(True, alpha=0.3)
        plt.xlabel("delta_q")
        plt.legend()
        plt.title(f"{jn}: r/p/y")
        plt.tight_layout()
        plt.savefig(OUT / f"{jn}_rpy.png", dpi=140)
        plt.close()

    with open(OUT / "joint_sweep.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    def slope_sign(jn: str, key: str) -> float:
        rr = [x for x in rows if x["joint_name"] == jn]
        dqv = np.array([x["delta_q"] for x in rr])
        yv = np.array([x[key] for x in rr])
        c = np.polyfit(dqv, yv, 1)
        return float(c[0])

    signs = {jn: {k: slope_sign(jn, k) for k in ["delta_ee_x", "delta_ee_y", "delta_ee_z"]} for jn in J}

    def most_neg(axis_key: str) -> str:
        return min(J, key=lambda j: signs[j][axis_key])

    rep = [
        "# Joint Kinematic Sweep Report",
        "",
        f"- joint most moving EE x negative: {most_neg('delta_ee_x')}",
        f"- joint most moving EE y negative: {most_neg('delta_ee_y')}",
        f"- joint most moving EE z negative: {most_neg('delta_ee_z')}",
        f"- jnt1 positive direction slope signs: dx={signs['jnt1']['delta_ee_x']:.6f}, dy={signs['jnt1']['delta_ee_y']:.6f}, dz={signs['jnt1']['delta_ee_z']:.6f}",
        "- see per-joint plots for surprising axis/sign behaviors and RPY coupling.",
    ]
    (OUT / "joint_sweep_report.md").write_text("\n".join(rep) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
