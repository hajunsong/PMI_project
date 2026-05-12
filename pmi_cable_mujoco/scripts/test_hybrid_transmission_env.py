#!/usr/bin/env python3
"""Verify PMICableArmEnv: jnt1 belt vs jnt2–4 cable split and z_cable shape."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np

from envs.pmi_cable_arm_env import PMICableArmEnv


def main() -> None:
    env = PMICableArmEnv(randomize_transmission=False)
    env.reset(seed=1)

    for step_i in range(30):
        obs, _, _, _, info = env.step(np.zeros(4, dtype=np.float32))

        tb = info["tau_belt_jnt1"]
        tt = info["tau_joint_delivered"]
        tv = info["tau_vsd"]
        lam_tr = info["lambda_tau_residual"]
        cd = info["cable_transmission"]

        print(
            f"step {step_i:02d} | tau_belt_jnt1={tb:+.5f} "
            f"tau_joint_delivered={np.array2string(tt, precision=3)}"
        )

        tau_drive_1 = tv[0] + lam_tr[0]
        recon0 = tau_drive_1 + tb
        assert np.isclose(tt[0], recon0, rtol=0, atol=0.05), (tt[0], recon0)

        assert "per_joint" in cd and len(cd["per_joint"]) == 3
        for ji, dj in enumerate(cd["per_joint"]):
            assert np.isclose(tt[1 + ji], float(dj["tau_joint_transmitted"]), rtol=0, atol=1e-6)

    assert info["z_cable"].shape == (3,)
    print("Hybrid transmission OK; z_cable shape:", info["z_cable"].shape)


if __name__ == "__main__":
    main()
