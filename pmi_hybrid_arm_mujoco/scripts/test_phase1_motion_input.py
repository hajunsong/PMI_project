#!/usr/bin/env python3
"""
PHASE 1: mode=motion
- q_act 에 사인 위치 명령 (position actuator)
- jnt 는 Python에서 이상 전동비로 강제 동기화 (MJCF equality 미사용; URDF mimic Python 경로)
"""

from __future__ import annotations

from pathlib import Path
import sys

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np
import mujoco as mj

from utils.mujoco_helpers import (
    DEFAULT_MODEL_PATH,
    apply_kinematic_transmission,
    load_yaml,
    transmission_from_config,
    site_pose_world,
    PKG_ROOT,
)


def main() -> None:
    mode = "motion"
    sim = load_yaml(PKG_ROOT / "configs" / "simulation.yaml")
    rob = load_yaml(PKG_ROOT / "configs" / "robot_transmission.yaml")
    vsd = load_yaml(PKG_ROOT / "configs" / "vsd_params.yaml")
    traj = vsd["trajectory"]

    rows = transmission_from_config(rob)
    ratios = np.array([r.ratio for r in rows], dtype=np.float64)

    model = mj.MjModel.from_xml_path(str(DEFAULT_MODEL_PATH))
    data = mj.MjData(model)

    dt = float(model.opt.timestep)
    tol = float(sim.get("phase1_tol_rad", 1e-3))
    T = float(sim.get("duration_sec_phase1", 5.5))
    n_step = int(round(T / dt))

    from utils.mujoco_helpers import q_act_trajectory_vectors

    log_t: list[float] = []
    q_act_h: list[np.ndarray] = []
    q_j_h: list[np.ndarray] = []
    q_je_h: list[np.ndarray] = []
    ee_h: list[np.ndarray] = []

    max_err_all = 0.0

    for k in range(n_step):
        t = k * dt
        q_des_vec, _ = q_act_trajectory_vectors(t, traj)
        data.ctrl[:] = q_des_vec

        mj.mj_step(model, data)
        apply_kinematic_transmission(model, data, rows)

        q_act = np.array([data.qpos[model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)]] for n in rob["actuator_order"]])
        q_j = np.array([data.qpos[model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)]] for n in rob["joint_side_order"]])
        q_exp = ratios * q_act
        err = np.abs(q_j - q_exp)
        max_err_all = max(max_err_all, float(err.max()))

        pos, _mat = site_pose_world(model, data, "end_effector")
        log_t.append(t)
        q_act_h.append(q_act.copy())
        q_j_h.append(q_j.copy())
        q_je_h.append(q_exp.copy())
        ee_h.append(pos.copy())

        if not np.isfinite(q_act).all() or not np.isfinite(q_j).all():
            raise RuntimeError("비정상 수치(NaN/Inf) 발생 — 시뮬 중단")

    t_arr = np.array(log_t)
    q_act_a = np.stack(q_act_h)
    q_j_a = np.stack(q_j_h)
    q_je_a = np.stack(q_je_h)
    ee_a = np.stack(ee_h)
    err_a = q_j_a - q_je_a

    print(f"mode={mode}")
    print(f"max(abs(jnt_actual - ratio*q_act)) = {max_err_all:.6g} rad")
    print(f"tolerance = {tol:g} rad → {'PASS' if max_err_all < tol else 'FAIL'}")

    fig_dir = PKG_ROOT / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)

    from utils.plotting import plot_joint_motion, plot_tracking_error

    plot_joint_motion(t_arr, q_act_a, q_j_a, q_je_a, ee_a, fig_dir / "phase1_motion.png")
    plot_tracking_error(t_arr, err_a, fig_dir / "phase1_joint_error.png")


if __name__ == "__main__":
    main()
