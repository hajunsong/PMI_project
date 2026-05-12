#!/usr/bin/env python3
"""
PHASE 2: mode = torque_vsd
- MJCF에는 equality를 넣지 않음(중복 DOF). 대신 스텝 후 `apply_kinematic_transmission` 으로 모터 축을 관절 상태에 맞춤(Phase1과 동일한 기하 전달).
- 관절 dof에 이상 전달 토크 tau_joint_ideal = tau_act_cmd / ratio 적용(Strategy A).

관절 변수 q_act 는 kinematic slave로 유지되며, VSD는 q_act, qdot_act(= q_jnt/ratio 에서 유도) 로 계산한다.
"""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np
import mujoco as mj

from controllers.vsd_controller import VSDController, VSDParams
from utils.mujoco_helpers import (
    PKG_ROOT,
    DEFAULT_MODEL_PATH,
    apply_kinematic_transmission,
    compile_model_for_simulation,
    ideal_actuator_torque_to_joint_torque,
    joint_id,
    load_yaml,
    q_act_trajectory_vectors,
    site_pose_world,
    transmission_from_config,
)


def actuator_joint_ranges(model: mj.MjModel, actuator_names: list[str]) -> tuple[np.ndarray, np.ndarray]:
    lo: list[float] = []
    hi: list[float] = []
    for n in actuator_names:
        jid = joint_id(model, n)
        lo.append(float(model.jnt_range[jid, 0]))
        hi.append(float(model.jnt_range[jid, 1]))
    return np.array(lo), np.array(hi)


def clamp_joint_space_desired(
    q_act_des: np.ndarray,
    ratios: np.ndarray,
    j_lo: np.ndarray,
    j_hi: np.ndarray,
) -> np.ndarray:
    qj = ratios * q_act_des
    qj = np.clip(qj, j_lo, j_hi)
    return qj / ratios


def main() -> None:
    mode = "torque_vsd"
    sim = load_yaml(PKG_ROOT / "configs" / "simulation.yaml")
    rob = load_yaml(PKG_ROOT / "configs" / "robot_transmission.yaml")
    vsd_cfg = load_yaml(PKG_ROOT / "configs" / "vsd_params.yaml")
    traj = vsd_cfg["trajectory"]
    traj_type = traj.get("type", "sinusoidal")

    rows = transmission_from_config(rob)
    actuator_names = list(rob["actuator_order"])
    joint_names_side = list(rob["joint_side_order"])
    ratios = np.array([r.ratio for r in rows], dtype=np.float64)

    model = compile_model_for_simulation(
        DEFAULT_MODEL_PATH,
        rows,
        equality_mode="none",
        strip_position_actuators=True,
    )
    data = mj.MjData(model)

    qa_lo, qa_hi = actuator_joint_ranges(model, actuator_names)
    dof_act = np.array([model.jnt_dofadr[joint_id(model, n)] for n in actuator_names])
    adr_j_side = np.array([model.jnt_qposadr[joint_id(model, n)] for n in joint_names_side])
    dof_all_jnt = np.array([model.jnt_dofadr[joint_id(model, nm)] for nm in joint_names_side])

    kp = np.array([vsd_cfg["Kp"][n] for n in actuator_names], dtype=np.float64)
    kd = np.array([vsd_cfg["Kd"][n] for n in actuator_names], dtype=np.float64)
    tmax = np.array([vsd_cfg["tau_max_nm"][n] for n in actuator_names], dtype=np.float64)
    ctr = VSDController(VSDParams(Kp=kp, Kd=kd, tau_max=tmax, joint_names_order=tuple(actuator_names)))

    dt = float(model.opt.timestep)
    T = float(sim.get("duration_sec_phase2", 6.5))
    n_step = int(round(T / dt))
    margin_rad = float(sim.get("phase2_joint_limit_margin_rad", 0.05))

    j_lo: list[float] = []
    j_hi: list[float] = []
    for nm in joint_names_side:
        jid = joint_id(model, nm)
        j_lo.append(float(model.jnt_range[jid, 0]) + margin_rad)
        j_hi.append(float(model.jnt_range[jid, 1]) - margin_rad)
    j_lo_a = np.array(j_lo)
    j_hi_a = np.array(j_hi)

    log_t: list[float] = []
    qad: list[np.ndarray] = []
    qaa: list[np.ndarray] = []
    qje: list[np.ndarray] = []
    qja: list[np.ndarray] = []
    tau_act: list[np.ndarray] = []
    tau_jnt: list[np.ndarray] = []
    ee: list[np.ndarray] = []

    switch = traj.get("step_switch_sec", 0.8)

    for k in range(n_step):
        t_s = k * dt
        q_des_raw, qd_des_raw = q_act_trajectory_vectors(
            t_s,
            traj,
            switch_override=switch if traj_type == "step" else None,
        )
        qa_des_raw = np.clip(q_des_raw, qa_lo, qa_hi)
        qa_des_vec = clamp_joint_space_desired(qa_des_raw, ratios, j_lo_a, j_hi_a)
        qa_des = np.clip(qa_des_vec, qa_lo, qa_hi)

        q_act = np.array([data.qpos[model.jnt_qposadr[joint_id(model, n)]] for n in actuator_names])
        q_dot_act = np.array([data.qvel[dof] for dof in dof_act])

        tau_cmd = ctr.compute(q_act, q_dot_act, qa_des, qd_des_raw)
        tau_j_ideal = ideal_actuator_torque_to_joint_torque(tau_cmd, ratios)

        data.qfrc_applied[:] = 0.0
        data.ctrl[:] = 0.0
        for dof_j, t_j in zip(dof_all_jnt, tau_j_ideal, strict=True):
            data.qfrc_applied[int(dof_j)] += float(t_j)

        mj.mj_step(model, data)
        apply_kinematic_transmission(model, data, rows)

        q_act_meas = np.array([data.qpos[model.jnt_qposadr[joint_id(model, n)]] for n in actuator_names])
        q_j_meas = np.array([data.qpos[adr] for adr in adr_j_side])

        pos, _ = site_pose_world(model, data, "end_effector")

        log_t.append(t_s)
        qad.append(qa_des.copy())
        qaa.append(q_act_meas.copy())
        qje.append((ratios * qa_des).copy())
        qja.append(q_j_meas.copy())
        tau_act.append(tau_cmd.copy())
        tau_jnt.append(tau_j_ideal.copy())
        ee.append(pos.copy())

        if not np.isfinite(data.qpos).all():
            raise RuntimeError("순환 중 비정상 qpos 발견(NaN)")

    ta = np.array(log_t)

    from utils.plotting import plot_phase2_bundle

    plot_phase2_bundle(
        ta,
        np.stack(qad),
        np.stack(qaa),
        np.stack(qje),
        np.stack(qja),
        np.stack(tau_act),
        np.stack(tau_jnt),
        np.stack(ee),
        PKG_ROOT / "figures" / "phase2_vsd.png",
    )

    jerr = np.stack(qja) - np.stack(qje)
    rmse_all = float(np.sqrt(np.mean(np.square(jerr))))
    print(f"mode={mode}")
    print(f"joint-tracking RMSE (모든 원소 평균) ≈ {rmse_all:.5g} rad")
    print("그래프: figures/phase2_vsd.png")


if __name__ == "__main__":
    main()
