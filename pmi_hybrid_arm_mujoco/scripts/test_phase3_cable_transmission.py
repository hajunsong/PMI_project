#!/usr/bin/env python3
"""
PHASE 3:
- jnt1 / q1_act: MJCF equality(벨트) + VSD actuator 토크
- jnt2~jnt4: equality 없음 → antagonistic 에서 나온 joint 토크를 관절 DOF 에 직접 가함
- 모터 쪽: 근사 반력 tau_m_residual = tau_vsd - r_motor * T_net
"""

from __future__ import annotations

import random
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np
import mujoco as mj

from controllers.vsd_controller import VSDController, VSDParams
from transmission.antagonistic_cable_joint import AntagonisticCableJoint, AntagonisticParams
from transmission.cable_params_io import cable_transmission_nominal
from transmission.randomization import randomize_cable_transmission_params, random_preload
from utils.mujoco_helpers import (
    DEFAULT_MODEL_PATH,
    PKG_ROOT,
    compile_model_for_simulation,
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
    mode = "cable_torque_vsd_phase3"
    sim = load_yaml(PKG_ROOT / "configs" / "simulation.yaml")
    rob = load_yaml(PKG_ROOT / "configs" / "robot_transmission.yaml")
    cable_yaml = load_yaml(PKG_ROOT / "configs" / "cable_params.yaml")
    vsd_cfg = load_yaml(PKG_ROOT / "configs" / "vsd_params.yaml")
    traj = vsd_cfg["trajectory"]

    rnd_cable_flag = bool(sim.get("randomize_cable_params", False))
    rnd_frac = float(sim.get("randomization_fraction", 0.05))
    use_antagonistic = bool(cable_yaml.get("use_antagonistic", True))
    if not use_antagonistic:
        raise SystemExit("config use_antagonistic=false 는 본 예제에서는 미구현")

    nominal_preload = float(cable_yaml.get("preload_tension_nominal", 1.0))

    rows_all = transmission_from_config(rob)
    rows_by_joint = {r.joint: r for r in rows_all}
    actuator_names = list(rob["actuator_order"])
    joint_side = list(rob["joint_side_order"])
    ratios = np.array([rows_by_joint[j].ratio for j in joint_side])

    rng = random.Random(1)
    antagonists: dict[str, AntagonisticCableJoint] = {}
    for jn in ("jnt2", "jnt3", "jnt4"):
        if rnd_cable_flag:
            cparams = randomize_cable_transmission_params(cable_yaml, rng=rng, scale=rnd_frac)
            preload = random_preload(nominal_preload, rng=rng, scale=rnd_frac)
        else:
            cparams = cable_transmission_nominal(cable_yaml)
            preload = nominal_preload
        antagonists[jn] = AntagonisticCableJoint(cparams, AntagonisticParams(preload_tension=preload))

    model = compile_model_for_simulation(
        DEFAULT_MODEL_PATH,
        rows_all,
        equality_mode="belt_only",
        strip_position_actuators=True,
    )
    data = mj.MjData(model)

    qa_lo, qa_hi = actuator_joint_ranges(model, actuator_names)
    dof_all_act = np.array([model.jnt_dofadr[joint_id(model, n)] for n in actuator_names])
    adr_j_side = np.array([model.jnt_qposadr[joint_id(model, nm)] for nm in joint_side])
    dof_all_jnt = np.array([model.jnt_dofadr[joint_id(model, nm)] for nm in joint_side])

    kp = np.array([vsd_cfg["Kp"][n] for n in actuator_names])
    kd = np.array([vsd_cfg["Kd"][n] for n in actuator_names])
    tmax = np.array([vsd_cfg["tau_max_nm"][n] for n in actuator_names])
    ctr = VSDController(VSDParams(Kp=kp, Kd=kd, tau_max=tmax, joint_names_order=tuple(actuator_names)))

    dt = float(model.opt.timestep)
    Tfin = float(sim.get("duration_sec_phase3", 7.5))
    n_step = int(round(Tfin / dt))
    margin_rad = float(sim.get("phase2_joint_limit_margin_rad", 0.05))

    j_lo: list[float] = []
    j_hi: list[float] = []
    for nm in joint_side:
        jid = joint_id(model, nm)
        j_lo.append(float(model.jnt_range[jid, 0]) + margin_rad)
        j_hi.append(float(model.jnt_range[jid, 1]) - margin_rad)
    j_lo_a = np.array(j_lo)
    j_hi_a = np.array(j_hi)

    traj_type = traj.get("type", "sinusoidal")
    switch = traj.get("step_switch_sec", 0.8)

    for ag in antagonists.values():
        ag.reset()

    logs: dict[str, list] = {"t": []}

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
        q_dot_act = np.array([data.qvel[d_i] for d_i in dof_all_act])

        tau_vsd_vec = ctr.compute(q_act, q_dot_act, qa_des, qd_des_raw)

        data.qfrc_applied[:] = 0.0

        tau_joint_cmd = np.zeros(4, dtype=np.float64)
        tau_joint_ideal = tau_vsd_vec / ratios
        tau_motor_applied = np.zeros(4, dtype=np.float64)
        tau_motor_residual = tau_vsd_vec.astype(np.float64).copy()

        tau_motor_applied[0] = tau_vsd_vec[0]

        for idx in range(1, 4):
            nm = joint_side[idx]
            rrow = rows_by_joint[nm]
            an = actuator_names[idx]

            qa = float(data.qpos[model.jnt_qposadr[joint_id(model, an)]])
            qj = float(data.qpos[adr_j_side[idx]])
            qad = float(data.qvel[dof_all_act[idx]])
            qjd = float(data.qvel[dof_all_jnt[idx]])

            tau_mot = tau_vsd_vec[idx]
            ag = antagonists[nm]
            tau_j_tr, diag = ag.transmit(
                tau_mot,
                rrow.r_joint,
                rrow.r_motor,
                qa,
                qj,
                qad,
                qjd,
                0.0,
                0.0,
                dt,
            )
            tau_joint_cmd[idx] = tau_j_tr
            T_net = float(diag["T_net"])

            tau_m_res = float(tau_mot - rrow.r_motor * T_net)
            tau_motor_residual[idx] = tau_m_res
            tau_motor_applied[idx] = tau_m_res

        data.qfrc_applied[dof_all_act] += tau_motor_applied
        for idx in range(1, 4):
            data.qfrc_applied[dof_all_jnt[idx]] += float(tau_joint_cmd[idx])

        mj.mj_step(model, data)

        q_j_meas = np.array([data.qpos[int(adr_j_side[i])] for i in range(4)])

        pos, mat = site_pose_world(model, data, "end_effector")

        if k == 0:
            for key in ("tau_vsd", "tau_motor_residual", "tau_joint_ideal", "tau_joint_partial", "qj", "qa", "qdes", "ee"):
                logs[key] = []

        logs["t"].append(t_s)
        logs["tau_vsd"].append(tau_vsd_vec.copy())
        logs["tau_motor_residual"].append(tau_motor_residual.copy())
        logs["tau_joint_ideal"].append(tau_joint_ideal.copy())
        logs["tau_joint_partial"].append(tau_joint_cmd.copy())
        logs["qj"].append(q_j_meas.copy())
        logs["qa"].append(q_act.copy())
        logs["qdes"].append(qa_des.copy())
        logs["ee"].append(np.concatenate([pos, mat.reshape(-1)]))

        if not np.isfinite(data.qpos).all():
            raise RuntimeError("순환 중 비정상 qpos 발견(NaN)")

    from utils.plotting import plot_phase2_bundle

    ta = np.array(logs["t"])
    qa_des_hist = np.stack(logs["qdes"])
    qaa_hist = np.stack(logs["qa"])
    qje_hist = qa_des_hist * ratios
    qja_hist = np.stack(logs["qj"])
    tau_act_hist = np.stack(logs["tau_vsd"])
    tau_j_ideal_hist = np.stack(logs["tau_joint_ideal"])

    fig_dir = PKG_ROOT / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    plot_phase2_bundle(
        ta,
        qa_des_hist,
        qaa_hist,
        qje_hist,
        qja_hist,
        tau_act_hist,
        tau_j_ideal_hist,
        np.stack(logs["ee"])[:, :3],
        fig_dir / "phase3_tracking_torque_bundle.png",
    )

    print(f"mode={mode}")
    print(f"저장 그림: {(fig_dir / 'phase3_tracking_torque_bundle.png').relative_to(PKG_ROOT)}")


if __name__ == "__main__":
    main()
