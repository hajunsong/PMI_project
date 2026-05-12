#!/usr/bin/env python3
"""
Phase B (베이스라인/디버그): IK 로 생성된 q_j_des 궤적 + 관절 공간 VSD.

목표 제어 파이프라인(카드 작업 공간 VSD·Jacobian 전치)은
``scripts/test_phaseB_task_space_vsd_tracking.py`` 와 ``configs/task_space_vsd.yaml`` 참고.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import mujoco as mj
import numpy as np

from controllers.vsd_joint_controller import JointSpaceVSD, JointVSDParams, actuator_torques_from_joint
from kinematics.forward_kinematics import fk_ee_rp
from kinematics.inverse_kinematics import IKConfig, solve_ik
from kinematics.numeric_utils import central_difference_joint_series, moving_average_columns
from kinematics.orientation_utils import angle_error
from kinematics.trajectory import CartesianQuinticPath, WaypointXYZ
from utils.mujoco_helpers import (
    DEFAULT_MODEL_PATH,
    PKG_ROOT,
    apply_ideal_qact_from_joint,
    apply_ideal_qjnt_equals_ratio_qact,
    joint_id,
    load_mjmodel,
    passive_mujoco_viewer,
    populate_ik_path_overlay_geoms,
)
from utils.path_tracking_io import (
    actuator_from_joint_positions,
    joint_actuator_bounds,
    joint_from_actuator_positions,
    load_path_tracking_yaml,
    ordered_transmission_arrays,
    roll_pitch_des_from_orientation_config,
)
from utils.plotting import plot_phase_b_vsd_tracking


def read_q(model: mj.MjModel, data: mj.MjData, names: list[str]) -> np.ndarray:
    return np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in names])


def read_v(model: mj.MjModel, data: mj.MjData, names: list[str]) -> np.ndarray:
    return np.array([float(data.qvel[int(model.jnt_dofadr[joint_id(model, n)])]) for n in names])


def clamp_chain(
    q_jnt_sol: np.ndarray,
    ratios: np.ndarray,
    qlj: np.ndarray,
    qhj: np.ndarray,
    qla: np.ndarray,
    qha: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    qa = actuator_from_joint_positions(q_jnt_sol, ratios).clip(qla, qha)
    qj_cmd = joint_from_actuator_positions(qa, ratios)
    qj_cmd = np.clip(qj_cmd, qlj, qhj)
    qa_out = actuator_from_joint_positions(qj_cmd, ratios).clip(qla, qha)
    qj_fin = joint_from_actuator_positions(qa_out, ratios)
    return qj_fin, qa_out


def main() -> None:
    cfg = load_path_tracking_yaml()
    sim = cfg["simulation"]
    dt = float(sim["dt"])
    dur = float(sim["duration"])
    plots = bool(sim.get("save_plots", True))
    passive_viewer = bool(sim.get("passive_viewer", False))
    vw_scale = float(sim.get("viewer_realtime_scale", 1.0))
    hold_initial_pose_sec = float(sim.get("hold_initial_pose_sec", 0.0))
    show_ik_overlay = passive_viewer and bool(sim.get("show_ik_path_overlay", True))
    overlay_npts = max(2, int(sim.get("ik_path_overlay_samples", 128)))

    j_ord, a_ord, ratios = ordered_transmission_arrays(cfg)
    ratios = ratios.astype(np.float64)
    qa0 = np.array(cfg["initial_actuator_rad"], dtype=np.float64)
    qj0 = joint_from_actuator_positions(qa0, ratios)

    ik_y = cfg["ik"]
    ik_rng = np.random.default_rng(int(sim.get("rng_seed", 0)))
    ms_trials = int(ik_y.get("multistart_random_trials", 10))
    ms_thr = float(ik_y.get("multistart_if_geom_above", 0.05))
    w = ik_y["weights"]
    ik_cfg = IKConfig(
        weights_pos=tuple(float(x) for x in w["position"]),
        weight_roll=float(w["roll"]),
        weight_pitch=float(w["pitch"]),
        damping=float(ik_y["damping"]),
        regularization=float(ik_y["regularization"]),
        max_iterations=int(ik_y["max_iterations"]),
        tolerance_ftol=float(ik_y["tolerance"]),
        joint_side_order=tuple(j_ord),
    )

    pB = cfg["phaseB_vsd"]
    kp = np.array([float(x) for x in pB["Kp_joint"]], dtype=np.float64)
    kd = np.array([float(x) for x in pB["Kd_joint"]], dtype=np.float64)
    tlim = np.array([float(x) for x in pB["tau_jnt_limit"]], dtype=np.float64)
    mode = str(pB["torque_application_mode"]).strip()
    smooth_n = int(pB.get("qdot_smooth_iterations", 0))

    vsd = JointSpaceVSD(JointVSDParams(Kp=kp, Kd=kd, tau_jnt_limit=tlim))

    model = load_mjmodel(DEFAULT_MODEL_PATH, strip_position_actuators=True)
    model.opt.timestep = dt
    model.opt.integrator = int(mj.mjtIntegrator.mjINT_IMPLICITFAST)
    data = mj.MjData(model)
    scratch = mj.MjData(model)

    roll_des_cst, pitch_des_cst = roll_pitch_des_from_orientation_config(
        cfg["orientation"], model, scratch, qj0, list(j_ord)
    )

    qlj, qhj, qla, qha = joint_actuator_bounds(model, j_ord, a_ord)

    for i, nm in enumerate(a_ord):
        data.qpos[int(model.jnt_qposadr[joint_id(model, nm)])] = qa0[i]
    apply_ideal_qjnt_equals_ratio_qact(model, data, j_ord, a_ord, ratios)
    mj.mj_forward(model, data)

    yaml_pf = PKG_ROOT / "configs" / "path_tracking.yaml"
    print(f"[path_tracking] {yaml_pf.resolve()}")
    print(f"[path_tracking] initial_actuator_rad ({','.join(a_ord)}) = {qa0}")
    print(f"[path_tracking] q_joint_rad ({','.join(j_ord)}) = {qj0}")

    dof_j = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in j_ord])
    dof_a = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in a_ord])

    wps_raw = cfg["waypoints"]
    spline = CartesianQuinticPath(
        [WaypointXYZ(float(r["t"]), float(r["x"]), float(r["y"]), float(r["z"])) for r in wps_raw]
    )

    t_ov = np.linspace(0.0, float(dur), overlay_npts, dtype=np.float64)
    path_xyz_overlay = np.array([spline.sample(float(t))[0] for t in t_ov], dtype=np.float64)

    n = int(round(dur / dt)) + 1
    ts = np.array([min(i * dt, dur) for i in range(n)], dtype=np.float64)

    q_des = np.zeros((n, 4))
    p_des_xyz = np.zeros((n, 3))
    q_seed = qj0.copy()
    for k, t_k in enumerate(ts):
        p_k, _v, _a = spline.sample(float(t_k))
        p_des_xyz[k] = p_k
        if float(t_k) <= hold_initial_pose_sec + 1e-12:
            qjk, _qa_unused = clamp_chain(qj0, ratios, qlj, qhj, qla, qha)
        else:
            q_sol, _diag = solve_ik(
                model,
                scratch,
                p_k,
                roll_des=roll_des_cst,
                pitch_des=pitch_des_cst,
                ik=ik_cfg,
                q_seed=q_seed,
                bounds_lo=qlj,
                bounds_hi=qhj,
                multistart_random_trials=ms_trials,
                multistart_if_geom_above=ms_thr,
                rng=ik_rng,
            )
            qjk, _qa_unused = clamp_chain(q_sol, ratios, qlj, qhj, qla, qha)
        q_des[k] = qjk
        q_seed = qjk.copy()

    q_dot_des = central_difference_joint_series(q_des, dt)
    if smooth_n > 0:
        q_dot_des = moving_average_columns(q_dot_des, smooth_n)

    roll_des_arr = np.full(n, roll_des_cst)
    pitch_des_arr = np.full(n, pitch_des_cst)

    q_act_logs = np.zeros((n, 4))
    q_joint_logs = np.zeros((n, 4))
    qdj_act_logs = np.zeros((n, 4))
    p_ee_logs = np.zeros((n, 3))
    roll_logs = np.zeros(n)
    pitch_logs = np.zeros(n)
    yaw_logs = np.zeros(n)
    tau_joint_logs = np.zeros((n, 4))
    tau_act_logs = np.zeros((n, 4))
    sat_logs = np.zeros((n, 4), dtype=bool)
    ee_err = np.zeros(n)

    sat_ev = 0
    viewer_stop_k: int | None = None

    with passive_mujoco_viewer(model, data, passive_viewer) as viewer:
        for k, _ in enumerate(ts):
            qj = read_q(model, data, j_ord)
            vj = read_v(model, data, j_ord)

            tau_j, sat = vsd.compute(qj, vj, q_des[k], q_dot_des[k])
            tau_act = actuator_torques_from_joint(tau_j, ratios)

            if np.any(sat):
                sat_ev += 1

            data.qfrc_applied[:] = 0.0
            data.ctrl[:] = 0.0

            if mode == "actuator":
                for i in range(4):
                    data.qfrc_applied[dof_a[i]] += tau_act[i]
            elif mode == "joint_direct_debug":
                for i in range(4):
                    data.qfrc_applied[dof_j[i]] += tau_j[i]
            else:
                raise ValueError(f"unknown torque_application_mode: {mode}")

            mj.mj_step(model, data)

            if mode == "actuator":
                apply_ideal_qjnt_equals_ratio_qact(model, data, j_ord, a_ord, ratios)
            else:
                apply_ideal_qact_from_joint(model, data, j_ord, a_ord, ratios)

            q_act_logs[k] = read_q(model, data, a_ord)
            q_joint_logs[k] = read_q(model, data, j_ord)
            qdj_act_logs[k] = read_v(model, data, j_ord)

            p_ee, ro, pit, yaw = fk_ee_rp(model, scratch, q_joint_logs[k], j_ord)
            p_ee_logs[k] = p_ee
            roll_logs[k] = ro
            pitch_logs[k] = pit
            yaw_logs[k] = yaw
            ee_err[k] = float(np.linalg.norm(p_des_xyz[k] - p_ee))

            tau_joint_logs[k] = tau_j
            tau_act_logs[k] = tau_act
            sat_logs[k] = sat

            if viewer is not None:
                if not viewer.is_running():
                    viewer_stop_k = k
                    break
                if show_ik_overlay:
                    with viewer.lock():
                        populate_ik_path_overlay_geoms(
                            viewer.user_scn,
                            path_xyz_overlay,
                            p_des_xyz[k],
                            p_ee,
                        )
                viewer.sync()
                if vw_scale > 0.0:
                    time.sleep(dt / vw_scale)

    run_len = (viewer_stop_k + 1) if viewer_stop_k is not None else n
    sl = slice(0, run_len)

    jp_err_norm = np.linalg.norm(q_joint_logs[sl] - q_des[sl], axis=-1)
    print("=== Phase B 요약 ===")
    print(f"hold_initial_pose_sec : {hold_initial_pose_sec:.5g} s")
    print(f"torque mode: {mode}")
    print(
        f"orientation roll/pitch : {cfg['orientation'].get('roll_pitch_reference', 'initial_pose')} "
        f"-> roll_des={roll_des_cst:.5g} rad, pitch_des={pitch_des_cst:.5g} rad"
    )
    print(f"passive_viewer : {passive_viewer}  (통계·플롯 구간 {run_len}/{n})")
    if passive_viewer and viewer_stop_k is not None:
        print(f"[viewer] 창 종료 프레임: {run_len} / {n}")

    ee_s = ee_err[sl]

    print(f"RMS EE pos error : {float(np.sqrt(np.mean(ee_s**2))):.5g} m")
    print(f"max EE pos error : {float(np.max(ee_s)):.5g} m")
    print(f"RMS joint pos err norm: {float(np.sqrt(np.mean(jp_err_norm**2))):.5g} rad")
    print(f"max joint pos err norm: {float(np.max(jp_err_norm)):.5g} rad")
    print(f"max |tau_joint| : {float(np.max(np.abs(tau_joint_logs[sl]))):.5g} N·m")
    print(f"max |tau_act|   : {float(np.max(np.abs(tau_act_logs[sl]))):.5g} N·m")
    sat_axis_events = int(np.sum(sat_logs[sl]))
    print(f"VSD steps with ≥1 satu joint : {sat_ev}")
    print(f"토크 포화 이벤트(축·시간 합): {sat_axis_events}")
    print(f"max |roll_err| : {float(np.max(np.abs([angle_error(roll_des_cst, r) for r in roll_logs[sl]]))):.5g} rad")

    print(f"max |pitch_err|: {float(np.max(np.abs([angle_error(pitch_des_cst, p) for p in pitch_logs[sl]]))):.5g} rad")

    chk = float(np.max(np.abs(tau_act_logs[sl] - (ratios * tau_joint_logs[sl]))))

    print(f"토크 매핑 수치 검증 max|tau_act - ratio*tau_joint|: {chk:.3e}")

    if plots:
        outp = PKG_ROOT / "figures" / "phaseB_ik_vsd"
        outp.parent.mkdir(parents=True, exist_ok=True)
        plot_phase_b_vsd_tracking(
            ts[sl],
            p_des=p_des_xyz[sl],
            p_act=p_ee_logs[sl],
            q_j_des=q_des[sl],
            q_j_act=q_joint_logs[sl],
            qdj_des=q_dot_des[sl],
            qdj_act=qdj_act_logs[sl],
            roll_des=roll_des_arr[sl],
            roll_act=roll_logs[sl],
            pitch_des=pitch_des_arr[sl],
            pitch_act=pitch_logs[sl],
            yaw_act=yaw_logs[sl],
            tau_j=tau_joint_logs[sl],
            tau_a=tau_act_logs[sl],
            saturated=sat_logs[sl],
            ee_err_norm=ee_err[sl],
            base_path=str(outp),
        )
        print(f"figures 접두어: {outp}")


if __name__ == "__main__":
    main()
