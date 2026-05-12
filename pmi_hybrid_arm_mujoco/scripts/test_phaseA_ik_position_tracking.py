#!/usr/bin/env python3
"""
Phase A: quintic Cartesian path -> weighted numeric IK -> q_act 위치 추종(q_j ≈ ratio * q_act 로 동기화).
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

from kinematics.forward_kinematics import fk_ee_rp
from kinematics.inverse_kinematics import IKConfig, solve_ik
from kinematics.orientation_utils import angle_error
from kinematics.trajectory import CartesianQuinticPath, WaypointXYZ
from utils.mujoco_helpers import (
    DEFAULT_MODEL_PATH,
    PKG_ROOT,
    apply_ideal_qjnt_equals_ratio_qact,
    compile_model_with_custom_position_actuators,
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
from utils.plotting import plot_phase_a_path_tracking


def read_q(model: mj.MjModel, data: mj.MjData, names: list[str]) -> np.ndarray:
    q = []
    for n in names:
        adr = int(model.jnt_qposadr[joint_id(model, n)])
        q.append(float(data.qpos[adr]))
    return np.array(q, dtype=np.float64)


def clamp_actuator_joint_pair(
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
    qj_final = joint_from_actuator_positions(qa_out, ratios)
    return qj_final, qa_out


def main() -> None:
    cfg = load_path_tracking_yaml()
    sc = cfg["simulation"]
    dt = float(sc["dt"])
    dur = float(sc["duration"])
    save_plots = bool(sc.get("save_plots", True))
    passive_viewer = bool(sc.get("passive_viewer", False))
    vw_scale = float(sc.get("viewer_realtime_scale", 1.0))
    hold_initial_pose_sec = float(sc.get("hold_initial_pose_sec", 0.0))
    show_ik_overlay = passive_viewer and bool(sc.get("show_ik_path_overlay", True))
    overlay_npts = max(2, int(sc.get("ik_path_overlay_samples", 128)))

    j_order, a_order, ratios = ordered_transmission_arrays(cfg)
    qa0 = np.array(cfg["initial_actuator_rad"], dtype=np.float64)
    qj0 = joint_from_actuator_positions(qa0, ratios)

    ik_y = cfg["ik"]
    ik_rng = np.random.default_rng(int(sc.get("rng_seed", 0)))
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
        joint_side_order=tuple(j_order),
    )

    pmode = cfg["phaseA_position_control"]
    act_cmd_mode = str(pmode.get("actuator_command_mode", "kinematic")).strip().lower()
    kp = [float(x) for x in pmode["actuator_position_kp"]]
    kd = [float(x) for x in pmode["actuator_position_kd"]]
    if act_cmd_mode == "kinematic":
        model = load_mjmodel(DEFAULT_MODEL_PATH, strip_position_actuators=True)
    elif act_cmd_mode == "servo":
        model = compile_model_with_custom_position_actuators(DEFAULT_MODEL_PATH, kp, kd)
        model.opt.integrator = int(mj.mjtIntegrator.mjINT_IMPLICITFAST)
    else:
        raise ValueError(f"unknown phaseA actuator_command_mode: {act_cmd_mode}")
    model.opt.timestep = dt

    data = mj.MjData(model)
    scratch = mj.MjData(model)

    roll_des_cst, pitch_des_cst = roll_pitch_des_from_orientation_config(
        cfg["orientation"], model, scratch, qj0, list(j_order)
    )

    qlj, qhj, qla, qha = joint_actuator_bounds(model, j_order, a_order)

    # 초기 pose
    for i, nm in enumerate(a_order):
        data.qpos[int(model.jnt_qposadr[joint_id(model, nm)])] = qa0[i]
    apply_ideal_qjnt_equals_ratio_qact(model, data, j_order, a_order, ratios)
    mj.mj_forward(model, data)

    yaml_pf = PKG_ROOT / "configs" / "path_tracking.yaml"
    print(f"[path_tracking] {yaml_pf.resolve()}")
    print(f"[path_tracking] initial_actuator_rad ({','.join(a_order)}) = {qa0}")
    print(f"[path_tracking] q_joint_rad ({','.join(j_order)}) = {qj0}")

    if hold_initial_pose_sec > 0.0:
        print(
            f"[path_tracking] hold_initial_pose_sec = {hold_initial_pose_sec:.5g}s "
            f"(구간에서는 IK 미적용)"
        )

    wps_raw = cfg["waypoints"]
    spline = CartesianQuinticPath(
        [WaypointXYZ(float(r["t"]), float(r["x"]), float(r["y"]), float(r["z"])) for r in wps_raw]
    )

    t_ov = np.linspace(0.0, float(dur), overlay_npts, dtype=np.float64)
    path_xyz_overlay = np.array([spline.sample(float(t))[0] for t in t_ov], dtype=np.float64)

    n = int(round(dur / dt)) + 1
    ts = np.array([min(i * dt, dur) for i in range(n)], dtype=np.float64)

    p_des = np.zeros((n, 3))
    p_act = np.zeros((n, 3))
    roll_des_arr = np.full(n, roll_des_cst)
    pitch_des_arr = np.full(n, pitch_des_cst)
    yaw_act = np.zeros(n)
    roll_act = np.zeros(n)
    pitch_act = np.zeros(n)
    q_j_des = np.zeros((n, 4))
    q_j_act = np.zeros((n, 4))
    q_a_des = np.zeros((n, 4))
    q_a_act = np.zeros((n, 4))
    ik_norm = np.zeros(n)
    ee_err = np.zeros(n)

    q_seed = qj0.copy()
    clip_warn = 0
    viewer_stop_k: int | None = None

    with passive_mujoco_viewer(model, data, passive_viewer) as viewer:
        for k, t_k in enumerate(ts):
            pos_d, _v, _a = spline.sample(float(t_k))
            p_des[k] = pos_d

            if float(t_k) <= hold_initial_pose_sec + 1e-12:
                ik_norm[k] = 0.0
                qj_cmd, qa_cmd = clamp_actuator_joint_pair(qj0, ratios, qlj, qhj, qla, qha)
                if np.max(np.abs(qj0 - qj_cmd)) > 1e-7 or np.max(
                    np.abs(actuator_from_joint_positions(qj0, ratios) - qa_cmd)
                ) > 1e-7:
                    clip_warn += 1
            else:
                q_sol, diag = solve_ik(
                    model,
                    scratch,
                    pos_d,
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
                ik_norm[k] = float(diag["ik_norm_geom"])

                qj_cmd, qa_cmd = clamp_actuator_joint_pair(q_sol, ratios, qlj, qhj, qla, qha)
                if np.max(np.abs(q_sol - qj_cmd)) > 1e-7 or np.max(
                    np.abs(actuator_from_joint_positions(q_sol, ratios) - qa_cmd)
                ) > 1e-7:
                    clip_warn += 1

            q_j_des[k] = qj_cmd
            q_a_des[k] = qa_cmd

            if act_cmd_mode == "kinematic":
                for i, nm in enumerate(a_order):
                    aid = joint_id(model, nm)
                    va = int(model.jnt_dofadr[aid])
                    qa = int(model.jnt_qposadr[aid])
                    data.qpos[qa] = float(qa_cmd[i])
                    data.qvel[va] = 0.0
                apply_ideal_qjnt_equals_ratio_qact(model, data, j_order, a_order, ratios)
                mj.mj_forward(model, data)
            else:
                data.ctrl[:] = qa_cmd
                mj.mj_step(model, data)
                apply_ideal_qjnt_equals_ratio_qact(model, data, j_order, a_order, ratios)

            q_a_act[k] = read_q(model, data, a_order)
            q_j_act[k] = read_q(model, data, j_order)

            p_ee, rol, pit, yaw = fk_ee_rp(model, scratch, q_j_act[k], j_order)
            p_act[k] = p_ee
            roll_act[k] = rol
            pitch_act[k] = pit
            yaw_act[k] = yaw

            ee_err[k] = float(np.linalg.norm(pos_d - p_ee))
            q_seed = qj_cmd.copy()

            if viewer is not None:
                if not viewer.is_running():
                    viewer_stop_k = k
                    break
                if show_ik_overlay:
                    with viewer.lock():
                        populate_ik_path_overlay_geoms(
                            viewer.user_scn,
                            path_xyz_overlay,
                            pos_d,
                            p_ee,
                        )
                viewer.sync()
                if vw_scale > 0.0:
                    time.sleep(dt / vw_scale)

    run_len = (viewer_stop_k + 1) if viewer_stop_k is not None else n
    sl = slice(0, run_len)

    trans_err = np.max(np.abs(q_j_des[sl] - ratios * q_a_des[sl]))
    trans_err_act = np.max(np.abs(q_j_act[sl] - ratios * q_a_act[sl]))

    ee_err_all = np.linalg.norm(p_des[sl] - p_act[sl], axis=-1)
    print("=== Phase A 요약 ===")
    print(f"hold_initial_pose_sec : {hold_initial_pose_sec:.5g} s")
    print(
        f"orientation roll/pitch : {cfg['orientation'].get('roll_pitch_reference', 'initial_pose')} "
        f"-> roll_des={roll_des_cst:.5g} rad, pitch_des={pitch_des_cst:.5g} rad"
    )
    print(f"actuator_command_mode : {act_cmd_mode}")
    print(f"passive_viewer : {passive_viewer}  (통계·플롯 구간 {run_len}/{n})")
    if passive_viewer and viewer_stop_k is not None:
        print(f"[viewer] 창 종료 프레임: {run_len} / {n}")
    print(f"RMS EE position error : {float(np.sqrt(np.mean(ee_err_all**2))):.5g} m")
    print(f"max EE position error : {float(np.max(ee_err_all)):.5g} m")
    print(f"max |roll_err|       : {float(np.max(np.abs([angle_error(roll_des_cst, r) for r in roll_act[sl]]))):.5g} rad")

    print(
        f"max |pitch_err|       : "
        f"{float(np.max(np.abs([angle_error(pitch_des_cst, p) for p in pitch_act[sl]]))):.5g} rad"
    )

    ik_seg = ik_norm[sl]

    print(
        f"IK geometric residual max/RMS : "
        f"{float(np.max(ik_seg)):.5g} / {float(np.sqrt(np.mean(np.square(ik_seg)))):.5g}"
    )
    print(f"max abs q_act_des   : {float(np.max(np.abs(q_a_des[sl]))):.5g} rad")
    print(f"min joint margin(des): {float(np.min(np.minimum(qhj - q_j_des[sl], q_j_des[sl] - qlj))):.5g} rad")
    print(f"전달 일치 |q_des_j - ratio*q_des_a|(max): {trans_err:.5g}")
    print(f"실제 |q_act - q_j/ratio|(max): {trans_err_act:.5g}")
    print(f"한계 클리핑 적용 프레임 수(근사): {clip_warn}")

    if save_plots:
        out = PKG_ROOT / "figures" / "phaseA_ik_tracking"
        out.parent.mkdir(parents=True, exist_ok=True)
        plot_phase_a_path_tracking(
            ts[sl],
            p_des[sl],
            p_act[sl],
            roll_des_arr[sl],
            pitch_des_arr[sl],
            yaw_act[sl],
            roll_act[sl],
            pitch_act[sl],
            q_j_des[sl],
            q_j_act[sl],
            q_a_des[sl],
            q_a_act[sl],
            ik_norm[sl],
            ee_err[sl],
            str(out),
        )
        print(f"그림 접두어: {out}")


if __name__ == "__main__":
    main()
