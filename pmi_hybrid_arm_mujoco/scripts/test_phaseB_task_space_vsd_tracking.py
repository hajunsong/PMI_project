#!/usr/bin/env python3
"""
Phase B: 작업 공간 VSD 궤적 추종 (IK 없음).

과제 모드: ``xyz`` / ``xyz_pitch`` / ``xyz_roll_pitch``
τ_total = τ_bias(qfrc_bias) + JᵀF (선택) + 관절 한계 게이트 + 토크 클립.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import mujoco as mj
import numpy as np

from controllers.task_space_vsd_controller import TaskSpaceVSDController, TaskSpaceVSDParams
from kinematics.forward_kinematics import fk_ee_rp
from kinematics.orientation_utils import angle_error
from kinematics.task_jacobian import TaskJacobianMode, compute_task_jacobian_mode, fk_task_y, task_dim
from kinematics.trajectory import CartesianQuinticPath, WaypointXYZ
from utils.logging_utils import (
    jac_xyz_numeric_vs_analytic_report,
    torque_scaling_report,
    virtual_work_slack_abs,
)
from utils.mujoco_helpers import (
    DEFAULT_MODEL_PATH,
    PKG_ROOT,
    VSD_DEBUG_MODEL_PATH,
    apply_ideal_qact_from_joint,
    apply_ideal_qjnt_equals_ratio_qact,
    joint_id,
    load_mjmodel,
    passive_mujoco_viewer,
    populate_ik_path_overlay_geoms,
)
from utils.path_tracking_io import (
    gains_limits_task_space_vsd,
    joint_actuator_bounds,
    load_task_space_vsd_yaml,
    ordered_transmission_arrays,
    roll_pitch_des_from_orientation_config,
)
from utils.plotting import plot_phase_b_task_space_vsd


def read_q(model: mj.MjModel, data: mj.MjData, names: list[str]) -> np.ndarray:
    return np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in names])


def read_v(model: mj.MjModel, data: mj.MjData, names: list[str]) -> np.ndarray:
    return np.array([float(data.qvel[int(model.jnt_dofadr[joint_id(model, n)])]) for n in names])


def pad5_task(vec: np.ndarray, m: int) -> np.ndarray:
    out = np.zeros(5, dtype=np.float64)
    out[:m] = np.asarray(vec, dtype=np.float64).reshape(m)
    return out


def pad5_bool(mask: np.ndarray, m: int) -> np.ndarray:
    out = np.zeros(5, dtype=bool)
    out[:m] = np.asarray(mask, dtype=bool).reshape(m)
    return out


def build_y_desired(
    task_mode: TaskJacobianMode,
    *,
    hold: bool,
    p_hold: np.ndarray,
    p_spline: np.ndarray | None,
    pv_spline: np.ndarray | None,
    roll_cst: float,
    pitch_cst: float,
) -> tuple[np.ndarray, np.ndarray]:
    if hold:
        p_d = np.asarray(p_hold, dtype=np.float64).reshape(3)
        pv = np.zeros(3)
    else:
        p_d = np.asarray(p_spline, dtype=np.float64).reshape(3)
        pv = np.asarray(pv_spline, dtype=np.float64).reshape(3)

    if task_mode == "xyz":
        return p_d.copy(), pv.copy()
    if task_mode == "xyz_pitch":
        return np.concatenate([p_d, [pitch_cst]]), np.concatenate([pv, [0.0]])
    return np.concatenate([p_d, [roll_cst, pitch_cst]]), np.concatenate([pv, [0.0, 0.0]])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=None)
    args = parser.parse_args()
    cfg_path = Path(args.config) if args.config is not None else PKG_ROOT / "configs" / "task_space_vsd.yaml"
    cfg = load_task_space_vsd_yaml(args.config)

    sim = cfg["simulation"]
    dt = float(sim["dt"])
    dur = float(sim["duration"])
    rng_seed = int(sim.get("rng_seed", 0))
    rng = np.random.default_rng(rng_seed)
    passive_viewer = bool(sim.get("passive_viewer", False))
    vw_scale = float(sim.get("viewer_realtime_scale", 1.0))
    hold_initial_pose_sec = float(sim.get("hold_initial_pose_sec", 0.0))
    save_plots = bool(sim.get("save_plots", True))
    validate_contracts = bool(sim.get("validate_contracts", False))
    overlay_on = passive_viewer and bool(sim.get("show_ik_path_overlay", True))
    overlay_n = max(2, int(sim.get("ik_path_overlay_samples", 128)))

    mjcf_key = sim.get("mjcf") or sim.get("mjcf_path")
    if mjcf_key:
        model_path = Path(mjcf_key)
        if not model_path.is_absolute():
            model_path = PKG_ROOT / model_path
    elif cfg_path.name == "task_space_vsd_debug.yaml":
        model_path = VSD_DEBUG_MODEL_PATH
    else:
        model_path = DEFAULT_MODEL_PATH

    tsv = cfg["task_space_vsd"]
    jc = str(tsv["jacobian"]["mode"]).strip().lower()
    if jc not in ("numerical", "mujoco_analytic"):
        raise ValueError(f"unknown jacobian mode: {jc}")
    j_eps = float(tsv["jacobian"].get("epsilon", 1e-6))
    jac_flip_sign = float(tsv["jacobian"].get("flip_sign", 1.0))
    torque_mode = str(tsv["torque_application_mode"]).strip()

    task_mode, g_k, g_d, lf, tj_lim, ta_lim = gains_limits_task_space_vsd(tsv)
    use_bias = bool(tsv.get("use_bias_compensation", False))

    jl = tsv.get("joint_limit", {})
    jl_enabled = bool(jl.get("enabled", False))
    jl_margin = float(jl.get("margin", 0.05))

    j_ord, a_ord, ratios = ordered_transmission_arrays(cfg)
    ratios_f = ratios.astype(np.float64)
    path_y = cfg["path"]
    qa0 = np.array(path_y["initial_actuator_rad"], dtype=np.float64)

    wps_raw = path_y["waypoints"]
    spline = CartesianQuinticPath(
        [WaypointXYZ(float(r["t"]), float(r["x"]), float(r["y"]), float(r["z"])) for r in wps_raw]
    )
    n = int(round(dur / dt)) + 1
    ts = np.array([min(i * dt, dur) for i in range(n)], dtype=np.float64)

    model = load_mjmodel(model_path, strip_position_actuators=True)
    model.opt.timestep = dt
    model.opt.integrator = int(mj.mjtIntegrator.mjINT_IMPLICITFAST)
    data = mj.MjData(model)
    scratch_pose = mj.MjData(model)
    jac_scratch = mj.MjData(model)

    for i, nm in enumerate(a_ord):
        data.qpos[int(model.jnt_qposadr[joint_id(model, nm)])] = qa0[i]
    apply_ideal_qjnt_equals_ratio_qact(model, data, j_ord, a_ord, ratios_f)
    mj.mj_forward(model, data)

    qj_init = read_q(model, data, j_ord)
    ori_cfg = tsv["desired_orientation"]
    roll_des_cst, pitch_des_cst = roll_pitch_des_from_orientation_config(
        ori_cfg, model, scratch_pose, qj_init, j_ord
    )
    p0, _, _, _ = fk_ee_rp(model, scratch_pose, qj_init, j_ord)
    y_hold_xyz = np.asarray(p0, dtype=np.float64).reshape(3)

    dof_j = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in j_ord])
    dof_a = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in a_ord])

    qlj, qhj, _qla, _qha = joint_actuator_bounds(model, j_ord, a_ord)

    vsd = TaskSpaceVSDController(
        TaskSpaceVSDParams(
            task_mode=task_mode,
            K_task=g_k,
            D_task=g_d,
            F_task_limit=lf,
            tau_jnt_limit=tj_lim,
            tau_act_limit=ta_lim,
            jac_flip_sign=jac_flip_sign,
            use_bias_compensation=use_bias,
            use_desired_velocity=bool(tsv.get("use_desired_velocity", True)),
            joint_limit_enabled=jl_enabled,
            joint_limit_margin=jl_margin,
            q_joint_min=qlj,
            q_joint_max=qhj,
        )
    )

    m = task_dim(task_mode)

    t_ov = np.linspace(0.0, float(dur), overlay_n, dtype=np.float64)
    path_xyz_overlay = np.array([spline.sample(float(t))[0] for t in t_ov])

    roll_des_arr = np.full(n, roll_des_cst)
    pitch_des_arr = np.full(n, pitch_des_cst)

    p_des_xyz = np.zeros((n, 3))
    p_act_log = np.zeros((n, 3))
    y_des_log = np.zeros((n, 5))
    y_act_log = np.zeros((n, 5))
    ydot_des_log = np.zeros((n, 5))
    roll_act_log = np.zeros(n)
    pitch_act_log = np.zeros(n)
    yaw_act_log = np.zeros(n)
    F_task_log = np.zeros((n, 5))
    e_task_norm = np.zeros(n)
    ed_task_norm = np.zeros(n)
    vw_slack_sample = np.zeros(n)
    tau_j_log = np.zeros((n, 4))
    tau_bias_log = np.zeros((n, 4))
    tau_task_log = np.zeros((n, 4))
    tau_a_log = np.zeros((n, 4))
    q_joint_log = np.zeros((n, 4))
    q_act_log = np.zeros((n, 4))
    ee_err_norm_log = np.zeros(n)
    jl_active_log = np.zeros((n, 4), dtype=bool)
    F_sat_log = np.zeros((n, 5), dtype=bool)
    tj_sat_log = np.zeros((n, 4), dtype=bool)
    ta_sat_log = np.zeros((n, 4), dtype=bool)
    joint_margin_log = np.zeros(n)

    if validate_contracts:
        mj.mj_forward(model, data)
        print("[validate] Jacobian xyz 줄:", jac_xyz_numeric_vs_analytic_report(model, data, jac_scratch, j_ord, epsilon=j_eps))
        jac_scratch.qpos[:] = data.qpos
        jac_scratch.qvel[:] = data.qvel
        mj.mj_forward(model, jac_scratch)
        Jh = float(jac_flip_sign) * compute_task_jacobian_mode(
            model, jac_scratch, joint_names=j_ord, task_mode=task_mode, ee_site_name="end_effector", mode=jc, epsilon=j_eps
        )
        qd = rng.normal(scale=5e-3, size=4)
        yd_a = Jh @ qd
        F_t = rng.normal(scale=1.0, size=m)
        tau_fake = Jh.T @ F_t
        vs = virtual_work_slack_abs(F_t, yd_a, tau_fake, qd)
        print(f"[validate] sample virtual-work slack |pw| = {vs:.3e}")

    nan_abort = False
    viewer_stop_k: int | None = None
    sat_joint_steps = 0
    sat_act_steps = 0
    lim_viol_joint = int(0)
    last_logged = -1

    with passive_mujoco_viewer(model, data, passive_viewer) as viewer:
        for k, t_k in enumerate(ts):
            qj_now = read_q(model, data, j_ord)
            vj_now = read_v(model, data, j_ord)

            hold = float(t_k) <= hold_initial_pose_sec + 1e-15
            if hold:
                y_des, ydot_des = build_y_desired(
                    task_mode, hold=True, p_hold=y_hold_xyz, p_spline=None, pv_spline=None, roll_cst=roll_des_cst, pitch_cst=pitch_des_cst
                )
            else:
                p_vec, pv, _pa = spline.sample(float(t_k))
                y_des, ydot_des = build_y_desired(
                    task_mode,
                    hold=False,
                    p_hold=y_hold_xyz,
                    p_spline=np.asarray(p_vec),
                    pv_spline=np.asarray(pv),
                    roll_cst=roll_des_cst,
                    pitch_cst=pitch_des_cst,
                )

            mj.mj_forward(model, data)
            tau_bias_joint = np.array([float(data.qfrc_bias[int(dof_j[i])]) for i in range(4)], dtype=np.float64)

            y_actual = fk_task_y(model, scratch_pose, qj_now, j_ord, task_mode)

            jac_scratch.qpos[:] = data.qpos
            jac_scratch.qvel[:] = data.qvel
            mj.mj_forward(model, jac_scratch)
            J_task = compute_task_jacobian_mode(
                model,
                jac_scratch,
                joint_names=j_ord,
                task_mode=task_mode,
                ee_site_name="end_effector",
                mode=jc,
                epsilon=j_eps,
            )

            out = vsd.compute(
                y_des=y_des,
                ydot_des=ydot_des,
                y_actual=y_actual,
                q_joint=qj_now,
                qdot_joint=vj_now,
                J_task=J_task,
                ratios_actuator=ratios_f,
                tau_bias_joint=tau_bias_joint if use_bias else None,
            )
            tau_j = np.asarray(out["tau_joint"]).reshape(4)
            tau_a = np.asarray(out["tau_act"]).reshape(4)
            tau_task_only = np.asarray(out["tau_task_jnt"]).reshape(4)

            vw_slack_sample[k] = virtual_work_slack_abs(
                out["F_task"],
                out["ydot_actual"],
                tau_task_only,
                vj_now.reshape(4),
            )

            chk_map = torque_scaling_report(tau_j, tau_a, ratios_f)
            if k == 0 and validate_contracts:
                print("[validate]", chk_map)

            if not np.all(np.isfinite(tau_j)) or not np.all(np.isfinite(tau_a)):
                nan_abort = True
                print(f"[abort] 비정상값 k={k} tau_j={tau_j} tau_act={tau_a}")
                viewer_stop_k = k
                break

            marg = np.min(np.minimum(qhj - qj_now, qj_now - qlj))

            joint_margin_log[k] = float(marg)

            flag_any_j = np.any(qj_now < qlj - 1e-6) or np.any(qj_now > qhj + 1e-6)
            lim_viol_joint += int(flag_any_j)

            data.qfrc_applied[:] = 0.0
            data.ctrl[:] = 0.0

            if torque_mode == "actuator":
                for i in range(4):
                    data.qfrc_applied[dof_a[i]] += tau_a[i]
            elif torque_mode == "joint_direct_debug":
                for i in range(4):
                    data.qfrc_applied[dof_j[i]] += tau_j[i]
            else:
                raise ValueError(f"unknown torque_application_mode={torque_mode}")

            mj.mj_step(model, data)

            if torque_mode == "actuator":
                apply_ideal_qjnt_equals_ratio_qact(model, data, j_ord, a_ord, ratios_f)
            else:
                apply_ideal_qact_from_joint(model, data, j_ord, a_ord, ratios_f)

            mj.mj_forward(model, data)
            q_after = read_q(model, data, j_ord)
            p_act_k, rk, pk, yawk = fk_ee_rp(model, scratch_pose, q_after, j_ord)
            y_full = fk_task_y(model, scratch_pose, q_after, j_ord, task_mode="xyz_roll_pitch")
            ee_err = float(np.linalg.norm(y_des[:3] - p_act_k))

            p_des_xyz[k] = y_des[:3]
            y_des_log[k] = pad5_task(y_des, m)
            p_act_log[k] = np.asarray(p_act_k, dtype=np.float64).copy()
            y_act_log[k] = pad5_task(y_full, 5)
            ydot_des_log[k] = pad5_task(ydot_des, m)

            roll_act_log[k] = float(rk)
            pitch_act_log[k] = float(pk)
            yaw_act_log[k] = float(yawk)

            ek = np.asarray(out["e_task"]).reshape(-1)
            dk = np.asarray(out["edot_task"]).reshape(-1)
            e_task_norm[k] = float(np.linalg.norm(ek))
            ed_task_norm[k] = float(np.linalg.norm(dk))
            F_task_log[k] = pad5_task(np.asarray(out["F_task"]), m)
            F_sat_log[k] = pad5_bool(np.asarray(out["F_saturated_axes"]), m)
            jl_active_log[k] = np.asarray(out["joint_limit_active"], dtype=bool).reshape(4)
            tj_sat_log[k] = np.asarray(out["tau_joint_saturated_axes"]).astype(bool).reshape(4)
            ta_sat_log[k] = np.asarray(out["tau_act_saturated_axes"]).astype(bool).reshape(4)
            tau_j_log[k] = tau_j.reshape(4)
            tau_bias_log[k] = np.asarray(out["tau_bias_jnt"]).reshape(4)
            tau_task_log[k] = tau_task_only.reshape(4)
            tau_a_log[k] = tau_a.reshape(4)
            ee_err_norm_log[k] = ee_err
            q_joint_log[k] = q_after.copy()
            q_act_log[k] = read_q(model, data, a_ord)

            if np.any(np.asarray(out["tau_joint_saturated_axes"])):
                sat_joint_steps += 1
            if np.any(np.asarray(out["tau_act_saturated_axes"])):
                sat_act_steps += 1

            last_logged = k

            if viewer is not None:
                if not viewer.is_running():
                    viewer_stop_k = k
                    break
                if overlay_on:
                    with viewer.lock():
                        populate_ik_path_overlay_geoms(
                            viewer.user_scn,
                            path_xyz_overlay,
                            np.asarray(y_des[:3]).reshape(-1),
                            np.asarray(p_act_k).reshape(-1),
                        )
                viewer.sync()
                if vw_scale > 0.0:
                    time.sleep(dt / vw_scale)

    run_len = last_logged + 1 if last_logged >= 0 else 0
    sl = slice(0, run_len)
    if run_len == 0:
        print("[warn] 유효 스텝이 없어 요약·플롯을 생략합니다.")
        return

    e_r = np.array([angle_error(roll_des_cst, r) for r in roll_act_log[sl]])
    e_pr = np.array([angle_error(pitch_des_cst, p) for p in pitch_act_log[sl]])
    ee_n = ee_err_norm_log[sl]
    jl_count = int(np.sum(np.any(jl_active_log[sl], axis=1)))

    print("=== Phase B 작업 공간 VSD 요약 ===")
    print(f"설정 파일: {cfg_path.resolve()}")
    print(f"MJCF: {model_path}")
    print(f"task_mode = {task_mode}, jacobian_mode = {jc}, torque_application_mode = {torque_mode}")
    print(f"use_bias_compensation = {use_bias}, joint_limit = {jl_enabled} (margin {jl_margin} rad)")
    print(f"orientation roll_pitch_reference : {ori_cfg.get('roll_pitch_reference', 'initial_pose')}")
    print(f"  → roll_des={roll_des_cst:.5g} rad, pitch_des={pitch_des_cst:.5g} rad")
    print(f"통계 길이: {run_len} / {n} steps")
    print(f"RMS EE 위치 오차: {float(np.sqrt(np.mean(ee_n**2))):.5g} m")
    print(f"max EE 위치 오차: {float(np.max(ee_n)):.5g} m")
    print(f"RMS roll err: {float(np.sqrt(np.mean(e_r**2))):.5g} rad")
    print(f"RMS pitch err: {float(np.sqrt(np.mean(e_pr**2))):.5g} rad")
    print(f"관절 한계 토크 게이트 활성 스텝 수: {jl_count}")
    print(f"max |tau_joint|: {float(np.max(np.abs(tau_j_log[sl]))):.5g} N·m")
    print(f"VSD 포화 포함 스텝 (joint/act): {sat_joint_steps}/{sat_act_steps}")
    print(f"관절 한계 초과 검출 프레임(근사): {lim_viol_joint}")
    chk = torque_scaling_report(tau_j_log[sl], tau_a_log[sl], ratios_f)
    print(chk)

    if save_plots and run_len > 1:
        outd = PKG_ROOT / "figures" / "phaseB_task_space_vsd"
        plot_phase_b_task_space_vsd(
            ts[sl],
            p_des=p_des_xyz[sl],
            p_act=p_act_log[sl],
            roll_des=roll_des_arr[sl],
            pitch_des=pitch_des_arr[sl],
            roll_act=roll_act_log[sl],
            pitch_act=pitch_act_log[sl],
            yaw_act=yaw_act_log[sl],
            F_task=F_task_log[sl],
            e_task_norm=e_task_norm[sl],
            ed_task_norm=ed_task_norm[sl],
            tau_j=tau_j_log[sl],
            tau_a=tau_a_log[sl],
            q_joint=q_joint_log[sl],
            q_actuator=q_act_log[sl],
            ee_pos_err_norm=ee_err_norm_log[sl],
            F_saturated_axes=F_sat_log[sl],
            tau_j_saturated=tj_sat_log[sl],
            tau_a_saturated=ta_sat_log[sl],
            base_path=str(outd),
        )
        print(f"그림 디렉토리 접두어: {outd}")


if __name__ == "__main__":
    main()
