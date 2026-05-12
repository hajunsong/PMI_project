"""헤드리스 작업 공간 VSD 시뮬 롤아웃 (진단 스크립트 공용)."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import mujoco as mj
import numpy as np

from controllers.task_space_vsd_controller import TaskSpaceVSDController, TaskSpaceVSDParams
from kinematics.forward_kinematics import fk_ee_rp
from kinematics.orientation_utils import angle_error
from kinematics.task_jacobian import TaskJacobianMode, compute_task_jacobian_mode, fk_task_y, task_dim
from kinematics.trajectory import CartesianQuinticPath, WaypointXYZ
from utils.mujoco_helpers import PKG_ROOT, VSD_DEBUG_MODEL_PATH, apply_ideal_qact_from_joint, apply_ideal_qjnt_equals_ratio_qact, joint_id, load_mjmodel


def read_q(model: mj.MjModel, data: mj.MjData, names: list[str]) -> np.ndarray:
    return np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in names])


def read_v(model: mj.MjModel, data: mj.MjData, names: list[str]) -> np.ndarray:
    return np.array([float(data.qvel[int(model.jnt_dofadr[joint_id(model, n)])]) for n in names])


def scale_waypoint_times_to_duration(cfg: dict[str, Any], new_duration: float) -> dict[str, Any]:
    """기하 웨이포인트는 유지하고 ``t`` 만 ``new_duration`` 에 맞게 선형 스케일."""
    c = copy.deepcopy(cfg)
    wps = c["path"]["waypoints"]
    tmax = max(float(w["t"]) for w in wps) if wps else 1.0
    scale = float(new_duration) / tmax if tmax > 1e-12 else 1.0
    for w in wps:
        w["t"] = float(w["t"]) * scale
    c["simulation"]["duration"] = float(new_duration)
    return c


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


def rollout_task_space_vsd(cfg: dict[str, Any], *, extended_log: bool = False) -> dict[str, Any]:
    """과제 속도·토크·F·특이값·마진까지 함께 로그.

    ``extended_log`` 는 이전 버전 호환용(미사용).

    설정 ``task_space_vsd.use_desired_velocity`` 가 False 면 컨트롤러 내에서 과제 속도항을 무시하여
    댐핑만 실제 속도와 스프링항으로 구성된다.
    """
    from utils.path_tracking_io import (
        actuator_from_joint_positions,
        gains_limits_task_space_vsd,
        joint_actuator_bounds,
        ordered_transmission_arrays,
        roll_pitch_des_from_orientation_config,
    )

    sim = cfg["simulation"]
    dt = float(sim["dt"])
    dur = float(sim["duration"])
    hold_initial_pose_sec = float(sim.get("hold_initial_pose_sec", 0.0))

    tsv = cfg["task_space_vsd"]
    jc = str(tsv["jacobian"]["mode"]).strip().lower()
    if jc not in ("numerical", "mujoco_analytic"):
        raise ValueError(f"unknown jacobian mode: {jc}")
    j_eps = float(tsv["jacobian"].get("epsilon", 1e-6))
    jac_flip_sign = float(tsv["jacobian"].get("flip_sign", 1.0))
    torque_mode = str(tsv["torque_application_mode"]).strip()

    task_mode, g_k, g_d, lf, tj_lim, ta_lim = gains_limits_task_space_vsd(tsv)
    use_bias = bool(tsv.get("use_bias_compensation", False))
    use_des_vel = bool(tsv.get("use_desired_velocity", True))
    jl_cfg = tsv.get("joint_limit", {})
    jl_enabled = bool(jl_cfg.get("enabled", False))
    jl_margin = float(jl_cfg.get("margin", 0.05))

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

    model_path = VSD_DEBUG_MODEL_PATH
    if sim.get("mjcf") or sim.get("mjcf_path"):
        mp = Path(sim.get("mjcf") or sim.get("mjcf_path"))
        model_path = mp if mp.is_absolute() else PKG_ROOT / mp

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
    qlj, qhj, qla, qha = joint_actuator_bounds(model, j_ord, a_ord)

    m_dim = task_dim(task_mode)
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
            use_desired_velocity=use_des_vel,
            joint_limit_enabled=jl_enabled,
            joint_limit_margin=jl_margin,
            q_joint_min=qlj,
            q_joint_max=qhj,
        )
    )

    # 로그 버퍼
    p_des_xyz = np.zeros((n, 3))
    p_act_log = np.zeros((n, 3))
    roll_act_log = np.zeros(n)
    pitch_act_log = np.zeros(n)
    e_task_norm = np.zeros(n)
    tau_j_log = np.zeros((n, 4))
    tau_bias_log = np.zeros((n, 4))
    tau_task_log = np.zeros((n, 4))
    tau_total_unc_log = np.zeros((n, 4))
    jl_clip_log = np.zeros((n, 4))
    ee_err_norm_log = np.zeros(n)
    q_joint_log = np.zeros((n, 4))
    qdot_joint_log = np.zeros((n, 4))
    q_act_log_st = np.zeros((n, 4))
    jl_active_log = np.zeros((n, 4), dtype=bool)
    tj_sat_log = np.zeros((n, 4), dtype=bool)

    vnorm_log = np.zeros(n)
    Fnorm_log = np.zeros(n)
    F_task_pad = np.zeros((n, 5))
    ydot_des_pad = np.zeros((n, 5))
    ydot_actual_pad = np.zeros((n, 5))
    edot_task_pad = np.zeros((n, 5))
    edotnorm_log = np.zeros(n)
    sv_min_log = np.zeros(n)
    sv_mat = np.zeros((n, 4))
    J_task_log = np.zeros((n, 5, 4))

    nan_abort = False
    sat_joint_steps = 0
    last_k = -1

    for k, t_k in enumerate(ts):
        qj_now = read_q(model, data, j_ord)
        vj_now = read_v(model, data, j_ord)

        hold = float(t_k) <= hold_initial_pose_sec + 1e-15
        if hold:
            y_des, ydot_des = build_y_desired(
                task_mode,
                hold=True,
                p_hold=y_hold_xyz,
                p_spline=None,
                pv_spline=None,
                roll_cst=roll_des_cst,
                pitch_cst=pitch_des_cst,
            )
        else:
            p_vec, pv, _ = spline.sample(float(t_k))
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

        if not np.all(np.isfinite(tau_j)):
            nan_abort = True
            break

        Fn = float(np.linalg.norm(np.asarray(out["F_task"]).reshape(-1)))

        sv = np.linalg.svd(J_task, compute_uv=False)
        for ii in range(min(4, int(sv.size))):
            sv_mat[k, ii] = float(sv[ii])
        svm = float(np.min(sv)) if sv.size else 0.0

        vnorm_log[k] = float(np.linalg.norm(vj_now))
        Fnorm_log[k] = Fn
        F_task_pad[k, :m_dim] = np.asarray(out["F_task"]).reshape(m_dim).copy()
        ydot_des_pad[k, :m_dim] = np.asarray(ydot_des).reshape(m_dim).copy()
        ydot_actual_pad[k, :m_dim] = np.asarray(out["ydot_actual"]).reshape(m_dim).copy()
        edot_task_pad[k, :m_dim] = np.asarray(out["edot_task"]).reshape(m_dim).copy()
        edotnorm_log[k] = float(np.linalg.norm(np.asarray(out["edot_task"]).reshape(-1)))
        sv_min_log[k] = svm
        J_task_log[k, :m_dim, :] = np.asarray(J_task).reshape(m_dim, 4).copy()

        data.qfrc_applied[:] = 0.0
        data.ctrl[:] = 0.0

        if torque_mode == "actuator":
            for i in range(4):
                data.qfrc_applied[dof_a[i]] += float(out["tau_act"][i])
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
        qa_after = read_q(model, data, a_ord)
        p_act_k, rk, pk, _ywk = fk_ee_rp(model, scratch_pose, q_after, j_ord)

        ek = np.asarray(out["e_task"]).reshape(-1)
        e_task_norm[k] = float(np.linalg.norm(ek))
        p_des_xyz[k] = y_des[:3]
        p_act_log[k] = np.asarray(p_act_k, dtype=np.float64).copy()
        roll_act_log[k] = float(rk)
        pitch_act_log[k] = float(pk)
        tau_j_log[k] = tau_j
        tau_bias_log[k] = np.asarray(out["tau_bias_jnt"]).reshape(4)
        tau_task_log[k] = np.asarray(out["tau_task_jnt"]).reshape(4)
        tau_total_unc_log[k] = np.asarray(out["tau_total_unc"]).reshape(4)
        jl_clip_log[k] = np.asarray(out["joint_limit_clipped_tau"]).reshape(4)
        jl_active_log[k] = np.asarray(out["joint_limit_active"], dtype=bool).reshape(4)
        tj_sat_log[k] = np.asarray(out["tau_joint_saturated_axes"]).astype(bool).reshape(4)
        ee_err_norm_log[k] = float(np.linalg.norm(y_des[:3] - p_act_k))
        q_joint_log[k] = q_after.copy()
        qdot_joint_log[k] = read_v(model, data, j_ord).copy()
        q_act_log_st[k] = qa_after.copy()

        if np.any(tj_sat_log[k]):
            sat_joint_steps += 1
        last_k = k

    run_len = last_k + 1 if last_k >= 0 else 0
    sl = slice(0, run_len)
    if run_len == 0:
        return {"error": "empty", "nan_abort": nan_abort}

    qj_sl = q_joint_log[sl]
    margin_j_mtx = np.minimum(qhj.reshape(1, 4) - qj_sl, qj_sl - qlj.reshape(1, 4))
    qa_sl = q_act_log_st[sl]
    margin_a_mtx = np.minimum(qha.reshape(1, 4) - qa_sl, qa_sl - qla.reshape(1, 4))
    act_margin_min = np.min(margin_a_mtx, axis=1)

    joint_margin_min = np.min(margin_j_mtx, axis=1)

    e_r = np.array([angle_error(roll_des_cst, r) for r in roll_act_log[sl]])
    e_pr = np.array([angle_error(pitch_des_cst, p) for p in pitch_act_log[sl]])
    ee_n = ee_err_norm_log[sl]
    jl_step_count = int(np.sum(np.any(jl_active_log[sl], axis=1)))

    vmax = float(np.max(vnorm_log[sl]))
    Fmax = float(np.max(Fnorm_log[sl]))

    out_dict: dict[str, Any] = {
        "task_mode": str(task_mode),
        "use_bias": use_bias,
        "use_desired_velocity": use_des_vel,
        "model_path": str(model_path),
        "j_ord": j_ord,
        "a_ord": a_ord,
        "ratios": ratios_f.copy(),
        "qlj": qlj.copy(),
        "qhj": qhj.copy(),
        "qla": qla.copy(),
        "qha": qha.copy(),
        "duration": dur,
        "dt": dt,
        "ts": ts[sl],
        "p_des_xyz": p_des_xyz[sl],
        "p_act": p_act_log[sl],
        "roll_des": roll_des_cst,
        "pitch_des": pitch_des_cst,
        "roll_act": roll_act_log[sl],
        "pitch_act": pitch_act_log[sl],
        "e_task_norm": e_task_norm[sl],
        "ee_err_norm": ee_n,
        "tau_j": tau_j_log[sl],
        "tau_bias": tau_bias_log[sl],
        "tau_task": tau_task_log[sl],
        "tau_total_unc": tau_total_unc_log[sl],
        "jl_active": jl_active_log[sl],
        "jl_clip": jl_clip_log[sl],
        "tj_sat": tj_sat_log[sl],
        "q_joint": q_joint_log[sl],
        "qdot_joint": qdot_joint_log[sl],
        "q_act": q_act_log_st[sl],
        "rms_pos": float(np.sqrt(np.mean(ee_n**2))),
        "max_pos": float(np.max(ee_n)),
        "rms_roll": float(np.sqrt(np.mean(e_r**2))),
        "rms_pitch": float(np.sqrt(np.mean(e_pr**2))),
        "max_tau": float(np.max(np.abs(tau_j_log[sl]))),
        "max_tau_before_clip": float(np.max(np.abs(tau_total_unc_log[sl]))),
        "jl_activation_steps": jl_step_count,
        "torque_sat_steps": sat_joint_steps,
        "final_pos_err": float(ee_n[-1]),
        "nan_abort": nan_abort,
        "m_dim": int(m_dim),
        "tj_lim_arr": np.asarray(tj_lim, dtype=np.float64).reshape(4),
        "max_velocity_norm": vmax,
        "max_task_force_norm": Fmax,
        "joint_vel_norm": vnorm_log[sl],
        "F_task_norm": Fnorm_log[sl],
        "F_task_pad": F_task_pad[sl],
        "ydot_des_pad": ydot_des_pad[sl],
        "ydot_actual_pad": ydot_actual_pad[sl],
        "edot_task_pad": edot_task_pad[sl],
        "edot_task_norm": edotnorm_log[sl],
        "sv_min": sv_min_log[sl],
        "sv_singular_values": sv_mat[sl],
        "J_task_log": J_task_log[sl],
        "joint_margin_min": joint_margin_min,
        "act_margin_min": act_margin_min,
    }
    return out_dict
