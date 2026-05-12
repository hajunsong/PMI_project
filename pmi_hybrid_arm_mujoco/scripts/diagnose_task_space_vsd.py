#!/usr/bin/env python3
"""
Task-space VSD 종합 진단: 모델·전달·Jacobian·부호·게인·제어 모드 비교.
출력: debug_outputs/task_space_vsd/
"""
from __future__ import annotations

import csv
import sys
import textwrap
from pathlib import Path
from typing import Any, Literal

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from controllers.task_space_vsd_controller import TaskSpaceVSDController, TaskSpaceVSDParams
from controllers.vsd_joint_controller import actuator_torques_from_joint
from kinematics.forward_kinematics import fk_ee_rp
from kinematics.orientation_utils import angle_error
from kinematics.task_jacobian import (
    compute_task_jacobian,
    compute_task_jacobian_mode,
    fk_task_pose5,
    fk_task_y,
    task_dim,
)
from kinematics.trajectory import CartesianQuinticPath, WaypointXYZ
from utils.logging_utils import jac_xyz_numeric_vs_analytic_report
from utils.mujoco_helpers import (
    PKG_ROOT,
    VSD_DEBUG_MODEL_PATH,
    apply_ideal_qact_from_joint,
    apply_ideal_qjnt_equals_ratio_qact,
    joint_id,
    load_mjmodel,
)
from utils.path_tracking_io import (
    gains_limits_task_space_vsd,
    joint_actuator_bounds,
    joint_from_actuator_positions,
    load_task_space_vsd_debug_yaml,
    roll_pitch_des_from_orientation_config,
)

OUT_DIR = PKG_ROOT / "debug_outputs" / "task_space_vsd"
CSV_LOG = OUT_DIR / "diagnostic_log.csv"
REPORT_MD = OUT_DIR / "diagnostic_report.md"
SWEEP_CSV = OUT_DIR / "gain_sweep.csv"
MODE_CSV = OUT_DIR / "mode_compare.csv"
STRUCT_TXT = OUT_DIR / "model_structure.txt"

INITIAL_ACTUATOR_RAD = np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=np.float64)
RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=np.float64)
J_ORDER = ["jnt1", "jnt2", "jnt3", "jnt4"]
A_ORDER = ["q1_act", "q2_act", "q3_act", "q4_act"]
SITE_EE = "end_effector"

WAYPOINTS_DIAG = [
    WaypointXYZ(0.0, 0.25, -0.20, -0.10),
    WaypointXYZ(0.5, 0.00, -0.35, -0.15),
    WaypointXYZ(1.0, -0.25, -0.20, -0.10),
]


def diag_build_y(
    tm: str,
    hold: bool,
    y_hold_xyz: np.ndarray,
    pv: np.ndarray | None,
    vv: np.ndarray | None,
    roll_cst: float,
    pitch_cst: float,
) -> tuple[np.ndarray, np.ndarray]:
    if hold:
        p_d = y_hold_xyz.reshape(3).copy()
        pv3 = np.zeros(3)
    else:
        p_d = np.asarray(pv, dtype=np.float64).reshape(3)
        pv3 = np.asarray(vv, dtype=np.float64).reshape(3)
    if tm == "xyz":
        return p_d, pv3
    if tm == "xyz_pitch":
        return np.concatenate([p_d, [pitch_cst]]), np.concatenate([pv3, [0.0]])
    return np.concatenate([p_d, [roll_cst, pitch_cst]]), np.concatenate([pv3, [0.0, 0.0]])


def read_q(model: mj.MjModel, data: mj.MjData, names: list[str]) -> np.ndarray:
    return np.array([float(data.qpos[int(model.jnt_qposadr[joint_id(model, n)])]) for n in names])


def read_v(model: mj.MjModel, data: mj.MjData, names: list[str]) -> np.ndarray:
    return np.array([float(data.qvel[int(model.jnt_dofadr[joint_id(model, n)])]) for n in names])


def ensure_out_dir() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)


def actuator_mode_description(model: mj.MjModel) -> str:
    if model.nu == 0:
        return (
            "이 컴파일 모델에는 액추에이터가 없습니다(strip_position_actuators 등). "
            "토크는 qfrc_applied로만 주입하는 시뮬 설정과 동일합니다."
        )
    lines = []
    for i in range(model.nu):
        nm = mj.mj_id2name(model, mj.mjtObj.mjOBJ_ACTUATOR, i) or f"act{i}"
        # biastype: mjBIAS_AFFINE 등
        bt = int(model.actuator_biastype[i])
        gid = int(model.actuator_trnid[i, 0])
        # gainprm 첫 비영 요소
        gp = model.actuator_gainprm[i].copy()
        bp = model.actuator_biasprm[i].copy()
        lines.append(
            f"  [{i}] {nm}: biastype={bt} trnid={gid} gainprm[:3]={gp[:3]} biasprm[:4]={bp[:4]}"
        )
    lines.append(
        "일반적으로 XML의 general + biastype=affine + biasprm 구조는 **위치 서보(목표 각도 ctrl)** 입니다. "
        "토크 명령을 data.ctrl에 넣는 방식과는 물리량이 다릅니다."
    )
    return "\n".join(lines)


def part1_model_structure(model_raw: mj.MjModel, model_sim: mj.MjModel, data_raw: mj.MjData) -> dict[str, Any]:
    lines: list[str] = []
    lines.append("=== RAW XML 모델 (액추에이터 포함) ===")
    lines.append(f"nq={model_raw.nq} nv={model_raw.nv} nu={model_raw.nu} nsensordata={model_raw.nsensordata}")
    lines.append("")
    lines.append("=== SIM 모델 (load_mjmodel strip_position_actuators) ===")
    lines.append(f"nq={model_sim.nq} nv={model_sim.nv} nu={model_sim.nu}")
    lines.append("")
    lines.append("--- Joints ---")
    for j in range(model_raw.njnt):
        nm = mj.mj_id2name(model_raw, mj.mjtObj.mjOBJ_JOINT, j) or f"j{j}"
        t = int(model_raw.jnt_type[j])
        qadr = int(model_raw.jnt_qposadr[j])
        dadr = int(model_raw.jnt_dofadr[j])
        r0, r1 = float(model_raw.jnt_range[j, 0]), float(model_raw.jnt_range[j, 1])
        lines.append(f"{nm}: type={t} qposadr={qadr} dofadr={dadr} range=[{r0}, {r1}]")

    lines.append("")
    lines.append("--- Actuators (raw model only if nu>0) ---")
    lines.append(actuator_mode_description(model_raw))

    lines.append("")
    lines.append("--- Sensors ---")
    for s in range(model_raw.nsensor):
        nm = mj.mj_id2name(model_raw, mj.mjtObj.mjOBJ_SENSOR, s) or f"s{s}"
        dim = int(model_raw.sensor_dim[s])
        lines.append(f"{nm}: dim={dim}")

    lines.append("")
    lines.append("--- Bodies (worldbody tree names via idx) ---")
    for b in range(1, model_raw.nbody):
        nm = mj.mj_id2name(model_raw, mj.mjtObj.mjOBJ_BODY, b) or f"b{b}"
        lines.append(nm)

    lines.append("")
    lines.append("--- Sites ---")
    for s in range(model_raw.nsite):
        nm = mj.mj_id2name(model_raw, mj.mjtObj.mjOBJ_SITE, s) or f"site{s}"
        lines.append(nm)

    sid = mj.mj_name2id(model_raw, mj.mjtObj.mjOBJ_SITE, SITE_EE)
    lines.append("")
    lines.append(f"--- Site {SITE_EE} id={sid} ---")
    mj.mj_forward(model_raw, data_raw)
    if sid >= 0:
        p = data_raw.site_xpos[sid].copy()
        xmat = data_raw.site_xmat[sid].reshape(3, 3).copy()
        lines.append(f"site_xpos={p}")
        lines.append(f"site_xmat(flat9 first 3)={data_raw.site_xmat[sid][:3]}")

    lines.append("")
    lines.append("--- Index summary ---")
    for nm in J_ORDER + A_ORDER:
        jid = joint_id(model_sim, nm)
        lines.append(
            f"{nm}: joint_id={jid} qposadr={model_sim.jnt_qposadr[jid]} dofadr={model_sim.jnt_dofadr[jid]}"
        )

    text = "\n".join(lines)
    STRUCT_TXT.write_text(text, encoding="utf-8")

    return {
        "raw_nu": int(model_raw.nu),
        "sim_nu": int(model_sim.nu),
        "structure_txt": STRUCT_TXT,
        "actuator_raw_description": actuator_mode_description(model_raw),
    }


def part2_initial_state(
    model: mj.MjModel,
    data: mj.MjData,
    scratch: mj.MjData,
) -> dict[str, Any]:
    qa0 = INITIAL_ACTUATOR_RAD.copy()
    q_exp = joint_from_actuator_positions(qa0, RATIOS)
    for i, nm in enumerate(A_ORDER):
        data.qpos[int(model.jnt_qposadr[joint_id(model, nm)])] = qa0[i]
    apply_ideal_qjnt_equals_ratio_qact(model, data, J_ORDER, A_ORDER, RATIOS)
    mj.mj_forward(model, data)
    qj = read_q(model, data, J_ORDER)
    p, r, pi, y = fk_ee_rp(model, scratch, qj, J_ORDER)
    qlj, qhj, qla, qha = joint_actuator_bounds(model, J_ORDER, A_ORDER)
    jmarg = np.minimum(qhj - qj, qj - qlj)
    amarg = np.minimum(qha - qa0, qa0 - qla)

    out = {
        "q_act_initial": qa0.copy(),
        "q_jnt_expected_ratio": q_exp,
        "q_jnt_actual": qj.copy(),
        "trans_err_norm": float(np.linalg.norm(qj - q_exp)),
        "ee_pos": p.copy(),
        "roll": float(r),
        "pitch": float(pi),
        "yaw": float(y),
        "joint_margin_min": float(np.min(jmarg)),
        "actuator_margin_min": float(np.min(amarg)),
        "coupling_note": (
            "MJCF에 equality/tendon 전달이 없으면 MuJoCo는 jnt와 q_act를 **자동 연결하지 않습니다**. "
            "시뮬에서는 Python에서 apply_ideal_qjnt_equals_ratio_qact 로 동기화합니다."
        ),
    }
    return out


def part3_transmission(
    model: mj.MjModel,
    data: mj.MjData,
) -> dict[str, Any]:
    mj.mj_forward(model, data)
    qa0 = read_q(model, data, A_ORDER)
    qj0 = read_q(model, data, J_ORDER)
    eps = 1e-3
    errs = []
    for i in range(4):
        adr = int(model.jnt_qposadr[joint_id(model, A_ORDER[i])])
        data.qpos[adr] = qa0[i] + eps
        mj.mj_forward(model, data)
        qj_p = read_q(model, data, J_ORDER)
        data.qpos[adr] = qa0[i] - eps
        mj.mj_forward(model, data)
        qj_m = read_q(model, data, J_ORDER)
        data.qpos[adr] = qa0[i]
        # jnt 변화량이 거의 0이면 XML에 기구적 결합 없음
        d_jnt = (qj_p - qj_m) / (2 * eps)
        errs.append(float(np.linalg.norm(d_jnt)))
    mj.mj_forward(model, data)
    expected = joint_from_actuator_positions(qa0, RATIOS)
    static_err = float(np.linalg.norm(read_q(model, data, J_ORDER) - expected))

    max_pert_err = max(errs) if errs else 0.0
    unmeshed = max_pert_err < 1e-8
    note = ""
    if unmeshed:
        note = (
            "q_act를 변동해도 jnt 연쇄가 없습니다(MuJoCo XML은 mimic/전달 제약 없음). "
            "컨트롤러는 q_act가 jnt를 구동한다고 가정할 수 없으며 Python 동기화가 필요합니다."
        )
    return {
        "max_abs_djnt_per_dqa_estimate": max_pert_err,
        "trans_position_static_err_norm": static_err,
        "independent_bodies_warning": unmeshed,
        "note": note,
    }


def part4_jacobian(
    model: mj.MjModel,
    data: mj.MjData,
    jac_workspace: mj.MjData,
    j_eps: float = 1e-6,
) -> dict[str, Any]:
    jac_workspace.qpos[:] = data.qpos
    jac_workspace.qvel[:] = data.qvel
    mj.mj_forward(model, jac_workspace)
    J_num = compute_task_jacobian(
        model,
        jac_workspace,
        joint_names=J_ORDER,
        ee_site_name=SITE_EE,
        mode="numerical",
        epsilon=j_eps,
    )
    jac_workspace.qpos[:] = data.qpos
    jac_workspace.qvel[:] = data.qvel
    mj.mj_forward(model, jac_workspace)
    J_hybrid = compute_task_jacobian(
        model,
        jac_workspace,
        joint_names=J_ORDER,
        ee_site_name=SITE_EE,
        mode="mujoco_analytic",
        epsilon=j_eps,
    )
    diff_xyz = float(np.linalg.norm(J_num[0:3, :] - J_hybrid[0:3, :]))
    u, s, _vh = np.linalg.svd(J_num, full_matrices=False)
    rank = int(np.linalg.matrix_rank(J_num, tol=1e-6 * float(s[0] if len(s) else 1)))
    cond = float(s[0] / s[-1]) if len(s) and s[-1] > 1e-15 else np.inf

    return {
        "J_num": J_num,
        "J_hybrid": J_hybrid,
        "J_pos_diff_norm": diff_xyz,
        "singular_values": s.copy(),
        "rank": rank,
        "condition_number": cond,
        "shape": J_num.shape,
        "jac_xyz_report": jac_xyz_numeric_vs_analytic_report(
            model, data, jac_workspace, J_ORDER, epsilon=j_eps
        ),
    }


def part5_virtual_work_and_sign(
    model: mj.MjModel,
    data: mj.MjData,
    scratch: mj.MjData,
    jac_workspace: mj.MjData,
    dof_j: np.ndarray,
    j_eps: float,
) -> dict[str, Any]:
    jac_workspace.qpos[:] = data.qpos
    jac_workspace.qvel[:] = data.qvel
    mj.mj_forward(model, jac_workspace)
    J = compute_task_jacobian(
        model, jac_workspace, joint_names=J_ORDER, ee_site_name=SITE_EE, mode="numerical", epsilon=j_eps
    )
    qpos0 = data.qpos.copy()
    qvel0 = data.qvel.copy()
    tests = []
    labels = ["Fx", "Fy", "Fz", "Mr", "Mp"]
    qdot_test = np.array([0.01, -0.005, 0.003, 0.002], dtype=np.float64)
    for ax in range(5):
        data.qpos[:] = qpos0
        data.qvel[:] = qvel0
        mj.mj_forward(model, data)
        F = np.zeros(5)
        F[ax] = 1.0
        tau = J.T @ F
        ydot = J @ qdot_test
        vw_inst = float(np.dot(F, ydot) - np.dot(tau, qdot_test))
        p0, *_ = fk_ee_rp(model, scratch, read_q(model, data, J_ORDER), J_ORDER)
        data.qvel[:] = 0.0
        data.qfrc_applied[:] = 0.0
        for _ in range(15):
            for k in range(4):
                data.qfrc_applied[dof_j[k]] = tau[k]
            mj.mj_step(model, data)
            apply_ideal_qact_from_joint(model, data, J_ORDER, A_ORDER, RATIOS)
        mj.mj_forward(model, data)
        p1, *_ = fk_ee_rp(model, scratch, read_q(model, data, J_ORDER), J_ORDER)
        delta = p1 - p0
        tests.append(
            {
                "axis": labels[ax],
                "tau_jnt": tau.copy(),
                "delta_p": delta.copy(),
                "virtual_work_slack": vw_inst,
            }
        )
    data.qpos[:] = qpos0
    data.qvel[:] = qvel0
    mj.mj_forward(model, data)

    sign_notes = []
    if tests[0]["delta_p"][0] < 0:
        sign_notes.append("Fx>0 시 +x 변위가 감소 방향이면 Jacobian 좌표계/부호를 의심.")
    return {"tests": tests, "sign_notes": sign_notes, "J_snapshot": J.copy()}


TorqueMode = Literal["joint_direct_debug", "actuator", "current_ctrl"]


def run_vsd_segment(
    *,
    dt: float,
    duration: float,
    torque_mode: str,
    task_mode: str,
    K_task: np.ndarray,
    D_task: np.ndarray,
    F_lim: np.ndarray,
    tj_lim: np.ndarray,
    ta_lim: np.ndarray,
    roll_des: float,
    pitch_des: float,
    use_bias_compensation: bool = True,
    use_desired_velocity: bool = True,
    joint_limit_enabled: bool = True,
    joint_limit_margin: float = 0.05,
    log_csv_path: Path | None = None,
    csv_rows_max: int | None = None,
    mjcf_path: Path | None = None,
) -> dict[str, Any]:
    spline = CartesianQuinticPath(list(WAYPOINTS_DIAG))
    n = int(round(duration / dt)) + 1
    ts = np.array([min(i * dt, duration) for i in range(n)])

    model_path = Path(mjcf_path) if mjcf_path is not None else VSD_DEBUG_MODEL_PATH
    model = load_mjmodel(model_path, strip_position_actuators=True)
    model.opt.timestep = dt
    model.opt.integrator = int(mj.mjtIntegrator.mjINT_IMPLICITFAST)
    data = mj.MjData(model)
    scratch = mj.MjData(model)
    jac_ws = mj.MjData(model)

    for i, nm in enumerate(A_ORDER):
        data.qpos[int(model.jnt_qposadr[joint_id(model, nm)])] = INITIAL_ACTUATOR_RAD[i]
    apply_ideal_qjnt_equals_ratio_qact(model, data, J_ORDER, A_ORDER, RATIOS)
    mj.mj_forward(model, data)

    qj_init = read_q(model, data, J_ORDER)
    p_hold, *_ = fk_ee_rp(model, scratch, qj_init, J_ORDER)
    y_hold = np.asarray(p_hold, dtype=np.float64).reshape(3)

    dof_j = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in J_ORDER])
    dof_a = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in A_ORDER])
    qlj, qhj, qla, qha = joint_actuator_bounds(model, J_ORDER, A_ORDER)

    m_dim = task_dim(task_mode)  # type: ignore[arg-type]
    vsd = TaskSpaceVSDController(
        TaskSpaceVSDParams(
            task_mode=task_mode,  # type: ignore[arg-type]
            K_task=K_task,
            D_task=D_task,
            F_task_limit=F_lim,
            tau_jnt_limit=tj_lim,
            tau_act_limit=ta_lim,
            jac_flip_sign=1.0,
            use_bias_compensation=use_bias_compensation,
            use_desired_velocity=use_desired_velocity,
            joint_limit_enabled=joint_limit_enabled,
            joint_limit_margin=joint_limit_margin,
            q_joint_min=qlj,
            q_joint_max=qhj,
        )
    )

    rows: list[dict[str, Any]] = []
    nan_stop = False
    sat_j = 0
    sat_a = 0

    # 로그 버퍼(플롯용)
    buf = {
        "t": [],
        "p_des": [],
        "p_act": [],
        "roll_des": [],
        "roll_act": [],
        "pitch_des": [],
        "pitch_act": [],
        "yaw_act": [],
        "F": [],
        "tau_j": [],
        "tau_bias": [],
        "tau_task": [],
        "tau_a": [],
        "ee_err": [],
        "e_task_norm": [],
        "sv": [],
        "e_task": [],
        "trans_err": [],
        "qj": [],
        "qa": [],
        "tau_j_sat": [],
        "tau_a_sat": [],
        "F_sat": [],
        "jl_act": [],
    }

    def mode_apply(tau_j: np.ndarray, tau_a: np.ndarray) -> None:
        data.qfrc_applied[:] = 0.0
        data.ctrl[:] = 0.0
        if torque_mode == "joint_direct_debug":
            for k in range(4):
                data.qfrc_applied[dof_j[k]] += tau_j[k]
        elif torque_mode == "actuator":
            for k in range(4):
                data.qfrc_applied[dof_a[k]] += tau_a[k]
        else:
            # current: 기본 재현 = joint_direct
            for k in range(4):
                data.qfrc_applied[dof_j[k]] += tau_j[k]

    for k, t_k in enumerate(ts):
        qj = read_q(model, data, J_ORDER)
        vj = read_v(model, data, J_ORDER)
        qa = read_q(model, data, A_ORDER)
        va = read_v(model, data, A_ORDER)

        hold_pose = float(t_k) <= 1e-15
        pv, vv = None, None
        if not hold_pose:
            pv, vv, _ = spline.sample(float(t_k))

        y_des, ydot_des = diag_build_y(task_mode, hold_pose, y_hold, pv, vv, roll_des, pitch_des)

        mj.mj_forward(model, data)
        tau_bias_joint = np.array([float(data.qfrc_bias[int(dof_j[i])]) for i in range(4)], dtype=np.float64)
        y_act = fk_task_y(model, scratch, qj, J_ORDER, task_mode=task_mode)  # type: ignore[arg-type]

        jac_ws.qpos[:] = data.qpos
        jac_ws.qvel[:] = data.qvel
        mj.mj_forward(model, jac_ws)
        Jt = compute_task_jacobian_mode(
            model,
            jac_ws,
            joint_names=J_ORDER,
            task_mode=task_mode,  # type: ignore[arg-type]
            ee_site_name=SITE_EE,
            mode="numerical",
            epsilon=1e-6,
        )

        out = vsd.compute(
            y_des=y_des,
            ydot_des=ydot_des,
            y_actual=y_act,
            q_joint=qj,
            qdot_joint=vj,
            J_task=Jt,
            ratios_actuator=RATIOS,
            tau_bias_joint=tau_bias_joint if use_bias_compensation else None,
        )
        tau_j = np.asarray(out["tau_joint"]).reshape(4)
        tau_a = np.asarray(out["tau_act"]).reshape(4)
        F_ap = np.asarray(out["F_task"]).reshape(m_dim)
        Fro = np.asarray(out["F_raw"]).reshape(m_dim)
        tau_bias_out = np.asarray(out["tau_bias_jnt"]).reshape(4)
        tau_task_only = np.asarray(out["tau_task_jnt"]).reshape(4)

        ek = np.asarray(out["e_task"]).reshape(-1)

        _, sv, _ = np.linalg.svd(Jt, full_matrices=False)

        Jflat_row: dict[str, float] = {}
        for r in range(m_dim):
            for c in range(4):
                Jflat_row[f"J_{r}_{c}"] = float(Jt[r, c])

        if not np.all(np.isfinite(tau_j)) or not np.all(np.isfinite(F_ap)):
            nan_stop = True
            break

        jm = float(np.min(np.minimum(qhj - qj, qj - qlj)))
        am = float(np.min(np.minimum(qha - qa, qa - qla)))

        data.qfrc_applied[:] = 0.0
        data.ctrl[:] = 0.0
        mode_apply(tau_j, tau_a)

        qfrc_before = data.qfrc_applied.copy()
        mj.mj_step(model, data)
        if torque_mode == "actuator":
            apply_ideal_qjnt_equals_ratio_qact(model, data, J_ORDER, A_ORDER, RATIOS)
        else:
            apply_ideal_qact_from_joint(model, data, J_ORDER, A_ORDER, RATIOS)

        mj.mj_forward(model, data)
        p_ee, r_a, pi_a, yw_a = fk_ee_rp(model, scratch, read_q(model, data, J_ORDER), J_ORDER)

        ee_err = float(np.linalg.norm(y_des[:3] - p_ee))

        ed = np.asarray(out["edot_task"]).reshape(-1)
        ydot_act = np.asarray(out["ydot_actual"]).reshape(-1)
        e_norm = float(np.linalg.norm(ek))

        if np.any(out["tau_joint_saturated_axes"]):
            sat_j += 1
        if np.any(out["tau_act_saturated_axes"]):
            sat_a += 1

        qj_post = read_q(model, data, J_ORDER)
        qa_post = read_q(model, data, A_ORDER)
        tre = float(np.linalg.norm(qj_post - joint_from_actuator_positions(qa_post, RATIOS)))

        row_pad: dict[str, Any] = {
            "t": float(t_k),
            **{f"qj{i+1}": float(qj[i]) for i in range(4)},
            **{f"qdj{i+1}": float(vj[i]) for i in range(4)},
            **{f"qa{i+1}": float(qa[i]) for i in range(4)},
            **{f"qda{i+1}": float(va[i]) for i in range(4)},
            "jnt_margin": jm,
            "act_margin": am,
            "task_mode": task_mode,
            "use_bias": use_bias_compensation,
            **{f"y_des_{c}": float(y_des[c]) if c < len(y_des) else float("nan") for c in range(5)},
            **{
                **{f"y_act_flat_{c}": float(fk_task_pose5(model, scratch, qj, J_ORDER)[c]) for c in range(5)},
            },
            "yaw_actual": float(yw_a),
            **{f"e_{c}": float(ek[c]) if c < len(ek) else float("nan") for c in range(5)},
            **{f"yd_des_{c}": float(ydot_des[c]) if c < len(ydot_des) else float("nan") for c in range(5)},
            **{f"yd_act_{c}": float(ydot_act[c]) if c < len(ydot_act) else float("nan") for c in range(5)},
            **{f"ed_{c}": float(ed[c]) if c < len(ed) else float("nan") for c in range(5)},
            **{f"K_{c}": float(K_task[c]) if c < len(K_task) else float("nan") for c in range(5)},
            **{f"D_{c}": float(D_task[c]) if c < len(D_task) else float("nan") for c in range(5)},
            **{f"F_raw_{c}": float(Fro[c]) if c < len(Fro) else float("nan") for c in range(5)},
            **{f"F_{c}": float(F_ap[c]) if c < len(F_ap) else float("nan") for c in range(5)},
        }
        row_pad.update(Jflat_row)
        row_pad.update(
            {
                **{f"sv{i}": float(sv[i]) if i < len(sv) else 0.0 for i in range(4)},
                **{f"tau_bias{i+1}": float(tau_bias_out[i]) for i in range(4)},
                **{f"tau_task{i+1}": float(tau_task_only[i]) for i in range(4)},
                "tau_unc_norm": float(np.linalg.norm(np.asarray(out["tau_total_unc"]))),
                **{f"tau_j{i+1}": float(tau_j[i]) for i in range(4)},
                **{f"tau_a{i+1}": float(tau_a[i]) for i in range(4)},
                **{f"jl_act{i}": bool(out["joint_limit_active"][i]) for i in range(4)},
                "tau_j_sat_any": bool(np.any(out["tau_joint_saturated_axes"])),
                "tau_a_sat_any": bool(np.any(out["tau_act_saturated_axes"])),
                "torque_mode": torque_mode,
                "ctrl_norm": float(np.linalg.norm(data.ctrl)),
                **{f"qfrc_applied_dof{dof_j[i]}": float(qfrc_before[dof_j[i]]) for i in range(4)},
                "qfrc_actuator_norm": float(np.linalg.norm(data.qfrc_actuator)),
                "qfrc_bias_norm": float(np.linalg.norm(data.qfrc_bias)),
                "qacc_norm": float(np.linalg.norm(data.qacc)),
                "ee_pos_err": ee_err,
                "e_task_norm": e_norm,
                "trans_err_norm": tre,
            }
        )
        row = row_pad

        if log_csv_path is not None and (csv_rows_max is None or len(rows) < csv_rows_max):
            rows.append(row)

        buf["t"].append(float(t_k))
        buf["p_des"].append(y_des[:3].copy())
        buf["p_act"].append(np.asarray(p_ee).copy())
        buf["roll_des"].append(roll_des)
        buf["roll_act"].append(float(r_a))
        buf["pitch_des"].append(pitch_des)
        buf["pitch_act"].append(float(pi_a))
        buf["yaw_act"].append(float(yw_a))
        buf["F"].append(np.concatenate([F_ap, np.zeros(max(0, 5 - len(F_ap)))]))
        buf["tau_j"].append(tau_j.copy())
        buf["tau_bias"].append(tau_bias_out.copy())
        buf["tau_task"].append(tau_task_only.copy())
        buf["tau_a"].append(tau_a.copy())
        buf["ee_err"].append(ee_err)
        buf["e_task_norm"].append(e_norm)
        buf["sv"].append(sv.copy() if len(sv) >= 4 else np.pad(sv, (0, 4 - len(sv))))
        buf["e_task"].append(np.concatenate([ek, np.zeros(max(0, 5 - len(ek)))]))
        buf["trans_err"].append(tre)
        buf["qj"].append(qj_post.copy())
        buf["qa"].append(qa_post.copy())
        buf["tau_j_sat"].append(bool(np.any(out["tau_joint_saturated_axes"])))
        buf["tau_a_sat"].append(bool(np.any(out["tau_act_saturated_axes"])))
        buf["F_sat"].append(bool(np.any(out["F_saturated_axes"])))
        buf["jl_act"].append(np.asarray(out["joint_limit_active"], dtype=bool).copy())

    # metrics
    t_arr = np.array(buf["t"])
    if len(t_arr) == 0:
        return {"error": "empty", "nan_stop": nan_stop}

    Pd = np.stack(buf["p_des"])
    Pa = np.stack(buf["p_act"])
    errp = np.linalg.norm(Pd - Pa, axis=1)
    ra = np.array(buf["roll_act"])
    pa = np.array(buf["pitch_act"])
    errs_r = np.array([angle_error(roll_des, float(ra[i])) for i in range(len(ra))])
    errs_p = np.array([angle_error(pitch_des, float(pa[i])) for i in range(len(pa))])

    ee_arr = np.array(buf["ee_err"])
    mono_run = 0
    if len(ee_arr) > 1:
        d = np.diff(ee_arr)
        run = 0
        for x in d:
            if x > 1e-5:
                run += 1
                mono_run = max(mono_run, run)
            else:
                run = 0

    met = {
        "rms_pos": float(np.sqrt(np.mean(errp**2))),
        "max_pos": float(np.max(errp)),
        "rms_roll": float(np.sqrt(np.mean(errs_r**2))),
        "rms_pitch": float(np.sqrt(np.mean(errs_p**2))),
        "max_tau_j": float(np.max(np.abs(np.stack(buf["tau_j"])))) if buf["tau_j"] else 0.0,
        "sat_j": sat_j,
        "sat_a": sat_a,
        "nan_stop": nan_stop,
        "monotonic_pos_err_growth_max_run": int(mono_run),
        "buffer": buf,
        "rows": rows,
    }

    if log_csv_path is not None and rows:
        cols = list(rows[0].keys())
        with open(log_csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(rows)

    return met


def plot_from_buffer(buf: dict[str, Any], tag: str) -> None:
    t = np.array(buf["t"])
    if len(t) < 2:
        return
    Pd = np.stack(buf["p_des"])
    Pa = np.stack(buf["p_act"])
    fig, ax = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for i, lb in enumerate(["x", "y", "z"]):
        ax[i].plot(t, Pd[:, i], label="des")
        ax[i].plot(t, Pa[:, i], "--", label="act")
        ax[i].set_ylabel(lb)
        ax[i].grid(True, alpha=0.3)
        ax[i].legend()
    ax[-1].set_xlabel("t [s]")
    plt.tight_layout()
    plt.savefig(OUT_DIR / f"plot_ee_xyz_{tag}.png", dpi=140)
    plt.close()

    fig = plt.figure(figsize=(6, 5))
    try:
        ax3 = fig.add_subplot(111, projection="3d")
        ax3.plot(Pa[:, 0], Pa[:, 1], Pa[:, 2], label="act")
        ax3.plot(Pd[:, 0], Pd[:, 1], Pd[:, 2], "--", label="des")
        ax3.set_xlabel("x")
        ax3.set_ylabel("y")
        ax3.set_zlabel("z")
        ax3.legend()
    except Exception:
        plt.close()
        fig, ax2 = plt.subplots(figsize=(5, 4))
        ax2.plot(Pa[:, 0], Pa[:, 1], label="act")
        ax2.plot(Pd[:, 0], Pd[:, 1], "--", label="des")
        ax2.set_aspect("equal")
        ax2.legend()
        ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(OUT_DIR / f"plot_path3d_{tag}.png", dpi=140)
    plt.close()

    rd = np.array(buf["roll_des"])[0]
    fig, ax = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    ax[0].plot(t, buf["roll_act"], label="roll act")
    ax[0].axhline(rd, color="k", ls="--", label="roll des")
    ax[1].plot(t, buf["pitch_act"], label="pitch act")
    ax[1].axhline(buf["pitch_des"][0], color="k", ls="--", label="pitch des")
    for a in ax:
        a.legend()
        a.grid(True, alpha=0.3)
    plt.savefig(OUT_DIR / f"plot_roll_pitch_{tag}.png", dpi=140)
    plt.close()

    plt.figure(figsize=(8, 3))
    plt.plot(t, buf["yaw_act"], color="gray")
    plt.ylabel("yaw")
    plt.xlabel("t")
    plt.grid(True, alpha=0.3)
    plt.savefig(OUT_DIR / f"plot_yaw_{tag}.png", dpi=140)
    plt.close()

    Fm = np.stack(buf["F"])
    fig, ax = plt.subplots(5, 1, figsize=(10, 9), sharex=True)
    labs = ["Fx", "Fy", "Fz", "Mr", "Mp"]
    for i in range(5):
        ax[i].plot(t, Fm[:, i])
        ax[i].set_ylabel(labs[i])
        ax[i].grid(True, alpha=0.3)
    ax[-1].set_xlabel("t")
    plt.tight_layout()
    plt.savefig(OUT_DIR / f"plot_F_task_{tag}.png", dpi=140)
    plt.close()

    Tj = np.stack(buf["tau_j"])
    fig, ax = plt.subplots(4, 1, figsize=(9, 7), sharex=True)
    for i in range(4):
        ax[i].plot(t, Tj[:, i], label=f"tau_j{i+1}")
        ax[i].grid(True, alpha=0.3)
    ax[-1].set_xlabel("t")
    plt.tight_layout()
    plt.savefig(OUT_DIR / f"plot_tau_jnt_{tag}.png", dpi=140)
    plt.close()

    Ta = np.stack(buf["tau_a"])
    fig, ax = plt.subplots(4, 1, figsize=(9, 7), sharex=True)
    for i in range(4):
        ax[i].plot(t, Ta[:, i], label=f"tau_a{i+1}")
        ax[i].grid(True, alpha=0.3)
    ax[-1].set_xlabel("t")
    plt.tight_layout()
    plt.savefig(OUT_DIR / f"plot_tau_act_{tag}.png", dpi=140)
    plt.close()

    if buf.get("tau_bias") and len(buf["tau_bias"]) == len(t):
        Tb = np.stack(buf["tau_bias"])
        Tk = np.stack(buf["tau_task"])
        fig, ax = plt.subplots(4, 1, figsize=(9, 7), sharex=True)
        for i in range(4):
            ax[i].plot(t, Tb[:, i], label="tau_bias")
            ax[i].plot(t, Tk[:, i], "--", label="tau_task")
            ax[i].set_ylabel(f"j{i+1}")
            ax[i].legend(fontsize=8)
            ax[i].grid(True, alpha=0.3)
        ax[-1].set_xlabel("t [s]")
        plt.suptitle("τ_bias (qfrc_bias) vs τ_task (JᵀF)", y=1.01)
        plt.tight_layout()
        plt.savefig(OUT_DIR / f"plot_tau_bias_task_{tag}.png", dpi=140)
        plt.close()

        Ttot = np.stack(buf["tau_j"])
        fig, ax = plt.subplots(4, 1, figsize=(9, 7), sharex=True)
        for i in range(4):
            ax[i].plot(t, Ttot[:, i], label="tau_total")
            ax[i].legend(fontsize=8)
            ax[i].grid(True, alpha=0.3)
        ax[-1].set_xlabel("t [s]")
        plt.tight_layout()
        plt.savefig(OUT_DIR / f"plot_tau_total_{tag}.png", dpi=140)
        plt.close()

    if buf.get("jl_act") and len(buf["jl_act"]) == len(t):
        JL = np.stack(buf["jl_act"]).astype(float)
        plt.figure(figsize=(8, 3))
        for i in range(4):
            plt.step(t, JL[:, i], where="post", label=f"jnt{i+1}")
        plt.ylabel("joint limit torque gate")
        plt.xlabel("t [s]")
        plt.legend(ncol=4, fontsize=8)
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(OUT_DIR / f"plot_joint_limit_flags_{tag}.png", dpi=140)
        plt.close()

    SV = np.stack(buf["sv"])
    plt.figure(figsize=(8, 4))
    for i in range(min(4, SV.shape[1])):
        plt.plot(t, SV[:, i], label=f"sv{i+1}")
    plt.legend()
    plt.xlabel("t")
    plt.ylabel("singular value")
    plt.grid(True, alpha=0.3)
    plt.savefig(OUT_DIR / f"plot_singular_values_{tag}.png", dpi=140)
    plt.close()

    if buf.get("e_task") and len(buf["e_task"]) == len(t):
        Et = np.stack(buf["e_task"])
        fig, ax = plt.subplots(5, 1, figsize=(10, 9), sharex=True)
        elabs = ["ex", "ey", "ez", "eroll", "epitch"]
        for i in range(5):
            ax[i].plot(t, Et[:, i])
            ax[i].set_ylabel(elabs[i])
            ax[i].grid(True, alpha=0.3)
        ax[-1].set_xlabel("t [s]")
        plt.tight_layout()
        plt.savefig(OUT_DIR / f"plot_task_error_{tag}.png", dpi=140)
        plt.close()

    if buf.get("trans_err"):
        plt.figure(figsize=(8, 3))
        plt.plot(t, buf["trans_err"], color="C1")
        plt.ylabel("||q_jnt - r*q_act||")
        plt.xlabel("t [s]")
        plt.grid(True, alpha=0.3)
        plt.savefig(OUT_DIR / f"plot_transmission_err_{tag}.png", dpi=140)
        plt.close()

    if buf.get("qj") and len(buf["qj"]) == len(t):
        QJ = np.stack(buf["qj"])
        QA = np.stack(buf["qa"])
        terr = QJ - QA * RATIOS[np.newaxis, :]
        fig, ax = plt.subplots(4, 1, figsize=(9, 7), sharex=True)
        for i in range(4):
            ax[i].plot(t, terr[:, i], label=f"jnt{i+1} - r·qa")
            ax[i].grid(True, alpha=0.3)
            ax[i].legend(loc="upper right", fontsize=8)
        ax[-1].set_xlabel("t [s]")
        plt.suptitle("Transmission mapping: q_jnt - ratio*q_act", y=1.01)
        plt.tight_layout()
        plt.savefig(OUT_DIR / f"plot_transmission_components_{tag}.png", dpi=140)
        plt.close()

        fig, ax = plt.subplots(4, 1, figsize=(9, 7), sharex=True)
        for i in range(4):
            ax[i].plot(t, QJ[:, i], label="q_jnt")
            ax[i].plot(t, QA[:, i] * RATIOS[i], "--", label="r*q_act")
            ax[i].legend(fontsize=8)
            ax[i].grid(True, alpha=0.3)
        ax[-1].set_xlabel("t [s]")
        plt.tight_layout()
        plt.savefig(OUT_DIR / f"plot_qj_qa_{tag}.png", dpi=140)
        plt.close()

    if buf.get("tau_j_sat") and len(buf["tau_j_sat"]) == len(t):
        fig, ax = plt.subplots(3, 1, figsize=(8, 5), sharex=True)
        ax[0].step(t, np.array(buf["F_sat"]).astype(int), where="post")
        ax[0].set_ylabel("F clip")
        ax[1].step(t, np.array(buf["tau_j_sat"]).astype(int), where="post")
        ax[1].set_ylabel("tau_j clip")
        ax[2].step(t, np.array(buf["tau_a_sat"]).astype(int), where="post")
        ax[2].set_ylabel("tau_a clip")
        ax[2].set_xlabel("t [s]")
        for a in ax:
            a.set_yticks([0, 1])
            a.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(OUT_DIR / f"plot_saturation_flags_{tag}.png", dpi=140)
        plt.close()


def main() -> None:
    ensure_out_dir()
    print(f"[diagnose] 출력 디렉토리: {OUT_DIR.resolve()}")

    j_eps = 1e-6
    model_raw = mj.MjModel.from_xml_path(str(VSD_DEBUG_MODEL_PATH))
    data_raw = mj.MjData(model_raw)
    model_sim = load_mjmodel(VSD_DEBUG_MODEL_PATH, strip_position_actuators=True)
    data_sim = mj.MjData(model_sim)
    scratch = mj.MjData(model_sim)
    jac_ws = mj.MjData(model_sim)

    # Part 1
    p1 = part1_model_structure(model_raw, model_sim, data_raw)
    print(textwrap.dedent(f"""
    === PART 1: model structure ===
    RAW nu={p1['raw_nu']}  SIM nu={p1['sim_nu']}
    상세: {p1['structure_txt']}
    """))

    # Part 2
    p2 = part2_initial_state(model_sim, data_sim, scratch)
    print("=== PART 2: initial state ===")
    print("q_act:", p2["q_act_initial"])
    print("expected q_jnt (ratio*q_act):", p2["q_jnt_expected_ratio"])
    print("actual q_jnt after sync:", p2["q_jnt_actual"])
    print("||jnt - expected||:", p2["trans_err_norm"])
    print("EE pos:", p2["ee_pos"], "rpy:", (p2["roll"], p2["pitch"], p2["yaw"]))
    print(p2["coupling_note"])

    # Part 3
    p3 = part3_transmission(model_sim, data_sim)
    print("=== PART 3: transmission perturbation ===")
    print(p3)

    # Part 4
    mj.mj_forward(model_sim, data_sim)
    p4 = part4_jacobian(model_sim, data_sim, jac_ws, j_eps=j_eps)
    print("=== PART 4: Jacobian ===")
    print("shape:", p4["shape"], "rank:", p4["rank"], "cond:", p4["condition_number"])
    print("sv:", p4["singular_values"])
    print("J_num:\n", p4["J_num"])
    print(p4["jac_xyz_report"], "diff_xyz_norm:", p4["J_pos_diff_norm"])

    # Part 5
    dof_j = np.array([int(model_sim.jnt_dofadr[joint_id(model_sim, n)]) for n in J_ORDER])
    p5 = part5_virtual_work_and_sign(model_sim, data_sim, scratch, jac_ws, dof_j, j_eps)
    print("=== PART 5: virtual work / sign ===")
    for t in p5["tests"]:
        print(t["axis"], "tau_norm", np.linalg.norm(t["tau_jnt"]), "delta_p", t["delta_p"], "vw_slack", t["virtual_work_slack"])

    dbg_yaml = load_task_space_vsd_debug_yaml()
    tsv_diag = dbg_yaml["task_space_vsd"]
    tm_yaml, Ky, Dy, Fly, tj_lim, ta_lim = gains_limits_task_space_vsd(tsv_diag)
    ori_d = tsv_diag["desired_orientation"]
    mj.mj_forward(model_sim, data_sim)
    roll_des_run, pitch_des_run = roll_pitch_des_from_orientation_config(
        ori_d,
        model_sim,
        scratch,
        read_q(model_sim, data_sim, J_ORDER),
        J_ORDER,
    )
    jl_cfg = tsv_diag.get("joint_limit", {})
    jb = bool(tsv_diag.get("use_bias_compensation", True))
    udv = bool(tsv_diag.get("use_desired_velocity", True))

    # Part 6 full log — task_space_vsd_debug.yaml 과제모드·게인
    met6 = run_vsd_segment(
        dt=0.001,
        duration=1.0,
        torque_mode="joint_direct_debug",
        task_mode=str(tm_yaml),
        K_task=Ky,
        D_task=Dy,
        F_lim=Fly,
        tj_lim=tj_lim,
        ta_lim=ta_lim,
        roll_des=roll_des_run,
        pitch_des=pitch_des_run,
        use_bias_compensation=jb,
        use_desired_velocity=udv,
        joint_limit_enabled=bool(jl_cfg.get("enabled", True)),
        joint_limit_margin=float(jl_cfg.get("margin", 0.05)),
        log_csv_path=CSV_LOG,
        csv_rows_max=None,
    )
    print("=== PART 6: VSD log ===")
    print("CSV ->", CSV_LOG, "rows", len(met6.get("rows", [])))
    print("monotonic pos err growth run (steps):", met6.get("monotonic_pos_err_growth_max_run"))
    if "buffer" in met6:
        plot_from_buffer(met6["buffer"], "part6")

    # Part 7
    short_dur = 0.6
    mode_results = []
    for label, mode in [
        ("A_joint", "joint_direct_debug"),
        ("B_actuator", "actuator"),
        ("C_current", "current_ctrl"),
    ]:
        m = run_vsd_segment(
            dt=0.002,
            duration=short_dur,
            torque_mode=mode,
            task_mode=str(tm_yaml),
            K_task=Ky.copy(),
            D_task=Dy.copy(),
            F_lim=Fly.copy(),
            tj_lim=tj_lim,
            ta_lim=ta_lim,
            roll_des=roll_des_run,
            pitch_des=pitch_des_run,
            use_bias_compensation=jb,
            use_desired_velocity=udv,
            joint_limit_enabled=bool(jl_cfg.get("enabled", True)),
            joint_limit_margin=float(jl_cfg.get("margin", 0.05)),
            log_csv_path=None,
        )
        mode_results.append(
            {
                "mode": label,
                "torque_mode": mode,
                "rms_pos": m.get("rms_pos"),
                "max_pos": m.get("max_pos"),
                "rms_roll": m.get("rms_roll"),
                "rms_pitch": m.get("rms_pitch"),
                "max_tau_j": m.get("max_tau_j"),
                "sat_j": m.get("sat_j"),
                "sat_a": m.get("sat_a"),
                "nan_stop": m.get("nan_stop"),
            }
        )
        if "buffer" in m:
            plot_from_buffer(m["buffer"], f"mode_{label}")

    with open(MODE_CSV, "w", newline="", encoding="utf-8") as f:
        if mode_results:
            w = csv.DictWriter(f, fieldnames=list(mode_results[0].keys()))
            w.writeheader()
            w.writerows(mode_results)

    # Part 8 gain sweep — xyz 과제만 3항 게인 브레킷 적용 (그 외는 medium만)
    if str(tm_yaml) == "xyz":
        sweeps = [
            ("low", np.array([10.0, 10.0, 10.0]), np.array([2.0, 2.0, 2.0])),
            ("medium", Ky.copy(), Dy.copy()),
            ("high", np.array([80.0, 80.0, 80.0]), np.array([8.0, 8.0, 8.0])),
        ]
        fl_sw = Fly.copy()
    else:
        sweeps = [("medium", Ky.copy(), Dy.copy())]
        fl_sw = Fly.copy()

    sweep_rows = []
    for name, Kk, Dk in sweeps:
        m = run_vsd_segment(
            dt=0.002,
            duration=0.5,
            torque_mode="joint_direct_debug",
            task_mode=str(tm_yaml),
            K_task=Kk,
            D_task=Dk,
            F_lim=fl_sw,
            tj_lim=tj_lim,
            ta_lim=ta_lim,
            roll_des=roll_des_run,
            pitch_des=pitch_des_run,
            use_bias_compensation=jb,
            use_desired_velocity=udv,
            joint_limit_enabled=bool(jl_cfg.get("enabled", True)),
            joint_limit_margin=float(jl_cfg.get("margin", 0.05)),
            log_csv_path=None,
        )
        sweep_rows.append(
            {
                "gain_set": name,
                "rms_pos": m.get("rms_pos"),
                "max_pos": m.get("max_pos"),
                "rms_roll": m.get("rms_roll"),
                "rms_pitch": m.get("rms_pitch"),
                "max_tau_j": m.get("max_tau_j"),
                "sat_j": m.get("sat_j"),
                "stable": not m.get("nan_stop", True),
            }
        )
    with open(SWEEP_CSV, "w", newline="", encoding="utf-8") as f:
        if sweep_rows:
            w = csv.DictWriter(f, fieldnames=list(sweep_rows[0].keys()))
            w.writeheader()
            w.writerows(sweep_rows)

    # Report
    report_lines = [
        "# Task-space VSD Diagnostic Report",
        "",
        "## 1. Model structure",
        f"- Raw XML nu={p1['raw_nu']} (position servos on q*_act if nu>0).",
        f"- Simulation model nu={p1['sim_nu']} (strip_position_actuators → torque via qfrc only).",
        f"- See `{STRUCT_TXT.name}` for joint/sensor/site listing.",
        "",
        "## 2. ACTUATOR MODE CHECK",
        "- **Raw XML:** `general` actuators with affine bias are **PD-style position actuators**; `ctrl` is typically desired joint coordinate (not torque).",
        "- **Task-space VSD in scripts** applies torque through `data.qfrc_applied` on joint or actuator dofs, with `ctrl=0` on stripped model.",
        "- If someone wrote torque into `data.ctrl` on the **raw** model, it would **not** behave like torque — mismatch guaranteed.",
        "- `data.qfrc_actuator` is zero when nu=0; applied wrench shows up in `qfrc_applied`.",
        "",
        "## 3. Initial state & transmission",
        f"- Expected q_jnt from ratio×q_act err norm: {p2['trans_err_norm']:.3e} after Python sync.",
        f"- Transmission perturbation test: max |Δjnt|/Δqa ~ {p3['max_abs_djnt_per_dqa_estimate']:.3e}  static err {p3['trans_position_static_err_norm']:.3e}",
        f"- {p3.get('note','')}",
        "",
        "## 4. Jacobian",
        f"- Shape {p4['shape']}, rank {p4['rank']}, cond ~ {p4['condition_number']:.3g}",
        f"- Singular values: {p4['singular_values']}",
        f"- Position Jacobian num vs MuJoCo hybrid xyz diff norm: {p4['J_pos_diff_norm']:.3e}",
        "**5D task / 4 joints:** J is 5×4; cannot span all R^5 — residual orientation/position tradeoff is expected.",
        "",
        "## 5. Virtual work & sign",
    ]
    for t in p5["tests"]:
        report_lines.append(f"- {t['axis']}: |τ|={np.linalg.norm(t['tau_jnt']):.3f}  Δp={t['delta_p']}  inst_slack={t['virtual_work_slack']:.2e}")
    if p5["sign_notes"]:
        report_lines.append("- **Sign:** " + " ".join(p5["sign_notes"]))

    report_lines.extend(
        [
            "",
            "## 6. VSD run (Part 6 CSV)",
            f"- MJCF 디버그 모델: `{VSD_DEBUG_MODEL_PATH.name}`",
            f"- task_mode=`{tm_yaml}`, use_bias_compensation={jb}",
            f"- Log: `{CSV_LOG.name}`",
            f"- RMS pos err: {met6.get('rms_pos')}, max: {met6.get('max_pos')}",
            f"- Monotonic position error growth max consecutive steps (threshold 1e-5): {met6.get('monotonic_pos_err_growth_max_run', 0)}",
            "",
            "## 7. Control modes A/B/C",
            f"- Table: `{MODE_CSV.name}`",
        ]
    )
    for mr in mode_results:
        report_lines.append(f"  - {mr}")

    report_lines.extend(
        [
            "",
            "## 8. Gain sweep",
            f"- `{SWEEP_CSV.name}`",
        ]
    )
    for s in sweep_rows:
        report_lines.append(f"  - {s}")

    report_lines.extend(
        [
            "",
            "## 9. Most likely failure causes (checklist)",
            "- **XML suitable for torque in ctrl?** No — position actuators; use stripped model + qfrc (current workflow) or replace with torque actuators.",
            "- **q_act mechanically coupled in MuJoCo?** No equality in MJCF; coupling is Python `apply_ideal_*`.",
            "- **q_jnt = ratio×q_act enforced in physics?** Not automatically; only after sync step.",
            "- **Jacobian correct?** Compare xyz rows vs MuJoCo; orientation rows use numerical hybrid in this codebase.",
            "- **τ = JᵀF direction?** See Part 5 short impulse tests.",
            "- **5D task vs 4DOF?** Yes, over-constrained subspace; expect non-zero residual unless gains/tasks aligned.",
            "- **Orientation gains too high?** Can saturate or fight position — reduce K_rp / D_rp or task weights.",
            "- **Clipping?** Check F_task and tau limits in config.",
            "- **Torque into position actuators?** If nu>0 and ctrl used incorrectly — yes, fatal; use qfrc or strip actuators.",
            "",
            "## 10. Recommended next fixes",
            "- Keep simulation on **stripped** model for torque; never send torque through `ctrl` on position-actuator XML.",
            "- Verify Python transmission sync each step matches deployed controller.",
            "- Tune medium gains first; watch singular values — near rank loss directions explode force demand.",
        ]
    )

    REPORT_MD.write_text("\n".join(report_lines), encoding="utf-8")
    print(f"[diagnose] 보고서: {REPORT_MD}")
    print("[diagnose] 완료.")


if __name__ == "__main__":
    main()
