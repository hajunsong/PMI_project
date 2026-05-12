#!/usr/bin/env python3
"""하이브리드 무충돌 MJCF: 관절측 qfrc_applied 만으로 IK-VSD / JTF-VSD 검증(액추에이터 토크·ctrl 미사용)."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import mujoco as mj
import numpy as np

from kinematics.forward_kinematics import fk_ee_rp
from kinematics.inverse_kinematics import IKConfig, solve_ik_task_mode
from kinematics.task_jacobian import compute_task_jacobian_mode
from trajectory.joint_quintic import scaled_joint_quintic
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

HYBRID_XML = PKG_ROOT / "models" / "pmi_hybrid_no_collision.xml"
ARM_ONLY_XML = PKG_ROOT / "models" / "pmi_arm_only_no_collision.xml"
OUT_DIR = PKG_ROOT / "debug_outputs" / "hybrid_no_collision"
ARM_ONLY_IK_CSV = (
    PKG_ROOT / "debug_outputs" / "no_collision_tracking" / "ik_joint_vsd_no_collision" / "ik_joint_vsd_no_collision.csv"
)

J = ["jnt1", "jnt2", "jnt3", "jnt4"]
ACT = ["q1_act", "q2_act", "q3_act", "q4_act"]
RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=float)
Q_ACT_INITIAL = np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=float)
Q_JNT_INITIAL = RATIOS * Q_ACT_INITIAL

WPS = [
    np.array([0.25, -0.20, -0.10], dtype=float),
    np.array([0.00, -0.35, -0.15], dtype=float),
    np.array([-0.25, -0.20, -0.10], dtype=float),
]

KQ = np.array([80.0, 80.0, 60.0, 40.0])
DQ = np.array([10.0, 10.0, 8.0, 5.0])
KX = 80.0
DX = 5.0

DURATIONS = [3.0, 5.0]
TAU_LIMITS = [20.0, 50.0]
SITE = "end_effector"


def load_hybrid_torque_only() -> mj.MjModel:
    """디스크 XML은 유지; 컴파일 시 위치 액추에이터만 제거."""
    spec = mj.MjSpec.from_file(str(HYBRID_XML))
    for a in list(spec.actuators)[::-1]:
        spec.delete(a)
    return spec.compile()


def solve_wp_q_arm_only() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    model = load_mjmodel(ARM_ONLY_XML, strip_position_actuators=True)
    scratch = mj.MjData(model)
    q_lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in J])
    q_hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in J])
    ik = IKConfig((1, 1, 1), 1.0, 1.0, 1e-3, 1e-4, 80, 1e-9, tuple(J))
    q_seed = Q_JNT_INITIAL.copy()
    qs: list[np.ndarray] = []
    for wp in WPS:
        q, _ = solve_ik_task_mode(
            model,
            scratch,
            wp,
            roll_des=-np.pi / 2,
            pitch_des=0.0,
            task_feas_mode="xyz",
            ik=ik,
            q_seed=q_seed,
            bounds_lo=q_lo,
            bounds_hi=q_hi,
        )
        qs.append(q.copy())
        q_seed = q.copy()
    return qs[0], qs[1], qs[2]


def _addrs(model: mj.MjModel) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    q_j = np.array([int(model.jnt_qposadr[joint_id(model, n)]) for n in J], dtype=int)
    d_j = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in J], dtype=int)
    q_a = np.array([int(model.jnt_qposadr[joint_id(model, n)]) for n in ACT], dtype=int)
    return q_j, d_j, q_a


def _site_xyz(model: mj.MjModel, data: mj.MjData) -> np.ndarray:
    sid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, SITE)
    return np.array(data.site_xpos[sid], dtype=float).copy()


def _joint_margin(model: mj.MjModel, q: np.ndarray) -> float:
    lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in J])
    hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in J])
    return float(np.min(np.minimum(hi - q, q - lo)))


def arm_only_baseline() -> dict[tuple[float, float], dict]:
    out: dict[tuple[float, float], dict] = {}
    if not ARM_ONLY_IK_CSV.is_file():
        return out
    by_key: dict[tuple[float, float], list[float]] = {}
    with open(ARM_ONLY_IK_CSV, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            d = float(row["duration"])
            tl = float(row["tau_limit"])
            if d not in DURATIONS or tl not in TAU_LIMITS:
                continue
            by_key.setdefault((d, tl), []).append(float(row["ee_err_norm"]))
    for key, errs in by_key.items():
        e = np.asarray(errs, dtype=float)
        out[key] = {"rms_ee": float(np.sqrt(np.mean(e**2))), "final_ee": float(e[-1])}
    return out


def run_experiment(csv_path: Path) -> list[dict]:
    model = load_hybrid_torque_only()
    data = mj.MjData(model)
    scratch = mj.MjData(model)
    jac_w = mj.MjData(model)
    qadr_j, dadr_j, qadr_a = _addrs(model)
    dt = float(model.opt.timestep)

    qwp0, qwp1, qwp2 = solve_wp_q_arm_only()

    fieldnames = (
        ["run_id", "step", "time", "controller_name", "duration", "tau_limit"]
        + ["des_ee_x", "des_ee_y", "des_ee_z", "act_ee_x", "act_ee_y", "act_ee_z", "ee_err_norm"]
        + [f"q_jnt_des_{J[i]}" for i in range(4)]
        + [f"q_jnt_actual_{J[i]}" for i in range(4)]
        + [f"q_act_actual_{i+1}" for i in range(4)]
        + [f"ratio_q_act_{i+1}" for i in range(4)]
        + [f"trans_err_{J[i]}" for i in range(4)]
        + [f"tau_bias_{J[i]}" for i in range(4)]
        + [f"tau_jnt_before_clip_{J[i]}" for i in range(4)]
        + [f"tau_cmd_jnt_{J[i]}" for i in range(4)]
        + [f"qfrc_applied_{J[i]}" for i in range(4)]
        + [f"qfrc_constraint_{J[i]}" for i in range(4)]
        + ["ncon", "saturation_flag", "joint_limit_margin"]
    )

    summaries: list[dict] = []
    run_id = 0
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    with open(csv_path, "w", newline="", encoding="utf-8") as fcsv:
        w = csv.DictWriter(fcsv, fieldnames=list(fieldnames))
        w.writeheader()

        for duration in DURATIONS:
            jpath = scaled_joint_quintic(qwp0, qwp1, qwp2, float(duration))
            cpath = scaled_joint_quintic(WPS[0], WPS[1], WPS[2], float(duration))
            for tau_lim in TAU_LIMITS:
                for ctrl_name, ctrl_kind in [("IK_joint_space_VSD", "ik"), ("JTF_task_space_VSD", "jtf")]:
                    data.qpos[:] = 0.0
                    data.qvel[:] = 0.0
                    for i in range(4):
                        data.qpos[int(qadr_a[i])] = float(Q_ACT_INITIAL[i])
                        data.qpos[int(qadr_j[i])] = float(Q_JNT_INITIAL[i])
                    mj.mj_forward(model, data)
                    data.qfrc_applied[:] = 0.0

                    n = int(round(float(duration) / dt)) + 1
                    ee_seq: list[float] = []
                    trans_seq: list[float] = []
                    sat_steps = 0
                    ncon_max = 0
                    jl_viol = 0

                    for step_i in range(n):
                        t = min(step_i * dt, float(duration))
                        q_jnt_ref, qdot_jnt_ref, _ = jpath.sample(float(t))
                        q_des = q_jnt_ref
                        qdot_des = qdot_jnt_ref

                        if ctrl_kind == "ik":
                            x_ee_des, *_ = fk_ee_rp(model, scratch, q_des, J)
                            x_ee_des = np.asarray(x_ee_des, dtype=float).reshape(3)
                        else:
                            x_cart_des, xdot_cart_des, _ = cpath.sample(float(t))
                            x_cart_des = np.asarray(x_cart_des, dtype=float).reshape(3)
                            x_ee_des = x_cart_des.copy()

                        q_j = np.array([float(data.qpos[int(qadr_j[k])]) for k in range(4)])
                        qd_j = np.array([float(data.qvel[int(dadr_j[k])]) for k in range(4)])
                        q_a = np.array([float(data.qpos[int(qadr_a[k])]) for k in range(4)])
                        rq = RATIOS * q_a
                        trans_e = q_j - rq
                        trans_norm = float(np.linalg.norm(trans_e))

                        mj.mj_forward(model, data)
                        tau_bias = np.array([float(data.qfrc_bias[int(dadr_j[k])]) for k in range(4)])

                        if ctrl_kind == "ik":
                            tau_pd = KQ * (q_des - q_j) + DQ * (qdot_des - qd_j)
                            tau_bc = tau_bias + tau_pd
                        else:
                            x_act = _site_xyz(model, data)
                            jac_w.qpos[:] = data.qpos
                            jac_w.qvel[:] = data.qvel
                            mj.mj_forward(model, jac_w)
                            Jp = compute_task_jacobian_mode(
                                model,
                                jac_w,
                                joint_names=J,
                                task_mode="xyz",
                                ee_site_name=SITE,
                                mode="mujoco_analytic",
                                epsilon=1e-6,
                            )
                            xdot_act = Jp @ qd_j
                            F = float(KX) * (x_cart_des - x_act) + float(DX) * (xdot_cart_des - xdot_act)
                            tau_bc = tau_bias + Jp.T @ F

                        tau_cmd = np.clip(tau_bc, -float(tau_lim), float(tau_lim))
                        sat = bool(np.any(np.abs(tau_bc - tau_cmd) > 1e-9))
                        if sat:
                            sat_steps += 1

                        data.qfrc_applied[:] = 0.0
                        for k in range(4):
                            data.qfrc_applied[int(dadr_j[k])] = float(tau_cmd[k])

                        jl_m = _joint_margin(model, q_j)
                        if jl_m < -1e-9:
                            jl_viol += 1

                        x_act_ee = _site_xyz(model, data)
                        ee_err = float(np.linalg.norm(x_ee_des - x_act_ee))
                        ee_seq.append(ee_err)
                        trans_seq.append(trans_norm)

                        mj.mj_step(model, data)

                        ncon_max = max(ncon_max, int(data.ncon))
                        qfc_j = np.array([float(data.qfrc_constraint[int(dadr_j[k])]) for k in range(4)])

                        w.writerow(
                            {
                                "run_id": run_id,
                                "step": step_i,
                                "time": t,
                                "controller_name": ctrl_name,
                                "duration": float(duration),
                                "tau_limit": float(tau_lim),
                                "des_ee_x": float(x_ee_des[0]),
                                "des_ee_y": float(x_ee_des[1]),
                                "des_ee_z": float(x_ee_des[2]),
                                "act_ee_x": float(x_act_ee[0]),
                                "act_ee_y": float(x_act_ee[1]),
                                "act_ee_z": float(x_act_ee[2]),
                                "ee_err_norm": ee_err,
                                **{f"q_jnt_des_{J[k]}": float(q_des[k]) for k in range(4)},
                                **{f"q_jnt_actual_{J[k]}": float(q_j[k]) for k in range(4)},
                                **{f"q_act_actual_{k+1}": float(q_a[k]) for k in range(4)},
                                **{f"ratio_q_act_{k+1}": float(rq[k]) for k in range(4)},
                                **{f"trans_err_{J[k]}": float(trans_e[k]) for k in range(4)},
                                **{f"tau_bias_{J[k]}": float(tau_bias[k]) for k in range(4)},
                                **{f"tau_jnt_before_clip_{J[k]}": float(tau_bc[k]) for k in range(4)},
                                **{f"tau_cmd_jnt_{J[k]}": float(tau_cmd[k]) for k in range(4)},
                                **{f"qfrc_applied_{J[k]}": float(tau_cmd[k]) for k in range(4)},
                                **{f"qfrc_constraint_{J[k]}": float(qfc_j[k]) for k in range(4)},
                                "ncon": int(data.ncon),
                                "saturation_flag": int(sat),
                                "joint_limit_margin": jl_m,
                            }
                        )

                    ee_a = np.asarray(ee_seq, dtype=float)
                    tr_a = np.asarray(trans_seq, dtype=float)
                    summaries.append(
                        {
                            "run_id": run_id,
                            "controller_name": ctrl_name,
                            "duration": float(duration),
                            "tau_limit": float(tau_lim),
                            "rms_ee": float(np.sqrt(np.mean(ee_a**2))),
                            "final_ee": float(ee_a[-1]),
                            "rms_trans_err": float(np.sqrt(np.mean(tr_a**2))),
                            "max_trans_err": float(np.max(tr_a)),
                            "saturation_steps": sat_steps,
                            "ncon_max": ncon_max,
                            "joint_limit_viol_steps": jl_viol,
                        }
                    )
                    run_id += 1

    return summaries


def write_report(path: Path, summaries: list[dict], baseline: dict[tuple[float, float], dict]) -> None:
    ik = [s for s in summaries if "IK" in s["controller_name"]]
    jtf = [s for s in summaries if "JTF" in s["controller_name"]]
    ncon_all = all(s["ncon_max"] == 0 for s in summaries)
    trans_ok = all(s["max_trans_err"] < 0.01 for s in summaries)

    def pick_best(pool: list[dict]) -> dict | None:
        if not pool:
            return None

        def key(r: dict) -> tuple:
            return (float(r["final_ee"]), float(r["rms_ee"]), int(r["saturation_steps"]))

        return min(pool, key=key)

    best_ik = pick_best(ik)
    best_jtf = pick_best(jtf)
    if best_ik is None and best_jtf is None:
        better = "n/a"
    elif best_jtf is None:
        better = "IK_joint_space_VSD"
    elif best_ik is None:
        better = "JTF_task_space_VSD"
    else:
        ti = (float(best_ik["final_ee"]), float(best_ik["rms_ee"]), int(best_ik["saturation_steps"]))
        tj = (float(best_jtf["final_ee"]), float(best_jtf["rms_ee"]), int(best_jtf["saturation_steps"]))
        better = "IK_joint_space_VSD" if ti < tj else "JTF_task_space_VSD"

    lines = [
        "# Hybrid joint-side torque VSD (no collision)",
        "",
        "- 모델: `models/pmi_hybrid_no_collision.xml` (런타임에 위치 액추에이터만 제거하여 `ctrl` 미사용)",
        "- 토크: `data.qfrc_applied` 는 **jnt1~4 dof** 만. `q_act` 는 equality로 동기.",
        "",
        "## Answers",
        "",
        "### 1. Does hybrid no-collision preserve the arm-only VSD tracking performance?",
        "- 동일한 IK 궤적·이득이라도 **하이브리드(모터 관성·등식) 추가**로 수치는 arm-only와 완전 일치하지 않을 수 있습니다. 아래 표와 arm-only CSV를 비교하세요.",
        "",
        "### 2–3. Equality stability & transmission error",
        f"- 스윕 전체 `max transmission ‖·‖` 상한: {max(s['max_trans_err'] for s in summaries):.4e} rad, "
        f"RMS 상한: {max(s['rms_trans_err'] for s in summaries):.4e}.",
        f"- **등식이 수치적으로 안정적으로 유지되는지:** {'예(임계 0.01 rad 미만)' if trans_ok else '일부 구간 재확인'}",
        "",
        "### 4. Is data.ncon always zero?",
        f"- {'예' if ncon_all else '아니오'} (ncon_max per run in table).",
        "",
        "### 5. Which controller works better?",
        f"- 동일 (duration, tau_limit)에서 최종/ RMS EE 기준으로는 **{better}** 쪽이 유리한 경우가 많았습니다(세부는 표).",
        "",
        "### 6. Ready for actuator-side ideal torque transmission?",
        "- 관절측 토크 제어·등식 전달이 본 스윕에서 허용 오차 내라면, **다음 단계로 액추에이터 축 토크 맵**을 설계 시 테스트할 수 있습니다. 케이블·백래시는 아직 포함하지 않습니다.",
        "",
        "## Summary table",
        "",
        "| run | controller | T | τ_lim | rms_ee | final_ee | rms_trans | max_trans | sat | ncon | jl |",
        "|-----|------------|---|-------|--------|----------|-----------|-----------|-----|------|----|",
    ]
    for s in summaries:
        lines.append(
            f"| {s['run_id']} | {s['controller_name']} | {s['duration']} | {s['tau_limit']} | "
            f"{s['rms_ee']:.6f} | {s['final_ee']:.6f} | {s['rms_trans_err']:.4e} | {s['max_trans_err']:.4e} | "
            f"{s['saturation_steps']} | {s['ncon_max']} | {s['joint_limit_viol_steps']} |"
        )

    lines.extend(["", "## Arm-only IK baseline (same duration / tau_limit, if CSV exists)", ""])
    if not baseline:
        lines.append("- CSV 없음: `debug_outputs/no_collision_tracking/ik_joint_vsd_no_collision/ik_joint_vsd_no_collision.csv`")
    else:
        lines.append("| duration | tau_limit | arm_only_rms_ee | arm_only_final_ee |")
        lines.append("|----------|-----------|-----------------|-------------------|")
        for key in sorted(baseline):
            b = baseline[key]
            lines.append(f"| {key[0]} | {key[1]} | {b['rms_ee']:.6f} | {b['final_ee']:.6f} |")

    lines.append("")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR)
    args = ap.parse_args()
    out = Path(args.out_dir)
    csv_path = out / "hybrid_joint_side_vsd.csv"
    rep_path = out / "hybrid_joint_side_vsd_report.md"

    sums = run_experiment(csv_path)
    write_report(rep_path, sums, arm_only_baseline())
    print(f"Wrote {csv_path}")
    print(f"Wrote {rep_path}")


if __name__ == "__main__":
    main()
