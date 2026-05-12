#!/usr/bin/env python3
"""하이브리드 무충돌: IK 관절 VSD 토크를 액추에이터 축에 tau_act = ratio ⊙ tau_jnt 로 전달(이상 전력 일관)."""

from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import mujoco as mj
import numpy as np

from kinematics.forward_kinematics import fk_ee_rp
from kinematics.inverse_kinematics import IKConfig, solve_ik_task_mode
from trajectory.joint_quintic import scaled_joint_quintic
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

HYBRID_XML = PKG_ROOT / "models" / "pmi_hybrid_no_collision.xml"
ARM_ONLY_XML = PKG_ROOT / "models" / "pmi_arm_only_no_collision.xml"
OUT_DIR = PKG_ROOT / "debug_outputs" / "hybrid_no_collision"
JOINT_SIDE_CSV = OUT_DIR / "hybrid_joint_side_vsd.csv"

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

DURATIONS = [3.0, 5.0]
TAU_JNT_LIMITS = [20.0, 50.0]
SITE = "end_effector"


def load_hybrid_torque_only() -> mj.MjModel:
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
    d_a = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in ACT], dtype=int)
    q_a = np.array([int(model.jnt_qposadr[joint_id(model, n)]) for n in ACT], dtype=int)
    return q_j, d_j, d_a, q_a


def _site_xyz(model: mj.MjModel, data: mj.MjData) -> np.ndarray:
    sid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, SITE)
    return np.array(data.site_xpos[sid], dtype=float).copy()


def _jnt_margin(model: mj.MjModel, q: np.ndarray) -> float:
    lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in J])
    hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in J])
    return float(np.min(np.minimum(hi - q, q - lo)))


def _act_margin(model: mj.MjModel, q: np.ndarray) -> float:
    lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in ACT])
    hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in ACT])
    return float(np.min(np.minimum(hi - q, q - lo)))


def load_joint_side_ik_baseline() -> list[dict]:
    """hybrid_joint_side_vsd.csv 에서 IK 관절측만 집계."""
    if not JOINT_SIDE_CSV.is_file():
        return []
    by_run: dict[int, list[dict]] = defaultdict(list)
    with open(JOINT_SIDE_CSV, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row.get("controller_name") != "IK_joint_space_VSD":
                continue
            by_run[int(row["run_id"])].append(row)
    rows: list[dict] = []
    for rid in sorted(by_run):
        r0 = by_run[rid][0]
        du = float(r0["duration"])
        tl = float(r0["tau_limit"])
        if du not in DURATIONS or tl not in TAU_JNT_LIMITS:
            continue
        ee = []
        je = []
        trans = []
        sat = 0
        ncon_max = 0
        jl = 0
        al = 0
        for row in by_run[rid]:
            ee.append(float(row["ee_err_norm"]))
            qd = np.array([float(row[f"q_jnt_des_{n}"]) for n in J])
            qa = np.array([float(row[f"q_jnt_actual_{n}"]) for n in J])
            je.append(float(np.linalg.norm(qd - qa)))
            te = np.array([float(row[f"trans_err_{n}"]) for n in J])
            trans.append(float(np.linalg.norm(te)))
            sat += int(row["saturation_flag"])
            ncon_max = max(ncon_max, int(row["ncon"]))
            if float(row["joint_limit_margin"]) < -1e-9:
                jl += 1
        ee_a = np.asarray(ee, dtype=float)
        je_a = np.asarray(je, dtype=float)
        tr_a = np.asarray(trans, dtype=float)
        rows.append(
            {
                "mode": "joint_side_IK_VSD",
                "duration": du,
                "tau_limit": tl,
                "rms_ee": float(np.sqrt(np.mean(ee_a**2))),
                "final_ee": float(ee_a[-1]),
                "rms_joint_error": float(np.sqrt(np.mean(je_a**2))),
                "final_joint_error": float(je_a[-1]),
                "max_transmission_error": float(np.max(tr_a)),
                "saturation_steps": sat,
                "ncon_max": ncon_max,
                "joint_limit_steps": jl,
                "actuator_limit_steps": al,
            }
        )
    return rows


def run_actuator_side(csv_path: Path) -> list[dict]:
    model = load_hybrid_torque_only()
    data = mj.MjData(model)
    scratch = mj.MjData(model)
    qadr_j, dadr_j, dadr_a, qadr_a = _addrs(model)
    dt = float(model.opt.timestep)

    qwp0, qwp1, qwp2 = solve_wp_q_arm_only()

    fieldnames = (
        ["run_id", "step", "time", "duration", "tau_jnt_limit"]
        + [f"q_jnt_des_{n}" for n in J]
        + [f"q_jnt_actual_{n}" for n in J]
        + [f"q_jnt_error_{n}" for n in J]
        + [f"q_act_actual_{i+1}" for i in range(4)]
        + [f"ratio_q_act_{i+1}" for i in range(4)]
        + [f"trans_err_{n}" for n in J]
        + [f"qdot_jnt_des_{n}" for n in J]
        + [f"qdot_jnt_actual_{n}" for n in J]
        + [f"qdot_act_actual_{i+1}" for i in range(4)]
        + [f"tau_bias_{n}" for n in J]
        + [f"tau_pd_{n}" for n in J]
        + [f"tau_jnt_before_clip_{n}" for n in J]
        + [f"tau_jnt_after_clip_{n}" for n in J]
        + [f"tau_act_cmd_{i+1}" for i in range(4)]
        + [f"qfrc_applied_q{i+1}_act" for i in range(4)]
        + [f"qfrc_applied_{n}" for n in J]
        + [f"qfrc_constraint_{n}" for n in J]
        + [f"qfrc_constraint_q{i+1}_act" for i in range(4)]
        + [
            "des_ee_x",
            "des_ee_y",
            "des_ee_z",
            "act_ee_x",
            "act_ee_y",
            "act_ee_z",
            "ee_err_norm",
            "ncon",
            "saturation_flag",
            "joint_limit_margin",
            "actuator_limit_margin",
        ]
    )

    summaries: list[dict] = []
    run_id = 0
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    with open(csv_path, "w", newline="", encoding="utf-8") as fcsv:
        w = csv.DictWriter(fcsv, fieldnames=list(fieldnames))
        w.writeheader()

        for duration in DURATIONS:
            jpath = scaled_joint_quintic(qwp0, qwp1, qwp2, float(duration))
            for tau_lim in TAU_JNT_LIMITS:
                data.qpos[:] = 0.0
                data.qvel[:] = 0.0
                for i in range(4):
                    data.qpos[int(qadr_a[i])] = float(Q_ACT_INITIAL[i])
                    data.qpos[int(qadr_j[i])] = float(Q_JNT_INITIAL[i])
                mj.mj_forward(model, data)
                data.qfrc_applied[:] = 0.0

                n = int(round(float(duration) / dt)) + 1
                ee_seq: list[float] = []
                je_seq: list[float] = []
                trans_seq: list[float] = []
                sat_steps = 0
                ncon_max = 0
                jl_steps = 0
                al_steps = 0
                max_qfc_j = 0.0
                max_qfc_a = 0.0

                for step_i in range(n):
                    t = min(step_i * dt, float(duration))
                    q_des, qd_des, _ = jpath.sample(float(t))
                    q_j = np.array([float(data.qpos[int(qadr_j[k])]) for k in range(4)])
                    qd_j = np.array([float(data.qvel[int(dadr_j[k])]) for k in range(4)])
                    q_a = np.array([float(data.qpos[int(qadr_a[k])]) for k in range(4)])
                    qd_a = np.array([float(data.qvel[int(dadr_a[k])]) for k in range(4)])
                    rq = RATIOS * q_a
                    trans_e = q_j - rq
                    q_err = q_des - q_j

                    mj.mj_forward(model, data)
                    tau_bias = np.array([float(data.qfrc_bias[int(dadr_j[k])]) for k in range(4)])
                    tau_pd = KQ * (q_des - q_j) + DQ * (qd_des - qd_j)
                    tau_j_bc = tau_bias + tau_pd
                    tau_j_clip = np.clip(tau_j_bc, -float(tau_lim), float(tau_lim))
                    sat = bool(np.any(np.abs(tau_j_bc - tau_j_clip) > 1e-9))
                    if sat:
                        sat_steps += 1

                    tau_act = RATIOS * tau_j_clip

                    data.qfrc_applied[:] = 0.0
                    for k in range(4):
                        data.qfrc_applied[int(dadr_a[k])] = float(tau_act[k])

                    jl_m = _jnt_margin(model, q_j)
                    al_m = _act_margin(model, q_a)
                    if jl_m < -1e-9:
                        jl_steps += 1
                    if al_m < -1e-9:
                        al_steps += 1

                    x_des, *_ = fk_ee_rp(model, scratch, q_des, J)
                    x_des = np.asarray(x_des, dtype=float).reshape(3)
                    x_act = _site_xyz(model, data)
                    ee_err = float(np.linalg.norm(x_des - x_act))
                    ee_seq.append(ee_err)
                    je_seq.append(float(np.linalg.norm(q_err)))
                    trans_seq.append(float(np.linalg.norm(trans_e)))

                    mj.mj_step(model, data)

                    ncon_max = max(ncon_max, int(data.ncon))
                    qfa = np.array([float(data.qfrc_applied[int(dadr_a[k])]) for k in range(4)])
                    qfj = np.array([float(data.qfrc_applied[int(dadr_j[k])]) for k in range(4)])
                    qfc_j = np.array([float(data.qfrc_constraint[int(dadr_j[k])]) for k in range(4)])
                    qfc_a = np.array([float(data.qfrc_constraint[int(dadr_a[k])]) for k in range(4)])
                    max_qfc_j = max(max_qfc_j, float(np.max(np.abs(qfc_j))))
                    max_qfc_a = max(max_qfc_a, float(np.max(np.abs(qfc_a))))

                    w.writerow(
                        {
                            "run_id": run_id,
                            "step": step_i,
                            "time": t,
                            "duration": float(duration),
                            "tau_jnt_limit": float(tau_lim),
                            **{f"q_jnt_des_{J[k]}": float(q_des[k]) for k in range(4)},
                            **{f"q_jnt_actual_{J[k]}": float(q_j[k]) for k in range(4)},
                            **{f"q_jnt_error_{J[k]}": float(q_err[k]) for k in range(4)},
                            **{f"q_act_actual_{k+1}": float(q_a[k]) for k in range(4)},
                            **{f"ratio_q_act_{k+1}": float(rq[k]) for k in range(4)},
                            **{f"trans_err_{J[k]}": float(trans_e[k]) for k in range(4)},
                            **{f"qdot_jnt_des_{J[k]}": float(qd_des[k]) for k in range(4)},
                            **{f"qdot_jnt_actual_{J[k]}": float(qd_j[k]) for k in range(4)},
                            **{f"qdot_act_actual_{k+1}": float(qd_a[k]) for k in range(4)},
                            **{f"tau_bias_{J[k]}": float(tau_bias[k]) for k in range(4)},
                            **{f"tau_pd_{J[k]}": float(tau_pd[k]) for k in range(4)},
                            **{f"tau_jnt_before_clip_{J[k]}": float(tau_j_bc[k]) for k in range(4)},
                            **{f"tau_jnt_after_clip_{J[k]}": float(tau_j_clip[k]) for k in range(4)},
                            **{f"tau_act_cmd_{k+1}": float(tau_act[k]) for k in range(4)},
                            **{f"qfrc_applied_q{k+1}_act": float(qfa[k]) for k in range(4)},
                            **{f"qfrc_applied_{J[k]}": float(qfj[k]) for k in range(4)},
                            **{f"qfrc_constraint_{J[k]}": float(qfc_j[k]) for k in range(4)},
                            **{f"qfrc_constraint_q{k+1}_act": float(qfc_a[k]) for k in range(4)},
                            "des_ee_x": float(x_des[0]),
                            "des_ee_y": float(x_des[1]),
                            "des_ee_z": float(x_des[2]),
                            "act_ee_x": float(x_act[0]),
                            "act_ee_y": float(x_act[1]),
                            "act_ee_z": float(x_act[2]),
                            "ee_err_norm": ee_err,
                            "ncon": int(data.ncon),
                            "saturation_flag": int(sat),
                            "joint_limit_margin": jl_m,
                            "actuator_limit_margin": al_m,
                        }
                    )

                ee_a = np.asarray(ee_seq, dtype=float)
                je_a = np.asarray(je_seq, dtype=float)
                tr_a = np.asarray(trans_seq, dtype=float)
                summaries.append(
                    {
                        "mode": "actuator_side_ideal",
                        "duration": float(duration),
                        "tau_limit": float(tau_lim),
                        "rms_ee": float(np.sqrt(np.mean(ee_a**2))),
                        "final_ee": float(ee_a[-1]),
                        "rms_joint_error": float(np.sqrt(np.mean(je_a**2))),
                        "final_joint_error": float(je_a[-1]),
                        "max_transmission_error": float(np.max(tr_a)),
                        "saturation_steps": sat_steps,
                        "ncon_max": ncon_max,
                        "joint_limit_steps": jl_steps,
                        "actuator_limit_steps": al_steps,
                        "max_qfrc_constraint_jnt": max_qfc_j,
                        "max_qfrc_constraint_act": max_qfc_a,
                    }
                )
                run_id += 1

    return summaries


def write_report(
    path: Path,
    actuator_sums: list[dict],
    joint_sums: list[dict],
) -> None:
    def keyrow(r: dict) -> tuple:
        return (float(r["duration"]), float(r["tau_limit"]))

    jmap = {keyrow(r): r for r in joint_sums}
    amap = {keyrow(r): r for r in actuator_sums}

    all_ncon_zero = all(r["ncon_max"] == 0 for r in actuator_sums)
    max_trans_all = max(r["max_transmission_error"] for r in actuator_sums)
    trans_ok = max_trans_all < 1e-4

    lines = [
        "# Actuator-side ideal torque transmission (hybrid no-collision)",
        "",
        "- 관절 VSD: `tau_jnt = tau_bias + Kq e + Dq edot`, **관절 토크 먼저** `±tau_jnt_limit` 클립.",
        "- 액추에이터 누력: `tau_act = ratio ⊙ tau_jnt_clipped` (전력 일관: τ_joint·q̇_joint ≈ τ_act·q̇_act).",
        "- `data.qfrc_applied` 는 **q_act dof 만** (jnt dof 0). 위치 액추에이터는 런타임 제거.",
        "",
        "## Answers",
        "",
        "### 1. Does actuator-side ideal torque reproduce joint-side tracking?",
        "- 동일 (duration, τ_lim)에서 EE·관절 오차를 아래 표로 비교. 수치가 근접하면 **동일 제어 목표가 액추에이터 경유로도 재현**됩니다.",
        "",
        "### 2. Is τ_act = ratio ⊙ τ_jnt correct in this equality model?",
        "- 등식 `q_jnt = ratio ⊙ q_act` 에서 순간 전력 일치를 쓰면 τ_act = ratio ⊙ τ_jnt 가 됩니다. 전달 잔차가 통계적으로 0에 가깝지 않으면 매핑·제약 해석을 재점검합니다.",
        "",
        "### 3. Is transmission error still below 1e-4 rad?",
        f"- 스윕 전체 `max‖q_jnt−ratio⊙q_act‖` 최댓값: **{max_trans_all:.4e}** rad → {'예' if trans_ok else '일부 조합 초과'}",
        "",
        "### 4. Is data.ncon always zero?",
        f"- {'예' if all_ncon_zero else '아니오'}",
        "",
        "### 5. Are equality constraint forces reasonable?",
        f"- 액추에이터 런별 `max|qfrc_constraint|` (jnt / act) 상한: "
        f"{max(r['max_qfrc_constraint_jnt'] for r in actuator_sums):.3g} / "
        f"{max(r['max_qfrc_constraint_act'] for r in actuator_sums):.3g} (구속·중력·관성에 따른 정상 범위인지 로그로 확인).",
        "",
        "### 6. Does actuator-side torque require different gain scaling?",
        f"- 본 스크립트는 **같은 Kq,Dq** 를 유지. EE/관절 오차가 관절측 대비 유의미하게 악화되면 이득·토크 한계·클립 순서를 조정할 수 있습니다.",
        "",
        "### 7. Ready for ideal q1 gear + q2~q4 cable layer?",
        "- 이상 액추에이터 토크 전달이 위 표에서 검증되면, **q1 벨트/기어 및 q2~q4 케이블 층**을 별도 제약·손실 모델로 얹을 준비가 됩니다. 케이블 탄성/감쇠/백래시는 **아직 미포함**.",
        "",
        "## Comparison table (joint-side vs actuator-side)",
        "",
        "| mode | duration | τ_lim | rms_ee | final_ee | rms_q_err | final_q_err | max_trans | sat | ncon | jl | al | max|qfc| jnt/act |",
        "|------|----------|-------|--------|----------|-----------|-------------|-----------|-----|------|----|----|----------------------|",
    ]

    keys = sorted(set(jmap.keys()) | set(amap.keys()))
    for k in keys:
        ja = jmap.get(k)
        ac = amap.get(k)
        for label, r in [("joint_side_IK_VSD", ja), ("actuator_side_ideal", ac)]:
            if r is None:
                continue
            lines.append(
                f"| {label} | {r['duration']} | {r['tau_limit']} | {r['rms_ee']:.6f} | {r['final_ee']:.6f} | "
                f"{r['rms_joint_error']:.6f} | {r['final_joint_error']:.6f} | {r['max_transmission_error']:.4e} | "
                f"{r['saturation_steps']} | {r['ncon_max']} | {r['joint_limit_steps']} | {r['actuator_limit_steps']} | "
                f"{r.get('max_qfrc_constraint_jnt', float('nan')):.3g}/{r.get('max_qfrc_constraint_act', float('nan')):.3g} |"
            )

    lines.append("")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR)
    args = ap.parse_args()
    out = Path(args.out_dir)
    csv_path = out / "actuator_side_ideal_torque.csv"
    rep_path = out / "actuator_side_ideal_torque_report.md"

    act_sums = run_actuator_side(csv_path)
    joint_sums = load_joint_side_ik_baseline()
    write_report(rep_path, act_sums, joint_sums)

    print(f"Wrote {csv_path}")
    print(f"Wrote {rep_path}")
    if not joint_sums:
        print(f"Note: baseline {JOINT_SIDE_CSV} missing or empty; comparison table may be incomplete.")


if __name__ == "__main__":
    main()
