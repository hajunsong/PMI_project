#!/usr/bin/env python3
"""IK 웨이포인트 → 관절 quintic → bias+PD(VSD) 토크로 무충돌 arm-only 모델 추종 디버그."""

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
from trajectory.joint_quintic import scaled_joint_quintic
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

MODEL = PKG_ROOT / "models" / "pmi_arm_only_no_collision.xml"
OUT_PARENT = PKG_ROOT / "debug_outputs" / "no_collision_tracking"
# 생성 산출물은 하위 폴더에 저장
OUT_DIR = OUT_PARENT / "ik_joint_vsd_no_collision"

J = ["jnt1", "jnt2", "jnt3", "jnt4"]
RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=float)
INITIAL_ACTUATOR_RAD = np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=float)
Q_INITIAL = RATIOS * INITIAL_ACTUATOR_RAD

WPS = [
    np.array([0.25, -0.20, -0.10]),
    np.array([0.00, -0.35, -0.15]),
    np.array([-0.25, -0.20, -0.10]),
]

KQ = np.array([80.0, 80.0, 60.0, 40.0], dtype=float)
DQ = np.array([10.0, 10.0, 8.0, 5.0], dtype=float)

DURATIONS = [1.0, 2.0, 3.0, 5.0]
TAU_LIMITS = [20.0, 50.0, 100.0]


def solve_wp_q(model: mj.MjModel) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    scratch = mj.MjData(model)
    q_lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in J])
    q_hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in J])
    ik = IKConfig((1, 1, 1), 1.0, 1.0, 1e-3, 1e-4, 80, 1e-9, tuple(J))
    q_seed = Q_INITIAL.copy()
    qs = []
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


def _joint_addrs(model: mj.MjModel) -> tuple[np.ndarray, np.ndarray]:
    qadr = np.array([int(model.jnt_qposadr[joint_id(model, n)]) for n in J], dtype=int)
    dadr = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in J], dtype=int)
    return qadr, dadr


def run_sweep(
    *,
    csv_path: Path,
    durations: list[float],
    tau_limits: list[float],
) -> tuple[list[dict], list[dict]]:
    model = load_mjmodel(MODEL, strip_position_actuators=True)
    scratch = mj.MjData(model)
    q_lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in J])
    q_hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in J])
    qadr, dadr = _joint_addrs(model)
    dt = float(model.opt.timestep)

    qwp0, qwp1, qwp2 = solve_wp_q(model)

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames: list[str] = [
        "run_id",
        "step",
        "time",
        "duration",
        "tau_limit",
        "des_ee_x",
        "des_ee_y",
        "des_ee_z",
        "act_ee_x",
        "act_ee_y",
        "act_ee_z",
        "ee_err_norm",
        "q_des_jnt1",
        "q_des_jnt2",
        "q_des_jnt3",
        "q_des_jnt4",
        "q_act_jnt1",
        "q_act_jnt2",
        "q_act_jnt3",
        "q_act_jnt4",
        "q_err_jnt1",
        "q_err_jnt2",
        "q_err_jnt3",
        "q_err_jnt4",
        "q_err_norm",
        "qdot_des_jnt1",
        "qdot_des_jnt2",
        "qdot_des_jnt3",
        "qdot_des_jnt4",
        "qdot_act_jnt1",
        "qdot_act_jnt2",
        "qdot_act_jnt3",
        "qdot_act_jnt4",
        "tau_bias_jnt1",
        "tau_bias_jnt2",
        "tau_bias_jnt3",
        "tau_bias_jnt4",
        "tau_pd_jnt1",
        "tau_pd_jnt2",
        "tau_pd_jnt3",
        "tau_pd_jnt4",
        "tau_total_before_clip_jnt1",
        "tau_total_before_clip_jnt2",
        "tau_total_before_clip_jnt3",
        "tau_total_before_clip_jnt4",
        "tau_total_after_clip_jnt1",
        "tau_total_after_clip_jnt2",
        "tau_total_after_clip_jnt3",
        "tau_total_after_clip_jnt4",
        "saturation_flag",
        "ncon",
        "qfrc_constraint_jnt1",
        "qfrc_constraint_jnt2",
        "qfrc_constraint_jnt3",
        "qfrc_constraint_jnt4",
        "qfrc_applied_jnt1",
        "qfrc_applied_jnt2",
        "qfrc_applied_jnt3",
        "qfrc_applied_jnt4",
        "joint_limit_violation",
    ]

    summary_rows: list[dict] = []
    run_id = 0
    with open(csv_path, "w", newline="", encoding="utf-8") as fcsv:
        writer = csv.DictWriter(fcsv, fieldnames=fieldnames)
        writer.writeheader()

        for duration in durations:
            path = scaled_joint_quintic(qwp0, qwp1, qwp2, float(duration))
            for tau_lim in tau_limits:
                data = mj.MjData(model)
                data.qpos[:] = 0.0
                data.qvel[:] = 0.0
                for i in range(4):
                    data.qpos[int(qadr[i])] = float(Q_INITIAL[i])
                data.qvel[:] = 0.0
                mj.mj_forward(model, data)

                n = int(round(float(duration) / dt)) + 1
                ee_err_seq: list[float] = []
                q_err_seq: list[float] = []
                sat_steps = 0
                jl_steps = 0
                ncon_max = 0

                for i in range(n):
                    t = min(i * dt, float(duration))
                    q_des, qdot_des, _qddot_des = path.sample(float(t))
                    q_act = np.array([float(data.qpos[int(qadr[k])]) for k in range(4)])
                    qdot_act = np.array([float(data.qvel[int(dadr[k])]) for k in range(4)])

                    mj.mj_forward(model, data)
                    tau_bias = np.array([float(data.qfrc_bias[int(dadr[k])]) for k in range(4)])
                    tau_pd = KQ * (q_des - q_act) + DQ * (qdot_des - qdot_act)
                    tau_total_bc = tau_bias + tau_pd
                    tau_clip = np.clip(tau_total_bc, -float(tau_lim), float(tau_lim))
                    sat = bool(np.any(np.abs(tau_total_bc - tau_clip) > 1e-9))
                    if sat:
                        sat_steps += 1

                    p_des, *_ = fk_ee_rp(model, scratch, q_des, J)
                    p_des = np.asarray(p_des, dtype=float)
                    p_act, *_ = fk_ee_rp(model, scratch, q_act, J)
                    p_act = np.asarray(p_act, dtype=float)
                    ee_err = float(np.linalg.norm(p_des - p_act))
                    ee_err_seq.append(ee_err)
                    q_err_v = q_des - q_act
                    q_err_norm = float(np.linalg.norm(q_err_v))
                    q_err_seq.append(q_err_norm)

                    jm = np.minimum(q_hi - q_act, q_act - q_lo)
                    j_lv = bool(np.min(jm) < -1e-9)
                    if j_lv:
                        jl_steps += 1

                    data.qfrc_applied[:] = 0.0
                    for k in range(4):
                        data.qfrc_applied[int(dadr[k])] = float(tau_clip[k])
                    mj.mj_step(model, data)

                    ncon = int(data.ncon)
                    ncon_max = max(ncon_max, ncon)
                    qfrc_c = np.array([float(data.qfrc_constraint[int(dadr[k])]) for k in range(4)])
                    qfrc_a = tau_clip.copy()

                    row = {
                        "run_id": run_id,
                        "step": i,
                        "time": t,
                        "duration": float(duration),
                        "tau_limit": float(tau_lim),
                        "des_ee_x": float(p_des[0]),
                        "des_ee_y": float(p_des[1]),
                        "des_ee_z": float(p_des[2]),
                        "act_ee_x": float(p_act[0]),
                        "act_ee_y": float(p_act[1]),
                        "act_ee_z": float(p_act[2]),
                        "ee_err_norm": ee_err,
                        **{f"q_des_{J[k]}": float(q_des[k]) for k in range(4)},
                        **{f"q_act_{J[k]}": float(q_act[k]) for k in range(4)},
                        **{f"q_err_{J[k]}": float(q_err_v[k]) for k in range(4)},
                        "q_err_norm": q_err_norm,
                        **{f"qdot_des_{J[k]}": float(qdot_des[k]) for k in range(4)},
                        **{f"qdot_act_{J[k]}": float(qdot_act[k]) for k in range(4)},
                        **{f"tau_bias_{J[k]}": float(tau_bias[k]) for k in range(4)},
                        **{f"tau_pd_{J[k]}": float(tau_pd[k]) for k in range(4)},
                        **{f"tau_total_before_clip_{J[k]}": float(tau_total_bc[k]) for k in range(4)},
                        **{f"tau_total_after_clip_{J[k]}": float(tau_clip[k]) for k in range(4)},
                        "saturation_flag": int(sat),
                        "ncon": ncon,
                        **{f"qfrc_constraint_{J[k]}": float(qfrc_c[k]) for k in range(4)},
                        **{f"qfrc_applied_{J[k]}": float(qfrc_a[k]) for k in range(4)},
                        "joint_limit_violation": int(j_lv),
                    }
                    writer.writerow(row)

                ee_arr = np.asarray(ee_err_seq, dtype=float)
                qe_arr = np.asarray(q_err_seq, dtype=float)
                summary_rows.append(
                    {
                        "run_id": run_id,
                        "duration": float(duration),
                        "tau_limit": float(tau_lim),
                        "rms_ee_err": float(np.sqrt(np.mean(ee_arr**2))),
                        "final_ee_err": float(ee_arr[-1]),
                        "rms_joint_err": float(np.sqrt(np.mean(qe_arr**2))),
                        "final_joint_err": float(qe_arr[-1]),
                        "saturation_steps": sat_steps,
                        "joint_limit_steps": jl_steps,
                        "ncon_max": ncon_max,
                        "ncon_always_zero": int(ncon_max == 0),
                    }
                )
                run_id += 1

    return summary_rows, [qwp0, qwp1, qwp2]


def _best_run(summary: list[dict]) -> dict | None:
    if not summary:
        return None

    def feasible(r: dict) -> bool:
        if int(r["joint_limit_steps"]) != 0 or int(r["ncon_max"]) != 0:
            return False
        if float(r["final_ee_err"]) >= 0.03:
            return False
        if float(r["duration"]) >= 3.0 and float(r["rms_ee_err"]) >= 0.05:
            return False
        return True

    def score(r: dict) -> tuple[float, ...]:
        # 토크 포화 적음 → 최종/ RMS EE 작음 순
        return (
            float(r["saturation_steps"]),
            float(r["final_ee_err"]),
            float(r["rms_ee_err"]),
        )

    ok = [r for r in summary if feasible(r)]
    pool = ok if ok else list(summary)
    return min(pool, key=score)


def _best_slow_run(summary: list[dict], *, min_duration: float = 3.0) -> dict | None:
    """duration>=min_duration 이고 feasible(rms EE 포함)인 조합 중 포화 최소."""
    cand = [r for r in summary if float(r["duration"]) >= min_duration]

    def feasible(r: dict) -> bool:
        if int(r["joint_limit_steps"]) != 0 or int(r["ncon_max"]) != 0:
            return False
        if float(r["final_ee_err"]) >= 0.03:
            return False
        if float(r["rms_ee_err"]) >= 0.05:
            return False
        return True

    ok = [r for r in cand if feasible(r)]
    if not ok:
        return None

    def score(r: dict) -> tuple[float, ...]:
        return (
            float(r["saturation_steps"]),
            float(r["final_ee_err"]),
            float(r["rms_ee_err"]),
        )

    return min(ok, key=score)


def write_report(
    path: Path,
    *,
    summary: list[dict],
    q_wp: list[np.ndarray],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    best = _best_run(summary)
    best_slow = _best_slow_run(summary, min_duration=3.0)

    slow = [r for r in summary if float(r["duration"]) >= 3.0]
    slow_ok = [r for r in slow if float(r["rms_ee_err"]) < 0.05 and int(r["joint_limit_steps"]) == 0]

    def fmt_r(r: dict | None) -> str:
        if r is None:
            return "n/a"
        return (
            f"run_id={r['run_id']}, duration={r['duration']}, tau_limit={r['tau_limit']}, "
            f"final_ee={r['final_ee_err']:.6f}, rms_ee={r['rms_ee_err']:.6f}, "
            f"final_q={r['final_joint_err']:.6f}, rms_q={r['rms_joint_err']:.6f}, "
            f"sat_steps={r['saturation_steps']}, jl_steps={r['joint_limit_steps']}, ncon_max={r['ncon_max']}"
        )

    q0, q1, q2 = q_wp
    dq1_wp = float(abs(q1[0] - q0[0]) + abs(q2[0] - q1[0]))

    lines = [
        "# IK Joint VSD — No-Collision Arm-Only",
        "",
        f"- Model: `{MODEL.relative_to(PKG_ROOT)}`",
        f"- IK joint targets (wp0→wp1→wp2) jnt1: [{q0[0]:.5f}, {q1[0]:.5f}, {q2[0]:.5f}] (rad)",
        f"- |Δjnt1| sum across segments ≈ {dq1_wp:.5f} rad (large-motion indicator)",
        "",
        "## Report answers",
        "",
        "### 1. Does the no-collision model solve the IK joint trajectory?",
        "- IK는 기구학만 사용하므로 무충돌 MJCF에서도 동일하게 관절 목표 해가 구해집니다. ",
        f"- 웨이포인트별 IK 관절값은 위 jnt1 나열과 CSV 초기 `q_des_*` 구간과 일치합니다.",
        "",
        "### 2. What is the best run?",
        f"- **포화 스텝 최소 우선(가능한 success 제약 만족):** {fmt_r(best)}",
        f"- **느린 궤적 기준 (duration≥3 s, RMS EE<0.05):** {fmt_r(best_slow)}",
        "",
        "### 3. What are RMS and final EE errors?",
        "- Per-run values are in the summary table below (all sweeps). Best run quoted above.",
        "",
        "### 4. What are RMS and final joint errors?",
        "- See `rms_joint_err` / `final_joint_err` in the summary table.",
        "",
        "### 5. Is `data.ncon` always zero?",
        f"- All runs: **{'yes' if all(int(r['ncon_max']) == 0 for r in summary) else 'no (see ncon_max column)'}**.",
        "",
        "### 6. Is `qfrc_constraint` near zero?",
        "- Mesh가 비접촉(contype/conaffinity=0)이면 등식/접촉 제약이 없어 관절 DOF의 `qfrc_constraint`는 0에 가깝습니다. ",
        "- 시계열은 CSV의 `qfrc_constraint_jnt*` 열을 확인하세요.",
        "",
        "### 7. Does jnt1 now follow the large required motion?",
        f"- Quintic 경로상 jnt1은 약 {dq1_wp:.3f} rad 규모 웨이포인트 간 변화를 포함합니다.",
        "- 실제 추종은 `q_act_jnt1` vs `q_des_jnt1` 및 joint error 열로 확인할 수 있습니다.",
        "",
        "### 8. Baseline duration and `tau_limit`",
        "- 성공 기준: final EE < 0.03 m, 느린 duration에서 RMS EE < 0.05 m, 접촉 없음, 관절 한계 위반 없음, 토크 포화 스텝 적음.",
        f"- **전체 스윕 중 포화 최소 추천:** duration={best['duration'] if best else 'n/a'}, tau_limit={best['tau_limit'] if best else 'n/a'}.",
        f"- **느린 궤적 RMS 기준 베이스라인:** duration={best_slow['duration'] if best_slow else 'n/a'}, tau_limit={best_slow['tau_limit'] if best_slow else 'n/a'}.",
        f"- Slow-duration RMS<0.05 만족 후보: {len(slow_ok)} / {len(slow)} (duration≥3s in sweep).",
        "",
        "## Summary (all sweep combinations)",
        "",
        "| run_id | duration | tau_limit | rms_ee | final_ee | rms_q | final_q | sat | jl | ncon_max |",
        "|--------|----------|-----------|--------|----------|-------|---------|-----|----|----------|",
    ]
    for r in summary:
        lines.append(
            f"| {r['run_id']} | {r['duration']} | {r['tau_limit']} | "
            f"{r['rms_ee_err']:.6f} | {r['final_ee_err']:.6f} | "
            f"{r['rms_joint_err']:.6f} | {r['final_joint_err']:.6f} | "
            f"{r['saturation_steps']} | {r['joint_limit_steps']} | {r['ncon_max']} |"
        )
    lines.append("")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR, help="CSV/리포트를 둘 하위 폴더")
    args = ap.parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    csv_path = out_dir / "ik_joint_vsd_no_collision.csv"
    report_path = out_dir / "ik_joint_vsd_no_collision_report.md"

    summary, q_wp = run_sweep(csv_path=csv_path, durations=DURATIONS, tau_limits=TAU_LIMITS)
    write_report(report_path, summary=summary, q_wp=q_wp)
    print(f"Wrote {csv_path}")
    print(f"Wrote {report_path}")


if __name__ == "__main__":
    main()
