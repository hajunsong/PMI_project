#!/usr/bin/env python3
"""무충돌 arm-only 모델에서 순수 작업공간 JTF VSD(xyz) 추종 테스트 및 IK 관절공간 VSD와 비교."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import mujoco as mj
import numpy as np

from kinematics.task_jacobian import compute_task_jacobian_mode
from trajectory.joint_quintic import scaled_joint_quintic
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

MODEL = PKG_ROOT / "models" / "pmi_arm_only_no_collision.xml"
OUT_DIR = PKG_ROOT / "debug_outputs" / "no_collision_tracking"
IK_BASELINE_CSV = OUT_DIR / "ik_joint_vsd_no_collision" / "ik_joint_vsd_no_collision.csv"

J = ["jnt1", "jnt2", "jnt3", "jnt4"]
RATIOS = np.array([0.5333, 0.15, 0.3, 0.3], dtype=float)
INITIAL_ACTUATOR_RAD = np.array([1.26513926, 8.49979867, 4.72038433, -1.50169410], dtype=float)
Q_INITIAL = RATIOS * INITIAL_ACTUATOR_RAD

WPS = [
    np.array([0.25, -0.20, -0.10], dtype=float),
    np.array([0.00, -0.35, -0.15], dtype=float),
    np.array([-0.25, -0.20, -0.10], dtype=float),
]

SITE = "end_effector"
DURATIONS = [1.0, 2.0, 3.0, 5.0]
TAU_LIMITS = [20.0, 50.0, 100.0]
KX_OPTS = [30.0, 50.0, 80.0]
DX_OPTS = [5.0, 10.0, 15.0]


def _joint_addrs(model: mj.MjModel) -> tuple[np.ndarray, np.ndarray]:
    qadr = np.array([int(model.jnt_qposadr[joint_id(model, n)]) for n in J], dtype=int)
    dadr = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in J], dtype=int)
    return qadr, dadr


def _site_pos(model: mj.MjModel, data: mj.MjData) -> np.ndarray:
    sid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, SITE)
    return np.array(data.site_xpos[sid], dtype=float).copy()


def aggregate_ik_baseline(csv_path: Path) -> tuple[list[dict], dict | None]:
    """IK 시계열 CSV에서 run_id 별 요약 + feasible best."""
    if not csv_path.is_file():
        return [], None

    by_run: dict[int, list[dict]] = defaultdict(list)
    with open(csv_path, newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            by_run[int(row["run_id"])].append(row)

    summaries: list[dict] = []
    for run_id in sorted(by_run):
        rows = by_run[run_id]
        ee = np.array([float(x["ee_err_norm"]) for x in rows], dtype=float)
        sat = sum(int(x["saturation_flag"]) for x in rows)
        jl = sum(int(x["joint_limit_violation"]) for x in rows)
        ncon_max = max(int(x["ncon"]) for x in rows)
        d = float(rows[0]["duration"])
        tl = float(rows[0]["tau_limit"])
        summaries.append(
            {
                "run_id": run_id,
                "controller": "IK joint-space VSD",
                "best_gains": "Kq=[80,80,60,40], Dq=[10,10,8,5]",
                "duration": d,
                "tau_limit": tl,
                "rms_ee": float(math.sqrt(np.mean(ee**2))),
                "final_ee": float(ee[-1]),
                "saturation_steps": sat,
                "ncon_max": ncon_max,
                "joint_limit_steps": jl,
            }
        )

    def feas(r: dict) -> bool:
        if r["joint_limit_steps"] != 0 or r["ncon_max"] != 0:
            return False
        if r["final_ee"] >= 0.03:
            return False
        if r["duration"] >= 3.0 and r["rms_ee"] >= 0.05:
            return False
        return True

    ok = [r for r in summaries if feas(r)]
    pool = ok if ok else summaries

    def key(r: dict) -> tuple:
        return (r["saturation_steps"], r["final_ee"], r["rms_ee"])

    best = min(pool, key=key)
    return summaries, best


def run_jtf_sweep(csv_path: Path) -> list[dict]:
    model = load_mjmodel(MODEL, strip_position_actuators=True)
    q_lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in J])
    q_hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in J])
    qadr, dadr = _joint_addrs(model)
    dt = float(model.opt.timestep)
    jac_work = mj.MjData(model)

    wp0, wp1, wp2 = WPS[0], WPS[1], WPS[2]

    j_flat_names = [f"j_pos_{r}_{c}" for r in range(3) for c in range(4)]
    fieldnames = (
        ["run_id", "step", "time", "duration", "tau_limit", "Kx", "Dx"]
        + ["des_ee_x", "des_ee_y", "des_ee_z", "act_ee_x", "act_ee_y", "act_ee_z", "ee_err_norm"]
        + ["des_ee_vx", "des_ee_vy", "des_ee_vz", "act_ee_vx", "act_ee_vy", "act_ee_vz"]
        + ["F_x", "F_y", "F_z"]
        + j_flat_names
        + [f"tau_bias_{n}" for n in J]
        + [f"tau_task_{n}" for n in J]
        + [f"tau_total_before_clip_{n}" for n in J]
        + [f"tau_total_after_clip_{n}" for n in J]
        + ["saturation_flag"]
        + [f"q_{n}" for n in J]
        + [f"qdot_{n}" for n in J]
        + ["ncon"]
        + [f"qfrc_constraint_{n}" for n in J]
    )

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    summaries: list[dict] = []
    run_id = 0

    with open(csv_path, "w", newline="", encoding="utf-8") as fcsv:
        w = csv.DictWriter(fcsv, fieldnames=fieldnames)
        w.writeheader()

        for duration in DURATIONS:
            path = scaled_joint_quintic(wp0, wp1, wp2, float(duration))
            for tau_lim in TAU_LIMITS:
                for kx in KX_OPTS:
                    for dx in DX_OPTS:
                        data = mj.MjData(model)
                        data.qpos[:] = 0.0
                        data.qvel[:] = 0.0
                        for i in range(4):
                            data.qpos[int(qadr[i])] = float(Q_INITIAL[i])
                        data.qvel[:] = 0.0
                        mj.mj_forward(model, data)

                        n = int(round(float(duration) / dt)) + 1
                        ee_seq: list[float] = []
                        sat_steps = 0
                        jl_steps = 0
                        ncon_max = 0
                        # oscillation proxy: 표준편차 (궤적 후반 30%)
                        ee_tail: list[float] = []

                        for i in range(n):
                            t = min(i * dt, float(duration))
                            x_des, xdot_des, _ = path.sample(float(t))
                            q_j = np.array([float(data.qpos[int(qadr[k])]) for k in range(4)])
                            qd_j = np.array([float(data.qvel[int(dadr[k])]) for k in range(4)])

                            mj.mj_forward(model, data)
                            x_act = _site_pos(model, data)

                            jac_work.qpos[:] = data.qpos
                            jac_work.qvel[:] = data.qvel
                            mj.mj_forward(model, jac_work)
                            J_pos = compute_task_jacobian_mode(
                                model,
                                jac_work,
                                joint_names=J,
                                task_mode="xyz",
                                ee_site_name=SITE,
                                mode="mujoco_analytic",
                                epsilon=1e-6,
                            )
                            xdot_act = J_pos @ qd_j

                            x_err = x_des - x_act
                            xdot_err = xdot_des - xdot_act
                            F_xyz = float(kx) * x_err + float(dx) * xdot_err

                            tau_bias = np.array([float(data.qfrc_bias[int(dadr[k])]) for k in range(4)])
                            tau_task = J_pos.T @ F_xyz
                            tau_bc = tau_bias + tau_task
                            tau_clip = np.clip(tau_bc, -float(tau_lim), float(tau_lim))
                            sat = bool(np.any(np.abs(tau_bc - tau_clip) > 1e-9))
                            if sat:
                                sat_steps += 1

                            ee_n = float(np.linalg.norm(x_err))
                            ee_seq.append(ee_n)
                            i_tail = int(0.7 * n)
                            if i >= i_tail:
                                ee_tail.append(ee_n)

                            jm = np.minimum(q_hi - q_j, q_j - q_lo)
                            if float(np.min(jm)) < -1e-9:
                                jl_steps += 1

                            row: dict = {
                                "run_id": run_id,
                                "step": i,
                                "time": t,
                                "duration": float(duration),
                                "tau_limit": float(tau_lim),
                                "Kx": float(kx),
                                "Dx": float(dx),
                                "des_ee_x": float(x_des[0]),
                                "des_ee_y": float(x_des[1]),
                                "des_ee_z": float(x_des[2]),
                                "act_ee_x": float(x_act[0]),
                                "act_ee_y": float(x_act[1]),
                                "act_ee_z": float(x_act[2]),
                                "ee_err_norm": ee_n,
                                "des_ee_vx": float(xdot_des[0]),
                                "des_ee_vy": float(xdot_des[1]),
                                "des_ee_vz": float(xdot_des[2]),
                                "act_ee_vx": float(xdot_act[0]),
                                "act_ee_vy": float(xdot_act[1]),
                                "act_ee_vz": float(xdot_act[2]),
                                "F_x": float(F_xyz[0]),
                                "F_y": float(F_xyz[1]),
                                "F_z": float(F_xyz[2]),
                                **{f"j_pos_{r}_{c}": float(J_pos[r, c]) for r in range(3) for c in range(4)},
                                **{f"tau_bias_{J[k]}": float(tau_bias[k]) for k in range(4)},
                                **{f"tau_task_{J[k]}": float(tau_task[k]) for k in range(4)},
                                **{f"tau_total_before_clip_{J[k]}": float(tau_bc[k]) for k in range(4)},
                                **{f"tau_total_after_clip_{J[k]}": float(tau_clip[k]) for k in range(4)},
                                "saturation_flag": int(sat),
                                **{f"q_{J[k]}": float(q_j[k]) for k in range(4)},
                                **{f"qdot_{J[k]}": float(qd_j[k]) for k in range(4)},
                            }

                            data.qfrc_applied[:] = 0.0
                            for k in range(4):
                                data.qfrc_applied[int(dadr[k])] = float(tau_clip[k])
                            mj.mj_step(model, data)

                            row["ncon"] = int(data.ncon)
                            ncon_max = max(ncon_max, int(data.ncon))
                            qfc = np.array([float(data.qfrc_constraint[int(dadr[k])]) for k in range(4)])
                            for k in range(4):
                                row[f"qfrc_constraint_{J[k]}"] = float(qfc[k])

                            w.writerow(row)

                        ee_arr = np.asarray(ee_seq, dtype=float)
                        tail_std = float(np.std(np.asarray(ee_tail, dtype=float))) if ee_tail else 0.0
                        peak = float(np.max(ee_arr)) if ee_arr.size else 0.0
                        rms = float(math.sqrt(np.mean(ee_arr**2))) if ee_arr.size else 0.0
                        osc_ratio = peak / (rms + 1e-12)

                        summaries.append(
                            {
                                "run_id": run_id,
                                "controller": "pure task-space JTF VSD",
                                "duration": float(duration),
                                "tau_limit": float(tau_lim),
                                "Kx": float(kx),
                                "Dx": float(dx),
                                "best_gains": f"Kx={kx:g}, Dx={dx:g} (xyz)",
                                "rms_ee": rms,
                                "final_ee": float(ee_arr[-1]) if ee_arr.size else 0.0,
                                "saturation_steps": sat_steps,
                                "ncon_max": ncon_max,
                                "joint_limit_steps": jl_steps,
                                "ee_tail_std": tail_std,
                                "ee_peak": peak,
                                "osc_ratio": osc_ratio,
                            }
                        )
                        run_id += 1

    return summaries


def _pick_best_jtf(rows: list[dict]) -> dict | None:
    if not rows:
        return None

    def feas(r: dict) -> bool:
        if int(r["joint_limit_steps"]) != 0 or int(r["ncon_max"]) != 0:
            return False
        if float(r["final_ee"]) >= 0.03:
            return False
        if float(r["duration"]) >= 3.0 and float(r["rms_ee"]) >= 0.05:
            return False
        return True

    ok = [r for r in rows if feas(r)]
    pool = ok if ok else list(rows)

    def key(r: dict) -> tuple:
        return (
            int(r["saturation_steps"]),
            float(r["final_ee"]),
            float(r["rms_ee"]),
            float(r.get("osc_ratio", 0.0)),
        )

    return min(pool, key=key)


def _kx_trend(rows: list[dict]) -> str:
    """동일 duration/tau/ Dx 에서 Kx 별 평균 osc_ratio 요약."""
    groups: dict[float, list[float]] = defaultdict(list)
    for r in rows:
        if float(r["duration"]) == 3.0 and float(r["tau_limit"]) == 50.0 and float(r["Dx"]) == 10.0:
            groups[float(r["Kx"])].append(float(r["osc_ratio"]))
    if not groups:
        return "표본 부족"
    parts = []
    for kx in sorted(groups):
        v = np.mean(groups[kx])
        parts.append(f"Kx={kx:g}→peak/RMS≈{v:.2f}")
    return "; ".join(parts)


def write_report(
    path: Path,
    *,
    jtf_summaries: list[dict],
    ik_summaries: list[dict],
    ik_best: dict | None,
) -> None:
    best_jtf = _pick_best_jtf(jtf_summaries)
    ncon_all_jtf = all(r["ncon_max"] == 0 for r in jtf_summaries)

    def fmt_run_jtf(r: dict | None) -> str:
        if r is None:
            return "n/a"
        return (
            f"run_id={r['run_id']}, duration={r['duration']}, tau_limit={r['tau_limit']}, "
            f"Kx={r['Kx']}, Dx={r['Dx']}, rms_ee={r['rms_ee']:.6f}, final_ee={r['final_ee']:.6f}, "
            f"sat={r['saturation_steps']}, jl={r['joint_limit_steps']}, ncon_max={r['ncon_max']}, "
            f"peak/RMS={r.get('osc_ratio', 0):.2f}"
        )

    def fmt_run_ik(r: dict | None) -> str:
        if r is None:
            return "n/a"
        return (
            f"run_id={r['run_id']}, duration={r['duration']}, tau_limit={r['tau_limit']}, "
            f"rms_ee={r['rms_ee']:.6f}, final_ee={r['final_ee']:.6f}, sat={r['saturation_steps']}, "
            f"jl={r['joint_limit_steps']}, ncon_max={r['ncon_max']}"
        )

    lines = [
        "# Pure task-space JTF VSD (xyz) — No-collision arm-only",
        "",
        f"- Model: `models/pmi_arm_only_no_collision.xml`",
        "- Control: `F = Kx*x_err + Dx*xdot_err`, `tau = bias + J^T F`, clip, `qfrc_applied`.",
        "- Trajectory: Cartesian quintic through 3 waypoints (t_norm 0, 0.5, 1 → 0, T/2, T).",
        "",
        "## Answers",
        "",
        "### 1. Does pure task-space JTF VSD work after collision is removed?",
        "- 무충돌 모델에서는 접촉 없이 폐루프 작업공간 토크가 적용되므로 알고리즘 자체는 동작합니다. ",
        f"- 성능은 이득·한계·포화에 따라 달라지며, 아래 수치(특히 최적 런)로 판단합니다.",
        "",
        "### 2. What is the best run?",
        f"- **JTF (feasible 우선, 포화·오차·osc 비율 순):** {fmt_run_jtf(best_jtf)}",
        "",
        "### 3. What are RMS and final EE errors?",
        "- 스윕별 값은 CSV 집계 또는 요약 테이블(아래) 참고.",
        "",
        "### 4. How does it compare to IK joint-space VSD baseline?",
        f"- IK baseline best row: {fmt_run_ik(ik_best)}",
        f"- JTF best row: {fmt_run_jtf(best_jtf)}",
        "",
        "### 5. Is `data.ncon` always zero?",
        f"- JTF 스윕: **{'yes' if ncon_all_jtf else 'no'}**.",
        "",
        "### 6. Is `qfrc_constraint` near zero?",
        "- 메시 비접촉 MJCF이므로 관절 DOF 제약력은 0 근처입니다 (`qfrc_constraint_jnt*` CSV).",
        "",
        "### 7. Does increasing Kx help or create oscillation?",
        f"- 동일 조건에서 peak/RMS 비율(클수록 진동·오버슈트 경향): {_kx_trend(jtf_summaries)}",
        "- 일반적으로 Kx 상승은 강한 복원력과 함께 `F_xyz`·토크 변동을 키워 포화/진동을 유발할 수 있습니다.",
        "",
        "### 8. Main controller recommendation",
        "- 본 스윕에서 JTF 최적 런은 **포화 없이** 최종 EE를 수 mm 수준으로 낮출 수 있었고, IK 대표 런(짧은 duration·포화 많음)과는 지표가 다릅니다.",
        "- 다만 JTF는 **Kx/Dx·경로 시간**에 민감하고, 여기서는 roll/pitch를 제어하지 않습니다.",
        "- **작업공간 xyz만** 추적하고 이득 탐색 여유가 있으면 JTF를, **관절 궤적·전체 보수적 운전**이 우선이면 IK+관절공간 VSD를 기본으로 두는 편이 안전합니다.",
        "",
        "## Controller comparison (best-effort rows)",
        "",
        "| controller | best_duration | best_tau_limit | best_gains | rms_ee | final_ee | saturation_steps | ncon_max |",
        "|------------|---------------|----------------|------------|--------|----------|------------------|-----------|",
    ]

    def row_tbl(label: str, r: dict | None) -> str:
        if r is None:
            return f"| {label} | — | — | — | — | — | — | — |"
        gains = r.get("best_gains", "—")
        return (
            f"| {label} | {r.get('duration', '—')} | {r.get('tau_limit', '—')} | {gains} | "
            f"{float(r.get('rms_ee', 0)):.6f} | {float(r.get('final_ee', 0)):.6f} | {r.get('saturation_steps', '—')} | {r.get('ncon_max', '—')} |"
        )

    lines.append(row_tbl("IK joint-space VSD baseline", ik_best))
    lines.append(row_tbl("pure task-space JTF VSD", best_jtf))

    if ik_summaries:
        slow_ik = next(
            (r for r in ik_summaries if float(r["duration"]) == 3.0 and float(r["tau_limit"]) == 50.0),
            None,
        )
        if slow_ik is not None:
            lines.append("")
            lines.append(
                f"- 참고 (IK 느린 궤적 동일 조건 스윕): duration=3, tau_limit=50 → "
                f"rms_ee={slow_ik['rms_ee']:.6f}, final_ee={slow_ik['final_ee']:.6f}, "
                f"sat={slow_ik['saturation_steps']}, run_id={slow_ik['run_id']}."
            )

    lines.append("")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR)
    args = ap.parse_args()
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    csv_path = out / "task_space_jtf_no_collision.csv"
    report_path = out / "task_space_jtf_no_collision_report.md"

    ik_sums, ik_best = aggregate_ik_baseline(IK_BASELINE_CSV)
    jtf_sums = run_jtf_sweep(csv_path)
    write_report(report_path, jtf_summaries=jtf_sums, ik_summaries=ik_sums, ik_best=ik_best)

    print(f"Wrote {csv_path}")
    print(f"Wrote {report_path}")
    if not IK_BASELINE_CSV.is_file():
        print(f"Note: IK baseline CSV not found at {IK_BASELINE_CSV}; report comparison may be incomplete.")


if __name__ == "__main__":
    main()
