#!/usr/bin/env python3
"""Batch cable parameter randomization (q2–q4): CLI profile, outputs under randomization_<profile>/."""

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
import yaml

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from kinematics.forward_kinematics import fk_ee_rp
from kinematics.inverse_kinematics import IKConfig, solve_ik_task_mode
from trajectory.joint_quintic import scaled_joint_quintic
from transmission.hybrid_transmission import HybridTransmission
from transmission.randomization import RANGE_KEYS_ARRAY
from utils.mujoco_helpers import PKG_ROOT, joint_id, load_mjmodel

HYBRID_XML = PKG_ROOT / "models" / "pmi_hybrid_no_collision.xml"
CABLE_LAYER_DEFAULT = PKG_ROOT / "configs" / "cable_layer.yaml"
DEBUG_LAYER = PKG_ROOT / "debug_outputs" / "cable_layer"

# Published medium (50 trials, seed 0) totals for regression-style safety checks vs medium_v2.
MEDIUM_BASELINE_SAT_TOTAL = 51687
MEDIUM_BASELINE_MAX_SAT_FRAC = 0.7565
MEDIUM_BASELINE_JL_TOTAL = 753
MEDIUM_BASELINE_AL_TOTAL = 754

DEFAULT_TAU_JNT_LIMIT = 20.0

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
DURATION = 5.0
SITE = "end_effector"


def load_hybrid_torque_only() -> mj.MjModel:
    spec = mj.MjSpec.from_file(str(HYBRID_XML))
    for a in list(spec.actuators)[::-1]:
        spec.delete(a)
    return spec.compile()


def solve_wp_q_arm_only() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    arm_only = PKG_ROOT / "models" / "pmi_arm_only_no_collision.xml"
    model = load_mjmodel(arm_only, strip_position_actuators=True)
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


def _addrs(model: mj.MjModel) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
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


def run_trial(
    hybrid: HybridTransmission,
    *,
    trial_seed: int,
    profile_name: str,
    qwp0: np.ndarray,
    qwp1: np.ndarray,
    qwp2: np.ndarray,
    tau_jnt_limit: float,
) -> tuple[dict, dict[str, np.ndarray]]:
    jpath = scaled_joint_quintic(qwp0, qwp1, qwp2, float(DURATION))
    hybrid.reset(randomize=True, seed=trial_seed, profile_name=profile_name)

    model = load_hybrid_torque_only()
    data = mj.MjData(model)
    scratch = mj.MjData(model)
    qadr_j, dadr_j, dadr_a, qadr_a = _addrs(model)
    dt = float(model.opt.timestep)

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    for i in range(4):
        data.qpos[int(qadr_a[i])] = float(Q_ACT_INITIAL[i])
        data.qpos[int(qadr_j[i])] = float(Q_JNT_INITIAL[i])
    mj.mj_forward(model, data)
    data.qfrc_applied[:] = 0.0

    n = int(round(float(DURATION) / dt)) + 1
    sat_steps = 0
    jl_viol = 0
    al_viol = 0
    ncon_max = 0
    ee_seq: list[float] = []
    jnorm_seq: list[float] = []
    tau_out_max = 0.0
    max_hys_z = 0.0
    max_tau_hys = 0.0
    max_tau_loss = 0.0
    max_tau_tr = 0.0

    tr = {
        "t": [],
        "ee_err": [],
        "des_x": [],
        "des_y": [],
        "des_z": [],
        "act_x": [],
        "act_y": [],
        "act_z": [],
    }

    q_err_last = np.zeros(4)

    for step_i in range(n):
        t = min(step_i * dt, float(DURATION))
        q_des, qd_des, _ = jpath.sample(float(t))
        q_j = np.array([float(data.qpos[int(qadr_j[k])]) for k in range(4)])
        qd_j = np.array([float(data.qvel[int(dadr_j[k])]) for k in range(4)])
        q_a = np.array([float(data.qpos[int(qadr_a[k])]) for k in range(4)])
        qd_a = np.array([float(data.qvel[int(dadr_a[k])]) for k in range(4)])
        q_err_last = q_des - q_j
        jnorm_seq.append(float(np.linalg.norm(q_err_last)))

        mj.mj_forward(model, data)
        tau_bias = np.array([float(data.qfrc_bias[int(dadr_j[k])]) for k in range(4)])
        tau_pd = KQ * (q_des - q_j) + DQ * (qd_des - qd_j)
        tau_j_unc = tau_bias + tau_pd
        tau_j_cmd = np.clip(tau_j_unc, -float(tau_jnt_limit), float(tau_jnt_limit))
        if bool(np.any(np.abs(tau_j_unc - tau_j_cmd) > 1e-9)):
            sat_steps += 1

        tau_act_ideal = RATIOS * tau_j_cmd
        tau_act_out = hybrid.transmit(tau_act_ideal, dt, q_a, qd_a)
        cr = hybrid.last_cable_result
        assert cr is not None
        tau_tr = np.abs(tau_act_out - tau_act_ideal)
        max_tau_tr = max(max_tau_tr, float(np.max(tau_tr)))
        tau_out_max = max(tau_out_max, float(np.max(np.abs(tau_act_out))))

        max_hys_z = max(max_hys_z, float(np.max(np.abs(cr.hys_z))))
        max_tau_hys = max(max_tau_hys, float(np.max(np.abs(cr.tau_hys))))
        tl = np.abs(cr.tau_loss)
        max_tau_loss = max(max_tau_loss, float(np.max(tl)))

        data.qfrc_applied[:] = 0.0
        for k in range(4):
            data.qfrc_applied[int(dadr_a[k])] = float(tau_act_out[k])

        jl_m = _jnt_margin(model, q_j)
        al_m = _act_margin(model, q_a)
        if jl_m < -1e-9:
            jl_viol += 1
        if al_m < -1e-9:
            al_viol += 1

        x_des, *_ = fk_ee_rp(model, scratch, q_des, J)
        x_des = np.asarray(x_des, dtype=float).reshape(3)
        x_act = _site_xyz(model, data)
        ee_err = float(np.linalg.norm(x_des - x_act))
        ee_seq.append(ee_err)

        tr["t"].append(float(t))
        tr["ee_err"].append(ee_err)
        tr["des_x"].append(float(x_des[0]))
        tr["des_y"].append(float(x_des[1]))
        tr["des_z"].append(float(x_des[2]))
        tr["act_x"].append(float(x_act[0]))
        tr["act_y"].append(float(x_act[1]))
        tr["act_z"].append(float(x_act[2]))

        mj.mj_step(model, data)
        ncon_max = max(ncon_max, int(data.ncon))

    ee_a = np.asarray(ee_seq, dtype=float)
    jn = np.asarray(jnorm_seq, dtype=float)
    metrics = {
        "rms_ee": float(np.sqrt(np.mean(ee_a**2))),
        "final_ee": float(ee_a[-1]),
        "max_ee": float(np.max(ee_a)),
        "rms_joint_error": float(np.sqrt(np.mean(jn**2))),
        "final_joint_error": float(np.linalg.norm(q_err_last)),
        "max_tau_act_out": float(tau_out_max),
        "saturation_steps": int(sat_steps),
        "joint_limit_steps": int(jl_viol),
        "actuator_limit_steps": int(al_viol),
        "ncon_max": int(ncon_max),
        "max_abs_hys_z": float(max_hys_z),
        "max_abs_tau_hys": float(max_tau_hys),
        "max_abs_tau_loss": float(max_tau_loss),
        "max_abs_tau_transmission_error": float(max_tau_tr),
        "num_steps": int(n),
    }
    ta = {k: np.asarray(v, dtype=np.float64) for k, v in tr.items()}
    return metrics, ta


def _validate_ranges_in_row(row: dict, ranges: dict) -> bool:
    ok = True
    for j in ("q2_act", "q3_act", "q4_act"):
        for key, r in ranges.items():
            if key == "backlash_slope":
                continue
            lo, hi = float(r[0]), float(r[1])
            v = float(row[f"{j}_{key}"])
            if v < lo - 1e-8 or v > hi + 1e-8:
                ok = False
    slo, shi = float(ranges["backlash_slope"][0]), float(ranges["backlash_slope"][1])
    b = float(row["backlash_slope"])
    if b < slo - 1e-8 or b > shi + 1e-8:
        ok = False
    return ok


def _mean3(row: dict, key: str) -> float:
    return float(np.mean([float(row[f"q2_act_{key}"]), float(row[f"q3_act_{key}"]), float(row[f"q4_act_{key}"])]))


def _pear(x: np.ndarray, y: np.ndarray) -> float:
    if np.std(x) < 1e-15 or np.std(y) < 1e-15:
        return float("nan")
    return float(np.corrcoef(x, y)[0, 1])


def aggregate_trials(csv_path: Path) -> dict | None:
    if not csv_path.is_file():
        return None
    with open(csv_path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    rms = np.array([float(r["rms_ee"]) for r in rows])
    fin = np.array([float(r["final_ee"]) for r in rows])
    sat = np.array([float(r["saturation_steps"]) for r in rows])
    ns = np.array([float(r["num_steps"]) for r in rows])
    sf = sat / np.maximum(ns, 1.0)
    jl_s = sum(int(r["joint_limit_steps"]) for r in rows)
    al_s = sum(int(r["actuator_limit_steps"]) for r in rows)
    tau_lim = float(rows[0].get("tau_jnt_limit", DEFAULT_TAU_JNT_LIMIT))
    return {
        "profile": csv_path.parent.name.replace("randomization_", ""),
        "num_trials": len(rows),
        "tau_jnt_limit": tau_lim,
        "mean_rms_ee": float(np.mean(rms)),
        "worst_rms_ee": float(np.max(rms)),
        "mean_final_ee": float(np.mean(fin)),
        "worst_final_ee": float(np.max(fin)),
        "total_saturation_steps": int(np.sum(sat)),
        "max_saturation_fraction": float(np.max(sf)),
        "joint_limit_steps": jl_s,
        "actuator_limit_steps": al_s,
        "ncon_max": max(int(r["ncon_max"]) for r in rows),
        "max_abs_hys_z": max(float(r["max_abs_hys_z"]) for r in rows),
        "max_abs_tau_hys": max(float(r["max_abs_tau_hys"]) for r in rows),
    }


def comparison_table_rows() -> list[tuple[str, Path]]:
    """(row label, CSV path) in display order for randomization_profile_comparison.md."""

    d = DEBUG_LAYER
    return [
        ("mild", d / "randomization_mild" / "randomization_trials.csv"),
        ("medium_v2", d / "randomization_medium_v2" / "randomization_trials.csv"),
        ("medium_train_tau20", d / "randomization_medium_train_tau20" / "randomization_trials.csv"),
        ("medium_train_tau30", d / "randomization_medium_train_tau30" / "randomization_trials.csv"),
        ("medium_train_tau40", d / "randomization_medium_train_tau40" / "randomization_trials.csv"),
        ("medium", d / "randomization_medium" / "randomization_trials.csv"),
        ("stress", d / "randomization_stress" / "randomization_trials.csv"),
    ]


def recommendation_markdown_appendix(debug_layer: Path) -> str:
    """Answers Q1–Q6 for randomization_profile_comparison.md (uses on-disk CSV aggregates)."""

    ag_v2 = aggregate_trials(debug_layer / "randomization_medium_v2" / "randomization_trials.csv")
    ag_med = aggregate_trials(debug_layer / "randomization_medium" / "randomization_trials.csv")
    mt: dict[int, dict] = {}
    for lim in (20, 30, 40):
        ag = aggregate_trials(debug_layer / f"randomization_medium_train_tau{lim}" / "randomization_trials.csv")
        if ag is not None:
            mt[lim] = ag

    lines: list[str] = ["\n## Recommendation (aggregated)\n\n"]

    # Q1: harder than medium_v2 — compare best train tau by mean RMS vs v2
    if ag_v2 and mt:
        best_mt = max(mt.values(), key=lambda a: a["mean_rms_ee"])
        harder = best_mt["mean_rms_ee"] > ag_v2["mean_rms_ee"]
        lines.append("### 1. medium_train이 medium_v2보다 어렵나?\n\n")
        lines.append(
            f"- **{'예' if harder else '재검토'}** — `medium_v2` 평균 RMS ≈ **{ag_v2['mean_rms_ee']:.6f} m**, "
            f"medium_train τ 스윕 중 최대 평균 RMS ≈ **{best_mt['mean_rms_ee']:.6f} m** (동일 배치 규모 가정).\n\n"
        )
    else:
        lines.append(
            "### 1. medium_train이 medium_v2보다 어렵나?\n\n- **CSV 부족** — `medium_v2` 또는 `medium_train_tau*` 배치를 먼저 생성하세요.\n\n"
        )

    # Q2 safer than medium
    if ag_med and mt:
        any_train = mt[min(mt.keys())]
        safer = (
            any_train["joint_limit_steps"] + any_train["actuator_limit_steps"] == 0
            and ag_med["joint_limit_steps"] + ag_med["actuator_limit_steps"] > 0
        ) or (
            any_train["total_saturation_steps"] < ag_med["total_saturation_steps"]
            and any_train["joint_limit_steps"] + any_train["actuator_limit_steps"]
            <= ag_med["joint_limit_steps"] + ag_med["actuator_limit_steps"]
        )
        lines.append("### 2. medium_train이 medium보다 안전한가?\n\n")
        lines.append(
            f"- **{'예 — 리밋/포화가 medium 대비 완만한 편' if safer else '일부 조건 재확인'}** "
            f"(medium: jl+al 합 ≈ **{ag_med['joint_limit_steps'] + ag_med['actuator_limit_steps']}**, "
            f"총 포화 ≈ **{ag_med['total_saturation_steps']}**).\n\n"
        )
    else:
        lines.append("### 2. medium_train이 medium보다 안전한가?\n\n- **CSV 부족**\n\n")

    lines.append("### 3. τ_jnt,lim 선택: 20 / 30 / 40 중 무엇이 좋은가?\n\n")
    if not mt:
        lines.append("- **데이터 없음** — 세 폴더를 생성한 뒤 다시 `--refresh-comparison-only`를 실행하세요.\n\n")
    else:

        def _core_ok(ag: dict) -> bool:
            return bool(
                ag["joint_limit_steps"] == 0
                and ag["actuator_limit_steps"] == 0
                and ag["ncon_max"] == 0
                and 0.003 <= ag["mean_rms_ee"] <= 0.010
                and ag["worst_rms_ee"] < 0.025
                and ag["total_saturation_steps"] < MEDIUM_BASELINE_SAT_TOTAL
            )

        ag20 = mt.get(20)
        ag30 = mt.get(30)

        feasible = [lim for lim in (20, 30, 40) if lim in mt and _core_ok(mt[lim])]

        best_lim_feas: int | None = None
        if ag30 is not None and _core_ok(ag30):
            if ag20 is not None and (
                ag30["total_saturation_steps"] < ag20["total_saturation_steps"]
                and ag30["joint_limit_steps"] == 0
                and ag30["actuator_limit_steps"] == 0
            ):
                best_lim_feas = 30
        if best_lim_feas is None and feasible:
            best_lim_feas = min(
                feasible,
                key=lambda lim: (mt[lim]["total_saturation_steps"], mt[lim]["max_saturation_fraction"]),
            )

        tbl = "| τ limit | mean RMS | worst RMS | total sat | max sat frac | jl/al |\n|:---:|:---:|:---:|:---:|:---:|:---:|\n"
        for lim in sorted(mt.keys()):
            ag = mt[lim]
            tbl += (
                f"| **{lim}** | {ag['mean_rms_ee']:.6f} | {ag['worst_rms_ee']:.6f} | "
                f"{ag['total_saturation_steps']} | {ag['max_saturation_fraction']:.4f} | "
                f"{ag['joint_limit_steps']}/{ag['actuator_limit_steps']} |\n"
            )
        lines.extend([tbl, "\n"])

        if ag30 and ag20:
            good30 = (
                ag30["joint_limit_steps"] == 0
                and ag30["actuator_limit_steps"] == 0
                and ag30["total_saturation_steps"] < ag20["total_saturation_steps"]
            )
            msg30 = (
                " τ=20 대비 포화가 줄고 리밋 위반이 없으므로 **SAC 학습 토크 상한은 τ=30**을 우선 권장합니다."
                if good30
                else ""
            )
        else:
            msg30 = ""

        lines.append(
            f"- 선택 요약: **{'τ=' + str(best_lim_feas) if best_lim_feas is not None else '기준 미충족 — 튜닝 필요'}**.{msg30}\n\n"
        )

    # Q4 SAC training suitability — use tau30 aggregate if exists else tau20
    ref = mt.get(30) or mt.get(20) or (next(iter(mt.values())) if mt else None)
    lines.append("### 4. 첫 SAC 잔차 학습 분포로 medium_train 적합한가?\n\n")
    if ref is None:
        lines.append("- **판단 보류** — medium_train 배치 결과가 필요합니다.\n\n")
    else:
        sac = (
            0.003 <= ref["mean_rms_ee"] <= 0.010
            and ref["worst_rms_ee"] < 0.025
            and ref["joint_limit_steps"] == 0
            and ref["actuator_limit_steps"] == 0
            and ref["ncon_max"] == 0
            and ref["total_saturation_steps"] < MEDIUM_BASELINE_SAT_TOTAL
        )
        lines.append(
            f"- **{'적합 가능' if sac else '일부 패스 기준 미달'}** "
            f"(참조 run: 평균 RMS **{ref['mean_rms_ee']:.6f} m**, 최악 RMS **{ref['worst_rms_ee']:.6f} m**).\n\n"
        )

    lines.append(
        "### 5. medium은 평가 전용으로 유지할까?\n\n"
        "- **예** — 학습 분포는 `medium_train` / `medium_v2` 계열을 권장하고, `medium`은 **평가·스트레스에 가까운 과도한 교란**으로 둡니다.\n\n"
        "### 6. stress는 평가 전용인가?\n\n"
        "- **예** — `stress`는 **진단 및 드물게 재평가**용입니다.\n\n"
    )

    return "".join(lines)


def write_profile_comparison(debug_layer: Path) -> None:
    out_path = debug_layer / "randomization_profile_comparison.md"
    parts: list[str] = []
    parts.append("# Randomization profile comparison\n\n")
    parts.append(
        "| profile | tau_limit | mean_rms_ee | worst_rms_ee | mean_final_ee | worst_final_ee | "
        "total_saturation_steps | max_saturation_fraction | joint_limit_steps | actuator_limit_steps | "
        "ncon_max | max_abs_hys_z | max_abs_tau_hys |\n"
    )
    parts.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n")
    any_row = False
    for label, csv_p in comparison_table_rows():
        ag = aggregate_trials(csv_p)
        if ag is None:
            continue
        any_row = True
        tau = float(ag["tau_jnt_limit"])
        parts.append(
            f"| {label} | **{tau:.0f}** | {ag['mean_rms_ee']:.6f} | {ag['worst_rms_ee']:.6f} | "
            f"{ag['mean_final_ee']:.6f} | {ag['worst_final_ee']:.6f} | {ag['total_saturation_steps']} | "
            f"{ag['max_saturation_fraction']:.4f} | {ag['joint_limit_steps']} | {ag['actuator_limit_steps']} | "
            f"{ag['ncon_max']} | {ag['max_abs_hys_z']:.5g} | {ag['max_abs_tau_hys']:.5g} |\n"
        )
    if not any_row:
        parts.append("| — | — | _(no CSVs found)_ |\n")

    parts.append(
        "\n_Source: 각 `randomization_*/randomization_trials.csv` 집계; `tau_jnt_limit` 컬럼이 없으면 **20**으로 가정._\n"
    )
    parts.append(recommendation_markdown_appendix(debug_layer))
    out_path.write_text("".join(parts), encoding="utf-8")


def correlation_ranking(rows: list[dict]) -> list[tuple[str, float]]:
    rms = np.array([float(r["rms_ee"]) for r in rows])
    pairs: list[tuple[str, float]] = []
    for key in RANGE_KEYS_ARRAY:
        x = np.array([_mean3(r, key) for r in rows])
        pairs.append((f"mean({key})", _pear(x, rms)))
    xb = np.array([float(r["backlash_slope"]) for r in rows])
    pairs.append(("backlash_slope", _pear(xb, rms)))
    pairs.sort(key=lambda t: abs(t[1]) if np.isfinite(t[1]) else 0.0, reverse=True)
    return pairs


def _join_report_parts(parts: list[str]) -> str:
    """Join report fragments so markdown tables/lists render (crit_lines used raw '' gaps)."""

    chunks: list[str] = []
    for p in parts:
        if p == "":
            chunks.append("\n")
        elif p.endswith("\n"):
            chunks.append(p)
        else:
            chunks.append(p + "\n")
    return "".join(chunks)


def write_report(
    rows: list[dict],
    ranges: dict,
    z_max: float,
    tau_hys_max: float,
    worst_trace: dict[str, np.ndarray],
    worst_trial_id: int,
    profile: str,
    report_path: Path,
    plots_dir: Path,
    tau_jnt_limit: float = DEFAULT_TAU_JNT_LIMIT,
) -> None:
    n = len(rows)
    rms = np.array([float(r["rms_ee"]) for r in rows])
    fin = np.array([float(r["final_ee"]) for r in rows])
    ok_rng = all(_validate_ranges_in_row(r, ranges) for r in rows)
    ns = np.array([float(r["num_steps"]) for r in rows])
    sat = np.array([float(r["saturation_steps"]) for r in rows])
    sat_frac = sat / np.maximum(ns, 1.0)
    pairs = correlation_ranking(rows)
    corr_top = ", ".join(f"**{k}**: r={v:.4f}" for k, v in pairs[:5] if np.isfinite(v))

    total_sat = int(np.sum(sat))
    jl_t = sum(int(r["joint_limit_steps"]) for r in rows)
    al_t = sum(int(r["actuator_limit_steps"]) for r in rows)

    mild_ref_rms = 0.000734

    header = "# Cable parameter randomization batch\n\n"
    header += f"Profile: **`{profile}`** — **{n}** trials — τ_jnt,lim = **{tau_jnt_limit:g}** N·m.\n\n"

    if profile == "mild":
        crit_lines = [
            "## Pass criteria (mild)",
            "",
            "| criterion | threshold | result |",
            "|---|---|---|",
            f"| mean RMS EE | < 0.005 m | **{float(np.mean(rms)):.6f}** |",
            f"| worst RMS EE | < 0.02 m | **{float(np.max(rms)):.6f}** |",
            f"| saturation steps total | rare / 0 | **{total_sat}** |",
            f"| joint/act violations | 0 | **{jl_t + al_t}** |",
            f"| ncon max | 0 | **{max(int(r['ncon_max']) for r in rows)}** |",
            f"| max |z| | ≤ {z_max} | **{max(float(r['max_abs_hys_z']) for r in rows):.4g}** |",
            f"| max |τ_hys| | ≤ {tau_hys_max} | **{max(float(r['max_abs_tau_hys']) for r in rows):.4g}** |",
            "",
        ]
        extra_tail: list[str] = [
            "## Answers (mild)\n\n",
            "### All samples in configured ranges?\n",
            f"- **{'예' if ok_rng else '아니오'}**\n\n",
            "### Correlation (top mean q2–q4 vs RMS EE)\n\n",
            corr_top + "\n\n",
        ]
    elif profile == "medium":
        mean_rm = float(np.mean(rms))
        worst_rm = float(np.max(rms))
        moderate_mean = 0.001 <= mean_rm <= 0.02
        ok_w = worst_rm < 0.05
        mean_ns = float(np.mean(ns))
        ok_sat = float(np.max(sat_frac)) <= 0.2 and total_sat <= max(1, int(0.1 * mean_ns * n))
        ok_lim = jl_t == 0 and al_t == 0
        ok_ncon = max(int(r["ncon_max"]) for r in rows) == 0
        hz_ok = max(float(r["max_abs_hys_z"]) for r in rows) <= z_max + 1e-6
        th_ok = max(float(r["max_abs_tau_hys"]) for r in rows) <= tau_hys_max + 1e-6

        stab_note = "**안정**" if hz_ok and th_ok and ok_ncon and ok_lim else "**주의 — 지표 재확인**"

        crit_lines = [
            "## Pass criteria (medium — SAC residual guidance)",
            "",
            "| criterion | target | result | ok? |",
            "|---|---|---|---|",
            f"| mean RMS EE | 0.001–0.02 m | **{mean_rm:.6f}** | **{'예' if moderate_mean else '아니오'}** |",
            f"| worst RMS EE | < 0.05 m | **{worst_rm:.6f}** | **{'예' if ok_w else '아니오'}** |",
            f"| saturation | not excessive | max frac **{float(np.max(sat_frac))*100:.2f}%**, total **{total_sat}** | **{'예' if ok_sat else '재검토'}** |",
            f"| joint/act violations | 0 | **{jl_t}** / **{al_t}** | **{'예' if ok_lim else '아니오'}** |",
            f"| ncon | 0 | **{max(int(r['ncon_max']) for r in rows)}** | **{'예' if ok_ncon else '아니오'}** |",
            f"| |z| | ≤ **{z_max}** | **{max(float(r['max_abs_hys_z']) for r in rows):.4g}** | **{'예' if hz_ok else '아니오'}** |",
            f"| |τ_hys| | ≤ **{tau_hys_max}** | **{max(float(r['max_abs_tau_hys']) for r in rows):.4g}** | **{'예' if th_ok else '아니오'}** |",
            "",
        ]

        sig_larger = mean_rm > 2.0 * mild_ref_rms or mean_rm > 0.002

        extra_tail = [
            "## Medium-profile analysis\n\n",
            "### 1. Is the medium profile stable?\n",
            f"- {stab_note} — RMS 평균 **{mean_rm:.6f} m**.\n\n",
            "### 2. Significantly larger error than mild?\n",
            f"- Mild 참고 RMS ≈ **{mild_ref_rms:.6f} m** 대비 **{mean_rm:.6f} m** "
            f"({'유의한 증가' if sig_larger else '증가 폭은 제한적'}).\n\n",
            "### 3. Mean/std/min/max RMS EE\n",
            f"- mean **{float(np.mean(rms)):.6f}**, std **{float(np.std(rms)):.6f}**, "
            f"min **{float(np.min(rms)):.6f}**, max **{float(np.max(rms)):.6f}**.\n\n",
            "### 4. How many trials saturate?\n",
            f"- 누적 포화 스텝 **{total_sat}**, trial당 최대 비율 **{float(np.max(sat_frac))*100:.2f}%**.\n\n",
            "### 5. Joint/actuator limit violations?\n",
            f"- 관절 **{jl_t}**, 액추 **{al_t}** 스텝.\n\n",
            "### 6. Hysteresis bounded?\n",
            f"- **{'예' if hz_ok and th_ok else '표 확인'}** (`z_max`, `tau_hys_max`).\n\n",
            "### 7. Parameters most correlated with high RMS?\n",
            f"- 상위: {corr_top} (전체는 아래 표·CSV).\n\n",
            "### 8. Suitable for SAC residual learning?\n",
            f"- 평균 RMS **{mean_rm:.6f}**: **{'적합에 가깝다' if moderate_mean and ok_lim and ok_ncon else '박스 완화 또는 게인 조정 검토'}**.\n\n",
            "### 9. Stress profile evaluation-only?\n",
            "- **예** — `stress`는 주로 **평가·한계 진단**용으로 두고, 학습 분포는 `medium` 이하 권장.\n\n",
            "### Range validation\n",
            f"- {'**예**' if ok_rng else '**아니오**'}\n\n",
        ]

    elif profile == "medium_v2":
        mean_rm = float(np.mean(rms))
        worst_rm = float(np.max(rms))
        band_ok = 0.003 <= mean_rm <= 0.012
        ok_w = worst_rm < 0.03
        sat_vs_medium = (
            total_sat < int(MEDIUM_BASELINE_SAT_TOTAL * 0.30)
            if MEDIUM_BASELINE_SAT_TOTAL > 0
            else total_sat <= 15000
        )
        sf_max = float(np.max(sat_frac))
        pref_sat_frac = sf_max < 0.30
        near_lim = jl_t <= 25 and al_t <= 25
        ok_lim = jl_t == 0 and al_t == 0
        ok_lim_tag = "**예**" if ok_lim else ("**근접**" if near_lim else "**아니오**")
        ok_ncon = max(int(r["ncon_max"]) for r in rows) == 0
        hz_ok = max(float(r["max_abs_hys_z"]) for r in rows) <= z_max + 1e-6
        th_ok = max(float(r["max_abs_tau_hys"]) for r in rows) <= tau_hys_max + 1e-6

        crit_lines = [
            "## Pass criteria (medium_v2 — first SAC residual distribution)",
            "",
            "| criterion | target | result | ok? |",
            "|---|---|---|---|",
            f"| mean RMS EE | 0.003–0.012 m | **{mean_rm:.6f}** | **{'예' if band_ok else '아니오'}** |",
            f"| worst RMS EE | < 0.03 m | **{worst_rm:.6f}** | **{'예' if ok_w else '아니오'}** |",
            f"| total saturation vs medium (~{MEDIUM_BASELINE_SAT_TOTAL}) | much lower | **{total_sat}** | "
            f"**{'예' if sat_vs_medium else '재검토'}** |",
            f"| max saturation fraction | < 30% pref. | **{sf_max*100:.2f}%** | **{'예' if pref_sat_frac else '재검토'}** |",
            f"| joint / actuator limit steps | 0 또는 근사 | **{jl_t} / {al_t}** | {ok_lim_tag} |",
            f"| ncon_max | 0 | **{max(int(r['ncon_max']) for r in rows)}** | **{'예' if ok_ncon else '아니오'}** |",
            f"| max |z| | ≤ **{z_max}** | **{max(float(r['max_abs_hys_z']) for r in rows):.4g}** | "
            f"**{'예' if hz_ok else '아니오'}** |",
            f"| max |τ_hys| | ≤ **{tau_hys_max}** | "
            f"**{max(float(r['max_abs_tau_hys']) for r in rows):.4g}** | **{'예' if th_ok else '아니오'}** |",
            "",
            "## Validation metrics snapshot\n\n",
            f"- mean RMS EE **{mean_rm:.6f}**, std **{float(np.std(rms)):.6f}**, min **{float(np.min(rms)):.6f}**, max **{float(np.max(rms)):.6f}**",
            "",
            f"- mean final EE **{float(np.mean(fin)):.6f}**, max final EE **{float(np.max(fin)):.6f}**",
            "",
            f"- total saturation steps **{total_sat}**, max saturation fraction/trial **{sf_max:.4f}**",
            "",
            f"- joint limit steps (sum) **{jl_t}**, actuator limit steps (sum) **{al_t}**",
            "",
            f"- ncon_max **{max(int(r['ncon_max']) for r in rows)}**, max_abs_hys_z **{max(float(r['max_abs_hys_z']) for r in rows):.5g}**, "
            f"max_abs_tau_hys **{max(float(r['max_abs_tau_hys']) for r in rows):.5g}**",
            "",
        ]

        safe_vs_medium = (
            total_sat < MEDIUM_BASELINE_SAT_TOTAL
            and sf_max < MEDIUM_BASELINE_MAX_SAT_FRAC
            and (jl_t + al_t) < (MEDIUM_BASELINE_JL_TOTAL + MEDIUM_BASELINE_AL_TOTAL)
        )
        sac_fit = band_ok and ok_w and pref_sat_frac and ok_ncon and hz_ok and th_ok and (ok_lim or near_lim)

        extra_tail = [
            "## Recommendation (medium_v2)\n\n",
            "### 1. medium_v2가 mild보다 어렵나?\n",
            f"- **{'예 — 평균 RMS가 더 큼.' if mean_rm > 3.0 * mild_ref_rms else '통계 재확인 — mild 대비 증폭이 제한적'}** "
            f"(mild 참고 RMS ≈ **{mild_ref_rms:.6f} m**).\n\n",
            "### 2. medium_v2가 medium보다 안전한가?\n",
            f"- 포화 및 리밋 측면에서 **{'예 — 합계/최대 비율이 medium 기준점 대비 완화' if safe_vs_medium else '재검토'}**.",
            "",
            f"(medium 참고 총 포화 ≈ **{MEDIUM_BASELINE_SAT_TOTAL}**, 최대 포화 비율 ≈ **{100*MEDIUM_BASELINE_MAX_SAT_FRAC:.2f}%**",
            f", 리밋 스텝 합 jl/al ≈ **{MEDIUM_BASELINE_JL_TOTAL}** / **{MEDIUM_BASELINE_AL_TOTAL}**).\n\n",
            "### 3. 첫 SAC 학습 분포로 적합한가?\n",
            f"- **{'적합 가능' if sac_fit else '일부 조건 미달 — 레인지 또는 제어 재조정 권장'}** (위 표 참고).\n\n",
            "### 4. medium은 평가 분포로 둘까?\n",
            "- **예** — 리밋/포화가 많으므로 **`medium`은 강건성·평가·진단**에 두고,\n\n",
            "  학습 기본값은 **`medium_v2`(또는 mild warm-start)** 로 두는 구성을 권장.\n\n",
            "### 5. stress는 계속 평가 전용인가?\n",
            "- **예** — `stress`는 **한계 검증 및 드물게 재평가**용 유지.\n\n",
            "### Correlation note\n\n",
            f"Pearson 상위 예시: {corr_top}. (전체 순위는 하단 표.)\n\n",
            "### Range validation\n\n",
            f"- **{'예' if ok_rng else '아니오'}**\n\n",
        ]

    elif profile == "medium_train":
        mean_rm = float(np.mean(rms))
        worst_rm = float(np.max(rms))
        band_ok = 0.003 <= mean_rm <= 0.010
        ok_w = worst_rm < 0.025
        ok_lim = jl_t == 0 and al_t == 0
        ok_ncon = max(int(r["ncon_max"]) for r in rows) == 0
        hz_ok = max(float(r["max_abs_hys_z"]) for r in rows) <= z_max + 1e-6
        th_ok = max(float(r["max_abs_tau_hys"]) for r in rows) <= tau_hys_max + 1e-6
        sat_below_med = total_sat < MEDIUM_BASELINE_SAT_TOTAL
        sf_max = float(np.max(sat_frac))
        pref_sat_frac = sf_max < 0.40
        sac_core = band_ok and ok_w and ok_lim and ok_ncon and hz_ok and th_ok and sat_below_med

        crit_lines = [
            "## Pass criteria (medium_train — first SAC residual)",
            "",
            "| criterion | target | result | ok? |",
            "|---|---|---|---|",
            f"| mean RMS EE | 0.003–0.010 m | **{mean_rm:.6f}** | **{'예' if band_ok else '아니오'}** |",
            f"| worst RMS EE | < 0.025 m | **{worst_rm:.6f}** | **{'예' if ok_w else '아니오'}** |",
            f"| joint / actuator limits | 0 | **{jl_t} / {al_t}** | **{'예' if ok_lim else '아니오'}** |",
            f"| ncon_max | 0 | **{max(int(r['ncon_max']) for r in rows)}** | **{'예' if ok_ncon else '아니오'}** |",
            f"| max |z| | ≤ **{z_max}** | **{max(float(r['max_abs_hys_z']) for r in rows):.4g}** | "
            f"**{'예' if hz_ok else '아니오'}** |",
            f"| max |τ_hys| | ≤ **{tau_hys_max}** | "
            f"**{max(float(r['max_abs_tau_hys']) for r in rows):.4g}** | **{'예' if th_ok else '아니오'}** |",
            f"| total saturation | < medium (~{MEDIUM_BASELINE_SAT_TOTAL}) | **{total_sat}** | "
            f"**{'예' if sat_below_med else '아니오'}** |",
            f"| max saturation fraction | pref. < 40% | **{sf_max*100:.2f}%** | **{'예' if pref_sat_frac else '재검토'}** |",
            "",
            "## Notes (batch-level)\n\n",
            "- τ 후보(20 / 30 / 40) 전체 요약 및 Q1–Q6은 **`randomization_profile_comparison.md`** 하단 권장 섹션을 참고하세요.\n\n",
        ]

        extra_tail = [
            "## Recommendation (this run)\n\n",
            "### Correlation (top)\n\n",
            f"{corr_top}\n\n",
            "### Range validation\n\n",
            f"- **{'예' if ok_rng else '아니오'}**\n\n",
            "### SAC suitability (this τ only)\n\n",
            f"- **{'핵심 조건 충족' if sac_core and pref_sat_frac else '일부 조건 미달 — τ 스윕 또는 레인지 재검토'}**.\n\n",
        ]

    elif profile == "stress":
        crit_lines = [
            "## Stress profile (evaluation / diagnostic)",
            "",
            f"- mean RMS EE **{float(np.mean(rms)):.6f}**",
            f"- worst RMS EE **{float(np.max(rms)):.6f}**",
            f"- saturation total **{total_sat}**",
            f"- violations jl/al **{jl_t}/{al_t}**",
            "",
        ]
        extra_tail = [
            "## Stress note\n\n",
            "학습용 분포보다 **강한 교란**입니다. SAC 본학습 전에는 `mild` / `medium_v2` / `medium_train` 검증을 권장합니다.\n\n",
            f"### Correlation (top)\n\n{corr_top}\n\n",
        ]

    else:
        crit_lines = [
            "## Randomization batch (generic)",
            "",
            f"- profile **`{profile}`**",
            f"- mean / worst RMS EE **{float(np.mean(rms)):.6f}** / **{float(np.max(rms)):.6f}**",
            f"- saturation total **{total_sat}**",
            f"- violations jl/al **{jl_t}/{al_t}**",
            "",
        ]
        extra_tail = [
            "## Notes\n\n",
            f"{corr_top}\n\n",
        ]

    order = np.argsort(-rms)
    worst5 = order[: min(5, n)]
    worst_line = ", ".join(
        f"trial **{int(rows[i]['trial_id'])}** (RMS={float(rows[i]['rms_ee']):.6f})" for i in worst5
    )

    common = [
        "## Summary statistics (RMS EE)\n\n",
        f"- mean **{float(np.mean(rms)):.6f}**, std **{float(np.std(rms)):.6f}**, "
        f"min **{float(np.min(rms)):.6f}**, max **{float(np.max(rms)):.6f}**",
        "",
        "### Final EE (min/mean/max)\n\n",
        f"- min **{float(np.min(fin)):.6f}**, mean **{float(np.mean(fin)):.6f}**, max **{float(np.max(fin)):.6f}**\n\n",
        "### Worst 5 trials (by RMS EE)\n\n",
        f"- {worst_line}\n\n",
        "### Worst trial plots\n\n",
        f"- Trial **{worst_trial_id}**: `worst_trial_ee_xyz.png`, `worst_trial_ee_err_norm.png`\n\n",
    ]

    lines = _join_report_parts([header, *crit_lines, *common, *extra_tail])

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(lines, encoding="utf-8")

    plots_dir.mkdir(parents=True, exist_ok=True)
    t = worst_trace["t"]
    fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    axs[0].plot(t, worst_trace["des_x"], label="des")
    axs[0].plot(t, worst_trace["act_x"], "--", label="act")
    axs[0].set_ylabel("x [m]")
    axs[0].legend(fontsize=8)
    axs[1].plot(t, worst_trace["des_y"], label="des")
    axs[1].plot(t, worst_trace["act_y"], "--", label="act")
    axs[1].set_ylabel("y [m]")
    axs[2].plot(t, worst_trace["des_z"], label="des")
    axs[2].plot(t, worst_trace["act_z"], "--", label="act")
    axs[2].set_ylabel("z [m]")
    axs[2].set_xlabel("time [s]")
    fig.suptitle(f"Worst EE xyz — profile={profile}, τ_lim={tau_jnt_limit:g}, trial={worst_trial_id}")
    fig.tight_layout()
    fig.savefig(plots_dir / "worst_trial_ee_xyz.png", dpi=120)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 3.5))
    ax.plot(t, worst_trace["ee_err"])
    ax.set_xlabel("time [s]")
    ax.set_ylabel("‖EE err‖ [m]")
    ax.set_title(f"Worst trial EE norm — τ_lim={tau_jnt_limit:g}, trial={worst_trial_id}")
    fig.tight_layout()
    fig.savefig(plots_dir / "worst_trial_ee_err_norm.png", dpi=120)
    plt.close(fig)


def make_summary_plots(rows: list[dict], plots_dir: Path, report_path: Path) -> None:
    plots_dir.mkdir(parents=True, exist_ok=True)
    rms = np.array([float(r["rms_ee"]) for r in rows])
    fin = np.array([float(r["final_ee"]) for r in rows])

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.hist(rms, bins=min(14, max(6, len(rows) // 2)), color="steelblue", edgecolor="white")
    ax.set_xlabel("RMS EE [m]")
    ax.set_title("Histogram: RMS EE")
    fig.tight_layout()
    fig.savefig(plots_dir / "hist_rms_ee.png", dpi=120)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.hist(fin, bins=min(14, max(6, len(rows) // 2)), color="coral", edgecolor="white")
    ax.set_xlabel("final EE [m]")
    ax.set_title("Histogram: final EE")
    fig.tight_layout()
    fig.savefig(plots_dir / "hist_final_ee.png", dpi=120)
    plt.close(fig)

    xs_td = np.array([_mean3(r, "tau_delay") for r in rows])
    xs_al = np.array([_mean3(r, "hys_alpha") for r in rows])
    xs_bw = np.array([_mean3(r, "backlash_width") for r in rows])

    for name, xv in [("tau_delay", xs_td), ("hys_alpha", xs_al), ("backlash_width", xs_bw)]:
        fig, ax = plt.subplots(figsize=(6, 5))
        ax.scatter(xv, rms, alpha=0.75)
        ax.set_xlabel(name)
        ax.set_ylabel("RMS EE [m]")
        fig.tight_layout()
        fig.savefig(plots_dir / f"scatter_{name}_vs_rms_ee.png", dpi=120)
        plt.close(fig)

    pairs = correlation_ranking(rows)
    extra = ["\n## Correlation ranking (Pearson vs RMS EE)\n\n", "| rank | parameter | r |\n|---|---:|---:|\n"]
    for idx, (k, v) in enumerate(pairs[:12], start=1):
        extra.append(f"| {idx} | {k} | {v:.4f} |\n" if np.isfinite(v) else f"| {idx} | {k} | nan |\n")
    extra.append("\n")
    report_path.write_text(report_path.read_text(encoding="utf-8") + "".join(extra), encoding="utf-8")


def _profile_ranges(cfg: dict, profile: str) -> dict:
    profs = cfg.get("randomization_profiles") or {}
    if profile in profs and isinstance(profs[profile], dict) and "ranges" in profs[profile]:
        return dict(profs[profile]["ranges"])
    return dict(cfg["randomization"]["ranges"])


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Cable parameter randomization batch")
    p.add_argument(
        "--profile",
        choices=("mild", "medium", "medium_v2", "medium_train", "stress"),
        default="mild",
    )
    p.add_argument("--num-trials", type=int, default=30)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument(
        "--tau-jnt-limit",
        type=float,
        default=None,
        help="If set, clip PD joint torque to ±this value and write under randomization_<profile>_tau<int>.",
    )
    p.add_argument(
        "--refresh-comparison-only",
        action="store_true",
        help="Regenerate randomization_profile_comparison.md from existing CSVs and exit.",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    if bool(args.refresh_comparison_only):
        write_profile_comparison(DEBUG_LAYER)
        print("Wrote", DEBUG_LAYER / "randomization_profile_comparison.md")
        return

    profile = str(args.profile)
    num_trials = int(args.num_trials)
    base_seed = int(args.seed)
    tau_cli = args.tau_jnt_limit
    tau_lim = float(DEFAULT_TAU_JNT_LIMIT if tau_cli is None else tau_cli)

    with open(CABLE_LAYER_DEFAULT, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg.setdefault("randomization", {})["enabled"] = True

    if tau_cli is None:
        out_root = DEBUG_LAYER / f"randomization_{profile}"
    else:
        out_root = DEBUG_LAYER / f"randomization_{profile}_tau{int(round(tau_lim))}"

    plots_dir = out_root / "plots"
    csv_path = out_root / "randomization_trials.csv"
    report_path = out_root / "randomization_report.md"

    ranges = _profile_ranges(cfg, profile)

    z_max = float(cfg["cable_hysteresis"]["z_max"])
    tau_hys_max = float(cfg["cable_hysteresis"]["tau_hys_max"])

    qwp0, qwp1, qwp2 = solve_wp_q_arm_only()
    hybrid = HybridTransmission(cable_config=cfg)

    rows: list[dict] = []
    worst_rms = -1.0
    worst_trace: dict[str, np.ndarray] | None = None
    worst_tid = 0

    for trial_id in range(num_trials):
        trial_seed = base_seed + 100_003 * trial_id
        metrics, trace = run_trial(
            hybrid,
            trial_seed=trial_seed,
            profile_name=profile,
            qwp0=qwp0,
            qwp1=qwp1,
            qwp2=qwp2,
            tau_jnt_limit=tau_lim,
        )
        params = hybrid.get_current_params()
        if "sample_seed" in params:
            del params["sample_seed"]

        row = {
            "trial_id": trial_id,
            "seed": trial_seed,
            "tau_jnt_limit": tau_lim,
            "randomization_profile": hybrid.last_randomization_profile or profile,
        }
        row.update(params)
        row.update(metrics)
        rows.append(row)

        if metrics["rms_ee"] > worst_rms:
            worst_rms = metrics["rms_ee"]
            worst_trace = trace
            worst_tid = trial_id

    assert worst_trace is not None

    out_root.mkdir(parents=True, exist_ok=True)
    if rows:
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

    write_report(
        rows,
        ranges,
        z_max,
        tau_hys_max,
        worst_trace,
        worst_tid,
        profile,
        report_path,
        plots_dir,
        tau_jnt_limit=tau_lim,
    )
    make_summary_plots(rows, plots_dir, report_path)

    write_profile_comparison(DEBUG_LAYER)
    cmp_path = DEBUG_LAYER / "randomization_profile_comparison.md"

    print("Wrote", csv_path)
    print("Wrote", report_path)
    print("Plots ->", plots_dir)
    print("Wrote", cmp_path)


if __name__ == "__main__":
    main()
