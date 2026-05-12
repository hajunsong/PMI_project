#!/usr/bin/env python3
"""하이브리드 무충돌 모델: q_act 위치 서보로 IK(quintic) 관절 궤적 추종, 이상 전달 검증."""

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

from validate_hybrid_no_collision_model import HYBRID_MODEL_XML, OUT_DIR, run_model_validation, write_validation_report
from validate_hybrid_no_collision_model import ACT_NAMES, INITIAL_ACTUATOR_RAD, JNT_NAMES, RATIOS, Q_JNT_INIT

ARM_ONLY_XML = PKG_ROOT / "models" / "pmi_arm_only_no_collision.xml"
WPS = [
    np.array([0.25, -0.20, -0.10]),
    np.array([0.00, -0.35, -0.15]),
    np.array([-0.25, -0.20, -0.10]),
]


def solve_wp_joints_arm_only() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    model = load_mjmodel(ARM_ONLY_XML, strip_position_actuators=True)
    scratch = mj.MjData(model)
    q_lo = np.array([float(model.jnt_range[joint_id(model, n), 0]) for n in JNT_NAMES])
    q_hi = np.array([float(model.jnt_range[joint_id(model, n), 1]) for n in JNT_NAMES])
    ik = IKConfig((1, 1, 1), 1.0, 1.0, 1e-3, 1e-4, 80, 1e-9, tuple(JNT_NAMES))
    q_seed = Q_JNT_INIT.copy()
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


def run_position_mapping(
    *,
    duration: float,
    csv_path: Path,
) -> dict:
    q0, q1, q2 = solve_wp_joints_arm_only()
    path = scaled_joint_quintic(q0, q1, q2, float(duration))

    model = mj.MjModel.from_xml_path(str(HYBRID_MODEL_XML))
    data = mj.MjData(model)
    scratch = mj.MjData(model)
    dt = float(model.opt.timestep)

    qadr_j = np.array([int(model.jnt_qposadr[joint_id(model, n)]) for n in JNT_NAMES])
    qadr_a = np.array([int(model.jnt_qposadr[joint_id(model, n)]) for n in ACT_NAMES])
    dadr_j = np.array([int(model.jnt_dofadr[joint_id(model, n)]) for n in JNT_NAMES])

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    for i in range(4):
        data.qpos[qadr_a[i]] = float(INITIAL_ACTUATOR_RAD[i])
        data.qpos[qadr_j[i]] = float(Q_JNT_INIT[i])
    mj.mj_forward(model, data)
    data.ctrl[:] = 0.0
    for i in range(model.nu):
        data.ctrl[i] = float(INITIAL_ACTUATOR_RAD[i])

    n = int(round(float(duration) / dt)) + 1
    trans_err = []
    ee_err = []
    ncon_max = 0
    qfc_max = 0.0

    fieldnames = (
        ["step", "time"]
        + [f"q_act_des_{i+1}" for i in range(4)]
        + [f"q_act_actual_{i+1}" for i in range(4)]
        + [f"q_jnt_des_{i+1}" for i in range(4)]
        + [f"q_jnt_actual_{i+1}" for i in range(4)]
        + [f"ratio_times_q_act_{i+1}" for i in range(4)]
        + [f"transmission_err_{i+1}" for i in range(4)]
        + ["ee_des_x", "ee_des_y", "ee_des_z", "ee_act_x", "ee_act_y", "ee_act_z", "ee_err_norm"]
        + ["ncon"]
        + [f"qfrc_constraint_{n}" for n in JNT_NAMES]
    )

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with open(csv_path, "w", newline="", encoding="utf-8") as fcsv:
        w = csv.DictWriter(fcsv, fieldnames=fieldnames)
        w.writeheader()

        for k in range(n):
            t = min(k * dt, float(duration))
            q_des, _, _ = path.sample(float(t))
            q_act_des = q_des / RATIOS

            for i in range(model.nu):
                qa = float(np.clip(q_act_des[i], model.jnt_range[joint_id(model, ACT_NAMES[i]), 0], model.jnt_range[joint_id(model, ACT_NAMES[i]), 1]))
                data.ctrl[i] = qa

            q_act_actual = np.array([float(data.qpos[qadr_a[i]]) for i in range(4)])
            q_jnt_actual = np.array([float(data.qpos[qadr_j[i]]) for i in range(4)])
            r_times_q = RATIOS * q_act_actual
            terr = q_jnt_actual - r_times_q
            trans_err.append(float(np.linalg.norm(terr)))

            p_des, *_ = fk_ee_rp(model, scratch, q_des, JNT_NAMES)
            p_act, *_ = fk_ee_rp(model, scratch, q_jnt_actual, JNT_NAMES)
            p_des = np.asarray(p_des, dtype=float)
            p_act = np.asarray(p_act, dtype=float)
            een = float(np.linalg.norm(p_des - p_act))
            ee_err.append(een)

            mj.mj_step(model, data)

            ncon_post = int(data.ncon)
            ncon_max = max(ncon_max, ncon_post)
            qfc = np.array([float(data.qfrc_constraint[int(dadr_j[i])]) for i in range(4)])
            qfc_max = max(qfc_max, float(np.max(np.abs(qfc))))

            row = {
                "step": k,
                "time": t,
                **{f"q_act_des_{i+1}": float(q_act_des[i]) for i in range(4)},
                **{f"q_act_actual_{i+1}": float(q_act_actual[i]) for i in range(4)},
                **{f"q_jnt_des_{i+1}": float(q_des[i]) for i in range(4)},
                **{f"q_jnt_actual_{i+1}": float(q_jnt_actual[i]) for i in range(4)},
                **{f"ratio_times_q_act_{i+1}": float(r_times_q[i]) for i in range(4)},
                **{f"transmission_err_{i+1}": float(terr[i]) for i in range(4)},
                "ee_des_x": float(p_des[0]),
                "ee_des_y": float(p_des[1]),
                "ee_des_z": float(p_des[2]),
                "ee_act_x": float(p_act[0]),
                "ee_act_y": float(p_act[1]),
                "ee_act_z": float(p_act[2]),
                "ee_err_norm": een,
                "ncon": ncon_post,
                **{f"qfrc_constraint_{JNT_NAMES[i]}": float(qfc[i]) for i in range(4)},
            }
            w.writerow(row)

    te = np.asarray(trans_err, dtype=float)
    ee = np.asarray(ee_err, dtype=float)
    return {
        "duration": float(duration),
        "rms_transmission_err_norm": float(np.sqrt(np.mean(te**2))),
        "max_transmission_err_norm": float(np.max(te)),
        "final_transmission_err_norm": float(te[-1]),
        "rms_ee_err": float(np.sqrt(np.mean(ee**2))),
        "max_ee_err": float(np.max(ee)),
        "final_ee_err": float(ee[-1]),
        "ncon_max": ncon_max,
        "qfrc_constraint_jnt_max_abs": qfc_max,
    }


def write_mapping_report(path: Path, m: dict) -> None:
    lines = [
        "# q_act position mapping (hybrid no-collision)",
        "",
        f"- Duration: {m['duration']} s",
        f"- RMS ‖q_jnt − ratio⊙q_act‖: {m['rms_transmission_err_norm']:.6e}",
        f"- Max ‖transmission err‖: {m['max_transmission_err_norm']:.6e}",
        f"- Final ‖transmission err‖: {m['final_transmission_err_norm']:.6e}",
        f"- RMS EE error (FK): {m['rms_ee_err']:.6f} m",
        f"- Max EE error: {m['max_ee_err']:.6f} m",
        f"- Final EE error: {m['final_ee_err']:.6f} m",
        f"- Max ncon over run: {m['ncon_max']}",
        f"- Max abs qfrc_constraint (jnt1..4): {m['qfrc_constraint_jnt_max_abs']:.6g}",
        "",
        "Timeseries: `qact_position_mapping.csv` (same folder).",
        "",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def write_summary_report(path: Path, val: dict, mp: dict) -> None:
    mesh_ok = val["mesh_xml_ok"] and val["mesh_model_ok"]
    ncon_ok_val = val["ncon_initial"] == 0
    ncon_ok_run = mp["ncon_max"] == 0

    tol_tau = 5e-3  # rad scale numerical/constraint softness
    tol_ee = 0.035  # m — 위치 서보만 사용 시현실적 허용
    map_ok = mp["max_transmission_err_norm"] < tol_tau and mp["rms_ee_err"] < tol_ee

    lines = [
        "# Hybrid no-collision — summary readiness",
        "",
        "## 1. Does hybrid no-collision model have zero contacts?",
        f"- 초기 자세 `ncon={val['ncon_initial']}` ({'예' if ncon_ok_val else '아니오'}). ",
        f"- 위치 매핑 롤아웃 중 `ncon_max={mp['ncon_max']}` ({'예' if ncon_ok_run else '아니오'}). ",
        f"- 메시 비접촉 플래그: {'예' if mesh_ok else '일부 실패'}",
        "",
        "## 2. Does q_act position mapping work?",
        f"- 전달 오차 벡터 노름: RMS={mp['rms_transmission_err_norm']:.4e}, max={mp['max_transmission_err_norm']:.4e}. ",
        f"- (손실 기준 예시: max‖·‖ < {tol_tau:g} rad → {'통과' if mp['max_transmission_err_norm'] < tol_tau else '재검토'})",
        "",
        "## 3. Does EE path tracking still work with ideal mapping?",
        f"- FK 기준 RMS EE 오차 ≈ {mp['rms_ee_err']:.4f} m, 최대 ≈ {mp['max_ee_err']:.4f} m. ",
        f"- ({'추적 양호' if mp['rms_ee_err'] < tol_ee else '이득·시간·포화 추가 튜닝 권장'})",
        "",
        "## 4. Is the model ready for ideal q_act torque transmission?",
        "- 등식으로 `jnt = ratio * q_act` 가 이미 물리 제약으로 걸려 있으므로, **다음 단계**는 ",
        "  액추에이터 측 토크/힘을 정의한 뒤 관절측 `qfrc_applied` 또는 토크 액추에이터로 ",
        "  동일 비율을 역동역학적으로 맞추는 설계가 됩니다. ",
        f"- 현재 검증(접촉 없음·전달 소오차): {'진행 가능' if mesh_ok and ncon_ok_run else '추가 점검 필요'}.",
        "",
        "## 5. Is the model ready for later cable transmission q2~q4?",
        "- **구조적으로** q2~q4 는 별도 equality/케이블 모델로 `jnt2..4` 와 `q2..4_act` 결합을 바꿀 수 있는 상태입니다. ",
        "- 본 MJCF에는 **케이블 동역학 미포함**. 케이블 도입 시 새 제약·마찰·프리스트레스를 추가해야 합니다.",
        "",
        "## Controller baselines (참고, 미혼합)",
        "- **A.** IK joint-space VSD: `tau = qfrc_bias + Kq(q_des−q)+Dq(qdot_des−qdot)` → `jnt1..4` `qfrc_applied`. ",
        "- **B.** 작업공간 JTF VSD: `F=Kx e + Dx edot`, `tau = qfrc_bias + J^T F` → `jnt` `qfrc_applied`. ",
        "- 위치 서보 XML은 **검증/동기용**이며, 토크 명령은 `data.ctrl` 로 직접 보내지 않는 것을 권장(토크 액추에이터 정의 시까지).",
        "",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR)
    ap.add_argument("--duration", type=float, default=5.0)
    args = ap.parse_args()
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    val = run_model_validation()
    write_validation_report(out / "model_validation_report.md", val)

    csv_path = out / "qact_position_mapping.csv"
    mp = run_position_mapping(duration=float(args.duration), csv_path=csv_path)
    write_mapping_report(out / "qact_position_mapping_report.md", mp)
    write_summary_report(out / "summary_report.md", val, mp)

    print(f"Wrote {out / 'model_validation_report.md'}")
    print(f"Wrote {csv_path}")
    print(f"Wrote {out / 'qact_position_mapping_report.md'}")
    print(f"Wrote {out / 'summary_report.md'}")


if __name__ == "__main__":
    main()
