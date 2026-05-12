#!/usr/bin/env python3
"""작업공간 VSD ``task_space_vsd.Ks`` / ``Kd`` 자동 탐색 (경로 추종 시뮬 기준).

목표(기본):
  - 위치: ``max_t ||e_xyz||_2 <= 5e-3`` [m]
  - 자세(roll/pitch만, FK 기준): ``max_t max(|wrap(r_des-r)|, |wrap(p_des-p)|) <= 0.5e-3`` [rad] (=0.5 mrad)

``demo_vsd_torque_path_follow`` 와 동일한 MJCF·궤적·전달 모델로 롤아웃한 뒤, 블록별 배율
``(ax,bx,ar,br)`` 로 ``Ks[:3]*=ax``, ``Kd[:3]*=bx``, ``Ks[3:5]*=ar``, ``Kd[3:5]*=br`` 를 탐색한다.

실행 (``pmi_cable_mujoco`` 에서)::

    python scripts/tune_task_space_vsd_gains.py
    python scripts/tune_task_space_vsd_gains.py --write
    python scripts/tune_task_space_vsd_gains.py --horizon-steps 1200 --settle-steps 80 --write-best-effort
    python scripts/tune_task_space_vsd_gains.py --pos-tol-m 0.005 --ori-tol-mrad 5 --write

``--write`` 는 **위치·자세 목표를 모두 만족한 경우에만** YAML 을 갱신한다. 미달이면 ``--write-best-effort`` 로 스코어 최저안을 저장할 수 있다.
sub-mrad 자세 목표는 케이블·짧은 평가 창에서는 거의 달성되지 않을 수 있으니, 검증은 ``--horizon-steps`` 를 크게 한 뒤 ``plot_vsd_torque_path_desired_vs_ee.py`` 로 재확인한다.
"""

from __future__ import annotations

import argparse
import copy
import os
import sys
from pathlib import Path

import numpy as np
import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import mujoco  # noqa: E402

import demo_vsd_torque_path_follow as vsd  # noqa: E402
from controllers.waypoint_trajectory import WaypointTrajectory  # noqa: E402
from kinematics.pmi_chain import fk_ee_pose_joint_rad  # noqa: E402


def _wrap_angle_vec(diff: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(diff), np.cos(diff))


def _rollout_metrics(
    *,
    Ks: np.ndarray,
    Kd: np.ndarray,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    traj,
    is_wp: bool,
    qpos_adr: list[int],
    dof_adr: list[int],
    tau_extra: np.ndarray,
    gravity_bias_ff: bool,
    gravity_compensation_gain: float,
    frame_skip: int,
    hybrid_dt: float,
    belt,
    cable,
    settle_steps: int,
    max_steps: int,
    burn_in_frac: float,
    waypoint_task_from: str = "fk_q_des",
) -> tuple[float, float, bool]:
    """Return ``(pos_max_m, ori_max_rad, ok_sim)`` over indices after burn-in."""
    mujoco.mj_resetData(model, data)
    if is_wp:
        q_seed = np.array([float(data.qpos[a]) for a in qpos_adr], dtype=np.float64)
        traj.on_reset(
            q_seed,
            mujoco_data=data,
            joint_qpos_adr=qpos_adr,
        )
    else:
        traj.reset()

    mujoco.mj_forward(model, data)
    if belt is not None:
        belt.reset()
    if cable is not None:
        cable.reset()

    settle_n = max(0, int(settle_steps))
    if settle_n > 0:
        vsd._settle_vsd_on_current_pose(
            model,
            data,
            qpos_adr=qpos_adr,
            dof_adr=dof_adr,
            frame_skip=frame_skip,
            settle_steps=settle_n,
            Ks=Ks,
            Kd=Kd,
            gravity_bias_ff=gravity_bias_ff,
            gravity_compensation_gain=float(gravity_compensation_gain),
            hybrid_dt=hybrid_dt,
            belt=belt,
            cable=cable,
        )
        if is_wp and isinstance(traj, WaypointTrajectory):
            q_now = np.array([float(data.qpos[a]) for a in qpos_adr], dtype=np.float64)
            traj.on_reset(
                q_now,
                mujoco_data=data,
                joint_qpos_adr=qpos_adr,
                apply_yaml_initial_pose=False,
            )
            mujoco.mj_forward(model, data)

    pos_list: list[float] = []
    ori_list: list[float] = []
    ok_sim = True

    for k in range(max_steps):
        try:
            q, q_des, _pe, _err, _meta = vsd.vsd_torque_path_control_step(
                model,
                data,
                traj=traj,
                qpos_adr=qpos_adr,
                dof_adr=dof_adr,
                Ks=Ks,
                Kd=Kd,
                tau_extra=tau_extra,
                gravity_bias_ff=gravity_bias_ff,
                gravity_compensation_gain=float(gravity_compensation_gain),
                frame_skip=frame_skip,
                hybrid_dt=hybrid_dt,
                belt=belt,
                cable=cable,
                waypoint_task_from=str(waypoint_task_from),
            )
        except (FloatingPointError, ValueError):
            ok_sim = False
            break

        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(q_des)):
            ok_sim = False
            break

        ee_d, rpy_d = fk_ee_pose_joint_rad(q_des)
        ee_a, rpy_a = fk_ee_pose_joint_rad(q)
        pos_list.append(float(np.linalg.norm(ee_d - ee_a)))
        dr = float(np.abs(_wrap_angle_vec(np.array([rpy_d[0] - rpy_a[0]]))[0]))
        dp = float(np.abs(_wrap_angle_vec(np.array([rpy_d[1] - rpy_a[1]]))[0]))
        ori_list.append(max(dr, dp))

    if not pos_list or not ok_sim:
        return 1e9, 1e9, False

    n0 = int(len(pos_list) * float(burn_in_frac))
    n0 = min(max(n0, 0), len(pos_list) - 1)
    pos_arr = np.asarray(pos_list[n0:], dtype=np.float64)
    ori_arr = np.asarray(ori_list[n0:], dtype=np.float64)
    return float(np.max(pos_arr)), float(np.max(ori_arr)), True


def _apply_scales(
    Ks0: np.ndarray, Kd0: np.ndarray, ax: float, bx: float, ar: float, br: float
) -> tuple[np.ndarray, np.ndarray]:
    Ks = Ks0.copy()
    Kd = Kd0.copy()
    Ks[:3] *= ax
    Kd[:3] *= bx
    Ks[3:5] *= ar
    Kd[3:5] *= br
    return Ks, Kd


def _score(pos_max: float, ori_max: float, pos_tol: float, ori_tol: float) -> float:
    return max(pos_max / pos_tol, ori_max / ori_tol)


def _local_refine(
    eval_scales,
    ax: float,
    bx: float,
    ar: float,
    br: float,
    rounds: int = 3,
) -> tuple[float, float, float, float, float, float, float]:
    """Return ``(ax,bx,ar,br, pm, om, sc)``."""
    pm, om, sc = eval_scales(ax, bx, ar, br)
    for _ in range(rounds):
        for dim in range(4):
            vals = [ax, bx, ar, br]
            base = float(vals[dim])
            for m in np.geomspace(0.93, 1.07, 5):
                cand = [ax, bx, ar, br]
                cand[dim] = float(base * m)
                pm2, om2, sc2 = eval_scales(*cand)
                if sc2 < sc:
                    ax, bx, ar, br = cand[0], cand[1], cand[2], cand[3]
                    pm, om, sc = pm2, om2, sc2
    return ax, bx, ar, br, pm, om, sc


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", type=Path, default=vsd.DEFAULT_CONFIG)
    ap.add_argument("--model", type=Path, default=vsd.DEFAULT_MODEL)
    ap.add_argument("--settle-steps", type=int, default=40)
    ap.add_argument("--steps", type=int, default=0, help="0이면 --horizon-steps 사용")
    ap.add_argument(
        "--horizon-steps",
        type=int,
        default=320,
        help="튜닝 롤아웃 최대 제어 스텝(짧을수록 빠름; 최종 검증은 plot 스크립트로 긴 궤적 권장)",
    )
    ap.add_argument("--ee-arrival-max-steps", type=int, default=12_000)
    ap.add_argument("--pos-tol-m", type=float, default=5e-3)
    ap.add_argument(
        "--ori-tol-mrad",
        type=float,
        default=0.5,
        help="roll/pitch 허용 오차 [millirad]. 기본 0.5 → 5e-4 rad",
    )
    ap.add_argument(
        "--burn-in-frac",
        type=float,
        default=0.06,
        help="지표 계산 시 앞쪽 스텝 비율 제외(초기 과도 제거)",
    )
    ap.add_argument("--random-trials", type=int, default=45)
    ap.add_argument(
        "--greedy-points",
        type=int,
        default=8,
        help="탐욕 1차 스캔: 각 축(ax,bx,ar,br)당 지오메트릭 샘플 수",
    )
    ap.add_argument("--greedy-sweeps", type=int, default=2, help="탐욕 축 스캔 반복 횟수")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-transmission", action="store_true")
    ap.add_argument("--belt-config", type=Path, default=REPO_ROOT / "configs" / "belt_params.yaml")
    ap.add_argument("--cable-config", type=Path, default=REPO_ROOT / "configs" / "cable_params.yaml")
    ap.add_argument(
        "--write",
        action="store_true",
        help="목표(위치·자세)를 만족할 때만 YAML 저장",
    )
    ap.add_argument(
        "--write-best-effort",
        action="store_true",
        help="목표 미달이어도 스코어 최저안을 YAML 에 저장 (주의: 공격적 게인일 수 있음)",
    )
    ap.add_argument("--dry-run", action="store_true", help="저장 생략 (--write / --write-best-effort)")
    args = ap.parse_args()

    ori_tol_rad = float(args.ori_tol_mrad) * 1e-3
    pos_tol = float(args.pos_tol_m)

    control_cfg = vsd._load_yaml(Path(args.config))
    sim_cfg = control_cfg.get("simulation", {})
    frame_skip = int(sim_cfg.get("frame_skip", 1))

    Ks0, Kd0, gff_yaml, gcomp_yaml = vsd._default_task_vsd_gains(control_cfg)
    gravity_bias_ff = bool(gff_yaml)
    gcomp_tune = float(gcomp_yaml)

    model = mujoco.MjModel.from_xml_path(os.fspath(args.model))
    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)
    ctrl_dt = dt * frame_skip
    hybrid_dt = dt

    belt = cable = None
    if not args.no_transmission:
        belt, cable = vsd.build_belt_cable_models(
            Path(args.belt_config),
            Path(args.cable_config),
            randomize=False,
        )

    traj, is_wp = vsd._build_trajectory(control_cfg, ctrl_dt)
    qpos_adr, dof_adr = vsd._joint_adr(model)
    tau_extra = np.zeros(4, dtype=np.float64)

    if is_wp and isinstance(traj, WaypointTrajectory):
        wp_len = traj.playback_num_steps()
    else:
        wp_len = 0

    cap_h = max(1, int(args.horizon_steps))
    if is_wp:
        raw = int(args.steps) if args.steps > 0 else min(int(args.ee_arrival_max_steps), max(wp_len, 1))
        max_steps = min(raw, cap_h, int(args.ee_arrival_max_steps))
    else:
        max_steps = int(args.steps) if args.steps > 0 else min(6000, cap_h)

    rollout_kw = dict(
        model=model,
        data=data,
        traj=traj,
        is_wp=is_wp,
        qpos_adr=qpos_adr,
        dof_adr=dof_adr,
        tau_extra=tau_extra,
        gravity_bias_ff=gravity_bias_ff,
        gravity_compensation_gain=gcomp_tune,
        frame_skip=frame_skip,
        hybrid_dt=hybrid_dt,
        belt=belt,
        cable=cable,
        settle_steps=int(args.settle_steps),
        max_steps=max_steps,
        burn_in_frac=float(args.burn_in_frac),
        waypoint_task_from="fk_q_des",
    )

    rng = np.random.default_rng(int(args.seed))

    best_sc = 1e9
    best_pm = 1e9
    best_om = 1e9
    best_scales = (1.0, 1.0, 1.0, 1.0)
    feasible_best_sc = np.inf
    feasible_scales: tuple[float, float, float, float] | None = None

    def eval_scales(ax: float, bx: float, ar: float, br: float) -> tuple[float, float, float]:
        Ks, Kd = _apply_scales(Ks0, Kd0, ax, bx, ar, br)
        pm, om, ok = _rollout_metrics(Ks=Ks, Kd=Kd, **rollout_kw)
        if not ok:
            return 1e9, 1e9, 1e9
        sc = _score(pm, om, pos_tol, ori_tol_rad)
        return pm, om, sc

    def note(ax: float, bx: float, ar: float, br: float, pm: float, om: float, sc: float) -> None:
        nonlocal best_sc, best_pm, best_om, best_scales, feasible_best_sc, feasible_scales
        if sc < best_sc:
            best_sc, best_pm, best_om, best_scales = sc, pm, om, (ax, bx, ar, br)
        if pm <= pos_tol and om <= ori_tol_rad and sc < feasible_best_sc:
            feasible_best_sc = sc
            feasible_scales = (ax, bx, ar, br)

    ax = bx = ar = br = 1.0
    gp = max(3, int(args.greedy_points))
    for _sw in range(int(args.greedy_sweeps)):
        for dim in range(4):
            vals = [ax, bx, ar, br]
            b0 = float(max(vals[dim], 1e-6))
            lo = 0.55 * b0 if dim < 2 else 0.45 * b0
            hi = min(50.0, 2.4 * b0) if dim >= 2 else min(5.5, 2.2 * b0)
            for m in np.geomspace(lo, hi, num=gp):
                cand = [ax, bx, ar, br]
                cand[dim] = float(np.clip(m, 0.04, 55.0))
                pm, om, sc = eval_scales(*cand)
                note(cand[0], cand[1], cand[2], cand[3], pm, om, sc)
        ax, bx, ar, br = best_scales

    for _t in range(int(args.random_trials)):
        ax2 = float(rng.uniform(0.75, 4.5))
        bx2 = float(rng.uniform(0.75, 3.8))
        ar2 = float(rng.uniform(0.45, 32.0))
        br2 = float(rng.uniform(0.45, 28.0))
        pm, om, sc = eval_scales(ax2, bx2, ar2, br2)
        note(ax2, bx2, ar2, br2, pm, om, sc)

    ax, bx, ar, br = feasible_scales if feasible_scales is not None else best_scales
    ax, bx, ar, br, best_pm, best_om, best_sc = _local_refine(eval_scales, ax, bx, ar, br, rounds=2)

    if best_om > ori_tol_rad:
        ax0, bx0 = ax, bx
        for _ in range(40):
            ar2 = float(rng.uniform(max(0.35, ar * 0.82), min(50.0, ar * 1.4)))
            br2 = float(rng.uniform(max(0.35, br * 0.82), min(45.0, br * 1.4)))
            pm, om, sc = eval_scales(ax0, bx0, ar2, br2)
            note(ax0, bx0, ar2, br2, pm, om, sc)
            if sc < best_sc:
                ax, bx, ar, br = ax0, bx0, ar2, br2
                best_pm, best_om, best_sc = pm, om, sc
        ax, bx, ar, br, best_pm, best_om, best_sc = _local_refine(eval_scales, ax, bx, ar, br, rounds=1)

    ax, bx, ar, br = best_scales
    best_pm, best_om, best_sc = eval_scales(ax, bx, ar, br)
    Ks_best, Kd_best = _apply_scales(Ks0, Kd0, ax, bx, ar, br)

    print(
        f"best scales ax,bx,ar,br = {ax:.4f}, {bx:.4f}, {ar:.4f}, {br:.4f}  "
        f"score={best_sc:.4f} (pos_max={best_pm:.6f} m, ori_rp_max={best_om:.6f} rad)"
    )
    print(
        f"targets: pos<={pos_tol:.4f} m, ori_rp<={ori_tol_rad:.6f} rad ({args.ori_tol_mrad} mrad)"
    )
    met_pos = best_pm <= pos_tol
    met_ori = best_om <= ori_tol_rad
    if met_pos and met_ori:
        print("OK: 두 목표 모두 만족.")
    else:
        if not met_pos:
            print("warning: 위치 목표 미달 — 게인·스텝 수·전달 모델을 추가로 조정하세요.")
        if not met_ori and float(args.ori_tol_mrad) < 2.0:
            print(
                "warning: roll/pitch 목표 미달 — sub-mrad 는 대부분의 시뮬 궤적에서 매우 빡빡합니다. "
                "`--ori-tol-mrad 5`(=5 mrad) 등 완화를 검토하세요."
            )
        elif not met_ori:
            print("warning: roll/pitch 목표 미달.")

    def _round_list(arr: np.ndarray) -> list[float]:
        return [float(f"{float(x):.6g}") for x in np.asarray(arr, dtype=np.float64).reshape(-1)]

    print("Ks:", _round_list(Ks_best))
    print("Kd:", _round_list(Kd_best))

    do_write = (args.write and met_pos and met_ori) or bool(args.write_best_effort)
    if not do_write and args.write and not (met_pos and met_ori):
        print("목표 미달: YAML 은 쓰지 않았습니다. 강제 저장은 --write-best-effort")
    if do_write and not args.dry_run:
        out_cfg = copy.deepcopy(control_cfg)
        block = out_cfg.setdefault("task_space_vsd", {})
        block["Ks"] = _round_list(Ks_best)
        block["Kd"] = _round_list(Kd_best)
        with open(Path(args.config), "w", encoding="utf-8") as f:
            yaml.safe_dump(
                out_cfg,
                f,
                default_flow_style=False,
                allow_unicode=True,
                sort_keys=False,
            )
        print(f"Wrote {args.config}")
    elif do_write and args.dry_run:
        print("dry-run: YAML not written")


if __name__ == "__main__":
    main()
