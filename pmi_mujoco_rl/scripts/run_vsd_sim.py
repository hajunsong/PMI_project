#!/usr/bin/env python3
"""
PMI URDF MuJoCo 환경에서 analysis ``run_vsd`` 와 같은 궤적·게인으로 작업공간 PD(VSD) 추종.

저장소 루트에서 실행하는 것을 권장::

    cd /path/to/PMI
    python pmi_mujoco_rl/scripts/run_vsd_sim.py --steps 500

옵션 ``--viewer`` 로 mujoco.viewer (별도 창) 동시 실행.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
from mujoco import Renderer

# ``main`` 안에서 ``import mujoco.viewer`` 하면 지역 ``mujoco`` 로 간주되어 상단 ``import mujoco`` 와 충돌한다.
from mujoco import viewer as mj_viewer

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from pmi_mujoco_rl.model import (
    LoadOptions,
    actuated_joints_for_model,
    ctrl_indices_for_actuators,
    load_pmi_model,
)
from pmi_mujoco_rl.paths import repo_root
from pmi_mujoco_rl.vsd import (
    KD_DEFAULT,
    KS_DEFAULT,
    WP_T,
    WP_X,
    WP_Y,
    WP_Z,
    apply_initial_from_recurdyn_row,
    operational_pd_torque,
    path_build_full_quintic,
    task_vector,
)


def main() -> None:
    ap = argparse.ArgumentParser(description="MuJoCo PMI VSD 시뮬레이션")
    ap.add_argument("--steps", type=int, default=3000, help="시뮬레이션 스텝 수 (dt는 모델 timestep)")
    ap.add_argument("--viewer", action="store_true", help="MuJoCo passive viewer 실행")
    ap.add_argument(
        "--no-csv-init",
        action="store_true",
        help="rec_data_path.csv 가 아닌 영(0) 자세로 초기화",
    )
    ap.add_argument(
        "--csv-out",
        type=str,
        default="pmi_mujoco_rl/artifacts/vsd_result.csv",
        help="비교용 결과 CSV 저장 경로",
    )
    ap.add_argument(
        "--video-out",
        type=str,
        default="pmi_mujoco_rl/artifacts/vsd_result.mp4",
        help="결과 영상 MP4 저장 경로 (빈 문자열이면 비활성)",
    )
    ap.add_argument("--video-width", type=int, default=960, help="영상 너비")
    ap.add_argument("--video-height", type=int, default=540, help="영상 높이")
    ap.add_argument("--video-fps", type=float, default=30.0, help="영상 FPS")
    ap.add_argument("--video-frame-skip", type=int, default=2, help="프레임 저장 간격(스텝)")
    args = ap.parse_args()

    # 토크 스케일: analysis와 동일한 Ks/Kd 는 큼 → 충분히 넓은 ctrl 범위 필요.
    opts = LoadOptions(
        timestep=0.001,
        gravity=(0.0, 0.0, -9.80665),
        ctrl_range=(-8000.0, 8000.0),
    )
    model = load_pmi_model(opts)
    data = mujoco.MjData(model)
    ctrl_i = ctrl_indices_for_actuators(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link4")
    if body_id < 0:
        raise RuntimeError("body 'link4' 없음 — URDF 확인")

    h = float(model.opt.timestep)
    stack_x = path_build_full_quintic(WP_T, WP_X, h)
    stack_y = path_build_full_quintic(WP_T, WP_Y, h)
    stack_z = path_build_full_quintic(WP_T, WP_Z, h)
    path_x = stack_x[:, 0]
    path_vx = stack_x[:, 1]
    path_y = stack_y[:, 0]
    path_vy = stack_y[:, 1]
    path_z = stack_z[:, 0]
    path_vz = stack_z[:, 1]
    n_path = len(path_x)
    print(
        f"path len x/y/z = {len(path_x)}, {len(path_y)}, {len(path_z)} "
        f"(timestep={h}, ctrl nu={model.nu})"
    )

    mujoco.mj_resetData(model, data)
    if not args.no_csv_init:
        csv_path = repo_root() / "analysis" / "recurdyn" / "rec_data_path.csv"
        if csv_path.is_file():
            rec = np.loadtxt(csv_path, delimiter=",")[:, 1:]
            apply_initial_from_recurdyn_row(model, data, rec[0])
            print(f"초기 관절: rec_data_path.csv 첫 행 ({csv_path})")
        else:
            print(f"경고: CSV 없음 — 영 자세로 시작 ({csv_path})")
    mujoco.mj_forward(model, data)

    viewer = None
    if args.viewer:
        viewer = mj_viewer.launch_passive(model, data)

    csv_out = Path(args.csv_out).resolve()
    csv_out.parent.mkdir(parents=True, exist_ok=True)

    video_out = Path(args.video_out).resolve() if args.video_out else None
    renderer = None
    frames: list[np.ndarray] = []
    if video_out is not None:
        video_out.parent.mkdir(parents=True, exist_ok=True)
        max_w = model.vis.global_.offwidth
        max_h = model.vis.global_.offheight
        width = min(args.video_width, max_w)
        height = min(args.video_height, max_h)
        renderer = Renderer(model, height=height, width=width)

    csv_rows: list[list[float]] = []
    jnames = actuated_joints_for_model(model)
    qpos_i = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, nm)] for nm in jnames]
    qvel_i = [model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, nm)] for nm in jnames]

    des_roll = -np.pi / 2.0
    des_pitch = 0.0

    try:
        for k in range(min(args.steps, n_path)):
            if viewer is not None and not viewer.is_running():
                break
            des_pos = np.array([path_x[k], path_y[k], path_z[k]], dtype=float)
            des_vel = np.array(
                [path_vx[k], path_vy[k], path_vz[k], 0.0, 0.0],
                dtype=float,
            )
            tau, err, _ev = operational_pd_torque(
                model,
                data,
                body_id,
                des_pos,
                des_roll,
                des_pitch,
                des_vel,
                KS_DEFAULT,
                KD_DEFAULT,
            )
            data.ctrl[ctrl_i] = tau
            mujoco.mj_step(model, data)

            x_now = task_vector(model, data, body_id)
            csv_rows.append(
                [
                    k * h,
                    des_pos[0],
                    des_pos[1],
                    des_pos[2],
                    des_roll,
                    des_pitch,
                    x_now[0],
                    x_now[1],
                    x_now[2],
                    x_now[3],
                    x_now[4],
                    err[0],
                    err[1],
                    err[2],
                    err[3],
                    err[4],
                    tau[0],
                    tau[1],
                    tau[2],
                    tau[3],
                    data.qpos[qpos_i[0]],
                    data.qpos[qpos_i[1]],
                    data.qpos[qpos_i[2]],
                    data.qpos[qpos_i[3]],
                    data.qvel[qvel_i[0]],
                    data.qvel[qvel_i[1]],
                    data.qvel[qvel_i[2]],
                    data.qvel[qvel_i[3]],
                ]
            )

            if renderer is not None and (k % max(1, args.video_frame_skip) == 0):
                renderer.update_scene(data, camera=-1)
                frames.append(np.flipud(renderer.render()).copy())

            if viewer is not None:
                viewer.sync()

            if k % 500 == 0:
                p = err[:3]
                print(f"step {k:5d}  |err_pos|={np.linalg.norm(p):.4f}  "
                      f"tau_max={np.max(np.abs(tau)):.1f}")

        print("완료.")
    finally:
        if viewer is not None:
            viewer.close()
        if renderer is not None:
            renderer.close()

    with csv_out.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "t",
                "des_x", "des_y", "des_z", "des_roll", "des_pitch",
                "act_x", "act_y", "act_z", "act_roll", "act_pitch",
                "err_x", "err_y", "err_z", "err_roll", "err_pitch",
                "tau1", "tau2", "tau3", "tau4",
                "q1", "q2", "q3", "q4",
                "dq1", "dq2", "dq3", "dq4",
            ]
        )
        w.writerows(csv_rows)
    print(f"CSV 저장: {csv_out}")

    if video_out is not None:
        if not frames:
            raise RuntimeError("영상 프레임이 없습니다. --video-frame-skip/--steps 확인 필요")
        imageio.mimsave(
            video_out,
            frames,
            fps=args.video_fps,
            codec="libx264",
            pixelformat="yuv420p",
        )
        print(f"영상 저장: {video_out}")


if __name__ == "__main__":
    main()
