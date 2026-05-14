"""Shared rollout metrics for workspace 5D VSD + residual RL."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable

import numpy as np

from envs.pmi_workspace_5d_residual_env import PMIWorkspace5DResidualEnv
from utils.smooth_tracking_metrics import episode_smooth_tracking_metrics


def workspace_smooth_score(m: dict[str, float]) -> float:
    hf = float(m.get("rms_highfreq", 0.0) or 0.0)
    p2p = float(m.get("p2p_error_norm", 0.0) or 0.0)
    rr = float(m.get("rms_roll_error", 0.0) or 0.0)
    pp = float(m.get("rms_pitch_error", 0.0) or 0.0)
    return float(
        m["rms_ee_error"]
        + m["final_ee_error"]
        + 0.5 * hf
        + 0.2 * p2p
        + 0.1 * rr
        + 0.1 * pp
        + 0.1 * m.get("saturation_fraction", 0.0)
        + 0.1 * m.get("limit_fraction", 0.0)
    )


def rollout_one_episode_metrics(
    *,
    policy_fn: Callable[[np.ndarray, int], np.ndarray],
    config_path: Path | None = None,
    config: dict[str, Any] | None = None,
    seed: int,
    options: dict[str, Any] | None = None,
    collect_rgb_frames: bool = False,
    rgb_frame_stride: int = 3,
) -> dict[str, Any]:
    if (config_path is None) == (config is None):
        raise ValueError("Exactly one of config_path or config must be provided.")
    if config is not None:
        env = PMIWorkspace5DResidualEnv(config=config)
    else:
        env = PMIWorkspace5DResidualEnv(config_path=config_path)  # type: ignore[arg-type]
    rgb_frames: list[np.ndarray] = []
    try:
        if collect_rgb_frames:
            env.render_mode = "rgb_array"
        obs, _ = env.reset(seed=seed, options=options or {})
        if collect_rgb_frames:
            fr = env.render()
            if fr is not None:
                rgb_frames.append(np.asarray(fr, dtype=np.uint8))
        dt = float(env.control_dt)

        ee_xyz: list[np.ndarray] = []
        ee_xyz_m: list[np.ndarray] = []
        ee_norm: list[float] = []
        ee_norm_m: list[float] = []
        e_roll: list[float] = []
        e_pitch: list[float] = []
        sat: list[int] = []
        jl: list[int] = []
        al: list[int] = []
        ncon: list[int] = []
        wrenches: list[np.ndarray] = []
        tau_res_l: list[np.ndarray] = []
        rewards: list[float] = []
        t_series: list[float] = []
        x_des: list[np.ndarray] = []
        x_act: list[np.ndarray] = []
        roll_des: list[float] = []
        roll_act: list[float] = []
        pitch_des: list[float] = []
        pitch_act: list[float] = []
        yaw_act: list[float] = []
        ee_hf: list[np.ndarray] = []
        tau_vsd_l: list[np.ndarray] = []
        tau_tot_l: list[np.ndarray] = []
        tau_ideal234: list[np.ndarray] = []
        tau_out234: list[np.ndarray] = []
        tau_act_ideal_full: list[np.ndarray] = []
        tau_act_noisy_full: list[np.ndarray] = []
        hz234: list[np.ndarray] = []

        step_i = 0
        term = trunc = False
        while True:
            a = policy_fn(obs, step_i)
            obs, r, term, trunc, inf = env.step(a)
            rewards.append(float(r))
            ee_xyz.append(np.asarray(inf["ee_err_xyz"], dtype=np.float64).reshape(3).copy())
            ee_norm.append(float(inf["ee_error_norm"]))
            ee_xyz_m.append(
                np.asarray(inf.get("ee_err_xyz_measured", inf["ee_err_xyz"]), dtype=np.float64).reshape(3).copy()
            )
            ee_norm_m.append(float(inf.get("ee_error_norm_measured", inf["ee_error_norm"])))
            e_roll.append(float(inf["e_roll"]))
            e_pitch.append(float(inf["e_pitch"]))
            ee_hf.append(np.asarray(inf.get("ee_err_highfreq_xyz", np.zeros(3)), dtype=np.float64).reshape(3).copy())
            sat.append(int(inf.get("saturation_count", 0)))
            jl.append(int(inf.get("joint_limit_violation", 0)))
            al.append(int(inf.get("actuator_limit_violation", 0)))
            ncon.append(int(inf.get("ncon", 0)))
            wrenches.append(np.asarray(inf.get("W_residual_used_5d", np.zeros(5)), dtype=np.float64).copy())
            tau_res_l.append(np.asarray(inf.get("tau_residual_jnt", np.zeros(4)), dtype=np.float64).copy())
            t_series.append(float(inf.get("time", 0.0)))

            x_des.append(np.asarray(inf["ee_des_xyz"], dtype=np.float64).reshape(3).copy())
            x_act.append(np.asarray(inf["ee_act_xyz"], dtype=np.float64).reshape(3).copy())
            roll_des.append(float(inf["roll_des"]))
            roll_act.append(float(inf["roll"]))
            pitch_des.append(float(inf["pitch_des"]))
            pitch_act.append(float(inf["pitch"]))
            yaw_act.append(float(inf["yaw"]))

            tau_vsd_l.append(np.asarray(inf.get("tau_vsd_jnt", np.zeros(4)), dtype=np.float64).copy())
            tau_tot_l.append(np.asarray(inf.get("tau_jnt_cmd", np.zeros(4)), dtype=np.float64).copy())
            tai = inf.get("tau_act_ideal")
            tao = inf.get("tau_act_out")
            if tai is not None and tao is not None:
                ta = np.asarray(tai, dtype=np.float64).reshape(4)
                tb = np.asarray(tao, dtype=np.float64).reshape(4)
                tau_ideal234.append(ta[1:4].copy())
                tau_out234.append(tb[1:4].copy())
            t_noisy = inf.get("tau_act_noisy_cmd")
            if tai is not None:
                tau_act_ideal_full.append(np.asarray(tai, dtype=np.float64).reshape(4).copy())
            else:
                tau_act_ideal_full.append(np.zeros(4, dtype=np.float64))
            if t_noisy is not None:
                tau_act_noisy_full.append(np.asarray(t_noisy, dtype=np.float64).reshape(4).copy())
            else:
                tau_act_noisy_full.append(tau_act_ideal_full[-1].copy())
            cr = env._transmission.last_cable_result
            if cr is not None:
                hz234.append(np.asarray(cr.hys_z, dtype=np.float64).copy())
            else:
                hz234.append(np.zeros(3, dtype=np.float64))

            if collect_rgb_frames and rgb_frame_stride > 0 and (step_i + 1) % rgb_frame_stride == 0:
                fr = env.render()
                if fr is not None:
                    rgb_frames.append(np.asarray(fr, dtype=np.uint8))

            step_i += 1
            if term or trunc:
                break

        exyz = np.stack(ee_xyz, axis=0)
        exyz_m = np.stack(ee_xyz_m, axis=0) if ee_xyz_m else exyz.copy()
        en = np.asarray(ee_norm, dtype=np.float64)
        en_m = np.asarray(ee_norm_m, dtype=np.float64)
        re = np.asarray(e_roll, dtype=np.float64)
        pe = np.asarray(e_pitch, dtype=np.float64)
        osc = episode_smooth_tracking_metrics(ee_err_xyz=exyz, ee_err_norm=en, dt=float(dt))
        n = max(1, len(sat))
        lim_steps = sum(1 for i in range(len(jl)) if jl[i] or al[i])
        wrenches_arr = np.stack(wrenches, axis=0) if wrenches else np.zeros((0, 5))
        tau_res_arr = np.stack(tau_res_l, axis=0) if tau_res_l else np.zeros((0, 4))

        wnorms = np.linalg.norm(wrenches_arr, axis=1) if wrenches_arr.size else np.array([])
        tnorms = np.linalg.norm(tau_res_arr, axis=1) if tau_res_arr.size else np.array([])
        hf_stack = np.stack(ee_hf, axis=0) if ee_hf else np.zeros((0, 3))
        ee_hf_norm_series = np.linalg.norm(hf_stack, axis=1) if hf_stack.size else np.array([])

        out: dict[str, Any] = {
            "rms_ee_error": float(osc.get("rms_ee", float(np.sqrt(np.mean(en**2))))),
            "rms_ee_error_measured": float(np.sqrt(np.mean(en_m**2))) if en_m.size else float("nan"),
            "final_ee_error": float(en[-1]) if en.size else float("nan"),
            "final_ee_error_measured": float(en_m[-1]) if en_m.size else float("nan"),
            "max_ee_error": float(np.max(en)) if en.size else float("nan"),
            "rms_roll_error": float(np.sqrt(np.mean(re**2))) if re.size else float("nan"),
            "rms_pitch_error": float(np.sqrt(np.mean(pe**2))) if pe.size else float("nan"),
            "rms_highfreq": float(osc.get("rms_ee_error_highfreq", float("nan"))),
            "rms_highfreq_env": float(np.sqrt(np.mean(ee_hf_norm_series**2))) if ee_hf_norm_series.size else float("nan"),
            "p2p_error_norm": float(osc.get("p2p_error_norm", float("nan"))),
            "saturation_fraction": float(np.mean(sat)) if sat else 0.0,
            "limit_fraction": float(lim_steps / n),
            "ncon_max": float(max(ncon) if ncon else 0),
            "mean_reward": float(np.mean(rewards)) if rewards else 0.0,
            "episode_return": float(np.sum(rewards)),
            "mean_residual_wrench_norm": float(np.mean(wnorms)) if wnorms.size else 0.0,
            "mean_tau_res_norm": float(np.mean(tnorms)) if tnorms.size else 0.0,
            "time": np.asarray(t_series, dtype=np.float64),
            "ee_err_xyz": exyz,
            "ee_err_xyz_measured": exyz_m,
            "e_norm": en,
            "e_roll": re,
            "e_pitch": pe,
            "W_residual_used": wrenches_arr,
            "tau_residual": tau_res_arr,
            "tau_vsd": np.stack(tau_vsd_l, axis=0) if tau_vsd_l else np.zeros((0, 4)),
            "tau_jnt_cmd": np.stack(tau_tot_l, axis=0) if tau_tot_l else np.zeros((0, 4)),
            "x_des": np.stack(x_des, axis=0) if x_des else np.zeros((0, 3)),
            "x_act": np.stack(x_act, axis=0) if x_act else np.zeros((0, 3)),
            "tau_ideal_q234": np.stack(tau_ideal234, axis=0) if tau_ideal234 else np.zeros((0, 3)),
            "tau_out_q234": np.stack(tau_out234, axis=0) if tau_out234 else np.zeros((0, 3)),
            "tau_act_ideal_full": np.stack(tau_act_ideal_full, axis=0) if tau_act_ideal_full else np.zeros((0, 4)),
            "tau_act_noisy_full": np.stack(tau_act_noisy_full, axis=0) if tau_act_noisy_full else np.zeros((0, 4)),
            "hys_z_q234": np.stack(hz234, axis=0) if hz234 else np.zeros((0, 3)),
            "roll_des": np.asarray(roll_des, dtype=np.float64),
            "roll_act": np.asarray(roll_act, dtype=np.float64),
            "pitch_des": np.asarray(pitch_des, dtype=np.float64),
            "pitch_act": np.asarray(pitch_act, dtype=np.float64),
            "yaw_act": np.asarray(yaw_act, dtype=np.float64),
            "ee_hf_norm": ee_hf_norm_series,
            "sat_per_step": np.asarray(sat, dtype=np.int32),
            "jl_per_step": np.asarray(jl, dtype=np.int32),
            "al_per_step": np.asarray(al, dtype=np.int32),
            "ncon_per_step": np.asarray(ncon, dtype=np.int32),
        }
        if collect_rgb_frames:
            out["rgb_frames"] = rgb_frames
        return out
    finally:
        env.close()
