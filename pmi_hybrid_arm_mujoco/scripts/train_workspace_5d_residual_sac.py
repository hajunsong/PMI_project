#!/usr/bin/env python3
"""SAC training for PMIWorkspace5DResidualEnv (workspace 5D VSD nominal + 5D wrench residual)."""

from __future__ import annotations

import argparse
import copy
import csv
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from utils.mujoco_helpers import load_yaml
from utils.smooth_tracking_metrics import episode_smooth_tracking_metrics
from utils.workspace_5d_rl_metrics import workspace_smooth_score


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=Path, default=_ROOT / "configs" / "rl_workspace_5d_sac.yaml")
    ap.add_argument("--run-name", type=str, required=True)
    ap.add_argument("--timesteps", type=int, default=30000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument(
        "--out-dir",
        type=Path,
        default=_ROOT / "debug_outputs" / "workspace_5d_residual_rl",
    )
    ap.add_argument("--eval-freq", type=int, default=5000)
    ap.add_argument("--eval-episodes", type=int, default=10)
    ap.add_argument("--checkpoint-freq", type=int, default=5000)
    ap.add_argument("--eval-seed-start", type=int, default=10000)
    ap.add_argument("--early-stop", action="store_true")
    ap.add_argument("--min-train-steps-before-stop", type=int, default=20000)
    ap.add_argument("--patience-evals", type=int, default=5)
    ap.add_argument("--min-improvement", type=float, default=0.003)
    ap.add_argument("--progress", action="store_true")
    ap.add_argument("--resume-from", type=Path, default=None)
    ap.add_argument(
        "--curriculum-stage",
        type=str,
        default=None,
        choices=("deterministic", "mild", "medium_v2", "medium_train"),
        help="Overrides curriculum.stage in YAML (default: use config). Order: deterministic → mild → medium_v2 → medium_train.",
    )
    ap.add_argument("--learning-rate", type=float, default=None, help="Override sac.learning_rate")
    ap.add_argument("--residual-force-scale", type=float, default=None, help="Override residual.residual_force_scale")
    ap.add_argument("--residual-moment-scale", type=float, default=None, help="Override residual.residual_moment_scale")
    ap.add_argument(
        "--vecnormalize-path",
        type=Path,
        default=None,
        help="Load VecNormalize obs statistics from this .pkl (e.g. same run as resume checkpoint). "
        "If missing, initializes fresh VecNormalize.",
    )
    ap.add_argument("--use-vecnormalize", dest="use_vecnormalize", action="store_true")
    ap.add_argument("--no-vecnormalize", dest="use_vecnormalize", action="store_false")
    ap.set_defaults(use_vecnormalize=True)
    return ap.parse_args()


def _aggregate_eval(rows: list[dict[str, Any]]) -> dict[str, float]:
    def mean_key(k: str) -> float:
        xs = [float(r[k]) for r in rows if k in r and float(r[k]) == float(r[k])]
        return float(np.mean(xs)) if xs else float("nan")

    m: dict[str, float] = {}
    for key in (
        "rms_ee_error",
        "final_ee_error",
        "rms_roll_error",
        "rms_pitch_error",
        "rms_highfreq",
        "p2p_error_norm",
        "saturation_fraction",
        "limit_fraction",
        "ncon_max",
        "mean_reward",
        "mean_residual_wrench_norm",
        "mean_tau_res_norm",
        "episode_return",
    ):
        m[key] = mean_key(key)
    m["smooth_score"] = workspace_smooth_score(m)
    m["frac_eps_any_ncon"] = float(np.mean([1.0 if float(r.get("ncon_max", 0)) > 0 else 0.0 for r in rows]))
    return m


def rollout_vecenv_episode_metrics(venv: Any, model: SAC, *, seed: int, control_dt: float) -> dict[str, Any]:
    """One episode on training VecEnv (with VecNormalize); restores venv.training flag."""
    was_training = getattr(venv, "training", True)
    if hasattr(venv, "training"):
        venv.training = False
    if hasattr(venv, "norm_reward"):
        venv.norm_reward = False

    venv.seed(int(seed))
    obs = venv.reset()
    if isinstance(obs, tuple):
        obs = obs[0]

    ee_xyz: list[np.ndarray] = []
    ee_norm: list[float] = []
    e_roll: list[float] = []
    e_pitch: list[float] = []
    sat: list[int] = []
    jl: list[int] = []
    al: list[int] = []
    ncon: list[int] = []
    wrenches: list[np.ndarray] = []
    tau_res_l: list[np.ndarray] = []
    rewards: list[float] = []

    n_steps = 0
    while True:
        action, _ = model.predict(obs, deterministic=True)
        step_out = venv.step(action)
        if len(step_out) == 5:
            obs, rewards_vec, terminated, truncated, infos = step_out
            done = np.logical_or(terminated, truncated)
        else:
            obs, rewards_vec, done, infos = step_out
        rewards.append(float(rewards_vec[0]))
        inf = infos[0] if isinstance(infos, list) else infos
        ee_xyz.append(np.asarray(inf["ee_err_xyz"], dtype=np.float64).reshape(3).copy())
        ee_norm.append(float(inf["ee_error_norm"]))
        e_roll.append(float(inf["e_roll"]))
        e_pitch.append(float(inf["e_pitch"]))
        sat.append(int(inf.get("saturation_count", 0)))
        jl.append(int(inf.get("joint_limit_violation", 0)))
        al.append(int(inf.get("actuator_limit_violation", 0)))
        ncon.append(int(inf.get("ncon", 0)))
        wrenches.append(np.asarray(inf.get("W_residual_used_5d", np.zeros(5)), dtype=np.float64).copy())
        tau_res_l.append(np.asarray(inf.get("tau_residual_jnt", np.zeros(4)), dtype=np.float64).copy())
        n_steps += 1
        if bool(done[0]):
            break

    if hasattr(venv, "training"):
        venv.training = was_training
    if hasattr(venv, "norm_reward"):
        venv.norm_reward = False

    exyz = np.stack(ee_xyz, axis=0)
    en = np.asarray(ee_norm, dtype=np.float64)
    re = np.asarray(e_roll, dtype=np.float64)
    pe = np.asarray(e_pitch, dtype=np.float64)
    osc = episode_smooth_tracking_metrics(ee_err_xyz=exyz, ee_err_norm=en, dt=float(control_dt))
    n = max(1, len(sat))
    lim_steps = sum(1 for i in range(len(jl)) if jl[i] or al[i])
    wrenches_arr = np.stack(wrenches, axis=0) if wrenches else np.zeros((0, 5))
    tau_res_arr = np.stack(tau_res_l, axis=0) if tau_res_l else np.zeros((0, 4))
    wnorms = np.linalg.norm(wrenches_arr, axis=1) if wrenches_arr.size else np.array([])
    tnorms = np.linalg.norm(tau_res_arr, axis=1) if tau_res_arr.size else np.array([])

    return {
        "rms_ee_error": float(osc.get("rms_ee", float(np.sqrt(np.mean(en**2))))),
        "final_ee_error": float(en[-1]) if en.size else float("nan"),
        "rms_roll_error": float(np.sqrt(np.mean(re**2))) if re.size else float("nan"),
        "rms_pitch_error": float(np.sqrt(np.mean(pe**2))) if pe.size else float("nan"),
        "rms_highfreq": float(osc.get("rms_ee_error_highfreq", float("nan"))),
        "p2p_error_norm": float(osc.get("p2p_error_norm", float("nan"))),
        "saturation_fraction": float(np.mean(sat)) if sat else 0.0,
        "limit_fraction": float(lim_steps / n),
        "ncon_max": float(max(ncon) if ncon else 0),
        "mean_reward": float(np.mean(rewards)) if rewards else 0.0,
        "episode_return": float(np.sum(rewards)),
        "mean_residual_wrench_norm": float(np.mean(wnorms)) if wnorms.size else 0.0,
        "mean_tau_res_norm": float(np.mean(tnorms)) if tnorms.size else 0.0,
    }


class Workspace5dEvalCallback(BaseCallback):
    def __init__(
        self,
        *,
        run_dir: Path,
        control_dt: float,
        eval_freq: int,
        eval_episodes: int,
        eval_seeds: list[int],
        vec_normalize_path: Path,
        use_vecnormalize: bool,
        early_stop: bool,
        min_train_steps_before_stop: int,
        patience_evals: int,
        min_improvement: float,
    ):
        super().__init__(verbose=0)
        self.run_dir = Path(run_dir)
        self.logs = self.run_dir / "logs"
        self.chk = self.run_dir / "checkpoints"
        self.control_dt = float(control_dt)
        self.eval_freq = int(eval_freq)
        self.eval_episodes = int(eval_episodes)
        self.eval_seeds = list(eval_seeds)
        self.vec_normalize_path = Path(vec_normalize_path)
        self.use_vn = bool(use_vecnormalize)
        self.early_stop = bool(early_stop)
        self.min_train_before_stop = int(min_train_steps_before_stop)
        self.patience_evals = int(patience_evals)
        self.min_improvement = float(min_improvement)

        self._best_rms = float("inf")
        self._best_final = float("inf")
        self._best_smooth = float("inf")
        self._stall_smooth = 0
        self._eval_count = 0

        self.logs.mkdir(parents=True, exist_ok=True)
        self.chk.mkdir(parents=True, exist_ok=True)

    def _append_eval_csv(self, row: dict[str, Any]) -> None:
        p = self.logs / "eval_log.csv"
        new = not p.is_file()
        keys = list(row.keys())
        with open(p, "a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            if new:
                w.writeheader()
            w.writerow(row)

    def _on_step(self) -> bool:
        if self.eval_freq <= 0 or self.n_calls % self.eval_freq != 0:
            return True

        model = self.model
        venv = self.training_env
        assert model is not None

        rows: list[dict[str, Any]] = []
        for sd in self.eval_seeds[: self.eval_episodes]:
            d = rollout_vecenv_episode_metrics(venv, model, seed=int(sd), control_dt=self.control_dt)
            rows.append(d)
        gs = int(self.num_timesteps)

        agg = _aggregate_eval(rows)
        self._eval_count += 1
        row = {
            "global_step": gs,
            "rms_ee_error": agg["rms_ee_error"],
            "final_ee_error": agg["final_ee_error"],
            "smooth_score": agg["smooth_score"],
            "saturation_fraction": agg["saturation_fraction"],
            "limit_fraction": agg["limit_fraction"],
            "mean_reward": agg["mean_reward"],
            "frac_eps_any_ncon": agg["frac_eps_any_ncon"],
        }
        self._append_eval_csv(row)

        prev_best_smooth = float(self._best_smooth)

        if agg["rms_ee_error"] + 1e-12 < self._best_rms:
            self._best_rms = float(agg["rms_ee_error"])
            model.save(str(self.chk / "best_model_by_ee_rms.zip"))
            if self.use_vn and isinstance(venv, VecNormalize):
                venv.save(str(self.vec_normalize_path))
        if agg["final_ee_error"] + 1e-12 < self._best_final:
            self._best_final = float(agg["final_ee_error"])
            model.save(str(self.chk / "best_model_by_final_ee.zip"))
            if self.use_vn and isinstance(venv, VecNormalize):
                venv.save(str(self.vec_normalize_path))
        if agg["smooth_score"] + 1e-12 < self._best_smooth:
            self._best_smooth = float(agg["smooth_score"])
            model.save(str(self.chk / "best_model_by_smooth_score.zip"))
            if self.use_vn and isinstance(venv, VecNormalize):
                venv.save(str(self.vec_normalize_path))

        if self.early_stop and gs >= self.min_train_before_stop:
            if agg["smooth_score"] <= prev_best_smooth - self.min_improvement + 1e-12:
                self._stall_smooth = 0
            else:
                self._stall_smooth += 1
            if self._stall_smooth >= self.patience_evals:
                (self.logs / "early_stop_reason.txt").write_text(
                    f"smooth_score stall {self.patience_evals} evals (best={self._best_smooth:.6f}) at step {gs}\n",
                    encoding="utf-8",
                )
                return False
        return True


def main() -> None:
    args = parse_args()
    cfg = load_yaml(args.config)
    cfg_merged = copy.deepcopy(cfg)
    if args.curriculum_stage is not None:
        cfg_merged.setdefault("curriculum", {})["stage"] = str(args.curriculum_stage)
    if args.learning_rate is not None:
        cfg_merged.setdefault("sac", {})["learning_rate"] = float(args.learning_rate)
    if args.residual_force_scale is not None:
        cfg_merged.setdefault("residual", {})["residual_force_scale"] = float(args.residual_force_scale)
    if args.residual_moment_scale is not None:
        cfg_merged.setdefault("residual", {})["residual_moment_scale"] = float(args.residual_moment_scale)

    sac_cfg = cfg_merged["sac"]
    env_cfg = cfg_merged["env"]
    control_dt = float(env_cfg.get("control_dt", 0.01))

    run_dir = Path(args.out_dir) / "runs" / str(args.run_name)
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "checkpoints").mkdir(parents=True, exist_ok=True)
    (run_dir / "logs").mkdir(parents=True, exist_ok=True)
    (run_dir / "vecnormalize").mkdir(parents=True, exist_ok=True)
    (run_dir / "tensorboard").mkdir(parents=True, exist_ok=True)

    vn_path = run_dir / "vecnormalize" / "vecnormalize.pkl"
    (run_dir / "logs" / "used_config.yaml").write_text(
        yaml.safe_dump(cfg_merged, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    (run_dir / "logs" / "training_args.txt").write_text(
        " ".join(sys.argv[1:]) + "\n", encoding="utf-8"
    )

    if args.resume_from is not None:
        print(
            "[train_workspace_5d_residual_sac] Resume loads policy/Q-network weights from the given .zip. "
            "Replay buffer state may not be restored unless you saved/loaded it separately; if in doubt, "
            "start a fresh run with the same hyperparameters and a new seed.",
            flush=True,
        )
        if args.use_vecnormalize:
            if args.vecnormalize_path is not None and Path(args.vecnormalize_path).is_file():
                print(
                    f"[train_workspace_5d_residual_sac] Using VecNormalize stats from {args.vecnormalize_path}",
                    flush=True,
                )
            else:
                print(
                    "[train_workspace_5d_residual_sac] VecNormalize: pass --vecnormalize-path to the .pkl from the **same** "
                    "run as the checkpoint, or expect observation scaling mismatch.",
                    flush=True,
                )

    seeds_eval = [int(args.eval_seed_start + i) for i in range(int(args.eval_episodes))]

    def make_env() -> Any:
        from envs.pmi_workspace_5d_residual_env import PMIWorkspace5DResidualEnv

        e = PMIWorkspace5DResidualEnv(config=cfg_merged)
        mon = str(run_dir / "logs" / "monitor_train")
        return Monitor(e, filename=mon, allow_early_resets=True)

    venv = DummyVecEnv([make_env])
    venv.seed(int(args.seed))
    if args.use_vecnormalize:
        vn_src = args.vecnormalize_path
        if vn_src is not None and Path(vn_src).is_file():
            venv = VecNormalize.load(str(vn_src), venv)
            print(f"[train_workspace_5d_residual_sac] Loaded VecNormalize from {vn_src}", flush=True)
        else:
            if vn_src is not None:
                print(
                    f"[train_workspace_5d_residual_sac] WARNING: --vecnormalize-path not found ({vn_src}); "
                    "initializing fresh VecNormalize.",
                    flush=True,
                )
            venv = VecNormalize(venv, norm_obs=True, norm_reward=False, clip_obs=10.0)

    ent = sac_cfg.get("ent_coef", "auto")
    if isinstance(ent, str) and ent.strip().lower() != "auto":
        ent = float(ent)

    policy_kwargs = dict(net_arch=[256, 256])
    if args.resume_from is not None:
        model = SAC.load(str(args.resume_from), env=venv, tensorboard_log=str(run_dir / "tensorboard"), verbose=1)
    else:
        model = SAC(
            "MlpPolicy",
            venv,
            learning_rate=float(sac_cfg.get("learning_rate", 1e-4)),
            buffer_size=int(sac_cfg.get("buffer_size", 200000)),
            batch_size=int(sac_cfg.get("batch_size", 256)),
            gamma=float(sac_cfg.get("gamma", 0.99)),
            tau=float(sac_cfg.get("tau", 0.005)),
            train_freq=int(sac_cfg.get("train_freq", 1)),
            gradient_steps=int(sac_cfg.get("gradient_steps", 1)),
            learning_starts=int(sac_cfg.get("learning_starts", 2000)),
            ent_coef=ent,
            verbose=1,
            tensorboard_log=str(run_dir / "tensorboard"),
            seed=int(args.seed),
            policy_kwargs=policy_kwargs,
        )

    eval_cb = Workspace5dEvalCallback(
        run_dir=run_dir,
        control_dt=control_dt,
        eval_freq=int(args.eval_freq),
        eval_episodes=int(args.eval_episodes),
        eval_seeds=seeds_eval,
        vec_normalize_path=vn_path,
        use_vecnormalize=bool(args.use_vecnormalize),
        early_stop=bool(args.early_stop),
        min_train_steps_before_stop=int(args.min_train_steps_before_stop),
        patience_evals=int(args.patience_evals),
        min_improvement=float(args.min_improvement),
    )
    ckpt_cb = CheckpointCallback(
        save_freq=max(int(args.checkpoint_freq), 1),
        save_path=str(run_dir / "checkpoints"),
        name_prefix="checkpoint",
    )

    learned = False
    try:
        model.learn(
            total_timesteps=int(args.timesteps),
            callback=[ckpt_cb, eval_cb],
            progress_bar=bool(args.progress),
        )
        learned = True
    except KeyboardInterrupt:
        pass

    model.save(str(run_dir / "checkpoints" / "final_model.zip"))
    if args.use_vecnormalize and isinstance(venv, VecNormalize):
        venv.save(str(vn_path))

    (run_dir / "logs" / "training_done.txt").write_text(
        f"learned_completed={learned}\ntimesteps_requested={args.timesteps}\n", encoding="utf-8"
    )
    print(f"Saved final model to {run_dir / 'checkpoints' / 'final_model.zip'}")


if __name__ == "__main__":
    main()
