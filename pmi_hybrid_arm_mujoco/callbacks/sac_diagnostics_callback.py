"""Periodic SAC eval, rollout stats, checkpoints, optional early-stop (physics unchanged)."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Sequence

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from envs.pmi_cable_residual_env import PMICableResidualEnv, _deep_merge


ACTION_L2_MAX = float(np.sqrt(3.0))

META_FIELDS = ("run_name", "seed", "profile", "eval_type")

EVAL_FIELDS = META_FIELDS + (
    "global_step",
    "utc_iso",
    "mean_episode_return",
    "std_episode_return",
    "mean_episode_length",
    "mean_ee_rms",
    "std_ee_rms_mean_across_eps",
    "mean_sat_frac",
    "mean_lim_frac",
    "frac_eps_any_ncon",
    "mean_q_err_rms",
    "mean_finite_frac",
    "mean_action_norm",
    "max_action_norm",
    "mean_residual_force_norm",
    "max_residual_force_norm",
    "mean_residual_tau_norm",
    "max_residual_tau_norm",
    "mean_action_rate",
)

ROLLOUT_FIELDS = META_FIELDS + (
    "global_step_after_rollout",
    "utc_iso",
    "rollout_steps",
    "mean_sat_frac",
    "mean_lim_frac",
    "mean_action_l2_ratio",
    "mean_max_abs_action",
)

CALLBACK_FIELDS = META_FIELDS + ("global_step", "utc_iso", "logger_keys_concat")


def clear_run_csv_logs(log_dir: Path, *, resume: bool) -> None:
    """새 런에서 이전 런과 CSV가 섞이지 않도록 삭제. ``--resume-from`` 사용 시 유지."""
    if resume:
        return
    log_dir = Path(log_dir)
    for fn in (
        "eval_log.csv",
        "callback_metrics.csv",
        "rollout_diagnostics.csv",
        "train_monitor.csv",
        "monitor.csv",
    ):
        p = log_dir / fn
        if p.is_file():
            p.unlink()
    # SB3 Monitor 기본 이름
    for p in log_dir.glob("*.monitor.csv"):
        p.unlink(missing_ok=True)


def _utc_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _csv_append(path: Path, fields: tuple[str, ...], row: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    new_file = not path.is_file()
    with path.open("a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(fields))
        if new_file:
            w.writeheader()
        w.writerow({k: row.get(k, "") for k in fields})


def write_eval_baseline_row(
    path: Path,
    *,
    run_name: str,
    seed: int | None,
    profile: str,
    metrics: dict[str, float],
) -> None:
    row: dict[str, Any] = {
        "run_name": run_name,
        "seed": "" if seed is None else str(int(seed)),
        "profile": profile,
        "eval_type": "baseline_zero",
        "global_step": 0,
        "utc_iso": _utc_iso(),
    }
    metric_keys = (
        "mean_episode_return",
        "std_episode_return",
        "mean_episode_length",
        "mean_ee_rms",
        "std_ee_rms_mean_across_eps",
        "mean_sat_frac",
        "mean_lim_frac",
        "frac_eps_any_ncon",
        "mean_q_err_rms",
        "mean_finite_frac",
        "mean_action_norm",
        "max_action_norm",
        "mean_residual_force_norm",
        "max_residual_force_norm",
        "mean_residual_tau_norm",
        "max_residual_tau_norm",
        "mean_action_rate",
    )
    for k in metric_keys:
        row[k] = metrics.get(k, "")
    _csv_append(path, EVAL_FIELDS, row)


def make_env_factory(
    config_path: Path,
    profile: str,
    config_overrides: dict[str, Any] | None = None,
) -> Callable[[], PMICableResidualEnv]:
    base = {"env": {"randomization_profile": str(profile), "randomize_cable": True}}
    merged = _deep_merge(base, config_overrides) if config_overrides else base

    def _make() -> PMICableResidualEnv:
        return PMICableResidualEnv(config_path=config_path, overrides=merged)

    return _make


def rollout_one_episode_vec(
    *,
    obs_policy: Callable[[np.ndarray], np.ndarray],
    config_path: Path,
    profile: str,
    seed: int,
    vec_normalize_path: Path | None,
    config_overrides: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Single episode on one DummyVecEnv; fresh env factory so reset(seed, options) is honored."""

    mk = make_env_factory(config_path, profile, config_overrides)
    v: Any = DummyVecEnv([mk])
    if vec_normalize_path is not None and Path(vec_normalize_path).is_file():
        v = VecNormalize.load(str(vec_normalize_path), v)
        v.training = False
        v.norm_reward = False

    v.seed(int(seed))
    v.set_options({"randomization_profile": str(profile), "randomize_cable": True})
    obs = v.reset()

    ret = 0.0
    ee: list[float] = []
    qe: list[float] = []
    sat_steps = 0
    lim_steps = 0
    steps = 0
    saw_ncon = False
    any_nan_obs = False
    finite = True

    anorms: list[float] = []
    axmax: list[float] = []
    fnorms: list[float] = []
    fxmax: list[float] = []
    tnlist: list[float] = []
    ttmax: list[float] = []
    frates: list[float] = []
    f_prev_vec = np.zeros(3, dtype=np.float64)

    while True:
        if not np.isfinite(obs).all():
            any_nan_obs = True
            finite = False
            break
        act = obs_policy(obs)
        if isinstance(act, tuple):
            act = act[0]
        aflat = np.asarray(act, dtype=np.float64).reshape(-1)
        anorms.append(float(np.linalg.norm(aflat)))
        axmax.append(float(np.max(np.abs(aflat))))
        obs, rew, done, infos = v.step(act)
        ret += float(rew[0])
        inf = infos[0]
        F = np.asarray(inf["F_residual_xyz"], dtype=np.float64).reshape(3)
        tau = np.asarray(inf["tau_residual_jnt"], dtype=np.float64).reshape(4)
        fnorms.append(float(np.linalg.norm(F)))
        fxmax.append(float(np.max(np.abs(F))))
        tnlist.append(float(np.linalg.norm(tau)))
        ttmax.append(float(np.max(np.abs(tau))))
        frates.append(float(np.linalg.norm(F - f_prev_vec)))
        f_prev_vec = F.copy()

        ee.append(float(inf.get("ee_error_norm", np.nan)))
        qe.append(float(inf.get("q_error_norm", np.nan)))
        sat_steps += int(bool(inf.get("saturation_count")))
        jl = int(inf.get("joint_limit_violation", 0))
        al = int(inf.get("actuator_limit_violation", 0))
        if jl or al:
            lim_steps += 1
        saw_ncon = saw_ncon or int(inf.get("ncon", 0)) > 0
        finite = finite and np.isfinite(obs).all() and np.isfinite(float(rew[0]))
        steps += 1
        if bool(done[0]):
            break

    ee_a = np.asarray(ee, dtype=np.float64)
    q_a = np.asarray(qe, dtype=np.float64)
    rms_ee = float(np.sqrt(np.nanmean(ee_a**2))) if ee_a.size else float("nan")
    rms_q = float(np.sqrt(np.nanmean(q_a**2))) if q_a.size else float("nan")

    fr_sat = sat_steps / max(1, steps)
    fr_lim = lim_steps / max(1, steps)
    v.close()
    return {
        "episode_return": ret,
        "episode_length": float(steps),
        "rms_ee": rms_ee,
        "rms_q": rms_q,
        "sat_frac": fr_sat,
        "lim_frac": fr_lim,
        "any_ncon": saw_ncon,
        "finite": finite,
        "nan_obs_early": bool(any_nan_obs),
        "mean_action_norm": float(np.mean(anorms)) if anorms else float("nan"),
        "max_action_norm": float(np.max(axmax)) if axmax else float("nan"),
        "mean_residual_force_norm": float(np.mean(fnorms)) if fnorms else float("nan"),
        "max_residual_force_norm": float(np.max(fxmax)) if fxmax else float("nan"),
        "mean_residual_tau_norm": float(np.mean(tnlist)) if tnlist else float("nan"),
        "max_residual_tau_norm": float(np.max(ttmax)) if ttmax else float("nan"),
        "mean_action_rate": float(np.mean(frates)) if frates else float("nan"),
    }


def evaluate_policy_vec(
    obs_policy: Callable[[np.ndarray], np.ndarray],
    *,
    config_path: Path,
    profile: str,
    n_episodes: int,
    seed_base: int,
    vec_normalize_path: Path | None,
    config_overrides: dict[str, Any] | None = None,
    episode_seeds: Sequence[int] | None = None,
) -> dict[str, float]:
    """If ``episode_seeds`` is set (length ``n_episodes``), those env seeds are used (reproducible eval)."""
    n = int(n_episodes)
    if episode_seeds is not None:
        seeds_list = [int(x) for x in episode_seeds]
        if len(seeds_list) != n:
            raise ValueError(f"episode_seeds length {len(seeds_list)} != n_episodes {n}")
    else:
        rng = np.random.default_rng(int(seed_base))
        seeds_list = [int(rng.integers(0, 2**31 - 1)) for _ in range(n)]

    eps_ret: list[float] = []
    eps_len: list[float] = []
    ee_rmss: list[float] = []
    q_rmss: list[float] = []
    sat_fracs: list[float] = []
    lim_fracs: list[float] = []
    ncon_flags: list[float] = []
    finite_eps: list[float] = []
    man: list[float] = []
    xa: list[float] = []
    mfn: list[float] = []
    xfn: list[float] = []
    mtn: list[float] = []
    xtn: list[float] = []
    mar: list[float] = []

    for sid in seeds_list:
        d = rollout_one_episode_vec(
            obs_policy=obs_policy,
            config_path=config_path,
            profile=profile,
            seed=sid,
            vec_normalize_path=vec_normalize_path,
            config_overrides=config_overrides,
        )
        eps_ret.append(float(d["episode_return"]))
        eps_len.append(float(d["episode_length"]))
        ee_rmss.append(float(d["rms_ee"]))
        q_rmss.append(float(d["rms_q"]))
        sat_fracs.append(float(d["sat_frac"]))
        lim_fracs.append(float(d["lim_frac"]))
        ncon_flags.append(float(d["any_ncon"]))
        finite_eps.append(1.0 if bool(d["finite"]) else 0.0)
        man.append(float(d["mean_action_norm"]))
        xa.append(float(d["max_action_norm"]))
        mfn.append(float(d["mean_residual_force_norm"]))
        xfn.append(float(d["max_residual_force_norm"]))
        mtn.append(float(d["mean_residual_tau_norm"]))
        xtn.append(float(d["max_residual_tau_norm"]))
        mar.append(float(d["mean_action_rate"]))

    er = np.asarray(eps_ret, dtype=np.float64)
    ee = np.asarray(ee_rmss, dtype=np.float64)
    return {
        "mean_episode_return": float(np.mean(er)),
        "std_episode_return": float(np.std(er)),
        "mean_episode_length": float(np.mean(eps_len)),
        "mean_ee_rms": float(np.nanmean(ee)),
        "std_ee_rms_mean_across_eps": float(np.nanstd(ee)),
        "mean_sat_frac": float(np.mean(sat_fracs)),
        "mean_lim_frac": float(np.mean(lim_fracs)),
        "frac_eps_any_ncon": float(np.mean(ncon_flags)),
        "mean_q_err_rms": float(np.nanmean(np.asarray(q_rmss, dtype=np.float64))),
        "mean_finite_frac": float(np.mean(np.asarray(finite_eps, dtype=np.float64))) if finite_eps else float("nan"),
        "mean_action_norm": float(np.nanmean(np.asarray(man, dtype=np.float64))) if man else float("nan"),
        "max_action_norm": float(np.nanmax(np.asarray(xa, dtype=np.float64))) if xa else float("nan"),
        "mean_residual_force_norm": float(np.nanmean(np.asarray(mfn, dtype=np.float64))) if mfn else float("nan"),
        "max_residual_force_norm": float(np.nanmax(np.asarray(xfn, dtype=np.float64))) if xfn else float("nan"),
        "mean_residual_tau_norm": float(np.nanmean(np.asarray(mtn, dtype=np.float64))) if mtn else float("nan"),
        "max_residual_tau_norm": float(np.nanmax(np.asarray(xtn, dtype=np.float64))) if xtn else float("nan"),
        "mean_action_rate": float(np.nanmean(np.asarray(mar, dtype=np.float64))) if mar else float("nan"),
    }


@dataclass
class EarlyStopConfig:
    enabled: bool = False
    patience_evals: int = 8
    min_improvement: float = 0.01
    min_train_steps_before_stop: int = 30000
    min_return_vs_baseline: float | None = None
    saturation_margin_over_baseline: float = 0.10
    limit_margin_over_baseline: float = 0.01
    hard_max_saturation_fraction: float = 0.95
    hard_max_limit_violation_fraction: float = 0.10
    max_saturation_step_frac: float | None = None
    max_limit_step_frac: float | None = None
    max_mean_ee_error: float | None = None
    action_l2_collapse_frac: float = 0.02
    action_abs_explosion: float = 1.02
    anomaly_patience_evals: int = 4


class SACDiagnosticsCallback(BaseCallback):
    """Logs logs/eval_log.csv, logs/rollout_diagnostics.csv, logs/callback_metrics.csv.

    Saves checkpoints/ checkpoint_<step>.zip and best_model; sets ``stopped_reason`` for stopped_model.zip.
    """

    def __init__(
        self,
        *,
        run_dir: Path,
        run_name: str,
        seed: int | None,
        config_path: Path,
        profile: str,
        eval_freq: int,
        eval_episodes: int,
        checkpoint_freq: int,
        vec_normalize_save_path: Path | None,
        eval_use_vec_normalize: bool,
        early: EarlyStopConfig,
        baseline_mean_return: float | None = None,
        baseline_sat_frac: float | None = None,
        baseline_lim_frac: float | None = None,
        rl_config_overrides: dict[str, Any] | None = None,
        eval_episode_seeds: Sequence[int] | None = None,
        verbose: int = 0,
    ) -> None:
        super().__init__(verbose)
        self.run_dir = Path(run_dir)
        self.run_name = str(run_name)
        self.seed_csv = "" if seed is None else str(int(seed))
        self.profile_p = str(profile)
        self.log_dir = self.run_dir / "logs"
        self.chk_dir = self.run_dir / "checkpoints"
        self.config_path = Path(config_path)
        self.profile = str(profile)
        self.eval_freq = int(eval_freq)
        self.eval_episodes = int(eval_episodes)
        self.checkpoint_freq = int(checkpoint_freq)
        self.vn_save_path = Path(vec_normalize_save_path) if vec_normalize_save_path else None
        self.eval_use_vn = bool(eval_use_vec_normalize)
        self.early = early
        self.baseline_mean_ret = baseline_mean_return
        self.baseline_sat_frac = baseline_sat_frac
        self.baseline_lim_frac = baseline_lim_frac
        self.rl_cfg_ov = rl_config_overrides
        seeds = [int(x) for x in eval_episode_seeds] if eval_episode_seeds is not None else None
        if seeds is not None and len(seeds) != self.eval_episodes:
            raise ValueError(f"eval_episode_seeds length {len(seeds)} != eval_episodes {self.eval_episodes}")
        self._eval_episode_seeds: list[int] | None = seeds

        self._last_eval_bucket = -1
        self._running_eval_best_ret = float("-inf")
        self._best_mean_ret = float("-inf")
        self._no_improve_evals = 0
        self._anomaly_streak = 0

        self.stopped_reason: str | None = None
        self._rollout_sat = self._rollout_lim = self._rollout_al2 = self._rollout_amax_sum = self._rollout_steps = 0
        self._roll_bad_streak = 0
        self._last_ckpt_bucket = -1

    def _on_training_start(self) -> None:
        self.chk_dir.mkdir(parents=True, exist_ok=True)
        self.log_dir.mkdir(parents=True, exist_ok=True)

    def _csv_meta_eval(self, eval_type: str) -> dict[str, Any]:
        return {"run_name": self.run_name, "seed": self.seed_csv, "profile": self.profile_p, "eval_type": eval_type}

    # --- rollout aggregates ---
    def _on_rollout_start(self) -> None:
        self._rollout_sat = self._rollout_lim = self._rollout_al2 = self._rollout_amax_sum = self._rollout_steps = 0

    def _on_rollout_end(self) -> None:
        if self._rollout_steps <= 0:
            return
        rs = float(self._rollout_steps)
        mratio = (self._rollout_al2 / rs) / ACTION_L2_MAX
        mam = self._rollout_amax_sum / rs
        row = {
            **self._csv_meta_eval("train_rollout"),
            "global_step_after_rollout": int(self.num_timesteps),
            "utc_iso": _utc_iso(),
            "rollout_steps": int(self._rollout_steps),
            "mean_sat_frac": self._rollout_sat / rs,
            "mean_lim_frac": self._rollout_lim / rs,
            "mean_action_l2_ratio": mratio,
            "mean_max_abs_action": mam,
        }
        _csv_append(self.log_dir / "rollout_diagnostics.csv", ROLLOUT_FIELDS, row)
        self._rollout_early_signals(mratio, mam)

    def _update_rollout_locals(self, loc: dict[str, Any]) -> None:
        infos = loc.get("infos")
        actions = loc.get("actions")
        if infos is None or actions is None:
            return
        self._rollout_steps += 1

        acts = np.asarray(actions, dtype=np.float64)
        ai = infos[0] if isinstance(infos, (list, tuple)) and infos else infos
        if isinstance(ai, dict):
            self._rollout_sat += int(bool(ai.get("saturation_count")))
            jl = int(ai.get("joint_limit_violation", 0))
            al = int(ai.get("actuator_limit_violation", 0))
            if jl or al:
                self._rollout_lim += 1

        a0 = acts[0].reshape(-1)
        na = np.linalg.norm(a0)
        self._rollout_al2 += float(na)
        self._rollout_amax_sum += float(np.max(np.abs(a0)))

    def _logger_snapshot_row(self) -> dict[str, Any]:
        keys_txt = ""
        logger = getattr(self.model, "logger", None)
        ntov = getattr(logger, "name_to_value", None) if logger is not None else None
        if isinstance(ntov, dict) and ntov:
            parts: list[str] = []
            for k in sorted(ntov.keys())[-32:]:
                v = ntov[k]
                try:
                    if isinstance(v, (int, np.integer)):
                        parts.append(f"{k}={int(v)}")
                    elif isinstance(v, (float, np.floating)):
                        fv = float(v)
                        if np.isnan(fv) or np.isinf(fv):
                            parts.append(f"{k}=non_finite")
                        else:
                            parts.append(f"{k}={fv:.6g}")
                    else:
                        parts.append(f"{k}={str(v)[:220]}")
                except Exception:
                    parts.append(f"{k}=<?>")
            keys_txt = ";".join(parts)
        return {
            "global_step": self.num_timesteps,
            "utc_iso": _utc_iso(),
            "logger_keys_concat": keys_txt[:8000],
        }

    def _vn_path_eval(self) -> Path | None:
        if self.eval_use_vn and self.vn_save_path is not None and self.vn_save_path.is_file():
            return self.vn_save_path
        return None

    def _save_checkpoint_zip(self, name_stem: str) -> Path:
        self.chk_dir.mkdir(parents=True, exist_ok=True)
        path_prefix = self.chk_dir / name_stem
        assert self.model is not None
        self.model.save(str(path_prefix))
        return Path(str(path_prefix) + ".zip")

    def _maybe_periodic_checkpoint(self) -> None:
        if self.checkpoint_freq <= 0 or self.num_timesteps <= 0:
            return
        bucket = self.num_timesteps // self.checkpoint_freq
        if bucket <= self._last_ckpt_bucket:
            return
        self._last_ckpt_bucket = bucket
        stem = f"checkpoint_{int(self.num_timesteps)}"
        try:
            self._save_checkpoint_zip(stem)
        except Exception:
            pass

    def _maybe_periodic_eval(self) -> None:
        if self.eval_freq <= 0 or self.num_timesteps <= 0:
            return
        bucket = self.num_timesteps // self.eval_freq
        if bucket <= self._last_eval_bucket:
            return
        self._last_eval_bucket = bucket
        try:
            self._run_evaluation_once()
        except Exception as exc:
            self.stopped_reason = f"eval_failed:{exc}"

    def _evaluate_policy_metrics(self) -> dict[str, float]:
        assert self.model is not None

        def policy(obs_batch: np.ndarray) -> np.ndarray:
            a, _ = self.model.predict(obs_batch, deterministic=True)
            return np.asarray(a, dtype=np.float32)

        vn = self._vn_path_eval()
        kw: dict[str, Any] = dict(
            config_path=self.config_path,
            profile=self.profile,
            n_episodes=self.eval_episodes,
            seed_base=0,
            vec_normalize_path=vn,
            config_overrides=self.rl_cfg_ov,
        )
        if self._eval_episode_seeds is not None:
            kw["episode_seeds"] = self._eval_episode_seeds
        return evaluate_policy_vec(policy, **kw)

    def _append_eval_csv(self, metrics_f: dict[str, float], eval_type: str, global_step: int) -> None:
        fk_num = (
            "mean_episode_return",
            "std_episode_return",
            "mean_episode_length",
            "mean_ee_rms",
            "std_ee_rms_mean_across_eps",
            "mean_sat_frac",
            "mean_lim_frac",
            "frac_eps_any_ncon",
            "mean_q_err_rms",
            "mean_finite_frac",
            "mean_action_norm",
            "max_action_norm",
            "mean_residual_force_norm",
            "max_residual_force_norm",
            "mean_residual_tau_norm",
            "max_residual_tau_norm",
            "mean_action_rate",
        )
        row_eval: dict[str, Any] = {
            **self._csv_meta_eval(eval_type),
            "global_step": int(global_step),
            "utc_iso": _utc_iso(),
            **{fk: metrics_f[fk] for fk in fk_num},
        }
        _csv_append(self.log_dir / "eval_log.csv", EVAL_FIELDS, row_eval)

    def _append_callback_snap(self, global_step: int) -> None:
        row_cb = self._csv_meta_eval("callback_followup_periodic_eval")
        row_cb.update(self._logger_snapshot_row())
        row_cb["global_step"] = int(global_step)
        _csv_append(self.log_dir / "callback_metrics.csv", CALLBACK_FIELDS, row_cb)

    def _run_evaluation_once(self) -> dict[str, float]:
        metrics_f = self._evaluate_policy_metrics()
        gs = int(self.num_timesteps)
        score = float(metrics_f["mean_episode_return"])

        self._append_eval_csv(metrics_f, "periodic_eval", gs)
        self._append_callback_snap(gs)

        if score > self._running_eval_best_ret + float(self.early.min_improvement):
            self._running_eval_best_ret = float(score)
            self._no_improve_evals = 0
        else:
            self._no_improve_evals += 1

        if score > self._best_mean_ret:
            self._best_mean_ret = float(score)
            try:
                self._save_checkpoint_zip("best_model")
            except Exception:
                pass

        self._try_eval_early_stop(score, metrics_f)
        return metrics_f

    def record_final_eval_csv(self, *, timestep_override: int | None = None) -> dict[str, float] | None:
        """평가 CSV에 ``final_eval`` 한 줄 추가(조기종료 카운터·best checkpoint 갱신 없음)."""
        if self.model is None:
            return None
        ts = timestep_override if timestep_override is not None else self.num_timesteps
        metrics_f = self._evaluate_policy_metrics()
        self._append_eval_csv(metrics_f, "final_eval", int(ts))
        return metrics_f

    def _try_eval_early_stop(self, score: float, metrics_f: dict[str, float]) -> None:
        if self.stopped_reason is not None or not self.early.enabled:
            return
        ts = int(self.num_timesteps)

        sat = float(metrics_f["mean_sat_frac"])
        lim = float(metrics_f["mean_lim_frac"])
        bs = self.baseline_sat_frac
        bl = self.baseline_lim_frac

        unhealthy_sat = (
            bs is not None and sat > float(bs) + float(self.early.saturation_margin_over_baseline)
        )
        unhealthy_lim = (
            bl is not None and lim > float(bl) + float(self.early.limit_margin_over_baseline)
        )
        hard_sat = sat > float(self.early.hard_max_saturation_fraction)
        hard_lim = lim > float(self.early.hard_max_limit_violation_fraction)

        emergency = (
            float(metrics_f["mean_finite_frac"]) < 1.0 - 1e-6
            or float(metrics_f["frac_eps_any_ncon"]) > 0.0
            or hard_sat
            or hard_lim
        )
        if emergency:
            self.stopped_reason = "early_stop:eval_emergency"
            return

        if ts < int(self.early.min_train_steps_before_stop):
            self._anomaly_streak = 0
            return

        thresh_soft = bool(unhealthy_sat or unhealthy_lim)
        if (
            self.early.max_saturation_step_frac is not None
            and sat > float(self.early.max_saturation_step_frac)
        ):
            thresh_soft = True
        if self.early.max_limit_step_frac is not None and lim > float(self.early.max_limit_step_frac):
            thresh_soft = True
        if (
            self.early.max_mean_ee_error is not None
            and float(metrics_f["mean_ee_rms"]) > float(self.early.max_mean_ee_error)
        ):
            thresh_soft = True

        baseline_fail = False
        if (
            self.baseline_mean_ret is not None
            and self.early.min_return_vs_baseline is not None
            and score < float(self.baseline_mean_ret) + float(self.early.min_return_vs_baseline)
        ):
            baseline_fail = True

        if thresh_soft or baseline_fail:
            self._anomaly_streak += 1
        else:
            self._anomaly_streak = 0

        plateau = self.early.patience_evals > 0 and self._no_improve_evals >= int(self.early.patience_evals)

        if plateau:
            self.stopped_reason = "early_stop:eval_plateau_patience"
        elif (
            self._anomaly_streak >= int(self.early.anomaly_patience_evals)
            and (thresh_soft or baseline_fail)
        ):
            self.stopped_reason = "early_stop:eval_threshold_patience"

    def _rollout_early_signals(self, mratio: float, mam: float) -> None:
        if self.stopped_reason is not None or not self.early.enabled:
            return
        ts = int(self.num_timesteps)
        if ts < int(self.early.min_train_steps_before_stop):
            return
        rs = float(self._rollout_steps)
        if rs < 32.0:
            return

        collapse = mratio <= float(self.early.action_l2_collapse_frac)
        explode = mam >= float(self.early.action_abs_explosion)
        roll_bad = collapse or explode
        if roll_bad:
            self._roll_bad_streak += 1
        else:
            self._roll_bad_streak = 0
            return

        rp = int(self.early.anomaly_patience_evals)
        if self._roll_bad_streak < rp:
            return
        if explode:
            self.stopped_reason = "early_stop:rollout_action_explosion_patience"
        elif collapse:
            self.stopped_reason = "early_stop:rollout_action_collapse_patience"

    def _on_step(self) -> bool:
        loc = getattr(self, "locals", None)
        if isinstance(loc, dict):
            self._update_rollout_locals(loc)
        self._maybe_periodic_checkpoint()
        self._maybe_periodic_eval()
        return self.stopped_reason is None

