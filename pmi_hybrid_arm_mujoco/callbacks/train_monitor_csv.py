"""Per-episode training CSV with run metadata (separate from SB3 ``*.monitor.csv``)."""

from __future__ import annotations

import csv
import time
from pathlib import Path
from typing import Any

import gymnasium as gym
from gymnasium.core import ActType, ObsType


class TrainMonitorCsvWrapper(gym.Wrapper[ObsType, ActType, ObsType, ActType]):
    """Logs ``train_monitor.csv`` with ``run_name``, ``seed``, ``profile``, ``eval_type=train_episode``."""

    FIELDS = ("run_name", "seed", "profile", "eval_type", "r", "l", "t")

    def __init__(
        self,
        env: gym.Env,
        csv_path: Path,
        *,
        run_name: str,
        seed: int | None,
        profile: str,
        eval_type: str = "train_episode",
        fresh: bool = True,
    ) -> None:
        super().__init__(env)
        self._path = Path(csv_path)
        self._run_name = str(run_name)
        self._seed = "" if seed is None else str(int(seed))
        self._profile = str(profile)
        self._eval_type = str(eval_type)
        self._t0 = time.time()
        self._rewards: list[float] = []
        self._fh: Any = None
        self._writer: csv.DictWriter | None = None
        self._fresh = bool(fresh)
        self._opened = False

    def _ensure_open(self) -> None:
        if self._opened:
            return
        self._path.parent.mkdir(parents=True, exist_ok=True)
        if self._fresh:
            self._fh = self._path.open("w", newline="", encoding="utf-8")
            self._writer = csv.DictWriter(self._fh, fieldnames=list(self.FIELDS))
            self._writer.writeheader()
        else:
            existed = self._path.is_file()
            self._fh = self._path.open("a", newline="", encoding="utf-8")
            self._writer = csv.DictWriter(self._fh, fieldnames=list(self.FIELDS))
            if not existed or self._path.stat().st_size == 0:
                self._writer.writeheader()
        self._opened = True

    def reset(self, **kwargs):
        self._rewards = []
        return self.env.reset(**kwargs)

    def step(self, action: ActType):
        self._ensure_open()
        obs, reward, terminated, truncated, info = self.env.step(action)
        self._rewards.append(float(reward))
        if terminated or truncated:
            assert self._writer is not None
            ep_rew = sum(self._rewards)
            ep_len = len(self._rewards)
            self._writer.writerow(
                {
                    "run_name": self._run_name,
                    "seed": self._seed,
                    "profile": self._profile,
                    "eval_type": self._eval_type,
                    "r": round(ep_rew, 6),
                    "l": ep_len,
                    "t": round(time.time() - self._t0, 6),
                }
            )
            self._fh.flush()
        return obs, reward, terminated, truncated, info

    def close(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None
            self._writer = None
            self._opened = False
        super().close()
