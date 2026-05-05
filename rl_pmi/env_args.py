"""
PMITrackEnv 공통 CLI 인자 — 학습·평가 스크립트에서 동일 환경 설정을 재사용합니다.
"""

from __future__ import annotations

import argparse
from typing import Any, Optional

import numpy as np


def parse_optional_float5(s: Optional[str]) -> Optional[np.ndarray]:
    if s is None or not str(s).strip():
        return None
    parts = [float(x.strip()) for x in str(s).split(",")]
    if len(parts) != 5:
        raise ValueError(
            f"쉼표로 구분한 실수 5개가 필요합니다 (ΔF 또는 Ks/Kd 성분). 입력 개수: {len(parts)}"
        )
    return np.asarray(parts, dtype=float)


def add_pmi_track_env_arguments(parser: argparse.ArgumentParser):
    """``PMITrackEnv`` 생성자와 대응되는 인자 그룹을 등록합니다."""
    g = parser.add_argument_group(
        "PMITrackEnv",
        "학습·평가 시 반드시 같은 값으로 맞추세요 (체크포인트와 분포가 일치해야 함).",
    )
    g.add_argument("--tau-limit", type=float, default=600.0, help="관절 토크 클립 [Nm]")
    g.add_argument(
        "--no-gravity-ff",
        action="store_true",
        help="중력 feedforward(qfrc_bias) 끔",
    )
    g.add_argument(
        "--no-residual-tau-penalty",
        action="store_true",
        help="보상에서 ||Δτ_RL||² 대신 ||a||² 페널티 사용",
    )
    g.add_argument(
        "--w-pos",
        type=float,
        default=1.0,
        help="보상: 위치 오차 가중",
    )
    g.add_argument("--w-rp", type=float, default=0.1, help="보상: roll/pitch 가중")
    g.add_argument("--w-vel", type=float, default=0.01, help="보상: 선속도 오차 가중")
    g.add_argument("--w-omega", type=float, default=0.01, help="보상: 각속도 오차 가중")
    g.add_argument(
        "--action-penalty",
        type=float,
        default=1e-4,
        help="보상: 행동(또는 잔여 토크) 크기 페널티 계수",
    )
    g.add_argument(
        "--reset-noise",
        type=float,
        default=0.05,
        help="reset 시 qpos 균일 노이즈 반폭 (학습·평가 분포 맞추기)",
    )
    g.add_argument(
        "--t-end",
        type=float,
        default=3.0,
        help="에피소드 시간 상한 [s] (내부 스텝 상한과 함께 결정)",
    )
    g.add_argument(
        "--delta-f-scale",
        metavar="F0,F1,F2,F3,F4",
        default=None,
        help="ΔF 스케일 5개 (쉼표). 미지정 시 환경 기본 [80,80,80,40,40]",
    )
    g.add_argument(
        "--ks",
        metavar="K0,...,K4",
        default=None,
        help="F_VSD 위치·자세 게인 5개 (쉼표). 미지정 시 run_vsd 기본",
    )
    g.add_argument(
        "--kd",
        metavar="D0,...,D4",
        default=None,
        help="F_VSD 속도 게인 5개 (쉼표). 미지정 시 run_vsd 기본",
    )
    return g


def pmi_track_env_kwargs(ns: Any, *, render_mode: Optional[str] = None) -> dict[str, Any]:
    """argparse 네임스페이스에서 ``PMITrackEnv(**kwargs)`` 용 dict 생성."""
    kw: dict[str, Any] = {
        "tau_limit": ns.tau_limit,
        "use_gravity_feedforward": not ns.no_gravity_ff,
        "penalize_residual_torque": not ns.no_residual_tau_penalty,
        "w_pos": ns.w_pos,
        "w_rp": ns.w_rp,
        "w_vel": ns.w_vel,
        "w_omega": ns.w_omega,
        "action_penalty": ns.action_penalty,
        "reset_noise": ns.reset_noise,
        "t_end": ns.t_end,
    }
    dfs = parse_optional_float5(getattr(ns, "delta_f_scale", None))
    if dfs is not None:
        kw["delta_f_scale"] = dfs
    ks = parse_optional_float5(getattr(ns, "ks", None))
    if ks is not None:
        kw["ks"] = ks
    kd = parse_optional_float5(getattr(ns, "kd", None))
    if kd is not None:
        kw["kd"] = kd
    if render_mode is not None:
        kw["render_mode"] = render_mode
    return kw
