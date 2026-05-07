#!/usr/bin/env python3
import argparse
import os
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path


SUMMARY_RE = re.compile(
    r"\[SUMMARY\] fe_pos\(avg=([0-9.eE+-]+), peak=([0-9.eE+-]+)\), "
    r"fe_q\(avg=([0-9.eE+-]+), peak=([0-9.eE+-]+)\), "
    r"fe_ori\(avg=([0-9.eE+-]+), peak=([0-9.eE+-]+)\)"
)


@dataclass
class EvalResult:
    score: float
    fe_pos_avg: float
    fe_pos_peak: float
    fe_q_avg: float
    fe_q_peak: float
    fe_ori_avg: float
    fe_ori_peak: float


def score_of(mode: str, fe_pos_avg: float, fe_pos_peak: float, fe_ori_avg: float) -> float:
    if mode == "conservative":
        return fe_pos_peak * 80.0 + fe_pos_avg * 80.0 + fe_ori_avg * 20.0
    if mode == "aggressive":
        return fe_pos_avg * 140.0 + fe_pos_peak * 35.0 + fe_ori_avg * 20.0
    if mode == "q_priority":
        # q-priority mode is handled in run_once with full metric set.
        return 0.0
    return fe_pos_avg * 100.0 + fe_pos_peak * 50.0 + fe_ori_avg * 20.0


def run_once(workdir: Path, ks: list[float], kd: list[float], mode: str) -> EvalResult:
    env = os.environ.copy()
    keys = ["X", "Y", "Z", "ROLL", "PITCH"]
    for i, key in enumerate(keys):
        env[f"VSD_KS_{key}"] = f"{ks[i]}"
        env[f"VSD_KD_{key}"] = f"{kd[i]}"
    p = subprocess.run(["./build/pmi_cpp"], cwd=workdir, text=True, capture_output=True, env=env)
    out = (p.stdout or "") + (p.stderr or "")
    m = SUMMARY_RE.search(out)
    if not m:
        raise RuntimeError("SUMMARY parse failed")
    vals = list(map(float, m.groups()))
    fe_pos_avg, fe_pos_peak, fe_q_avg, fe_q_peak, fe_ori_avg, fe_ori_peak = vals
    if mode == "q_priority":
        score = (
            fe_q_avg * 180.0
            + fe_q_peak * 70.0
            + fe_pos_avg * 20.0
            + fe_pos_peak * 8.0
            + fe_ori_avg * 10.0
        )
    else:
        score = score_of(mode, fe_pos_avg, fe_pos_peak, fe_ori_avg)
    return EvalResult(
        score=score,
        fe_pos_avg=fe_pos_avg,
        fe_pos_peak=fe_pos_peak,
        fe_q_avg=fe_q_avg,
        fe_q_peak=fe_q_peak,
        fe_ori_avg=fe_ori_avg,
        fe_ori_peak=fe_ori_peak,
    )


def main():
    ap = argparse.ArgumentParser(description="Coordinate-descent tuner for per-axis Ks/Kd.")
    ap.add_argument("--mode", choices=["balanced", "conservative", "aggressive", "q_priority"], default="balanced")
    ap.add_argument("--rounds", type=int, default=2)
    ap.add_argument("--workdir", type=Path, default=Path(__file__).resolve().parent)
    args = ap.parse_args()

    ks = [19800.0, 19800.0, 19800.0, 2700.0, 2700.0]
    kd = [1540.0, 1540.0, 1540.0, 16.0, 16.0]
    ratios = [0.85, 0.92, 1.0, 1.08, 1.15]

    best = run_once(args.workdir, ks, kd, args.mode)
    print(f"[BASE] score={best.score:.6f} pos(avg={best.fe_pos_avg:.6f},peak={best.fe_pos_peak:.6f}) "
          f"q(avg={best.fe_q_avg:.6f},peak={best.fe_q_peak:.6f}) ori(avg={best.fe_ori_avg:.6f},peak={best.fe_ori_peak:.6f})")

    for _ in range(args.rounds):
        for idx in range(5):
            local_best_val = ks[idx]
            local_best = best
            for r in ratios:
                trial_ks = ks.copy()
                trial_ks[idx] = ks[idx] * r
                res = run_once(args.workdir, trial_ks, kd, args.mode)
                if res.score < local_best.score:
                    local_best = res
                    local_best_val = trial_ks[idx]
            ks[idx] = local_best_val
            best = local_best

        for idx in range(5):
            local_best_val = kd[idx]
            local_best = best
            for r in ratios:
                trial_kd = kd.copy()
                trial_kd[idx] = kd[idx] * r
                res = run_once(args.workdir, ks, trial_kd, args.mode)
                if res.score < local_best.score:
                    local_best = res
                    local_best_val = trial_kd[idx]
            kd[idx] = local_best_val
            best = local_best

    print("[BEST]")
    print("Ks =", [round(v, 6) for v in ks])
    print("Kd =", [round(v, 6) for v in kd])
    print(f"score={best.score:.6f}")
    print(f"fe_pos(avg={best.fe_pos_avg:.6f}, peak={best.fe_pos_peak:.6f})")
    print(f"fe_q(avg={best.fe_q_avg:.6f}, peak={best.fe_q_peak:.6f})")
    print(f"fe_ori(avg={best.fe_ori_avg:.6f}, peak={best.fe_ori_peak:.6f})")


if __name__ == "__main__":
    main()

