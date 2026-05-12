#!/usr/bin/env python3
from __future__ import annotations

import ast
import csv
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "debug_outputs" / "task_space_vsd_next"


def read_csv_dict(path: Path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def main() -> None:
    d = read_csv_dict(OUT / "duration_sweep.csv")
    g = read_csv_dict(OUT / "xyz_bias_gain_sweep.csv")
    v = read_csv_dict(OUT / "velocity_feedforward_compare.csv")
    ik = read_csv_dict(OUT / "ik_feasibility.csv")

    rms_d = [float(x["rms_pos"]) for x in d]
    duration_improves = rms_d[-1] < rms_d[0]

    ik_modes = ["xyz", "xyz_pitch", "xyz_roll_pitch"]
    ik_feas = {}
    for m in ik_modes:
        rr = [r for r in ik if r["task_mode"] == m]
        ik_feas[m] = all(x["success"].lower() == "true" for x in rr)

    stable = [r for r in g if r["stable"].lower() == "true"]
    best_rms = min(stable, key=lambda x: float(x["rms_pos"])) if stable else None

    better_ff = min(v, key=lambda x: float(x["rms_pos"]))

    peak_rows = read_csv_dict(OUT / "peak_error_analysis.csv")
    peak = {r["key"]: r["value"] for r in peak_rows}

    # recommendation mapping
    if not ik_feas["xyz"]:
        rec = "B. Modify waypoint/path because it is infeasible"
    elif duration_improves:
        rec = "A. Keep J.T F task-space VSD and tune gains/duration"
    elif ik_feas["xyz"] and (not ik_feas["xyz_pitch"] or not ik_feas["xyz_roll_pitch"]):
        rec = "C. Add resolved-rate or IK-based joint target controller"
    else:
        rec = "D. Move to operational-space control with better dynamics compensation"

    lines = [
        "# Task-space VSD Next Summary",
        "",
        "1. Duration sweep conclusion",
        f"- RMS improve with longer duration: {duration_improves}",
        "2. IK feasibility conclusion",
        f"- xyz feasible: {ik_feas['xyz']}",
        f"- xyz_pitch feasible: {ik_feas['xyz_pitch']}",
        f"- xyz_roll_pitch feasible: {ik_feas['xyz_roll_pitch']}",
        "3. Peak error conclusion",
        f"- peak time: {peak.get('time_peak')}",
        f"- peak near middle waypoint: {abs(float(peak.get('time_peak', '0')) - 0.5) <= 0.15}",
        f"- min joint/act margin: {peak.get('joint_limit_margins_min')} / {peak.get('actuator_limit_margins_min')}",
        "4. Gain sweep conclusion",
        f"- best stable by rms: {best_rms}",
        "5. Desired velocity conclusion",
        f"- better feedforward case by rms: {better_ff['case']}",
        "6. Recommended next controller change",
        f"- {rec}",
    ]

    (OUT / "summary_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
