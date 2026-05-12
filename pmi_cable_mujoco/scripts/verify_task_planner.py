#!/usr/bin/env python3
"""Smoke test: ``kinematics.pmi_chain`` + ``planning.TaskPathPlanner`` vs PMI_Server PathPlanner logic."""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import numpy as np

from kinematics.pmi_chain import fk_ee_pose_joint_rad, joint_rad_from_actuator_rad
from planning.task_path_planner import TaskPathPlanner, Waypoint


def main() -> None:
    q0 = joint_rad_from_actuator_rad(
        np.array([0.445671, 1.906108, -0.01106, -1.66866], dtype=float)
    )
    ee, rpy = fk_ee_pose_joint_rad(q0)
    print("q_jnt0", q0)
    print("EE0", ee, "rpy", rpy)

    wps = [
        Waypoint(0.0, 0.25, -0.2, -0.1),
        Waypoint(0.5, 0.0, -0.35, -0.15),
        Waypoint(1.0, -0.25, -0.2, -0.1),
    ]
    p = TaskPathPlanner()
    p.set_waypoints(wps)
    ok = p.plan(0.02, q0)  # coarser dt for quick smoke test (env uses 0.002)
    print("plan ok", ok, "samples", len(p.path_samples))
    Q, Qd = p.joint_playback_sequence(q0)
    print("playback Q shape", Q.shape, "first row", Q[0], "last row", Q[-1])


if __name__ == "__main__":
    main()
