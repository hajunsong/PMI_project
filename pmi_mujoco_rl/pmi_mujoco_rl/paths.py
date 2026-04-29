"""저장소 루트 및 ROS 패키지 경로."""

from __future__ import annotations

from pathlib import Path


def repo_root() -> Path:
    """`pmi_mujoco_rl` 패키지가 들어 있는 저장소 루트 (PMI_project)."""
    return Path(__file__).resolve().parent.parent.parent


def pmi_description_dir() -> Path:
    """`pmi_description` ROS 패키지 디렉터리."""
    return repo_root() / "ros_ws" / "pmi_description"


def pmi_urdf_path() -> Path:
    return pmi_description_dir() / "urdf" / "pmi_description.urdf"
