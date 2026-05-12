"""Joint-space reference from EE waypoints — uses ``planning.TaskPathPlanner`` (PMI_Server PathPlanner port)."""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from kinematics.pmi_chain import joint_rad_from_actuator_rad
from planning.task_path_planner import TaskPathPlanner, Waypoint


class WaypointTrajectory:
    """
    Precomputes ``(q_des, qdot_des)`` from absolute-time xyz waypoints + initial joint seed,
    matching ``PMI_Server`` ``PathPlanner`` + playback chaining.
    """

    def __init__(self, traj_cfg: Dict[str, Any], ctrl_dt: float) -> None:
        self._ctrl_dt = float(ctrl_dt)
        self._planner_dt = float(traj_cfg.get("planner_dt", self._ctrl_dt))
        wps = traj_cfg.get("waypoints", [])
        self._waypoints: List[Waypoint] = []
        for w in wps:
            self._waypoints.append(
                Waypoint(float(w["t"]), float(w["x"]), float(w["y"]), float(w["z"]))
            )
        act0 = traj_cfg.get("initial_actuator_rad")
        if act0 is not None:
            self._q0_from_config = joint_rad_from_actuator_rad(np.asarray(act0, dtype=float))
        else:
            self._q0_from_config = None
        self._q_traj: np.ndarray = np.zeros((0, 4))
        self._qd_traj: np.ndarray = np.zeros((0, 4))
        self._xyz_traj: np.ndarray = np.zeros((0, 3))
        self._wps_xyz: np.ndarray = np.array(
            [[float(w["x"]), float(w["y"]), float(w["z"])] for w in wps],
            dtype=np.float64,
        )
        self._k = 0
        self._built = False
        self._idx_last_emitted = 0

    def reset(self, phase_scale: float = 1.0) -> None:
        self._k = 0
        self._built = False
        self._idx_last_emitted = 0

    def on_reset(
        self,
        q_joint_current: np.ndarray,
        *,
        mujoco_data=None,
        joint_qpos_adr: Optional[List[int]] = None,
        apply_yaml_initial_pose: bool = True,
    ) -> None:
        """
        Call after MuJoCo ``mj_resetData`` (and optional pose write).

        If ``initial_actuator_rad`` was in YAML and ``apply_yaml_initial_pose`` is True (기본),
        그 값으로 MuJoCo ``qpos`` 를 덮어쓴 뒤 ``mj_forward`` 하고, 플래너 시드도 그 자세로 맞춘다.

        ``apply_yaml_initial_pose=False`` 이면 **현재** ``q_joint_current`` 만 시드로 쓰고 시뮬 ``qpos`` 는
        건드리지 않는다. 경로 재생 전 ``settle`` 등으로 관절이 이미 움직인 뒤에는 이 플래그로 재플랜해야
        첫 샘플 ``q_des`` 와 실제 ``q`` 가 일치한다.
        """
        q0 = np.asarray(q_joint_current, dtype=np.float64).reshape(4)
        if apply_yaml_initial_pose and self._q0_from_config is not None:
            q0 = self._q0_from_config.copy()
            if mujoco_data is not None and joint_qpos_adr is not None:
                import mujoco

                for i, adr in enumerate(joint_qpos_adr):
                    mujoco_data.qpos[adr] = float(q0[i])
                mujoco.mj_forward(mujoco_data.model, mujoco_data)

        planner = TaskPathPlanner()
        planner.set_waypoints(self._waypoints)
        if len(self._waypoints) < 2 or not planner.plan(self._planner_dt, q0):
            self._q_traj = np.tile(q0, (1, 1))
            self._qd_traj = np.zeros_like(self._q_traj)
            self._xyz_traj = np.zeros((0, 3))
            self._built = True
            self._k = 0
            self._idx_last_emitted = 0
            return

        q_series, qd_series = planner.joint_playback_sequence(q0)
        xyz_series = np.array(
            [[float(s.x), float(s.y), float(s.z)] for s in planner.path_samples],
            dtype=np.float64,
        )
        match_xyz = xyz_series.shape[0] == q_series.shape[0]
        if not match_xyz:
            xyz_series = np.zeros((0, 3), dtype=np.float64)

        # Resample to ctrl_dt if planner_dt differs
        if abs(self._planner_dt - self._ctrl_dt) < 1e-12:
            self._q_traj = q_series
            self._qd_traj = qd_series
            self._xyz_traj = xyz_series if match_xyz else np.zeros((0, 3), dtype=np.float64)
        else:
            t_old = np.arange(len(q_series), dtype=float) * self._planner_dt
            t_end = float(t_old[-1]) if len(t_old) else 0.0
            if t_end <= 0:
                self._q_traj = q_series
                self._qd_traj = qd_series
                self._xyz_traj = xyz_series if match_xyz else np.zeros((0, 3), dtype=np.float64)
            else:
                t_new = np.arange(0.0, t_end + 0.5 * self._ctrl_dt, self._ctrl_dt)
                cols = []
                for j in range(4):
                    cols.append(np.interp(t_new, t_old, q_series[:, j]))
                self._q_traj = np.stack(cols, axis=1)
                cols_d = []
                for j in range(4):
                    cols_d.append(np.interp(t_new, t_old, qd_series[:, j]))
                self._qd_traj = np.stack(cols_d, axis=1)
                if match_xyz and xyz_series.shape[0] > 0:
                    cols_x = []
                    for j in range(3):
                        cols_x.append(np.interp(t_new, t_old, xyz_series[:, j]))
                    self._xyz_traj = np.stack(cols_x, axis=1)
                else:
                    self._xyz_traj = np.zeros((0, 3), dtype=np.float64)

        self._built = True
        self._k = 0
        self._idx_last_emitted = 0

    def peek(self) -> Tuple[np.ndarray, np.ndarray]:
        if not self._built or self._q_traj.shape[0] == 0:
            return np.zeros(4), np.zeros(4)
        k = min(self._k, self._q_traj.shape[0] - 1)
        return self._q_traj[k].copy(), self._qd_traj[k].copy()

    def step(self) -> Tuple[np.ndarray, np.ndarray]:
        out = self.peek()
        if self._built and self._q_traj.shape[0] > 0:
            self._idx_last_emitted = int(min(self._k, self._q_traj.shape[0] - 1))
            self._k = min(self._k + 1, self._q_traj.shape[0] - 1)
        return out

    def cartesian_path_xyz(self) -> np.ndarray:
        """플래너 작업공간 경로 (N×3), 뷰어 폴리라인용."""
        return self._xyz_traj

    def user_waypoints_xyz(self) -> np.ndarray:
        """YAML 사용자 웨이포인트 좌표 (n×3)."""
        return self._wps_xyz

    def workspace_target_xyz_at_step(self) -> Optional[np.ndarray]:
        """방금 ``step()`` 가 방출한 샘플에 대응하는 목표 xyz (없으면 None)."""
        if (
            self._xyz_traj.size == 0
            or self._idx_last_emitted < 0
            or self._idx_last_emitted >= self._xyz_traj.shape[0]
        ):
            return None
        return self._xyz_traj[self._idx_last_emitted].copy()

    def workspace_desired_pos_vel_at_step(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        ``step()`` 직후 호출: 방출 샘플에 대응하는 목표 (x,y,z) 및 수치 미분 선속도 [m/s].

        ``run_vsd`` 의 ``path_*`` 속도와 같이 쓰려면 5D ``des_vel`` 은
        ``np.r_[v_xyz, 0.0, 0.0]`` 로 구성하면 된다.
        """
        xyz = self.workspace_target_xyz_at_step()
        if xyz is None:
            return None, None
        if self._xyz_traj.shape[0] < 2:
            return xyz, np.zeros(3, dtype=np.float64)
        k = int(self._idx_last_emitted)
        n = int(self._xyz_traj.shape[0])
        k = max(0, min(k, n - 1))
        dt = max(float(self._ctrl_dt), 1e-12)
        if k <= 0:
            v = (self._xyz_traj[1] - self._xyz_traj[0]) / dt
        elif k >= n - 1:
            v = (self._xyz_traj[n - 1] - self._xyz_traj[n - 2]) / dt
        else:
            v = (self._xyz_traj[k + 1] - self._xyz_traj[k - 1]) / (2.0 * dt)
        return xyz, v.astype(np.float64)

    def playback_num_steps(self) -> int:
        """``step()`` 한 번이 한 샘플이므로, 경로 1회 재생에 필요한 제어 스텝 수."""
        if not self._built or self._q_traj.shape[0] == 0:
            return 0
        return int(self._q_traj.shape[0])

    def final_waypoint_xyz_world(self) -> np.ndarray:
        """YAML 웨이포인트 중 ``t`` 가 가장 큰 점의 (x,y,z) — EE 도착 판정 목표."""
        if not self._waypoints:
            return np.zeros(3, dtype=np.float64)
        w = max(self._waypoints, key=lambda wp: wp.t)
        return np.array([w.x, w.y, w.z], dtype=np.float64)
