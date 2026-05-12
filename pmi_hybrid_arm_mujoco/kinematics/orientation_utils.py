"""Orientation conventions and stable angle helpers.

Euler convention used for roll/pitch/yaw reporting and IK residuals:
-----------------------------------------------------------------------
We use scipy's **extrinsic** rotation sequence ``'xyz'``, i.e.

    R_wb = Rz(yaw) @ Ry(pitch) @ Rx(roll)

applied as post-multiplication ``v_world = R_wb @ v_body`` compatible with
SciPy ``Rotation.from_matrix(R_wb).as_euler('xyz', degrees=False)``
returning ``(roll, pitch, yaw)``.

For **inverse kinematics** we only penalize roll and pitch; yaw may take any value.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as Rrot


def angle_error(target: float, actual: float) -> float:
    """Smallest wrapped difference target - actual on [-pi, pi]."""
    d = float(target) - float(actual)
    return float((d + np.pi) % (2 * np.pi) - np.pi)


def euler_xyz_from_rotation_matrix(r_wb: np.ndarray) -> tuple[float, float, float]:
    """World orientation of body frame rotation matrix ``R_wb``. Returns (roll, pitch, yaw) radians."""
    m = np.asarray(r_wb, dtype=np.float64).reshape(3, 3)
    r = Rrot.from_matrix(m)
    rol, pit, yaw = r.as_euler("xyz", degrees=False)
    return float(rol), float(pit), float(yaw)
