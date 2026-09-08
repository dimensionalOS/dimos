# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Habitat (y-up, -z forward) to dimos/ROS (z-up, x forward) frame conversion."""

from __future__ import annotations

import numpy as np
import numpy.typing as npt

# ros = R_ROS_HAB @ habitat. Proper rotation (det = +1), so handedness and
# rotation sign are preserved: forward -z -> +x, left -x -> +y, up +y -> +z.
R_ROS_HAB: npt.NDArray[np.float64] = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ]
)

# Camera optical (x right, y down, z forward) to ROS body (x forward, y left, z up).
# open3d's create_from_rgbd_image emits points in the optical convention.
R_ROS_OPT: npt.NDArray[np.float64] = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ]
)


# R_ROS_OPT as a quaternion (x, y, z, w) — the transform a `camera` (body) frame
# needs to reach its `camera_optical` child, i.e. REP-103's rpy (-pi/2, 0, -pi/2).
# Without this frame the images hang off a body frame whose +z is UP, and any
# viewer that points the pinhole down +z renders the camera looking at the ceiling.
OPTICAL_QUAT_XYZW: tuple[float, float, float, float] = (-0.5, 0.5, -0.5, 0.5)


def position_to_ros(p_hab: npt.ArrayLike) -> npt.NDArray[np.float64]:
    """Habitat position to ROS."""
    return R_ROS_HAB @ np.asarray(p_hab, dtype=np.float64)


def position_to_habitat(p_ros: npt.ArrayLike) -> npt.NDArray[np.float64]:
    """ROS position to habitat."""
    return R_ROS_HAB.T @ np.asarray(p_ros, dtype=np.float64)


def quat_to_ros(q_hab_wxyz: npt.ArrayLike) -> npt.NDArray[np.float64]:
    """Habitat quaternion (w, x, y, z) to ROS (x, y, z, w).

    Conjugating a rotation by the proper rotation ``R_ROS_HAB`` maps the axis and
    leaves the angle alone, so the scalar part is unchanged and the vector part
    rotates like any other vector.
    """
    q = np.asarray(q_hab_wxyz, dtype=np.float64)
    w, v = q[0], R_ROS_HAB @ q[1:]
    return np.array([v[0], v[1], v[2], w])


def yaw_from_habitat_quat(q_hab_wxyz: npt.ArrayLike) -> float:
    """ROS yaw (about +z) from a habitat yaw-only quaternion."""
    q = np.asarray(q_hab_wxyz, dtype=np.float64)
    return float(2.0 * np.arctan2(q[2], q[0]))


def habitat_quat_from_yaw(yaw: float) -> npt.NDArray[np.float64]:
    """Habitat quaternion (w, x, y, z) for a ROS yaw about +z."""
    return np.array([np.cos(yaw / 2.0), 0.0, np.sin(yaw / 2.0), 0.0])


def habitat_heading(yaw: float) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
    """Habitat-frame (forward, left) unit vectors for a ROS yaw."""
    c, s = np.cos(yaw), np.sin(yaw)
    forward = np.array([-s, 0.0, -c])
    left = np.array([-c, 0.0, s])
    return forward, left


def pose_matrix(position: npt.ArrayLike, yaw: float) -> npt.NDArray[np.float64]:
    """4x4 ROS pose from position and yaw."""
    c, s = np.cos(yaw), np.sin(yaw)
    t = np.eye(4)
    t[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    t[:3, 3] = np.asarray(position, dtype=np.float64)
    return t


def optical_to_body_matrix() -> npt.NDArray[np.float64]:
    """4x4 transform from camera optical frame to ROS body frame."""
    t = np.eye(4)
    t[:3, :3] = R_ROS_OPT
    return t
