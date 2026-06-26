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

"""Devkit-verified self-tests for the official KITTI odometry error metric."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.navigation.jnav.utils.kitti import kitti_odometry_error

ZERO_ERROR_TOLERANCE = 1e-6


def _straight_trajectory(scale: float = 1.0) -> list[np.ndarray]:
    """A 1000 m straight run along x, one frame per metre, scaled along-track."""
    poses = []
    for index in range(1001):
        pose = np.eye(4)
        pose[0, 3] = float(index) * scale
        poses.append(pose)
    return poses


def test_identical_trajectory_has_zero_error() -> None:
    ground_truth = _straight_trajectory()
    result = kitti_odometry_error([pose.copy() for pose in ground_truth], ground_truth)
    assert result["translational_percent"] < ZERO_ERROR_TOLERANCE
    assert result["rotational_deg_per_m"] < ZERO_ERROR_TOLERANCE


def test_one_percent_scale_drift_reads_one_percent() -> None:
    ground_truth = _straight_trajectory()
    scaled = _straight_trajectory(scale=1.01)
    result = kitti_odometry_error(scaled, ground_truth)
    assert 0.8 < result["translational_percent"] < 1.2


def test_constant_yaw_rate_has_nonzero_rotational_error() -> None:
    ground_truth = _straight_trajectory()
    yawed = []
    for index in range(1001):
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_euler("z", index * 0.01, degrees=True).as_matrix()
        pose[0, 3] = float(index)
        yawed.append(pose)
    result = kitti_odometry_error(yawed, ground_truth)
    assert result["rotational_deg_per_m"] > 0.0
