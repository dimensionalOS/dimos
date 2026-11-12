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

"""The grasp blueprint's pick check on the dual OpenYAM workcell model."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.manipulation.planning.gripper_contact import GraspContactChecker
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import (
    DUAL_OPENYAM_TABLE_TOP_Z,
    dual_openyam_grasp_model_config,
)

# A can 5 cm wide and 8 cm tall in the right arm's reach, world frame, metres.
X, Y, WIDTH, HEIGHT = 0.20, -0.09, 0.05, 0.08


def _top_down(z: float) -> Pose:
    q = Rotation.from_euler("xyz", [np.pi, 0.0, 0.0]).as_quat()
    return Pose(position=Vector3(x=X, y=Y, z=z), orientation=Quaternion(q[0], q[1], q[2], q[3]))


def _can() -> NDArray:
    """Points 4 mm apart on the can's box faces."""
    half = np.array([WIDTH, WIDTH, HEIGHT]) / 2.0
    faces = []
    for axis in range(3):
        u, v = (a for a in range(3) if a != axis)
        mesh_u, mesh_v = np.meshgrid(
            np.arange(-half[u], half[u] + 1e-9, 0.004), np.arange(-half[v], half[v] + 1e-9, 0.004)
        )
        for side in (-1.0, 1.0):
            face = np.zeros((mesh_u.size, 3))
            face[:, u], face[:, v], face[:, axis] = (
                mesh_u.ravel(),
                mesh_v.ravel(),
                side * half[axis],
            )
            faces.append(face)
    return np.concatenate(faces) + np.array([X, Y, DUAL_OPENYAM_TABLE_TOP_Z + HEIGHT / 2])


def _table() -> NDArray:
    """Table points 1 cm apart, minus those under the can."""
    x, y = np.meshgrid(np.arange(-0.1, 0.5, 0.01), np.arange(-0.4, 0.4, 0.01))
    points = np.stack([x.ravel(), y.ravel(), np.full(x.size, DUAL_OPENYAM_TABLE_TOP_Z)], axis=1)
    under = (np.abs(points[:, 0] - X) < 0.03) & (np.abs(points[:, 1] - Y) < 0.03)
    return points[~under]


def test_pick_check_keeps_a_top_down_grasp_and_drops_one_into_the_table() -> None:
    checker = GraspContactChecker(dual_openyam_grasp_model_config(), "right_manipulator")
    grasps = [
        _top_down(DUAL_OPENYAM_TABLE_TOP_Z + HEIGHT / 2),
        _top_down(DUAL_OPENYAM_TABLE_TOP_Z - 0.005),
    ]
    kept = checker.check(grasps, _can(), _table(), "world", 0.005, 0.003)
    assert list(kept) == [True, False]
