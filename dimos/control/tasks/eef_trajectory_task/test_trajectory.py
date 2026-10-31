# Copyright 2025-2026 Dimensional Inc.
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

"""EEF waypoint contract, interpolation and process-boundary behavior."""

import math
import pickle

import numpy as np
import pytest

from dimos.control.tasks.eef_trajectory_task.trajectory import EefTrajectory, EefTrajectoryPoint
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


def test_eef_translation_rotation_and_grippers_share_one_interpolation_clock():
    trajectory = EefTrajectory(
        [
            EefTrajectoryPoint(
                0.0,
                {
                    "left": PoseStamped(frame_id="world", position=[0, 0, 0]),
                    "right": PoseStamped(frame_id="world", position=[0, 0, 0]),
                },
                {"left/gripper": 0.0},
            ),
            EefTrajectoryPoint(
                2.0,
                {
                    "left": PoseStamped(
                        frame_id="world", position=[2, 0, 0], orientation=[0, 0, 1, 0]
                    ),
                    "right": PoseStamped(frame_id="world", position=[0, 4, 0]),
                },
                {"left/gripper": 1.0},
            ),
        ]
    )
    restored = pickle.loads(pickle.dumps(trajectory))
    sample = restored.sample(1.0)
    assert sample.poses["left"].x == 1.0
    assert sample.poses["right"].y == 2.0
    assert sample.grippers == {"left/gripper": 0.5}
    q = sample.poses["left"].orientation
    assert [q.x, q.y, q.z, q.w] == pytest.approx([0, 0, math.sqrt(0.5), math.sqrt(0.5)])
    assert restored.sample(10).poses["left"].x == 2.0


@pytest.mark.parametrize(
    "points",
    [
        [],
        [EefTrajectoryPoint(0, {"tool": PoseStamped(frame_id="world")})],
        [
            EefTrajectoryPoint(1, {"tool": PoseStamped(frame_id="world")}),
            EefTrajectoryPoint(2, {"tool": PoseStamped(frame_id="world")}),
        ],
        [
            EefTrajectoryPoint(0, {"tool": PoseStamped(frame_id="world")}),
            EefTrajectoryPoint(0, {"tool": PoseStamped(frame_id="world")}),
        ],
        [
            EefTrajectoryPoint(0, {"tool": PoseStamped(frame_id="world")}),
            EefTrajectoryPoint(np.inf, {"tool": PoseStamped(frame_id="world")}),
        ],
        [
            EefTrajectoryPoint(0, {"tool": PoseStamped(frame_id="world")}),
            EefTrajectoryPoint(1, {"other": PoseStamped(frame_id="world")}),
        ],
        [
            EefTrajectoryPoint(0, {"tool": PoseStamped(frame_id="base")}),
            EefTrajectoryPoint(1, {"tool": PoseStamped(frame_id="world")}),
        ],
        [
            EefTrajectoryPoint(0, {"tool": PoseStamped(frame_id="world")}, {"g": 0}),
            EefTrajectoryPoint(1, {"tool": PoseStamped(frame_id="world")}, {"g": 1.1}),
        ],
    ],
)
def test_invalid_eef_waypoints_are_rejected_before_execution(points):
    with pytest.raises(ValueError):
        EefTrajectory(points)
