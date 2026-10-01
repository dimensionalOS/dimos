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

from __future__ import annotations

from dimos_generated.geometry_msgs.msg import PoseStamped
import pytest

from dimos.msgs.geometry import yaw
from dimos.robot.unitree.type.odometry import pose_from_webrtc_odometry
from dimos.utils.testing.replay import SensorReplay

_EXPECTED_TOTAL_RAD = -4.05212


def test_dataset_size() -> None:
    """Ensure the replay contains the expected number of messages."""
    assert sum(1 for _ in SensorReplay(name="raw_odometry_rotate_walk").iterate()) == 179


def test_odometry_conversion_and_count() -> None:
    """Each replay entry converts to :class:`PoseStamped` and count is correct."""
    for raw in SensorReplay(name="raw_odometry_rotate_walk").iterate():
        odom = pose_from_webrtc_odometry(raw)
        assert isinstance(raw, dict)
        assert isinstance(odom, PoseStamped)


def test_total_rotation_travel_iterate() -> None:
    total_rad = 0.0
    prev_yaw: float | None = None

    for odom in SensorReplay(
        name="raw_odometry_rotate_walk", autocast=pose_from_webrtc_odometry
    ).iterate():
        angle = yaw(odom.pose.orientation)
        if prev_yaw is not None:
            diff = angle - prev_yaw
            total_rad += diff
        prev_yaw = angle

    assert total_rad == pytest.approx(_EXPECTED_TOTAL_RAD, abs=0.001)
