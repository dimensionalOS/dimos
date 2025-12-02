# Copyright 2025 Dimensional Inc.
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

from operator import add, sub

import pytest
import reactivex.operators as ops

from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.utils.testing import SensorReplay

_EXPECTED_TOTAL_RAD = -4.05212


def test_dataset_size() -> None:
    """Ensure the replay contains the expected number of messages."""
    assert sum(1 for _ in SensorReplay(name="raw_odometry_rotate_walk").iterate()) == 179


def test_odometry_conversion_and_count() -> None:
    """Each replay entry converts to :class:`Odometry` and count is correct."""
    for raw in SensorReplay(name="raw_odometry_rotate_walk").iterate():
        odom = Odometry.from_msg(raw)
        assert isinstance(raw, dict)
        assert isinstance(odom, Odometry)


def test_last_yaw_value() -> None:
    """Verify yaw of the final message (regression guard)."""
    last_msg = SensorReplay(name="raw_odometry_rotate_walk").stream().pipe(ops.last()).run()

    assert last_msg is not None, "Replay is empty"
    assert last_msg["data"]["pose"]["orientation"] == {
        "x": 0.01077,
        "y": 0.008505,
        "z": 0.499171,
        "w": -0.866395,
    }


def test_total_rotation_travel_iterate() -> None:
    total_rad = 0.0
    prev_yaw: float | None = None

    for odom in SensorReplay(name="raw_odometry_rotate_walk", autocast=Odometry.from_msg).iterate():
        yaw = odom.orientation.radians.z
        if prev_yaw is not None:
            diff = yaw - prev_yaw
            total_rad += diff
        prev_yaw = yaw

    assert total_rad == pytest.approx(_EXPECTED_TOTAL_RAD, abs=0.001)


def test_total_rotation_travel_rxpy() -> None:
    total_rad = (
        SensorReplay(name="raw_odometry_rotate_walk", autocast=Odometry.from_msg)
        .stream()
        .pipe(
            ops.map(lambda odom: odom.orientation.radians.z),
            ops.pairwise(),  # [1,2,3,4] -> [[1,2], [2,3], [3,4]]
            ops.starmap(sub),  # [sub(1,2), sub(2,3), sub(3,4)]
            ops.reduce(add),
        )
        .run()
    )

    assert total_rad == pytest.approx(4.05, abs=0.01)
