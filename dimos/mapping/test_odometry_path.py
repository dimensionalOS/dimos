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

from __future__ import annotations

import asyncio
from unittest.mock import MagicMock

import pytest

from dimos.mapping.odometry_path import OdometryPath
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry


def odometry(ts: float, x: float, y: float = 0.0, frame_id: str = "odom") -> Odometry:
    return Odometry(
        ts=ts,
        frame_id=frame_id,
        child_frame_id="base_link",
        pose=Pose(position=Vector3(x, y, 0.0), orientation=[0.0, 0.0, 0.0, 1.0]),
    )


@pytest.fixture()
def module():
    path_module = OdometryPath()
    path_module.path = MagicMock()
    try:
        yield path_module
    finally:
        path_module.stop()


def feed(module: OdometryPath, *odometries: Odometry) -> None:
    for msg in odometries:
        asyncio.run(module.handle_odometry(msg))


def published_paths(module: OdometryPath) -> list:
    return [call.args[0] for call in module.path.publish.call_args_list]


def test_a_pose_closer_than_min_step_is_not_added_to_the_path(module):
    module.config.min_step_meters = 0.5
    module.config.min_publish_interval_seconds = 0.0

    feed(module, odometry(0.0, 0.0), odometry(1.0, 0.1), odometry(2.0, 0.6))

    assert [len(path.poses) for path in published_paths(module)] == [1, 2]


def test_publishing_is_throttled_to_min_publish_interval(module):
    module.config.min_step_meters = 0.0
    module.config.min_publish_interval_seconds = 1.0

    feed(module, *(odometry(step * 0.25, step) for step in range(5)))

    assert [path.ts for path in published_paths(module)] == [0.0, 1.0]


def test_a_backwards_stamp_publishes_instead_of_waiting_out_the_interval(module):
    module.config.min_step_meters = 0.0
    module.config.min_publish_interval_seconds = 10.0

    feed(module, odometry(100.0, 0.0), odometry(0.0, 1.0))

    assert [path.ts for path in published_paths(module)] == [100.0, 0.0]


def test_the_path_is_capped_at_max_poses(module):
    module.config.min_step_meters = 0.0
    module.config.min_publish_interval_seconds = 0.0
    module._poses = type(module._poses)(maxlen=3)

    feed(module, *(odometry(float(step), step) for step in range(6)))

    last = published_paths(module)[-1]
    assert [pose.position.x for pose in last.poses] == [3.0, 4.0, 5.0]


def test_an_empty_frame_id_follows_the_odometry(module):
    module.config.min_publish_interval_seconds = 0.0

    feed(module, odometry(0.0, 0.0, frame_id="wheel_odom"))
    assert published_paths(module)[-1].frame_id == "wheel_odom"

    module.config.frame_id = "map"
    feed(module, odometry(1.0, 5.0))
    assert published_paths(module)[-1].frame_id == "map"
