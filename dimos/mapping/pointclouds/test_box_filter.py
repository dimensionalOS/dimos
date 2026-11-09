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
from collections.abc import Iterator
import math
from typing import Any, cast

import numpy as np
import pytest

from dimos.mapping.pointclouds.box_filter import PointCloudBoxFilter
from dimos.message_utils import geometry, pointcloud, time as message_time
from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos.messages.sensor_msgs.msg import PointCloud2
from dimos.messages.std_msgs.msg import Header
from dimos.protocol.tf.tf import MultiTBuffer


@pytest.fixture
def module() -> Iterator[PointCloudBoxFilter]:
    instance = PointCloudBoxFilter()
    # A local transform buffer stands in for the tf stream.
    cast("dict[str, Any]", instance.__dict__)["_tf"] = MultiTBuffer()
    yield instance
    cast("dict[str, Any]", instance.__dict__)["_tf"] = None
    instance.dispose()


@pytest.fixture
def sent(module: PointCloudBoxFilter) -> list[PointCloud2]:
    clouds: list[PointCloud2] = []
    module.filtered_cloud.subscribe(clouds.append)
    return clouds


def _cloud(points: list[list[float]]) -> PointCloud2:
    return pointcloud.from_xyz(
        np.asarray(points, dtype=np.float32),
        header=Header(stamp=message_time.time_from_seconds(1.0), frame_id="world"),
    )


def _feed(module: PointCloudBoxFilter, cloud: PointCloud2) -> None:
    asyncio.run(module.handle_cloud(cloud))


def _place_tool(module: PointCloudBoxFilter, at: Vector3, yaw: float, ts: float) -> None:
    module.tfbuffer.receive_transform(
        TransformStamped(
            header=Header(stamp=message_time.time_from_seconds(ts), frame_id="world"),
            child_frame_id="tool",
            transform=Transform(
                translation=at, rotation=geometry.quaternion_from_euler(0.0, 0.0, yaw)
            ),
        )
    )


def test_a_fixed_box_drops_the_points_inside(
    module: PointCloudBoxFilter, sent: list[PointCloud2]
) -> None:
    module.set_box(
        "target",
        Pose(position=Point(1.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
        Vector3(0.2, 0.2, 0.2),
    )

    _feed(module, _cloud([[1.0, 0.0, 0.0], [1.05, -0.05, 0.05], [2.0, 0.0, 0.0]]))

    np.testing.assert_allclose(pointcloud.xyz(sent[-1]).astype(np.float32), [[2.0, 0.0, 0.0]])


def test_a_box_on_a_moving_frame_follows_it(
    module: PointCloudBoxFilter, sent: list[PointCloud2]
) -> None:
    # 20 cm along the tool's x axis; the tool is turned 90 degrees, so in the
    # world the box sits 20 cm along y from the tool.
    module.set_box(
        "held",
        Pose(position=Point(0.2, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
        Vector3(0.1, 0.1, 0.1),
        "tool",
    )
    cloud = _cloud([[1.0, 0.2, 0.0], [1.2, 0.0, 0.0], [2.0, 0.2, 0.0]])

    _place_tool(module, Vector3(1.0, 0.0, 0.0), math.pi / 2, ts=1.0)
    _feed(module, cloud)
    np.testing.assert_allclose(
        pointcloud.xyz(sent[-1]).astype(np.float32), [[1.2, 0.0, 0.0], [2.0, 0.2, 0.0]]
    )

    _place_tool(module, Vector3(2.0, 0.0, 0.0), math.pi / 2, ts=2.0)
    _feed(module, cloud)
    np.testing.assert_allclose(
        pointcloud.xyz(sent[-1]).astype(np.float32), [[1.0, 0.2, 0.0], [1.2, 0.0, 0.0]]
    )
