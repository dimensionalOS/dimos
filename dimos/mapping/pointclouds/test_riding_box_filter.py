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
from typing import Any, cast

import numpy as np
import pytest

from dimos.mapping.pointclouds.riding_box_filter import RidingBox, RidingBoxFilter
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.tf.tf import MultiTBuffer

GRIPPER = RidingBox(name="gripper", frame_id="tool", center=(0.0, 0.0, -0.1), size=(0.1, 0.1, 0.1))


def _make(**config: Any) -> RidingBoxFilter:
    module = RidingBoxFilter(**config)
    # A local transform buffer stands in for the tf stream.
    cast("dict[str, Any]", module.__dict__)["_tf"] = MultiTBuffer()
    return module


@pytest.fixture
def module() -> Iterator[RidingBoxFilter]:
    instance = _make(boxes=[GRIPPER])
    yield instance
    cast("dict[str, Any]", instance.__dict__)["_tf"] = None
    instance.dispose()


@pytest.fixture
def sent(module: RidingBoxFilter) -> list[PointCloud2]:
    clouds: list[PointCloud2] = []
    module.filtered_cloud.subscribe(clouds.append)
    return clouds


def _cloud(points: list[list[float]], ts: float) -> PointCloud2:
    return PointCloud2.from_numpy(
        np.asarray(points, dtype=np.float32), frame_id="world", timestamp=ts
    )


def _feed(module: RidingBoxFilter, cloud: PointCloud2) -> None:
    asyncio.run(module.handle_cloud(cloud))


def _tool_at(module: RidingBoxFilter, x: float, ts: float) -> None:
    module.tfbuffer.receive_transform(
        Transform(translation=Vector3(x, 0.0, 0.5), frame_id="world", child_frame_id="tool", ts=ts)
    )


def test_the_box_is_where_its_frame_was_when_the_cloud_was_captured(
    module: RidingBoxFilter, sent: list[PointCloud2]
) -> None:
    """A cloud arriving after the arm moved on still loses the gripper where it saw it."""
    _tool_at(module, 1.0, ts=1.0)
    _tool_at(module, 2.0, ts=2.0)
    # Captured at 1 s: the gripper hung at x = 1, 10 cm under the tool.
    _feed(module, _cloud([[1.0, 0.0, 0.4], [2.0, 0.0, 0.4]], ts=1.0))

    np.testing.assert_allclose(sent[-1].points_f32(), [[2.0, 0.0, 0.4]])


def test_clouds_wait_for_the_frame_rather_than_pass_the_gripper(
    module: RidingBoxFilter, sent: list[PointCloud2]
) -> None:
    """The gripper passed once would stay in the map as a ghost."""
    _feed(module, _cloud([[1.0, 0.0, 0.4], [2.0, 0.0, 0.0]], ts=1.0))
    assert sent == []

    _tool_at(module, 1.0, ts=1.0)
    _feed(module, _cloud([[1.0, 0.0, 0.4], [2.0, 0.0, 0.0]], ts=1.0))
    np.testing.assert_allclose(sent[-1].points_f32(), [[2.0, 0.0, 0.0]])


def test_a_held_object_rides_and_drops(module: RidingBoxFilter, sent: list[PointCloud2]) -> None:
    _tool_at(module, 1.0, ts=1.0)
    module.ride_box("held", Pose(position=Vector3(0.0, 0.0, -0.25)), Vector3(0.1, 0.1, 0.1), "tool")
    cloud = _cloud([[1.0, 0.0, 0.25], [2.0, 0.0, 0.0]], ts=1.0)

    _feed(module, cloud)
    np.testing.assert_allclose(sent[-1].points_f32(), [[2.0, 0.0, 0.0]])

    assert module.drop_box("held")
    assert not module.drop_box("held")
    _feed(module, cloud)
    assert len(sent[-1].points_f32()) == 2


def test_with_no_boxes_the_cloud_passes() -> None:
    module = _make()
    sent: list[PointCloud2] = []
    module.filtered_cloud.subscribe(sent.append)
    try:
        cloud = _cloud([[1.0, 0.0, 0.0]], ts=1.0)
        _feed(module, cloud)
        assert sent == [cloud]
    finally:
        cast("dict[str, Any]", module.__dict__)["_tf"] = None
        module.dispose()
