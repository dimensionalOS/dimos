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

from collections.abc import Callable, Iterator
from pathlib import Path
from typing import Any, cast

import numpy as np
import pytest

from dimos.manipulation.planning.utils.point_cloud_self_filter import PointCloudSelfFilter
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.robot.assets.model import RobotModel

# A 20cm cube on `arm`, so the filter has real volume to exclude and the clear
# mask has real cells to name, with no external assets.
_URDF = """<?xml version="1.0"?>
<robot name="box_robot">
  <link name="base"/>
  <link name="arm">
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <joint name="shoulder" type="fixed">
    <parent link="base"/><child link="arm"/>
  </joint>
</robot>
"""


@pytest.fixture
def make_filter(tmp_path: Path) -> Iterator[Callable[..., PointCloudSelfFilter]]:
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(_URDF)
    modules: list[PointCloudSelfFilter] = []

    def make(**overrides: Any) -> PointCloudSelfFilter:
        settings: dict[str, Any] = {
            "model": RobotModel.from_file(urdf),
            "padding_m": 0.01,
            "voxel_size": 0.05,
            "tf_tolerance_s": 0.001,
            "tf_forward_tolerance_s": 0.0,
        }
        settings.update(overrides)
        module = PointCloudSelfFilter(**settings)
        cast("dict[str, Any]", module.__dict__)["_tf"] = MultiTBuffer()
        modules.append(module)
        return module

    yield make
    for module in modules:
        cast("dict[str, Any]", module.__dict__)["_tf"] = None
        module.dispose()


def _place_arm(module: PointCloudSelfFilter, at: tuple[float, float, float], ts: float) -> None:
    """Put the arm at `at` in both the camera and world frames."""
    for parent in ("camera", "world"):
        module.tfbuffer.receive_transform(
            Transform(
                translation=Vector3(*at),
                frame_id=parent,
                child_frame_id="arm",
                ts=ts,
            )
        )


def _cloud(points: list[list[float]], ts: float = 1.0) -> PointCloud2:
    return PointCloud2.from_numpy(
        np.asarray(points, dtype=np.float32).reshape((-1, 3)),
        frame_id="camera",
        timestamp=ts,
    )


def _keys(cloud: PointCloud2, voxel_size: float) -> set[tuple[int, int, int]]:
    points = cloud.points_f32()
    if not len(points):
        return set()
    return {tuple(k) for k in np.floor(points / voxel_size).astype(int).tolist()}


def test_points_on_the_robot_are_dropped_and_the_rest_survive(
    make_filter: Callable[..., PointCloudSelfFilter],
) -> None:
    module = make_filter()
    _place_arm(module, (0.0, 0.0, 0.0), 1.0)

    result = module.filter_cloud(_cloud([[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [2.0, 0.0, 0.0]]))

    assert result is not None
    filtered, _ = result
    np.testing.assert_allclose(filtered.points_f32(), [[2.0, 0.0, 0.0]], atol=1e-6)


def test_the_mask_also_covers_where_the_robot_just_was(
    make_filter: Callable[..., PointCloudSelfFilter],
) -> None:
    # This is the whole point of the mask. Ray tracing cannot clear the volume
    # a link has vacated, because the link occluded it while it was there, so
    # the ghost stays until the filter names those cells itself.
    module = make_filter()

    _place_arm(module, (1.0, 0.0, 0.0), 1.0)
    first = module.filter_cloud(_cloud([], ts=1.0))
    assert first is not None

    _place_arm(module, (2.0, 0.0, 0.0), 2.0)
    second = module.filter_cloud(_cloud([], ts=2.0))
    assert second is not None

    keys = _keys(second[1], 0.05)
    assert (20, 0, 0) in keys, "the cell the arm vacated must still be cleared"
    assert (40, 0, 0) in keys, "and the cell it moved into"

    _place_arm(module, (2.0, 0.0, 0.0), 3.0)
    third = module.filter_cloud(_cloud([], ts=3.0))
    assert third is not None
    assert (20, 0, 0) not in _keys(third[1], 0.05), "a stale cell is cleared once, not forever"


def test_the_mask_quantizes_the_way_the_mapper_does(
    make_filter: Callable[..., PointCloudSelfFilter],
) -> None:
    # The crate floors world coordinates by voxel_size and emits cell centers.
    # A mask built any other way names cells the map does not hold.
    module = make_filter()
    _place_arm(module, (1.0, 0.0, 0.0), 1.0)

    result = module.filter_cloud(_cloud([]))

    assert result is not None
    points = result[1].points_f32()
    offsets = points / 0.05 - np.floor(points / 0.05)
    np.testing.assert_allclose(offsets, 0.5, atol=1e-5)


def test_a_cloud_without_capture_time_tf_is_dropped_whole(
    make_filter: Callable[..., PointCloudSelfFilter],
) -> None:
    # Half-filtering would let the arm into the map. Better to lose the frame.
    module = make_filter()

    assert module.filter_cloud(_cloud([[2.0, 0.0, 0.0]])) is None
