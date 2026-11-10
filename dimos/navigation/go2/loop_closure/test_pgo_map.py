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

from dataclasses import asdict

import numpy as np
import pytest

from dimos.message_utils import pointcloud, time as message_time
from dimos_messages.geometry_msgs.msg import Point, Pose, Quaternion
from dimos_messages.sensor_msgs.msg import PointCloud2
from dimos_messages.std_msgs.msg import Header

pytest.importorskip("gtsam")

from dimos.navigation.go2.loop_closure.pgo_map import PGOMap

VOXEL = 0.05


def _floor(x0: float, x1: float) -> np.ndarray:
    xs, ys = np.meshgrid(np.arange(x0, x1, VOXEL), np.arange(-1.0, 1.0, VOXEL))
    return np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], axis=1) + 0.0123


def _box(x: float) -> np.ndarray:
    zs = np.arange(0.05, 1.0, VOXEL)
    return np.stack([np.full(zs.size, x), np.zeros(zs.size), zs], axis=1) + 0.0123


def _frame(points: np.ndarray, ts: float) -> PointCloud2:
    return pointcloud.from_xyz(
        points.astype(np.float32),
        header=Header(stamp=message_time.time_from_seconds(ts), frame_id="world"),
    )


def _pose(x: float) -> Pose:
    return Pose(position=Point(x, 0, 0), orientation=Quaternion(0, 0, 0, 1))


def _near(world_map: PGOMap, at: np.ndarray, radius: float = 0.03) -> int:
    pts = pointcloud.xyz(world_map.global_map())
    return int((np.linalg.norm(pts - at, axis=1) < radius).sum())


def _wall(x: float, ts: float) -> PointCloud2:
    ys, zs = np.meshgrid(np.arange(0.025, 1, VOXEL), np.arange(0.025, 1, VOXEL))
    pts = np.stack([np.full(ys.size, x), ys.ravel(), zs.ravel()], axis=1)
    return _frame(pts, ts)


def _xs(world_map: PGOMap) -> set[int]:
    return {int(x) for x in np.floor(pointcloud.xyz(world_map.global_map())[:, 0] / VOXEL)}


def test_the_map_is_what_the_keyframes_saw_while_nothing_moves() -> None:
    world_map = PGOMap()
    floor = _floor(0.0, 6.0)
    for i in range(5):
        world_map.add(_frame(floor, float(i)), _pose(1.0 + i))
    assert world_map.pgo.n_keyframes == 5
    got = pointcloud.xyz(world_map.global_map())
    assert len(got) == len(floor)
    # keyframes keep voxel centres, so within a voxel of what was seen
    np.testing.assert_allclose(np.sort(got[:, 0]), np.sort(floor[:, 0]), atol=VOXEL)
    world_map.dispose()


def test_a_younger_keyframe_takes_the_columns_it_sees() -> None:
    world_map = PGOMap()
    box_top = _box(3.0)[-1]
    # the first keyframe sees a box standing on the floor
    world_map.add(_frame(np.concatenate([_floor(0.0, 6.0), _box(3.0)]), 0.0), _pose(1.0))
    world_map.add(_frame(_floor(0.0, 6.0), 0.5), _pose(1.2))  # same keyframe, no box
    assert _near(world_map, box_top) == 0, "within a keyframe the later frame wins"
    world_map.dispose()

    world_map = PGOMap()
    world_map.add(_frame(np.concatenate([_floor(0.0, 6.0), _box(3.0)]), 0.0), _pose(1.0))
    world_map.add(_frame(_floor(4.0, 6.0), 1.0), _pose(2.0))  # a new keyframe, elsewhere
    assert _near(world_map, box_top) == 1, "a keyframe keeps what no younger one has seen"
    world_map.add(_frame(_floor(2.0, 4.0), 2.0), _pose(3.0))  # the box is gone
    assert world_map.pgo.n_keyframes == 3
    assert _near(world_map, box_top) == 0, "the youngest keyframe owns the column"
    world_map.dispose()


def test_real_optimizer_map_preserves_sensor_stamp_and_world_frame() -> None:
    actual_map = PGOMap()
    try:
        cloud = _wall(1.025, 100)
        cloud.header.stamp = message_time.time_from_nanoseconds(1700000000123456789)
        before = asdict(cloud)
        assert not actual_map.add(cloud, _pose(1.0))
        assert actual_map.pgo.n_keyframes == 1 and actual_map.pgo.n_loops == 0
        output = actual_map.global_map()
        assert output.header.stamp == cloud.header.stamp and output.header.frame_id == "world"
        assert _xs(actual_map) == {20}
        np.testing.assert_equal(asdict(cloud), before)
    finally:
        actual_map.dispose()
