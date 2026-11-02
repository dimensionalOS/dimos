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


import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

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
    return PointCloud2.from_numpy(points.astype(np.float32), timestamp=ts)


def _near(world_map: PGOMap, at: np.ndarray, radius: float = 0.03) -> int:
    pts = world_map.global_map().points_f32()
    return int((np.linalg.norm(pts - at, axis=1) < radius).sum())


def test_the_map_is_what_the_keyframes_saw_while_nothing_moves() -> None:
    world_map = PGOMap()
    floor = _floor(0.0, 6.0)
    for i in range(5):
        world_map.add(_frame(floor, float(i)), Pose(1.0 + i, 0.0, 0.0))
    assert world_map.pgo.n_keyframes == 5
    got = world_map.global_map().points_f32()
    assert len(got) == len(floor)
    # keyframes keep voxel centres, so within a voxel of what was seen
    np.testing.assert_allclose(np.sort(got[:, 0]), np.sort(floor[:, 0]), atol=VOXEL)
    world_map.dispose()


def test_a_younger_keyframe_takes_the_columns_it_sees() -> None:
    world_map = PGOMap()
    box_top = _box(3.0)[-1]
    # the first keyframe sees a box standing on the floor
    world_map.add(_frame(np.concatenate([_floor(0.0, 6.0), _box(3.0)]), 0.0), Pose(1.0, 0, 0))
    world_map.add(_frame(_floor(0.0, 6.0), 0.5), Pose(1.2, 0, 0))  # same keyframe, no box
    assert _near(world_map, box_top) == 0, "within a keyframe the later frame wins"
    world_map.dispose()

    world_map = PGOMap()
    world_map.add(_frame(np.concatenate([_floor(0.0, 6.0), _box(3.0)]), 0.0), Pose(1.0, 0, 0))
    world_map.add(_frame(_floor(4.0, 6.0), 1.0), Pose(2.0, 0, 0))  # a new keyframe, elsewhere
    assert _near(world_map, box_top) == 1, "a keyframe keeps what no younger one has seen"
    world_map.add(_frame(_floor(2.0, 4.0), 2.0), Pose(3.0, 0, 0))  # the box is gone
    assert world_map.pgo.n_keyframes == 3
    assert _near(world_map, box_top) == 0, "the youngest keyframe owns the column"
    world_map.dispose()
