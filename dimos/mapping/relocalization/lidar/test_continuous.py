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

from dimos.mapping.relocalization.lidar.clouds import LiveMap
from dimos.mapping.relocalization.lidar.relocalize import MID360, LidarRelocalizer
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

PREMAP = 1


def local_map(points, flags=None):
    points = np.asarray(points, dtype=np.float32)
    flags = (
        np.zeros(len(points), dtype=np.uint32)
        if flags is None
        else np.asarray(flags, dtype=np.uint32)
    )
    return PointCloud2.from_numpy(points, frame_id="world", flags=flags)


def held(live):
    return sorted(map(tuple, live.cloud("world", None, 0).points_f32().round(2).tolist()))


def test_live_map_keeps_only_live_voxels():
    live = LiveMap(0.1)
    live.fold(local_map([[1, 0, 0], [2, 0, 0], [3, 0, 0]], [0, PREMAP, 0]))
    assert held(live) == [(1, 0, 0), (3, 0, 0)]


def test_a_local_map_without_the_premap_bit_is_an_error():
    cloud = PointCloud2.from_numpy(np.asarray([[1, 0, 0]], dtype=np.float32), frame_id="world")
    with pytest.raises(ValueError, match="premap"):
        LiveMap(0.1).fold(cloud)


def test_one_point_per_voxel():
    live = LiveMap(1.0)
    live.fold(local_map([[1.2, 0.2, 0.2], [1.4, 0.4, 0.4]]))
    assert len(live) == 1


def test_the_match_cloud_keeps_near_the_robot():
    live = LiveMap(0.1)
    live.fold(local_map([[1, 0, 0], [50, 0, 0]]))
    near = live.cloud("world", (0.0, 0.0), 10.0).points_f32().tolist()
    assert near == [[1, 0, 0]]


def _room(rng):
    """Points on the walls, floor and a few boxes of a 12 x 8 m room."""
    faces = []
    for _ in range(6000):
        u, v = rng.uniform(0, 1, 2)
        faces.append(
            [
                [u * 12, v * 8, 0],
                [0, u * 8, v * 3],
                [12, u * 8, v * 3],
                [u * 12, 0, v * 3],
                [u * 12, 8, v * 3],
                [2 + u, 2 + v, 1],
                [7 + 2 * u, 5, v * 2],
            ][rng.integers(7)]
        )
    return np.asarray(faces, dtype=np.float32)


def test_a_near_fix_is_refined_by_icp_alone():
    rng = np.random.default_rng(0)
    room = _room(rng)
    relocalizer = LidarRelocalizer(PointCloud2.from_numpy(room).pointcloud, MID360)
    # The live world sits 0.3 m and 3 deg off the map; the current fix is off by that much.
    truth = Transform(
        translation=Vector3(0.3, -0.2, 0.0),
        rotation=Quaternion.from_euler(Vector3(0.0, 0.0, np.radians(3.0))),
        frame_id="world",
        child_frame_id="map",
    )
    world_room = (truth.to_matrix() @ np.c_[room, np.ones(len(room))].T).T[:, :3]
    live = PointCloud2.from_numpy(world_room.astype(np.float32)).pointcloud
    near = Transform(frame_id="world", child_frame_id="map")
    fix = relocalizer.attempt(live, "world", "map", near).fix
    assert fix is not None
    assert (fix.translation - truth.translation).length() < 0.05
