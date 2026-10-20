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

from dimos_cdrgen.registry import encode
import numpy as np
import pytest

from dimos.messages.geometry_msgs.msg import (
    Point,
    Pose,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz
from dimos.msgs.time import time_from_nanoseconds, time_from_seconds
from dimos.navigation.go2.loop_closure.pgo_map import PGOMap

# TODO(PY311): drop. Skip before importing test_pgo, which skips mid-import without gtsam.
pytest.importorskip("gtsam")

from dimos.navigation.go2.loop_closure.test_pgo import _graph_with_drift_at

VOXEL = 0.05


def _wall(x, ts):
    ys, zs = np.meshgrid(np.arange(0.025, 1, VOXEL), np.arange(0.025, 1, VOXEL))
    pts = np.stack([np.full(ys.size, x), ys.ravel(), zs.ravel()], axis=1)
    return pointcloud_from_xyz(pts, header=Header(stamp=time_from_seconds(ts), frame_id="world"))


def _xs(world_map):
    return {int(x) for x in np.floor(pointcloud_xyz(world_map.global_map())[:, 0] / VOXEL)}


@pytest.fixture
def controlled_map(mocker):
    pgo = mocker.patch(
        "dimos.navigation.go2.loop_closure.pgo_map._PGOState", autospec=True
    ).return_value
    pgo.n_keyframes = 0
    pgo.n_loops = 0
    world_map = PGOMap(rebuild_cooldown_s=10.0)
    try:
        yield world_map, pgo
    finally:
        world_map.dispose()


def test_rebuilds_on_loop_closure_and_respects_cooldown(controlled_map):
    world_map, pgo = controlled_map
    pose = Pose(position=Point(1, 0, 0), orientation=Quaternion(0, 0, 0, 1))

    # the same wall seen twice, the first time 0.5 m off; no loop yet
    assert not world_map.add(_wall(1.025, 100.0), pose)
    assert not world_map.add(_wall(1.525, 101.0), pose)
    assert _xs(world_map) == {20, 30}

    # loop closes: the frame at t=100 was really 0.5 m further along x
    pgo.n_loops = 1
    pgo.snapshot.return_value = _graph_with_drift_at(
        [
            TransformStamped(
                header=Header(stamp=time_from_seconds(100), frame_id="world"),
                child_frame_id="body",
                transform=Transform(
                    translation=Vector3(0.5, 0, 0), rotation=Quaternion(0, 0, 0, 1)
                ),
            ),
            TransformStamped(
                header=Header(stamp=time_from_seconds(101), frame_id="world"),
                child_frame_id="body",
                transform=Transform(translation=Vector3(0, 0, 0), rotation=Quaternion(0, 0, 0, 1)),
            ),
        ]
    )
    assert world_map.add(_wall(1.525, 102.0), pose)
    assert _xs(world_map) == {30}

    # a second loop inside the cooldown waits for it
    pgo.n_loops = 2
    assert not world_map.add(_wall(1.525, 105.0), pose)
    assert world_map.add(_wall(1.525, 112.5), pose)


@pytest.fixture
def actual_map():
    value = PGOMap()
    try:
        yield value
    finally:
        value.dispose()


def test_real_optimizer_map_preserves_sensor_stamp_and_world_frame(actual_map):
    cloud = _wall(1.025, 100)
    cloud.header.stamp = time_from_nanoseconds(1700000000123456789)
    before = encode(cloud)
    pose = Pose(position=Point(1, 0, 0), orientation=Quaternion(0, 0, 0, 1))
    assert not actual_map.add(cloud, pose)
    assert actual_map.n_keyframes == 1 and actual_map.n_loops == 0
    output = actual_map.global_map()
    assert output.header.stamp == cloud.header.stamp and output.header.frame_id == "world"
    assert _xs(actual_map) == {20}
    assert encode(cloud) == before
