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

from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz
from dimos.msgs.time import time_from_seconds
from dimos.navigation.go2.loop_closure.pgo_map import PGOMap

# TODO(PY311): drop. Skip before importing test_pgo, which skips mid-import without gtsam.
pytest.importorskip("gtsam")

from dimos.navigation.go2.loop_closure.test_pgo import _graph_with_drift_at

VOXEL = 0.05


class _FakePGO:
    """Stands in for the optimizer: the test decides when a loop closes."""

    n_keyframes = 0
    n_loops = 0
    graph = None

    def process(self, *args):
        pass

    def snapshot(self):
        return self.graph


def _wall(x, ts):
    ys, zs = np.meshgrid(np.arange(0.025, 1, VOXEL), np.arange(0.025, 1, VOXEL))
    pts = np.stack([np.full(ys.size, x), ys.ravel(), zs.ravel()], axis=1)
    return pointcloud_from_xyz(pts, header=Header(stamp=time_from_seconds(ts), frame_id=""))


def _xs(world_map):
    return {int(x) for x in np.floor(pointcloud_xyz(world_map.global_map())[:, 0] / VOXEL)}


def test_rebuilds_on_loop_closure_and_respects_cooldown():
    world_map = PGOMap(rebuild_cooldown_s=10.0)
    world_map._pgo = pgo = _FakePGO()
    pose = PoseStamped(
        header=Header(stamp=time_from_seconds(0.0), frame_id=""),
        pose=Pose(
            position=Point(x=1.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        ),
    )

    # the same wall seen twice, the first time 0.5 m off; no loop yet
    assert not world_map.add(_wall(1.025, 100.0), pose.pose)
    assert not world_map.add(_wall(1.525, 101.0), pose.pose)
    assert _xs(world_map) == {20, 30}

    # loop closes: the frame at t=100 was really 0.5 m further along x
    pgo.n_loops = 1
    pgo.graph = _graph_with_drift_at(
        [
            TransformStamped(
                header=Header(stamp=time_from_seconds(100.0), frame_id=""),
                child_frame_id="",
                transform=Transform(
                    translation=Vector3(0.5, 0.0, 0.0),
                    rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            ),
            TransformStamped(
                header=Header(stamp=time_from_seconds(101.0), frame_id=""),
                child_frame_id="",
                transform=Transform(
                    translation=Vector3(0.0, 0.0, 0.0),
                    rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            ),
        ]
    )
    assert world_map.add(_wall(1.525, 102.0), pose.pose)
    assert _xs(world_map) == {30}

    # a second loop inside the cooldown waits for it
    pgo.n_loops = 2
    assert not world_map.add(_wall(1.525, 105.0), pose.pose)
    assert world_map.add(_wall(1.525, 112.5), pose.pose)
