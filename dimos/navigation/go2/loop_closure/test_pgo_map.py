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

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.go2.loop_closure.pgo_map import PGOMap
from dimos.navigation.go2.loop_closure.test_pgo import _graph_with_drift_at

pytest.importorskip("gtsam")

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
    return PointCloud2.from_numpy(pts, timestamp=ts)


def _xs(world_map):
    return {int(x) for x in np.floor(world_map.global_map().points_f32()[:, 0] / VOXEL)}


def test_rebuilds_on_loop_closure_and_respects_cooldown():
    world_map = PGOMap(rebuild_cooldown_s=10.0)
    world_map._pgo = pgo = _FakePGO()
    pose = PoseStamped(1.0, 0.0, 0.0)

    # the same wall seen twice, the first time 0.5 m off; no loop yet
    assert not world_map.add(_wall(1.025, 100.0), pose)
    assert not world_map.add(_wall(1.525, 101.0), pose)
    assert _xs(world_map) == {20, 30}

    # loop closes: the frame at t=100 was really 0.5 m further along x
    pgo.n_loops = 1
    pgo.graph = _graph_with_drift_at(
        [
            Transform(translation=Vector3(0.5, 0.0, 0.0), ts=100.0),
            Transform(translation=Vector3(0.0, 0.0, 0.0), ts=101.0),
        ]
    )
    assert world_map.add(_wall(1.525, 102.0), pose)
    assert _xs(world_map) == {30}
    positions, quats = world_map.placed.keyframe_poses()
    np.testing.assert_allclose(positions, [[0.5, 0.0, 0.0], [0.0, 0.0, 0.0]], atol=1e-9)
    assert quats.shape == (2, 4)
    assert world_map.placed.loop_segments().shape == (0, 2, 3)

    # a second loop inside the cooldown waits for it
    pgo.n_loops = 2
    assert not world_map.add(_wall(1.525, 105.0), pose)
    assert world_map.add(_wall(1.525, 112.5), pose)
