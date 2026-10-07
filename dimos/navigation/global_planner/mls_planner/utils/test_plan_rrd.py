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
from pytest_mock import MockerFixture

pytest.importorskip("dimos_voxel_ray_tracing")
pytest.importorskip("dimos_mls_planner")

from dimos.mapping.ray_tracing.transformer import RayTraceMap
from dimos.memory.tf import StreamTF
from dimos.memory.type.observation import Observation
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.global_planner.mls_planner.mls_planner import MLSPlanner
from dimos.navigation.global_planner.mls_planner.utils.plan_rrd import Seeding, SeedStage

VOXEL_SIZE = 0.1


def _patch(x0: float) -> np.ndarray:
    side = np.arange(5, dtype=np.float32) * VOXEL_SIZE + 0.45
    return np.array([(x0 + x, y, 0.05) for x in side for y in side], dtype=np.float32)


def test_seeding_lands_one_region_per_step_and_matches_a_full_rebuild(
    mocker: MockerFixture,
) -> None:
    # Three floor patches, each in its own region cell.
    cloud = np.vstack([_patch(0.0), _patch(8.0), _patch(16.0)])
    loaded_map = Observation(id=0, ts=5.0, _data=PointCloud2.from_numpy(cloud, frame_id="map"))
    tf_lookup = mocker.create_autospec(StreamTF, instance=True)
    tf_lookup.get.return_value = Transform(frame_id="odom", child_frame_id="map")
    # No support gate, so every seeded voxel reaches the planner.
    ray = RayTraceMap(voxel_size=VOXEL_SIZE, support_min=0)
    planner = MLSPlanner(voxel_size=VOXEL_SIZE, robot_height=0.5)
    seeding = Seeding(loaded_map, ray, [planner], tf_lookup, "odom")
    start = (0.5, 0.5, 0.0)

    seeding.step(4.0, start)
    assert seeding.stage is SeedStage.PENDING and planner.voxel_count() == 0

    left = []
    for _ in range(4):
        seeding.step(5.0, start)
        left.append(seeding.left)
    assert left == [3, 2, 1, 0]
    assert seeding.stage is SeedStage.LOADING
    seeding.step(5.0, start)
    assert seeding.stage is SeedStage.DONE

    rebuilt = MLSPlanner(voxel_size=VOXEL_SIZE, robot_height=0.5)
    rebuilt.update_global_map(cloud)
    assert planner.voxel_count() == rebuilt.voxel_count() == len(cloud)


def test_seeding_without_a_loaded_map_does_nothing(mocker: MockerFixture) -> None:
    ray = RayTraceMap(voxel_size=VOXEL_SIZE)
    planner = MLSPlanner(voxel_size=VOXEL_SIZE, robot_height=0.5)
    tf_lookup = mocker.create_autospec(StreamTF, instance=True)
    seeding = Seeding(None, ray, [planner], tf_lookup, "odom")
    seeding.step(5.0, (0.0, 0.0, 0.0))
    assert seeding.stage is SeedStage.ABSENT
    assert planner.voxel_count() == 0
    tf_lookup.get.assert_not_called()
