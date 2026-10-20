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

from unittest.mock import MagicMock

import numpy as np

from dimos.manipulation.grasping.contact_filter import drop_touching, scene_obstacles
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header

TABLE_Z = 0.745


def _cloud(*points: tuple[float, float, float]) -> PointCloud2:
    return PointCloud2.from_numpy(
        np.asarray(points, dtype=np.float32).reshape((-1, 3)), frame_id="world", timestamp=1.0
    )


def _xyz(points: np.ndarray) -> set[tuple[float, float, float]]:
    return {tuple(round(float(v), 3) for v in p) for p in points}


def test_the_scene_keeps_what_is_near_the_object_but_not_on_it() -> None:
    # A can whose lowest seen point is 5 mm above the table.
    target = _cloud(*((0.5, 0.0, TABLE_Z + z) for z in (0.005, 0.05, 0.08)))
    table = [(0.5 + dx, dy, TABLE_Z) for dx in (-0.1, -0.05, 0.05, 0.1) for dy in (-0.1, 0.0, 0.1)]
    neighbour = (0.6, 0.0, TABLE_Z + 0.05)
    scene = _cloud(
        *table,
        neighbour,
        (0.5, 0.008, TABLE_Z + 0.05),  # the can's own edge, missed by its mask: dropped
        (0.5, 0.0, TABLE_Z + 0.25),  # the robot's own finger near its tool tip: dropped
        (1.2, 0.0, TABLE_Z),  # out of the gripper's reach: dropped
    )

    found = scene_obstacles(scene, target, [Vector3(0.5, 0.0, TABLE_Z + 0.3)])

    assert _xyz(found) == _xyz(np.asarray([*table, neighbour], dtype=np.float32))


def test_drop_touching_keeps_the_clear_grasps() -> None:
    model = MagicMock()
    model.check.return_value = np.array([False, True, False, True])
    candidates = GraspCandidateArray(
        Header(1.0, "world"), [GraspCandidate(Pose(), score) for score in (0.9, 0.8, 0.3, 0.2)]
    )
    obstacles = np.zeros((1, 3))

    kept = drop_touching(model, candidates, _cloud((0.5, 0.0, 0.8)), obstacles, 0.005, 0.003)

    assert [c.score for c in kept.candidates] == [0.8, 0.2]
    assert kept.header.frame_id == "world"
    poses, target, scene, contact_margin, obstacle_margin = model.check.call_args.args
    assert poses.shape == (4, 4, 4)
    assert target.shape == (1, 3)
    assert scene is obstacles
    assert (contact_margin, obstacle_margin) == (0.005, 0.003)
