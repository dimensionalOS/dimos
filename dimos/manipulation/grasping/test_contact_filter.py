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

from dimos.manipulation.grasping.contact_filter import add_facts, drop_touching, scene_obstacles
from dimos.manipulation.planning.spec.gripper import GraspContact
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


def _xyz(cloud: PointCloud2) -> set[tuple[float, float, float]]:
    return {tuple(round(float(v), 3) for v in p) for p in cloud.points_f32()}


def test_the_scene_splits_into_the_table_under_the_object_and_other_objects() -> None:
    # A can whose lowest seen point is 5 mm above the table.
    target = _cloud(*((0.5, 0.0, TABLE_Z + z) for z in (0.005, 0.05, 0.08)))
    table = [(0.5 + dx, dy, TABLE_Z) for dx in (-0.1, -0.05, 0.05, 0.1) for dy in (-0.1, 0.0, 0.1)]
    scene = _cloud(
        *table,
        (0.5, 0.006, TABLE_Z + 0.002),  # the table right at the can's base: table
        (0.5, 0.008, TABLE_Z + 0.05),  # the can's own edge, missed by its mask: dropped
        (0.6, 0.0, TABLE_Z + 0.05),  # a neighbour: objects
        (0.5, 0.0, TABLE_Z + 0.25),  # the robot's own finger near its tool tip: dropped
        (1.2, 0.0, TABLE_Z),  # out of the gripper's reach: dropped
    )

    found = scene_obstacles(scene, target, [Vector3(0.5, 0.0, TABLE_Z + 0.3)])

    assert _xyz(found["table"]) == _xyz(_cloud(*table, (0.5, 0.006, TABLE_Z + 0.002)))
    assert _xyz(found["objects"]) == {(0.6, 0.0, round(TABLE_Z + 0.05, 3))}
    assert found["table"].frame_id == found["objects"].frame_id == "world"


def test_with_no_table_under_the_object_every_point_is_an_object() -> None:
    target = _cloud((0.5, 0.0, 0.9))

    found = scene_obstacles(scene=_cloud((0.6, 0.0, 1.0)), target=target, tips=[])

    assert len(found["table"].points_f32()) == 0
    assert _xyz(found["objects"]) == {(0.6, 0.0, 1.0)}


def _array(*scores: float) -> GraspCandidateArray:
    return GraspCandidateArray(
        Header(1.0, "world"), [GraspCandidate(Pose(), score) for score in scores]
    )


def test_drop_touching_keeps_the_clear_grasps() -> None:
    manipulation = MagicMock()
    manipulation.check_grasp_contacts.return_value = [
        GraspContact(("target",)),
        GraspContact(),
        GraspContact(("table", "objects"), while_closing=True),
        GraspContact(),
    ]

    kept, facts = drop_touching(
        manipulation,
        _array(0.9, 0.8, 0.3, 0.2),
        _cloud(),
        {"table": _cloud(), "objects": _cloud()},
        0.005,
        0.003,
        "arm",
    )

    assert [c.score for c in kept.candidates] == [0.8, 0.2]
    del facts["seconds"]
    assert facts == {
        "checked": 4,
        "rejected": 2,
        "target": 1,
        "table": 1,
        "objects": 1,
        "while_closing": 1,
    }


def test_drop_touching_keeps_everything_without_a_gripper_spec() -> None:
    candidates = _array(0.9)
    manipulation = MagicMock()
    manipulation.check_grasp_contacts.return_value = None

    kept, facts = drop_touching(manipulation, candidates, _cloud(), {}, 0.005, 0.003, None)

    assert kept is candidates
    assert facts == {"skipped": "the robot model has no gripper spec"}


def test_a_pick_adds_up_the_checks_of_each_scan() -> None:
    total: dict[str, object] = {}
    add_facts(total, {"checked": 4, "rejected": 2, "seconds": 0.25})
    add_facts(total, {"checked": 3, "rejected": 0, "seconds": 0.5})

    assert total == {"checked": 7, "rejected": 2, "seconds": 0.75}
