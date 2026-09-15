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

from pathlib import Path

import pytest

from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.suites.pointcloud.sim.dimsim_pointcloud_mapping import ROOMS, grade_rooms
from dimos.evals.types import Outcome
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


def _outcome(tmp_path: Path, positions: list[tuple[float, float]], answer: str) -> Outcome:
    path = tmp_path / "memory.db"
    with SqliteStore(path=str(path)) as store:
        odom = store.stream("odom", PoseStamped)
        for i, (x, y) in enumerate(positions):
            odom.append(PoseStamped(position=[x, y, 0.5], frame_id="world", ts=float(i)))
    trajectory = TrajectoryBuilder("Count rooms", name="test")
    trajectory.step(message=answer, request=tmp_path / "request", response=tmp_path / "response")
    return Outcome(trajectory=trajectory.build("answer"), artifacts={"recording": path})


def test_living_side_of_kitchen_door_does_not_earn_kitchen_visit(tmp_path: Path) -> None:
    # The kitchen starts at world y=-2. This living-room sample was within
    # 1.5m of the old, bathroom-labelled kitchen representative (3,-2.5).
    outcome = _outcome(tmp_path, [(2.5, -1.1)], "4")

    assert grade_rooms()(outcome) == 0.5


@pytest.mark.parametrize("answer,expected", [("4", 1.0), ("3", 0.5)])
def test_visiting_all_interiors_preserves_count_and_coverage_weights(
    tmp_path: Path, answer: str, expected: float
) -> None:
    # Cross the kitchen, main and bathroom doorways in published world axes.
    path = [
        (2.0, 2.5),
        (2.5, -2.0),
        (2.5, -3.0),
        (2.5, -2.0),
        (0.0, 0.0),
        (-3.0, -2.5),
        (-2.5, 1.0),
        (-2.5, 2.0),
    ]
    outcome = _outcome(tmp_path, path, answer)

    assert grade_rooms()(outcome) == expected


@pytest.mark.parametrize(
    "room,xmin,xmax,ymin,ymax",
    [
        ("living_dining", 0, 5, -2, 6),
        ("kitchen", 0, 5, -6, -2),
        ("bedroom", -5, 0, -6, 1),
        ("bathroom", -5, 0, 1, 6),
    ],
)
def test_default_visit_disk_is_inside_its_named_physical_room(
    room: str, xmin: float, xmax: float, ymin: float, ymax: float
) -> None:
    # Apartment structure.glb wall footprints after Three.js (x,y,z) ->
    # published world (z,x,y); these bounds also validate the room names.
    x, y = ROOMS[room]

    assert xmin < x - 1.5 < x + 1.5 < xmax
    assert ymin < y - 1.5 < y + 1.5 < ymax
