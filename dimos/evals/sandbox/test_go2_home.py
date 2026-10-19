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

import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.evals.sandbox.case import Conversation
from dimos.evals.sandbox.go2_home import (
    PLANTS,
    SUITE,
    came_home,
    home_owner,
    plant_visits,
    what_went_wrong,
)
from dimos.evals.sandbox.scene_grading import GroundTruth

SIDEBOARD = PLANTS[2]


class OpenHouse:
    """Nothing in the way of any line of sight, or a wall *wall* metres along every one."""

    def __init__(self, wall: float = -1.0) -> None:
        self.wall = wall

    def cast_static(
        self, origins: np.ndarray, directions: np.ndarray, max_range: float
    ) -> np.ndarray:
        return np.full(len(origins), self.wall)


def truth_of(path: list[tuple[float, float, float]], contacts: list | None = None) -> GroundTruth:
    """Ground truth of the dog at each (x, y, yaw), at 10 Hz."""
    t = np.arange(len(path)) / 10.0
    quats = Rotation.from_euler("z", [[yaw] for _, _, yaw in path]).as_quat()
    poses = np.column_stack([np.array(path)[:, :2], np.full(len(path), 0.3), quats])
    contact_t = np.array([c[0] for c in contacts or []], np.float64)
    return GroundTruth(
        t, poses, contact_t, [c[1] for c in contacts or []], np.zeros(0), np.zeros((0, 2, 3))
    )


def stop_at(x: float, y: float, yaw: float, seconds: float) -> list[tuple[float, float, float]]:
    return [(x, y, yaw)] * round(seconds * 10)


# 0.8 m in front of the sideboard plant, facing it (it is to the south).
x, y = SIDEBOARD.xy[0], SIDEBOARD.xy[1] + 0.8
FACING_IT = -math.pi / 2


def test_stopping_by_a_plant_facing_it_visits_it() -> None:
    truth = truth_of(stop_at(0, 0, 0, 1) + stop_at(x, y, FACING_IT, 3))
    assert plant_visits(truth, OpenHouse()) == {SIDEBOARD.name: 1.0}


def test_passing_facing_away_too_briefly_or_through_a_wall_is_no_visit() -> None:
    walking_past = [(x - 1.0 + 0.05 * i, y, FACING_IT) for i in range(40)]  # 0.5 m/s
    assert plant_visits(truth_of(walking_past), OpenHouse()) == {}
    assert plant_visits(truth_of(stop_at(x, y, -FACING_IT, 5)), OpenHouse()) == {}
    assert plant_visits(truth_of(stop_at(x, y, FACING_IT, 1.5)), OpenHouse()) == {}
    assert plant_visits(truth_of(stop_at(x, y, FACING_IT, 5)), OpenHouse(wall=0.3)) == {}


def test_the_owner_says_what_was_missed_and_who_was_walked_into() -> None:
    path = stop_at(4.0, 3.0, 0, 1) + stop_at(x, y, FACING_IT, 3) + stop_at(4.0, 3.0, 0, 1)
    truth = truth_of(path, contacts=[(1.5, [("trunk", "person")]), (1.6, [("foot", "floor")])])
    assert came_home(truth, 0.0, plant_visits(truth, OpenHouse()))
    problems = what_went_wrong(truth, 0.0, OpenHouse())
    assert "the palm in the back bedroom" in problems[0] and SIDEBOARD.name not in problems[0]
    assert problems[1] == "It walked into one of us once! It must never touch anyone."
    assert len(problems) == 2  # it came home, and bumped into nothing else


def test_the_dimos_case_is_the_home_case_plus_the_framework_requirement() -> None:
    plain, dimos = SUITE
    assert (plain.id, dimos.id) == ("home_plants", "home_plants_dimos")
    added = dimos.prompt.removeprefix(plain.prompt)
    assert added != dimos.prompt and "Dimensional framework" in added
    assert plain.model_copy(update={"id": dimos.id, "prompt": dimos.prompt}) == dimos


def test_the_owner_gives_up_after_nagging_an_agent_that_never_calls_done() -> None:
    conversation = Conversation(exec=lambda command: None, case_dir=Path("."))
    owner = home_owner(conversation, nag="go on", nags=3)
    assert list(owner) == ["go on"] * 3
