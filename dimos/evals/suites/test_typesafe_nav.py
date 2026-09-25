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

"""The grader over synthetic odom recordings, and case/agent goal agreement."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import pytest

from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.agents.typesafe_policy import derive_goal_label, load_scene, scene_labels
from dimos.evals.scorers import ramp
from dimos.evals.suites.typesafe_nav import (
    ARRIVAL_BAND_M,
    CASES,
    SCENE,
    SUITE,
    distance_to_box,
    goal_box,
    reached,
)
from dimos.evals.types import Outcome
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

SPAWN = (3.0, 2.0)  # DimSim apartment spawnPoint, in the ROS world frame
COUCH = (0.156, 3.028, 1.956, 5.735)


def _outcome(tmp_path: Path, xy: Sequence[tuple[float, float]]) -> Outcome:
    db = tmp_path / "memory.db"
    store = SqliteStore(path=str(db))
    odom = store.stream("odom", PoseStamped)
    for i, (x, y) in enumerate(xy):
        odom.append(
            PoseStamped(position=(x, y, 0.0), orientation=(0, 0, 0, 1), frame_id="world"),
            ts=1000.0 + i,
        )
    store.stop()
    trajectory = TrajectoryBuilder("navigate", name="test").build("answer")
    return Outcome(trajectory=trajectory, artifacts={"recording": db})


def test_couch_box_is_pinned_to_the_scene_file() -> None:
    assert goal_box("sectional") == COUCH


def test_bathtub_is_in_the_bathroom() -> None:
    """x < 0 puts it past the main wall; y > 1.07 puts it past the bathroom wall."""
    minx, miny, maxx, maxy = goal_box("bathtub")
    assert maxx < 0.0 and miny > 1.07
    assert distance_to_box(*SPAWN, (minx, miny, maxx, maxy)) > ARRIVAL_BAND_M


def test_every_case_names_a_goal_the_agent_resolves_identically() -> None:
    """The agent derives the goal from the instruction; the grader from CASES.
    They must land on the same object, or the eval measures nothing."""
    labels = scene_labels(SCENE)
    assert [c.id for c in SUITE] == [case_id for case_id, _, _ in CASES]
    for case, (_, instruction, goal) in zip(SUITE, CASES, strict=True):
        assert case.inputs == instruction
        derived = derive_goal_label(instruction, labels)
        assert load_scene(SCENE, derived).goal_box == goal_box(goal)


def test_each_case_has_its_own_environment() -> None:
    envs = [case.environment for case in SUITE]
    assert len({id(env) for env in envs}) == len(envs)


def test_case_tags_allow_selecting_one() -> None:
    assert {"bathtub"} & SUITE[1].tags and not {"bathtub"} & SUITE[0].tags


@pytest.mark.parametrize(
    ("xy", "expected"),
    [
        ((1.0, 4.0), 0.0),  # inside
        ((1.0, 2.028), 1.0),  # straight below the bottom edge
        ((3.0, 2.0), ((3.0 - 1.956) ** 2 + (2.0 - 3.028) ** 2) ** 0.5),  # off a corner
    ],
)
def test_distance_to_box(xy: tuple[float, float], expected: float) -> None:
    assert distance_to_box(*xy, COUCH) == pytest.approx(expected)


def test_robot_that_never_moved_earns_no_directness(tmp_path: Path) -> None:
    """Regression: odom jitter must not earn directness credit. Sitting at
    spawn is still 1.46 m from the couch, inside the 2 m band, so arrival
    alone is what remains."""
    score = reached("sectional")(_outcome(tmp_path, [SPAWN] * 50))
    assert score == pytest.approx(0.7 * ramp(distance_to_box(*SPAWN, COUCH), ARRIVAL_BAND_M))


def test_touching_the_couch_scores_full_arrival(tmp_path: Path) -> None:
    """Where the second live run actually ended: pressed against the couch."""
    end = (1.23, 3.19)  # inside the box, 0.16 m past its south edge
    n = 20
    path = [
        (SPAWN[0] + (end[0] - SPAWN[0]) * i / n, SPAWN[1] + (end[1] - SPAWN[1]) * i / n)
        for i in range(n + 1)
    ]
    score = reached("sectional")(_outcome(tmp_path, path))
    ideal = distance_to_box(*SPAWN, COUCH)
    travelled = ((end[0] - SPAWN[0]) ** 2 + (end[1] - SPAWN[1]) ** 2) ** 0.5
    assert score == pytest.approx(0.7 * 1.0 + 0.3 * min(1.0, ideal / travelled))
    assert score >= 0.6
