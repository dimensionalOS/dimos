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

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import pytest

from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.agents.typesafe_policy import derive_goal_label, load_scene, scene_labels
from dimos.evals.environments.habitat import HabitatEnvironment
from dimos.evals.suites import typesafe_habitat_hm3d as hm3d, typesafe_habitat_hssd as hssd
from dimos.evals.suites.typesafe_nav import positions, reached
from dimos.evals.types import Outcome
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry

SUITES = [hm3d, hssd]


def _odometry_outcome(tmp_path: Path, xy: Sequence[tuple[float, float]]) -> Outcome:
    """A recording the way Habitat leaves one: nav_msgs Odometry on ``odometry``."""
    db = tmp_path / "memory.db"
    store = SqliteStore(path=str(db))
    odometry = store.stream("odometry", Odometry)
    for i, (x, y) in enumerate(xy):
        odometry.append(Odometry(frame_id="world", pose=Pose(position=(x, y, 0.0))), ts=1000.0 + i)
    store.stop()
    trajectory = TrajectoryBuilder("navigate", name="test").build("answer")
    return Outcome(trajectory=trajectory, artifacts={"recording": db})


@pytest.mark.parametrize("suite", SUITES)
def test_two_cases_each_with_its_own_habitat_launch(suite) -> None:
    assert len(suite.SUITE) == 2
    assert all(isinstance(case.environment, HabitatEnvironment) for case in suite.SUITE)
    assert len({id(case.environment) for case in suite.SUITE}) == 2
    assert all(
        case.environment.config.blueprint == ["habitat-teleop", "mcp-server"]
        for case in suite.SUITE
    )


@pytest.mark.parametrize("suite", SUITES)
def test_every_goal_names_exactly_one_object(suite) -> None:
    """The agent resolves the goal from the instruction; it must be unambiguous."""
    labels = scene_labels(suite.SCENE)
    for case in suite.SUITE:
        goal = derive_goal_label(case.inputs, labels)
        assert case.id.endswith(goal)
        assert sum(goal in label.lower() for label in labels) == 1, goal


@pytest.mark.parametrize("suite", SUITES)
def test_cases_spawn_where_the_benchmark_does(suite) -> None:
    for case in suite.SUITE:
        assert case.environment.config.start_position_ros_override is not None
        assert case.environment.config.scene_id in case.tags


def test_hssd_dataset_lives_under_the_habitat_data_mount() -> None:
    assert str(hssd.DATASET).endswith(
        "target/habitat/data/hssd-hab/hssd-hab.scene_dataset_config.json"
    )
    assert hm3d.SUITE[0].environment.config.scene_dataset_config is None


def test_grader_reads_habitat_odometry(tmp_path: Path) -> None:
    box = load_scene(hm3d.SCENE, "couch").goal_box
    start = (-0.209, -0.059)
    # The couch's nearest edge point: full arrival, and the straight line to it
    # is the ideal path, so full directness.
    beside = (min(max(start[0], box[0]), box[2]), min(max(start[1], box[1]), box[3]))
    outcome = _odometry_outcome(tmp_path, [start, beside])
    with __import__("dimos.evals.types", fromlist=["recording"]).recording(outcome) as store:
        assert len(positions(store)) == 2
    assert reached("couch", hm3d.SCENE)(outcome) == pytest.approx(1.0)
