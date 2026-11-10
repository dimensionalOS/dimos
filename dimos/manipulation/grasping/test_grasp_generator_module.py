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

from collections.abc import Iterator
import json
import math
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from dimos.manipulation.grasping.grasp_generator_module import (
    GraspGeneratorModule,
    _approach_angle,
)
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header


def _tilted(tilt_deg: float, score: float = 1.0) -> GraspCandidate:
    """A grasp on the cup whose approach leans ``tilt_deg`` from straight down, towards +Y."""
    roll = -math.pi + math.radians(tilt_deg)
    return GraspCandidate(
        Pose(Vector3(0.1, 0.0, 0.2), Quaternion.from_euler(Vector3(roll, 0, 0))), score
    )


class FakeGenerator(GraspGeneratorModule):
    """Returns whatever grasps the test sets."""

    grasps: list[GraspCandidate]

    def propose_grasps(self, object_pointcloud: PointCloud2) -> GraspCandidateArray:
        return GraspCandidateArray(Header(1.0, object_pointcloud.frame_id), self.grasps)


def _scene(points: list[list[float]] | None) -> Any:
    """A scene whose every object has these points, in the world frame; None knows no object."""
    scene = MagicMock()
    scene.get_object_pointcloud_by_object_id.return_value = (
        None
        if points is None
        else PointCloud2.from_numpy(
            np.asarray(points, dtype=np.float32).reshape((-1, 3)), frame_id="world", timestamp=1.0
        )
    )
    return scene


@pytest.fixture
def generator() -> Iterator[FakeGenerator]:
    instance = FakeGenerator()
    instance._scene = _scene([[0.08, -0.02, 0.18], [0.12, 0.02, 0.22]])
    yield instance
    instance.stop()


def test_a_straight_down_grasp_is_0_degrees_from_straight_down() -> None:
    heuristic = HeuristicGraspModule()
    heuristic._scene = _scene([[0.08, -0.02, 0.18], [0.12, 0.02, 0.22], [0.1, 0.0, 0.2]])
    try:
        down = heuristic._propose_for("cup-1", [0.0, 0.0, -1.0], 30.0, 3)
        up = heuristic._propose_for("cup-1", [0.0, 0.0, 1.0], 30.0, 3)
    finally:
        heuristic.stop()

    assert down["grasps"][0]["angle_deg"] == pytest.approx(0.0, abs=1e-6)
    assert up["grasps"] == []
    assert up["nearest_angle_deg"] == pytest.approx(180.0)


def test_the_approach_angle_is_measured_along_the_pick_motion() -> None:
    """For any tilt, the pre-grasp to grasp direction has an angle of 0 to the grasp's approach."""
    grasp = PoseStamped(
        frame_id="world",
        position=Vector3(0.1, 0.0, 0.2),
        orientation=Quaternion.from_euler(Vector3(-2.6, 0.4, 0.9)),
    )
    pregrasp = PickAndPlaceModule._offset_pose(grasp, 0.1)
    motion = (grasp.position - pregrasp.position).normalize()

    assert _approach_angle(grasp, motion) == pytest.approx(0.0, abs=1e-5)
    assert _approach_angle(grasp, -motion) == pytest.approx(180.0, abs=1e-5)


def test_the_cone_keeps_grasps_inside_the_angle_and_drops_the_rest(
    generator: FakeGenerator,
) -> None:
    generator.grasps = [_tilted(0.0, 0.9), _tilted(50.0, 0.8), _tilted(20.0, 0.7), _tilted(90.0)]

    # Not unit length: the direction is what counts.
    proposed = generator._propose_for("cup-1", [0.0, 0.0, -2.0], 30.0, 3)

    assert [g["angle_deg"] for g in proposed["grasps"]] == pytest.approx([0.0, 20.0], abs=1e-6)
    assert [g["score"] for g in proposed["grasps"]] == [0.9, 0.7]
    assert (proposed["considered"], proposed["nearest_angle_deg"]) == (4, pytest.approx(0.0))


def test_the_best_scores_come_first_cut_to_the_limit(generator: FakeGenerator) -> None:
    generator.grasps = [_tilted(5.0, 0.2), _tilted(10.0, 0.9), _tilted(0.0, 0.6)]

    proposed = generator._propose_for("cup-1", [0.0, 0.0, -1.0], 30.0, 2)

    assert [(g["rank"], g["score"]) for g in proposed["grasps"]] == [(0, 0.9), (1, 0.6)]
    first = proposed["grasps"][0]
    assert first["position"] == pytest.approx([0.1, 0.0, 0.2])
    assert first["orientation"] == pytest.approx(list(_tilted(10.0).pose.orientation.to_tuple()))
    assert first["generator"] == proposed["generator"] == "FakeGenerator"
    assert (proposed["object_id"], proposed["frame"]) == ("cup-1", "world")
    json.dumps(proposed)


def test_an_empty_cone_says_how_near_the_nearest_grasp_is(generator: FakeGenerator) -> None:
    generator.grasps = [_tilted(0.0), _tilted(40.0)]

    proposed = generator._propose_for("cup-1", [0.0, 1.0, 0.0], 30.0, 3)

    assert proposed["grasps"] == []
    assert proposed["nearest_angle_deg"] == pytest.approx(50.0)
    assert proposed["considered"] == 2


def test_no_grasps_at_all_has_no_nearest_angle(generator: FakeGenerator) -> None:
    generator.grasps = []

    proposed = generator._propose_for("cup-1", [0.0, 0.0, -1.0], 30.0, 3)

    assert (proposed["grasps"], proposed["nearest_angle_deg"], proposed["considered"]) == (
        [],
        None,
        0,
    )


def test_without_an_approach_every_grasp_counts(generator: FakeGenerator) -> None:
    generator.grasps = [_tilted(0.0), _tilted(90.0)]

    proposed = generator._propose_for("cup-1", None, 30.0, 3)

    assert [g["angle_deg"] for g in proposed["grasps"]] == [0.0, 0.0]


@pytest.mark.parametrize("points", [None, []])
def test_an_object_without_points_is_refused(
    generator: FakeGenerator, points: list[list[float]] | None
) -> None:
    generator._scene = _scene(points)

    with pytest.raises(ValueError, match="No points for object cup-9"):
        generator._propose_for("cup-9", None, 30.0, 3)


def test_a_stack_without_a_scene_is_refused(generator: FakeGenerator) -> None:
    generator._scene = None

    with pytest.raises(ValueError, match="no scene"):
        generator._propose_for("cup-1", None, 30.0, 3)


@pytest.mark.parametrize(
    ("approach", "limit", "match"),
    [([0.0, 0.0, 0.0], 3, "approach must be a direction"), (None, 0, "limit")],
)
def test_bad_arguments_are_refused(
    generator: FakeGenerator, approach: list[float] | None, limit: int, match: str
) -> None:
    with pytest.raises(ValueError, match=match):
        generator._propose_for("cup-1", approach, 30.0, limit)
