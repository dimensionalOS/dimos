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
from types import SimpleNamespace

import pytest

from dimos.manipulation.grasping.grasp_fallback import (
    GraspProposals,
    grasp_sources,
    provider_name,
    same_grasp,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header


def _pose(x: float, yaw: float = 0.0) -> Pose:
    return Pose(Vector3(x, 0.0, 0.1), Quaternion.from_euler(Vector3(-math.pi, 0.0, yaw)))


def _array(*xs: float) -> GraspCandidateArray:
    return GraspCandidateArray(Header(1.0, "world"), [GraspCandidate(_pose(x)) for x in xs])


@pytest.mark.parametrize(
    "remote_name, name",
    [
        ("GraspGenXModule", "graspgenx"),
        ("HeuristicGraspModule", "heuristic"),
        (None, "provider"),
    ],
)
def test_provider_name(remote_name: str | None, name: str) -> None:
    assert provider_name(SimpleNamespace(remote_name=remote_name)) == name


def test_grasp_sources_name_each_source_once() -> None:
    graspgenx = SimpleNamespace(remote_name="GraspGenXModule")
    heuristic = SimpleNamespace(remote_name="HeuristicGraspModule")

    assert grasp_sources(graspgenx, True) == ["graspgenx", "heuristic"]
    assert grasp_sources(graspgenx, False) == ["graspgenx"]
    assert grasp_sources(heuristic, True) == ["heuristic"]


def test_a_grasp_turned_half_a_turn_is_the_same_grasp() -> None:
    assert same_grasp(_pose(0.0), _pose(0.01, math.pi), 0.02, 0.35)
    assert not same_grasp(_pose(0.0), _pose(0.03), 0.02, 0.35)
    assert not same_grasp(_pose(0.0), _pose(0.0, math.pi / 2.0), 0.02, 0.35)


def test_each_source_is_capped_and_skipped_grasps_are_left_out() -> None:
    proposals = GraspProposals(
        "graspgenx", ["graspgenx", "heuristic"], _array(0.1, 0.2, 0.3), _array(0.5, 0.6, 0.7)
    )

    grasps = proposals.to_try(2, skip=lambda c: c.pose.position.x == pytest.approx(0.1))

    assert [(g.source, g.rank) for g in grasps] == [
        ("graspgenx", 1),
        ("graspgenx", 2),
        ("heuristic", 0),
        ("heuristic", 1),
    ]
