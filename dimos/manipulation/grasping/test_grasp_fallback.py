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
from unittest.mock import MagicMock

import numpy as np
import pytest

from dimos.manipulation.grasping.grasp_fallback import (
    GraspProposals,
    SceneCheck,
    grasp_sources,
    propose_with_fallback,
    provider_name,
    same_grasp,
)
from dimos.manipulation.grasping.grasp_gen_spec import SceneGrasps
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
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


def _cloud() -> PointCloud2:
    """A 4 cm cube of points around (0.1, 0, 0.2), metres."""
    corners = np.array(np.meshgrid([0.08, 0.12], [-0.02, 0.02], [0.18, 0.22])).reshape(3, -1).T
    return PointCloud2.from_numpy(corners.astype(np.float32), frame_id="world", timestamp=1.0)


def _graspgenx() -> MagicMock:
    provider = MagicMock()
    provider.remote_name = "GraspGenXModule"
    provider.propose_grasps.return_value = _array(0.1, 0.2)
    provider.propose_grasps_in_scene.return_value = SceneGrasps(_array(0.3), 37)
    return provider


def test_a_scene_check_takes_the_checked_grasps_and_counts_the_dropped_ones() -> None:
    provider = _graspgenx()
    scene = _cloud()

    proposals = propose_with_fallback(
        provider, _cloud(), True, HeuristicGraspConfig(), SceneCheck(provider, scene, 0.01)
    )

    provider.propose_grasps.assert_not_called()
    _, scene_arg, threshold = provider.propose_grasps_in_scene.call_args.args
    assert scene_arg is scene
    assert threshold == 0.01
    assert proposals.provided is not None
    assert [c.pose.position.x for c in proposals.provided] == pytest.approx([0.3])
    facts = proposals.facts()
    assert facts["scene_filtered_by_source"] == {"graspgenx": 37}
    assert facts["candidates_by_source"]["graspgenx"] == 1
    # Heuristic grasps are still added, unchecked.
    assert facts["candidates_by_source"]["heuristic"] > 0
    assert "scene_filter_skipped" not in facts


def test_without_a_scene_check_nothing_is_counted() -> None:
    provider = _graspgenx()

    proposals = propose_with_fallback(provider, _cloud(), False, HeuristicGraspConfig())

    provider.propose_grasps_in_scene.assert_not_called()
    assert proposals.facts()["scene_filtered_by_source"] == {}


def test_a_skipped_scene_check_says_why() -> None:
    proposals = propose_with_fallback(
        _graspgenx(), _cloud(), False, HeuristicGraspConfig(), scene_skipped="no scene cloud"
    )

    facts = proposals.facts()
    assert facts["scene_filtered_by_source"] == {}
    assert facts["scene_filter_skipped"] == "no scene cloud"


def test_a_failing_scene_check_falls_back_like_a_failing_provider() -> None:
    provider = _graspgenx()
    provider.propose_grasps_in_scene.side_effect = RuntimeError("CUDA out of memory")
    check = SceneCheck(provider, _cloud(), 0.01)

    proposals = propose_with_fallback(provider, _cloud(), True, HeuristicGraspConfig(), check)

    assert proposals.provided is None
    assert proposals.facts()["grasp_provider_error"] == "RuntimeError: CUDA out of memory"
    assert proposals.facts()["scene_filtered_by_source"] == {}
    with pytest.raises(RuntimeError, match="CUDA"):
        propose_with_fallback(provider, _cloud(), False, HeuristicGraspConfig(), check)
