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

"""Ranked grasp-proposal markers, without a running Viser server."""

import inspect
from unittest.mock import MagicMock

from dimos_generated.geometry_msgs.msg import Point
import numpy as np
import pytest

pytest.importorskip("viser", reason="Viser optional dependency is not installed")

from dimos_generated.dimos_msgs.msg import GraspCandidate, GraspCandidateArray
from dimos_generated.geometry_msgs.msg import Pose, Quaternion
from dimos_generated.std_msgs.msg import Header

from dimos.manipulation.visualization.viser.scene import (
    GRASP_PROPOSAL_DRAW_LIMIT,
    GRASP_PROPOSAL_EMPHASIS_COUNT,
    ViserManipulationScene,
)
from dimos.msgs.time import time_from_seconds


def _scene() -> ViserManipulationScene:
    return ViserManipulationScene(MagicMock(), MagicMock())


def _proposals(count: int) -> GraspCandidateArray:
    return GraspCandidateArray(
        header=Header(stamp=time_from_seconds(1.0), frame_id="world"),
        candidates=[
            GraspCandidate(
                pose=Pose(
                    position=Point(x=0.1 * index, y=0.0, z=0.2),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                score=1.0 - 0.01 * index,
            )
            for index in range(count)
        ],
    )


def test_proposals_draw_leaders_and_the_rest_as_separate_groups() -> None:
    """The leaders are drawn thicker, so they are their own line-segment node."""
    scene = _scene()

    scene.show_grasp_proposals(_proposals(10))

    calls = scene.server.scene.add_line_segments.call_args_list
    assert len(calls) == 2
    drawn = sum(len(call.kwargs["points"]) for call in calls)
    assert drawn == 20  # two segments per grasp: approach, and the closing axis
    assert scene.server.scene.add_label.call_count == GRASP_PROPOSAL_EMPHASIS_COUNT


def test_the_drawn_arrays_match_what_viser_accepts() -> None:
    """A mocked server takes any shape; the real one silently raises behind a log."""
    from viser import SceneApi

    scene = _scene()

    scene.show_grasp_proposals(_proposals(6))

    signature = inspect.signature(SceneApi.add_line_segments)
    for call in scene.server.scene.add_line_segments.call_args_list:
        signature.bind(scene.server.scene, *call.args, **call.kwargs)
        points = np.asarray(call.kwargs["points"])
        colors = np.asarray(call.kwargs["colors"])
        # add_line_segments wants (N, 2, 3) endpoints and one colour per endpoint.
        assert points.shape[1:] == (2, 3)
        assert colors.shape == points.shape
    label_signature = inspect.signature(SceneApi.add_label)
    for call in scene.server.scene.add_label.call_args_list:
        label_signature.bind(scene.server.scene, *call.args, **call.kwargs)


def test_ranking_reads_as_a_colour_gradient() -> None:
    """Scores only mean anything against each other, so rank drives the colour."""
    scene = _scene()

    scene.show_grasp_proposals(_proposals(10))

    trailing, leaders = scene.server.scene.add_line_segments.call_args_list
    best = np.asarray(leaders.kwargs["colors"])[0][0]
    worst = np.asarray(trailing.kwargs["colors"])[-1][0]
    assert best[1] > worst[1]  # green fades out towards the worst rank
    assert best[0] < worst[0]  # red comes up


def test_a_hundred_proposals_do_not_all_get_drawn() -> None:
    """Drawing every candidate buries the ranking it is there to show."""
    scene = _scene()

    scene.show_grasp_proposals(_proposals(100))

    drawn = sum(
        len(call.kwargs["points"]) for call in scene.server.scene.add_line_segments.call_args_list
    )
    assert drawn == 2 * GRASP_PROPOSAL_DRAW_LIMIT


def test_an_empty_array_clears_the_markers() -> None:
    scene = _scene()
    scene.show_grasp_proposals(_proposals(4))

    scene.show_grasp_proposals(GraspCandidateArray())

    assert scene._grasp_proposal_handles == []


def test_redrawing_removes_the_previous_proposals() -> None:
    """A stale overlay beside a fresh one is worse than no overlay."""
    scene = _scene()
    scene.show_grasp_proposals(_proposals(4))
    handle = scene.server.scene.add_line_segments.return_value
    handle.remove.assert_not_called()

    scene.show_grasp_proposals(_proposals(2))

    assert handle.remove.called
