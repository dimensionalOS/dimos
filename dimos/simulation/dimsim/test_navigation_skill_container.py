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

import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.simulation.dimsim.navigation_skill_container import (
    DimSimNavigationSkillContainer,
)


@pytest.fixture
def container(mocker) -> DimSimNavigationSkillContainer:
    mocker.patch("dimos.models.vl.qwen.QwenVlModel")
    navigation = DimSimNavigationSkillContainer()
    try:
        yield navigation
    finally:
        navigation.stop()


def test_semantic_match_at_current_viewpoint_does_not_start_navigation(
    mocker,
    container,
) -> None:
    spatial_memory = mocker.patch.object(
        container,
        "_spatial_memory",
        create=True,
    )
    spatial_memory.query_detection_viewpoint.return_value = None
    spatial_memory.query_by_text.return_value = [{"distance": 0.1}]
    spatial_memory.eval_memory_generation.return_value = 1
    mocker.patch.object(
        container,
        "_get_goal_pose_from_result",
        return_value=PoseStamped(position=Vector3(2.1, 3.1, 0)),
    )
    navigate = mocker.patch.object(container, "_navigate_to")
    container._latest_odom = PoseStamped(position=Vector3(2.0, 3.0, 0.5))

    result = container._navigate_using_semantic_map("bathtub")

    assert result == (
        "The semantic-map match for 'bathtub' is the current camera viewpoint, "
        "not the object's position. No navigation was started. Observe the current "
        "frame and approach the visible object with bounded local movement."
    )
    navigate.assert_not_called()


def test_semantic_viewpoint_can_be_revisited_after_moving_away(
    mocker,
    container,
) -> None:
    spatial_memory = mocker.patch.object(
        container,
        "_spatial_memory",
        create=True,
    )
    spatial_memory.query_detection_viewpoint.return_value = None
    spatial_memory.eval_memory_generation.return_value = 1
    spatial_memory.query_by_text.return_value = [{"id": "frame-1", "distance": 0.1}]
    goal = PoseStamped(position=Vector3(2.1, 3.1, 0))
    mocker.patch.object(container, "_get_goal_pose_from_result", return_value=goal)
    navigate = mocker.patch.object(container, "_navigate_to", return_value="started")
    container._latest_odom = PoseStamped(position=Vector3(2.0, 3.0, 0.5))

    container._navigate_using_semantic_map("bathtub")
    container._latest_odom = PoseStamped(position=Vector3(8.0, 8.0, 0.5))
    result = container._navigate_using_semantic_map("gray soaking tub")

    assert result == "started"
    navigate.assert_called_once_with(
        goal,
        (
            "Found a prior camera viewpoint matching 'gray soaking tub', "
            "not a confirmed object position"
        ),
    )


def test_exact_lookout_viewpoint_precedes_semantic_embedding_match(
    mocker,
    container,
) -> None:
    spatial_memory = mocker.patch.object(
        container,
        "_spatial_memory",
        create=True,
    )
    detected = PoseStamped(position=Vector3(2.1, 3.1, 0))
    spatial_memory.query_detection_viewpoint.return_value = detected
    navigate = mocker.patch.object(container, "_navigate_to", return_value="started")
    container._latest_odom = PoseStamped(position=Vector3(8.0, 8.0, 0.5))

    result = container._navigate_using_semantic_map("the bathtub")

    assert result == "started"
    navigate.assert_called_once_with(
        detected,
        (
            "Found the camera viewpoint where the lookout detected 'the bathtub', "
            "not a confirmed object position"
        ),
    )
    spatial_memory.query_by_text.assert_not_called()
