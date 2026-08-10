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

"""Hermetic tests for singular Viser preview data."""

from pathlib import Path
from unittest.mock import MagicMock

import pytest

pytest.importorskip("viser", reason="Viser optional dependency is not installed")

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import (
    PlanningSceneInfo,
    VisualizationSession,
    VisualizationStateFrame,
)
from dimos.manipulation.visualization.viser.animation import (
    PreviewAnimation,
    PreviewFrame,
    preview_tick_times,
    scaled_frame_delays,
)
from dimos.manipulation.visualization.viser.visualizer import ViserManipulationVisualizer
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.assets.model import RobotModel


def _model() -> RobotModelConfig:
    return RobotModelConfig(
        model=RobotModel.from_file(Path("/model.urdf")),
        joint_names=["left/j1", "right/j1"],
        planning_groups=[
            PlanningGroupDefinition("left_arm", ("left/j1",), "base", "left/tool"),
            PlanningGroupDefinition("right_arm", ("right/j1",), "base", "right/tool"),
        ],
    )


def test_preview_timing_uses_one_model_track() -> None:
    preview = PreviewAnimation(
        ("left/j1", "right/j1"),
        (
            PreviewFrame(0.0, (0.0, 0.0)),
            PreviewFrame(1.0, (1.0, 0.5)),
            PreviewFrame(3.0, (2.0, 1.0)),
        ),
    )
    assert preview_tick_times(preview) == (0.0, 1.0, 3.0)
    assert scaled_frame_delays(preview.frames, 6.0) == (2.0, 4.0)


def test_visualizer_builds_full_model_preview_from_selected_canonical_joints() -> None:
    visualizer = ViserManipulationVisualizer()
    visualizer._model_config = _model()
    visualizer._current_state = JointState(name=["left/j1", "right/j1"], position=[0.1, 0.2])
    trajectory = JointTrajectory(
        joint_names=["right/j1"],
        points=[TrajectoryPoint(positions=[0.8], time_from_start=1.0)],
    )
    preview = visualizer._raw_preview_animation(trajectory)
    assert preview == PreviewAnimation(("left/j1", "right/j1"), (PreviewFrame(1.0, (0.1, 0.8)),))


def test_visualizer_rejects_unknown_or_duplicate_trajectory_joints() -> None:
    visualizer = ViserManipulationVisualizer()
    visualizer._model_config = _model()
    visualizer._current_state = JointState(name=["left/j1", "right/j1"], position=[0.1, 0.2])
    for names in (["unknown"], ["left/j1", "left/j1"]):
        trajectory = JointTrajectory(
            joint_names=names,
            points=[TrajectoryPoint(positions=[0.0] * len(names), time_from_start=1.0)],
        )
        assert visualizer._raw_preview_animation(trajectory) is None


def test_visualizer_initializes_and_updates_one_scene_model() -> None:
    visualizer = ViserManipulationVisualizer()
    scene = MagicMock()
    visualizer._scene = scene
    visualizer._runtime = MagicMock()
    visualizer._initialize_scene(PlanningSceneInfo(model=_model()))
    scene.register_model.assert_called_once()

    state = JointState(name=["left/j1", "right/j1"], position=[0.1, 0.2])
    visualizer.update_state(VisualizationStateFrame(joint_state=state))
    scene.update_current_model.assert_called_once_with(state)


def test_visualization_session_contains_singular_model() -> None:
    session = VisualizationSession(PlanningSceneInfo(model=_model()), operator=object())
    assert session.scene.model.joint_names == ["left/j1", "right/j1"]
