# Copyright 2025-2026 Dimensional Inc.
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

"""Generated-plan materialization tests."""

from pathlib import Path
from unittest.mock import MagicMock

import pytest

from dimos.manipulation.manipulation_module import ManipulationState
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.planners.roboplan_config import RoboPlanCartesianPathConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import PlanningResult
from dimos.manipulation.planning.trajectory_generator.config import (
    SimpleTrapezoidParametrizationConfig,
)
from dimos.manipulation.planning.trajectory_generator.simple_parametrizer import (
    SimpleTrapezoidParametrizer,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.assets.model import RobotModel


class RecordingGenerator:
    calls: list[list[list[float]]] = []
    limits: tuple[list[float], list[float]] | None = None

    def __init__(
        self,
        num_joints: int,
        max_velocity: list[float],
        max_acceleration: list[float],
        points_per_segment: int = 50,
    ) -> None:
        self.num_joints = num_joints
        self.points_per_segment = points_per_segment
        RecordingGenerator.limits = (list(max_velocity), list(max_acceleration))

    def generate(self, waypoints: list[list[float]]) -> JointTrajectory:
        RecordingGenerator.calls.append(waypoints)
        return JointTrajectory(
            points=[
                TrajectoryPoint(
                    time_from_start=float(index),
                    positions=list(point),
                    velocities=[0.0] * self.num_joints,
                )
                for index, point in enumerate(waypoints)
            ]
        )


def _model() -> RobotModelConfig:
    return RobotModelConfig(
        model=RobotModel.from_file(Path("/robot.urdf")),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["left/a", "left/b", "right/c"],
        base_link="base",
        planning_groups=[
            PlanningGroupDefinition(
                name="left_arm",
                joint_names=("left/b", "left/a"),
                base_link="base",
                tip_link="left_tip",
            ),
            PlanningGroupDefinition(
                name="right_arm",
                joint_names=("right/c",),
                base_link="base",
                tip_link="right_tip",
            ),
        ],
        max_velocity=3.0,
        max_acceleration=4.0,
    )


def _module(monkeypatch: pytest.MonkeyPatch, module_factory):
    RecordingGenerator.calls = []
    RecordingGenerator.limits = None
    monkeypatch.setattr(
        "dimos.manipulation.planning.trajectory_generator."
        "simple_parametrizer.JointTrajectoryGenerator",
        RecordingGenerator,
    )
    model = _model()
    module = module_factory()
    module._world_monitor = MagicMock()
    module._world_monitor.world = MagicMock()
    module._world_monitor.world.get_model_config.return_value = model
    module._world_monitor.planning_groups = PlanningGroupRegistry([model])
    module._planner = MagicMock()
    module._trajectory_parametrizer = SimpleTrapezoidParametrizer(
        SimpleTrapezoidParametrizationConfig()
    )
    module._state = ManipulationState.PLANNING
    module._planning_epoch = 1
    return module


def _path(names: list[str], first: list[float], second: list[float]) -> list[JointState]:
    return [JointState(name=names, position=first), JointState(name=names, position=second)]


def test_materializes_once_with_reordered_groups_heterogeneous_limits_and_distinct_path(
    monkeypatch,
    module_factory,
):
    module = _module(monkeypatch, module_factory)
    names = ["left/b", "left/a", "right/c"]
    path = _path(names, [0.0, 0.0, 0.0], [0.2, 0.1, 0.3])
    module._planner.plan_selected_joint_path.return_value = PlanningResult(
        status=PlanningStatus.SUCCESS, path=path
    )

    assert module._plan_selected_path(("left_arm", "right_arm"), path[0], path[-1], 1, 1.0)
    assert RecordingGenerator.calls == [[[0.0, 0.0, 0.0], [0.2, 0.1, 0.3]]]
    assert RecordingGenerator.limits == ([3.0, 3.0, 3.0], [4.0, 4.0, 4.0])
    assert module._last_plan is not None
    assert module._last_plan.path is not module._last_plan.trajectory.points
    assert module._last_plan.trajectory.joint_names == names
    assert module._last_plan.trajectory.points[-1].time_from_start == 1.0


def test_cartesian_plan_preserves_planner_timestamps_and_velocities(monkeypatch, module_factory):
    module = _module(monkeypatch, module_factory)
    module._state = ManipulationState.IDLE
    names = ["left/b", "left/a"]
    start = JointState(name=names, position=[0.0, 0.0])
    path = [
        JointState(name=names, position=[0.0, 0.0], velocity=[0.0, 0.0]),
        JointState(name=names, position=[0.2, 0.1], velocity=[0.4, 0.2]),
    ]
    module._world_monitor.current_model_joint_state.return_value = start
    module._planner.plan_cartesian_path.return_value = PlanningResult(
        status=PlanningStatus.SUCCESS,
        path=path,
        timestamps=[0.0, 0.25],
    )

    plan = module.generate_cartesian_plan(
        {
            "left_arm": (
                Transform.identity(),
                Transform(translation=Vector3(0.01, 0.0, 0.0)),
            )
        },
        RoboPlanCartesianPathConfig(
            velocity_scale=0.8,
            acceleration_scale=0.6,
        ),
        speed_scale=0.5,
    )

    assert plan is not None
    assert [point.time_from_start for point in plan.trajectory.points] == [0.0, 0.25]
    assert [point.velocities for point in plan.trajectory.points] == [
        [0.0, 0.0],
        [0.4, 0.2],
    ]
    assert RecordingGenerator.calls == []
    request = module._planner.plan_cartesian_path.call_args.kwargs
    assert request["start"].name == names
    assert request["auxiliary_groups"] == ()
    assert request["config"].velocity_scale == 0.4
    assert request["config"].acceleration_scale == 0.3


def test_zero_generation_after_caching_for_status_and_completion(monkeypatch, module_factory):
    module = _module(monkeypatch, module_factory)
    names = ["left/b", "left/a"]
    path = _path(names, [0.0, 0.0], [1.0, 1.0])
    module._planner.plan_selected_joint_path.return_value = PlanningResult(
        status=PlanningStatus.SUCCESS, path=path
    )
    assert module._plan_selected_path(("left_arm",), path[0], path[-1], 1, 1.0)
    RecordingGenerator.calls = []

    module.get_state()
    module.has_planned_path()
    assert RecordingGenerator.calls == []
