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

from collections.abc import Iterator
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from dimos.manipulation._test_manipulation_helpers import (
    close_test_runtimes,
    install_runtime,
    make_module,
)
from dimos.manipulation.execution_runtime import PreparedPlan
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import PlanningResult
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


class RecordingGenerator:
    calls: list[list[list[float]]] = []
    limits: tuple[list[float], list[float]] | None = None
    fail = False

    def __init__(
        self, num_joints: int, max_velocity: list[float], max_acceleration: list[float]
    ) -> None:
        self.num_joints = num_joints
        RecordingGenerator.limits = (list(max_velocity), list(max_acceleration))

    def generate(self, waypoints: list[list[float]]) -> JointTrajectory:
        RecordingGenerator.calls.append(waypoints)
        if RecordingGenerator.fail:
            raise RuntimeError("boom")
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


def _robot(name: str, joints: list[str], velocity: float, acceleration: float) -> RobotModelConfig:
    return RobotModelConfig(
        name=name,
        model_path=Path("/robot.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=joints,
        base_link="base",
        planning_groups=[
            PlanningGroupDefinition(
                name="group", joint_names=tuple(reversed(joints)), base_link="base", tip_link="tip"
            )
        ],
        max_velocity=velocity,
        max_acceleration=acceleration,
        coordinator_task_name=f"task_{name}",
    )


@pytest.fixture(autouse=True)
def _close_runtime() -> Iterator[None]:
    yield
    close_test_runtimes()


def _module(monkeypatch: pytest.MonkeyPatch):
    RecordingGenerator.calls = []
    RecordingGenerator.limits = None
    RecordingGenerator.fail = False
    monkeypatch.setattr(
        "dimos.manipulation.manipulation_module.JointTrajectoryGenerator", RecordingGenerator
    )
    left = _robot("left", ["a", "b"], 1.0, 2.0)
    right = _robot("right", ["c"], 3.0, 4.0)
    module = make_module()
    module._robots = {
        "left": ("left_id", left, MagicMock()),
        "right": ("right_id", right, MagicMock()),
    }
    module._world_monitor = MagicMock()
    module._world_monitor.world = MagicMock()
    module._world_monitor.planning_groups = PlanningGroupRegistry([left, right])
    module._planner = MagicMock()
    install_runtime(module, [left, right])
    return module


def _path(names: list[str], first: list[float], second: list[float]) -> list[JointState]:
    return [JointState(name=names, position=first), JointState(name=names, position=second)]


def test_materializes_once_with_reordered_groups_heterogeneous_limits_and_distinct_path(
    monkeypatch,
):
    module = _module(monkeypatch)
    names = ["left/b", "left/a", "right/c"]
    path = _path(names, [0.0, 0.0, 0.0], [0.2, 0.1, 0.3])
    module._planner.plan_selected_joint_path.return_value = PlanningResult(
        status=PlanningStatus.SUCCESS, path=path
    )

    token = module._begin_group_planning()
    assert token is not None
    assert module._plan_selected_path(("left/group", "right/group"), path[0], path[-1], token)
    assert RecordingGenerator.calls == [[[0.0, 0.0, 0.0], [0.2, 0.1, 0.3]]]
    assert RecordingGenerator.limits == ([1.0, 1.0, 3.0], [2.0, 2.0, 4.0])
    ready = module._execution_runtime.snapshot().ready_plan
    assert isinstance(ready, PreparedPlan)
    assert ready.generated_plan.path is not ready.generated_plan.trajectory.points
    assert ready.generated_plan.trajectory.joint_names == names
    assert ready.generated_plan.trajectory.points[-1].time_from_start == 1.0


@pytest.mark.parametrize(
    ("path", "message"),
    [
        (_path(["left/a", "left/b"], [0.0, 0.0], [1.0, 1.0]), "joint names"),
        (_path(["left/b", "left/a"], [0.0], [1.0]), "dimension"),
        (_path(["left/b", "left/a"], [0.0, float("nan")], [1.0, 1.0]), "non-finite"),
    ],
)
def test_rejects_malformed_or_nonfinite_waypoints(monkeypatch, path, message):
    module = _module(monkeypatch)
    module._planner.plan_selected_joint_path.return_value = PlanningResult(
        status=PlanningStatus.SUCCESS, path=path
    )

    token = module._begin_group_planning()
    assert token is not None
    assert not module._plan_selected_path(("left/group",), path[0], path[-1], token)
    assert module._execution_runtime.snapshot().ready_plan is None
    assert message in (module.get_error() or "")


def test_rejects_invalid_limits_and_generator_failure_without_caching(monkeypatch):
    module = _module(monkeypatch)
    module._robots["left"][1].max_velocity = 0.0
    names = ["left/b", "left/a"]
    path = _path(names, [0.0, 0.0], [1.0, 1.0])
    module._planner.plan_selected_joint_path.return_value = PlanningResult(
        status=PlanningStatus.SUCCESS, path=path
    )

    token = module._begin_group_planning()
    assert token is not None
    assert not module._plan_selected_path(("left/group",), path[0], path[-1], token)
    assert module._execution_runtime.snapshot().ready_plan is None
    assert RecordingGenerator.calls == []

    module = _module(monkeypatch)
    RecordingGenerator.fail = True
    module._planner.plan_selected_joint_path.return_value = PlanningResult(
        status=PlanningStatus.SUCCESS, path=path
    )
    token = module._begin_group_planning()
    assert token is not None
    assert not module._plan_selected_path(("left/group",), path[0], path[-1], token)
    assert module._execution_runtime.snapshot().ready_plan is None
    assert len(RecordingGenerator.calls) == 1


def test_zero_generation_after_caching_for_status_and_completion(monkeypatch):
    module = _module(monkeypatch)
    names = ["left/b", "left/a"]
    path = _path(names, [0.0, 0.0], [1.0, 1.0])
    module._planner.plan_selected_joint_path.return_value = PlanningResult(
        status=PlanningStatus.SUCCESS, path=path
    )
    token = module._begin_group_planning()
    assert token is not None
    assert module._plan_selected_path(("left/group",), path[0], path[-1], token)
    RecordingGenerator.calls = []

    assert module.get_trajectory_status() is not None
    assert RecordingGenerator.calls == []
