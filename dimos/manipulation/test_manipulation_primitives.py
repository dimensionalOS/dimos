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

"""Behavior tests for the primitive manipulation RPCs and skill adapter."""

from collections.abc import Iterator
from pathlib import Path
from unittest.mock import MagicMock

import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.manipulation_spec import (
    CommandResult,
    ExecutionResult,
    ExecutionStatus,
    ManipulationSpec,
    PlanningGroupInfo,
    PlanResult,
    PlanStatus,
)
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.assets.model import RobotModel


def _model(
    *,
    groups: tuple[PlanningGroupDefinition, ...] = (
        PlanningGroupDefinition("tool", ("j0",), "base", "tcp"),
    ),
    gripper: bool = False,
    home: list[float] | None = None,
) -> RobotModelConfig:
    joint_names = list(dict.fromkeys(name for group in groups for name in group.joint_names))
    return RobotModelConfig(
        model=RobotModel.from_file(Path("/robot.urdf")),
        joint_names=joint_names,
        base_link="base",
        home_joints=home,
        gripper_hardware_id="arm" if gripper else None,
        planning_groups=list(groups),
    )


def _plan() -> GeneratedPlan:
    names = ["j0"]
    return GeneratedPlan(
        group_ids=("tool",),
        status=PlanningStatus.SUCCESS,
        trajectory=JointTrajectory(
            joint_names=names,
            points=[
                TrajectoryPoint(positions=[0.0], time_from_start=0.0),
                TrajectoryPoint(positions=[0.1], time_from_start=1.0),
            ],
        ),
    )


def _set_groups(module, model: RobotModelConfig) -> None:
    module.config.model = model
    module._world_monitor = MagicMock()
    module._world_monitor.planning_groups = PlanningGroupRegistry((model,))


def test_move_linear_uses_world_relative_target_and_default_speed(
    module_factory,
    mocker: MockerFixture,
) -> None:
    module = module_factory()
    _set_groups(module, _model())
    generated = _plan()
    generate = mocker.patch.object(module, "generate_cartesian_plan", return_value=generated)
    execute = mocker.patch.object(
        module,
        "execute",
        return_value=ExecutionResult(ExecutionStatus.ACCEPTED),
    )

    result = module.move_linear(dx=0.02, dz=-0.01, blocking=False)

    assert result.succeeded
    targets, config = generate.call_args.args
    start, relative = targets["tool"]
    assert start == Transform.identity()
    assert relative.translation.x == pytest.approx(0.02)
    assert relative.translation.y == pytest.approx(0.0)
    assert relative.translation.z == pytest.approx(-0.01)
    assert generate.call_args.kwargs["check_collision"] is False
    assert generate.call_args.kwargs["speed_scale"] == pytest.approx(0.5)
    execute.assert_called_once_with(blocking=False, timeout=None)


def test_get_state_returns_every_group_with_presets(module_factory) -> None:
    module = module_factory()
    _set_groups(module, _model(gripper=True, home=[0.3]))
    module._init_joints = JointState(name=["j0"], position=[-0.2])
    module._world_monitor.current_group_joint_state.return_value = JointState(
        name=["j0"], position=[0.1]
    )
    module._world_monitor.get_group_ee_pose.return_value = None
    module._control_coordinator.task_invoke.return_value = [0.04]

    snapshot = module.get_state()

    group = snapshot.groups["tool"]
    assert group.joints == JointState(name=["j0"], position=[0.1])
    assert group.gripper_position == pytest.approx(0.04)
    assert group.joint_presets["home"].position == [0.3]
    assert group.joint_presets["init"].position == [-0.2]
    module._control_coordinator.task_invoke.assert_called_once_with(
        "arm_gripper", "get_normalized", {}
    )


def test_get_state_skips_gripper_telemetry_without_control_hardware(module_factory) -> None:
    module = module_factory()
    _set_groups(module, _model(gripper=False))
    module._world_monitor.current_group_joint_state.return_value = JointState(
        name=["j0"], position=[0.1]
    )
    module._world_monitor.get_group_ee_pose.return_value = None

    snapshot = module.get_state()

    assert snapshot.groups["tool"].gripper_position is None
    module._control_coordinator.task_invoke.assert_not_called()


def test_set_gripper_position_routes_normalized_command(module_factory) -> None:
    module = module_factory()
    _set_groups(module, _model(gripper=True))
    module._control_coordinator.task_invoke.return_value = True

    result = module.set_gripper_position(0.4, "tool")

    assert result.succeeded
    module._control_coordinator.task_invoke.assert_called_once_with(
        "arm_gripper", "set_normalized", {"values": [0.4]}
    )


def test_move_linear_rejects_ambiguous_default_group(module_factory) -> None:
    module = module_factory()
    _set_groups(
        module,
        _model(
            groups=(
                PlanningGroupDefinition("left", ("j0",), "base", "left_tcp"),
                PlanningGroupDefinition("right", ("j1",), "base", "right_tcp"),
            )
        ),
    )

    result = module.move_linear(dx=0.01)

    assert result.plan.status is PlanStatus.AMBIGUOUS_GROUP
    assert result.execution is None


def test_explicit_group_must_support_requested_capability(module_factory) -> None:
    module = module_factory()
    _set_groups(
        module,
        _model(groups=(PlanningGroupDefinition("tool", ("j0",), "base"),)),
    )

    pose_result = module._resolve_pose_group("tool")
    gripper_result = module._resolve_gripper_group("tool")

    assert isinstance(pose_result, CommandResult)
    assert pose_result.message == "Planning group 'tool' is not pose-capable"
    assert isinstance(gripper_result, CommandResult)
    assert gripper_result.message == "Planning group 'tool' is not gripper-capable"


def test_omitted_group_selects_unique_capable_group(module_factory) -> None:
    module = module_factory()
    _set_groups(module, _model(gripper=True))

    result = module._resolve_gripper_group(None)

    assert not isinstance(result, CommandResult)
    assert result.id == "tool"


@pytest.fixture
def skills() -> Iterator[ManipulationSkills]:
    adapter = ManipulationSkills()
    yield adapter
    adapter.stop()


def test_legacy_skill_adapter_delegates_to_primitive_rpcs(
    skills: ManipulationSkills,
    mocker: MockerFixture,
) -> None:
    manipulation = mocker.Mock(spec=ManipulationSpec)
    manipulation.list_planning_groups.return_value = (
        PlanningGroupInfo("tool", ("j0",), "base", "tcp", False),
    )
    manipulation.plan_to_joints.return_value = PlanResult(PlanStatus.SUCCEEDED, plan=_plan())
    manipulation.execute.return_value = ExecutionResult(ExecutionStatus.COMPLETED)
    skills.manipulation = manipulation

    result = skills.move_to_joints("0.25")

    assert result.is_success()
    target = manipulation.plan_to_joints.call_args.args[0]["tool"]
    assert target.name == ["j0"]
    assert target.position == [0.25]
    manipulation.execute.assert_called_once_with(blocking=True)
