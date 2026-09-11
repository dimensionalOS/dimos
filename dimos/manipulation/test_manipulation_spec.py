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

"""Contract tests for agent-readable manipulation results."""

from dataclasses import fields, is_dataclass
import inspect
import pickle
from types import UnionType
from typing import get_args, get_origin, get_type_hints

from dimos.manipulation.manipulation_module import ManipulationModule
import dimos.manipulation.manipulation_spec as manipulation_spec
from dimos.manipulation.manipulation_spec import (
    CommandResult,
    ExecutionResult,
    ExecutionStatus,
    ManipulationSnapshot,
    ManipulationSpec,
    MoveResult,
    OperationStatus,
    PlanningGroupInfo,
    PlanningGroupState,
    PlanResult,
    PlanStatus,
)
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.spec.utils import spec_annotation_compliance


def _local_result_dataclasses() -> set[type[object]]:
    found: set[type[object]] = set()

    def visit(annotation: object) -> None:
        origin = get_origin(annotation)
        if origin is not None or isinstance(annotation, UnionType):
            for argument in get_args(annotation):
                visit(argument)
            return
        if not inspect.isclass(annotation):
            return
        if annotation.__module__ != manipulation_spec.__name__ or not is_dataclass(annotation):
            return
        if annotation in found:
            return
        found.add(annotation)
        hints = get_type_hints(annotation)
        for item in fields(annotation):
            visit(hints[item.name])

    for member in ManipulationSpec.__dict__.values():
        if inspect.isfunction(member):
            visit(get_type_hints(member)["return"])
    return found


def test_every_spec_result_dataclass_has_deliberate_repr() -> None:
    result_types = _local_result_dataclasses()

    assert result_types == {
        PlanningGroupInfo,
        PlanningGroupState,
        ManipulationSnapshot,
        PlanResult,
        ExecutionResult,
        CommandResult,
        MoveResult,
    }
    for result_type in result_types:
        assert "__repr__" in result_type.__dict__
        assert not result_type.__dataclass_params__.repr


def test_public_result_repr_does_not_truncate_message() -> None:
    result = PlanResult(PlanStatus.FAILED, "x" * 1_000)

    assert "x" * 1_000 in repr(result)


def test_state_repr_exposes_values_for_repl_use() -> None:
    state = PlanningGroupState(
        JointState(name=["arm/j0"], position=[0.25]),
        None,
        0.75,
        {"home": JointState(name=["arm/j0"], position=[0.0])},
    )
    snapshot = ManipulationSnapshot(
        timestamp=12.5,
        operation_status=OperationStatus.IDLE,
        error=None,
        has_pending_plan=False,
        execution_status=ExecutionStatus.IDLE,
        groups={"arm/tool": state},
    )

    text = repr(snapshot)

    assert "12.5" in text
    assert "arm/tool" in text
    assert "0.25" in text
    assert "0.75" in text
    assert "home" in text
    assert repr(pickle.loads(pickle.dumps(snapshot))) == text


def test_plan_result_repr_summarizes_trajectory_without_dumping_points() -> None:
    plan = GeneratedPlan(
        group_ids=("arm/manipulator",),
        trajectory=JointTrajectory(
            joint_names=["arm/j0"],
            points=[
                TrajectoryPoint(positions=[0.0], velocities=[0.0], time_from_start=0.0),
                TrajectoryPoint(positions=[123.456], velocities=[0.0], time_from_start=1.0),
            ],
        ),
        path=[
            JointState(name=["arm/j0"], position=[0.0]),
            JointState(name=["arm/j0"], position=[123.456]),
        ],
        status=PlanningStatus.SUCCESS,
        path_length=123.456,
        iterations=7,
    )
    result = PlanResult(PlanStatus.SUCCEEDED, plan=plan)

    text = repr(result)

    assert "waypoints=2" in text
    assert "path_length=123" in text
    assert "iterations=7" in text
    assert "TrajectoryPoint" not in text
    assert "positions" not in text


def test_manipulation_module_exposes_no_skills() -> None:
    assert not any(hasattr(method, "__skill__") for method in ManipulationModule.rpcs.values())


def test_manipulation_module_satisfies_group_native_spec() -> None:
    assert spec_annotation_compliance(ManipulationModule, ManipulationSpec)
    for name, member in ManipulationSpec.__dict__.items():
        if inspect.isfunction(member) and not name.startswith("_"):
            assert name in ManipulationModule.rpcs
    assert "plan_to_pose" not in ManipulationModule.rpcs
