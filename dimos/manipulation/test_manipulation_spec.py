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

from dataclasses import dataclass
import pickle

import pytest

from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import (
    AGENT_READABLE_TYPES,
    MAX_AGENT_REPR_LENGTH,
    CommandResult,
    CommandStatus,
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
    assert_agent_readable_result,
)
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.spec.utils import spec_annotation_compliance


@pytest.mark.parametrize(
    "result",
    [
        PlanningGroupInfo("arm", ("arm/j0",), "world", "tool", True),
        PlanningGroupState(JointState(name=["arm/j0"], position=[0.0]), None, 0.1),
        ManipulationSnapshot(
            timestamp=0.0,
            operation_status=OperationStatus.IDLE,
            error=None,
            has_pending_plan=False,
            execution_status=ExecutionStatus.IDLE,
            groups={},
        ),
        PlanResult(PlanStatus.FAILED, "unreachable"),
        ExecutionResult(ExecutionStatus.REJECTED, "busy"),
        CommandResult(CommandStatus.SUCCEEDED, "opened"),
        MoveResult(
            PlanResult(PlanStatus.NO_MOTION, "zero displacement"),
            None,
            (0.0, 0.0, 0.0),
            False,
        ),
    ],
)
def test_public_result_repr_is_bounded_and_shows_status(result: object) -> None:
    assert_agent_readable_result(result)

    text = repr(result)

    status = getattr(result, "status", None)
    if status is not None:
        assert status.name in text
    message = getattr(result, "message", None)
    if message:
        assert message in text
    assert len(text) <= MAX_AGENT_REPR_LENGTH
    assert repr(pickle.loads(pickle.dumps(result))) == text


def test_generated_dataclass_repr_is_rejected() -> None:
    @dataclass
    class UnboundedResult:
        status: str

    with pytest.raises(TypeError, match="generated dataclass repr"):
        assert_agent_readable_result(UnboundedResult("ok"))


def test_public_result_repr_truncates_without_hiding_typed_message() -> None:
    result = PlanResult(PlanStatus.FAILED, "x" * 1_000)

    assert len(repr(result)) == MAX_AGENT_REPR_LENGTH
    assert result.message == "x" * 1_000


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
    assert len(text) <= MAX_AGENT_REPR_LENGTH


def test_every_registered_agent_readable_type_has_deliberate_repr() -> None:
    assert AGENT_READABLE_TYPES == (
        PlanningGroupInfo,
        PlanningGroupState,
        ManipulationSnapshot,
        PlanResult,
        ExecutionResult,
        CommandResult,
        MoveResult,
    )


def test_manipulation_module_exposes_only_primitive_and_obstacle_rpcs() -> None:
    infrastructure = {
        "build",
        "get_skills",
        "peek_stream",
        "set_module_ref",
        "set_transport",
        "start",
        "stop",
    }

    assert set(ManipulationModule.rpcs) - infrastructure == {
        "add_obstacle",
        "cancel",
        "execute",
        "get_state",
        "list_planning_groups",
        "move_linear",
        "plan_to_joints",
        "plan_to_poses",
        "remove_obstacle",
        "set_gripper_position",
        "update_obstacle",
        "update_obstacle_pose",
        "wait_for_execution",
    }
    assert not any(hasattr(method, "__skill__") for method in ManipulationModule.rpcs.values())


def test_manipulation_module_satisfies_group_native_spec() -> None:
    assert spec_annotation_compliance(ManipulationModule, ManipulationSpec)
