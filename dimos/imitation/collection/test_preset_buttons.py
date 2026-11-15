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

"""What the left controller's X and Y buttons ask the arm planner to do."""

from collections.abc import Iterator
import threading
from typing import Any
from unittest.mock import Mock

import pytest

from dimos.imitation.collection.preset_buttons import PresetButtonsModule
from dimos.manipulation.manipulation_spec import (
    ExecutionResult,
    ExecutionStatus,
    ManipulationSnapshot,
    ManipulationSpec,
    OperationStatus,
    PlanningGroupInfo,
    PlanningGroupState,
    PlanResult,
    PlanStatus,
)
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.teleop.webxr.controller_types import Buttons
from dimos.utils.testing.waiting import wait_until

INIT = JointState(name=["j1", "j2"], position=[0.1, 0.2])
HOME = JointState(name=["j1", "j2"], position=[0.0, 0.0])


def _snapshot(presets: dict[str, JointState]) -> ManipulationSnapshot:
    return ManipulationSnapshot(
        timestamp=0.0,
        operation_status=OperationStatus.IDLE,
        error=None,
        has_pending_plan=False,
        execution_status=ExecutionStatus.IDLE,
        groups={"manipulator": PlanningGroupState(None, None, None, presets)},
    )


@pytest.fixture
def planner() -> Mock:
    """Stands in for ManipulationModule: one arm, both presets known, every move completes."""
    planner = Mock(spec=ManipulationSpec)
    planner.list_planning_groups.return_value = (
        PlanningGroupInfo("manipulator", ("j1", "j2"), "base", "link_tcp", True),
    )
    planner.get_state.return_value = _snapshot({"init": INIT, "home": HOME})
    planner.plan_to_joints.return_value = PlanResult(
        PlanStatus.SUCCEEDED, plan=GeneratedPlan(("manipulator",), JointTrajectory())
    )
    planner.execute.return_value = ExecutionResult(ExecutionStatus.ACCEPTED)
    planner.wait_for_execution.return_value = ExecutionResult(ExecutionStatus.COMPLETED)
    planner.cancel.return_value = ExecutionResult(ExecutionStatus.NO_EXECUTION)
    return planner


@pytest.fixture
def module(planner: Mock) -> Iterator[PresetButtonsModule]:
    instance = PresetButtonsModule()
    instance.manipulation = planner
    yield instance
    instance.stop()


class _LongMove:
    """Makes the fake planner's move last until finish() or a cancel."""

    def __init__(self, planner: Mock) -> None:
        self.started = threading.Event()
        self._done = threading.Event()
        self._result = ExecutionResult(ExecutionStatus.COMPLETED)
        planner.wait_for_execution.side_effect = self._wait
        planner.cancel.side_effect = self._cancel

    def finish(self) -> None:
        self._done.set()

    def _wait(self, *args: Any, **kwargs: Any) -> ExecutionResult:
        self.started.set()
        self._done.wait(timeout=5.0)
        return self._result

    def _cancel(self) -> ExecutionResult:
        self._result = ExecutionResult(ExecutionStatus.ABORTED, "Cancelled")
        self._done.set()
        return self._result


def _buttons(*names: str) -> Buttons:
    """A Buttons message with the named buttons held, e.g. "left_primary" for X."""
    msg = Buttons()
    for name in names:
        msg.set_attribute(name, True)
    return msg


def _tap(module: PresetButtonsModule, *names: str) -> None:
    """Press the named buttons together, then release everything."""
    module._on_buttons(_buttons(*names))
    module._on_buttons(Buttons())


def _wait_until_idle(module: PresetButtonsModule) -> None:
    wait_until(lambda: not module._moving, timeout=5.0, interval=0.01)


def _planned_targets(planner: Mock) -> list[JointState]:
    return [call.args[0]["manipulator"] for call in planner.plan_to_joints.call_args_list]


def test_x_moves_to_the_start_pose_and_y_moves_home(
    module: PresetButtonsModule, planner: Mock
) -> None:
    _tap(module, "left_primary")
    _wait_until_idle(module)
    _tap(module, "left_secondary")
    _wait_until_idle(module)

    assert _planned_targets(planner) == [INIT, HOME]
    assert planner.execute.call_count == 2


@pytest.mark.parametrize("grip", ["left_grip", "right_grip"])
def test_x_and_y_do_nothing_while_a_grip_is_held(
    module: PresetButtonsModule, planner: Mock, grip: str
) -> None:
    _tap(module, grip, "left_primary")
    _tap(module, grip, "left_secondary")

    assert not module._moving
    planner.plan_to_joints.assert_not_called()


def test_presses_during_a_preset_move_are_ignored(
    module: PresetButtonsModule, planner: Mock
) -> None:
    move = _LongMove(planner)
    _tap(module, "left_primary")
    assert move.started.wait(timeout=5.0)

    _tap(module, "left_secondary")
    move.finish()
    _wait_until_idle(module)

    assert _planned_targets(planner) == [INIT]


def test_a_grip_during_a_preset_move_cancels_it(module: PresetButtonsModule, planner: Mock) -> None:
    move = _LongMove(planner)
    _tap(module, "left_primary")
    assert move.started.wait(timeout=5.0)

    module._on_buttons(_buttons("right_grip"))
    module._on_buttons(_buttons("right_grip"))
    _wait_until_idle(module)

    planner.cancel.assert_called_once()


def test_a_grip_while_the_move_is_planning_sends_no_motion(
    module: PresetButtonsModule, planner: Mock
) -> None:
    planning = threading.Event()
    release = threading.Event()
    plan = planner.plan_to_joints.return_value

    def slow_plan(*args: Any, **kwargs: Any) -> PlanResult:
        planning.set()
        release.wait(timeout=5.0)
        return plan

    planner.plan_to_joints.side_effect = slow_plan
    _tap(module, "left_primary")
    assert planning.wait(timeout=5.0)

    module._on_buttons(_buttons("left_grip"))
    release.set()
    _wait_until_idle(module)

    planner.execute.assert_not_called()


def test_stop_cancels_a_running_preset_move(module: PresetButtonsModule, planner: Mock) -> None:
    move = _LongMove(planner)
    _tap(module, "left_primary")
    assert move.started.wait(timeout=5.0)

    module.stop()
    _tap(module, "left_secondary")

    planner.cancel.assert_called_once()
    assert not module._moving
    assert _planned_targets(planner) == [INIT]


def test_x_before_the_start_pose_is_known_moves_nothing(
    module: PresetButtonsModule, planner: Mock
) -> None:
    planner.get_state.return_value = _snapshot({"home": HOME})

    _tap(module, "left_primary")
    _wait_until_idle(module)

    planner.plan_to_joints.assert_not_called()
    planner.execute.assert_not_called()
