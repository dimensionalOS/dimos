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

"""Tests for the Quest-triggered OpenArm homing module."""

from __future__ import annotations

from collections.abc import Iterator
from unittest.mock import MagicMock

import pytest
import pytest_mock

from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.protocol.rpc.pubsubrpc import LCMRPC
from dimos.robot.manipulators.openarm.config import OPENARM_ARM_JOINTS
from dimos.robot.manipulators.openarm.homing_module import (
    OPENARM_HOME_POSITIONS,
    OpenArmHomingModule,
)
from dimos.teleop.quest.quest_types import Buttons

RIGHT_THUMBSTICK = 1 << Buttons.BITS["right_thumbstick"]
LEFT_PRIMARY = 1 << Buttons.BITS["left_primary"]
RIGHT_PRIMARY = 1 << Buttons.BITS["right_primary"]


@pytest.fixture
def module(mocker: pytest_mock.MockerFixture) -> Iterator[OpenArmHomingModule]:
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.MagicMock(), None))
    mocker.patch.object(LCMRPC, "__init__", return_value=None)
    mocker.patch.object(LCMRPC, "serve_module_rpc", return_value=None)
    mocker.patch.object(LCMRPC, "start", return_value=None)
    mocker.patch.object(LCMRPC, "stop", return_value=None)
    homing = OpenArmHomingModule()
    coordinator = MagicMock()
    coordinator.get_joint_positions.return_value = {name: 0.5 for name in OPENARM_ARM_JOINTS}
    coordinator.execute_trajectory.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.ACCEPTED
    )
    homing._control_coordinator = coordinator
    yield homing
    homing.stop()


def _buttons(bits: int) -> Buttons:
    msg = Buttons()
    msg.data = bits
    return msg


def test_home_pose_covers_all_arm_joints() -> None:
    assert set(OPENARM_HOME_POSITIONS) == set(OPENARM_ARM_JOINTS)
    assert OPENARM_HOME_POSITIONS["openarm_left_joint4"] == 1.554
    assert OPENARM_HOME_POSITIONS["openarm_right_joint4"] == 1.564


def test_button_press_executes_homing_trajectory(module: OpenArmHomingModule) -> None:
    module._on_buttons(_buttons(RIGHT_THUMBSTICK))

    coordinator = module._control_coordinator
    coordinator.execute_trajectory.assert_called_once()
    trajectory = coordinator.execute_trajectory.call_args.args[0]
    assert list(trajectory.joint_names) == list(OPENARM_ARM_JOINTS)
    assert len(trajectory.points) >= 2
    assert trajectory.points[0].positions == [0.5] * len(OPENARM_ARM_JOINTS)
    assert trajectory.points[-1].positions == [
        OPENARM_HOME_POSITIONS[name] for name in OPENARM_ARM_JOINTS
    ]
    assert trajectory.points[-1].time_from_start >= 5.0
    times = [p.time_from_start for p in trajectory.points]
    assert times == sorted(times)


def test_press_is_edge_triggered_not_level_triggered(module: OpenArmHomingModule) -> None:
    module._on_buttons(_buttons(RIGHT_THUMBSTICK))
    module._on_buttons(_buttons(RIGHT_THUMBSTICK))
    module._on_buttons(_buttons(0))
    module._on_buttons(_buttons(RIGHT_THUMBSTICK))

    assert module._control_coordinator.execute_trajectory.call_count == 2


@pytest.mark.parametrize("deadman", [LEFT_PRIMARY, RIGHT_PRIMARY, LEFT_PRIMARY | RIGHT_PRIMARY])
def test_press_while_deadman_held_is_ignored(module: OpenArmHomingModule, deadman: int) -> None:
    module._on_buttons(_buttons(RIGHT_THUMBSTICK | deadman))

    module._control_coordinator.execute_trajectory.assert_not_called()


def test_missing_joint_positions_do_not_execute(module: OpenArmHomingModule) -> None:
    module._control_coordinator.get_joint_positions.return_value = {"openarm_left_joint1": 0.0}

    assert module.go_home() is False
    module._control_coordinator.execute_trajectory.assert_not_called()


def test_duration_scales_with_distance_and_caps(module: OpenArmHomingModule) -> None:
    module._control_coordinator.get_joint_positions.return_value = {
        name: 6.0 for name in OPENARM_ARM_JOINTS
    }

    module.go_home()

    trajectory = module._control_coordinator.execute_trajectory.call_args.args[0]
    assert trajectory.points[-1].time_from_start == pytest.approx(15.0)


def test_rejected_execution_returns_false(module: OpenArmHomingModule) -> None:
    module._control_coordinator.execute_trajectory.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.START_STATE_MISMATCH, "moved"
    )

    assert module.go_home() is False
