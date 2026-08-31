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

from collections.abc import Callable, Iterator
from typing import cast
from unittest.mock import Mock

import pytest
from pytest_mock import MockerFixture

from dimos.imitation.policy.lerobot.module import RolloutStatus
from dimos.imitation.policy.rollout_controller import (
    POLICY_GRIPPER_TASK_NAME,
    POLICY_ROLLOUT_TASK_NAME,
    POLICY_TASK_NAMES,
    PolicyControlSpec,
    PolicyRolloutSpec,
    QuestRolloutControllerModule,
)
from dimos.msgs.control_msgs.TaskPreemption import TaskPreemption
from dimos.teleop.quest.quest_types import Buttons


def _status(*, active: bool) -> RolloutStatus:
    return {
        "active": active,
        "policy_path": "checkpoint",
        "task": "pick",
        "device": "cpu",
        "observations_ready": True,
        "commands_published": 0,
        "last_error": None,
    }


ControllerFactory = Callable[..., tuple[QuestRolloutControllerModule, Mock, Mock]]


@pytest.fixture
def make_controller(mocker: MockerFixture) -> Iterator[ControllerFactory]:
    controllers: list[QuestRolloutControllerModule] = []

    def _make(*, active: bool = False) -> tuple[QuestRolloutControllerModule, Mock, Mock]:
        controller = QuestRolloutControllerModule()
        policy = mocker.Mock(spec=PolicyRolloutSpec)
        policy.rollout_status.return_value = _status(active=active)
        policy.start_rollout.return_value = _status(active=True)
        policy.stop_rollout.return_value = _status(active=False)
        control = mocker.Mock(spec=PolicyControlSpec)
        controller._policy = cast("PolicyRolloutSpec", policy)
        controller._control = cast("PolicyControlSpec", control)
        controllers.append(controller)
        return controller, policy, control

    yield _make
    for controller in controllers:
        controller.stop()


def test_a_rising_edges_toggle_rollout(make_controller: ControllerFactory) -> None:
    controller, policy, control = make_controller()
    pressed = Buttons()
    pressed.right_primary = True

    controller._on_buttons(pressed)
    controller._on_buttons(pressed)
    controller._on_buttons(Buttons())
    policy.rollout_status.return_value = _status(active=True)
    controller._on_buttons(pressed)

    policy.start_rollout.assert_called_once_with()
    policy.stop_rollout.assert_called_once_with()
    assert [call.args for call in control.task_invoke.call_args_list] == [
        (task_name, "cancel") for task_name in POLICY_TASK_NAMES
    ]


def test_a_does_not_start_rollout_while_arm_grip_is_held(
    make_controller: ControllerFactory,
) -> None:
    controller, policy, control = make_controller()
    buttons = Buttons()
    buttons.right_primary = True
    buttons.right_grip = True

    controller._on_buttons(buttons)

    policy.start_rollout.assert_not_called()
    policy.stop_rollout.assert_not_called()
    control.task_invoke.assert_not_called()


@pytest.mark.parametrize("preempted_task", [POLICY_ROLLOUT_TASK_NAME, POLICY_GRIPPER_TASK_NAME])
def test_policy_preemption_stops_and_releases_entire_rollout(
    make_controller: ControllerFactory,
    preempted_task: str,
) -> None:
    controller, policy, control = make_controller(active=True)

    controller._on_task_preempted(
        TaskPreemption(
            timestamp=1.0,
            preempted_task=preempted_task,
            preempting_task="teleop_openyam",
            joints=["arm/joint1"],
        )
    )

    policy.stop_rollout.assert_called_once_with()
    assert [call.args for call in control.task_invoke.call_args_list] == [
        (task_name, "cancel") for task_name in POLICY_TASK_NAMES
    ]


def test_unrelated_preemption_is_ignored(make_controller: ControllerFactory) -> None:
    controller, policy, control = make_controller(active=True)

    controller._on_task_preempted(
        TaskPreemption(
            timestamp=1.0,
            preempted_task="navigation",
            preempting_task="safety",
            joints=["base/vx"],
        )
    )

    policy.stop_rollout.assert_not_called()
    control.task_invoke.assert_not_called()
