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

from collections.abc import Iterator
from typing import cast
from unittest.mock import Mock

import pytest
from pytest_mock import MockerFixture

from dimos.imitation.policy.lerobot.module import RolloutStatus
from dimos.imitation.policy.rollout_supervisor import (
    POLICY_TASK_NAMES,
    PolicyControlSpec,
    PolicyRolloutSpec,
    PolicyRolloutSupervisor,
    _Request,
)


def _status(*, active: bool, error: str | None = None) -> RolloutStatus:
    return {
        "active": active,
        "policy_path": "checkpoint",
        "task": "pick",
        "device": "cpu",
        "observations_ready": True,
        "commands_published": 0,
        "last_error": error,
    }


@pytest.fixture
def supervisor(
    mocker: MockerFixture,
) -> Iterator[tuple[PolicyRolloutSupervisor, Mock, Mock]]:
    module = PolicyRolloutSupervisor()
    policy = mocker.Mock(spec=PolicyRolloutSpec)
    policy.start_rollout.return_value = _status(active=True)
    policy.stop_rollout.return_value = _status(active=False)
    control = mocker.Mock(spec=PolicyControlSpec)
    control.task_invoke.return_value = True
    module._policy = cast("PolicyRolloutSpec", policy)
    module._control = cast("PolicyControlSpec", control)
    yield module, policy, control
    module.stop()


def test_rpc_request_only_enqueues_work(
    supervisor: tuple[PolicyRolloutSupervisor, Mock, Mock],
) -> None:
    module, policy, control = supervisor

    module.request_rollout_start()

    assert module._requests.get_nowait() is _Request.START
    policy.start_rollout.assert_not_called()
    control.task_invoke.assert_not_called()


def test_start_activates_all_tasks_before_policy(
    supervisor: tuple[PolicyRolloutSupervisor, Mock, Mock],
) -> None:
    module, policy, control = supervisor

    module._start_rollout()

    assert [call.args for call in control.task_invoke.call_args_list] == [
        (task_name, "activate") for task_name in POLICY_TASK_NAMES
    ]
    policy.start_rollout.assert_called_once_with()
    assert module._rollout_active


def test_manual_override_stops_policy_and_deactivates_tasks(
    supervisor: tuple[PolicyRolloutSupervisor, Mock, Mock],
) -> None:
    module, policy, control = supervisor
    module._rollout_active = True

    module._handle(_Request.OVERRIDE_STARTED)

    policy.stop_rollout.assert_called_once_with()
    assert [call.args for call in control.task_invoke.call_args_list] == [
        (task_name, "deactivate") for task_name in POLICY_TASK_NAMES
    ]
    assert module._override_active
    assert not module._rollout_active


def test_override_blocks_start(
    supervisor: tuple[PolicyRolloutSupervisor, Mock, Mock],
) -> None:
    module, policy, control = supervisor
    module._override_active = True

    module._start_rollout()

    policy.start_rollout.assert_not_called()
    control.task_invoke.assert_not_called()


def test_partial_activation_failure_rolls_back(
    supervisor: tuple[PolicyRolloutSupervisor, Mock, Mock],
) -> None:
    module, policy, control = supervisor
    control.task_invoke.side_effect = [True, False, True]

    with pytest.raises(RuntimeError, match="failed to activate"):
        module._start_rollout()

    assert [call.args for call in control.task_invoke.call_args_list] == [
        (POLICY_TASK_NAMES[0], "activate"),
        (POLICY_TASK_NAMES[1], "activate"),
        (POLICY_TASK_NAMES[0], "deactivate"),
    ]
    policy.start_rollout.assert_not_called()
