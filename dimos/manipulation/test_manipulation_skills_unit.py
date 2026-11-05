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

"""What the recovery skills report back to a caller that branches on it."""

from collections.abc import Iterator
from unittest.mock import MagicMock

import pytest

from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.manipulation_spec import (
    ExecutionResult,
    ExecutionStatus,
)
from dimos.robot.manipulators.common.agent_prompts import (
    BASE_MANIPULATION_AGENT_SYSTEM_PROMPT,
    MANIPULATION_AGENT_SYSTEM_PROMPT,
)


@pytest.fixture
def skills() -> Iterator[ManipulationSkills]:
    instance = ManipulationSkills()
    instance.manipulation = MagicMock()
    yield instance
    instance.stop()


@pytest.mark.parametrize("status", [ExecutionStatus.UNCERTAIN, ExecutionStatus.FAULT])
def test_cancel_reports_an_unconfirmed_stop(
    skills: ManipulationSkills, status: ExecutionStatus
) -> None:
    """A plain "Cancelled" would invite the next motion into a still-moving arm."""
    skills.manipulation.cancel.return_value = ExecutionResult(status, "stop not confirmed")

    with pytest.raises(RuntimeError, match=f"not confirmed.*{status.name}: stop not confirmed"):
        skills.cancel()


def test_cancel_reports_a_confirmed_stop(skills: ManipulationSkills) -> None:
    skills.manipulation.cancel.return_value = ExecutionResult(ExecutionStatus.ABORTED, "Cancelled")

    assert skills.cancel().message == "Cancelled"


@pytest.mark.parametrize(
    "prompt", [BASE_MANIPULATION_AGENT_SYSTEM_PROMPT, MANIPULATION_AGENT_SYSTEM_PROMPT]
)
def test_recovery_prompts_use_available_cancel_skill(skills, prompt):
    assert "reset" not in prompt.lower()
    assert "planning failure, inspect the error, correct the target, and retry" in prompt
    assert "cancel" in prompt
    skills.manipulation.cancel.return_value = ExecutionResult(ExecutionStatus.ABORTED, "Cancelled")
    assert skills.cancel().message == "Cancelled"
