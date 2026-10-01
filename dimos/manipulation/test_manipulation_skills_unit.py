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
    CommandResult,
    CommandStatus,
    ExecutionResult,
    ExecutionStatus,
)


@pytest.fixture
def skills() -> Iterator[ManipulationSkills]:
    instance = ManipulationSkills()
    instance.manipulation = MagicMock()
    yield instance
    instance.stop()


@pytest.mark.parametrize("status", [ExecutionStatus.UNCERTAIN, ExecutionStatus.FAULT])
def test_cancel_reports_an_unconfirmed_stop_as_a_failure(
    skills: ManipulationSkills, status: ExecutionStatus
) -> None:
    """An ok here lets an agent command its next motion into a still-moving arm."""
    skills.manipulation.cancel.return_value = ExecutionResult(status, "stop not confirmed")

    result = skills.cancel()

    assert not result.success
    assert result.error_code == "EXECUTION_FAILED"
    assert "not confirmed" in result.message


def test_cancel_reports_a_confirmed_stop_as_success(skills: ManipulationSkills) -> None:
    skills.manipulation.cancel.return_value = ExecutionResult(ExecutionStatus.ABORTED, "Cancelled")

    assert skills.cancel().success


def test_reset_surfaces_a_refused_recovery(skills: ManipulationSkills) -> None:
    skills.manipulation.reset.return_value = CommandResult(
        CommandStatus.FAILED, "stop not confirmed"
    )

    result = skills.reset()

    assert not result.success
    assert result.error_code == "EXECUTION_FAILED"
