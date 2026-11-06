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

"""Skills for working with a human operator."""

from __future__ import annotations

from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.control.tasks.operator_hold_task.operator_hold_spec import OperatorHoldSpec
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OPERATOR_HOLD_TASK_NAME,
    HoldRoute,
    OperatorHoldStatus,
)
from dimos.core.core import rpc
from dimos.core.module import Module


class TeleopSkills(Module):
    """Skills for working with a human operator.

    ``request_operator`` stops the robot and asks a person for help. Only the
    operator can release the hold, so no skill releases it.
    """

    _coordinator: OperatorHoldSpec

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill
    def request_operator(self, reason: str) -> SkillResult:
        """Stop the robot and wait for a person to help.

        Every joint stops where it is and stays there until a human operator
        releases it. You cannot release it yourself. Use it when you are stuck,
        unsure it is safe to go on, or need a person to fix something.

        Args:
            reason: Why you need a person, in plain words. The operator sees
                this text, e.g. "The box slipped out of the gripper".
        """
        status = self._coordinator.task_invoke(
            OPERATOR_HOLD_TASK_NAME,
            "request",
            {"route": HoldRoute.AGENT.value, "reason": reason},
        )
        if not isinstance(status, OperatorHoldStatus):
            raise RuntimeError("This robot has no operator hold task; nothing was stopped")
        return SkillResult(
            "Operator hold is on: the robot is stopped until a person releases it. "
            f"The operator sees: {status.reason}"
        )
