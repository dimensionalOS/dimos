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

"""What a module needs from the coordinator to ask for the operator hold."""

from __future__ import annotations

from typing import Any, Protocol

from dimos.spec.utils import Spec


class OperatorHoldSpec(Spec, Protocol):
    """A coordinator that runs the "operator_hold" task, e.g. ControlCoordinator."""

    def task_invoke(self, task_name: str, method: str, kwargs: dict[str, Any] | None = None) -> Any:
        """Run one command on a coordinator task and return what it returns.

        Args:
            task_name: Task to address, e.g. "operator_hold".
            method: Command to run, e.g. "request".
            kwargs: The command's arguments by name; None when it takes none.

        Returns None when the coordinator has no task by that name.
        """
        ...
