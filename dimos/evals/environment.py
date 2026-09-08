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

"""The lifecycle shared by eval environments."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence
from contextlib import ExitStack
from typing import TYPE_CHECKING, Any

from dimos.protocol.service.spec import Configurable

if TYPE_CHECKING:
    from dimos.evals.agent import Agent
    from dimos.evals.types import RunningEnvironment


class Environment(Configurable, ABC):
    """What exists for a case, including ownership of the resources it starts."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._resources = ExitStack()

    @property
    def has_robot(self) -> bool:
        """Whether this environment supplies an MCP server without agent modules."""
        return False

    def preflight(self, agent: Agent) -> None:
        """Check compatibility before any environment starts."""
        return None

    @abstractmethod
    def start(self, modules: Sequence[str]) -> RunningEnvironment:
        """Start the environment with the agent's additional blueprint names."""

    def settle(self, budget_s: float) -> None:
        """Wait for ongoing actions to finish, within the remaining case budget."""
        return None

    def stop(self) -> None:
        """Release every acquired resource, including after a failed start."""
        self._resources.close()
