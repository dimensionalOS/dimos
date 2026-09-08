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

"""Base contract for evaluation agents."""

from __future__ import annotations

from abc import ABC, abstractmethod
from pathlib import Path
from typing import TYPE_CHECKING

from dimos.evals.types import RunningEnvironment, Trajectory
from dimos.protocol.service.spec import BaseConfig, Configurable

if TYPE_CHECKING:
    from dimos.evals.environments.base import Environment


class AgentConfig(BaseConfig):
    modules: tuple[str, ...] = ()


class Agent(Configurable, ABC):
    """Run an instruction independently of the case and its grader.

    ``modules`` names the blueprints the environment launches for this agent.
    An empty sequence uses only what the environment already provides.
    """

    config: AgentConfig

    def preflight(self, environment: Environment) -> None:
        """Raise if this agent cannot use the environment, before it starts."""
        return None

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        """Tools available to this agent; direct model calls have none."""
        return ()

    @abstractmethod
    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        """Run the instruction and save its trajectory and provider payloads.

        Return only after agent work is finished. Agents that support a time
        limit return their partial trajectory marked ``timeout`` when it expires.
        """
