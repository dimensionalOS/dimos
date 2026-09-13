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
from typing import TYPE_CHECKING, Any

from pydantic import field_validator

from dimos.evals.types import RunningEnvironment, Trajectory
from dimos.protocol.service.spec import BaseConfig, Configurable

if TYPE_CHECKING:
    from dimos.evals.environments.base import Environment


class AgentConfig(BaseConfig):
    modules: tuple[str, ...] = ()
    allowed_tools: tuple[str, ...] | None = None

    @field_validator("allowed_tools")
    @classmethod
    def validate_allowed_tools(cls, names: tuple[str, ...] | None) -> tuple[str, ...] | None:
        if names is not None and (
            any(not name or name.strip() != name for name in names) or len(names) != len(set(names))
        ):
            raise ValueError("allowed_tools must contain unique, nonempty tool names")
        return names


class ModelAgentConfig(AgentConfig):
    model: str = "gpt-5.6-luna"


class Agent(Configurable, ABC):
    """Run an instruction independently of the case and its grader.

    ``modules`` names the blueprints the environment launches for this agent.
    An empty sequence uses only what the environment already provides.
    """

    config: AgentConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.validate_tools()

    def validate_tools(self) -> None:
        """Adapters must enforce explicit allowlists, or reject them.

        None preserves native defaults; an empty tuple disables every tool.
        Tool selection does not restrict what an allowed shell can execute.
        """
        if self.config.allowed_tools is not None:
            raise ValueError(f"{type(self).__name__} does not support allowed_tools")

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
