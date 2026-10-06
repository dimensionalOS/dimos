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

"""An evaluation whose entire input is the case's prompt."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

from dimos.evals.environments.base import Environment
from dimos.evals.types import RunningEnvironment

if TYPE_CHECKING:
    from dimos.evals.agents.base import Agent


class Prompt(Environment):
    """No recording, robot, or tools; the case supplies the complete input."""

    def preflight(self, agent: Agent) -> None:
        if agent.config.modules:
            raise ValueError("a prompt-only environment cannot launch agent modules")
        if agent.available_tools(()):
            raise ValueError("a prompt-only environment requires an agent without tools")

    def start(self, modules: Sequence[str]) -> RunningEnvironment:
        if modules:
            raise ValueError("a prompt-only environment cannot launch agent modules")
        return RunningEnvironment(mcp_url="", streams=(), artifacts={})
