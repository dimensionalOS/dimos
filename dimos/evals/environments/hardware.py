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

"""A real robot driven through ``dimos run``, with an operator in the loop.

The operator readies the scene before each case and confirms it; the code never
moves the robot to reset it. When a case ends the robot is halted in place, then
dimos stops the way ``dimos stop`` does, so each module's stop() runs (a Go2 lies down).
"""

from __future__ import annotations

import sys
from typing import TYPE_CHECKING

from pydantic import Field

from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.evals.environments.live import LiveEnvironment, LiveEnvironmentConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Sequence

    from dimos.e2e_tests.dimos_cli_call import DimosCliCall
    from dimos.evals.agents.base import EvalSubject
    from dimos.evals.types import RunningEnvironment

logger = setup_logger()


class HardwareEnvironmentConfig(LiveEnvironmentConfig):
    task: str
    """The case's instruction, shown to the operator before the robot starts."""
    stop_tool: str | None = None
    """MCP tool that halts motion before the robot disconnects, e.g. ``cancel``."""
    module_env: dict[str, str] = Field(default_factory=dict)
    launch_timeout_s: float = 300.0
    graceful_stop: bool = True


class HardwareEnvironment(LiveEnvironment):
    """A real robot; the operator resets the scene and confirms it is ready."""

    config: HardwareEnvironmentConfig

    def preflight(self, agent: EvalSubject) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("hardware evals need an operator at an interactive terminal")
        super().preflight(agent)

    def start(self, modules: Sequence[str]) -> RunningEnvironment:
        answer = input(
            f"\nNext eval task: {self.config.task}\n"
            "Is the scene reset and the robot ready, with the area clear? [y/N] "
        )
        if not answer.strip().lower().startswith("y"):
            raise RuntimeError("operator did not confirm the robot is ready")
        return super().start(modules)

    def configure_launch(self, proc: DimosCliCall) -> None:
        proc.simulator = None  # the real robot: no --simulation
        proc.extra_env.update(self.config.module_env)

    def stop(self) -> None:
        try:
            if self.config.stop_tool is not None and self._mcp_url is not None:
                try:
                    result = McpAdapter(self._mcp_url).call_tool(self.config.stop_tool)
                    # A tool that raises comes back as an error result, not an exception.
                    if result.get("isError"):
                        raise RuntimeError(f"{self.config.stop_tool}: {result.get('content')}")
                except Exception as e:
                    logger.error(
                        "HALT FAILED: the robot may still be moving, use the e-stop",
                        tool=self.config.stop_tool,
                        error=e,
                    )
        finally:
            super().stop()
