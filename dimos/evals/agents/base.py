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
import os
from pathlib import Path
import re
from typing import TYPE_CHECKING, Any

from pydantic import Field, field_validator, model_validator
from typing_extensions import Self

from dimos.evals.types import RunningEnvironment, Trajectory
from dimos.protocol.service.spec import BaseConfig, Configurable

if TYPE_CHECKING:
    from dimos.evals.environments.base import Environment


NO_DIMOS_KEYWORDS = ("dimos", "dimensionalos")

NO_DIMOS_GUIDANCE = (
    "There is no robotics framework installed. You may use any public tool, library, SDK or "
    "web resource, and write whatever code you need under the current directory. Do not "
    "install, clone, import or run dimOS or anything from the dimensionalOS organisation; "
    "tool calls that mention it are denied."
)


def strip_dimos(env: dict[str, str]) -> dict[str, str]:
    """A process environment with no DIMOS_* variables and no PATH entry that ships dimOS."""
    env = {k: v for k, v in env.items() if not k.startswith("DIMOS_")}
    env["PATH"] = os.pathsep.join(
        d for d in env.get("PATH", "").split(os.pathsep) if not _ships_dimos(d)
    )
    return env


def _ships_dimos(directory: str) -> bool:
    d = Path(directory)
    return (d / "dimos").exists() or "dimos" in d.parts


class AgentConfig(BaseConfig):
    modules: tuple[str, ...] = ()
    allowed_tools: tuple[str, ...] | None = None
    # Tool calls whose arguments mention any of these (case-insensitive, whole token) are denied.
    excluded_keywords: tuple[str, ...] = ()
    # Cap on one bash call's runtime, seconds; the model's own timeout is clamped to it.
    max_tool_seconds: float | None = Field(default=300.0, gt=0)
    # Robot or data handed over without dimOS; excluded_keywords defaults to dimOS's names.
    no_dimos: bool = False

    @field_validator("excluded_keywords")
    @classmethod
    def validate_excluded_keywords(cls, words: tuple[str, ...]) -> tuple[str, ...]:
        cleaned = tuple(w.strip().lower() for w in words)
        if any(not w or not re.fullmatch(r"[a-z0-9_.-]+", w) for w in cleaned):
            raise ValueError("excluded_keywords must be nonempty words (letters, digits, _ . -)")
        return cleaned

    @model_validator(mode="after")
    def _no_dimos_defaults(self) -> Self:
        if self.no_dimos and not self.excluded_keywords:
            object.__setattr__(self, "excluded_keywords", NO_DIMOS_KEYWORDS)
        return self

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
    """The thing under evaluation, behind one interface.

    A subclass adapts one way of answering a case: a coding harness plus a model
    (Pi, dimcode), dimOS's own agent loop (the MCP client), or a single model call
    with no tools (question/answer, blind). The runner gives it the case prompt and
    a running environment and gets back a trajectory; grading happens elsewhere.

    ``config.modules`` names extra blueprints the environment launches for this
    agent; empty means only what the environment already provides.
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
