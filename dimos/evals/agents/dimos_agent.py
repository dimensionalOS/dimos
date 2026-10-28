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

"""The DimOS agent (`dimos agent run`) as an eval agent."""

from __future__ import annotations

from collections.abc import Generator
from dataclasses import replace
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
from typing import Annotated, ClassVar, Literal
from urllib.parse import urlsplit

from pydantic import BaseModel, ConfigDict, Field, JsonValue, TypeAdapter
from pydantic.alias_generators import to_camel

from dimos.evals.agents.lib.pi_config import RunPaths
from dimos.evals.agents.pi import PiAdapter, PiAdapterConfig, read_pi_events
from dimos.evals.environments.base import Environment
from dimos.evals.types import RunningEnvironment


class AssistantItem(BaseModel):
    kind: Literal["assistant"]
    message: dict[str, JsonValue]


class ToolResult(BaseModel):
    model_config = ConfigDict(alias_generator=to_camel)

    tool_call_id: str
    content: list[JsonValue]


class ToolResultItem(BaseModel):
    kind: Literal["tool_result"]
    result: ToolResult


class OtherItem(BaseModel):
    kind: Literal["input", "user", "compaction", "summary", "reset"]


class ItemLine(BaseModel):
    item: Annotated[AssistantItem | ToolResultItem | OtherItem, Field(discriminator="kind")]


class Done(BaseModel):
    status: Literal["done"]


class Unanswered(BaseModel):
    status: Literal["unanswered"]
    reason: str


class OutcomeLine(BaseModel):
    outcome: Annotated[Done | Unanswered, Field(discriminator="status")]


_run_line: TypeAdapter[ItemLine | OutcomeLine] = TypeAdapter(ItemLine | OutcomeLine)


def _unserved_port() -> int:
    """A local TCP port that nothing listens on."""
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


class DimosAgentAdapterConfig(PiAdapterConfig):
    cli: str = "dimos"
    """The `dimos` executable whose `dimos agent run` answers each case."""


class DimosAgentAdapter(PiAdapter):
    """One fresh DimOS agent gateway per case, driven through `dimos agent run --json`.

    The gateway keeps its config, state, cache and socket in per-case directories, sends
    model requests to the trace proxy through models.json, and links only to the case's
    McpServer. The agent keeps its own system prompt, tools and skills; the shared case
    guidance accompanies the user instruction.
    """

    config: DimosAgentAdapterConfig

    # The harness's tools without DimOS; inventory and agent_module join once the gateway links.
    default_tools: ClassVar[tuple[str, ...]] = (
        "read", "bash", "edit", "write", "stream_get", "job",
    )  # fmt: skip
    tool_names: ClassVar[tuple[str, ...] | None] = None
    robot_via_bash: ClassVar[bool] = False

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        """The harness's tools. The gateway, not the case's McpServer, decides the rest, and the
        runner records them from the raw requests."""
        return self.default_tools

    def validate_tools(self) -> None:
        name = type(self).__name__
        if self.config.allowed_tools is not None:
            raise ValueError(f"{name} does not support allowed_tools: its tool set is fixed")
        if self.config.no_dimos:
            raise ValueError(f"{name} does not support no_dimos: the DimOS agent is dimOS")
        if self.config.skills:
            raise ValueError(f"{name} does not support skills: it reads its own skills directory")
        if self.config.excluded_keywords:
            raise ValueError(
                f"{name} does not support excluded_keywords: it has no live tool guard"
            )
        if "max_tool_seconds" in self.config.model_fields_set:
            raise ValueError(
                f"{name} does not support max_tool_seconds: its bash has no per-call time limit"
            )

    def preflight(self, environment: Environment) -> None:
        super().preflight(environment)
        # Checks Node and builds the agent when its inputs changed, before any case starts.
        version = subprocess.run(
            [shutil.which(self.config.cli) or self.config.cli, "agent", "version"],
            capture_output=True,
            text=True,
        )
        if version.returncode:
            tail = "\n".join(version.stderr.strip().splitlines()[-20:])
            raise RuntimeError(f"`{self.config.cli} agent version` failed:\n{tail}")

    def _prepare_case(self, env: RunningEnvironment, run_dir: Path) -> str:
        # The agent reaches the robot through its own tools and prompt, not an MCP tool listing.
        return super()._prepare_case(replace(env, mcp_url=""), run_dir)

    def _configure(self, env: RunningEnvironment, paths: RunPaths, proxy_url: str) -> None:
        # Without a robot, a port nothing serves keeps the gateway off any other DimOS on this host.
        mcp = urlsplit(env.mcp_url or f"http://127.0.0.1:{_unserved_port()}/mcp")
        self._mcp_env = {"MCPSERVER__HOST": str(mcp.hostname), "MCPSERVER__PORT": str(mcp.port)}
        provider: dict[str, JsonValue] = {"baseUrl": proxy_url, "apiKey": "$DIMOS_EVAL_API_KEY"}
        if self.config.max_output_tokens is not None:
            provider["modelOverrides"] = {
                self.config.model: {"maxTokens": self.config.max_output_tokens}
            }
        agent_config = paths.config / "dimos" / "agent"
        agent_config.mkdir(parents=True, exist_ok=True)
        (agent_config / "config.json").write_text(
            json.dumps(
                {
                    "model": {"provider": self.config.provider, "modelId": self.config.model},
                    "thinkingLevel": self.config.thinking,
                }
            )
        )
        (agent_config / "models.json").write_text(
            json.dumps({"providers": {self.config.provider: provider}})
        )

    def _build_pi_command(self, inputs: str, system_prompt: str, paths: RunPaths) -> list[str]:
        return [
            shutil.which(self.config.cli) or self.config.cli,
            "agent", "run", "--json", "--cwd", str(paths.workspace),
            system_prompt + "\n\n" + inputs,
        ]  # fmt: skip

    def _build_process_env(self, paths: RunPaths) -> dict[str, str]:
        return {
            **super()._build_process_env(paths),
            **self._mcp_env,
            "XDG_CONFIG_HOME": str(paths.config),
            "XDG_STATE_HOME": str(paths.workspace / "agent-state"),
            "XDG_CACHE_HOME": str(paths.cache),
            # `dimos agent` drops variables whose value it loaded from .env, the provider key included.
            "DIMOS_EVAL_API_KEY": os.environ[self._key_env],
        }

    def _process_events(
        self, proc: subprocess.Popen[bytes], paths: RunPaths, deadline: float
    ) -> Generator[dict[str, JsonValue], None, None]:
        assert proc.stdout is not None
        for record in read_pi_events(proc.stdout, deadline):
            line = _run_line.validate_python(record)
            if isinstance(line, OutcomeLine):
                if isinstance(line.outcome, Unanswered):
                    raise RuntimeError(f"dimos agent run ended unanswered: {line.outcome.reason}")
            elif isinstance(line.item, AssistantItem):
                yield {"type": "message_end", "message": line.item.message}
            elif isinstance(line.item, ToolResultItem):
                result = line.item.result
                yield {
                    "type": "tool_execution_end",
                    "toolCallId": result.tool_call_id,
                    "result": {"content": result.content},
                }
