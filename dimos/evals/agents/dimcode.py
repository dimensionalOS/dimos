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

"""Evaluate the production dimcode gateway through its local session protocol."""

from __future__ import annotations

from collections.abc import Generator
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time
from typing import ClassVar
from uuid import uuid4

from pydantic import JsonValue

from dimos.evals.agents.lib.pi_config import (
    DimcodeConfig,
    GatewayCommand,
    GatewayEvent,
    GatewayRequest,
    GatewayResponse,
    McpEndpoint,
    NewSession,
    Prompt,
    RunPaths,
    SelectModel,
    SessionSettings,
    Shutdown,
    gateway_packet,
    gateway_request,
)
from dimos.evals.agents.pi import PiAdapter, PiAdapterConfig, read_pi_events
from dimos.evals.types import RunningEnvironment


class DimcodeAdapterConfig(PiAdapterConfig):
    cli: str = "dimcode"


class DimcodeAdapter(PiAdapter):
    """One fresh gateway and session per case, using dimcode's own agent loop.

    The gateway uses a case-specific config and a new session.
    Shared case context accompanies the user instruction; dimcode retains its
    production system prompt, skills, rendering and MCP extensions.
    """

    config: DimcodeAdapterConfig

    default_tools: ClassVar[tuple[str, ...]] = (*PiAdapter.default_tools, "dimcode_render")
    # MCP extensions register tool names when the gateway creates its session.
    tool_names: ClassVar[tuple[str, ...] | None] = None
    robot_via_bash: ClassVar[bool] = False

    def validate_tools(self) -> None:
        if self.config.sandbox or self.config.skills:
            raise ValueError("dimcode uses its production environment and skills")

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        if self.config.allowed_tools is not None:
            return self.selected_tools
        return super().available_tools(environment_tools)

    def _configure(self, env: RunningEnvironment, paths: RunPaths, proxy_url: str) -> None:
        super()._configure(env, paths, proxy_url)
        dimos = shutil.which("dimos")
        config = DimcodeConfig(
            workspace=paths.workspace,
            python=Path(sys.executable),
            dimos=Path(dimos) if dimos else None,
            mcp=(McpEndpoint(name="eval", url=env.mcp_url),) if env.mcp_url else (),
        )
        settings = SessionSettings(
            default_provider=self.config.provider,
            default_model=self.config.model,
            default_thinking_level=self.config.thinking,
        )
        (paths.config / "config.json").write_text(config.model_dump_json(exclude_none=True))
        (paths.config / "settings.json").write_text(settings.model_dump_json(by_alias=True))

    def _build_pi_command(self, inputs: str, system_prompt: str, paths: RunPaths) -> list[str]:
        (paths.workspace / "dimcode-prompt.txt").write_text(system_prompt + "\n\n" + inputs)
        return [self.config.cli, "gateway"]

    def _build_process_env(self, paths: RunPaths) -> dict[str, str]:
        return {**super()._build_process_env(paths), "DIMCODE_HOME": str(paths.config)}

    def _process_events(
        self, proc: subprocess.Popen[bytes], paths: RunPaths, deadline: float
    ) -> Generator[dict[str, JsonValue], None, None]:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            while True:
                if time.monotonic() >= deadline:
                    raise TimeoutError
                if proc.poll() is not None:
                    raise RuntimeError("dimcode gateway exited during startup; see pi-stderr.txt")
                try:
                    sock.connect(str(self._runtime_dir / "dimcode.sock"))
                    break
                except (FileNotFoundError, ConnectionRefusedError):
                    time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
            commands = iter(
                (
                    NewSession(str(paths.workspace)),
                    SelectModel(self.config.provider, self.config.model),
                    Prompt((paths.workspace / "dimcode-prompt.txt").read_text()),
                )
            )

            def send(command: GatewayCommand) -> str:
                request_id = str(uuid4())
                sock.sendall(
                    gateway_request.dump_json(GatewayRequest(request_id, command), by_alias=True)
                    + b"\n"
                )
                return request_id

            request_id = send(next(commands))
            with sock.makefile("rb", buffering=0) as incoming:
                for record in read_pi_events(incoming, deadline):
                    packet = gateway_packet.validate_python(record)
                    if isinstance(packet, GatewayResponse) and packet.id == request_id:
                        if packet.error:
                            raise RuntimeError(packet.error)
                        command = next(commands, None)
                        if command is not None:
                            if isinstance(command, Prompt):
                                self._check_tools_ready(paths)
                            request_id = send(command)
                    elif isinstance(packet, GatewayEvent):
                        event = packet.event
                        if event["type"] == "turn_error":
                            raise RuntimeError(str(event.get("message", "dimcode turn failed")))
                        if event["type"] == "idle":
                            send(Shutdown())
                            return
                        yield event
            raise RuntimeError("dimcode disconnected before completion")
