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
import json
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time
from typing import Any, ClassVar
from uuid import uuid4

from dimos.evals.agents.pi import PiAdapter, PiAdapterConfig, read_pi_events


class DimcodeAdapterConfig(PiAdapterConfig):
    cli: str = "dimcode"


class DimcodeAdapter(PiAdapter):
    """One fresh gateway and session per case, using dimcode's own agent loop.

    No personal daemon, config, credentials file or session history is reused.
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

    def _write_system_prompt(self, files: dict[str, Path], mcp_url: str, run_dir: Path) -> str:
        prompt = super()._write_system_prompt(files, mcp_url, run_dir)
        config = {
            "workspace": str(run_dir),
            "python": sys.executable,
            "dimos": shutil.which("dimos"),
            "mcp": [{"name": "eval", "url": mcp_url}] if mcp_url else [],
        }
        if config["dimos"] is None:
            del config["dimos"]
        agent_dir = run_dir / ".pi-agent"
        (agent_dir / "config.json").write_text(json.dumps(config))
        (agent_dir / "settings.json").write_text(
            json.dumps(
                {
                    "defaultProvider": self.config.provider,
                    "defaultModel": self.config.model,
                    "defaultThinkingLevel": self.config.thinking,
                }
            )
        )
        return prompt

    def _build_pi_command(self, inputs: str, system_prompt: str, run_dir: Path) -> list[str]:
        (run_dir / "dimcode-prompt.txt").write_text(system_prompt + "\n\n" + inputs)
        return [self.config.cli, "gateway"]

    def _build_process_env(self, run_dir: Path) -> dict[str, str]:
        env = super()._build_process_env(run_dir)
        env["DIMCODE_HOME"] = str(run_dir / ".pi-agent")
        return env

    def _process_events(
        self, proc: subprocess.Popen[bytes], run_dir: Path, deadline: float
    ) -> Generator[dict[str, Any], None, None]:
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
                [
                    {"type": "new_session", "cwd": str(run_dir)},
                    {
                        "type": "set_model",
                        "provider": self.config.provider,
                        "modelId": self.config.model,
                    },
                    {"type": "prompt", "message": (run_dir / "dimcode-prompt.txt").read_text()},
                ]
            )

            def send(command: dict[str, Any]) -> str:
                request_id = str(uuid4())
                sock.sendall((json.dumps({"id": request_id, "command": command}) + "\n").encode())
                return request_id

            request_id = send(next(commands))
            with sock.makefile("rb", buffering=0) as incoming:
                for packet in read_pi_events(incoming, deadline):
                    if packet["type"] == "response" and packet.get("id") == request_id:
                        if packet.get("error"):
                            raise RuntimeError(packet["error"])
                        command = next(commands, None)
                        if command is not None:
                            if command["type"] == "prompt":
                                self._check_tools_ready(run_dir)
                            request_id = send(command)
                    elif packet["type"] == "event":
                        event = packet["event"]
                        if event["type"] == "turn_error":
                            raise RuntimeError(event["message"])
                        if event["type"] == "idle":
                            send({"type": "shutdown"})
                            return
                        yield event
            raise RuntimeError("dimcode disconnected before completion")
