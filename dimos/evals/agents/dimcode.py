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
import tempfile
import time
from typing import Any
from uuid import uuid4

from dimos.evals.agents.lib.pi_to_atif import PiToAtif
from dimos.evals.agents.pi import PiAdapter, PiAdapterConfig
from dimos.evals.environments.base import Environment
from dimos.evals.types import EndedBy


class DimcodeAdapterConfig(PiAdapterConfig):
    cli: str = "dimcode"


class DimcodeAdapter(PiAdapter):
    """One fresh gateway and session per case, using dimcode's own agent loop.

    No personal daemon, config, credentials file or session history is reused.
    Shared case context accompanies the user instruction; dimcode retains its
    production system prompt, skills, rendering and MCP extensions.
    """

    config: DimcodeAdapterConfig

    def preflight(self, environment: Environment) -> None:
        super().preflight(environment)
        if self.config.skills or self.config.tools != PiAdapterConfig().tools:
            raise ValueError(
                "dimcode uses its production skills and tools; overrides are unsupported"
            )

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        return (*super().available_tools(environment_tools), "dimcode_render")

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
        # Unix socket paths have a small platform limit; the eval path can be long.
        env["XDG_RUNTIME_DIR"] = str(self._runtime_dir)
        return env

    def _run_pi_process(
        self, command: list[str], run_dir: Path, events: PiToAtif, timeout_s: float
    ) -> EndedBy:
        with tempfile.TemporaryDirectory(prefix="dimos-eval-") as runtime:
            self._runtime_dir = Path(runtime)
            return super()._run_pi_process(command, run_dir, events, timeout_s)

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
            with sock.makefile("rb") as incoming:

                def receive() -> dict[str, Any]:
                    sock.settimeout(max(0.001, deadline - time.monotonic()))
                    line = incoming.readline(16 * 1024 * 1024 + 1)
                    if not line or not line.endswith(b"\n"):
                        raise RuntimeError("dimcode disconnected or exceeded the protocol buffer")
                    return dict(json.loads(line))

                def call(command: dict[str, Any]) -> Generator[dict[str, Any], None, None]:
                    request_id = str(uuid4())
                    sock.sendall(
                        (json.dumps({"id": request_id, "command": command}) + "\n").encode()
                    )
                    while True:
                        packet = receive()
                        if packet["type"] == "event":
                            yield packet["event"]
                        elif packet.get("id") == request_id:
                            if packet.get("error"):
                                raise RuntimeError(packet["error"])
                            return

                yield from call({"type": "new_session", "cwd": str(run_dir)})
                yield from call(
                    {
                        "type": "set_model",
                        "provider": self.config.provider,
                        "modelId": self.config.model,
                    }
                )
                prompt_id = str(uuid4())
                sock.sendall(
                    (
                        json.dumps(
                            {
                                "id": prompt_id,
                                "command": {
                                    "type": "prompt",
                                    "message": (run_dir / "dimcode-prompt.txt").read_text(),
                                },
                            }
                        )
                        + "\n"
                    ).encode()
                )
                while True:
                    packet = receive()
                    if packet["type"] != "event":
                        if packet.get("id") == prompt_id and packet.get("error"):
                            raise RuntimeError(packet["error"])
                        continue
                    event = packet["event"]
                    if event["type"] == "turn_error":
                        yield {
                            "type": "message_end",
                            "message": {
                                "role": "assistant",
                                "stopReason": "error",
                                "errorMessage": event["message"],
                            },
                        }
                    elif event["type"] == "idle":
                        # Closing the owned gateway also closes its provider sessions.
                        # Shutdown may close this socket before replying.
                        sock.sendall(
                            (
                                json.dumps({"id": str(uuid4()), "command": {"type": "shutdown"}})
                                + "\n"
                            ).encode()
                        )
                        return
                    else:
                        yield event
