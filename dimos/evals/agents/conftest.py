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

"""Shared local-provider and native-harness fixtures; no paid inference."""

from collections.abc import Iterator
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import shutil
import threading
from typing import Any

from pydantic import BaseModel, JsonValue
import pytest

from dimos.evals.agents.dimcode import DimcodeAdapter
from dimos.evals.agents.lib import pi_config
from dimos.evals.agents.lib.pi_config import Provider
from dimos.evals.agents.pi import PiAdapter
from dimos.evals.types import RunningEnvironment, ToolCall, Trajectory
from dimos.memory.store.sqlite import SqliteStore


class ProviderRequest(BaseModel):
    model: str
    tools: tuple[dict[str, JsonValue], ...] = ()
    input: tuple[dict[str, JsonValue], ...] = ()
    messages: tuple[dict[str, JsonValue], ...] = ()
    max_output_tokens: int | None = None
    max_completion_tokens: int | None = None
    max_tokens: int | None = None

    @property
    def tool_names(self) -> set[str]:
        return {str(tool["name"]) for tool in self.tools}

    @property
    def outputs(self) -> str:
        outputs = [
            item.get("output") for item in self.input if item.get("type") == "function_call_output"
        ]
        for message in self.messages:
            content = message.get("content")
            if isinstance(content, list):
                outputs.extend(
                    block.get("content")
                    for block in content
                    if isinstance(block, dict) and block.get("type") == "tool_result"
                )
        return json.dumps(outputs)


@dataclass
class ScriptedProvider:
    name: Provider
    requests: list[ProviderRequest] = field(default_factory=list)
    routes: list[str] = field(default_factory=list)
    calls: list[ToolCall] = field(default_factory=list)

    @property
    def model(self) -> str:
        return "gpt-6-astra" if self.name == "openai" else "claude-fable-5-1"

    def call(self, name: str, **arguments: JsonValue) -> None:
        self.calls.append(
            ToolCall(
                tool_call_id=f"call_{len(self.calls)}", function_name=name, arguments=arguments
            )
        )

    def response(self) -> bytes:
        index = len(self.requests) - 1
        call = self.calls[index] if index < len(self.calls) else None
        events: list[dict[str, JsonValue]]
        if self.name == "openai":
            item: dict[str, JsonValue] = (
                dict(
                    type="function_call",
                    id=f"fc_{index}",
                    call_id=call.tool_call_id,
                    name=call.function_name,
                    arguments=json.dumps(call.arguments),
                )
                if call
                else dict(
                    type="message",
                    id=f"out_{index}",
                    role="assistant",
                    content=[dict(type="output_text", text="OK", annotations=[])],
                )
            )
            response: dict[str, JsonValue] = dict(
                id=f"resp_{index}",
                status="completed",
                output=[item],
                model=self.model,
                usage=dict(input_tokens=10, output_tokens=5),
            )
            events = [
                dict(type="response.created", response=dict(id=response["id"])),
                dict(type="response.output_item.added", output_index=0, item=item),
                dict(type="response.output_item.done", output_index=0, item=item),
                dict(type="response.completed", response=response),
            ]
        else:
            block: dict[str, JsonValue] = (
                dict(type="tool_use", id=call.tool_call_id, name=call.function_name, input={})
                if call
                else dict(type="text", text="")
            )
            delta: dict[str, JsonValue] = (
                dict(type="input_json_delta", partial_json=json.dumps(call.arguments))
                if call
                else dict(type="text_delta", text="OK")
            )
            events = [
                dict(
                    type="message_start",
                    message=dict(
                        id=f"msg_{index}",
                        type="message",
                        role="assistant",
                        model=self.model,
                        content=[],
                        stop_reason=None,
                        stop_sequence=None,
                        usage=dict(input_tokens=10, output_tokens=0),
                    ),
                ),
                dict(type="content_block_start", index=0, content_block=block),
                dict(type="content_block_delta", index=0, delta=delta),
                dict(type="content_block_stop", index=0),
                dict(
                    type="message_delta",
                    delta=dict(stop_reason="tool_use" if call else "end_turn", stop_sequence=None),
                    usage=dict(output_tokens=5),
                ),
                dict(type="message_stop"),
            ]
        return "".join(
            f"event: {event['type']}\ndata: {json.dumps(event)}\n\n" for event in events
        ).encode()


@pytest.fixture(autouse=True)
def isolated_agent_paths(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(pi_config, "CONFIG_DIR", tmp_path / "config")
    monkeypatch.setattr(pi_config, "CACHE_DIR", tmp_path / "cache")
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    monkeypatch.setenv("PI_OFFLINE", "1")
    monkeypatch.delenv("PI_CODING_AGENT_DIR", raising=False)


@pytest.fixture(params=["openai", "anthropic"])
def provider(
    request: pytest.FixtureRequest, monkeypatch: pytest.MonkeyPatch
) -> Iterator[ScriptedProvider]:
    backend: Provider = "openai" if request.param == "openai" else "anthropic"
    scripted = ScriptedProvider(backend)

    class Server(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: Any) -> None:
            pass

        def do_POST(self) -> None:
            scripted.routes.append(self.path)
            scripted.requests.append(
                ProviderRequest.model_validate_json(
                    self.rfile.read(int(self.headers["Content-Length"]))
                )
            )
            payload = scripted.response()
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

    with ThreadingHTTPServer(("127.0.0.1", 0), Server) as server:
        thread = threading.Thread(target=lambda: server.serve_forever(poll_interval=0.01))
        thread.start()
        monkeypatch.setenv(f"{backend.upper()}_API_KEY", "offline-test-key")
        monkeypatch.setenv(f"{backend.upper()}_BASE_URL", f"http://127.0.0.1:{server.server_port}")
        try:
            yield scripted
        finally:
            server.shutdown()
            thread.join()


@dataclass(frozen=True)
class NativeHarness:
    adapter: type[PiAdapter]
    cli: str
    sandbox: bool
    root: Path
    environment: RunningEnvironment

    @property
    def workspace(self) -> Path:
        return self.root / "workspace" if self.sandbox else self.root

    def path(self, name: str) -> str:
        return str(Path("/workspace") / name if self.sandbox else self.root / name)

    def agent(self, provider: ScriptedProvider, allowed: tuple[str, ...] | None) -> PiAdapter:
        return self.adapter(
            cli=self.cli,
            sandbox=self.sandbox,
            allowed_tools=allowed,
            provider=provider.name,
            model=provider.model,
            max_steps=10,
            max_output_tokens=1024,
        )

    def run(self, agent: PiAdapter) -> Trajectory:
        return agent.run(
            "Inspect selected observations.", self.environment, self.root, timeout_s=10
        )


@pytest.fixture(params=["pi", "dimcode", "sandbox"])
def harness(request: pytest.FixtureRequest, tmp_path: Path) -> Iterator[NativeHarness]:
    name = str(request.param)
    cli_name = "dimcode" if name == "dimcode" else "pi"
    executable = shutil.which(os.environ.get(f"EVAL_{cli_name.upper()}_CLI", cli_name))
    if executable is None:
        pytest.skip(f"requires installed {cli_name}")
    if name == "sandbox" and not Path("/usr/bin/bwrap").exists():
        pytest.skip("requires bubblewrap")
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    with SqliteStore(path=tmp_path / "source.db") as store:
        stream = store.stream("facts", str)
        stream.append("selected-observation", ts=1)
        yield NativeHarness(
            DimcodeAdapter if name == "dimcode" else PiAdapter,
            executable,
            name == "sandbox",
            run_dir,
            RunningEnvironment(mcp_url="", streams=(stream,), artifacts={}),
        )
