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

"""Native Pi/dimcode calls against a local scripted provider, without API spend.

Set EVAL_PI_CLI and EVAL_DIMCODE_CLI to installed executables to run these tests.
"""

from collections.abc import Iterator
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import shutil
import threading
from typing import Any

import pytest

from dimos.evals.agents.dimcode import DimcodeAdapter
from dimos.evals.agents.pi import PiAdapter
from dimos.evals.types import RunningEnvironment
from dimos.memory.store.sqlite import SqliteStore


@pytest.fixture
def provider(monkeypatch: pytest.MonkeyPatch) -> Iterator[tuple[list[Any], list[Any]]]:
    requests: list[Any] = []
    calls: list[Any] = []

    class Server(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: Any) -> None:
            pass

        def do_POST(self) -> None:
            requests.append(json.loads(self.rfile.read(int(self.headers["Content-Length"]))))
            index = len(requests)
            item = (
                dict(
                    type="function_call",
                    id=f"fc_{index}",
                    call_id=f"call_{index}",
                    name=calls[index - 1][0],
                    arguments=json.dumps(calls[index - 1][1]),
                )
                if index <= len(calls)
                else dict(
                    type="message",
                    id=f"msg_{index}",
                    role="assistant",
                    content=[dict(type="output_text", text="OK", annotations=[])],
                )
            )
            response: dict[str, Any] = dict(
                id=f"resp_{index}",
                status="completed",
                output=[item],
                model="gpt-6-astra",
                usage=dict(input_tokens=10, output_tokens=5),
            )
            events: list[dict[str, Any]] = [
                dict(type="response.created", response=dict(id=response["id"])),
                dict(type="response.output_item.added", output_index=0, item=item),
                dict(type="response.output_item.done", output_index=0, item=item),
                dict(type="response.completed", response=response),
            ]
            payload = "".join(
                "event: " + e["type"] + "\ndata: " + json.dumps(e) + "\n\n" for e in events
            ).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

    server = ThreadingHTTPServer(("127.0.0.1", 0), Server)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    monkeypatch.setenv("OPENAI_API_KEY", "offline-test-key")
    monkeypatch.setenv("OPENAI_BASE_URL", f"http://127.0.0.1:{server.server_port}/v1")
    try:
        yield requests, calls
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def executable(adapter: type[PiAdapter]) -> str:
    cli = "dimcode" if adapter is DimcodeAdapter else "pi"
    path = shutil.which(os.environ.get(f"EVAL_{cli.upper()}_CLI", cli))
    if not path:
        pytest.skip(f"requires installed {cli}")
    return path


@pytest.mark.parametrize(
    "adapter,sandbox,allowed",
    [
        (PiAdapter, False, ("bash", "grep")),
        (PiAdapter, False, ()),
        (PiAdapter, True, ("bash", "grep")),
        (PiAdapter, True, ("grep",)),
        (DimcodeAdapter, False, ("bash", "grep")),
        (DimcodeAdapter, False, ()),
    ],
)
def test_native_tool_allowlist_enforces_execution(
    adapter: type[PiAdapter],
    sandbox: bool,
    allowed: tuple[str, ...],
    provider: tuple[list[Any], list[Any]],
    tmp_path: Path,
) -> None:
    cli = executable(adapter)
    if sandbox and not Path("/usr/bin/bwrap").exists():
        pytest.skip("requires bubblewrap")
    requests, calls = provider
    forbidden = tmp_path / "forbidden.txt"
    secret = tmp_path / "host-only.txt"
    secret.write_text("host-only-content")
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    calls.append(("write", dict(path=str(forbidden), content="must not execute")))
    if "bash" in allowed:
        path = "/workspace" if sandbox else str(run_dir)
        calls.append(("bash", dict(command=f"printf allowed > {path}/worked.txt")))
    if "grep" in allowed:
        path = "/input" if sandbox else str(tmp_path)
        (tmp_path / "facts.txt").write_text("selected-observation")
        calls.append(("grep", dict(pattern="selected-observation", path=path, context=1)))
        if sandbox:
            calls.append(("grep", dict(pattern="host-only-content", path=str(secret), context=1)))
    agent = adapter(
        cli=cli, allowed_tools=allowed, sandbox=sandbox, model="gpt-6-astra", max_steps=10
    )
    with SqliteStore(path=tmp_path / "source.db") as store:
        stream = store.stream("facts", str)
        stream.append("selected-observation", ts=1)
        result = agent.run(
            "Inspect the selected observations.",
            RunningEnvironment(mcp_url="", streams=(stream,), artifacts={}),
            run_dir,
            timeout_s=30,
        )
    assert result.extra.ended_by == "answer", result.extra
    assert result.final_answer == "OK"
    assert len(requests) == len(calls) + 1
    assert all({t["name"] for t in r.get("tools", [])} == set(allowed) for r in requests)
    assert not forbidden.exists(), "disallowed tool executed despite not being advertised"
    workspace = run_dir / "workspace" if sandbox else run_dir
    assert (workspace / "worked.txt").exists() == ("bash" in allowed)
    outputs = [
        str(item["output"])
        for item in requests[-1]["input"]
        if item.get("type") == "function_call_output"
    ]
    if "grep" in allowed:
        assert any("selected-observation" in output for output in outputs)
        if sandbox:
            assert "Path not found" in outputs[-1]
            assert not any("host-only-content" in output for output in outputs)


@pytest.mark.parametrize("failure", ["unknown", "missing_extension"])
def test_dimcode_rejects_unapplied_policy_before_model_call(
    failure: str,
    provider: tuple[list[Any], list[Any]],
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    cli = executable(DimcodeAdapter)
    agent = DimcodeAdapter(
        cli=cli, allowed_tools=("unknown_tool",) if failure == "unknown" else ("bash",)
    )
    if failure == "missing_extension":
        write = agent._write_model_config

        def without_extension(run_dir: Path, url: str) -> None:
            write(run_dir, url)
            (run_dir / ".pi-agent/extensions/allowed_tools.js").unlink()

        monkeypatch.setattr(agent, "_write_model_config", without_extension)
    result = agent.run(
        "Never sent",
        RunningEnvironment(mcp_url="", streams=(), artifacts={}),
        tmp_path,
        timeout_s=30,
    )
    assert result.extra.ended_by == "error"
    assert "allowlist" in result.extra.error
    assert provider[0] == []
