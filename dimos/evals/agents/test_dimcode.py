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

"""Exercise gateway disconnects and malformed packets through a real subprocess/socket."""

import json
import os
from pathlib import Path
import socket
import sys

from pydantic import JsonValue
import pytest

from dimos.evals.agents.dimcode import DimcodeAdapter
from dimos.evals.agents.lib.pi_config import (
    Prompt,
    SessionSettings,
    Shutdown,
    ToolPolicyState,
    gateway_request,
)
from dimos.evals.types import RunningEnvironment


def serve_gateway() -> None:
    """Small protocol peer for transport failures; native startup is tested separately."""
    config = Path(os.environ["DIMCODE_HOME"])
    settings = SessionSettings.model_validate_json((config / "settings.json").read_text())
    assert settings.default_provider == "anthropic"
    assert settings.default_thinking_level == "medium"
    ready = ToolPolicyState(tools=DimcodeAdapter.default_tools, unknown=())
    (config / "extensions/runtime-ready.json").write_text(ready.model_dump_json())
    raw = Path.cwd() / "raw"
    raw.mkdir(exist_ok=True)
    (raw / "000-request.json").write_text("{}")
    (raw / "000-response.json").write_text('{"body":{"id":"msg_1"}}')
    with socket.socket(socket.AF_UNIX) as server:
        server.bind(str(Path(os.environ["XDG_RUNTIME_DIR"]) / "dimcode.sock"))
        server.listen(1)
        conn, _ = server.accept()
        with conn, conn.makefile("rb") as stream:

            def send(packet: dict[str, JsonValue]) -> None:
                conn.sendall((json.dumps(packet) + "\n").encode())

            for line in stream:
                packet = gateway_request.validate_json(line)
                command = packet.command
                if isinstance(command, Shutdown):
                    break
                if isinstance(command, Prompt):
                    # Events may precede the prompt acknowledgement.
                    send(
                        {
                            "type": "event",
                            "event": {
                                "type": "message_end",
                                "message": {
                                    "role": "assistant",
                                    "responseId": "msg_1",
                                    "model": "claude-fable-5-1",
                                    "content": [{"type": "text", "text": "42"}],
                                    "usage": {"input": 10, "output": 2, "cost": {"total": 0.1}},
                                },
                            },
                        }
                    )
                    if "disconnect" in command.message:
                        break
                    if "malformed" in command.message:
                        send({"type": "event", "event": "not-an-event"})
                        break
                    send({"type": "event", "event": {"type": "idle"}})
                send({"type": "response", "id": packet.id, "data": None})


@pytest.mark.parametrize("mode", ["answer", "disconnect", "malformed"])
def test_gateway_failure_preserves_completed_steps(tmp_path: Path, mode: str) -> None:
    cli = tmp_path / "fake-dimcode"
    cli.write_text(
        f"#!{sys.executable}\n"
        "from dimos.evals.agents.test_dimcode import serve_gateway\n"
        "serve_gateway()\n"
    )
    cli.chmod(0o755)
    case_dir = tmp_path / "case"
    case_dir.mkdir()
    agent = DimcodeAdapter(cli=str(cli), provider="anthropic", model="claude-fable-5-1")
    result = agent.run(
        mode,
        RunningEnvironment(mcp_url="", streams=(), artifacts={}),
        case_dir,
        timeout_s=10,
    )
    assert result.final_answer == "42", result.extra
    assert result.final_metrics.total_cost_usd == 0.1
    assert result.extra.ended_by == ("answer" if mode == "answer" else "error")
    assert not agent._runtime_dir.exists()
