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

"""Exercise gateway protocol and cleanup with a real isolated subprocess/socket."""

from pathlib import Path
import sys
import textwrap

import pytest

from dimos.evals.agents.dimcode import DimcodeAdapter
from dimos.evals.types import RunningEnvironment


@pytest.mark.parametrize("failure", [False, True])
def test_gateway_turn_and_disconnect_preserve_trace(tmp_path: Path, failure: bool) -> None:
    cli = tmp_path / "fake-dimcode"
    cli.write_text(
        f"#!{sys.executable}\n"
        + textwrap.dedent(r"""
        import json, os, socket
        from pathlib import Path
        runtime = Path(os.environ["XDG_RUNTIME_DIR"])
        config = Path(os.environ["DIMCODE_HOME"])
        root = Path.cwd()
        settings = json.loads((config / "settings.json").read_text())
        assert settings["defaultProvider"] == "anthropic"
        assert settings["defaultThinkingLevel"] == "medium"
        assert Path(os.environ["HOME"]) == root / "home"
        (root / "raw").mkdir(exist_ok=True)
        (root / "raw/000-request.json").write_text("{}")
        (root / "raw/000-response.json").write_text(json.dumps({"body": {"id": "msg_1"}}))
        with socket.socket(socket.AF_UNIX) as server:
            server.bind(str(runtime / "dimcode.sock"))
            server.listen(1)
            conn, _ = server.accept()
            with conn, conn.makefile("rb") as stream:
                def send(packet):
                    conn.sendall((json.dumps(packet) + "\n").encode())
                for line in stream:
                    packet = json.loads(line)
                    kind = packet["command"]["type"]
                    if kind == "shutdown":
                        break
                    if kind == "prompt":
                        # Events may precede the acknowledgement of prompt.
                        send({"type":"event", "event": {"type":"message_end", "message": {
                            "role":"assistant", "responseId":"msg_1", "model":"claude-fable-5-1",
                            "content":[{"type":"text","text":"42"}],
                            "usage":{"input":10,"output":2,"cost":{"total":0.1}}
                        }}})
                        if "disconnect" in packet["command"]["message"]:
                            break
                        send({"type":"event", "event":{"type":"idle"}})
                    send({"type":"response", "id":packet["id"], "data":None})
    """)
    )
    cli.chmod(0o755)
    case_dir = tmp_path / "case"
    case_dir.mkdir()
    agent = DimcodeAdapter(cli=str(cli), provider="anthropic", model="claude-fable-5-1")
    result = agent.run(
        "disconnect" if failure else "Question",
        RunningEnvironment(mcp_url="", streams=(), artifacts={}),
        case_dir,
        timeout_s=10,
    )
    assert result.final_answer == "42"
    assert result.final_metrics.total_cost_usd == 0.1
    assert result.extra.ended_by == ("error" if failure else "answer")
    assert not agent._runtime_dir.exists()
