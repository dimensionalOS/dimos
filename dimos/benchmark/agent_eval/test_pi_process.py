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

from __future__ import annotations

import hashlib
import sys

import pytest

from dimos.benchmark.agent_eval.config import RuntimeCredential
from dimos.benchmark.agent_eval.pi_adapter import CodePolicyCallLog
from dimos.benchmark.agent_eval.pi_process import NodePiSessionFactory

_HOST = r"""
import json, pathlib, sys
def send(value):
    print(json.dumps(value, separators=(",", ":")), flush=True)
start = json.loads(sys.stdin.readline())
send({"version":1,"type":"session_started","id":start["id"],"tools":["python_exec"]})
for line in sys.stdin:
    frame = json.loads(line)
    if frame["type"] == "prompt":
        send({"version":1,"type":"tool_call","id":"tool-1","tool":"python_exec","params":{"code":"1 + 1"}})
        reply = json.loads(sys.stdin.readline())
        assert reply["type"] == "tool_reply" and reply["ok"]
        send({"version":1,"type":"turn_complete","id":frame["id"],"policy_call_count":1,"final_text":"done"})
    elif frame["type"] == "dispose":
        pathlib.Path("pi-session").mkdir()
        pathlib.Path("pi-prompt").mkdir()
        pathlib.Path("pi-session/native.jsonl").write_text('{"type":"session"}\n')
        pathlib.Path("pi-prompt/system.txt").write_text("system")
        pathlib.Path("pi-prompt/initial.txt").write_text(start["initial_prompt"])
        send({"version":1,"type":"session_closed","id":start["id"],"evidence":{
          "state":"complete","persisted":True,"relative_path":"pi-session/native.jsonl",
          "system_prompt":{"relative_path":"pi-prompt/system.txt","byte_count":6,"sha256":"0"*64},
          "initial_prompt":{"relative_path":"pi-prompt/initial.txt","byte_count":len(start["initial_prompt"]),"sha256":"1"*64}
        }})
        break
"""


class _Mcp:
    def wait_for_ready(self, timeout: float) -> bool:
        return True

    def list_tools(self):
        return []

    def call_tool(self, name, arguments=None):
        assert name == "python_exec"
        return {"content": [{"type": "text", "text": "2"}]}


def test_node_pi_process_roundtrip_and_native_evidence(tmp_path) -> None:
    attempt = tmp_path / ("attempt_" + "a" * 32)
    attempt.mkdir()
    calls = CodePolicyCallLog(attempt / "code-policy-calls.jsonl")
    factory = NodePiSessionFactory(
        command=(sys.executable, "-c", _HOST),
        credential=RuntimeCredential(
            auth_mode="environment",
            binding_name="OPENAI_API_KEY",
            value="secret",
        ),
        model="gpt-5.6-luna",
        thinking_level="medium",
        startup_timeout_s=2.0,
    )

    session = factory.create(
        attempt_path=attempt,
        public_prompt="Navigate to the bathtub.",
        code_policy_session_id="code_policy_session_" + "b" * 32,
        call_log=calls,
        mcp=_Mcp(),
    )
    turn = session.prompt("Navigate to the bathtub.", 2.0)
    session.dispose()
    calls.close()

    assert turn.policy_call_count == 1
    references = session.artifact_references()
    assert {item.path for item in references} == {
        "pi-adapter.stderr.log",
        "pi-session/native.jsonl",
        "pi-prompt/system.txt",
        "pi-prompt/initial.txt",
    }
    retained = (attempt / "code-policy-calls.jsonl").read_text()
    assert "secret" not in retained
    assert hashlib.sha256((attempt / "pi-session/native.jsonl").read_bytes()).hexdigest()


def test_node_pi_process_reaps_child_when_startup_times_out(tmp_path) -> None:
    attempt = tmp_path / ("attempt_" + "c" * 32)
    attempt.mkdir()
    calls = CodePolicyCallLog(attempt / "code-policy-calls.jsonl")
    factory = NodePiSessionFactory(
        command=(sys.executable, "-c", "import time; time.sleep(60)"),
        credential=RuntimeCredential(
            auth_mode="environment",
            binding_name="OPENAI_API_KEY",
            value="secret",
        ),
        model="gpt-5.6-luna",
        thinking_level="medium",
        startup_timeout_s=0.05,
    )

    with pytest.raises(TimeoutError, match="session_started"):
        factory.create(
            attempt_path=attempt,
            public_prompt="Navigate to the bathtub.",
            code_policy_session_id="code_policy_session_" + "d" * 32,
            call_log=calls,
            mcp=_Mcp(),
        )

    calls.close()
    assert (attempt / "pi-adapter.stderr.log").read_bytes() == b""
