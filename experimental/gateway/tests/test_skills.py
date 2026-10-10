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

import json
from types import SimpleNamespace
from typing import Any

from fastapi.testclient import TestClient
import pytest

from dimos.core.coordination.module_coordinator import ModuleDescriptor
from experimental.gateway import skills
from experimental.gateway.state import State

SCHEMA = {
    "description": "Say something.",
    "title": "speak",
    "type": "object",
    "properties": {"text": {"type": "string"}},
    "required": ["text"],
}


class FakeBus:
    def __init__(self, running: bool = True) -> None:
        self.running = running
        self.calls: list[tuple[str, Any]] = []

    def call(self, name: str, args: list[Any], kwargs: dict[str, Any], timeout: float) -> Any:
        self.calls.append((name, kwargs))
        if not self.running:
            raise TimeoutError()
        if name == "Coordinator/list_modules":
            return [ModuleDescriptor("Speaker", "x.Speaker", ["get_skills", "speak"], "Speaker")]
        if name == "Speaker/get_skills":
            info = SimpleNamespace(
                func_name="speak", args_schema=json.dumps(SCHEMA), lifecycle="sync", uses=[]
            )
            return [info]
        if kwargs.get("text") == "explode":
            raise RuntimeError("speaker on fire")
        return f"said {kwargs['text']}"


@pytest.fixture
def bus(state: State, monkeypatch: pytest.MonkeyPatch) -> FakeBus:
    fake = FakeBus()
    state.skills = skills.Skills(fake)
    monkeypatch.setattr(skills, "live_run", lambda: skills.Run("run-1", "demo", None))
    return fake


def test_skills_are_listed_and_called(client: TestClient, bus: FakeBus) -> None:
    listed = client.get("/dimos/skills").json()
    assert listed["run"] == {"runId": "run-1", "blueprint": "demo"}
    assert [
        (s["name"], s["module"], s["description"], s["required"]) for s in listed["skills"]
    ] == [("speak", "Speaker", "Say something.", ["text"])]
    called = client.post(
        "/dimos/skills/call", json={"skill": "speak", "args": {"text": "hi"}}
    ).json()
    assert called["ok"] and called["text"] == "said hi" and called["via"] == "rpc"
    failed = client.post(
        "/dimos/skills/call", json={"skill": "speak", "args": {"text": "explode"}}
    ).json()
    assert not failed["ok"] and "speaker on fire" in failed["text"]


def test_bad_skill_calls_are_refused_before_reaching_the_robot(
    client: TestClient, bus: FakeBus
) -> None:
    assert client.post("/dimos/skills/call", json={"skill": "fly"}).status_code == 404
    missing = client.post("/dimos/skills/call", json={"skill": "speak", "args": {}})
    assert missing.status_code == 400 and "missing text" in missing.json()["error"]
    assert not [name for name, _ in bus.calls if name == "Speaker/speak"]


def test_nothing_running_means_no_skills(client: TestClient, bus: FakeBus) -> None:
    bus.running = False
    assert client.get("/dimos/skills").json() == {"skills": [], "run": None, "errors": []}
    assert client.post("/dimos/skills/call", json={"skill": "speak"}).status_code == 409


def test_mcp_speaks_json_rpc(client: TestClient, bus: FakeBus) -> None:
    def rpc(method: str, params: Any = None) -> Any:
        body = {"jsonrpc": "2.0", "id": 1, "method": method, "params": params or {}}
        return client.post("/dimos/mcp", json=body).json()

    assert rpc("initialize")["result"]["serverInfo"]["name"] == "dimos-skills"
    assert [t["name"] for t in rpc("tools/list")["result"]["tools"]] == [
        "list_skills",
        "call_skill",
    ]
    listed = rpc("tools/call", {"name": "list_skills", "arguments": {"query": "say"}})
    assert json.loads(listed["result"]["content"][0]["text"])["skills"][0]["name"] == "speak"
    called = rpc(
        "tools/call",
        {"name": "call_skill", "arguments": {"skill": "speak", "args": {"text": "yo"}}},
    )
    assert called["result"] == {"content": [{"type": "text", "text": "said yo"}], "isError": False}
    assert rpc("nope")["error"]["code"] == -32601
    assert (
        client.post(
            "/dimos/mcp", json={"jsonrpc": "2.0", "method": "notifications/initialized"}
        ).status_code
        == 202
    )
