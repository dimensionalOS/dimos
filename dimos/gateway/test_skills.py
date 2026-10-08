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

"""GET /dimos/skills, POST /dimos/skills/call and POST /dimos/mcp against a fake dimos RPC bus (what a coordinator and
its modules answer for a plain, agent-less unitree-go2) and, for a skill that holds a capability, a fake McpServer."""

from __future__ import annotations

from collections.abc import Iterator
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from typing import Any

from fastapi.testclient import TestClient
import pytest

from dimos.agents.skill_result import SkillResult
from dimos.core.coordination.module_coordinator import ModuleDescriptor
from dimos.core.module import SkillInfo
from dimos.core.run_registry import RunEntry
from dimos.gateway import events, skills
from dimos.gateway.app import ServerState, create_app
from dimos.gateway.uploads import Uploads


def schema(description: str, properties: dict[str, Any], required: list[str]) -> str:
    return json.dumps(
        {
            "description": description,
            "title": "x",
            "type": "object",
            "properties": properties,
            "required": required,
        }
    )


SKILLS = {
    "GO2Connection": [
        SkillInfo("GO2Connection", "get_battery_soc", schema("The battery's charge.", {}, [])),
    ],
    "UnitreeSkillContainer": [
        SkillInfo(
            "UnitreeSkillContainer",
            "execute_sport_command",
            schema(
                "Execute a Unitree sport command, e.g. FrontJump.",
                {"command_name": {"type": "string"}},
                ["command_name"],
            ),
        ),
    ],
    "PatrollingModule": [
        SkillInfo(
            "PatrollingModule",
            "start_patrol",
            schema("Patrol.", {}, []),
            ("locomotion",),
            "background",
        ),
    ],
}


class FakeBus:
    """A coordinator and its modules, as dimos's module RPC answers them."""

    def __init__(self, up: bool = True, broken: tuple[str, ...] = ()) -> None:
        self.up = up
        self.broken = broken
        self.calls: list[tuple[str, dict[str, Any]]] = []

    def call(self, name: str, args: list[Any], kwargs: dict[str, Any], timeout: float) -> Any:
        if not self.up:
            raise TimeoutError(f"RPC call to '{name}' timed out")
        module, method = name.rsplit("/", 1)
        if name == "Coordinator/list_modules":
            return [
                ModuleDescriptor(m, f"x.{m}", ["start", "stop", "get_skills"], m)
                for m in [*SKILLS, *self.broken]
            ]
        if module in self.broken:
            raise TimeoutError(f"RPC call to '{name}' timed out after {timeout} seconds")
        if method == "get_skills":
            return SKILLS[module]
        self.calls.append((name, kwargs))
        if kwargs.get("command_name") == "Fly":
            raise ValueError("no command Fly")
        if method == "get_battery_soc":
            return 87
        return SkillResult(message=f"{method} ran with {json.dumps(kwargs)}") if kwargs else None


RUN = skills.Run("20260101-120000-unitree-go2", "unitree-go2", None)


@pytest.fixture
def bus(monkeypatch: pytest.MonkeyPatch) -> FakeBus:
    fake = FakeBus()
    monkeypatch.setitem(skills._cache, "key", None)
    monkeypatch.setattr(skills, "bus", fake)
    monkeypatch.setattr(skills, "live_run", lambda: RUN)
    return fake


@pytest.fixture
def client(server_home: Path, checkout: Path, fake_worker: list[str]) -> Iterator[TestClient]:
    the_bus = events.Bus()
    uploads = Uploads(checkout, the_bus, None, server_home / "uploads.log", worker=fake_worker)
    with TestClient(
        create_app(ServerState(checkout, the_bus, uploads), background=False)
    ) as client:
        yield client


def test_nothing_running_lists_no_skills_and_refuses_a_call(
    client: TestClient, bus: FakeBus
) -> None:
    bus.up = False
    assert client.get("/dimos/skills").json() == {"skills": [], "run": None, "errors": []}
    response = client.post("/dimos/skills/call", json={"skill": "get_battery_soc"})
    assert response.status_code == 409
    assert "no blueprint is running" in response.json()["error"]


def test_a_blueprint_without_an_agent_lists_its_skills(client: TestClient, bus: FakeBus) -> None:
    answer = client.get("/dimos/skills").json()
    assert [s["name"] for s in answer["skills"]] == [
        "execute_sport_command",
        "get_battery_soc",
        "start_patrol",
    ]
    sport = answer["skills"][0]
    assert sport["module"] == "UnitreeSkillContainer"
    assert sport["description"] == "Execute a Unitree sport command, e.g. FrontJump."
    assert sport["params"] == {
        "type": "object",
        "properties": {"command_name": {"type": "string"}},
        "required": ["command_name"],
    }
    assert sport["required"] == ["command_name"]
    assert sport["runId"] == RUN.run_id
    patrol = answer["skills"][2]
    assert (patrol["lifecycle"], patrol["uses"]) == ("background", ["locomotion"])
    assert answer["run"] == {"runId": RUN.run_id, "blueprint": "unitree-go2"}
    assert answer["errors"] == []


def test_a_module_that_doesnt_answer_is_named(
    client: TestClient, bus: FakeBus, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(skills, "bus", FakeBus(broken=("NativeSlam",)))
    answer = client.get("/dimos/skills").json()
    assert len(answer["skills"]) == 3
    assert answer["errors"][0]["module"] == "NativeSlam"
    assert "TimeoutError" in answer["errors"][0]["error"]


def test_a_call_goes_over_module_rpc(client: TestClient, bus: FakeBus) -> None:
    response = client.post(
        "/dimos/skills/call",
        json={"skill": "execute_sport_command", "args": {"command_name": "FrontJump"}},
    )
    assert response.status_code == 200
    answer = response.json()
    assert (answer["ok"], answer["via"], answer["module"]) == (True, "rpc", "UnitreeSkillContainer")
    # a SkillResult answers as McpServer encodes it for an agent
    assert (
        json.loads(answer["text"])["message"]
        == 'execute_sport_command ran with {"command_name": "FrontJump"}'
    )
    assert bus.calls == [
        ("UnitreeSkillContainer/execute_sport_command", {"command_name": "FrontJump"})
    ]

    battery = client.post("/dimos/skills/call", json={"skill": "get_battery_soc"}).json()
    assert battery["text"] == "87"

    failed = client.post(
        "/dimos/skills/call",
        json={"skill": "execute_sport_command", "args": {"command_name": "Fly"}},
    ).json()
    assert failed["ok"] is False and "no command Fly" in failed["text"]


def test_a_call_is_refused_before_it_reaches_the_robot(client: TestClient, bus: FakeBus) -> None:
    missing_arg = client.post("/dimos/skills/call", json={"skill": "execute_sport_command"})
    assert missing_arg.status_code == 400
    assert "missing command_name" in missing_arg.json()["error"]
    unknown_arg = client.post(
        "/dimos/skills/call", json={"skill": "get_battery_soc", "args": {"speed": 1}}
    )
    assert unknown_arg.status_code == 400
    missing = client.post("/dimos/skills/call", json={"skill": "fly"})
    assert missing.status_code == 404
    assert "execute_sport_command" in missing.json()["error"]
    wrong_module = client.post(
        "/dimos/skills/call", json={"skill": "get_battery_soc", "module": "PatrollingModule"}
    )
    assert wrong_module.status_code == 404
    other_run = client.post("/dimos/skills/call", json={"skill": "get_battery_soc", "runId": "r0"})
    assert other_run.status_code == 404
    assert bus.calls == []


class FakeMcp(BaseHTTPRequestHandler):
    calls: list[dict[str, Any]] = []

    def log_message(self, *args: Any) -> None:
        pass

    def do_POST(self) -> None:
        request = json.loads(self.rfile.read(int(self.headers["content-length"])))
        if request["method"] == "tools/list":
            result: dict[str, Any] = {"tools": [{"name": "start_patrol", "inputSchema": {}}]}
        else:
            FakeMcp.calls.append(request["params"])
            result = {
                "content": [
                    {"type": "text", "text": "Cannot start 'start_patrol': capability held"}
                ]
            }
        body = json.dumps({"jsonrpc": "2.0", "id": request["id"], "result": result}).encode()
        self.send_response(200)
        self.send_header("content-type", "application/json")
        self.send_header("content-length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def test_a_capability_holding_skill_goes_through_the_runs_mcp_server_when_one_answers(
    client: TestClient, bus: FakeBus, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        skills, "live_run", lambda: skills.Run(RUN.run_id, RUN.blueprint, "http://127.0.0.1:1/mcp")
    )
    nobody = client.post("/dimos/skills/call", json={"skill": "start_patrol"}).json()
    assert nobody["via"] == "rpc"
    assert bus.calls == [("PatrollingModule/start_patrol", {})]

    server = ThreadingHTTPServer(("127.0.0.1", 0), FakeMcp)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    FakeMcp.calls = []
    url = f"http://127.0.0.1:{server.server_port}/mcp"
    monkeypatch.setattr(skills, "live_run", lambda: skills.Run(RUN.run_id, RUN.blueprint, url))
    try:
        answer = client.post("/dimos/skills/call", json={"skill": "start_patrol"}).json()
        assert answer["via"] == "mcp"
        assert "capability held" in answer["text"]
        assert FakeMcp.calls == [{"name": "start_patrol", "arguments": {}}]
        # one without a capability stays on module RPC even then
        assert (
            client.post("/dimos/skills/call", json={"skill": "get_battery_soc"}).json()["via"]
            == "rpc"
        )
    finally:
        server.shutdown()


def test_the_mcp_server_has_two_fixed_tools(client: TestClient, bus: FakeBus) -> None:
    def rpc(method: str, params: dict[str, Any] | None = None) -> Any:
        return client.post(
            "/dimos/mcp", json={"jsonrpc": "2.0", "id": 1, "method": method, "params": params or {}}
        ).json()

    assert (
        rpc("initialize", {"protocolVersion": "2025-03-26"})["result"]["protocolVersion"]
        == "2025-03-26"
    )
    assert (
        client.post(
            "/dimos/mcp", json={"jsonrpc": "2.0", "method": "notifications/initialized"}
        ).status_code
        == 202
    )
    assert [t["name"] for t in rpc("tools/list")["result"]["tools"]] == [
        "list_skills",
        "call_skill",
    ]

    listed = rpc("tools/call", {"name": "list_skills", "arguments": {"query": "jump"}})["result"]
    assert [s["name"] for s in json.loads(listed["content"][0]["text"])["skills"]] == [
        "execute_sport_command"
    ]

    called = rpc(
        "tools/call",
        {
            "name": "call_skill",
            "arguments": {"skill": "execute_sport_command", "args": {"command_name": "FrontJump"}},
        },
    )["result"]
    assert called["isError"] is False
    assert "FrontJump" in called["content"][0]["text"]
    assert (
        rpc("tools/call", {"name": "call_skill", "arguments": {"skill": "fly"}})["result"][
            "isError"
        ]
        is True
    )


def test_a_runs_mcp_url_follows_its_port_override() -> None:
    entry = RunEntry("r", 1, "b", "t", "/tmp", config_overrides={"mcp_port": 9123})
    assert skills.mcp_url(entry) == "http://localhost:9123/mcp"
    after_run = RunEntry(
        "r", 1, "b", "t", "/tmp", original_argv=["dimos", "run", "b", "--mcp-port", "9991"]
    )
    assert skills.mcp_url(after_run) == "http://localhost:9991/mcp"
    assert skills.mcp_url(
        RunEntry("r", 1, "b", "t", "/tmp", original_argv=["--mcp-port=9992"])
    ).endswith(":9992/mcp")
    assert skills.mcp_url(RunEntry("r", 1, "b", "t", "/tmp")).startswith("http://localhost:")
