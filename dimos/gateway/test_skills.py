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

"""GET /dimos/skills, POST /dimos/skills/call and POST /dimos/mcp against a fake run's MCP server (what McpServer
answers for a blueprint with one skill module)."""

from __future__ import annotations

from collections.abc import Iterator
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from typing import Any

from fastapi.testclient import TestClient
import pytest

from dimos.core.run_registry import RunEntry
from dimos.gateway import events, skills
from dimos.gateway.app import ServerState, create_app
from dimos.gateway.uploads import Uploads

TOOLS = [
    {
        "name": "execute_sport_command",
        "description": "Execute a Unitree sport command, e.g. FrontJump.",
        "inputSchema": {
            "type": "object",
            "properties": {"command_name": {"type": "string"}},
            "required": ["command_name"],
        },
        "_meta": {"dimos/uses": ["locomotion"], "dimos/lifecycle": "instant"},
    },
    {
        "name": "current_time",
        "description": "The time.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "list_modules",
        "description": "List modules.",
        "inputSchema": {"type": "object", "properties": {}},
    },
]
MODULES = {
    "UnitreeSkillContainer": ["execute_sport_command", "current_time"],
    "McpServer": ["list_modules"],
}


class FakeMcp(BaseHTTPRequestHandler):
    calls: list[dict[str, Any]] = []

    def log_message(self, *args: Any) -> None:
        pass

    def do_POST(self) -> None:
        request = json.loads(self.rfile.read(int(self.headers["content-length"])))
        if request["method"] == "tools/list":
            result: dict[str, Any] = {"tools": TOOLS}
        else:
            FakeMcp.calls.append(request["params"])
            name, args = request["params"]["name"], request["params"].get("arguments") or {}
            if name == "list_modules":
                text = json.dumps({"modules": MODULES})
            elif args.get("command_name") == "Fly":
                result = {"content": [{"type": "text", "text": "no command Fly"}], "isError": True}
                text = ""
            else:
                text = f"{name} ran with {json.dumps(args)}"
            if text:
                result = {"content": [{"type": "text", "text": text}]}
        body = json.dumps({"jsonrpc": "2.0", "id": request["id"], "result": result}).encode()
        self.send_response(200)
        self.send_header("content-type", "application/json")
        self.send_header("content-length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


@pytest.fixture
def fake_run(monkeypatch: pytest.MonkeyPatch) -> Iterator[skills.Server]:
    server = ThreadingHTTPServer(("127.0.0.1", 0), FakeMcp)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    run = skills.Server(
        "20260101-120000-unitree-go2-agentic",
        "unitree-go2-agentic",
        f"http://127.0.0.1:{server.server_port}/mcp",
    )
    FakeMcp.calls = []
    monkeypatch.setattr(skills, "live_servers", lambda: [run])
    yield run
    server.shutdown()


@pytest.fixture
def client(server_home: Path, checkout: Path, fake_worker: list[str]) -> Iterator[TestClient]:
    bus = events.Bus()
    uploads = Uploads(checkout, bus, None, server_home / "uploads.log", worker=fake_worker)
    with TestClient(create_app(ServerState(checkout, bus, uploads), background=False)) as client:
        yield client


def test_nothing_running_lists_no_skills_and_refuses_a_call(
    client: TestClient, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(skills, "live_servers", lambda: [])
    assert client.get("/dimos/skills").json() == {"skills": [], "runs": []}
    response = client.post("/dimos/skills/call", json={"skill": "current_time"})
    assert response.status_code == 409
    assert "no blueprint is running" in response.json()["error"]


def test_a_run_without_an_mcp_server_says_so(
    client: TestClient, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        skills,
        "live_servers",
        lambda: [skills.Server("r1", "unitree-go2", "http://127.0.0.1:1/mcp")],
    )
    answer = client.get("/dimos/skills").json()
    assert answer["skills"] == []
    assert answer["runs"][0]["up"] is False
    assert "only agentic blueprints" in answer["runs"][0]["error"]


def test_skills_list_with_their_module_and_params(
    client: TestClient, fake_run: skills.Server
) -> None:
    answer = client.get("/dimos/skills").json()
    sport = next(s for s in answer["skills"] if s["name"] == "execute_sport_command")
    assert sport["module"] == "UnitreeSkillContainer"
    assert sport["required"] == ["command_name"]
    assert sport["uses"] == ["locomotion"]
    assert sport["runId"] == fake_run.run_id
    assert answer["runs"] == [
        {
            "runId": fake_run.run_id,
            "blueprint": "unitree-go2-agentic",
            "mcpUrl": fake_run.url,
            "up": True,
            "error": None,
        }
    ]


def test_a_call_goes_through_the_runs_mcp_server(
    client: TestClient, fake_run: skills.Server
) -> None:
    response = client.post(
        "/dimos/skills/call",
        json={"skill": "execute_sport_command", "args": {"command_name": "FrontJump"}},
    )
    assert response.status_code == 200
    answer = response.json()
    assert answer["ok"] is True
    assert answer["text"] == 'execute_sport_command ran with {"command_name": "FrontJump"}'
    assert answer["module"] == "UnitreeSkillContainer"
    assert {
        "name": "execute_sport_command",
        "arguments": {"command_name": "FrontJump"},
    } in FakeMcp.calls

    failed = client.post(
        "/dimos/skills/call",
        json={"skill": "execute_sport_command", "args": {"command_name": "Fly"}},
    )
    assert failed.json()["ok"] is False and failed.json()["text"] == "no command Fly"

    missing = client.post("/dimos/skills/call", json={"skill": "fly"})
    assert missing.status_code == 404
    assert "execute_sport_command" in missing.json()["error"]
    wrong_module = client.post(
        "/dimos/skills/call", json={"skill": "current_time", "module": "McpServer"}
    )
    assert wrong_module.status_code == 404


def test_the_mcp_server_has_two_fixed_tools(client: TestClient, fake_run: skills.Server) -> None:
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
    assert skills.mcp_url(RunEntry("r", 1, "b", "t", "/tmp")).startswith("http://localhost:")
