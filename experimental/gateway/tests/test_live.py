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

from collections.abc import Iterator
from dataclasses import dataclass, field
import os
from pathlib import Path
import subprocess
import sys
import threading
from typing import Any

from fastapi.testclient import TestClient
import pytest

from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.introspection.module.info import ModuleInfo, RpcInfo, StreamInfo
from dimos.core.run_registry import RunEntry
from dimos.porcelain.dimos import Dimos
from experimental.gateway import launches, live
from experimental.gateway.state import State

SPEAK = {
    "name": "speak",
    "doc": "Say something.",
    "params": [],
    "return_type": "str",
    "skill": True,
}


@dataclass
class Reading:
    ts: float
    speed: float


@dataclass
class FakeDimos:
    """What the gateway uses of a connected `Dimos`: two Speaker instances (namespaced) and a Camera."""

    calls: list[tuple[str, Any]] = field(default_factory=list)
    stopped: bool = False

    def list_modules(self) -> list[ModuleInfo]:
        speaker = [RpcInfo("start"), RpcInfo("speak"), RpcInfo("volume", documentation="Loudness.")]
        return [
            ModuleInfo(
                "robot0/speaker",
                rpcs=speaker,
                instance_name="robot0/speaker",
                class_name="Speaker",
                qualified_path="x.Speaker",
            ),
            ModuleInfo(
                "robot1/speaker",
                rpcs=speaker,
                instance_name="robot1/speaker",
                class_name="Speaker",
                qualified_path="x.Speaker",
            ),
            ModuleInfo(
                "Camera",
                rpcs=[],
                instance_name="Camera",
                class_name="Camera",
                qualified_path="x.Camera",
                documentation="A camera.",
            ),
        ]

    def list_streams(self) -> list[StreamInfo]:
        return [
            StreamInfo("image", "Image", "Camera", "out", "/image"),
            StreamInfo("odom", "Odometry", "robot0/speaker", "in", "/robot0/odom"),
        ]

    def get_module(self, name: str) -> Any:
        fake = self

        class Handle:
            def __getattr__(self, rpc: str) -> Any:
                def call(*args: Any, **kwargs: Any) -> Any:
                    fake.calls.append((f"{name}/{rpc}", kwargs))
                    if kwargs.get("text") == "explode":
                        raise RuntimeError("speaker on fire")
                    return {"said": kwargs.get("text")}

                return call

        return Handle()

    def restart(self, name: str) -> None:
        self.calls.append(("restart", name))

    def unload(self, name: str) -> None:
        self.calls.append(("unload", name))
        if name == "class x.Camera":
            raise ValueError("camera is busy")

    def peek_stream(self, name: str, timeout: float) -> Any:
        return Reading(12.5, 1.0) if name == "image" else None

    def stop(self) -> None:
        self.stopped = True


@pytest.fixture
def dimos(state: State, monkeypatch: pytest.MonkeyPatch) -> dict[str, Any]:
    world: dict[str, Any] = {"run": "run-1", "apps": []}

    def connect() -> FakeDimos:
        world["apps"].append(FakeDimos())
        return world["apps"][-1]

    state.live = live.Live(connect, lambda: world["run"])
    # stands in for importing the module class from its class path
    monkeypatch.setattr(live, "load_class", lambda path: f"class {path}")
    monkeypatch.setattr(
        state.scanner,
        "cache",
        lambda: {"modules": {"speaker": {"class": "x.Speaker", "rpcs": [SPEAK]}}},
    )
    return world


def test_instances_carry_their_rpcs_and_streams(client: TestClient, dimos: dict[str, Any]) -> None:
    modules = client.get("/active/modules").json()["modules"]
    assert [module["id"] for module in modules] == ["camera", "robot0:speaker", "robot1:speaker"]
    camera, speaker = modules[0], modules[1]
    assert (camera["class"], camera["catalogId"], camera["namespace"], camera["doc"]) == (
        "Camera",
        "x.Camera",
        None,
        "A camera.",
    )
    assert camera["streams"] == [
        {
            "name": "image",
            "type": "Image",
            "direction": "out",
            "channel": "/image",
            "module": "camera",
        }
    ]
    assert speaker["namespace"] == "robot0"
    # every Module's own RPCs (start) are left out; volume isn't in the scan, so it has no params or skill flag
    assert [(rpc["name"], rpc["skill"], rpc["doc"]) for rpc in speaker["rpcs"]] == [
        ("speak", True, "Say something."),
        ("volume", None, "Loudness."),
    ]
    assert client.get("/active/modules/robot1:speaker").json()["id"] == "robot1:speaker"
    assert client.get("/active/modules/nope").status_code == 404
    assert [
        rpc["name"] for rpc in client.get("/active/modules/robot0:speaker/rpcs").json()["rpcs"]
    ] == [
        "speak",
        "volume",
    ]
    skills = client.get("/active/skills").json()["skills"]
    assert [(skill["module"], skill["name"]) for skill in skills] == [
        ("robot0:speaker", "speak"),
        ("robot1:speaker", "speak"),
    ]
    assert [stream["module"] for stream in client.get("/active/streams").json()["streams"]] == [
        "camera",
        "robot0:speaker",
    ]
    assert (
        client.get("/active/modules/robot0:speaker/streams").json()["streams"][0]["name"] == "odom"
    )


def test_an_rpc_is_called_on_one_instance(client: TestClient, dimos: dict[str, Any]) -> None:
    called = client.post(
        "/active/modules/robot1:speaker/rpcs/speak", json={"kwargs": {"text": "hi"}}
    )
    assert called.json() == {"ok": True, "result": {"said": "hi"}, "error": None}
    app = dimos["apps"][-1]
    assert app.calls == [("robot1/speaker/speak", {"text": "hi"})]
    failed = client.post(
        "/active/modules/robot0:speaker/rpcs/speak", json={"kwargs": {"text": "explode"}}
    ).json()
    assert not failed["ok"] and "speaker on fire" in failed["error"]
    assert client.post("/active/modules/robot0:speaker/rpcs/fly").status_code == 404
    assert client.post("/active/modules/nope/rpcs/speak").status_code == 404
    assert len(app.calls) == 2


def test_restart_and_stop_go_by_the_instances_class(
    client: TestClient, dimos: dict[str, Any]
) -> None:
    assert client.post("/active/modules/robot0:speaker/restart").json() == {
        "ok": True,
        "error": None,
    }
    assert client.post("/active/modules/robot1:speaker/stop").json() == {"ok": True, "error": None}
    busy = client.post("/active/modules/camera/stop").json()
    assert not busy["ok"] and "camera is busy" in busy["error"]
    assert client.post("/active/modules/nope/stop").status_code == 404
    calls = [call for call in dimos["apps"][-1].calls if call[0] in ("restart", "unload")]
    assert calls == [
        ("restart", "class x.Speaker"),
        ("unload", "class x.Speaker"),
        ("unload", "class x.Camera"),
    ]


def test_peek_returns_one_message(client: TestClient, dimos: dict[str, Any]) -> None:
    assert client.get("/active/streams/image/peek").json() == {
        "name": "image",
        "type": "Image",
        "value": {"ts": 12.5, "speed": 1.0},
        "ts": 12.5,
    }
    assert client.get("/active/streams/odom/peek?timeout=0.1").status_code == 504
    assert client.get("/active/streams/nope/peek").status_code == 404


def test_one_connection_kept_until_the_run_changes(
    client: TestClient, dimos: dict[str, Any]
) -> None:
    client.get("/active/modules")
    client.get("/active/streams")
    assert len(dimos["apps"]) == 1
    dimos["run"] = "run-2"
    client.get("/active/modules")
    assert len(dimos["apps"]) == 2 and dimos["apps"][0].stopped
    dimos["run"] = None
    for path in ("/active/modules", "/active/streams", "/active/skills", "/active/modules/camera"):
        assert client.get(path).status_code == 409, path
    assert client.post("/active/modules/camera/rpcs/x").status_code == 409
    assert dimos["apps"][1].stopped


def test_instance_ids_are_the_coordinators_keys() -> None:
    assert live.instance_id("robot0/go2connection") == "robot0:go2connection"
    assert live.instance_id("fleet/robot0/go2connection") == "fleet:robot0:go2connection"
    assert live.instance_id("GO2Connection") == "go2connection"


@pytest.fixture
def terminal_run(home: Path, monkeypatch: pytest.MonkeyPatch) -> Iterator[subprocess.Popen[bytes]]:
    """A `dimos run` started in a terminal, its env without DIMOS_TRANSPORT, and the transport connect would use."""
    monkeypatch.delenv("DIMOS_TRANSPORT", raising=False)
    environment = {name: value for name, value in os.environ.items() if name != "DIMOS_TRANSPORT"}
    sleeper = [sys.executable, "-c", "import time; time.sleep(60)"]
    process = subprocess.Popen(sleeper, env=environment, start_new_session=True)
    threading.Thread(target=process.wait, daemon=True).start()
    monkeypatch.setattr(Dimos, "connect", staticmethod(lambda: global_config.transport))
    monkeypatch.setattr(global_config, "transport", global_config.transport)
    yield process
    process.kill()


def registered(pid: int, **fields: Any) -> None:
    RunEntry(
        run_id="20260101-120000-demo",
        pid=pid,
        blueprint="demo",
        started_at="2026-01-01T12:00:00+00:00",
        log_dir="/tmp",
        **fields,
    ).save()


def test_connect_uses_the_active_runs_transport_not_the_last_launchs(
    terminal_run: subprocess.Popen[bytes],
) -> None:
    # the gateway's last launch ran on lcm and has since stopped
    launches._save(
        {
            "blueprint": "demo",
            "started_at": "2026-01-01T11:00:00Z",
            "pid": 999_999_999,
            "config": {"global": {"transport": "lcm"}, "modules": {}},
        }
    )
    registered(terminal_run.pid)
    assert live.connect_dimos() == GlobalConfig().transport == "zenoh"
    registered(terminal_run.pid, config_overrides={"transport": "lcm"})
    assert live.connect_dimos() == "lcm"
    registered(terminal_run.pid, original_argv=["dimos", "run", "demo", "--transport", "lcm"])
    assert live.connect_dimos() == "lcm"
    registered(terminal_run.pid, original_argv=["dimos", "--transport=lcm", "run", "demo"])
    assert live.connect_dimos() == "lcm"
