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
from pathlib import Path
import socket
import threading

import pytest
import zenoh

from dimos.core.global_config import global_config
from dimos.protocol.service.zenohservice import ZenohSessionPool
from experimental.gateway.utils import zenoh_events
from experimental.gateway.utils.events import Bus
from experimental.gateway.utils.uploads import Queue


@pytest.fixture
def no_namespace_env(monkeypatch: pytest.MonkeyPatch) -> None:
    for name in (zenoh_events.NAMESPACE_ENV, "DIMOS_APP"):
        monkeypatch.delenv(name, raising=False)


def write_desktop_config(server_home: Path, text: str) -> None:
    home = server_home / "home"
    home.mkdir(parents=True, exist_ok=True)
    (home / "config.yaml").write_text(text)


def test_host_chunk() -> None:
    assert zenoh_events.host_chunk("Jeffs-MacBook.local") == "jeffs-macbook-local"
    assert zenoh_events.host_chunk("a_b c") == "a-b-c"


@pytest.mark.parametrize("bad", ["", "a//b", "/a", "a/", "a/*", "a/**/b", "a/$*", "a?b", "a#b"])
def test_namespace_rejects_wildcards_and_empty_chunks(bad: str) -> None:
    with pytest.raises(ValueError):
        zenoh_events.check_namespace(bad)


def test_namespace_default_is_desktops(
    server_home: Path, no_namespace_env: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(socket, "gethostname", lambda: "Jeffs-MacBook.local")
    assert zenoh_events.resolve_namespace() == "dimos-desktop/jeffs-macbook-local-5555"
    write_desktop_config(server_home, "desktop:\n  port: 7341\n")
    assert zenoh_events.resolve_namespace() == "dimos-desktop/jeffs-macbook-local-7341"
    write_desktop_config(server_home, "desktop:\n  port: 7341\n  namespace: lab/robot-desk\n")
    assert zenoh_events.resolve_namespace() == "lab/robot-desk"


def test_namespace_order(
    server_home: Path, no_namespace_env: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    write_desktop_config(server_home, "desktop:\n  namespace: from/config\n")
    monkeypatch.setenv("DIMOS_APP", json.dumps({"zenohNamespace": "from/app"}))
    assert zenoh_events.resolve_namespace() == "from/app"
    monkeypatch.setenv(zenoh_events.NAMESPACE_ENV, "from/env")
    assert zenoh_events.resolve_namespace() == "from/env"
    assert zenoh_events.resolve_namespace("from/flag") == "from/flag"
    write_desktop_config(server_home, "desktop:\n  namespace: bad/*\n")
    monkeypatch.delenv(zenoh_events.NAMESPACE_ENV)
    monkeypatch.delenv("DIMOS_APP")
    with pytest.raises(ValueError):
        zenoh_events.resolve_namespace()


def test_connect_order(no_namespace_env: None, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "zenoh_connect", "tcp/a:1, tcp/b:2")
    assert zenoh_events.resolve_connect() == ["tcp/a:1", "tcp/b:2"]
    monkeypatch.setenv("DIMOS_APP", json.dumps({"zenohConnect": "tcp/10.0.0.2:7447"}))
    assert zenoh_events.resolve_connect() == ["tcp/10.0.0.2:7447"]
    assert zenoh_events.resolve_connect("tcp/flag:3") == ["tcp/flag:3"]
    assert zenoh_events.resolve_connect("") == []


def test_every_bus_event_reaches_its_key() -> None:
    sent: list[tuple[str, dict[str, object]]] = []
    bus = Bus()
    bus.sinks.append(
        zenoh_events.Publisher("ns/a", lambda key, payload: sent.append((key, json.loads(payload))))
    )
    record = {
        "timestamp": "t",
        "level": "error",
        "logger": "nav",
        "event": "e",
        "extra": {},
        "raw": "{}",
    }
    login = {
        "state": "idle",
        "url": None,
        "urlComplete": None,
        "code": None,
        "expiresAt": None,
        "email": None,
        "error": None,
    }
    upload, _ = Queue().enqueue(Path("/r/a.mcap"), 4, None, None)
    events = [
        {"type": "launch", "launch": None},
        {"type": "log", "runId": "r", "record": record},
        {"type": "upload", "upload": upload},
        {"type": "uploads", "waitingForLogin": False, "cleared": True},
        {"type": "upload-removed", "id": "u1"},
        {"type": "cloud-login", "login": login},
    ]
    for event in events:
        bus.send(event)
    assert sent == [(f"ns/a/dimos/events/{event['type']}", event) for event in events]


def test_a_failing_sink_never_stops_the_bus() -> None:
    def broken(event: dict[str, object]) -> None:
        raise RuntimeError("zenoh is down")

    seen: list[dict[str, object]] = []
    bus = Bus()
    bus.sinks += [broken, seen.append]
    bus.send({"type": "uploads", "waitingForLogin": True})
    assert seen == [{"type": "uploads", "waitingForLogin": True}]


def free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return int(probe.getsockname()[1])


def test_a_local_peer_hears_the_events(monkeypatch: pytest.MonkeyPatch) -> None:
    """A real zenoh session dials a subscriber's listener; no multicast, so nothing else on the machine joins."""
    monkeypatch.setattr(global_config, "zenoh_multicast", False)
    monkeypatch.setattr(global_config, "zenoh_mode", "peer")
    endpoint = f"tcp/127.0.0.1:{free_port()}"
    listener_config = zenoh.Config()
    listener_config.insert_json5("listen/endpoints", json.dumps([endpoint]))
    listener_config.insert_json5("scouting/multicast/enabled", "false")
    received: list[tuple[str, str, dict[str, object]]] = []
    arrived = threading.Event()

    def on_sample(sample: zenoh.Sample) -> None:
        received.append(
            (str(sample.key_expr), str(sample.encoding), json.loads(sample.payload.to_bytes()))
        )
        if len(received) == 2:
            arrived.set()

    pool = ZenohSessionPool()
    with zenoh.open(listener_config) as listener:
        subscriber = listener.declare_subscriber("test-ns/dimos/events/**", on_sample)
        try:
            bus = Bus()
            bus.sinks.append(zenoh_events.open_publisher("test-ns", [endpoint], pool))
            bus.send({"type": "uploads", "waitingForLogin": False})
            bus.send({"type": "upload-removed", "id": "u1"})
            assert arrived.wait(10)
        finally:
            subscriber.undeclare()
            pool.close_all()
    assert received == [
        (
            "test-ns/dimos/events/uploads",
            "application/json",
            {"type": "uploads", "waitingForLogin": False},
        ),
        (
            "test-ns/dimos/events/upload-removed",
            "application/json",
            {"type": "upload-removed", "id": "u1"},
        ),
    ]
