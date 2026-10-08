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
from pathlib import Path
import sys
import textwrap
from typing import Any

import fastapi.routing
from pydantic import BaseModel, TypeAdapter
import pytest

from dimos.gateway import config, events, logs
from dimos.gateway.models import DimosEvent

# Stands in for cloud_worker.py: the same arguments and MARKER lines, no network. A path with "slow" in it uploads
# until killed, "nologin" isn't logged in; the login answers "ok" once the file `<tmp>/approve` exists.
FAKE_WORKER = textwrap.dedent(
    """
    import json, os, sys, time
    def emit(value):
        print("\\n@@DIMOS_SERVER@@" + json.dumps(value), flush=True)
    command, args = sys.argv[1], sys.argv[2:]
    if command == "account":
        emit({"loggedIn": True, "email": "a@b.c", "scopes": [], "source": "stored", "cloudUrl": "x", "error": None})
    elif command == "logout":
        emit({"loggedOut": True})
    elif command == "login":
        emit({"event": "code", "url": "https://console/device", "urlComplete": None, "code": "ABCD",
              "expiresIn": 600, "interval": 1})
        while not os.path.exists(os.path.join(os.path.dirname(__file__), "approve")):
            time.sleep(0.05)
        emit({"event": "done", "status": "ok", "email": "a@b.c"})
    elif command == "upload":
        path = args[0]
        if "nologin" in path:
            emit({"event": "error", "code": "not_logged_in", "message": "Not logged in"})
            sys.exit(0)
        emit({"event": "progress", "phase": "upload", "done": 0, "total": 100})
        while "slow" in path:
            time.sleep(0.05)
        emit({"event": "progress", "phase": "upload", "done": 100, "total": 100})
        emit({"event": "result", "uploadId": "cloud-1", "state": "complete", "skipped": False, "quota": {},
              "link": "https://console.x/console/data"})
    """
)


@pytest.fixture
def server_home(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """DIMOS_HOME, the gateway's state and dimos's logs all under tmp_path; no live run registry."""
    monkeypatch.setenv("DIMOS_HOME", str(tmp_path / "home"))
    monkeypatch.delenv(config.RANGE_ENV, raising=False)
    monkeypatch.setattr(config, "gateway_dir", lambda: tmp_path / "state" / "server")
    monkeypatch.setattr(logs, "LOG_DIR", tmp_path / "state" / "logs")
    from dimos.core import run_registry
    from dimos.gateway import runs

    monkeypatch.setattr(run_registry, "REGISTRY_DIR", tmp_path / "state" / "runs")

    monkeypatch.setattr(runs, "registry_runs", lambda: [])
    # nor a real dimos run's coordinator on this machine's bus
    monkeypatch.setattr(runs, "coordinator_on_bus", lambda: False)
    return tmp_path


@pytest.fixture
def fake_worker(tmp_path: Path) -> list[str]:
    script = tmp_path / "fake_worker.py"
    script.write_text(FAKE_WORKER)
    return [sys.executable, str(script)]


@pytest.fixture
def checkout(tmp_path: Path) -> Path:
    """A dimos checkout whose `dimos` is a script that logs its arguments and sleeps until stopped."""
    root = tmp_path / "dimos"
    (root / ".venv" / "bin").mkdir(parents=True)
    (root / "pyproject.toml").write_text('[project]\nname = "dimos"\nversion = "0.0.14"\n')
    program = root / ".venv" / "bin" / "dimos"
    program.write_text(
        f"#!{sys.executable}\nimport sys, time\nprint('args', *sys.argv[1:], flush=True)\ntime.sleep(60)\n"
    )
    program.chmod(0o755)
    return root


def undeclared(value: Any, where: str = "") -> list[str]:
    """Fields a validated model carries that its class doesn't declare (models allow extras, so nothing is lost)."""
    if isinstance(value, BaseModel):
        found = [f"{where}.{name}" for name in value.model_extra or {}]
        for name in type(value).model_fields:
            found += undeclared(getattr(value, name), f"{where}.{name}")
        return found
    if isinstance(value, list):
        return [
            problem for i, item in enumerate(value) for problem in undeclared(item, f"{where}[{i}]")
        ]
    if isinstance(value, dict):
        return [
            problem
            for key, item in value.items()
            for problem in undeclared(item, f"{where}[{key}]")
        ]
    return []


def strictly(annotation: Any, value: Any, what: str) -> None:
    """`value` validates against `annotation` with no undeclared field."""
    extra = undeclared(TypeAdapter(annotation).validate_python(value), what)
    assert not extra, f"{what} has fields its model doesn't declare: {extra}"


@pytest.fixture
def check_model() -> Any:
    """`check_model(annotation, value, what)`: the strict check, for answers and events made outside a request."""
    return strictly


@pytest.fixture(autouse=True)
def no_desktop(monkeypatch: pytest.MonkeyPatch) -> None:
    """Tests never reach a real Desktop (port 9 refuses): the ones that want one fake it."""
    monkeypatch.setenv("DESKTOP_URL", "http://127.0.0.1:9")


@pytest.fixture(autouse=True)
def strict_answers(monkeypatch: pytest.MonkeyPatch) -> Iterator[None]:
    """Every answer a test gets matches its route's response model, and every event the DimosEvent schema, exactly:
    no missing, mistyped or undeclared field (so openapi.json describes what the gateway really sends)."""
    serialize = fastapi.routing.serialize_response

    async def checked(**kwargs: Any) -> Any:
        field = kwargs.get("field")
        if field is not None:
            strictly(field.field_info.annotation, kwargs["response_content"], "the answer")
        return await serialize(**kwargs)

    send = events.Bus.send

    def send_checked(bus: events.Bus, event: dict[str, Any]) -> None:
        strictly(DimosEvent, event, f"the {event.get('type')} event")
        send(bus, event)

    monkeypatch.setattr(fastapi.routing, "serialize_response", checked)
    monkeypatch.setattr(events.Bus, "send", send_checked)
    yield
