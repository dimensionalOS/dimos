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
from contextlib import suppress
import json
import os
from pathlib import Path
import signal
import sys
from typing import Any

from fastapi.testclient import TestClient
import pytest

from dimos.core import run_registry
from experimental.gateway import events, launches, logs, store
from experimental.gateway.app import create_app
from experimental.gateway.state import State, new_state

FAKE_DIMOS = """#!{python}
import json, os, pathlib, sys, time
print("args", *sys.argv[1:], flush=True)
print("secret", os.environ.get("AGENT__API_KEY"), flush=True)
print("dtop", os.environ.get("DTOP"), flush=True)
print("gossip", os.environ.get("ZENOH_GOSSIP"), flush=True)
if "fail" in sys.argv:
    print("ValueError: no robot answered", flush=True)
    sys.exit(1)
if "slow" in sys.argv:
    # a startup that never gets as far as registering
    time.sleep(60)
    sys.exit(0)
# registers like `dimos run` does, under the id it picks itself
blueprint = sys.argv[sys.argv.index("run") + 1]
run_id = time.strftime("%Y%m%d-%H%M%S-") + blueprint + "-" + str(os.getpid())
registry = pathlib.Path(os.environ["FAKE_REGISTRY_DIR"])
registry.mkdir(parents=True, exist_ok=True)
entry = dict(run_id=run_id, pid=os.getpid(), blueprint=blueprint, started_at="2026-01-01T12:00:00+00:00", log_dir="/tmp")
# atomically, like RunEntry.save: the gateway's polling deletes an entry it reads half written
(registry / (run_id + ".tmp")).write_text(json.dumps(entry))
(registry / (run_id + ".tmp")).replace(registry / (run_id + ".json"))
time.sleep(60)
"""


@pytest.fixture
def home(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    monkeypatch.setattr(store, "STATE_DIR", tmp_path / "state")
    monkeypatch.setattr(run_registry, "REGISTRY_DIR", tmp_path / "state" / "runs")
    monkeypatch.setenv("FAKE_REGISTRY_DIR", str(tmp_path / "state" / "runs"))
    monkeypatch.setattr(logs, "LOG_DIR", tmp_path / "logs")
    monkeypatch.setattr(launches, "LOG_DIR", tmp_path / "logs")
    # no ~/.dimos/config.yaml of the machine running the tests
    monkeypatch.setenv("DIMOS_HOME", str(tmp_path / "dimos_home"))
    # nor its DTOP, which a launch passes on
    monkeypatch.delenv(launches.DTOP_ENV, raising=False)
    # nor a git repo the temp dir happens to be inside
    monkeypatch.setenv("GIT_CEILING_DIRECTORIES", str(tmp_path))
    return tmp_path


@pytest.fixture
def checkout(home: Path) -> Path:
    root = home / "dimos"
    (root / ".venv" / "bin").mkdir(parents=True)
    (root / "pyproject.toml").write_text('[project]\nname = "dimos"\nversion = "0.0.14"\n')
    program = root / ".venv" / "bin" / "dimos"
    program.write_text(FAKE_DIMOS.format(python=sys.executable))
    program.chmod(0o755)
    return root


@pytest.fixture
def sent() -> list[tuple[str, Any]]:
    return []


@pytest.fixture
def state(checkout: Path, sent: list[tuple[str, Any]]) -> State:
    bus = events.Bus("ns", lambda key, payload: sent.append((key, json.loads(payload))))
    return new_state(checkout, bus)


@pytest.fixture
def client(state: State, monkeypatch: pytest.MonkeyPatch) -> Iterator[TestClient]:
    spawned: list[tuple[int, float | None]] = []
    spawn = launches._spawn

    def remembered(*args: Any) -> int:
        pid = spawn(*args)
        spawned.append((pid, launches.started_time(pid)))
        return pid

    monkeypatch.setattr(launches, "_spawn", remembered)
    with TestClient(create_app(state, background=False)) as test_client:
        yield test_client
    # what a test launched dies with it (the fake dimos sleeps a minute), unless its pid now names another process
    for pid, started in spawned:
        if launches.started_time(pid) == started:
            with suppress(ProcessLookupError, PermissionError):
                os.killpg(pid, signal.SIGKILL)
