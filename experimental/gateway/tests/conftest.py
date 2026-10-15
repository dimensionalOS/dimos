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
import json
from pathlib import Path
import sys
from typing import Any

from fastapi.testclient import TestClient
import pytest

from dimos.core import run_registry
from experimental.gateway import events, launches, logs, store
from experimental.gateway.app import create_app
from experimental.gateway.state import State, new_state

FAKE_DIMOS = """#!{python}
import os, sys, time
print("args", *sys.argv[1:], flush=True)
print("secret", os.environ.get("TYPESAFE_API_KEY"), flush=True)
if "fail" in sys.argv:
    print("ValueError: no robot answered", flush=True)
    sys.exit(1)
time.sleep(60)
"""


@pytest.fixture
def home(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    monkeypatch.setattr(store, "STATE_DIR", tmp_path / "state")
    monkeypatch.setattr(run_registry, "REGISTRY_DIR", tmp_path / "state" / "runs")
    monkeypatch.setattr(logs, "LOG_DIR", tmp_path / "logs")
    monkeypatch.setattr(launches, "LOG_DIR", tmp_path / "logs")
    monkeypatch.setattr(launches, "coordinator_on_bus", lambda: False)
    monkeypatch.delenv("DIMOS_ZENOH_NAMESPACE", raising=False)
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
def client(state: State) -> Iterator[TestClient]:
    with TestClient(create_app(state, background=False)) as test_client:
        yield test_client
