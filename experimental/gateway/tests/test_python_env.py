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

"""GET /dimos/python: the python dimos runs with, and that its command and env really import the checkout's dimos."""

from collections.abc import Iterator
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any

from fastapi.testclient import TestClient
import pytest

import dimos
from experimental.gateway.server.app import create_app
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import events, python_env
from experimental.gateway.utils.uploads import Uploads

REPO = Path(dimos.__file__).resolve().parent.parent


@pytest.fixture(autouse=True)
def fresh_cache() -> Iterator[None]:
    python_env._cache.clear()
    yield
    python_env._cache.clear()


def answer_for(dimos_dir: Path, server_home: Path) -> Any:
    bus = events.Bus()
    uploads = Uploads(dimos_dir, bus, None, server_home / "uploads.log", worker=["true"])
    state = ServerState(dimos_dir=dimos_dir, bus=bus, uploads=uploads)
    with TestClient(create_app(state, background=False)) as client:
        return client.get("/dimos/python")


def imported_from(answer: dict[str, Any], cwd: Path) -> str:
    """Where `import dimos` comes from with the answer's command and env, run elsewhere like an agent would."""
    env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"} | answer["env"]
    done = subprocess.run(
        [*answer["command"], "-c", "import dimos; print(dimos.__file__)"],
        env=env,
        cwd=cwd,
        capture_output=True,
        text=True,
        timeout=60,
        check=True,
    )
    return done.stdout.strip()


def test_the_repo_checkout(server_home: Path, tmp_path: Path) -> None:
    response = answer_for(REPO, server_home)
    assert response.status_code == 200
    answer = response.json()
    assert set(answer) == {
        "python",
        "command",
        "dimosDir",
        "version",
        "dimosVersion",
        "env",
        "example",
    }
    python = answer["python"]
    assert os.path.isabs(python) and os.access(python, os.X_OK)
    assert answer["command"] == [python]
    assert answer["dimosDir"] == str(REPO)
    assert re.fullmatch(r"3\.\d+\.\d+\S*", answer["version"])
    assert set(answer["env"]) <= {"PYTHONPATH"}
    assert python in answer["example"]
    assert Path(imported_from(answer, tmp_path)).is_relative_to(REPO)


def test_a_checkout_only_pythonpath_finds(server_home: Path, tmp_path: Path) -> None:
    """A venv python that doesn't have the checkout installed: it's used with PYTHONPATH=<checkout>."""
    checkout = tmp_path / "other"
    (checkout / "dimos").mkdir(parents=True)
    (checkout / "dimos" / "__init__.py").write_text("")
    (checkout / "pyproject.toml").write_text('[project]\nname = "dimos"\nversion = "9.9.9"\n')
    (checkout / ".venv" / "bin").mkdir(parents=True)
    # the base python (no pyvenv.cfg beside it), so it has no dimos of its own
    (checkout / ".venv" / "bin" / "python").symlink_to(os.path.realpath(sys.executable))
    answer = answer_for(checkout, server_home).json()
    assert answer["python"] == str(checkout / ".venv" / "bin" / "python")
    assert answer["env"] == {"PYTHONPATH": str(checkout)}
    assert answer["dimosVersion"] == "9.9.9"
    assert answer["example"].startswith(f"PYTHONPATH={checkout} {checkout}/.venv/bin/python -c ")
    assert imported_from(answer, tmp_path) == str(checkout / "dimos" / "__init__.py")


def test_no_python_imports_the_checkout(server_home: Path, tmp_path: Path) -> None:
    empty = tmp_path / "empty"
    empty.mkdir()
    response = answer_for(empty, server_home)
    assert response.status_code == 500
    assert "no python imports dimos" in response.json()["error"]
    assert not python_env._cache


def test_found_once_then_cached(monkeypatch: pytest.MonkeyPatch) -> None:
    probes: list[str] = []
    real = python_env.probe

    def counted(python: str, extra: dict[str, str]) -> dict[str, Any] | None:
        probes.append(python)
        return real(python, extra)

    monkeypatch.setattr(python_env, "probe", counted)
    first = python_env.python_command(REPO)
    count = len(probes)
    assert count >= 1
    assert python_env.python_command(REPO) is first
    assert len(probes) == count
