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

"""dimos.yaml's `provides:` lists every /dimos route, in the shape apps declare theirs, and is current."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest
import yaml

from dimos.gateway import provides


@pytest.fixture(scope="module")
def offered() -> dict[str, Any]:
    return provides.provides(provides.gateway_app())


def test_dimos_yaml_provides_is_current() -> None:
    assert provides.stale_problems() == []


def test_a_stale_provides_is_a_problem(tmp_path: Path) -> None:
    edited = tmp_path / "dimos.yaml"
    text = provides.DIMOS_YAML.read_text()
    edited.write_text(
        text.replace('    - {method: GET, path: "healthz"', '    - {method: GET, path: "gone"', 1)
    )
    (problem,) = provides.stale_problems(edited)
    assert provides.WRITE_COMMAND in problem
    edited.write_text(text.replace("provides:\n", "offers:\n", 1))
    assert provides.stale_problems(edited) != []


def test_it_reads_back_as_written(offered: dict[str, Any]) -> None:
    assert yaml.safe_load(provides.DIMOS_YAML.read_text())["provides"] == offered


def test_every_route_relative_to_dimos(offered: dict[str, Any]) -> None:
    endpoints = offered["endpoints"]
    assert len(endpoints) >= 50
    keys = {(e["method"], e["path"]) for e in endpoints}
    assert len(keys) == len(endpoints)
    assert {
        ("GET", "runs"),
        ("POST", "runs"),
        ("GET", "runs/{runId}/log"),
        ("GET", "msgs.js"),
    } <= keys
    for endpoint in endpoints:
        assert endpoint["method"] in {"GET", "POST", "PUT", "PATCH", "DELETE"}
        assert not endpoint["path"].startswith("/") and ".." not in endpoint["path"]
        assert endpoint["description"] and "\n" not in endpoint["description"]


def test_the_rest_of_dimos_yaml_is_kept() -> None:
    text = provides.DIMOS_YAML.read_text()
    replaced = provides.updated(text, {"description": "d", "endpoints": []})
    assert replaced.replace(
        provides.block({"description": "d", "endpoints": []}), ""
    ) == provides.BLOCK.sub("", text)
    assert yaml.safe_load(replaced)["robots"] == yaml.safe_load(text)["robots"]
