# Copyright 2025-2026 Dimensional Inc.
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

"""Diagnostics routes expose bounded local evidence without accepting arbitrary paths."""

import json

from fastapi import FastAPI
from fastapi.testclient import TestClient
import pytest

from dimos.diagnostics.store import DiagnosticsStore
from experimental.gateway.routes.diagnostics import router


@pytest.fixture
def client(tmp_path, monkeypatch):
    monkeypatch.setenv("DIMOS_DIAGNOSTICS_DIR", str(tmp_path))
    app = FastAPI()
    app.include_router(router)
    with TestClient(app) as client:
        yield client


def test_status_topology_and_incidents(client, tmp_path):
    directory = tmp_path / "run" / "process"
    directory.mkdir(parents=True)
    (directory / "status.json").write_text(
        json.dumps({"process": "p", "at_ns": 12, "topology": [{"channel": "hand"}]})
    )
    (directory / "incident-1.jsonl").write_text("{}\n")
    assert client.get("/diagnostics/run/status").json()[0]["process"] == "p"
    assert client.get("/diagnostics/run/topology").json()["processes"][0]["edges"] == [
        {"channel": "hand"}
    ]
    assert client.get("/diagnostics/run/incidents").json() == [
        {"id": "process/incident-1.jsonl", "bytes": 3}
    ]


def test_missing_run_is_empty_not_healthy(client):
    assert client.get("/diagnostics/missing/status").json() == []
    assert (
        client.get("/diagnostics/capabilities").json()["physical_display"]
        == "optical_test_required"
    )


def test_store_rejects_traversal_and_symlinks(tmp_path):
    root = tmp_path / "root"
    root.mkdir()
    store = DiagnosticsStore(root)
    with pytest.raises(ValueError):
        store.run("../outside")
    (root / "escape").symlink_to(tmp_path)
    with pytest.raises(ValueError):
        store.run("escape")


def test_clock_endpoint_and_incident_download(client, tmp_path):
    reply = client.get("/diagnostics/clock").json()
    assert reply["schema"] == "dimos.clock/v1"
    assert reply["sent_ns"] >= reply["received_ns"]
    directory = tmp_path / "run" / "process"
    directory.mkdir(parents=True)
    payload = '{"kind":"manifest","process":"process"}\n'
    (directory / "incident-one.jsonl").write_text(payload)
    assert client.get("/diagnostics/run/incidents/process/incident-one.jsonl").text == payload
    assert (
        client.get("/diagnostics/run/incidents/process/incident-missing.jsonl").status_code == 404
    )
    assert (
        client.get("/diagnostics/run/incidents/process/incident-one.jsonl/analysis").json()[
            "qualification"
        ]
        == "incomplete"
    )
