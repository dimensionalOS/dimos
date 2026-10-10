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

from collections.abc import Callable
import json
import os
from pathlib import Path
import sqlite3
import time
from typing import Any

from fastapi.testclient import TestClient
import pytest

from dimos.core.run_registry import RunEntry
from experimental.gateway import launches, overrides as ov


def until(check: Callable[[], bool], seconds: float = 10.0) -> None:
    deadline = time.monotonic() + seconds
    while not check():
        assert time.monotonic() < deadline, "timed out"
        time.sleep(0.05)


def output_has(client: TestClient, text: str) -> Callable[[], bool]:
    return lambda: text in (client.get("/dimos/runs").json()["launch"] or {}).get("output", "")


@pytest.fixture
def launched(client: TestClient) -> Any:
    yield
    launch = launches.current_launch()
    if launch and launch["phase"] in launches.ACTIVE:
        client.post("/dimos/runs/stop")


def test_launch_runs_dimos_with_saved_and_one_off_config(
    client: TestClient, launched: None
) -> None:
    client.put(
        "/dimos/global-config", json={"overrides": {"n_workers": 3, "typesafe_api_key": "k1"}}
    )
    started = client.post(
        "/dimos/runs",
        json={"blueprint": "demo", "replay": True, "overrides": {"global": {"n_workers": 4}}},
    )
    assert started.status_code == 200, started.text
    assert started.json()["phase"] == "starting"
    until(output_has(client, "secret k1"))
    launch = client.get("/dimos/runs").json()["launch"]
    assert "--n-workers=4" in launch["output"] and "--replay" in launch["output"]
    assert "k1" not in launch["output"].splitlines()[0]
    assert launch["overrides"]["typesafe_api_key"] == ov.HIDDEN
    assert launch["oneOff"]["global"] == {"n_workers": 4, "replay": True}
    again = client.post("/dimos/runs", json={"blueprint": "demo"})
    assert again.status_code == 400
    stopped = client.post("/dimos/runs/stop")
    assert stopped.status_code == 200, stopped.text
    assert client.get("/dimos/runs").json()["launch"]["phase"] == "stopped"


def test_a_failed_launch_reports_its_last_output_line(client: TestClient) -> None:
    client.post("/dimos/runs", json={"blueprint": "fail"})
    until(lambda: client.get("/dimos/runs").json()["launch"]["phase"] == "failed")
    assert client.get("/dimos/runs").json()["launch"]["error"] == "ValueError: no robot answered"


def test_restart_launches_the_last_blueprint_again(client: TestClient, launched: None) -> None:
    assert client.post("/dimos/runs/restart").status_code == 400
    client.post("/dimos/runs", json={"blueprint": "demo", "overrides": {"n_workers": 2}})
    until(output_has(client, "args"))
    first = client.get("/dimos/runs").json()["launch"]["pid"]
    restarted = client.post("/dimos/runs/restart")
    assert restarted.status_code == 200, restarted.text
    assert restarted.json()["pid"] != first
    assert restarted.json()["oneOff"]["global"] == {"n_workers": 2}


def test_runs_lists_the_registry(client: TestClient) -> None:
    entry = RunEntry(
        run_id="20260101-120000-demo",
        pid=os.getpid(),
        blueprint="demo",
        started_at="2026-01-01T12:00:00+00:00",
        log_dir="/tmp/x",
    )
    entry.save()
    runs = client.get("/dimos/runs").json()["runs"]
    assert runs == [
        {
            "run_id": "20260101-120000-demo",
            "pid": os.getpid(),
            "blueprint": "demo",
            "started_at": "2026-01-01T12:00:00+00:00",
            "log_dir": "/tmp/x",
        }
    ]
    refused = client.post("/dimos/runs", json={"blueprint": "demo"})
    assert refused.status_code == 400
    assert "stop it first" in refused.json()["error"]


def test_run_log_pages_and_tails(client: TestClient, home: Path) -> None:
    run = home / "logs" / "20260101-120000-demo"
    run.mkdir(parents=True)
    lines = [
        {"level": "info", "event": "hello", "logger": "a"},
        {"level": "error", "event": "boom", "logger": "b"},
    ]
    (run / "main.jsonl").write_text("".join(json.dumps(line) + "\n" for line in lines))
    page = client.get("/dimos/runs/latest/log").json()
    assert page["runId"] == "20260101-120000-demo"
    assert [r["event"] for r in page["records"]] == ["hello", "boom"]
    assert page["loggers"] == ["a", "b"]
    errors = client.get("/dimos/runs/20260101-120000-demo/log?level=error").json()
    assert [r["event"] for r in errors["records"]] == ["boom"]
    with (run / "main.jsonl").open("a") as handle:
        handle.write(json.dumps({"level": "info", "event": "later"}) + "\n")
    tail = client.get(f"/dimos/runs/latest/log?after={page['offset']}").json()
    assert [r["event"] for r in tail["records"]] == ["later"]


def test_replays_lists_sample_recordings(client: TestClient, checkout: Path) -> None:
    (checkout / "data").mkdir()
    with sqlite3.connect(checkout / "data" / "go2_short.db") as db:
        db.execute("CREATE TABLE _streams (name TEXT)")
        db.executemany("INSERT INTO _streams VALUES (?)", [("odom",), ("lidar",)])
        db.execute("CREATE TABLE odom (ts REAL)")
        db.executemany("INSERT INTO odom VALUES (?)", [(1.0,), (3.5,)])
    (checkout / "data" / "pointer.db").write_text("version https://git-lfs.github.com/spec/v1\n")
    replays = {r["name"]: r for r in client.get("/dimos/replays").json()["replays"]}
    assert replays["go2_short"]["streams"] == ["lidar", "odom"]
    assert replays["go2_short"]["duration"] == 2.5
    assert "LFS" in replays["pointer"]["error"]
