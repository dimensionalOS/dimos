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

"""Runs on this computer that this gateway didn't start: found, listed, their logs read, and stopped."""

from __future__ import annotations

from dataclasses import asdict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from fastapi.testclient import TestClient
import pytest

from experimental.gateway.server.app import create_app
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import events, local_runs, runs
from experimental.gateway.utils.uploads import Uploads

# a stand-in for `dimos run`: its argv reads like one, it exits on Ctrl-C like one
FAKE_RUN = """
import signal, sys, time
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
open(sys.argv[1], "a").close()
while True:
    time.sleep(0.1)
"""


def test_run_argv_reads_dimos_run_command_lines() -> None:
    script = [
        "/x/.venv/bin/python3",
        "/x/.venv/bin/dimos",
        "--replay",
        "run",
        "unitree-go2",
        "--a.b=1",
    ]
    assert local_runs.run_argv(script) == [
        "/x/.venv/bin/dimos",
        "--replay",
        "run",
        "unitree-go2",
        "--a.b=1",
    ]
    assert local_runs.run_argv(["python", "-m", "dimos", "run", "xarm"]) == ["dimos", "run", "xarm"]
    assert local_runs.run_argv(["python", "-m", "experimental.gateway", "--socket", "s"]) is None
    assert local_runs.run_argv(["/x/bin/dimos", "status"]) is None
    assert (
        local_runs.blueprint_of(["dimos", "--replay", "run", "unitree-go2", "--x=1"])
        == "unitree-go2"
    )


def test_registry_dir_follows_the_processes_state_home(tmp_path: Path) -> None:
    assert local_runs.registry_dir({"XDG_STATE_HOME": str(tmp_path)}) == tmp_path / "dimos" / "runs"
    assert local_runs.registry_dir({"HOME": str(tmp_path)}) == tmp_path / ".local/state/dimos/runs"


def test_an_entry_older_than_its_process_is_a_reused_pid() -> None:
    now = time.time()
    registered = datetime.fromtimestamp(now, timezone.utc).isoformat()
    assert local_runs.same_process(registered, now - 30)
    assert not local_runs.same_process(registered, now + 60)


@pytest.fixture
def client(server_home: Path, checkout: Path, fake_worker: list[str]):
    bus = events.Bus()
    uploads = Uploads(checkout, bus, None, server_home / "uploads.log", worker=fake_worker)
    with TestClient(
        create_app(ServerState(dimos_dir=checkout, bus=bus, uploads=uploads), background=False)
    ) as client:
        yield client


@pytest.fixture
def other_home_run(tmp_path: Path):
    """A `dimos ... run` process registered in another DIMOS_HOME's registry, as a test Desktop or an agent makes."""
    state = tmp_path / "other_state"
    script = tmp_path / "bin" / "dimos"
    script.parent.mkdir()
    script.write_text(FAKE_RUN)
    log_dir = tmp_path / "other_logs" / "20260101-120000-unitree-go2"
    log_dir.mkdir(parents=True)
    (log_dir / "main.jsonl").write_text(
        json.dumps(
            {"timestamp": "2026-01-01T12:00:00Z", "level": "info", "logger": "x", "event": "hello"}
        )
        + "\n"
    )
    ready = tmp_path / "ready"
    process = subprocess.Popen(
        [sys.executable, str(script), str(ready), "--replay", "run", "unitree-go2"],
        env={**os.environ, "XDG_STATE_HOME": str(state)},
        start_new_session=True,
    )
    # reaped as soon as it exits, as a run some other shell started would be (a zombie still counts as alive)
    exit_codes: list[int] = []
    threading.Thread(target=lambda: exit_codes.append(process.wait()), daemon=True).start()
    deadline = time.monotonic() + 10
    while not ready.exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    from dimos.core.run_registry import RunEntry

    entry = RunEntry(
        run_id=log_dir.name,
        pid=process.pid,
        blueprint="unitree-go2",
        started_at=datetime.now(timezone.utc).isoformat(),
        log_dir=str(log_dir),
    )
    (state / "dimos" / "runs").mkdir(parents=True)
    (state / "dimos" / "runs" / f"{entry.run_id}.json").write_text(json.dumps(asdict(entry)))
    yield process, entry, exit_codes
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def test_another_homes_run_is_listed_its_log_read_and_it_stops(
    client: TestClient, monkeypatch: pytest.MonkeyPatch, other_home_run
) -> None:
    process, entry, exit_codes = other_home_run
    from dimos.core.run_registry import REGISTRY_DIR

    # the real scan, kept to this test's own process
    monkeypatch.setattr(
        runs,
        "other_runs",
        lambda: [r for r in local_runs.scan(REGISTRY_DIR) if r["pid"] == process.pid],
    )
    listed = client.get("/dimos/runs").json()["runs"]
    assert len(listed) == 1, listed
    run = listed[0]
    assert run["run_id"] == entry.run_id and run["blueprint"] == "unitree-go2"
    assert run["registry"].endswith("other_state/dimos/runs")
    assert run["ours"] is False and run["stoppable"] is True and run["whyNot"] is None
    assert "run unitree-go2" in run["command"]

    log = client.get(f"/dimos/runs/{entry.run_id}/log").json()
    assert [record["event"] for record in log["records"]] == ["hello"]

    stopped = client.post("/dimos/runs/stop", json={"runId": entry.run_id})
    assert stopped.status_code == 200, stopped.text
    assert exit_codes == [0], "stopped with Ctrl-C first, as a terminal would"
    assert client.get("/dimos/runs").json()["runs"] == []


def test_an_unregistered_run_is_listed_by_its_process(server_home: Path, other_home_run) -> None:
    process, entry, _ = other_home_run
    for file in (Path(entry.log_dir).parents[1] / "other_state" / "dimos" / "runs").glob("*.json"):
        file.unlink()
    from dimos.core.run_registry import REGISTRY_DIR

    found = [r for r in local_runs.scan(REGISTRY_DIR) if r["pid"] == process.pid]
    assert len(found) == 1
    assert found[0]["registry"] is None and found[0]["blueprint"] == "unitree-go2"
    assert found[0]["run_id"] == f"pid-{process.pid}"


def test_another_users_run_cant_be_stopped(
    client: TestClient, monkeypatch: pytest.MonkeyPatch
) -> None:
    theirs = {
        "run_id": "20260101-120000-xarm",
        "pid": 999999,
        "blueprint": "xarm",
        "started_at": "2026-01-01T12:00:00+00:00",
        "log_dir": "",
        "registry": None,
        "owner": "someone",
        "command": None,
        "ours": False,
        "stoppable": False,
        "whyNot": "started by someone; only they (or root) can stop it",
    }
    monkeypatch.setattr(runs, "other_runs", lambda: [theirs])
    assert client.get("/dimos/runs").json()["runs"] == [theirs]
    refused = client.post("/dimos/runs/stop", json={"runId": theirs["run_id"]})
    assert refused.status_code == 500 and "only they" in refused.json()["error"]


def test_a_run_heard_only_on_the_bus_is_listed_apart(
    server_home: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(runs, "coordinator_reachable", lambda connect: True)
    watch = local_runs.BusWatch(every=0)
    watch._probe(False, ["tcp/10.0.0.2:7447"])
    assert watch.get([], ["tcp/10.0.0.2:7447"]) == [
        {
            "where": "network",
            "peer": "tcp/10.0.0.2:7447",
            "note": "a dimos run answers through the zenoh connection; it runs on that computer, stop it there",
        }
    ]
    # a coordinator answering locally that a listed run accounts for is no news
    monkeypatch.setattr(runs, "coordinator_on_bus", lambda: True)
    assert local_runs.probe(True, []) == []
    assert local_runs.probe(False, [])[0]["where"] == "local"
