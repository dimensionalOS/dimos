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
import subprocess
import sys
import threading
import time
from typing import Any

from fastapi.testclient import TestClient
import psutil
import pytest

from dimos.core.run_registry import RunEntry
from experimental.gateway import launches


def until(check: Callable[[], bool], seconds: float = 10.0) -> None:
    deadline = time.monotonic() + seconds
    while not check():
        assert time.monotonic() < deadline, "timed out"
        time.sleep(0.05)


def launch_of(client: TestClient) -> dict[str, Any]:
    return client.get("/active").json()["launch"] or {}


def output_has(client: TestClient, text: str) -> Callable[[], bool]:
    return lambda: text in launch_of(client).get("output", "")


def running(client: TestClient) -> Callable[[], bool]:
    return lambda: bool(client.get("/active").json()["running"])


def stopped(client: TestClient) -> Callable[[], bool]:
    return lambda: launch_of(client).get("phase") == "stopped"


def test_launch_runs_dimos_with_exactly_the_config_it_is_sent(client: TestClient) -> None:
    assert client.get("/active").json() == {
        "running": False,
        "runId": None,
        "pid": None,
        "blueprint": None,
        "startedAt": None,
        "logDir": None,
        "launch": None,
    }
    assert client.post("/active/stop").status_code == 409
    config = {"global": {"n_workers": 4, "replay": True}, "modules": {"agent": {"api_key": "k1"}}}
    started = client.post("/active/launch", json={"blueprint": "demo", "config": config})
    assert started.status_code == 200, started.text
    assert started.json()["phase"] == "starting"
    until(output_has(client, "secret k1"))
    until(output_has(client, "dtop 1"))
    until(running(client))
    now = client.get("/active").json()
    launch = now["launch"]
    assert now["blueprint"] == "demo" and launch["runId"] == now["runId"]
    assert client.get("/active/id").json() == {"runId": now["runId"]}
    first = launch["output"].splitlines()[0]
    assert "--n-workers=4" in first and "--replay" in first and "run demo" in first
    assert "k1" not in first and "--run-id" not in first
    assert launch["config"] == {
        "global": config["global"],
        "modules": {"agent": {"api_key": launches.HIDDEN}},
    }
    assert client.post("/active/launch", json={"blueprint": "demo"}).status_code == 409
    out = client.post("/active/stop")
    assert out.status_code == 200, out.text
    assert out.json()["output"].startswith(
        f"Stopping {now['runId']} (PID {now['pid']}) with SIGTERM"
    )
    until(stopped(client))
    assert client.get("/active/id").status_code == 409


def test_a_bad_config_or_name_is_refused(client: TestClient) -> None:
    assert client.post("/active/launch", json={}).status_code == 400
    assert (
        client.post("/active/launch", json={"blueprint": "demo", "module": "cam"}).status_code
        == 400
    )
    assert client.post("/active/launch", json={"blueprint": "-x"}).status_code == 400
    bad = client.post(
        "/active/launch", json={"blueprint": "demo", "config": {"global": {"robot_iq": 1}}}
    )
    assert bad.status_code == 400 and "robot_iq" in bad.json()["error"]
    assert client.post("/active/exec", json={"args": ["--n-workers=7"]}).status_code == 400


def test_a_module_launches_like_a_blueprint_and_exec_relaunches_as_typed(
    client: TestClient,
) -> None:
    started = client.post("/active/launch", json={"module": "cam", "config": {"n_workers": 3}})
    assert started.status_code == 200, started.text
    until(output_has(client, "args"))
    first = launch_of(client)["output"].splitlines()[0]
    assert "run cam" in first and "--n-workers=3" in first
    until(running(client))
    client.post("/active/stop")
    until(stopped(client))
    typed = client.post("/active/exec", json={"args": ["demo", "--n-workers=7"]})
    assert typed.status_code == 200, typed.text
    until(running(client))
    assert launch_of(client)["output"].splitlines()[0].endswith("run demo --n-workers=7")
    old = client.get("/active").json()["runId"]
    again = client.post("/active/relaunch")
    assert again.status_code == 200, again.text
    until(running(client))
    assert client.get("/active").json()["runId"] != old
    assert launch_of(client)["output"].splitlines()[0].endswith("run demo --n-workers=7")


def test_relaunch_starts_the_last_launch_with_its_secrets(client: TestClient) -> None:
    assert client.post("/active/relaunch").status_code == 409
    config = {"global": {"n_workers": 2}, "modules": {"agent": {"api_key": "k2"}}}
    client.post("/active/launch", json={"blueprint": "demo", "config": config})
    until(running(client))
    first = client.get("/active").json()["runId"]
    relaunched = client.post("/active/relaunch")
    assert relaunched.status_code == 200, relaunched.text
    until(output_has(client, "secret k2"))
    until(running(client))
    second = client.get("/active").json()["runId"]
    assert second != first and launch_of(client)["config"]["global"] == {"n_workers": 2}
    client.post("/active/stop")
    until(stopped(client))
    # a stopped launch relaunches too
    assert client.post("/active/relaunch").status_code == 200
    until(running(client))
    assert client.get("/active").json()["runId"] not in (first, second)


def test_relaunch_restarts_a_launch_stuck_starting(client: TestClient) -> None:
    started = client.post("/active/launch", json={"blueprint": "slow"})
    assert started.status_code == 200, started.text
    stuck_pid = started.json()["pid"]
    until(output_has(client, "args"))
    assert launch_of(client)["phase"] == "starting"
    relaunched = client.post("/active/relaunch")
    assert relaunched.status_code == 200, relaunched.text
    assert relaunched.json()["phase"] == "starting" and relaunched.json()["pid"] != stuck_pid
    assert not launches.alive(stuck_pid)


def test_relaunch_copies_a_run_the_gateway_did_not_launch(
    client: TestClient, tmp_path: Path
) -> None:
    command = [sys.executable, "-c", "import time; time.sleep(60)", "run", "outside"]
    env = {**os.environ, "OUTSIDE_RUN": "yes"}
    outside = subprocess.Popen(command, cwd=tmp_path, env=env, start_new_session=True)
    threading.Thread(target=outside.wait, daemon=True).start()
    RunEntry(
        run_id="20260101-120000-outside",
        pid=outside.pid,
        blueprint="outside",
        started_at="2026-01-01T12:00:00+00:00",
        log_dir=str(tmp_path),
    ).save()
    assert client.get("/active").json()["blueprint"] == "outside"
    relaunched = client.post("/active/relaunch")
    assert relaunched.status_code == 200, relaunched.text
    # the stop reaped it; poll() could miss a reap the waiting thread has not finished recording
    assert outside.wait() is not None
    again = psutil.Process(relaunched.json()["pid"])
    assert again.cmdline()[-2:] == ["run", "outside"]
    assert again.cwd() == str(tmp_path.resolve())
    assert again.environ()["OUTSIDE_RUN"] == "yes"
    assert relaunched.json()["blueprint"] == "outside"


def test_a_run_started_elsewhere_is_the_active_one(client: TestClient) -> None:
    RunEntry(
        run_id="20260101-120000-demo",
        pid=os.getpid(),
        blueprint="demo",
        started_at="2026-01-01T12:00:00+00:00",
        log_dir="/tmp/x",
    ).save()
    now = client.get("/active").json()
    assert (now["running"], now["runId"], now["pid"], now["launch"]) == (
        True,
        "20260101-120000-demo",
        os.getpid(),
        None,
    )
    refused = client.post("/active/launch", json={"blueprint": "demo"})
    assert refused.status_code == 409 and "stop it first" in refused.json()["error"]


def test_a_failed_launch_reports_its_last_output_line(client: TestClient) -> None:
    client.post("/active/launch", json={"blueprint": "fail"})
    until(lambda: launch_of(client).get("phase") == "failed")
    assert launch_of(client)["error"] == "ValueError: no robot answered"


def test_logs_of_the_active_run_and_past_ones(client: TestClient, home: Path) -> None:
    assert client.get("/active/log").status_code == 409
    assert client.get("/history").json() == {"runs": []}
    assert client.get("/history/latest/logs").status_code == 404
    for run_id in ("20260101-110000-old", "20260101-120000-demo"):
        (home / "logs" / run_id).mkdir(parents=True)
    run = home / "logs" / "20260101-120000-demo"
    lines = [
        {"timestamp": "2026-01-01T12:00:01", "level": "info", "event": "hello", "logger": "a"},
        {"timestamp": "2026-01-01T12:00:02", "level": "error", "event": "boom", "logger": "b"},
    ]
    (run / "main.jsonl").write_text("".join(json.dumps(line) + "\n" for line in lines))
    (home / "logs" / "20260101-110000-old" / "main.jsonl").write_text("")
    page = client.get("/history/latest/logs").json()
    assert page["runId"] == "20260101-120000-demo"
    assert [record["event"] for record in page["records"]] == ["hello", "boom"]
    assert page["loggers"] == ["a", "b"]
    past = client.get("/history").json()["runs"]
    assert [past_run["runId"] for past_run in past] == [
        "20260101-120000-demo",
        "20260101-110000-old",
    ]
    assert past[0]["endedAt"] == "2026-01-01T12:00:02" and not past[0]["running"]
    # neither is in the registry: the blueprint comes from the id, the start from the log
    assert (past[0]["blueprint"], past[0]["startedAt"]) == ("demo", "2026-01-01T12:00:01")
    assert (past[1]["blueprint"], past[1]["startedAt"]) == ("old", None)
    assert client.get("/history/latest").json()["logDir"] == str(run)
    assert client.get("/history/20990101-000000-nope").status_code == 404
    errors = client.get("/history/20260101-120000-demo/logs?level=error").json()
    assert [record["event"] for record in errors["records"]] == ["boom"]
    assert client.get("/history/20990101-000000-nope/logs").status_code == 404
    RunEntry(
        run_id="20260101-120000-demo",
        pid=os.getpid(),
        blueprint="demo",
        started_at="2026-01-01T12:00:00+00:00",
        log_dir=str(run),
    ).save()
    with (run / "main.jsonl").open("a") as handle:
        handle.write(json.dumps({"level": "info", "event": "later"}) + "\n")
    tail = client.get(f"/active/log?after={page['offset']}").json()
    assert [record["event"] for record in tail["records"]] == ["later"]
    assert client.get("/history").json()["runs"][0]["running"]


def test_recordings_is_the_standard_directory(
    client: TestClient, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    assert client.get("/history/recordings").json() == {"path": str(launches.store.RECORDINGS_DIR)}
    (tmp_path / "dimos_home").mkdir(exist_ok=True)
    (tmp_path / "dimos_home" / "config.yaml").write_text(
        f"recordings:\n  dir: {tmp_path / 'recs'}\n"
    )
    assert client.get("/history/recordings").json() == {"path": str(tmp_path / "recs")}


def test_dtop_env_is_the_one_dimos_reads(monkeypatch: pytest.MonkeyPatch) -> None:
    from dimos.core.global_config import GlobalConfig

    monkeypatch.setenv(launches.DTOP_ENV, "1")
    assert GlobalConfig().dtop is True


def test_a_reused_pid_is_not_the_launch(client: TestClient) -> None:
    launches._save({"blueprint": "demo", "started_at": "x", "pid": os.getpid(), "pid_started": 0.0})
    assert launches.current_launch()["phase"] == "failed"


def test_shared_config_yaml_sets_the_namespace_connect_socket_and_gossip(
    client: TestClient, tmp_path: Path
) -> None:
    from experimental.gateway import events, store

    assert (events.resolve_namespace(None), store.setting("dimos-gateway", "socket")) == (
        None,
        None,
    )
    (tmp_path / "dimos_home").mkdir(exist_ok=True)
    (tmp_path / "dimos_home" / "config.yaml").write_text(
        "zenoh: {namespace: desk1, connect: 'tcp/10.0.0.2:7447', gossip: false}\n"
        "dimos-gateway: {socket: /tmp/gw.sock}\ndesktop: {port: 5555}\n"
    )
    assert events.resolve_namespace(None) == "desk1"
    assert events.resolve_namespace("given") == "given"
    assert events.resolve_connect(None) == ["tcp/10.0.0.2:7447"]
    assert store.setting("dimos-gateway", "socket") == "/tmp/gw.sock"
    client.post("/active/launch", json={"blueprint": "demo"})
    until(output_has(client, "gossip false"))
