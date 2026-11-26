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

"""Two real HTTP host processes exercise session readiness and bundle collection."""

from contextlib import ExitStack
import json
import os
from pathlib import Path
import selectors
import subprocess
import sys
import threading
import time

import pytest
import requests
from typer.testing import CliRunner

from dimos.cli.commands.diagnostics import app
from dimos.diagnostics.analysis import load_bundle
from dimos.diagnostics.runtime import Runtime
from dimos.diagnostics.session_client import SessionClient, SessionPlan

SERVER = """
import json,socket
from fastapi import FastAPI
import uvicorn
from experimental.gateway.routes.diagnostics import router
sock=socket.socket();sock.bind(('127.0.0.1',0));sock.listen(16)
app=FastAPI();app.include_router(router)
print(json.dumps({'url':f'http://127.0.0.1:{sock.getsockname()[1]}/diagnostics'}),flush=True)
uvicorn.Server(uvicorn.Config(app,log_level='error')).run(sockets=[sock])
"""


@pytest.fixture
def hosts(tmp_path):
    with ExitStack() as cleanup:
        endpoints = {}
        for name in ("robot", "operator"):
            root = tmp_path / name
            process = subprocess.Popen(
                [sys.executable, "-c", SERVER],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env={**os.environ, "DIMOS_DIAGNOSTICS_DIR": str(root), "DIMOS_DIAGNOSTICS": "off"},
            )

            def stop(child=process):
                child.terminate()
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=5)
                child.stdout.close()
                child.stderr.close()

            cleanup.callback(stop)
            with selectors.DefaultSelector() as selector:
                selector.register(process.stdout, selectors.EVENT_READ)
                assert selector.select(20), "host failed to announce its endpoint"
                line = process.stdout.readline()
            assert line, process.stderr.read()
            url = json.loads(line)["url"]
            deadline = time.monotonic() + 10
            while True:
                try:
                    assert requests.get(url + "/session", timeout=1).status_code == 200
                    break
                except requests.ConnectionError:
                    assert time.monotonic() < deadline
                    threading.Event().wait(0.02)
            endpoints[name] = {"url": url, "root": root}
        yield endpoints


def make_client(hosts):
    return SessionClient(
        SessionPlan.model_validate(
            {
                "run_id": "lap",
                "hosts": {
                    "robot": {
                        "url": hosts["robot"]["url"],
                        "required": [
                            {"name": "runtime", "kind": "python", "event": "test.compute.end"}
                        ],
                    },
                    "operator": {
                        "url": hosts["operator"]["url"],
                        "required": [
                            {"name": "viewer", "kind": "browser", "event": "video.presented"}
                        ],
                    },
                },
            }
        )
    )


def test_two_hosts_require_browser_then_collect_one_bundle(hosts, tmp_path):
    client = make_client(hosts)
    assert client.start()["started"] is True
    assert client.status()["ready"] is False
    runtime = Runtime("lap", hosts["robot"]["root"] / "lap")
    try:
        runtime.event("test.compute.end", duration_ns=42)
        manifest = {
            "schema": "dimos.incident/v1",
            "run_id": "lap",
            "process": "browser-test",
            "clock_domain": "browser/performance",
        }
        event = {
            "schema": "dimos.trace/v1",
            "event": "video.presented",
            "process": "browser-test",
            "clock_domain": "browser/performance",
            "mono_ns": 10,
            "context": None,
        }
        client.request(
            "operator",
            "POST",
            "/browser",
            json={"batchId": "one", "manifest": manifest, "events": [event]},
        )
        deadline = time.monotonic() + 10
        while not client.status()["ready"]:
            assert time.monotonic() < deadline, client.status()
            threading.Event().wait(0.05)
        result = client.collect(tmp_path / "bundle")
        assert result["complete"] is True
        manifests, events = load_bundle(tmp_path / "bundle")
        assert {m["run_id"] for m in manifests} == {"lap"}
        assert {"test.compute.end", "video.presented"} <= {e["event"] for e in events}
        assert set(result["hosts"]) == {"robot", "operator"}
        assert all("error" not in v for v in client.control("stop").values())
        assert client.status()["ready"] is False
    finally:
        runtime.close()


def test_missing_endpoint_prevents_starting_other_hosts(hosts):
    client = make_client(hosts)
    client.plan.hosts["operator"].url = hosts["operator"]["url"] + "/missing"
    assert client.start()["started"] is False
    assert requests.get(hosts["robot"]["url"] + "/session", timeout=2).json()["session"] is None


def test_sdk_uploads_to_runtime_without_an_app_trace_server(hosts, tmp_path):
    # Run the shared uploader against the actual authenticated-route implementation.
    sdk = os.environ.get("DIMOS_TEST_GATEWAY_SDK")
    if not sdk:
        pytest.skip("set DIMOS_TEST_GATEWAY_SDK to the shared gateway client directory")
    client = make_client(hosts)
    assert client.start()["started"] is True
    source = tmp_path / "browser.ts"
    source.write_text(f"""
import {{openDiagnostics}} from {json.dumps((Path(sdk) / "diagnostics_session.ts").as_uri())};
const session=await openDiagnostics(new URL({json.dumps(hosts["operator"]["url"] + "/")}));
if(!session)throw new Error('missing session');
session.trace.event('video.presented');
session.upload.credit=256*1024;
try{{await session.upload.flush(session.trace);}}finally{{session.close();}}
""")
    result = subprocess.run(
        ["deno", "run", "--allow-net", str(source)], capture_output=True, text=True, timeout=15
    )
    assert result.returncode == 0, result.stderr
    status = client.status()
    assert status["hosts"]["operator"]["ready"] is True
    assert status["hosts"]["robot"]["ready"] is False
    manifests, events = load_bundle(hosts["operator"]["root"] / "lap")
    assert manifests[0]["collector"] == "browser"
    assert "video.presented" in {e["event"] for e in events}


def test_session_cli_launches_normal_command_with_shared_run_only_after_preflight(hosts, tmp_path):
    client = make_client(hosts)
    config = tmp_path / "session.json"
    config.write_text(client.plan.model_dump_json())
    launched = tmp_path / "child.json"
    script = (
        "import json,os;from pathlib import Path;Path("
        + repr(str(launched))
        + ").write_text(json.dumps({key:os.environ.get(key) for key in ['DIMOS_DIAGNOSTICS','DIMOS_DIAGNOSTICS_RUN']}))"
    )
    result = CliRunner().invoke(
        app, ["session", "start", str(config), "--", sys.executable, "-c", script]
    )
    assert result.exit_code == 0, result.output
    assert json.loads(launched.read_text()) == {
        "DIMOS_DIAGNOSTICS": "trace",
        "DIMOS_DIAGNOSTICS_RUN": "lap",
    }
    launched.unlink()
    client.plan.hosts["operator"].url += "/missing"
    config.write_text(client.plan.model_dump_json())
    result = CliRunner().invoke(
        app, ["session", "start", str(config), "--", sys.executable, "-c", script]
    )
    assert result.exit_code == 2
    assert not launched.exists()
