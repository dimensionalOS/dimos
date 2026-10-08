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


"""The /dimos routes, with fakes: a temporary DIMOS_HOME and state, a fake `dimos` and cloud worker, no network."""

from collections.abc import Callable, Iterator
import json
from pathlib import Path
import sys
import time
from typing import Any

from fastapi.testclient import TestClient
import pytest

from dimos.constants import RECORDINGS_DIR
from dimos.gateway import app as app_module, blueprints, config, events, runs
from dimos.gateway.app import ServerState, create_app
from dimos.gateway.uploads import Uploads


@pytest.fixture
def state(server_home: Path, checkout: Path, fake_worker: list[str]) -> ServerState:
    bus = events.Bus()
    uploads = Uploads(checkout, bus, None, server_home / "uploads.log", worker=fake_worker)
    return ServerState(dimos_dir=checkout, bus=bus, uploads=uploads)


@pytest.fixture
def client(state: ServerState) -> Iterator[TestClient]:
    with TestClient(create_app(state, background=False)) as client:
        yield client


def test_health_info_and_paths(
    client: TestClient, checkout: Path, given_gateway: Callable[..., None]
) -> None:
    assert client.get("/healthz").text == "ok"
    assert client.get("/dimos/healthz").text == "ok"
    info = client.get("/dimos/info").json()
    assert info == {
        "dir": str(checkout),
        "found": True,
        "installed": True,
        "version": "0.0.14",
        "range": "",
        "inRange": True,
    }
    given_gateway(dimosRange=">=0.0.14b1 <0.1")
    assert client.get("/dimos/info").json()["inRange"] is True
    given_gateway(dimosRange=">=0.1")
    assert client.get("/dimos/info").json()["inRange"] is False
    given_gateway()
    paths = client.get("/dimos/paths").json()
    assert paths["dimosDir"] == str(checkout)
    # no recordings.dir in config.yaml and no Desktop: where dimos itself records
    assert paths["recordingsDir"] == str(RECORDINGS_DIR)
    assert set(paths) == {"dimosDir", "runsDir", "logsDirs", "recordingsDir", "server"}
    server = paths["server"]
    assert server["exe"] == sys.executable and server["exeModified"] > 0
    assert server["kind"] == "dimos" and 0 < server["startedAt"] <= time.time()
    # no zenoh publisher: SSE only, so Desktop relays
    assert server["zenohNamespace"] is None
    # started by Desktop: the folder it gives its apps
    given_gateway(recordingsDir=str(checkout / "desktop_recordings"))
    assert client.get("/dimos/paths").json()["recordingsDir"] == str(
        checkout / "desktop_recordings"
    )
    missing = client.get("/dimos/nope")
    assert (missing.status_code, missing.json()) == (404, {"error": "no such route: /dimos/nope"})


def test_paths_says_where_its_events_go_on_zenoh(client: TestClient, state: ServerState) -> None:
    # serve() sets it once its publisher is open: Desktop then stops relaying the SSE stream
    state.zenoh_namespace = "dimos-desktop/host-5555"
    assert (
        client.get("/dimos/paths").json()["server"]["zenohNamespace"] == "dimos-desktop/host-5555"
    )


def test_blueprint_list_is_read_in_process(client: TestClient) -> None:
    listed = client.get("/dimos/blueprints").json()["blueprints"]
    # not scanned yet: the discovery fields are null
    unscanned = {"importable": None, "import_error": None, "missing_module": None}
    assert {"name": "unitree-go2-basic", "kind": "builtin", **unscanned} in listed
    assert not any(b["name"].startswith("demo-") for b in listed)
    assert client.get("/dimos/blueprints?fresh=1").json()["blueprints"] == listed


# what introspect.py answers, in full, so the answers are checked against their models
INTROSPECTED: dict[str, dict[str, Any]] = {
    "blueprint": {
        "name": "unitree-go2",
        "modules": [
            {
                "name": "camera",
                "class": "dimos.hardware.camera.CameraModule",
                "streams": [
                    {"name": "color_image", "type": "dimos.msgs.Image", "direction": "out"}
                ],
            }
        ],
    },
    "config": {
        "name": "unitree-go2",
        "modules": [
            {
                "module": "camera",
                "class": "dimos.hardware.camera.CameraModule",
                "args": [
                    {
                        "name": "fps",
                        "type": "int",
                        "default": 30,
                        "description": "frames a second",
                        "required": False,
                        "base": False,
                        "value": 15,
                        "json_compatible": True,
                        "schema": {"type": "integer", "minimum": 1},
                    },
                    {
                        "name": "mode",
                        "type": "Literal['rgb', 'depth']",
                        "default": "rgb",
                        "description": None,
                        "required": False,
                        "base": False,
                        "choices": ["rgb", "depth"],
                        "json_compatible": True,
                        "schema": {"enum": ["rgb", "depth"], "type": "string"},
                    },
                    {
                        "name": "aes_128_key",
                        "type": "str | None",
                        "default": None,
                        "description": None,
                        "required": False,
                        "base": False,
                        "value": "blueprint-secret",
                        "json_compatible": True,
                        "schema": {"anyOf": [{"type": "string"}, {"type": "null"}]},
                    },
                    {
                        "name": "on_frame",
                        "type": "Callable",
                        "default": None,
                        "description": None,
                        "required": False,
                        "base": False,
                        "json_compatible": False,
                    },
                ],
            },
            {"module": "odd", "class": "x.Odd", "args": [], "error": "TypeError: not pydantic"},
        ],
    },
    "catalog": {
        "blueprints": [
            {
                "name": "unitree-go2",
                "ref": "dimos.robot.unitree.go2:bp",
                "robot": "go2",
                "modules": ["Cam"],
                "doc": "The Go2 with its camera.",
            }
        ],
        "modules": [
            {
                "name": "Cam",
                "class": "dimos.hardware.camera.CameraModule",
                "doc": "A camera.",
                "robots": ["go2"],
                "inputs": [],
                "outputs": [{"name": "color_image", "type": "Image"}],
                "skills": ["snap"],
            }
        ],
        "skills": [
            {
                "name": "snap",
                "doc": "Take a picture.",
                "params": [{"name": "size", "type": "int", "default": "1"}],
                "module": "Cam",
                "robots": ["go2"],
            }
        ],
        "errors": ["blueprint g1: ImportError: no mujoco"],
    },
}


def test_blueprint_details_are_introspected_and_cached(
    client: TestClient, monkeypatch: pytest.MonkeyPatch
) -> None:
    calls: list[list[str]] = []

    async def fake(dimos_dir: Path, args: list[str], **_: Any) -> dict[str, Any]:
        calls.append(args)
        if args[1:] == ["broken"]:
            raise blueprints.IntrospectError("ImportError: no torch")
        return INTROSPECTED[args[0]]

    monkeypatch.setattr(blueprints, "introspect", fake)
    assert client.get("/dimos/blueprints/unitree-go2").json() == INTROSPECTED["blueprint"]
    client.get("/dimos/blueprints/unitree-go2")
    # absent optional fields (choices, value, error) stay absent; each arg says whether it's a secret
    shown = client.get("/dimos/blueprints/unitree-go2/config").json()
    assert shown["overrides"] == {}
    args = shown["modules"][0]["args"]
    assert [arg.pop("secret") for arg in args] == [False, False, True, False]
    assert args[2].pop("value") == "•••"
    expected = INTROSPECTED["config"]["modules"][0]["args"]
    assert args == [
        expected[0],
        expected[1],
        {k: v for k, v in expected[2].items() if k != "value"},
        expected[3],
    ]
    assert client.get("/dimos/catalog").json() == INTROSPECTED["catalog"]
    assert calls == [["blueprint", "unitree-go2"], ["config", "unitree-go2"], ["catalog"]]
    broken = client.get("/dimos/blueprints/broken")
    assert (broken.status_code, broken.json()) == (500, {"error": "ImportError: no torch"})
    assert client.get("/dimos/blueprints/-rf").status_code == 400


def test_source_reads_only_python_files_in_the_checkout(client: TestClient, checkout: Path) -> None:
    (checkout / "dimos" / "arm").mkdir(parents=True)
    (checkout / "dimos" / "arm" / "module.py").write_text("class Arm:\n    pass\n")
    (checkout.parent / "secret.py").write_text("token = 1\n")
    (checkout / "notes.txt").write_text("hi\n")
    answer = client.get("/dimos/source", params={"file": "dimos/arm/module.py"})
    assert answer.json() == {"file": "dimos/arm/module.py", "text": "class Arm:\n    pass\n"}
    absolute = str(checkout / "dimos" / "arm" / "module.py")
    assert (
        client.get("/dimos/source", params={"file": absolute})
        .json()["text"]
        .startswith("class Arm")
    )
    for outside in ("../secret.py", str(checkout.parent / "secret.py"), "notes.txt"):
        assert client.get("/dimos/source", params={"file": outside}).status_code == 400, outside
    assert client.get("/dimos/source", params={"file": "dimos/nope.py"}).status_code == 404


async def test_introspection_runs_in_a_child_with_a_timeout(tmp_path: Path) -> None:
    python = tmp_path / "python"
    python.write_text(
        f"#!{sys.executable}\n"
        "import sys, time, json\n"
        "args = sys.argv[3:]\n"
        "print('noise from an import')\n"
        "if args[1] == 'hang': time.sleep(30)\n"
        "if args[1] == 'crash': raise SystemExit('segfault-ish')\n"
        "result = {'error': 'KeyError: x'} if args[1] == 'bad' else {'name': args[1]}\n"
        "print('\\n@@DIMOS_SERVER@@' + json.dumps(result))\n"
    )
    python.chmod(0o755)
    assert await blueprints.introspect(tmp_path, ["blueprint", "ok"], python=str(python)) == {
        "name": "ok"
    }
    # only the hang gets the short timeout: a slow CI machine can take over a second just to start python
    for name, message, timeout in [
        ("bad", "KeyError: x", 120),
        ("crash", "segfault-ish", 120),
        ("hang", "took over 1 s", 1),
    ]:
        with pytest.raises(blueprints.IntrospectError, match=message):
            await blueprints.introspect(
                tmp_path, ["blueprint", name], timeout=timeout, python=str(python)
            )


def test_global_config_overrides_live_in_desktops_config(client: TestClient) -> None:
    config.config_file().parent.mkdir(parents=True)
    config.config_file().write_text("desktop:\n  port: 7341\ndimos:\n  dir: /somewhere\n")
    value = client.get("/dimos/global-config").json()
    assert "robot_ip" in value["schema"]["properties"] and "robot_ip" in value["defaults"]
    assert value["overrides"] == {}
    saved = client.put(
        "/dimos/global-config", json={"overrides": {"robot_ip": "10.0.0.2", "n_workers": None}}
    )
    assert saved.json()["overrides"] == {"robot_ip": "10.0.0.2"}
    on_disk = config.load_desktop_config()
    assert on_disk["desktop"] == {"port": 7341} and on_disk["dimos"]["dir"] == "/somewhere"
    for bad, why in [
        ({"a-b": 1}, "bad config key: a-b"),
        (
            {"robot_ipp": "x"},
            "overrides.robot_ipp: no such GlobalConfig field (did you mean robot_ip?)",
        ),
        ({"n_workers": "many"}, "n_workers"),
    ]:
        refused = client.put("/dimos/global-config", json={"overrides": bad})
        assert refused.status_code == 400 and why in refused.json()["error"], refused.json()
    # a saved override dimos no longer has stops a launch with its name, before dimos is started
    config.set_global_config_overrides({"renamed_away": 1})
    stale = client.post("/dimos/runs", json={"blueprint": "unitree-go2"})
    assert stale.status_code == 400 and "renamed_away" in stale.json()["error"]
    assert client.put("/dimos/global-config", json={}).status_code == 400


def test_launch_log_and_stop(
    client: TestClient, state: ServerState, monkeypatch: pytest.MonkeyPatch
) -> None:
    sent: list[dict[str, Any]] = []
    state.bus.sinks.append(sent.append)
    # Ctrl-C can land before python handles it (a restart right after a launch): don't wait 20 s for it
    monkeypatch.setattr(
        runs,
        "STOP_WAITS",
        ((runs.signal.SIGINT, 2.0), (runs.signal.SIGTERM, 10.0), (runs.signal.SIGKILL, 5.0)),
    )
    assert client.get("/dimos/runs").json() == {"runs": [], "launch": None}
    nothing_yet = client.post("/dimos/runs/restart")
    assert (
        nothing_yet.status_code == 400 and "hasn't launched anything" in nothing_yet.json()["error"]
    )
    config.config_file().parent.mkdir(parents=True)
    config.config_file().write_text("dimos:\n  global_config:\n    robot_ip: 10.0.0.2\n")
    launched = client.post(
        "/dimos/runs",
        json={"blueprint": "unitree-go2", "replay": True, "overrides": {"n_workers": 2}},
    ).json()
    assert (launched["blueprint"], launched["phase"]) == ("unitree-go2", "starting")
    assert launched["output"].startswith(
        "$ dimos --n-workers=2 --replay --rerun-open=none --rerun-web --robot-ip=10.0.0.2 run unitree-go2"
    )
    overrides = {
        "robot_ip": "10.0.0.2",
        "n_workers": 2,
        "replay": True,
        "rerun_open": "none",
        "rerun_web": True,
    }
    assert launched["overrides"] == overrides
    assert [step["state"] for step in launched["steps"]] == ["now", "todo", "todo", "todo"]
    assert launched["problems"] == []
    again = client.post("/dimos/runs", json={"blueprint": "unitree-go2"})
    assert again.status_code == 400 and "still starting" in again.json()["error"]

    # it shows as running once it is in dimos's registry
    entry = {
        "run_id": "r1",
        "pid": launched["pid"],
        "blueprint": "unitree-go2",
        "started_at": "t",
        "log_dir": "d",
    }
    monkeypatch.setattr(runs, "registry_runs", lambda: [entry])
    listed = client.get("/dimos/runs").json()
    assert listed["runs"] == [entry] and (listed["launch"]["phase"], listed["launch"]["runId"]) == (
        "running",
        "r1",
    )
    # out of the registry while its process still exits: stopping, never starting again
    monkeypatch.setattr(runs, "registry_runs", lambda: [])
    stopping = client.get("/dimos/runs").json()["launch"]
    # its run id outlives its registry entry, so its log stays reachable (Logs, view=logs)
    assert (stopping["phase"], stopping["runId"], stopping["logDir"]) == ("stopping", "r1", "d")

    stopped = client.post("/dimos/runs/stop")
    assert stopped.json() == {"output": f"stopped unitree-go2 (pid {launched['pid']})"}
    assert client.get("/dimos/runs").json()["launch"]["phase"] == "stopped"
    # each phase once: the route's event and the watcher's are one
    assert [e["launch"]["phase"] for e in sent if e["type"] == "launch"] == [
        "starting",
        "stopping",
        "stopped",
    ]
    nothing = client.post("/dimos/runs/stop", json={"runId": "nope"})
    assert nothing.status_code == 500 and nothing.json() == {"error": "no live run nope"}
    assert client.post("/dimos/runs", json={"blueprint": "-x"}).status_code == 400

    # a restart launches the same blueprint with the same global config, stopping it first if it still runs
    for _ in range(2):
        restarted = client.post("/dimos/runs/restart").json()
        assert (restarted["blueprint"], restarted["phase"], restarted["overrides"]) == (
            "unitree-go2",
            "starting",
            overrides,
        )
        assert restarted["pid"] != launched["pid"]
        launched = restarted
    # ... with the config saved since: its own values (n_workers, replay) still on top
    config.config_file().write_text(
        "dimos:\n  global_config:\n    robot_ip: 10.0.0.3\n    n_workers: 5\n"
    )
    restarted = client.post("/dimos/runs/restart").json()
    assert restarted["overrides"] == {**overrides, "robot_ip": "10.0.0.3"}
    assert restarted["oneOff"] == launched["oneOff"]
    config.config_file().write_text("dimos:\n  global_config:\n    robot_ip: 10.0.0.2\n")
    assert client.post("/dimos/runs/stop").json()["output"].startswith("stopped unitree-go2")
    # stopped while it was still starting: stopped, not failed
    assert client.get("/dimos/runs").json()["launch"]["phase"] == "stopped"


# a `dimos run` whose worker (its process group, as dimos's are) shrugs off Ctrl-C, holds the run's port, and takes a
# while to let go of it after SIGTERM: the leader is gone well before the run is
LINGERING_DIMOS = """
import os, signal, socket, sys, time
marks = os.environ["FAKE_MARKS"]
def mark(what):
    with open(os.path.join(marks, what), "w") as out:
        out.write(repr(time.time()))
mark(f"start-{os.getpid()}")
if os.fork() == 0:
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind(("127.0.0.1", int(os.environ["FAKE_PORT"])))
    except OSError:
        mark(f"port-taken-{os.getpgid(0)}")
        os._exit(1)
    server.listen()
    def term(*_):
        time.sleep(1)
        mark(f"worker-exit-{os.getpgid(0)}")
        os._exit(0)
    signal.signal(signal.SIGTERM, term)
    while True:
        time.sleep(1)
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
while True:
    time.sleep(1)
"""


def test_a_relaunch_starts_only_once_the_old_run_is_gone_and_its_ports_are_free(
    client: TestClient, checkout: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import socket

    (checkout / ".venv" / "bin" / "dimos").write_text(f"#!{sys.executable}\n{LINGERING_DIMOS}")
    marks = tmp_path / "marks"
    marks.mkdir()
    with socket.socket() as free:
        free.bind(("127.0.0.1", 0))
        port = free.getsockname()[1]
    monkeypatch.setenv("FAKE_MARKS", str(marks))
    monkeypatch.setenv("FAKE_PORT", str(port))
    monkeypatch.setattr(
        runs,
        "STOP_WAITS",
        ((runs.signal.SIGINT, 1.0), (runs.signal.SIGTERM, 10.0), (runs.signal.SIGKILL, 5.0)),
    )
    first = client.post("/dimos/runs", json={"blueprint": "unitree-go2"}).json()
    # its worker is up and holds the port
    for _ in range(200):
        if runs.listening(runs.run_processes(first["pid"], None)):
            break
        time.sleep(0.05)
    assert list(runs.listening(runs.run_processes(first["pid"], None))) == [("127.0.0.1", port)]

    second = client.post("/dimos/runs/restart").json()
    assert second["phase"] == "starting" and second["pid"] != first["pid"]
    # the old leader went at Ctrl-C, its worker only a second after SIGTERM: the new run started after both
    assert not runs.group_alive(first["pid"])
    old_gone = float((marks / f"worker-exit-{first['pid']}").read_text())
    for _ in range(200):
        if (marks / f"start-{second['pid']}").exists():
            break
        time.sleep(0.05)
    assert float((marks / f"start-{second['pid']}").read_text()) > old_gone
    for _ in range(200):
        if runs.listening(runs.run_processes(second["pid"], None)):
            break
        time.sleep(0.05)
    assert not (marks / f"port-taken-{second['pid']}").exists()
    assert client.post("/dimos/runs/stop").json()["output"].startswith("stopped unitree-go2")
    assert not runs.group_alive(second["pid"])


def test_a_launch_is_refused_while_another_dimos_run_is_on_this_machine(
    client: TestClient, monkeypatch: pytest.MonkeyPatch
) -> None:
    # another run's coordinator would get this one's module start and stop calls (they share RPC names)
    other = {
        "run_id": "r9",
        "pid": 4242,
        "blueprint": "unitree-g1",
        "started_at": "t",
        "log_dir": "d",
    }
    monkeypatch.setattr(runs, "registry_runs", lambda: [other])
    refused = client.post("/dimos/runs", json={"blueprint": "unitree-go2"})
    assert (
        refused.status_code == 400
        and "unitree-g1 (run r9, pid 4242) is running" in refused.json()["error"]
    )
    # ... or one from another checkout or Desktop, which only the bus knows about
    monkeypatch.setattr(runs, "registry_runs", lambda: [])
    monkeypatch.setattr(runs, "coordinator_on_bus", lambda: True)
    refused = client.post("/dimos/runs", json={"blueprint": "unitree-go2"})
    assert refused.status_code == 400 and "another dimos run" in refused.json()["error"]


def test_a_launch_event_goes_out_once_per_change() -> None:
    bus = events.Bus()
    sent: list[dict[str, Any]] = []
    bus.sinks.append(sent.append)
    launch = {
        "blueprint": "b",
        "phase": "starting",
        "startedAt": "t",
        "pid": 1,
        "output": "",
        "runId": None,
        "logDir": None,
        "error": None,
        "overrides": {},
        "modules": {},
        "oneOff": {"global": {}, "modules": {}},
        "steps": [],
        "problems": [],
    }
    bus.launch(launch)
    bus.launch(
        {**launch, "output": "more"}
    )  # the watcher, a second later: same (blueprint, phase, runId)
    bus.launch({**launch, "phase": "running", "runId": "r1", "logDir": "d"})
    bus.launch(None)
    assert [e["launch"] and e["launch"]["phase"] for e in sent] == ["starting", "running", None]


# a `dimos run` that logs as the real one does (dimos's own logger, to DIMOS_RUN_LOG_DIR, then to its run's dir,
# LOG_DIR/<YYYYmmdd-HHMMSS>-<blueprint>) and fails deploying a module that needs a package that isn't installed
FAILING_DIMOS = """
import os, sys, time
from dimos.utils.logging_config import set_run_log_dir, setup_logger
logger = setup_logger()
logger.info("Starting DimOS")
run_dir = os.path.join(os.environ["FAKE_RUN_LOGS"], time.strftime("%Y%m%d-%H%M%S") + "-unitree-g1")
set_run_log_dir(run_dir)
logger.info("Building the blueprint")
logger.info("Starting the modules")
logger.info("Deployed module.", module="A")
try:
    import not_a_real_package_xyz
except ModuleNotFoundError:
    logger.error("Failed to deploy module", module="B", exc_info=True)
print("\\x1b[31mError: it broke\\x1b[0m", file=sys.stderr)
sys.exit(1)
"""


def test_a_failed_launch_says_how_far_it_got_and_why(
    client: TestClient, checkout: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    (checkout / ".venv" / "bin" / "dimos").write_text(f"#!{sys.executable}\n{FAILING_DIMOS}")
    monkeypatch.setenv("FAKE_RUN_LOGS", str(tmp_path / "run_logs"))
    monkeypatch.setattr("dimos.constants.LOG_DIR", tmp_path / "run_logs")
    client.post("/dimos/runs", json={"blueprint": "unitree-g1"})
    for _ in range(200):
        launch = client.get("/dimos/runs").json()["launch"]
        if launch["phase"] == "failed":
            break
        time.sleep(0.05)
    assert launch["phase"] == "failed"
    assert [(step["code"], step["state"]) for step in launch["steps"]] == [
        ("starting", "done"),
        ("building", "done"),
        ("starting_modules", "failed"),
        ("running", "todo"),
    ]
    assert launch["steps"][2]["data"] == {"deployed": 1, "total": None}
    [problem] = launch["problems"]
    assert (problem["code"], problem["data"]["missing_module"], problem["data"]["module"]) == (
        "missing_python_package",
        "not_a_real_package_xyz",
        "B",
    )
    assert launch["error"] == "No module named 'not_a_real_package_xyz'"
    # never registered, yet its log dir names its run, so Logs can show it
    [run_dir] = (tmp_path / "run_logs").iterdir()
    assert (launch["runId"], launch["logDir"]) == (run_dir.name, str(run_dir))


GO2_CONFIG = {
    "name": "unitree-go2",
    "modules": [
        {
            "module": "go2connection",
            "class": "x.GO2Connection",
            "args": [
                {
                    "name": "lidar",
                    "type": "bool",
                    "default": True,
                    "description": None,
                    "required": False,
                    "base": False,
                    "json_compatible": True,
                    "schema": {"type": "boolean"},
                },
                {
                    "name": "aes_128_key",
                    "type": "str | None",
                    "default": None,
                    "description": None,
                    "required": False,
                    "base": False,
                    "json_compatible": True,
                    "schema": {"anyOf": [{"type": "string"}, {"type": "null"}]},
                },
            ],
        },
        {
            "module": "camera",
            "class": "x.Camera",
            "args": [
                {
                    "name": "fps",
                    "type": "int",
                    "default": 30,
                    "description": None,
                    "required": False,
                    "base": False,
                    "json_compatible": True,
                    "schema": {"type": "integer"},
                },
                {
                    "name": "codec",
                    "type": "str",
                    "default": "h264",
                    "description": None,
                    "required": False,
                    "base": False,
                    "json_compatible": True,
                    "schema": {"type": "string"},
                },
            ],
        },
    ],
}

# a `dimos` that writes its argv and the secret part of its environment, then waits to be stopped
ECHO_DIMOS = """
import json, os, sys, time
seen = {"argv": sys.argv[1:], "env": {k: v for k, v in os.environ.items() if "KEY" in k}}
open(os.environ["FAKE_SEEN"], "w").write(json.dumps(seen))
time.sleep(60)
"""


def test_a_launch_with_its_own_global_and_module_config_and_a_secret(
    client: TestClient, checkout: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    async def fake(dimos_dir: Path, args: list[str], **_: Any) -> dict[str, Any]:
        return GO2_CONFIG

    monkeypatch.setattr(blueprints, "introspect", fake)
    (checkout / ".venv" / "bin" / "dimos").write_text(f"#!{sys.executable}\n{ECHO_DIMOS}")
    seen_file = tmp_path / "seen.json"
    monkeypatch.setenv("FAKE_SEEN", str(seen_file))
    config.config_file().parent.mkdir(parents=True)
    config.config_file().write_text(
        "dimos:\n  global_config:\n    robot_ip: 10.0.0.2\n    n_workers: 4\n"
    )

    # saved module config: checked, kept, a secret shown as •••
    saved = client.put(
        "/dimos/blueprints/unitree-go2/config",
        json={"overrides": {"camera": {"fps": 20}, "go2connection": {"aes_128_key": "saved-key"}}},
    ).json()
    assert saved["overrides"] == {"camera": {"fps": 20}, "go2connection": {"aes_128_key": "•••"}}
    assert config.module_config("unitree-go2")["go2connection"] == {"aes_128_key": "saved-key"}
    # ••• sent back keeps the saved secret
    client.put(
        "/dimos/blueprints/unitree-go2/config",
        json={"overrides": {"camera": {"fps": 20}, "go2connection": {"aes_128_key": "•••"}}},
    )
    assert config.module_config("unitree-go2")["go2connection"] == {"aes_128_key": "saved-key"}
    refused = client.put(
        "/dimos/blueprints/unitree-go2/config", json={"overrides": {"camera": {"fsp": 1}}}
    )
    assert refused.json() == {
        "error": "overrides.camera.fsp: no such field of camera (did you mean fps?)"
    }

    for bad, why in [
        ({"global": {}, "robot_ip": "x"}, "overrides has `robot_ip` next to `global`/`modules`"),
        (
            {"modules": {"go2conection": {"lidar": False}}},
            "overrides.modules.go2conection: no such module",
        ),
        (
            {"global": {"n_workers": "8"}},
            'overrides.global.n_workers: needs a whole number, got string "8"',
        ),
        ([1], "overrides must be an object, not list"),
    ]:
        refused = client.post("/dimos/runs", json={"blueprint": "unitree-go2", "overrides": bad})
        assert refused.status_code == 400 and refused.json()["error"].startswith(why), (
            refused.json()
        )

    launched = client.post(
        "/dimos/runs",
        json={
            "blueprint": "unitree-go2",
            "overrides": {
                "global": {"n_workers": 8, "robot_ip": None},
                "modules": {"camera": {"codec": "jpeg"}, "go2connection": {"lidar": False}},
            },
        },
    ).json()
    for _ in range(200):
        if seen_file.exists() and seen_file.read_text():
            break
        time.sleep(0.05)
    seen = json.loads(seen_file.read_text())
    # the secret is in the environment, never in argv, the launch log, the record or the answer
    assert seen["argv"] == [
        "--n-workers=8",
        "--rerun-open=none",
        "--rerun-web",
        "run",
        "unitree-go2",
        "--camera.codec=jpeg",
        "--camera.fps=20",
        "--go2connection.lidar=false",
    ]
    assert seen["env"]["GO2CONNECTION__AES_128_KEY"] == "saved-key"
    assert launched["output"].startswith(
        "$ GO2CONNECTION__AES_128_KEY=••• dimos --n-workers=8 --rerun-open=none --rerun-web run unitree-go2"
    )
    assert launched["overrides"] == {"n_workers": 8, "rerun_open": "none", "rerun_web": True}
    assert launched["modules"] == {
        "camera": {"codec": "jpeg", "fps": 20},
        "go2connection": {"aes_128_key": "•••", "lidar": False},
    }
    assert launched["oneOff"] == {
        "global": {"n_workers": 8, "robot_ip": None},
        "modules": {"camera": {"codec": "jpeg"}, "go2connection": {"lidar": False}},
    }
    for leaked in (runs.launch_file(), runs.launch_log()):
        assert "saved-key" not in leaked.read_text()
    assert oct(runs.secrets_file().stat().st_mode & 0o777) == "0o600"

    # a restart passes the same config, the secret included
    seen_file.unlink()
    restarted = client.post("/dimos/runs/restart").json()
    assert (restarted["modules"], restarted["oneOff"]) == (launched["modules"], launched["oneOff"])
    for _ in range(200):
        if seen_file.exists() and seen_file.read_text():
            break
        time.sleep(0.05)
    again = json.loads(seen_file.read_text())
    assert again == seen
    client.post("/dimos/runs/stop")


def test_launch_refuses_a_version_outside_desktops_range(
    client: TestClient, given_gateway: Callable[..., None]
) -> None:
    given_gateway(dimosRange=">=0.1")
    refused = client.post("/dimos/runs", json={"blueprint": "unitree-go2"})
    assert refused.status_code == 400 and "outside the range" in refused.json()["error"]


def test_run_log(client: TestClient, checkout: Path) -> None:
    run = checkout / "logs" / "20260101-000000-x"
    run.mkdir(parents=True)
    (run / "main.jsonl").write_text(
        '{"level":"info","event":"a"}\n{"level":"error","event":"b","logger":"nav"}\n'
    )
    page = client.get("/dimos/runs/latest/log?level=warning").json()
    assert (page["runId"], [r["event"] for r in page["records"]], page["loggers"]) == (
        "20260101-000000-x",
        ["b"],
        ["nav"],
    )
    tail = client.get(f"/dimos/runs/20260101-000000-x/log?after={page['offset']}").json()
    assert tail["records"] == []


async def test_events_start_with_the_launch_then_follow_the_bus() -> None:
    bus = events.Bus()
    stream = bus.stream(lambda: {"type": "launch", "launch": None}, keep_alive=0.05)
    assert await anext(stream) == 'data: {"type": "launch", "launch": null}\n\n'
    bus.send({"type": "upload-removed", "id": "u1"})
    assert json.loads((await anext(stream))[len("data: ") :]) == {
        "type": "upload-removed",
        "id": "u1",
    }
    assert await anext(stream) == ":\n\n"
    await stream.aclose()
    assert not bus.queues


def test_uploads(client: TestClient, tmp_path: Path) -> None:
    mcap = tmp_path / "a.mcap"
    mcap.write_bytes(b"1234")
    bad = client.post("/dimos/uploads", json={"path": str(tmp_path / "a.txt")})
    assert bad.status_code == 400 and bad.json()["error"].startswith("no such file")
    upload = client.post("/dimos/uploads", json={"path": str(mcap), "robotId": "go2"}).json()
    assert (upload["id"], upload["state"], upload["robotId"], upload["size"]) == (
        "u1",
        "queued",
        "go2",
        4,
    )
    assert client.get("/dimos/uploads").json() == {"uploads": [upload], "waitingForLogin": False}
    assert client.post("/dimos/uploads/u1/retry").status_code == 409
    assert client.delete("/dimos/uploads/u1").json() == {"ok": True}
    assert client.post("/dimos/uploads/u1/retry").json()["state"] == "queued"
    assert client.delete("/dimos/uploads/nope").status_code == 404
    assert client.post("/dimos/uploads/nope/retry").status_code == 404
    client.delete("/dimos/uploads/u1")
    assert client.delete("/dimos/uploads").json() == {"uploads": [], "waitingForLogin": False}
    assert client.get("/dimos/uploads/uploaded").json() == {"byPath": {}}
    assert client.get("/dimos/uploads/uploaded", params={"path": str(mcap)}).json() is None


def test_cloud_login_account_logout(client: TestClient) -> None:
    assert client.get("/dimos/cloud/login").json()["state"] == "idle"
    assert client.get("/dimos/cloud/account").json()["email"] == "a@b.c"
    started = client.post("/dimos/cloud/login").json()
    assert (started["state"], started["code"]) == ("pending", "ABCD")
    assert client.delete("/dimos/cloud/login").json()["state"] == "idle"
    assert client.post("/dimos/cloud/logout").json()["loggedIn"] is True
    page = client.get("/dimos/cloud/login/page?theme=dark")
    assert page.headers["content-type"].startswith("text/html") and "dimos-cloud-login" in page.text


def test_server_stop(client: TestClient, state: ServerState) -> None:
    stopped: list[bool] = []
    state.exit = lambda: stopped.append(True)
    assert client.post("/dimos/server/stop").json() == {"stopping": True}
    for _ in range(40):
        if stopped:
            break
        time.sleep(0.05)
    assert stopped


def test_robots(client: TestClient, checkout: Path) -> None:
    # the fake checkout has no robots.json: the gateway falls back to its own
    answer = client.get("/dimos/robots").json()
    basic = answer["robots"]["go2"]["blueprints"]["unitree-go2-basic"]
    assert basic["robot"] == "go2" and basic["registered"] is True
    assert answer["robots"]["go2"]["recommended"][0] == "unitree-go2"
    pick, ip, recording = basic["recommended_config"]
    assert pick["kind"] == "pick" and [c["label"] for c in pick["choices"]] == [
        "Robot",
        "Replay",
        "Simulator",
    ]
    assert pick["choices"][2]["set"] == {"replay": False, "simulation": "mujoco"}
    assert ip["id"] == "robot_ip" and ip["choices"] is None
    assert (ip["key"], ip["scope"], ip["global"], ip["when"]) == (
        "robot_ip",
        "global",
        "robot_ip",
        {"replay": False, "simulation": ""},
    )
    assert recording["kind"] == "recording" and recording["when"] == {
        "replay": True,
        "simulation": "",
    }
    assert "tags" not in basic and "modes" not in basic and "recommended_app" not in basic
    (spot_ip,) = answer["robots"]["spot"]["blueprints"]["spot"]["recommended_config"]
    assert (spot_ip["key"], spot_ip["scope"], spot_ip["module"]) == (
        "spothighlevel.ip",
        "module",
        "spothighlevel",
    )
    assert "global" not in spot_ip
    assert answer["unlisted"] == []
    # the checkout's own file wins
    own = checkout / "dimos" / "gateway" / "robots.json"
    own.parent.mkdir(parents=True, exist_ok=True)
    doc = json.loads((Path(__file__).parent / "robots.json").read_text())
    doc["robots"] = {"go2": {**doc["robots"]["go2"], "name": "My Go2"}}
    own.write_text(json.dumps(doc))
    answer = client.get("/dimos/robots").json()
    assert list(answer["robots"]) == ["go2"] and answer["robots"]["go2"]["name"] == "My Go2"
    assert "spot" in answer["unlisted"]


def test_blueprint_view_serves_its_page_and_only_its_own_files(client: TestClient) -> None:
    page = client.get("/dimos/blueprint_view", params={"name": "unitree-go2-basic"})
    assert page.status_code == 200 and page.headers["content-type"].startswith("text/html")
    assert '<script type="module" src="blueprint_view/app.js">' in page.text
    unknown = client.get("/dimos/blueprint_view", params={"name": "no-such-blueprint"})
    assert (unknown.status_code, unknown.json()) == (
        404,
        {"error": "no such blueprint: no-such-blueprint"},
    )
    assert client.get("/dimos/blueprint_view", params={"name": "-rf"}).status_code == 400
    assert client.get("/dimos/blueprint_view").status_code == 400
    logs = {"name": "unitree-go2-basic", "view": "logs", "run": "20260101-120000-unitree-go2"}
    assert client.get("/dimos/blueprint_view", params=logs).status_code == 200
    assert client.get("/dimos/blueprint_view", params={**logs, "view": "graph"}).status_code == 400
    assert client.get("/dimos/blueprint_view", params={**logs, "run": "../x y"}).status_code == 400
    for name, media in (
        ("app.js", "text/javascript"),
        ("graph.js", "text/javascript"),
        ("layout.js", "text/javascript"),
        ("view.css", "text/css"),
        ("portal.css", "text/css"),
    ):
        served = client.get(f"/dimos/blueprint_view/{name}")
        assert served.status_code == 200, name
        assert served.headers["content-type"].startswith(media), name
        assert (
            served.content
            == (Path(app_module.__file__).parent / "blueprint_view" / name).read_bytes()
        )
    for outside in (
        "index.html",
        "nope.js",
        "..%2Fapp.py",
        "..%2F..%2Fserver%2Fapp.py",
        "%2E%2E%2Frobots.json",
        "app.py",
        "../app.py",
    ):
        assert client.get(f"/dimos/blueprint_view/{outside}").status_code == 404, outside
