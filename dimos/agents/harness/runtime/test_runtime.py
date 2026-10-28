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


from __future__ import annotations

from collections.abc import Callable, Iterator
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
from typing import Any, Protocol
from uuid import UUID

import numpy as np
import pytest
import requests

from dimos.agents.annotation import skill
from dimos.agents.harness.runtime import load as runtime_load
from dimos.agents.harness.runtime.agent_module import AgentModule
from dimos.agents.harness.runtime.agent_module_manager import AgentModuleManager, _Record
from dimos.agents.harness.runtime.inventory import environment, relay_endpoint
from dimos.agents.harness.runtime.load import mcp_server_blueprint
from dimos.agents.skill_result import SkillResult
from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.introspection.module.info import ModuleInfo, RpcInfo
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.memory import tap
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.agent_msgs.AgentStep import AgentStep
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.spec.utils import Spec


def test_module_owner_remains_readable_during_record_updates(tmp_path, monkeypatch) -> None:
    monkeypatch.syspath_prepend(str(tmp_path))
    manager = AgentModuleManager(
        tmp_path / "records",
        tmp_path / "dimos_agent",
        connect=lambda: pytest.fail("Metadata access must not connect to the coordinator"),
        reconnect=lambda: pytest.fail("Metadata access must not reconnect to the coordinator"),
    )
    record = _Record(owner=42, class_name="Watcher", last_error=None)
    manager._save("watcher", record)
    ready = threading.Barrier(2)

    def update_records() -> None:
        ready.wait(timeout=5)
        for _ in range(1000):
            with manager._lock:
                manager._save("watcher", record)

    with ThreadPoolExecutor(max_workers=1) as executor:
        writer = executor.submit(update_records)
        ready.wait(timeout=5)
        owners = [manager.owner("watcher") for _ in range(1000)]
        writer.result(timeout=5)

    assert owners == [42] * 1000


class HarnessProbe(Module):
    """Publishes a moving position at 20 Hz and a small image at 2 Hz."""

    position: Out[Vector3]
    color_image: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._scale = 1.0
        self._stopping = threading.Event()
        self._publisher: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._publisher = threading.Thread(target=self._publish, daemon=True)
        self._publisher.start()

    @rpc
    def stop(self) -> None:
        self._stopping.set()
        if self._publisher is not None:
            self._publisher.join(timeout=2.0)
        super().stop()

    def _publish(self) -> None:
        step = 0
        frame = Image.from_numpy(np.zeros((48, 64, 3), dtype=np.uint8))
        while not self._stopping.wait(0.05):
            step += 1
            self.position.publish(Vector3(math.cos(step / 20), math.sin(step / 20), 0.0))
            if step % 10 == 0:
                self.color_image.publish(frame)

    @skill
    def recorded_steps(self) -> SkillResult:
        """Reopen the ordinary recording and return its persisted agent_step messages."""
        path = STATE_DIR / "recordings" / os.environ["DIMOS_RUN_ID"] / "memory.db"
        steps = []
        if path.exists():
            store = SqliteStore(path=str(path), must_exist=True)
            store.start()
            try:
                if "agent_step" in store.list_streams():
                    for observation in store.stream("agent_step", AgentStep):
                        step = observation.data
                        steps.append(
                            {
                                "conversation": step.conversation,
                                "entry": step.entry,
                                "kind": step.kind,
                                "ts": step.ts,
                                "text": step.text,
                            }
                        )
            finally:
                store.stop()
        return SkillResult(metadata={"steps": steps})

    @rpc
    def scale(self, factor: float = 2.0) -> float:
        """Multiply the probe's scale by factor and return the new scale.

        A plain RPC, not a skill: clients and modules call it directly.
        """
        self._scale *= factor
        return self._scale

    @skill
    def echo(self, text: str) -> str:
        """Return the given text.

        Args:
            text: Text to return.
        """
        return text


class ScaleSpec(Spec, Protocol):
    def scale(self, factor: float = 2.0) -> float: ...


class HarnessExtra(Module):
    """A module deployed while the system runs."""

    @rpc
    def extra_ping(self) -> str:
        """Return pong."""
        return "pong"


def serve_system() -> None:
    with pytest.MonkeyPatch.context() as patch:
        patch.setenv("DIMOS_RUN_ID", "harness-test")
        patch.setattr(global_config, "record", "sqlite")
        patch.setattr(global_config, "record_engine", "python")
        patch.setattr(global_config, "record_topics", "*")
        patch.setattr(tap, "RECORDINGS_DIR", STATE_DIR / "recordings")
        coordinator = ModuleCoordinator.build(
            autoconnect(HarnessProbe.blueprint(), mcp_server_blueprint())
        )
        with tap.recording(coordinator):
            coordinator.loop()


def _free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port: int = probe.getsockname()[1]
        return port


def _until(check: Callable[[], Any], label: str, timeout: float = 20.0) -> Any:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = check()
        if value:
            return value
        time.sleep(0.1)
    raise AssertionError(f"Timed out: {label}")


def _try(check: Callable[[], Any]) -> Any:
    try:
        return check()
    except Exception:
        return False


def _env(state_home: Path, port: int) -> dict[str, str]:
    return {
        **os.environ,
        "DIMOS_TRANSPORT": "lcm",
        "XDG_STATE_HOME": str(state_home),
        "DIMOS_RUN_LOG_DIR": str(state_home / "dimos" / "logs" / "test"),
        "MCPSERVER__PORT": str(port),
        "MCPSERVER__HOST": "127.0.0.1",
        "N_WORKERS": "1",
        "RELAYBRIDGEMODULE__LOCAL_PORT": "0",
        "RELAYBRIDGEMODULE__WEB_BUILD": "false",
        "RELAYBRIDGEMODULE__OPEN_BROWSER": "false",
        "RELAYBRIDGEMODULE__RTC": "false",
    }


def _python(code: str, state_home: Path, port: int) -> None:
    subprocess.run([sys.executable, "-c", code], check=True, timeout=90, env=_env(state_home, port))


def _load(state_home: Path, port: int) -> tuple[dict[str, list[str]], str]:
    completed = subprocess.run(
        [sys.executable, "-m", "dimos.cli.dimos", "agent", "load"],
        capture_output=True,
        text=True,
        timeout=120,
        env=_env(state_home, port),
    )
    assert completed.returncode == 0, completed.stderr
    return json.loads(completed.stdout), completed.stdout


class Client:
    def __init__(self, url: str) -> None:
        self.url = url
        self.conversation = 42
        self.notifications: list[dict[str, Any]] = []
        self._http = requests.Session()
        self._response: requests.Response | None = None
        self._connected = threading.Event()
        self._listener = threading.Thread(target=self._listen, daemon=True)

    def rpc(self, method: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        reply = self._http.post(
            self.url,
            json={"jsonrpc": "2.0", "id": 1, "method": method, "params": params or {}},
            timeout=60,
        ).json()
        result: dict[str, Any] = reply["result"]
        return result

    def call_raw(
        self, tool: str, arguments: dict[str, Any], meta: dict[str, Any]
    ) -> dict[str, Any]:
        return self.rpc("tools/call", {"name": tool, "arguments": arguments, "_meta": meta})

    def call(self, tool: str, /, **arguments: Any) -> dict[str, Any]:
        result = self.call_raw(tool, arguments, {"dimos/conversation": self.conversation})
        assert not result.get("isError"), result["content"][0]["text"]
        metadata: dict[str, Any] = result["structuredContent"]["metadata"]
        return metadata

    def error(self, tool: str, /, **arguments: Any) -> str:
        result = self.call_raw(tool, arguments, {})
        assert result.get("isError") is True, result
        text: str = result["content"][0]["text"]
        return text

    def listen(self) -> None:
        self._listener.start()
        _until(self._connected.is_set, "SSE stream")

    def _listen(self) -> None:
        with self._http.get(
            self.url, headers={"Accept": "text/event-stream"}, stream=True, timeout=None
        ) as response:
            self._response = response
            self._connected.set()
            try:
                for raw in response.iter_lines():
                    line = raw.decode("utf-8", "replace")
                    if line.startswith("data: "):
                        self.notifications.append(json.loads(line[6:]))
            except (requests.RequestException, ValueError):
                # close() ended the stream from another thread.
                return

    def events(self, module: str) -> list[dict[str, Any]]:
        return [
            message["params"]
            for message in self.notifications
            if message["method"] == "dimos/harness_event" and message["params"]["module"] == module
        ]

    def statuses(self, since: int = 0) -> list[dict[str, Any]]:
        return [
            message["params"]
            for message in self.notifications[since:]
            if message["method"] == "dimos/harness_status"
        ]

    def close(self) -> None:
        if self._response is not None:
            self._response.close()
        self._http.close()
        if self._listener.ident is not None:
            self._listener.join(timeout=5.0)


@pytest.fixture(scope="module")
def state_home(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return tmp_path_factory.mktemp("state")


@pytest.fixture(scope="module")
def port() -> int:
    return _free_port()


@pytest.fixture(scope="module")
def system(state_home: Path, port: int) -> Iterator[subprocess.Popen[bytes]]:
    process = subprocess.Popen(
        [
            sys.executable,
            "-c",
            "from dimos.agents.harness.runtime.test_runtime import serve_system; serve_system()",
        ],
        env=_env(state_home, port),
    )
    try:
        yield process
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


@pytest.fixture(scope="module")
def mcp_url(system: subprocess.Popen[bytes], port: int) -> str:
    url = f"http://127.0.0.1:{port}/mcp"
    client = Client(url)
    _until(
        lambda: _try(lambda: client.call_raw("echo", {"text": "up"}, {})["content"][0]["text"])
        == "up",
        "McpServer",
        60.0,
    )
    client.close()
    return url


@pytest.fixture(scope="module")
def runtime(mcp_url: str, state_home: Path, port: int) -> Iterator[Client]:
    loaded, stdout = _load(state_home, port)
    assert loaded == {"loaded": ["RelayBridgeModule", "HarnessRuntime"], "already": ["McpServer"]}
    assert stdout.count("\n") == 1
    client = Client(mcp_url)
    _until(
        lambda: "inventory" in {tool["name"] for tool in client.rpc("tools/list")["tools"]},
        "harness tools",
    )
    client.listen()
    try:
        yield client
    finally:
        client.close()


def _running(runtime: Client, instance: str) -> list[dict[str, Any]]:
    modules = runtime.call("inventory")["runtime"]["modules"]
    return [module for module in modules if module["instance"] == instance]


def test_load_again_reports_both_as_already(runtime: Client, state_home: Path, port: int) -> None:
    loaded, _ = _load(state_home, port)
    assert loaded == {"loaded": [], "already": ["RelayBridgeModule", "HarnessRuntime", "McpServer"]}


def test_inventory_follows_a_module_loaded_at_runtime(
    runtime: Client, state_home: Path, port: int
) -> None:
    inventory = runtime.call("inventory")
    assert set(inventory) == {"environment", "runtime", "agent_modules"}
    instances = {module["instance"] for module in inventory["runtime"]["modules"]}
    assert {"HarnessProbe", "HarnessRuntime", "McpServer"} <= instances
    stream = next(item for item in inventory["runtime"]["streams"] if item["name"] == "position")
    assert (stream["module"], stream["type"], stream["direction"]) == (
        "HarnessProbe",
        "Vector3",
        "out",
    )
    assert stream["channel"]
    assert inventory["environment"]["python_api_guide"].endswith("python-api.md")
    assert inventory["environment"]["transport"] == "lcm"
    relay = inventory["environment"]["relay"]
    assert set(relay) == {"url", "robot"}
    assert requests.get(f"{relay['url']}/api/info", timeout=5).ok
    assert relay["robot"]

    _python(
        "from dimos.porcelain.dimos import Dimos\n"
        "from dimos.agents.harness.runtime.test_runtime import HarnessExtra\n"
        "app = Dimos.connect()\napp.run(HarnessExtra)\napp.stop()",
        state_home,
        port,
    )
    _until(lambda: _running(runtime, "HarnessExtra"), "the added module")
    keyed = {fact["key"]: fact for fact in runtime.call("inventory", compact=True)["facts"]}
    assert keyed["rpc:HarnessExtra.extra_ping"]["line"] == (
        "HarnessExtra.extra_ping() -> str — Return pong."
    )


def test_compact_inventory_lists_facts_and_drills_down(runtime: Client) -> None:
    keyed = {fact["key"]: fact for fact in runtime.call("inventory", compact=True)["facts"]}
    assert keyed["rpc:HarnessProbe.scale"] == {
        "section": "RPCs",
        "key": "rpc:HarnessProbe.scale",
        "line": "HarnessProbe.scale(factor: float = 2.0) -> float — Multiply the probe's scale "
        "by factor and return the new scale.",
    }
    assert "rpc:HarnessProbe.start" not in keyed
    assert keyed["stream:HarnessProbe.position"]["line"].startswith(
        "HarnessProbe.position out Vector3"
    )
    assert keyed["rpc:HarnessProbe.echo"]["line"] == (
        "HarnessProbe.echo(text: str) -> str — Return the given text."
    )
    assert keyed["env:dimos"]["line"].endswith("· transport lcm")

    detail = runtime.call_raw("inventory", {"module": "HarnessProbe"}, {})
    module_text = detail["content"][0]["text"]
    assert "scale(factor: float = 2.0) -> float" in module_text
    assert "Multiply the probe's scale" in module_text
    assert "clients and modules call it directly" not in module_text
    position = next(line for line in module_text.splitlines() if "HarnessProbe.position" in line)
    assert position.endswith("· from dimos.msgs.geometry_msgs.Vector3 import Vector3")
    full = runtime.call_raw("inventory", {"module": "HarnessProbe", "rpc": "scale"}, {})
    assert "clients and modules call it directly" in full["content"][0]["text"]
    assert "nope" in runtime.error("inventory", module="HarnessProbe", rpc="nope")
    assert "rpc needs module" in runtime.error("inventory", rpc="scale")


def test_mcp_tools_include_trajectory_publication_and_module_management(
    runtime: Client,
) -> None:
    tools = {tool["name"] for tool in runtime.rpc("tools/list")["tools"]}
    assert {"inventory", "agent_module", "publish_steps", "echo", "recorded_steps"} <= tools
    assert not {"stream_get", "events", "record_steps"} & tools


def test_trajectory_publication_uses_the_ordinary_recording(runtime: Client) -> None:
    steps = [
        {"conversation": 42, "entry": "e1", "kind": "user", "text": "go", "ts": 1.5},
        {"conversation": 42, "entry": "e2", "kind": "assistant", "text": "done", "ts": 2.5},
    ]
    assert runtime.call("recorded_steps") == {"steps": []}
    assert "conversation" in runtime.error(
        "publish_steps", steps=[steps[0], {**steps[1], "conversation": True}]
    )
    assert runtime.call("publish_steps", steps=steps) == {"published": 2}
    _until(
        lambda: len(runtime.call("recorded_steps")["steps"]) >= len(steps),
        "persisted trajectory entries",
    )
    assert runtime.call("recorded_steps")["steps"] == steps


def test_a_module_created_at_run_time_reaches_inventory_and_status_listeners(
    runtime: Client, tmp_path: Path
) -> None:
    ticker = """
import threading

from dimos.agents.harness.runtime.agent_module import AgentModule
from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Vector3 import Vector3


class Ticker(AgentModule):
    \"\"\"Publishes a new stream at 10 Hz.\"\"\"

    tick: Out[Vector3]

    @rpc
    def start(self) -> None:
        super().start()
        self.halt = threading.Event()
        threading.Thread(target=self.run, daemon=True).start()

    @rpc
    def stop(self) -> None:
        self.halt.set()
        super().stop()

    def run(self) -> None:
        while not self.halt.wait(0.1):
            self.tick.publish(Vector3(1.0, 2.0, 3.0))
"""
    written = tmp_path / "ticker.py"
    written.write_text(ticker)
    seen = len(runtime.notifications)
    try:
        _module(runtime, op="create", name="ticker", path=str(written))
        _until(
            lambda: {"cause": "agent_module"} in runtime.statuses(seen),
            "the create's status",
        )
        assert {"cause": "system_modules"} in runtime.statuses(seen)
        assert any(
            (item["module"], item["name"]) == ("Ticker", "tick") and item["channel"]
            for item in runtime.call("inventory")["runtime"]["streams"]
        )
    finally:
        runtime.call("agent_module", op="delete", name="ticker")


@pytest.fixture
def watcher_source() -> str:
    return """
from dimos.agents.harness.runtime.agent_module import AgentModule
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.msgs.sensor_msgs.Image import Image


class Watcher(AgentModule):
    \"\"\"Reports the probe's frames.\"\"\"

    notify_interval_s = 1.0
    color_image: In[Image]

    @rpc
    def start(self) -> None:
        super().start()
        self.subscribe(
            self.color_image,
            lambda frame: self.notify("{label}", frame=frame, shape=list(frame.shape)),
        )
"""


def _modules_dir(state_home: Path) -> Path:
    return state_home / "dimos" / "modules" / "dimos_agent"


def _module(runtime: Client, **arguments: Any) -> tuple[str, dict[str, Any]]:
    result = runtime.call_raw(
        "agent_module", arguments, {"dimos/conversation": runtime.conversation}
    )
    assert "isError" not in result, result["content"][0]["text"]
    message: str = result["structuredContent"]["message"]
    outcome: dict[str, Any] = result["structuredContent"]["metadata"]
    return message, outcome


@pytest.mark.skipif_no_turbojpeg
def test_agent_module_lifecycle_and_events(
    runtime: Client, state_home: Path, tmp_path: Path, watcher_source: str
) -> None:
    written = tmp_path / "watcher.py"
    written.write_text(watcher_source.format(label="first"))
    source = _modules_dir(state_home) / "watch.py"
    text, created = _module(runtime, op="create", name="watch", path=str(written))
    assert text == f"watch created: running r1 from {source}"
    assert created == {
        "name": "watch",
        "op": "create",
        "ok": True,
        "state": "running",
        "path": str(source),
        "revision": 1,
        "error": None,
        "traceback": None,
    }
    assert source.read_text() == watcher_source.format(label="first")
    (module,) = _running(runtime, "Watcher")
    assert (module["class"], module["path"]) == ("Watcher", "dimos_agent.watch_r1.Watcher")
    stream = next(
        item
        for item in runtime.call("inventory")["runtime"]["streams"]
        if (item["module"], item["name"]) == ("Watcher", "color_image")
    )
    assert stream["direction"] == "in"

    _until(lambda: len(runtime.events("watch")) >= 2, "two events")
    first, second = runtime.events("watch")[:2]
    assert (first["conversation"], first["text"], first["module"]) == (
        runtime.conversation,
        "first",
        "watch",
    )
    assert UUID(first["id"]).version == 4
    assert UUID(second["id"]).version == 4
    assert first["id"] != second["id"]
    assert set(first) == {
        "id",
        "module",
        "conversation",
        "at",
        "text",
        "data",
        "exports",
        "suppressed",
    }
    assert second["at"] - first["at"] >= 0.9
    (export,) = first["exports"]
    path = Path(export["path"])
    assert (export["kind"], path.suffix) == ("image", ".jpg")
    assert export["sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert first["data"] == {"frame": export["path"], "shape": [48, 64, 3]}

    compact = runtime.call("inventory", compact=True)
    lines = {fact["key"]: fact["line"] for fact in compact["facts"]}
    assert lines["agent_module:watch"] == "watch [Watcher] running r1"
    assert runtime.call("agent_module", op="list")["agent_modules"] == [
        {
            "name": "watch",
            "class": "Watcher",
            "state": "running",
            "revision": 1,
            "owner": runtime.conversation,
            "last_error": None,
            "path": str(source),
        }
    ]

    source.write_text(watcher_source.format(label="second"))
    text, _ = _module(runtime, op="restart", name="watch")
    assert text == f"watch restarted: running r2 from {source}"
    assert [module["path"] for module in _running(runtime, "Watcher")] == [
        "dimos_agent.watch_r2.Watcher"
    ]
    _until(lambda: runtime.events("watch")[-1]["text"] == "second", "an event of the edited code")
    assert sorted(path.name for path in source.parent.glob("watch*")) == [
        "watch.py",
        "watch_r2.pyc",
    ]

    text, _ = _module(runtime, op="unload", name="watch")
    assert text == f"watch unloaded: not running; its source is {source}"
    assert _running(runtime, "Watcher") == []
    count = len(runtime.events("watch"))
    time.sleep(1.5)
    assert len(runtime.events("watch")) == count

    text, _ = _module(runtime, op="restart", name="watch")
    assert text == f"watch restarted: running r3 from {source}"
    source.write_text(
        watcher_source.format(label="third") + '        raise RuntimeError("half done")\n'
    )
    _, failed = _module(runtime, op="restart", name="watch")
    assert (failed["ok"], failed["state"]) == (False, "stopped")
    assert _running(runtime, "Watcher") == []
    text, deleted = _module(runtime, op="delete", name="watch")
    assert text == f"watch deleted, with its source {source}"
    assert (deleted["state"], deleted["revision"]) == ("absent", None)
    assert _running(runtime, "Watcher") == []
    assert sorted(path.name for path in source.parent.glob("watch*")) == []
    assert runtime.call("agent_module", op="list")["agent_modules"] == []


def test_an_edit_that_does_not_compile_leaves_the_old_code_running(
    runtime: Client, tmp_path: Path
) -> None:
    caller = """
from dimos.agents.harness.runtime.agent_module import AgentModule
from dimos.agents.harness.runtime.test_runtime import ScaleSpec
from dimos.core.core import rpc


class Caller(AgentModule):
    _probe: ScaleSpec

    @rpc
    def start(self) -> None:
        super().start()
        self.notify("scale", value=self._probe.scale(3.0))
"""
    written = tmp_path / "caller.py"
    written.write_text(caller)
    _, created = _module(runtime, op="create", name="caller", path=str(written))
    assert (created["state"], created["revision"]) == ("running", 1)
    notices = _until(lambda: runtime.events("caller"), "the Spec call's result")
    assert notices[0]["data"] == {"value": 3.0}

    source = Path(created["path"])
    source.write_text(caller + "\n    def pending(self) -> None:\n        return (\n")
    text, failed = _module(runtime, op="restart", name="caller")
    error = "SyntaxError: '(' was never closed (caller.py, line 16)"
    trace = f"  File \"{source}\", line 16\n    return (\n           ^\nSyntaxError: '(' was never closed"
    assert failed == {
        "name": "caller",
        "op": "restart",
        "ok": False,
        "state": "running",
        "path": str(source),
        "revision": 1,
        "error": error,
        "traceback": trace,
    }
    assert text == f"caller restart failed: {error}\nNow running r1 from {source}."
    assert [module["path"] for module in _running(runtime, "Caller")] == [
        "dimos_agent.caller_r1.Caller"
    ]
    (listed,) = runtime.call("agent_module", op="list")["agent_modules"]
    assert (listed["state"], listed["revision"], listed["last_error"]) == ("running", 1, error)
    runtime.call("agent_module", op="delete", name="caller")


def test_private_names_are_refused(runtime: Client, tmp_path: Path) -> None:
    private_name = """
from dimos.agents.harness.runtime.agent_module import AgentModule


class Looper(AgentModule):
    def _loop(self) -> None:
        pass
"""
    written = tmp_path / "looper.py"
    written.write_text(private_name)
    text, refused = _module(runtime, op="create", name="looper", path=str(written))
    source = Path(refused["path"])
    error = "TypeError: Looper defines _loop, which AgentModule or Module uses; choose other names"
    assert (refused["ok"], refused["state"], refused["error"]) == (False, "absent", error)
    assert refused["traceback"].startswith(
        f'Traceback (most recent call last):\n  File "{source}", line 5, in <module>\n'
        "    class Looper(AgentModule):\n"
    )
    assert text == f"looper create failed: {error}\nNow not created; its source is {source}."
    assert runtime.call("agent_module", op="list")["agent_modules"] == []
    source.unlink()

    with pytest.raises(TypeError, match="_tf, _tools"):

        class Clash(AgentModule):
            _tf: None
            _tools = {}


def test_agent_module_errors_are_visible(runtime: Client, state_home: Path, tmp_path: Path) -> None:
    callback_fails = """
from dimos.agents.harness.runtime.agent_module import AgentModule
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Vector3 import Vector3


class Divider(AgentModule):
    position: In[Vector3]

    @rpc
    def start(self) -> None:
        super().start()
        self.subscribe(self.position, lambda message: 1 / 0)
"""
    source = _modules_dir(state_home) / "bad.py"
    source.write_text("class (")
    text, failed = _module(runtime, op="create", name="bad")
    assert text == (
        "bad create failed: SyntaxError: invalid syntax (bad.py, line 1)\n"
        f"Now not created; its source is {source}."
    )
    assert failed["traceback"] == (
        f'  File "{source}", line 1\n    class (\n          ^\nSyntaxError: invalid syntax'
    )
    source.write_text("x = 1")
    text, failed = _module(runtime, op="create", name="bad")
    assert text == (
        "bad create failed: ValueError: An agent module source defines exactly one subclass "
        "of dimos.agents.harness.runtime.agent_module.AgentModule; found 0\n"
        f"Now not created; its source is {source}."
    )
    source.unlink()
    assert "create needs the module's source" in runtime.error(
        "agent_module", op="create", name="bad"
    )
    assert "Agent module names" in runtime.error("agent_module", op="create", name="Bad-Name")
    assert "do not end in _r and digits" in runtime.error(
        "agent_module", op="create", name="bad_r2"
    )
    assert "path must be absolute" in runtime.error(
        "agent_module", op="create", name="bad", path="bad.py"
    )
    assert "No agent module named 'bad'" in runtime.error("agent_module", op="restart", name="bad")
    assert runtime.call("agent_module", op="list")["agent_modules"] == []

    written = tmp_path / "divider.py"
    written.write_text(callback_fails)
    runtime.call("agent_module", op="create", name="divider", path=str(written))
    log_path = state_home / "dimos" / "logs" / "test" / "main.jsonl"
    entries = _until(
        lambda: [
            line for line in log_path.read_text().splitlines() if '"module": "divider"' in line
        ],
        "the callback traceback in the standard DimOS log",
    )
    entry = json.loads(entries[0])
    assert (entry["event"], entry["stream"]) == ("Agent module callback failed", "position")
    assert "ZeroDivisionError" in entry["exception"]
    assert "Traceback (most recent call last):" in entry["exception"]
    assert (
        f'File "{_modules_dir(state_home) / "divider.py"}", line 14, in <lambda>'
        in entry["exception"]
    )
    runtime.call("agent_module", op="delete", name="divider")


@pytest.mark.skipif_no_turbojpeg
def test_live_notices_resume_across_a_harness_reload(
    runtime: Client, state_home: Path, port: int, tmp_path: Path, watcher_source: str
) -> None:
    written = tmp_path / "auto.py"
    written.write_text(watcher_source.format(label="auto"))
    _, created = _module(runtime, op="create", name="auto", path=str(written))
    assert (created["state"], created["revision"]) == ("running", 1)
    _until(lambda: runtime.events("auto"), "an event before the reload")
    before = {notice["id"] for notice in runtime.events("auto")}

    _python(
        "from dimos.porcelain.dimos import Dimos\n"
        "from dimos.agents.harness.runtime.harness_runtime import HarnessRuntime\n"
        "app = Dimos.connect()\napp.unload(HarnessRuntime)\napp.stop()",
        state_home,
        port,
    )
    assert "inventory" not in {tool["name"] for tool in runtime.rpc("tools/list")["tools"]}
    loaded, _ = _load(state_home, port)
    assert loaded == {"loaded": ["HarnessRuntime"], "already": ["RelayBridgeModule", "McpServer"]}
    (listed,) = runtime.call("agent_module", op="list")["agent_modules"]
    assert (listed["state"], listed["revision"]) == ("running", 1)
    later = _until(
        lambda: [notice for notice in runtime.events("auto") if notice["id"] not in before],
        "a live notice after the reload",
    )
    assert (later[-1]["conversation"], later[-1]["text"]) == (runtime.conversation, "auto")
    assert not (state_home / "dimos" / "harness" / "events.jsonl").exists()
    runtime.call("agent_module", op="delete", name="auto")


def test_load_preserves_an_authored_bridge_and_discovers_its_endpoint(mocker):
    app = mocker.Mock()
    app.list_modules.return_value = [
        ModuleInfo("cockpit", class_name="CustomBridge", rpcs=[RpcInfo("relay_endpoint")]),
        ModuleInfo("HarnessRuntime", class_name="HarnessRuntime"),
        ModuleInfo("McpServer", class_name="McpServer"),
    ]
    endpoint = {"url": "http://relay.example:8080", "robot": "custom"}
    app.get_module.return_value.relay_endpoint.return_value = endpoint
    mocker.patch.object(runtime_load.Dimos, "connect", return_value=app)
    mocker.patch.object(runtime_load, "mcp_server_url", return_value="http://localhost:9990/mcp")
    app.find_module_by_spec.return_value.mcp_url.return_value = "http://localhost:9990/mcp"

    result = runtime_load.load()

    assert result.to_json() == {
        "loaded": [],
        "already": ["CustomBridge", "HarnessRuntime", "McpServer"],
    }
    app.run.assert_not_called()
    app.unload.assert_not_called()
    app.stop.assert_called_once_with()
    assert relay_endpoint(app) == endpoint
    app.get_module.assert_called_once_with("cockpit")
    app.list_modules.return_value = []
    assert relay_endpoint(app) is None
    assert environment().to_json()["relay"] is None
