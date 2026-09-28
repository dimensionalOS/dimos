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

"""Python calling a Rust NativeModule over JSON-RPC.

The end-to-end test builds and runs the rpc_planner example:
    uv run pytest -m native_e2e dimos/core/test_native_rpc.py --no-cov
"""

from collections.abc import Iterator
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import pickle
import shutil
import socket
import subprocess
import threading
import time
from typing import Any, cast

import pytest
from pytest_mock import MockerFixture
import zenoh

from dimos.core.core import native_rpc, rpc
from dimos.core.demos.rpc_planner import ToyPlanner
from dimos.core.global_config import global_config
from dimos.core.native_module import NativeModule
from dimos.core.rpc_client import RPCClient
from dimos.protocol.rpc.jsonrpc import JsonRPC, JsonRPCError
from dimos.protocol.rpc.spec import RPCInspectable, RPCServer
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.protocol.service import zenohservice
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohSessionPool
from dimos.utils.testing.waiting import wait_until

ROOT = Path(__file__).resolve().parents[2]
START = [0.0, 0.0, 0.0]
GOAL = [1.0, 2.0, 3.0]


def test_native_rpc_takes_only_what_named_json_params_can_carry() -> None:
    def start(self: Any) -> None: ...
    def _hidden(self: Any) -> None: ...
    def default(self: Any, x: int = 1) -> None: ...
    def variadic(self: Any, *xs: int) -> None: ...
    def positional(self: Any, x: int, /) -> None: ...

    for fn in (start, _hidden, default, variadic, positional):
        with pytest.raises(TypeError):
            native_rpc(fn)

    def plan(self: Any, start: list[float], *, goal: list[float]) -> None: ...

    native_rpc(plan)
    assert getattr(plan, "__native_rpc__", False) and getattr(plan, "__rpc__", False)


def test_module_serving_skips_native_methods() -> None:
    served: list[str] = []

    class Server:
        def serve_rpc(self, f: Any, name: str) -> None:
            served.append(name)

    class Mixed:
        @rpc
        def local(self) -> str:
            return "python"

        @native_rpc
        def remote(self, x: int) -> int:
            raise NotImplementedError

        rpcs = {"local": local, "remote": remote}

    RPCServer.serve_module_rpc(cast("RPCServer", Server()), cast("RPCInspectable", Mixed()), "m")
    assert served == ["m/local"]


def test_client_sends_native_methods_over_json_rpc(mocker: MockerFixture) -> None:
    json_rpc = mocker.patch("dimos.core.rpc_client.JsonRPC").return_value
    json_rpc.named_params = True
    json_rpc.start.side_effect = [ConnectionError("no router yet"), None]
    json_rpc.call_sync.return_value = ({"toy": True}, lambda: None)
    default = mocker.Mock(named_params=False, rpc_timeouts={}, default_rpc_timeout=1.0)
    client = RPCClient.remote(ToyPlanner, "toy", rpc=default)
    plan = client.plan
    json_rpc.start.assert_not_called()

    with pytest.raises(ConnectionError):
        plan(START, goal=GOAL)
    assert plan(START, goal=GOAL) == {"toy": True}
    assert json_rpc.start.call_count == 2  # the failed start is retried, not reused
    json_rpc.call_sync.assert_called_once_with("toy/plan", ([], {"start": START, "goal": GOAL}))
    assert client.start._rpc is default  # lifecycle stays on the module's own transport

    client.stop_rpc_client()
    json_rpc.stop.assert_called_once()


def test_pickled_client_keeps_native_rpc_routing(mocker: MockerFixture) -> None:
    default = mocker.Mock(named_params=False, rpc_timeouts={}, default_rpc_timeout=1.0)
    restored_default = mocker.Mock(named_params=False, rpc_timeouts={}, default_rpc_timeout=1.0)
    mocker.patch("dimos.core.rpc_client.rpc_backend", return_value=lambda: restored_default)
    json_rpc = mocker.patch("dimos.core.rpc_client.JsonRPC").return_value
    json_rpc.named_params = True
    json_rpc.call_sync.return_value = ({"toy": True}, lambda: None)

    client = RPCClient.remote(ToyPlanner, "toy", rpc=default)
    restored = pickle.loads(pickle.dumps(client))

    assert restored.plan(START, goal=GOAL) == {"toy": True}
    json_rpc.call_sync.assert_called_once_with("toy/plan", ([], {"start": START, "goal": GOAL}))
    restored_default.call_sync.assert_not_called()


def test_native_rpc_readiness_keeps_one_call_pending(mocker: MockerFixture) -> None:
    json_rpc = mocker.patch("dimos.core.native_module.JsonRPC")
    unsubscribe = mocker.Mock()

    def call(_name: str, _args: Any, callback: Any) -> Any:
        callback({"methods": ["plan"], "token": "test-token"})
        return unsubscribe

    json_rpc.return_value.call.side_effect = call
    NativeModule._wait_native_rpc(
        mocker.Mock(
            config=mocker.Mock(native_rpc_start_timeout=180.0),
            _rpc_name="toy",
            _native_rpc_methods=["plan"],
            _native_rpc_token="test-token",
        ),
        mocker.Mock(poll=mocker.Mock(return_value=None)),
    )
    json_rpc.assert_called_once_with(default_rpc_timeout=180.0)
    json_rpc.return_value.call.assert_called_once_with("toy/_ready", ([], {}), mocker.ANY)
    unsubscribe.assert_called_once()


def test_native_rpc_readiness_rejects_a_child_serving_other_methods(mocker: MockerFixture) -> None:
    json_rpc = mocker.patch("dimos.core.native_module.JsonRPC")
    unsubscribe = mocker.Mock()

    def call(_name: str, _args: Any, callback: Any) -> Any:
        callback({"methods": ["other"], "token": "t"})
        return unsubscribe

    json_rpc.return_value.call.side_effect = call
    module = mocker.Mock(
        config=mocker.Mock(native_rpc_start_timeout=1.0),
        _rpc_name="toy",
        _native_rpc_methods=["plan"],
        _native_rpc_token="t",
    )
    with pytest.raises(RuntimeError, match="does not match"):
        NativeModule._wait_native_rpc(module, mocker.Mock(poll=mocker.Mock(return_value=None)))
    unsubscribe.assert_called_once()
    json_rpc.return_value.stop.assert_called_once()


def test_native_rpc_readiness_accepts_a_slow_reply(
    zenoh_rpc: ZenohRPC, mocker: MockerFixture
) -> None:
    ready = {"methods": ["plan"], "token": "slow-token"}
    server = JsonRPC(default_rpc_timeout=3.0)
    server.start()

    def slow_ready() -> dict[str, Any]:
        time.sleep(0.2)
        return ready

    server.serve_rpc(slow_ready, "slow/_ready")
    module = mocker.Mock(
        config=mocker.Mock(native_rpc_start_timeout=3.0),
        _rpc_name="slow",
        _native_rpc_methods=["plan"],
        _native_rpc_token="slow-token",
    )
    try:
        NativeModule._wait_native_rpc(module, mocker.Mock(poll=mocker.Mock(return_value=None)))
    finally:
        server.stop()


def test_native_rpc_needs_the_default_zenoh_session(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "transport", "zenoh")
    for config in ({"stdin_config": True, "session": ZenohConfig()}, {"stdin_config": False}):
        module = ToyPlanner(executable="planner", **config)
        try:
            with pytest.raises(ValueError, match="native_rpc needs"):
                module.start()
        finally:
            module.stop()


def test_failed_stdin_write_stops_child(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    executable = tmp_path / "close-stdin"
    executable.write_text(
        "#!/usr/bin/env python3\n"
        "import os, signal, time\n"
        "def stop(*_):\n"
        "    os.posix_spawnp('sleep', ['sleep', '3'], os.environ)\n"
        "    raise SystemExit\n"
        "signal.signal(signal.SIGTERM, stop)\n"
        "os.close(0)\n"
        "while True: time.sleep(1)\n"
    )
    executable.chmod(0o755)
    module = ToyPlanner(executable=str(executable), stdin_config=True, shutdown_timeout=0.2)
    monkeypatch.setattr(global_config, "transport", "zenoh")
    monkeypatch.setattr(module, "_stdin_blob", lambda _: b"x" * 2_000_000)
    try:
        with pytest.raises(BrokenPipeError):
            module.start()
        assert module._process is None
        assert not [
            t.name
            for t in threading.enumerate()
            if t.name.startswith("native-") and "close-stdin" in t.name
        ]
    finally:
        module.stop()


def test_a_child_that_crashes_at_startup_is_reported(tmp_path: Path, zenoh_rpc: ZenohRPC) -> None:
    executable = tmp_path / "crash"
    executable.write_text("#!/bin/sh\nread -r line\nexit 3\n")
    executable.chmod(0o755)
    module = ToyPlanner(
        executable=str(executable),
        stdin_config=True,
        instance_name="crash",
        native_rpc_start_timeout=3.0,
        shutdown_timeout=30.0,
    )
    started = time.monotonic()
    try:
        with pytest.raises(RuntimeError, match="exited with code 3 before serving RPC"):
            module.start()
        # Cleanup must not wait out shutdown_timeout: a watchdog blocked behind start() once did.
        assert time.monotonic() - started < 10
        wait_until(lambda: module._watchdog is None, timeout=5)
    finally:
        module.stop()


@pytest.fixture(scope="module")
def binary() -> str:
    if shutil.which("cargo") is None:
        pytest.skip("cargo is needed to build the rpc_planner example")
    build = subprocess.run(
        [
            "cargo",
            "build",
            "--locked",
            "-p",
            "dimos-module",
            "--example",
            "rpc_planner",
            "--message-format=json-render-diagnostics",
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return str(
        next(
            item["executable"]
            for line in build.stdout.splitlines()
            if (item := json.loads(line)).get("executable")
            and item["target"]["name"] == "rpc_planner"
        )
    )


@pytest.fixture
def zenoh_rpc(monkeypatch: pytest.MonkeyPatch) -> Iterator[ZenohRPC]:
    pool = ZenohSessionPool()
    monkeypatch.setattr(zenohservice, "default_session_pool", pool)
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        endpoint = f"tcp/127.0.0.1:{listener.getsockname()[1]}"
    config = zenoh.Config()
    for key, value in {
        "mode": "router",
        "listen/endpoints": [endpoint],
        "scouting/multicast/enabled": False,
        "scouting/gossip/enabled": False,
    }.items():
        config.insert_json5(key, json.dumps(value))
    with zenoh.open(config):
        for key, value in {
            "transport": "zenoh",
            "robot_ip": None,
            "robot_ips": None,
            "zenoh_mode": "client",
            "zenoh_connect": endpoint,
            "zenoh_multicast": False,
            "zenoh_gossip": False,
            "zenoh_scouting": False,
        }.items():
            monkeypatch.setattr(global_config, key, value)
        transport = ZenohRPC(default_rpc_timeout=5.0)
        transport.start()
        try:
            yield transport
        finally:
            transport.stop()
            pool.close_all()


@pytest.mark.native_e2e
def test_python_calls_rust_through_native_module(binary: str, zenoh_rpc: ZenohRPC) -> None:
    module = ToyPlanner(executable=binary, stdin_config=True, instance_name="toy")
    client = RPCClient.remote(ToyPlanner, "toy", rpc=zenoh_rpc)
    try:
        client.start()  # returns once the child serves exactly the declared methods
        assert client.plan(START, goal=GOAL) == {"waypoints": [START, GOAL], "toy": True}
        # A slow call does not hold up the next one.
        with ThreadPoolExecutor(max_workers=1) as pool:
            paused = pool.submit(client.pause, seconds=3.0)
            wait_until(client.pause_active, timeout=2)
            assert client.plan(START, goal=GOAL)["toy"] is True
            assert paused.result(timeout=10) == {"paused": 3.0}
        with pytest.raises(JsonRPCError) as rejected:
            client.plan(start=[0.0], goal=GOAL)
        assert rejected.value.code == -32602
        # The Python stub is never served on the pickle route.
        assert not list(zenoh_rpc.session.get("dimos/rpc/toy/plan", timeout=0.2))
    finally:
        client.stop_rpc_client()
        module.stop()
