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

from collections.abc import Iterator
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import pickle
import signal
import socket
import subprocess
import sys
import threading
import time
from typing import Any

import pytest
from pytest_mock import MockerFixture
import zenoh

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import native_rpc, rpc as rpc_method
from dimos.core.demos.rpc_planner import ToyPlanner
from dimos.core.global_config import global_config
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.protocol.rpc.jsonrpc import ROUTE, RpcError, call
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.protocol.service import zenohservice
from dimos.protocol.service.zenohservice import ZenohSessionPool

ROOT = Path(__file__).resolve().parents[2]
START = [0.0, 0.0, 0.0]
GOAL = [1.0, 2.0, 3.0]


@pytest.fixture(scope="module")
def binary() -> str:
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
def rpc(monkeypatch: pytest.MonkeyPatch) -> Iterator[ZenohRPC]:
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
        client = ZenohRPC(default_rpc_timeout=2.0)
        client.start()
        try:
            yield client
        finally:
            client.stop()
            pool.close_all()


@pytest.fixture
def planner(binary: str, rpc: ZenohRPC) -> Iterator[RPCClient]:
    module = ToyPlanner(executable=binary, stdin_config=True, instance_name="toy")
    proxy = RPCClient.remote(ToyPlanner, remote_name="toy", rpc=rpc)
    try:
        proxy.build()
        proxy.start()
        assert module._process is not None
        process = module._process
        yield proxy
        proxy.stop()
        assert process.wait(timeout=5) == -signal.SIGTERM
    finally:
        proxy.stop_rpc_client()
        module.stop()


def test_native_call_and_pickle_lifecycle(planner: RPCClient, rpc: ZenohRPC) -> None:
    expected = {"waypoints": [START, GOAL], "toy": True}
    assert planner.plan(START, goal=GOAL) == expected
    assert call(rpc.session, "toy/_ready")["methods"] == ["plan"]
    assert "plan" in planner.rpcs
    restored = pickle.loads(pickle.dumps(planner.plan))
    restored.set_rpc(rpc)
    assert restored(start=START, goal=GOAL) == expected
    with pytest.raises(TypeError):
        planner.plan(START)
    # The Python stub must never be served on the legacy pickle route.
    assert not list(rpc.session.get("dimos/rpc/toy/plan", timeout=0.1))


def test_blueprint(binary: str, rpc: ZenohRPC, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "n_workers", 1)
    blueprint = autoconnect(ToyPlanner.blueprint(executable=binary, stdin_config=True))
    coordinator = ModuleCoordinator.build(blueprint)
    try:
        planner = coordinator.get_instance(ToyPlanner)
        assert planner.plan(START, GOAL) == {"waypoints": [START, GOAL], "toy": True}
        assert "plan" in coordinator.list_modules()[0].rpc_names
    finally:
        coordinator.stop()


def test_receiver_name(planner: RPCClient, rpc: ZenohRPC) -> None:
    class RenamedPlanner(ToyPlanner):
        @native_rpc
        def plan(module, start: list[float], goal: list[float]) -> dict[str, Any]:  # noqa: N805
            raise NotImplementedError

    proxy = RPCClient.remote(RenamedPlanner, remote_name="toy", rpc=rpc)
    assert proxy.plan(START, GOAL) == {"waypoints": [START, GOAL], "toy": True}


@pytest.mark.parametrize("start", [[0, 0], [True, 0, 0], ["bad", 0, 0]])
def test_invalid_arguments(planner: RPCClient, start: Any) -> None:
    with pytest.raises(RpcError) as error:
        planner.plan(start, GOAL)
    assert error.value.code == -32602
    assert planner.plan(START, GOAL)["toy"] is True


def test_bad_requests(planner: RPCClient, rpc: ZenohRPC) -> None:
    with pytest.raises(RpcError) as error:
        call(rpc.session, "toy/missing")
    assert error.value.code == -32601
    for payload, code in [
        ("{", -32700),
        (json.dumps({"jsonrpc": "2.0", "id": 1, "method": "other/plan"}), -32600),
    ]:
        replies = list(rpc.session.get(f"{ROUTE}/toy/plan", payload=payload, timeout=1))
        assert len(replies) == 1 and replies[0].ok is not None
        assert json.loads(replies[0].ok.payload.to_bytes())["error"]["code"] == code


@pytest.mark.parametrize("id", [0, 1.5, None, "request-1"])
def test_request_ids(planner: RPCClient, rpc: ZenohRPC, id: Any) -> None:
    request = {
        "jsonrpc": "2.0",
        "id": id,
        "method": "toy/plan",
        "params": {"start": START, "goal": GOAL},
    }
    replies = list(rpc.session.get(f"{ROUTE}/toy/plan", payload=json.dumps(request), timeout=1))
    assert len(replies) == 1
    reply = replies[0].ok
    assert reply is not None
    assert json.loads(reply.payload.to_bytes()) == {
        "jsonrpc": "2.0",
        "id": id,
        "result": {"waypoints": [START, GOAL], "toy": True},
    }


@pytest.mark.parametrize("method", ["toy/plan", "toy/missing"])
def test_notifications(planner: RPCClient, rpc: ZenohRPC, method: str) -> None:
    request = {"jsonrpc": "2.0", "method": method, "params": {"start": START, "goal": GOAL}}
    assert not list(rpc.session.get(f"{ROUTE}/toy/plan", payload=json.dumps(request), timeout=1))


@pytest.mark.parametrize(
    ("payload", "code"),
    [
        ('{"jsonrpc":"2.0","id":1,"method":"toy/plan","params":{"x":NaN}}', -32700),
        ('{"jsonrpc":"1.0","id":1,"method":"toy/plan"}', -32600),
        ('{"jsonrpc":"2.0","id":true,"method":"toy/plan"}', -32600),
    ],
)
def test_invalid_envelopes(planner: RPCClient, rpc: ZenohRPC, payload: str, code: int) -> None:
    replies = list(rpc.session.get(f"{ROUTE}/toy/plan", payload=payload, timeout=1))
    assert len(replies) == 1
    reply = replies[0].ok
    assert reply is not None
    response = json.loads(reply.payload.to_bytes())
    assert response["id"] is None
    assert response["error"]["code"] == code


def test_large_number_preserves_parse_error(planner: RPCClient) -> None:
    with pytest.raises(RpcError, match="Parse error") as error:
        planner.plan([10**400, 0, 0], GOAL)
    assert error.value.code == -32700
    assert planner.plan(START, GOAL)["toy"] is True


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_params(planner: RPCClient, value: float) -> None:
    with pytest.raises(ValueError, match="JSON compliant"):
        planner.plan([value, 0, 0], GOAL)


def test_timeout_does_not_repeat(rpc: ZenohRPC) -> None:
    pending: list[zenoh.Query] = []
    rpc.rpc_timeouts["slow/plan"] = 0.1
    proxy = RPCClient.remote(ToyPlanner, remote_name="slow", rpc=rpc)
    with rpc.session.declare_queryable(f"{ROUTE}/slow/plan", pending.append):
        with pytest.raises(TimeoutError):
            proxy.plan(START, GOAL)
        assert len(pending) == 1
    pending.clear()


def test_peer_timeout_message_is_not_a_deadline(rpc: ZenohRPC) -> None:
    def reply(query: zenoh.Query) -> None:
        query.reply_err("Timeout")

    with rpc.session.declare_queryable(f"{ROUTE}/test/plan", reply):
        with pytest.raises(ConnectionError, match="Timeout"):
            call(rpc.session, "test/plan")


@pytest.mark.parametrize(
    ("response", "exception"),
    [
        ({"id": True, "result": {}}, ValueError),
        ({"id": 1, "result": {}, "error": {"code": -1, "message": "bad"}}, ValueError),
        ({"id": 1, "error": {"code": "bad", "message": "bad"}}, ValueError),
        ({"id": 1, "error": None}, ValueError),
        ({"id": None, "error": {"code": -32700, "message": "Parse error"}}, RpcError),
        ({"id": None, "error": {"code": -32600, "message": "Invalid Request"}}, RpcError),
        ({"id": None, "result": {}}, ValueError),
        ({"id": None, "error": {"code": -32602, "message": "Invalid params"}}, ValueError),
        ({"id": 2, "error": {"code": -32700, "message": "Parse error"}}, ValueError),
        ({"error": {"code": -32700, "message": "Parse error"}}, ValueError),
    ],
)
def test_reply_validation(
    rpc: ZenohRPC, response: dict[str, Any], exception: type[Exception]
) -> None:
    def reply(query: zenoh.Query) -> None:
        query.reply(query.key_expr, json.dumps({"jsonrpc": "2.0", **response}))

    with rpc.session.declare_queryable(f"{ROUTE}/test/plan", reply):
        with pytest.raises(exception):
            call(rpc.session, "test/plan")


@pytest.mark.parametrize("code", ["import time; time.sleep(10)", "raise SystemExit(1)"])
def test_failed_start_cleans_up(rpc: ZenohRPC, code: str) -> None:
    module = ToyPlanner(
        executable=sys.executable,
        extra_args=["-c", code],
        stdin_config=True,
        native_rpc_start_timeout=0.2,
    )
    before = time.monotonic()
    try:
        with pytest.raises((TimeoutError, RuntimeError)):
            module.start()
        assert time.monotonic() - before < 3
        assert module._process is None
    finally:
        module.stop()


def test_stale_readiness_cannot_start_a_new_process(rpc: ZenohRPC) -> None:
    module = ToyPlanner(
        executable=sys.executable,
        extra_args=["-c", "import time; time.sleep(10)"],
        instance_name="stale",
        stdin_config=True,
        native_rpc_start_timeout=0.2,
    )
    try:
        previous = json.loads(module._stdin_blob({}))["rpc"]
        previous.pop("name")

        def stale_reply(query: zenoh.Query) -> None:
            query.reply(query.key_expr, json.dumps({"jsonrpc": "2.0", "id": 1, "result": previous}))

        with rpc.session.declare_queryable(f"{ROUTE}/stale/_ready", stale_reply):
            with pytest.raises(ValueError):
                module.start()
        assert module._process is None
    finally:
        module.stop()


def test_unread_stdin_cannot_block_startup(rpc: ZenohRPC) -> None:
    class LargeConfig(NativeModuleConfig):
        payload: str
        cli_exclude: frozenset[str] = frozenset({"payload"})

    class LargePlanner(ToyPlanner):
        config: LargeConfig

    module = LargePlanner(
        executable=sys.executable,
        extra_args=["-c", "import time; time.sleep(2)"],
        stdin_config=True,
        native_rpc_start_timeout=0.1,
        payload="x" * 200_000,
    )
    before = time.monotonic()
    try:
        with pytest.raises(TimeoutError):
            module.start()
        assert time.monotonic() - before < 1
        assert module._process is None
    finally:
        module.stop()


@pytest.mark.parametrize("module_type", [NativeModule, ToyPlanner])
def test_stopped_instance_fails_before_spawn(
    rpc: ZenohRPC, mocker: MockerFixture, module_type: type[NativeModule]
) -> None:
    module = module_type(executable=sys.executable, stdin_config=True)
    module.stop()
    spawn = mocker.spy(subprocess, "Popen")
    start_main = mocker.spy(module, "_start_main")
    bind_handlers = mocker.spy(module, "_auto_bind_handlers")
    with pytest.raises(RuntimeError, match="stopped"):
        module.start()
    spawn.assert_not_called()
    start_main.assert_not_called()
    bind_handlers.assert_not_called()


def test_invalid_transport_does_not_start_handlers(rpc: ZenohRPC, mocker: MockerFixture) -> None:
    module = ToyPlanner(executable=sys.executable, stdin_config=True)
    start_main = mocker.patch.object(module, "_start_main")
    mocker.patch.object(global_config, "transport", "lcm")
    try:
        with pytest.raises(ValueError, match="active Zenoh"):
            module.start()
        start_main.assert_not_called()
        assert module._process is None
    finally:
        module.stop()


def test_stop_before_ready_reply_cannot_succeed(rpc: ZenohRPC, mocker: MockerFixture) -> None:
    module = ToyPlanner(
        executable=sys.executable,
        extra_args=["-c", "import time; time.sleep(10)"],
        stdin_config=True,
    )

    def reply(*args: Any, **kwargs: Any) -> dict[str, Any]:
        module.stop()
        return {"methods": ["plan"], "token": module._native_rpc_token}

    mocker.patch("dimos.core.native_module.call_json", side_effect=reply)
    try:
        with pytest.raises(RuntimeError, match="stopped"):
            module.start()
        assert module._process is None
    finally:
        module.stop()


def test_overlapping_start_cannot_skip_readiness(rpc: ZenohRPC, mocker: MockerFixture) -> None:
    module = ToyPlanner(
        executable=sys.executable,
        extra_args=["-c", "import time; time.sleep(10)"],
        stdin_config=True,
    )

    def wait(process: subprocess.Popen[bytes]) -> None:
        with pytest.raises(RuntimeError, match="still starting"):
            module.start()

    mocker.patch.object(module, "_wait_native_rpc", side_effect=wait)
    try:
        module.start()
        module.start()
    finally:
        module.stop()


def test_stop_during_spawn_cleans_up(rpc: ZenohRPC, mocker: MockerFixture) -> None:
    module = ToyPlanner(
        executable=sys.executable,
        extra_args=["-c", "import sys,time; sys.stdin.readline(); time.sleep(10)"],
        stdin_config=True,
        native_rpc_start_timeout=0.2,
    )
    spawning, release = threading.Event(), threading.Event()
    processes: list[subprocess.Popen[bytes]] = []
    popen = subprocess.Popen

    def spawn(*args: Any, **kwargs: Any) -> subprocess.Popen[bytes]:
        spawning.set()
        assert release.wait(3)
        process = popen(*args, **kwargs)
        processes.append(process)
        return process

    mocker.patch.object(subprocess, "Popen", side_effect=spawn)
    try:
        with ThreadPoolExecutor(max_workers=2) as executor:
            starting = executor.submit(module.start)
            try:
                assert spawning.wait(3)
                stopping = executor.submit(module.stop)
                try:
                    stopping.result(timeout=0.1)
                except TimeoutError:
                    pass
            finally:
                release.set()
            with pytest.raises(RuntimeError):
                starting.result(timeout=3)
            stopping.result(timeout=3)
        assert processes[0].poll() is not None
        assert module._process is None
    finally:
        module.stop()
        for process in processes:
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=3)


@pytest.mark.parametrize("descriptor", [staticmethod, classmethod])
@pytest.mark.parametrize("native_outer", [False, True])
def test_native_descriptors_are_rejected(descriptor: Any, native_outer: bool) -> None:
    def plan(receiver: Any, goal: list[float]) -> list[float]:
        raise NotImplementedError

    with pytest.raises(TypeError, match="instance method"):
        method = native_rpc(descriptor(plan)) if native_outer else descriptor(native_rpc(plan))
        type("InvalidPlanner", (NativeModule,), {"plan": method})


@pytest.mark.parametrize("wrapped", [False, True])
def test_async_native_declaration_is_rejected(wrapped: bool) -> None:
    async def plan(self: Any, goal: list[float]) -> list[float]:
        return goal

    with pytest.raises(TypeError, match="sync method"):
        native_rpc(rpc_method(plan) if wrapped else plan)


def test_registration_must_match(binary: str, rpc: ZenohRPC) -> None:
    class WrongPlanner(ToyPlanner):
        @native_rpc
        def missing(self) -> None:
            raise NotImplementedError

    module = WrongPlanner(executable=binary, stdin_config=True)
    try:
        with pytest.raises(RuntimeError):
            module.start()
        assert module._process is None
    finally:
        module.stop()
