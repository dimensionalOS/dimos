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

"""JsonRPC keeps ZenohRPC's calls and changes only the messages, so these tests pin the messages.

test_spec.py runs the shared transport contract against JsonRPC as well.
"""

from collections.abc import Callable, Iterator
from contextlib import contextmanager
import json
import os
import threading
from typing import Any

import pytest
from pytest_mock import MockerFixture
import zenoh

from dimos.core.coordination.python_worker import PythonWorker
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.rpc_client import RPCClient
from dimos.porcelain.module_handle import RemoteModuleProxy
from dimos.protocol.rpc.jsonrpc import JsonRPC, JsonRPCError
from dimos.protocol.rpc.rpc_utils import RemoteError
from dimos.protocol.rpc.zenohrpc import EXCEPTION_ENCODING
from dimos.protocol.service import zenohservice
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohSessionPool
from dimos.utils.testing.waiting import wait_until


@contextmanager
def rpc_pair() -> Iterator[tuple[JsonRPC, JsonRPC]]:
    pool = ZenohSessionPool()
    server = JsonRPC(default_rpc_timeout=5.0, session_pool=pool)
    client = JsonRPC(default_rpc_timeout=5.0, session_pool=pool)
    server.start()
    client.start()
    try:
        yield server, client
    finally:
        server.stop()
        client.stop()
        pool.close_all()


def raw_call(rpc: JsonRPC, name: str, body: Any) -> list[Any]:
    """The replies a peer in another language gets for one query."""
    payload = body if isinstance(body, bytes) else json.dumps(body).encode()
    replies = rpc.session.get(f"dimos/rpc/v1/{name}", payload=payload, timeout=5)
    return [json.loads(reply.ok.payload.to_bytes()) for reply in replies]  # type: ignore[union-attr]


@contextmanager
def raw_server(rpc: JsonRPC, name: str, answer: Callable[[Any], Any]) -> Iterator[None]:
    """A peer in another language, serving `name` with answer(request) -> response."""

    def on_query(query: zenoh.Query) -> None:
        response = answer(json.loads(query.payload.to_bytes()))  # type: ignore[union-attr]
        payload = response if isinstance(response, bytes) else json.dumps(response).encode()
        query.reply(query.key_expr, payload)

    queryable = rpc.session.declare_queryable(f"dimos/rpc/v1/{name}", on_query, complete=True)
    try:
        yield
    finally:
        queryable.undeclare()


def test_a_server_answers_json_rpc_requests() -> None:
    with rpc_pair() as (server, client):
        server.serve_rpc(lambda a, b=0: a + b, "calc/add")
        named = {"jsonrpc": "2.0", "id": 7, "method": "calc/add", "params": {"a": 1, "b": 2}}
        positional = {"jsonrpc": "2.0", "id": "x", "method": "calc/add", "params": [1]}
        assert raw_call(client, "calc/add", named) == [{"jsonrpc": "2.0", "id": 7, "result": 3}]
        assert raw_call(client, "calc/add", positional) == [
            {"jsonrpc": "2.0", "id": "x", "result": 1}
        ]


def test_a_client_sends_json_rpc_requests() -> None:
    seen: list[dict[str, Any]] = []

    def answer(request: dict[str, Any]) -> dict[str, Any]:
        seen.append(request)
        return {"jsonrpc": "2.0", "id": request.get("id"), "result": "ok"}

    with rpc_pair() as (server, client), raw_server(server, "peer/echo", answer):
        assert client.call_sync("peer/echo", ([], {"value": 1}))[0] == "ok"
        client.call_nowait("peer/echo", ([2], {}))
        wait_until(lambda: len(seen) == 2, timeout=5)
    assert isinstance(seen[0].pop("id"), int)
    assert seen == [
        {"jsonrpc": "2.0", "method": "peer/echo", "params": {"value": 1}},
        {"jsonrpc": "2.0", "method": "peer/echo", "params": [2]},
    ]


@pytest.mark.parametrize(
    ("body", "code"),
    [
        (b"not json", -32700),
        (b'{"jsonrpc": "2.0", "id": NaN, "method": "calc/add", "params": {"a": 1}}', -32700),
        (b'{"jsonrpc": "2.0", "id": 1e309, "method": "calc/add", "params": {"a": 1}}', -32600),
        ({"jsonrpc": "2.0", "id": True, "method": "calc/add", "params": {"a": 1}}, -32600),
        ({"jsonrpc": "2.0", "id": [], "method": "calc/add", "params": {"a": 1}}, -32600),
        ({"jsonrpc": "2.0", "id": {}, "method": "calc/add", "params": {"a": 1}}, -32600),
        ({"jsonrpc": "1.0", "id": 1, "method": "calc/add"}, -32600),
        ({"jsonrpc": "2.0", "id": 1, "method": "calc/sub"}, -32600),
        ({"jsonrpc": "2.0", "id": 1, "method": "calc/add", "params": {"c": 1}}, -32602),
        ({"jsonrpc": "2.0", "id": 1, "method": "calc/add", "params": {"a": "x"}}, -32000),
        (
            {"jsonrpc": "2.0", "id": 1, "method": "calc/add", "params": {"a": 1.0, "b": 1e308}},
            -32603,
        ),
    ],
)
def test_a_bad_request_gets_its_json_rpc_error_code(body: Any, code: int) -> None:
    with rpc_pair() as (server, client):
        # b * 10 overflows 1e308 to infinity, which JSON cannot carry.
        server.serve_rpc(lambda a, b=0: a + b * 10, "calc/add")
        [reply] = raw_call(client, "calc/add", body)
    assert reply["error"]["code"] == code
    assert reply["id"] == (None if code in (-32700, -32600) else 1)


def test_a_failed_call_raises_what_the_server_sent() -> None:
    class CustomError(Exception):
        pass

    def fail(kind: str) -> None:
        if kind == "builtin":
            raise ValueError("bad value")
        if kind == "coded":
            raise JsonRPCError(4001, "rejected", {"why": "test"})
        raise CustomError("not a builtin")

    with rpc_pair() as (server, client):
        server.serve_rpc(fail, "svc/fail")
        with pytest.raises(ValueError, match="bad value"):
            client.call_sync("svc/fail", ([], {"kind": "builtin"}))
        with pytest.raises(JsonRPCError) as coded:
            client.call_sync("svc/fail", ([], {"kind": "coded"}))
        assert (coded.value.code, coded.value.data) == (4001, {"why": "test"})
        with pytest.raises(RemoteError):
            client.call_sync("svc/fail", ([], {"kind": "custom"}))


def test_an_odd_peer_reply_still_answers_the_call() -> None:
    exit_data = {"type_name": "SystemExit", "type_module": "builtins", "args": [], "traceback": ""}
    bad_data = {"type_name": "ValueError", "type_module": "builtins", "args": 5, "traceback": ""}
    responses: dict[str, dict[str, Any]] = {
        "exit": {"error": {"code": -32000, "message": "bye", "data": exit_data}},
        "bad data": {"error": {"code": -32000, "message": "odd", "data": bad_data}},
        "garbage": {"neither": "result nor error"},
    }

    def answer(request: dict[str, Any]) -> dict[str, Any]:
        return {"jsonrpc": "2.0", "id": request["id"], **responses[request["params"]["kind"]]}

    with rpc_pair() as (server, client), raw_server(server, "peer/odd", answer):
        # A call that lost its answer would raise TimeoutError instead.
        with pytest.raises(JsonRPCError, match="bye"):
            client.call_sync("peer/odd", ([], {"kind": "exit"}), rpc_timeout=2)
        with pytest.raises(JsonRPCError, match="odd"):
            client.call_sync("peer/odd", ([], {"kind": "bad data"}), rpc_timeout=2)
        with pytest.raises(ValueError, match="Invalid JSON-RPC response"):
            client.call_sync("peer/odd", ([], {"kind": "garbage"}), rpc_timeout=2)


def test_a_transport_error_is_not_unpickled(mocker: MockerFixture) -> None:
    loads = mocker.patch("dimos.protocol.rpc.zenohrpc.pickle.loads")
    with rpc_pair() as (server, client):

        def answer(query: zenoh.Query) -> None:
            query.reply_err(b"forged pickle", encoding=EXCEPTION_ENCODING)

        queryable = server.session.declare_queryable(
            "dimos/rpc/v1/peer/unsafe", answer, complete=True
        )
        try:
            with pytest.raises(ValueError, match="Unexpected JSON-RPC transport error"):
                client.call_sync("peer/unsafe", ([], {}), rpc_timeout=1)
        finally:
            queryable.undeclare()
    loads.assert_not_called()


def test_a_deep_peer_reply_still_answers_the_call() -> None:
    deep = b'{"jsonrpc":"2.0","id":1,"result":' + b"[" * 10_000 + b"0" + b"]" * 10_000 + b"}"

    with rpc_pair() as (server, client), raw_server(server, "peer/deep", lambda _: deep):
        with pytest.raises(ValueError, match="Invalid JSON-RPC response"):
            client.call_sync("peer/deep", ([], {}), rpc_timeout=1)


def test_a_notification_runs_without_a_reply() -> None:
    lines: list[str] = []
    with rpc_pair() as (server, client):
        server.serve_rpc(lambda line: lines.append(line), "log/write")
        client.call_nowait("log/write", ([], {"line": "first"}))
        wait_until(lambda: lines == ["first"], timeout=5)
        notification = {"jsonrpc": "2.0", "method": "log/write", "params": {"line": "second"}}
        assert raw_call(client, "log/write", notification) == []
        assert lines == ["first", "second"]


def test_a_call_waits_for_a_server_that_starts_late() -> None:
    with rpc_pair() as (server, client):
        threading.Timer(0.3, server.serve_rpc, (lambda: "up", "late/ping")).start()
        assert client.call_sync("late/ping", ([], {}))[0] == "up"


def test_a_call_without_a_server_times_out() -> None:
    with rpc_pair() as (_, client):
        with pytest.raises(TimeoutError):
            client.call_sync("nobody/home", ([], {}), rpc_timeout=0.3)
        assert not client._pending


def test_a_names_only_proxy_sends_the_params_it_is_given() -> None:
    with rpc_pair() as (server, client):
        server.serve_rpc(lambda value: value, "module/echo")
        proxy = RemoteModuleProxy(client, "module", {"echo"})
        assert proxy.echo(value="named") == "named"
        assert proxy.echo("positional") == "positional"
        with pytest.raises(TypeError, match="cannot mix"):
            proxy.echo("positional", value="named")
        assert not client._pending


class EchoModule(Module):
    @rpc
    def echo(self, value: str, fname: str = "") -> list[Any]:
        return [os.getpid(), value + fname]


@pytest.fixture
def worker_echo(monkeypatch: pytest.MonkeyPatch, unused_tcp_port: int) -> Iterator[RPCClient]:
    endpoint = f"tcp/127.0.0.1:{unused_tcp_port}"
    pool = ZenohSessionPool()
    monkeypatch.setattr(zenohservice, "default_session_pool", pool)
    transport = JsonRPC(
        default_rpc_timeout=10.0,
        mode="client",
        connect=[endpoint],
        multicast=False,
        session_pool=pool,
    )
    worker = PythonWorker()
    try:
        pool.acquire(ZenohConfig(mode="router", listen=[endpoint], multicast=False))
        transport.start()
        worker.start_process()
        worker.deploy_module(
            EchoModule,
            GlobalConfig(
                transport="zenoh",
                zenoh_mode="client",
                zenoh_connect=endpoint,
                zenoh_multicast=False,
                robot_ip=None,
                robot_ips=None,
            ),
            {"rpc_transport": JsonRPC, "instance_name": "echo-worker"},
        )
        yield RPCClient.remote(EchoModule, "echo-worker", rpc=transport)
    finally:
        worker.shutdown()
        transport.stop()
        pool.close_all()


def test_a_worker_module_answers_named_calls(worker_echo: RPCClient) -> None:
    pid, text = worker_echo.echo("hi", fname="!")
    assert pid != os.getpid()
    assert text == "hi!"
