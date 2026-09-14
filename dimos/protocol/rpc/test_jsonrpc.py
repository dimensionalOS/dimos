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

import asyncio
from collections.abc import Callable, Iterator
from contextlib import contextmanager
import os
import pickle
from types import FunctionType
from typing import Any, cast

import pytest
from pytest_mock import MockerFixture

from dimos.core.coordination.python_worker import Actor, MethodCallProxy, PythonWorker
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.rpc_client import RpcCall, RPCClient
from dimos.porcelain.module_handle import RemoteModuleProxy
from dimos.protocol.rpc.jsonrpc import JsonRPC, JsonRPCError
from dimos.protocol.rpc.pubsubrpc import ShmRPC
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.protocol.service import zenohservice
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohSessionPool
from dimos.utils.testing.waiting import wait_until


class EchoModule(Module):
    @rpc
    def echo(self, value: str = "hello", suffix: str = "") -> list[Any]:
        return [os.getpid(), value + suffix]

    def use_client(self, client: RPCClient) -> list[Any]:
        try:
            return [os.getpid(), client.echo("from worker", suffix="!")]
        finally:
            client.stop_rpc_client()


@pytest.fixture(params=[(JsonRPC, "dimos/rpc/v1"), (ZenohRPC, "dimos/rpc")], ids=["json", "pickle"])
def worker_client(
    request: pytest.FixtureRequest, monkeypatch: pytest.MonkeyPatch, unused_tcp_port: int
) -> Iterator[tuple[RPCClient, Actor]]:
    rpc_type, prefix = request.param
    endpoint = f"tcp/127.0.0.1:{unused_tcp_port}"
    pool = ZenohSessionPool()
    monkeypatch.setattr(zenohservice, "default_session_pool", pool)
    transport = rpc_type(mode="client", connect=[endpoint], multicast=False, session_pool=pool)
    worker = PythonWorker()
    try:
        pool.acquire(ZenohConfig(mode="router", listen=[endpoint], multicast=False))
        transport.start()
        worker.start_process()
        actor = worker.deploy_module(
            EchoModule,
            GlobalConfig(
                transport="zenoh",
                zenoh_mode="client",
                zenoh_connect=endpoint,
                zenoh_multicast=False,
                robot_ip=None,
                robot_ips=None,
            ),
            {"rpc_transport": rpc_type, "instance_name": "echo-worker"},
        )
        with transport.session.declare_querier(f"{prefix}/echo-worker/echo") as querier:
            wait_until(lambda: querier.matching_status, timeout=5)
        yield RPCClient.remote(EchoModule, "echo-worker", rpc=transport), actor
    finally:
        worker.shutdown()
        transport.stop()
        pool.close_all()


@contextmanager
def rpc_pair(rpc_type: type[JsonRPC] = JsonRPC) -> Iterator[tuple[JsonRPC, JsonRPC]]:
    pool = ZenohSessionPool()
    server = rpc_type(session_pool=pool)
    client = rpc_type(session_pool=pool)
    server.start()
    client.start()
    try:
        yield server, client
    finally:
        server.stop()
        client.stop()
        pool.close_all()


def test_codec_subclass_sets_version_and_bytes() -> None:
    class V3(JsonRPC):
        version = "v3"

        @staticmethod
        def encode(value: Any) -> bytes:
            return b"v3:" + JsonRPC.encode(value)

        @staticmethod
        def decode(payload: bytes) -> Any:
            assert payload.startswith(b"v3:")
            return JsonRPC.decode(payload[3:])

    with rpc_pair(V3) as (server, client):
        unsubscribe = server.serve_rpc(lambda value: value, "echo")
        try:
            result, _ = client.call_sync("echo", (["hello"], {}), rpc_timeout=1)
            assert result == "hello"
            assert client._route("echo") == "dimos/rpc/v3/echo"
            with pytest.raises(TypeError, match="cannot mix"):
                client.call_sync("echo", (["hello"], {"value": "hello"}), rpc_timeout=1)
            with pytest.raises(ValueError, match="JSON compliant"):
                client.call_sync("echo", ([float("nan")], {}), rpc_timeout=1)
            assert not client._pending
        finally:
            unsubscribe()


def test_module_client_survives_worker_handoff(worker_client: tuple[RPCClient, Actor]) -> None:
    client, actor = worker_client
    pid, value = client.echo("hi", suffix="!")
    assert pid != os.getpid()
    assert value == "hi!"
    assert client.echo() == [pid, "hello"]
    assert MethodCallProxy(actor).use_client(client).result() == [pid, [pid, "from worker!"]]

    call = pickle.loads(pickle.dumps(client.echo))
    call.set_rpc(client.rpc)
    assert call("after pickle", suffix="!") == [pid, "after pickle!"]


def test_unencodable_reply_reports_the_cause() -> None:
    with rpc_pair() as (server, client):
        server.serve_rpc(lambda: object(), "unencodable")
        with pytest.raises(JsonRPCError, match="not JSON serializable") as error:
            client.call_sync("unencodable", ([], {}), rpc_timeout=1)
        assert error.value.code == -32603


def test_names_only_proxy_uses_standard_params() -> None:
    with rpc_pair() as (server, client):
        server.serve_rpc(lambda value: value, "module/echo")
        proxy = RemoteModuleProxy(client, "module", {"echo"})
        assert proxy.echo(value="named") == "named"
        assert proxy.echo("positional") == "positional"
        with pytest.raises(TypeError, match="cannot mix"):
            proxy.echo("positional", value="named")
        assert not client._pending


@pytest.mark.parametrize("payload", [{}, {"jsonrpc": "2.0", "method": "echo", "id": True}])
def test_invalid_request_replies_with_null_id(payload: Any, mocker: MockerFixture) -> None:
    transport = JsonRPC()
    query, handler = mocker.Mock(), mocker.Mock()
    query.payload.to_bytes.return_value = transport.encode(payload)
    transport._execute(handler, "echo", query)
    response = transport.decode(query.reply.call_args.args[1])
    assert response == {
        "jsonrpc": "2.0",
        "id": None,
        "error": {"code": -32600, "message": "Invalid Request"},
    }
    handler.assert_not_called()


@pytest.mark.parametrize("code", [-32700, -32600])
def test_response_accepts_null_id_for_request_errors(code: int) -> None:
    response = {
        "jsonrpc": "2.0",
        "id": None,
        "error": {"code": code, "message": "bad request"},
    }
    error = JsonRPC()._response(response, 1)
    assert isinstance(error, JsonRPCError)
    assert error.code == code


def test_stopped_client_cannot_be_sent_to_worker(mocker: MockerFixture) -> None:
    transport = mocker.Mock()
    mocker.patch("dimos.core.rpc_client.rpc_backend", return_value=lambda: transport)
    client = RPCClient.remote(EchoModule)
    client.stop_rpc_client()
    with pytest.raises(RuntimeError, match="stopped RPC client"):
        pickle.dumps(client)
    transport.start.assert_called_once()
    transport.stop.assert_called_once()


def test_shm_client_handoff_preserves_config() -> None:
    transport = ShmRPC(default_capacity=8192, default_rpc_timeout=1)
    client = RPCClient.remote(EchoModule, rpc=transport)
    restored = None
    try:
        transport.start()
        transport.serve_rpc(cast("FunctionType", lambda value: value), "EchoModule/echo")
        restored = pickle.loads(pickle.dumps(client))
        assert restored.rpc.config == transport.config
        assert restored.rpc.default_rpc_timeout == 1
        assert restored.rpc._topics == {}
        assert restored.echo("hello") == "hello"
    finally:
        if restored is not None:
            restored.stop_rpc_client()
        transport.stop()


@pytest.mark.parametrize(
    "params, code",
    [({}, -32602), ({"extra": 1}, -32602), ([], -32602), ([1, 2], -32602), (["bad"], -32000)],
)
def test_invalid_arguments_are_not_handler_errors(
    params: Any, code: int, mocker: MockerFixture
) -> None:
    transport = JsonRPC()
    query = mocker.Mock()
    query.payload.to_bytes.return_value = transport.encode(
        {"jsonrpc": "2.0", "id": 1, "method": "echo", "params": params}
    )
    transport._execute(lambda value: value + 1, "echo", query)
    response = transport.decode(query.reply.call_args.args[1])
    assert response["error"]["code"] == code


def test_module_parameters_cannot_override_dispatch(mocker: MockerFixture) -> None:
    module = mocker.Mock()
    module.rpcs = {"echo": lambda **kwargs: kwargs}
    module.echo = module.rpcs["echo"]
    transport = JsonRPC()
    serve = mocker.patch.object(transport, "serve_rpc")
    transport.serve_module_rpc(module, "module")
    handler, name = serve.call_args.args
    query = mocker.Mock()
    query.payload.to_bytes.return_value = transport.encode(
        {"jsonrpc": "2.0", "id": 1, "method": name, "params": {"fname": "private"}}
    )
    transport._execute(handler, name, query)
    module.private.assert_not_called()
    assert transport.decode(query.reply.call_args.args[1])["result"] == {"fname": "private"}


@pytest.mark.parametrize("method", [lambda *args: args, lambda **kwargs: kwargs, lambda x, /: x])
def test_named_proxy_rejects_unsupported_signatures(method: Callable[..., Any]) -> None:
    transport = JsonRPC()
    call = RpcCall(method, transport, "echo", "module", [])
    with pytest.raises(TypeError, match="positional-only or variadic"):
        call()
    assert not transport._pending


def test_failed_send_does_not_keep_callback(mocker: MockerFixture) -> None:
    transport = JsonRPC()
    session = mocker.Mock()
    transport._session = session
    session.get.side_effect = RuntimeError("send failed")
    with pytest.raises(RuntimeError, match="send failed"):
        transport.call_cb("echo", ([], {}), mocker.Mock())
    assert not transport._pending


async def test_async_missing_service_finishes_without_retry() -> None:
    with rpc_pair() as (_, client):
        client.default_rpc_timeout = 0.05
        with pytest.raises(ConnectionError, match="no reply"):
            await asyncio.wait_for(client.call_async("missing", ([], {})), 0.5)
        assert not client._pending


def test_callback_exception_does_not_deliver_twice(mocker: MockerFixture) -> None:
    transport = JsonRPC()
    session = mocker.Mock()
    transport._session = session
    mocker.patch("zenoh.handlers.Callback", side_effect=lambda callback, **_: callback)
    callback = mocker.Mock(side_effect=RuntimeError("callback failed"))
    transport.call_cb("echo", ([], {}), callback)
    reply = mocker.Mock(err=None)
    reply.ok.payload.to_bytes.return_value = transport.encode(
        {"jsonrpc": "2.0", "id": 1, "result": "hello"}
    )
    on_reply = session.get.call_args.args[1]
    with pytest.raises(RuntimeError, match="callback failed"):
        on_reply(reply)
    callback.assert_called_once_with("hello")


def test_handler_protocol_error_keeps_request_id(mocker: MockerFixture) -> None:
    transport = JsonRPC()
    query = mocker.Mock()
    query.payload.to_bytes.return_value = transport.encode(
        {"jsonrpc": "2.0", "id": 37, "method": "echo"}
    )
    handler = mocker.Mock(side_effect=JsonRPCError(-32600, "handler failed"))
    transport._execute(handler, "echo", query)
    response = transport.decode(query.reply.call_args.args[1])
    assert response["id"] == 37
    assert response["error"]["code"] == -32000
