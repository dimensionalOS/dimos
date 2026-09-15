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
import threading
from types import FunctionType
from typing import Any, cast

import pytest
from pytest_mock import MockerFixture
import zenoh

from dimos.core.coordination.python_worker import Actor, MethodCallProxy, PythonWorker
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.rpc_client import RpcCall, RPCClient
from dimos.experimental.isolated_python.module import IsolatedPythonModule
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


def test_isolated_methods_resolve_at_call_time(
    mocker: MockerFixture, request: pytest.FixtureRequest
) -> None:
    class Contract(IsolatedPythonModule):
        implementation = "runtime:Runtime"

        @rpc
        def echo(self, value: str) -> str:
            raise NotImplementedError

    mocker.patch.object(JsonRPC, "start")
    serve = mocker.patch.object(JsonRPC, "serve_rpc")
    module = Contract(rpc_transport=JsonRPC, instance_name="module")
    request.addfinalizer(module.stop)
    transport = module.rpc
    assert isinstance(transport, JsonRPC)
    handler = next(call.args[0] for call in serve.call_args_list if call.args[1] == "module/echo")
    for result in ("first runtime", "restarted runtime"):
        client = mocker.Mock()
        client.echo.return_value = result
        module._runtime_client = client
        for params in ({}, {"fname": "private"}, {"value": "hello"}):
            query = mocker.Mock()
            query.payload.to_bytes.return_value = transport.encode(
                {"jsonrpc": "2.0", "id": 1, "method": "module/echo", "params": params}
            )
            transport._execute(handler, "module/echo", query)
            response = transport.decode(query.reply.call_args.args[1])
            if "value" in params:
                assert response["result"] == result
            else:
                assert response["error"]["code"] == -32602
                client.echo.assert_not_called()
        client.echo.assert_called_once_with(value="hello")


def test_named_proxy_leaves_omitted_defaults_on_server() -> None:
    def echo(value: Any = b"server default") -> str:
        return str(value)

    with rpc_pair() as (server, client):
        server.serve_rpc(echo, "module/echo")
        call = RpcCall(echo, client, "echo", "module", [])
        assert call() == "b'server default'"
        assert call("explicit") == "explicit"
        with pytest.raises(TypeError, match="not JSON serializable"):
            call(b"explicit bytes")


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
    session.declare_querier.side_effect = RuntimeError("send failed")
    with pytest.raises(RuntimeError, match="send failed"):
        transport.call_sync("echo", ([], {}), rpc_timeout=1)
    assert not transport._pending


def test_slow_send_does_not_block_other_calls(
    waiting_client: tuple[JsonRPC, Any], mocker: MockerFixture, request: pytest.FixtureRequest
) -> None:
    transport, querier = waiting_client
    querier.matching_status.matching = True
    sending, release, answered = threading.Event(), threading.Event(), threading.Event()
    results: list[Any] = []

    def send(callback: Callable[..., Any], **kwargs: Any) -> None:
        call_id = transport.decode(kwargs["payload"])["id"]
        if call_id == 1:
            sending.set()
            assert release.wait(2)
        payload = transport.encode({"jsonrpc": "2.0", "id": call_id, "result": "hello"})
        reply = mocker.Mock(err=None)
        reply.ok.payload.to_bytes.return_value = payload
        callback(reply)

    def receive(value: Any) -> None:
        results.append(value)
        answered.set()

    querier.get.side_effect = send
    transport.call_cb("echo", ([], {}), lambda _: None)
    request.addfinalizer(release.set)
    assert sending.wait(1)
    caller = threading.Thread(target=lambda: transport.call_cb("echo", ([], {}), receive))
    request.addfinalizer(lambda: caller.join(2))
    request.addfinalizer(release.set)
    caller.start()
    assert answered.wait(1)
    assert results == ["hello"]


@pytest.mark.parametrize("fails", [False, True])
def test_cancellation_during_worker_start_prevents_send(
    fails: bool,
    waiting_client: tuple[JsonRPC, Any],
    mocker: MockerFixture,
    request: pytest.FixtureRequest,
) -> None:
    transport, querier = waiting_client
    querier.matching_status.matching = True
    thread_type = threading.Thread
    starting, release, cancelled = threading.Event(), threading.Event(), threading.Event()
    errors: list[Exception] = []

    def make_worker(**kwargs: Any) -> threading.Thread:
        worker = thread_type(**kwargs)
        start = worker.start

        def delayed_start() -> None:
            starting.set()
            assert release.wait(2)
            if fails:
                raise RuntimeError("thread start failed")
            start()

        mocker.patch.object(worker, "start", side_effect=delayed_start)
        return worker

    def call() -> None:
        try:
            transport.call_cb("echo", ([], {}), lambda _: None)
        except Exception as error:
            errors.append(error)

    mocker.patch("dimos.protocol.rpc.jsonrpc.threading.Thread", side_effect=make_worker)
    caller = thread_type(target=call)
    request.addfinalizer(lambda: caller.join(2))
    request.addfinalizer(release.set)
    caller.start()
    assert starting.wait(1)
    assert transport._pending_lock.acquire(timeout=1)
    try:
        cancel = transport._pending[1]
    finally:
        transport._pending_lock.release()

    def cancel_call() -> None:
        cancel()
        cancelled.set()

    canceller = thread_type(target=cancel_call)
    request.addfinalizer(lambda: canceller.join(2))
    request.addfinalizer(release.set)
    canceller.start()
    wait_until(lambda: not transport._pending, timeout=1)
    release.set()
    assert cancelled.wait(1)
    caller.join(1)
    assert [str(error) for error in errors] == (["thread start failed"] if fails else [])
    assert not transport._pending
    querier.get.assert_not_called()


async def test_async_missing_service_finishes_without_retry() -> None:
    with rpc_pair() as (_, client):
        client.default_rpc_timeout = 0.05
        with pytest.raises(TimeoutError, match="timed out"):
            await asyncio.wait_for(client.call_async("missing", ([], {})), 0.5)
        assert not client._pending


def test_waits_for_late_responder_and_sends_once(mocker: MockerFixture) -> None:
    with rpc_pair() as (server, client):
        client.default_rpc_timeout = 2
        received = threading.Event()
        results: list[Any] = []
        handler = mocker.Mock(side_effect=lambda value: value)

        def on_result(value: Any) -> None:
            results.append(value)
            received.set()

        client.call_cb("late", (["hello"], {}), on_result)
        assert not received.is_set()
        server.serve_rpc(handler, "late")
        assert received.wait(3)
        assert results == ["hello"]
        handler.assert_called_once_with("hello")
        assert not client._pending


async def test_cancelled_async_call_does_not_wait_for_discovery() -> None:
    with rpc_pair() as (_, client):
        task = asyncio.create_task(client.call_async("missing", ([], {})))
        await asyncio.sleep(0)
        assert client._pending
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert not client._pending


@pytest.mark.parametrize("result", ["late reply", RuntimeError("late error")])
async def test_async_cancellation_ignores_queued_delivery(
    result: Any, mocker: MockerFixture
) -> None:
    transport = JsonRPC()
    loop = asyncio.get_running_loop()
    loop_error = mocker.patch.object(loop, "call_exception_handler")
    unsubscribe = mocker.Mock()

    def call(name: str, args: Any, callback: Callable[..., Any]) -> Any:
        task = asyncio.current_task()
        assert task is not None
        loop.call_soon(task.cancel)
        callback(result)
        return unsubscribe

    mocker.patch.object(transport, "call", side_effect=call)
    with pytest.raises(asyncio.CancelledError):
        await transport.call_async("echo", ([], {}))
    await asyncio.sleep(0)
    loop_error.assert_not_called()
    unsubscribe.assert_called_once()


def test_executed_request_without_reply_is_not_retried(mocker: MockerFixture) -> None:
    with rpc_pair() as (server, client):
        executed = mocker.Mock(side_effect=lambda query: query.drop())
        with server.session.declare_queryable(client._route("drop"), executed, complete=True):
            with pytest.raises(ConnectionError, match="may have executed"):
                client.call_sync("drop", ([], {}), rpc_timeout=1)
        executed.assert_called_once()
        assert not client._pending


@pytest.fixture
def waiting_client(mocker: MockerFixture) -> Iterator[tuple[JsonRPC, Any]]:
    transport = JsonRPC(default_rpc_timeout=1)
    transport._session = session = mocker.Mock()
    querier = mocker.MagicMock()
    session.declare_querier.return_value = querier
    querier.__enter__.return_value = querier
    querier.matching_status.matching = False
    mocker.patch("zenoh.handlers.Callback", side_effect=lambda callback, **_: callback)
    yield transport, querier
    transport.stop()


@pytest.mark.parametrize("end", ["cancel", "timeout", "stop"])
def test_discovery_cleanup_prevents_late_send(
    end: str, waiting_client: tuple[JsonRPC, Any], mocker: MockerFixture
) -> None:
    transport, querier = waiting_client
    transport.default_rpc_timeout = 0.2
    session: Any = transport.session
    callback = mocker.Mock()
    cancel = transport.call_cb("late", ([], {}), callback)
    wait_until(lambda: querier.declare_matching_listener.called, timeout=1)
    assert (
        session.declare_querier.call_args.kwargs["congestion_control"]
        == zenoh.CongestionControl.DROP
    )
    listener_callback = querier.declare_matching_listener.call_args.args[0]
    if end == "cancel":
        cancel()
    elif end == "timeout":
        wait_until(lambda: callback.called, timeout=1)
    else:
        transport.stop()
    listener_callback(mocker.Mock(matching=True))
    querier.get.assert_not_called()
    querier.declare_matching_listener.return_value.__exit__.assert_called_once()
    querier.__exit__.assert_called_once()
    assert not transport._pending
    if end == "timeout":
        assert isinstance(callback.call_args.args[0], TimeoutError)
        callback.assert_called_once()
    else:
        callback.assert_not_called()


def test_discovery_and_reply_share_one_deadline(
    waiting_client: tuple[JsonRPC, Any], mocker: MockerFixture
) -> None:
    transport, querier = waiting_client
    now = mocker.patch("dimos.protocol.rpc.jsonrpc.time.monotonic", return_value=10)
    callback = mocker.Mock()
    transport.call_cb("late", ([], {}), callback)
    wait_until(lambda: querier.declare_matching_listener.called, timeout=1)
    matching = querier.declare_matching_listener.call_args.args[0]
    now.return_value = 10.8
    matching(mocker.Mock(matching=True))
    matching(mocker.Mock(matching=True))
    wait_until(lambda: querier.get.called, timeout=1)
    querier.get.assert_called_once()
    now.return_value = 11
    reply = mocker.Mock(err=None)
    reply.ok.payload.to_bytes.return_value = transport.encode(
        {"jsonrpc": "2.0", "id": 1, "result": "too late"}
    )
    querier.get.call_args.args[0](reply)
    wait_until(lambda: callback.called, timeout=1)
    callback.assert_called_once()
    assert isinstance(callback.call_args.args[0], TimeoutError)
    assert not transport._pending


def test_matching_after_deadline_does_not_send(
    waiting_client: tuple[JsonRPC, Any], mocker: MockerFixture
) -> None:
    transport, querier = waiting_client
    now = mocker.patch("dimos.protocol.rpc.jsonrpc.time.monotonic", return_value=10)
    callback = mocker.Mock()
    transport.call_cb("late", ([], {}), callback)
    wait_until(lambda: querier.declare_matching_listener.called, timeout=1)
    now.return_value = 11
    querier.declare_matching_listener.call_args.args[0](mocker.Mock(matching=True))
    wait_until(lambda: callback.called, timeout=1)
    querier.get.assert_not_called()
    callback.assert_called_once()
    assert isinstance(callback.call_args.args[0], TimeoutError)


def test_stop_during_querier_creation_prevents_registration(
    waiting_client: tuple[JsonRPC, Any], mocker: MockerFixture
) -> None:
    transport, querier = waiting_client
    session = transport.session
    querier.matching_status.matching = True

    def declare(*args: Any, **kwargs: Any) -> Any:
        transport.stop()
        return querier

    mocker.patch.object(session, "declare_querier", side_effect=declare)
    callback = mocker.Mock()
    transport.call_cb("echo", ([], {}), callback)
    wait_until(lambda: querier.__exit__.called, timeout=1)
    assert not transport._pending
    querier.get.assert_not_called()
    callback.assert_not_called()
    with pytest.raises(RuntimeError, match="Call start"):
        transport.call_cb("echo", ([], {}), callback)


def test_reply_during_listener_registration_cleans_up(
    waiting_client: tuple[JsonRPC, Any], mocker: MockerFixture
) -> None:
    transport, querier = waiting_client
    listener = querier.declare_matching_listener.return_value
    reply = mocker.Mock(err=None)
    reply.ok.payload.to_bytes.return_value = transport.encode(
        {"jsonrpc": "2.0", "id": 1, "result": "hello"}
    )
    inside_callback = False

    def get(callback: Callable[..., Any], **kwargs: Any) -> None:
        nonlocal inside_callback
        assert not inside_callback
        inside_callback = True
        callback(reply)
        inside_callback = False

    def register(callback: Callable[..., Any]) -> Any:
        nonlocal inside_callback
        inside_callback = True
        callback(mocker.Mock(matching=True))
        inside_callback = False
        return listener

    def cleanup(*args: Any) -> None:
        assert not inside_callback

    querier.get.side_effect = get
    querier.__exit__.side_effect = cleanup
    listener.__exit__.side_effect = cleanup
    querier.declare_matching_listener.side_effect = register
    callback = mocker.Mock()
    transport.call_cb("echo", ([], {}), callback)
    wait_until(lambda: callback.called, timeout=1)
    callback.assert_called_once_with("hello")
    querier.get.assert_called_once()
    listener.__exit__.assert_called_once()
    querier.__exit__.assert_called_once()
    assert not transport._pending


def test_callback_exception_does_not_deliver_twice(
    waiting_client: tuple[JsonRPC, Any], mocker: MockerFixture
) -> None:
    transport, querier = waiting_client
    querier.matching_status.matching = True
    worker = mocker.patch("dimos.protocol.rpc.jsonrpc.threading.Thread")
    worker.return_value.start.side_effect = lambda: worker.call_args.kwargs["target"]()
    callback = mocker.Mock(side_effect=RuntimeError("callback failed"))
    reply = mocker.Mock(err=None)
    reply.ok.payload.to_bytes.return_value = transport.encode(
        {"jsonrpc": "2.0", "id": 1, "result": "hello"}
    )
    querier.get.side_effect = lambda on_reply, **_: on_reply(reply)
    with pytest.raises(RuntimeError, match="callback failed"):
        transport.call_cb("echo", ([], {}), callback)
    callback.assert_called_once_with("hello")
    assert not transport._pending


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
