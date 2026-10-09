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

from contextlib import contextmanager
import importlib
import inspect
import socket
import threading
from typing import Protocol
import uuid

import pytest
from reactivex.disposable import Disposable

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.demos.stress_test_module import StressTestModule
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module
from dimos.core.stream import IO, In, Out
from dimos.core.transport import LCMTransport, SHMTransport, pLCMTransport, pSHMTransport
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.porcelain.dimos import Dimos
from dimos.porcelain.module_handle import RemoteModuleProxy
from dimos.porcelain.remote_module_source import RemoteModuleSource
from dimos.spec.utils import Spec


class PingSpec(Spec, Protocol):
    def ping(self) -> str: ...


class NamedRemoteModule(Module):
    @rpc
    def ping_name(self) -> str:
        return self.config.instance_name or "default"


class NamedSpec(Spec, Protocol):
    def ping_name(self) -> str: ...


class WrongReturnSpec(Spec, Protocol):
    def ping_name(self) -> int: ...


class StreamModule(Module):
    messages: Out[bytes]
    commands: In[bytes]
    state: IO[bytes]
    unwired: In[str]

    @rpc
    def emit(self, name: str, message: bytes) -> None:
        getattr(self, name).transport.publish(message)


class VectorStreamModule(Module):
    messages: Out[Vector3]

    @rpc
    def emit(self, message: Vector3) -> None:
        self.messages.publish(message)


@contextmanager
def _remote_source_with_instances(*instance_names: str):
    coordinator = ModuleCoordinator(g=GlobalConfig(n_workers=0, viewer="none"))
    coordinator.start()
    try:
        for instance_name in instance_names:
            coordinator.deploy(NamedRemoteModule, instance_name=instance_name)
        coordinator.start_rpc_service()
        source = RemoteModuleSource()
        try:
            yield source
        finally:
            source.close()
    finally:
        coordinator.stop()


def test_connect_no_running_system():
    with pytest.raises(RuntimeError, match="No running DimOS coordinator"):
        Dimos.connect(timeout=0.5)


def test_connect_uses_default_timeout_without_registry_precondition(mocker):
    source = mocker.patch("dimos.porcelain.dimos.RemoteModuleSource")

    app = Dimos.connect()
    try:
        source.assert_called_once_with(timeout=5.0)
    finally:
        app.stop()


def test_connect_skill_call(running_app, client):
    assert client.skills.ping() == "pong"
    assert client.skills.echo(message="hello") == "hello"
    client.stop()
    assert running_app.is_running
    assert running_app.skills.ping() == "pong"


def test_connect_rpc_method_call(client):
    module = client.StressTestModule
    assert module.ping() == "pong"


def test_typed_lookup_calls_the_same_remote_module(client, running_app):
    for app in (client, running_app):
        module = app.find_module_by_spec(PingSpec)
        assert module is app.get_module("StressTestModule")
        assert module.ping() == "pong"


def test_typed_lookup_preserves_missing_module_error(client):
    with pytest.raises(LookupError, match="missing-module"):
        client.find_module_by_spec(PingSpec, instance_name="missing-module")


@pytest.mark.parametrize(
    "descriptor_changes, error",
    [
        ({"rpc_names": []}, "PingSpec RPC signatures"),
        (
            {"qualified_path": "dimos.missing_module.MissingModule"},
            "cannot inspect module classes",
        ),
    ],
    ids=["unadvertised-rpc", "unavailable-signatures"],
)
def test_spec_lookup_rejects_unverifiable_module(client, mocker, descriptor_changes, error):
    descriptors = client._source.list_module_descriptors()
    mocker.patch.object(
        client._source,
        "list_module_descriptors",
        return_value=[descriptor._replace(**descriptor_changes) for descriptor in descriptors],
    )

    with pytest.raises(LookupError, match=error):
        client.find_module_by_spec(PingSpec)


def test_spec_lookup_rejects_signature_mismatch():
    with _remote_source_with_instances("robot0/named"):
        app = Dimos.connect()
        try:
            with pytest.raises(LookupError, match="WrongReturnSpec RPC signatures"):
                app.find_module_by_spec(WrongReturnSpec)
        finally:
            app.stop()


def test_spec_lookup_requires_explicit_instance_when_ambiguous():
    with _remote_source_with_instances("robot0/named", "robot1/named"):
        app = Dimos.connect()
        try:
            with pytest.raises(ValueError, match="robot0/named.*robot1/named"):
                app.find_module_by_spec(NamedSpec)
            selected = app.find_module_by_spec(NamedSpec, instance_name="robot1/named")
            assert selected.ping_name() == "robot1/named"
        finally:
            app.stop()


def test_rpc_proxy_preserves_signature_and_documentation(client):
    method = client.StressTestModule.slow

    assert str(inspect.signature(method)) == "(seconds: 'float' = 1.0) -> 'str'"
    assert method.__name__ == "slow"
    assert method.__doc__ == StressTestModule.slow.__doc__
    assert "self" not in inspect.signature(method).parameters


def test_connect_restart_invalidates_cache(client):
    source = client._source
    m_before = source.get_module("StressTestModule")
    client.restart(StressTestModule, reload_source=False)
    m_after = source.get_module("StressTestModule")
    assert m_before is not m_after
    assert client.skills.ping() == "pong"


def test_connect_run_by_name_adds_module(running_app, client):
    client.run("mcp-server")
    assert "McpServer" in client._source.list_module_names()
    assert "McpServer" in running_app._source.list_module_names()


def test_connect_repr_marks_remote(client):
    rep = repr(client)
    assert "remote" in rep
    assert "StressTestModule" in rep


def test_connect_stop_does_not_kill_remote(running_app, client):
    client.stop()
    assert not client.is_running
    assert running_app.is_running
    assert running_app.skills.ping() == "pong"


def test_connect_list_module_names(client):
    names = client._source.list_module_names()
    assert "StressTestModule" in names


def test_connect_get_module_caches(client):
    source = client._source
    m1 = source.get_module("StressTestModule")
    m2 = source.get_module("StressTestModule")
    assert m1 is m2


def test_get_module_class_name_resolves_single_namespaced_instance():
    with _remote_source_with_instances("robot0/namedremotemodule") as source:
        module = source.get_module("NamedRemoteModule")
        assert module.ping_name() == "robot0/namedremotemodule"


def test_get_module_class_name_raises_when_ambiguous():
    with _remote_source_with_instances(
        "robot0/namedremotemodule", "robot1/namedremotemodule"
    ) as source:
        with pytest.raises(ValueError, match="Multiple instances"):
            source.get_module("NamedRemoteModule")

        module = source.get_module("robot1/namedremotemodule")
        assert module.ping_name() == "robot1/namedremotemodule"


def test_cached_class_lookup_becomes_ambiguous_after_live_module_addition():
    with _remote_source_with_instances("robot0/namedremotemodule") as source:
        assert source.get_module("NamedRemoteModule").ping_name() == "robot0/namedremotemodule"

        source._coord.call(
            "load_blueprint",
            NamedRemoteModule.blueprint(instance_name="robot1/namedremotemodule"),
        )

        with pytest.raises(ValueError, match="robot1/namedremotemodule"):
            source.get_module("NamedRemoteModule")


def test_dimos_discovery_distinguishes_same_class_instances():
    with _remote_source_with_instances(
        "robot0/namedremotemodule", "robot1/namedremotemodule"
    ) as source:
        app = Dimos()
        app._source = source
        try:
            assert {module.instance_name for module in app.list_modules()} == {
                "robot0/namedremotemodule",
                "robot1/namedremotemodule",
            }
            assert app.get_module("robot1/namedremotemodule").ping_name() == (
                "robot1/namedremotemodule"
            )
            with pytest.raises(ValueError, match="robot0/namedremotemodule"):
                app.get_module("NamedRemoteModule")
        finally:
            app.stop()


def test_remote_proxy_fallback_when_class_unimportable(client, monkeypatch):
    """If `importlib.import_module` raises, get_module returns a names-only proxy."""
    real_import = importlib.import_module

    def fake_import(name: str, *args, **kwargs):  # type: ignore[no-untyped-def]
        if "stress_test_module" in name:
            raise ImportError(name)
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr("dimos.porcelain.remote_module_source.importlib.import_module", fake_import)
    client._source.invalidate("StressTestModule")
    proxy = client._source.get_module("StressTestModule")
    assert isinstance(proxy, RemoteModuleProxy)
    assert proxy.ping() == "pong"
    assert str(inspect.signature(proxy.ping)) == "(*args, **kwargs)"

    info = client.describe("StressTestModule.ping")
    assert info.signature is None
    assert info.documentation is None


@pytest.fixture
def stream_module(running_app, client):
    module = running_app._coordinator.deploy(StreamModule, instance_name="remote/streams")
    for name in ("messages", "commands", "state"):
        module.set_transport(name, pLCMTransport(f"/remote_{uuid.uuid4().hex}/{name}"))
    module.start()
    return module


@pytest.fixture
def collected():
    received = []
    event = threading.Event()

    def callback(message):
        received.append(message)
        event.set()

    return received, event, callback


def test_connect_discovers_only_wired_streams(running_app, client, stream_module):
    for app in (running_app, client):
        streams = app.list_streams("remote/streams")
        assert {(s.name, s.type_name, s.module_name, s.direction) for s in streams} == {
            ("messages", "bytes", "remote/streams", "out"),
            ("commands", "bytes", "remote/streams", "in"),
            ("state", "bytes", "remote/streams", "inout"),
        }
        assert all(s.channel.endswith(f"/{s.name}") for s in streams)
    assert client.list_streams(client.get_module("remote/streams")) == streams
    assert {s.module_name for s in client.list_streams()} >= {"remote/streams"}


@pytest.mark.parametrize("name", ["messages", "commands", "state"])
def test_connect_subscribes_to_every_stream_direction(client, stream_module, collected, name):
    module = client.get_module("remote/streams")
    stream = getattr(module, name)
    received, event, callback = collected
    with Disposable(stream.subscribe(callback)):
        stream_module.emit(name, b"hello")
        assert event.wait(timeout=5.0)
        assert received == [b"hello"]
    assert getattr(module, name) is stream
    with pytest.raises(AttributeError, match="missing"):
        _ = module.missing


def test_unsubscribe_and_client_stop_leave_daemon_running(
    running_app, client, stream_module, collected, wait_until
):
    stream = client.get_module("remote/streams").messages
    received, event, callback = collected
    unsubscribed = []
    with Disposable(stream.subscribe(callback)):
        with Disposable(stream.subscribe(unsubscribed.append)):
            stream_module.emit("messages", b"first")
            assert event.wait(timeout=5.0)
            wait_until(lambda: unsubscribed == [b"first"], timeout=5.0)
        event.clear()
        stream_module.emit("messages", b"second")
        assert event.wait(timeout=5.0)
        assert unsubscribed == [b"first"]
        assert received == [b"first", b"second"]
        client.stop()
        stream_module.emit("messages", b"after close")
    assert not client.is_running
    assert running_app.skills.ping() == "pong"
    assert received == [b"first", b"second"]


@pytest.fixture(params=[pLCMTransport, LCMTransport])
def custom_lcm_transport(request):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    args = () if request.param is pLCMTransport else (Vector3,)
    transport = request.param(
        f"/custom_{uuid.uuid4().hex}", *args, url=f"udpm://239.255.76.68:{port}?ttl=0"
    )
    try:
        yield transport
    finally:
        transport.stop()


def test_connected_stream_uses_configured_lcm_bus(
    running_app, client, custom_lcm_transport, collected
):
    running_app.run(VectorStreamModule)
    owner = running_app.VectorStreamModule
    owner.set_transport("messages", custom_lcm_transport)
    received, event, callback = collected
    with Disposable(client.VectorStreamModule.messages.subscribe(callback)):
        owner.emit(Vector3(1.0, 2.0, 3.0))
        assert event.wait(timeout=5.0)
        assert received == [Vector3(1.0, 2.0, 3.0)]


@pytest.fixture(params=[pSHMTransport, SHMTransport])
def shm_transport(request):
    transport = request.param(f"/early_{uuid.uuid4().hex}", default_capacity=1024)
    try:
        yield transport
    finally:
        transport.stop()


def test_early_shm_subscription_can_close_and_reconnect(
    running_app, client, stream_module, shm_transport, collected
):
    stream_module.set_transport("messages", shm_transport)
    received, event, callback = collected
    with Disposable(client.get_module("remote/streams").messages.subscribe(callback)):
        stream_module.emit("messages", b"first")
        assert event.wait(timeout=5.0)
        client.stop()
    second = Dimos.connect()
    try:
        event.clear()
        with Disposable(second.get_module("remote/streams").messages.subscribe(callback)):
            stream_module.emit("messages", b"second")
            assert event.wait(timeout=5.0)
            assert received == [b"first", b"second"]
    finally:
        second.stop()
