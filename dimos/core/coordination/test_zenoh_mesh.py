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

"""Coordinator/worker connectivity over explicit loopback zenoh endpoints.

With ``zenoh_scouting`` off, multicast scouting is pinned to the loopback
interface, which macOS never delivers on. The coordinator therefore meshes
itself and its workers over explicit TCP endpoints; these tests must pass
with multicast scouting fully dead.
"""

from collections.abc import Iterator
import time
from types import SimpleNamespace

import pytest
from reactivex.disposable import Disposable

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import pZenohTransport
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.service import zenohservice


def test_mesh_endpoints_feed_zenoh_config_defaults() -> None:
    endpoint = zenohservice.allocate_mesh_endpoint()
    assert endpoint.startswith("tcp/127.0.0.1:")

    sibling = "tcp/127.0.0.1:1"
    zenohservice.configure_zenoh_mesh(endpoint, (sibling,))
    try:
        config = zenohservice.ZenohConfig()
        assert config.listen == [endpoint]
        assert sibling in config.connect
    finally:
        zenohservice.configure_zenoh_mesh(None, ())

    config = zenohservice.ZenohConfig()
    assert config.listen == []
    assert sibling not in config.connect


def test_allocate_mesh_endpoint_never_reissues_a_port(monkeypatch: pytest.MonkeyPatch) -> None:
    """Back-to-back allocations must never hand out one port twice.

    A worker binds its port only once its interpreter has booted, and the
    kernel reissues a just-closed probe port -- without the dedup, twin
    workers race for one listener: the loser dies EADDRINUSE and the winner
    dials itself.
    """
    ports = iter([47843, 47843, 47843, 50000])

    class FakeSocket:
        def __init__(self) -> None:
            self._port = next(ports)
            self.closed = False
            created.append(self)

        def bind(self, addr: tuple[str, int]) -> None:
            pass

        def getsockname(self) -> tuple[str, int]:
            return ("127.0.0.1", self._port)

        def close(self) -> None:
            self.closed = True

    created: list[FakeSocket] = []

    monkeypatch.setattr(zenohservice, "socket", SimpleNamespace(socket=FakeSocket))
    monkeypatch.setattr(zenohservice, "_allocated_mesh_endpoints", set())

    assert zenohservice.allocate_mesh_endpoint() == "tcp/127.0.0.1:47843"
    assert zenohservice.allocate_mesh_endpoint() == "tcp/127.0.0.1:50000"
    assert all(sock.closed for sock in created)


def test_own_listen_endpoint_is_never_dialed() -> None:
    """A port collision must not turn into a session dialing its own listener."""
    endpoint = zenohservice.allocate_mesh_endpoint()
    sibling = "tcp/127.0.0.1:1"
    zenohservice.configure_zenoh_mesh(endpoint, (endpoint, sibling))
    try:
        config = zenohservice.ZenohConfig()
        assert endpoint not in config.connect
        assert sibling in config.connect
    finally:
        zenohservice.configure_zenoh_mesh(None, ())


def test_diverged_config_rides_the_mesh_session() -> None:
    """A config diverging after the mesh session opened must not rebind its port.

    A deploy's host-config sync can shift session settings after the eager
    mesh session opened (a robot_ip arriving at deploy time shifts the connect
    endpoints); the pool must hand back the session holding the mesh port
    instead of failing zenoh.open on a second bind.
    """
    pool = zenohservice.ZenohSessionPool()
    zenohservice.configure_zenoh_mesh(zenohservice.allocate_mesh_endpoint(), ())
    try:
        mesh_session = pool.acquire(zenohservice.ZenohConfig())
        diverged = zenohservice.ZenohConfig(connect=["tcp/10.11.12.13:7447"])
        assert diverged.session_key != zenohservice.ZenohConfig().session_key
        assert pool.acquire(diverged) is mesh_session
    finally:
        zenohservice.configure_zenoh_mesh(None, ())
        pool.close_all()


def test_released_mesh_closes_the_dialing_sessions() -> None:
    """Full worker-pool teardown must reclaim the sessions dialing its workers.

    Zenoh retries refused dials forever, so a long-lived process (a pytest
    worker, a daemon restarting blueprints) would otherwise accumulate one
    session per coordinator lifecycle, each burning threads and sockets on
    dead ports. A transiently emptied roster (the last worker being replaced
    during a restart) must NOT close anything: live RPC clients keep using
    the sessions they already hold.
    """
    worker = zenohservice.allocate_mesh_endpoint()
    zenohservice.configure_zenoh_mesh(None, (worker,))
    try:
        config = zenohservice.ZenohConfig()
        assert worker in config.connect
        session = zenohservice.default_session_pool.acquire(config)

        zenohservice.configure_zenoh_mesh(None, ())
        assert not session.is_closed()
    finally:
        zenohservice.configure_zenoh_mesh(None, (worker,))
        zenohservice.release_zenoh_mesh()

    assert session.is_closed()
    assert config.session_key not in zenohservice.default_session_pool._sessions


class MeshSource(Module):
    mesh_data: Out[Vector3]

    @rpc
    def emit(self, x: float) -> bool:
        self.mesh_data.publish(Vector3(x, 0.0, 0.0))
        return True


class MeshSink(Module):
    mesh_data: In[Vector3]

    received = 0

    @rpc
    def start(self) -> None:
        def _on_data(msg: Vector3) -> None:
            self.received += 1

        unsub = self.mesh_data.subscribe(_on_data)
        self.register_disposable(Disposable(unsub))


@pytest.fixture
def zenoh_transport(monkeypatch: pytest.MonkeyPatch) -> Iterator[None]:
    # Pin the parent's scouting to a nonexistent interface: zenoh logs an
    # error and scouts nothing, so the coordinator can only reach its workers
    # through the explicit mesh endpoints — on every platform, like on macOS
    # where loopback multicast is never delivered. (Workers are spawned via
    # forkserver and don't see this monkeypatch, which is fine: the mesh, not
    # scouting, is what this test must exercise.)
    monkeypatch.setattr(zenohservice, "LOOPBACK_INTERFACE", "dimosnoif0")
    previous = global_config.transport
    global_config.update(transport="zenoh")
    try:
        yield
    finally:
        global_config.update(transport=previous)


@pytest.mark.timeout(90)
def test_rpc_and_streams_over_explicit_mesh(zenoh_transport: None) -> None:
    coordinator = ModuleCoordinator()
    coordinator.start()
    try:
        source = coordinator.deploy(MeshSource)
        sink = coordinator.deploy(MeshSink)

        source.mesh_data.transport = pZenohTransport("dimos/test/mesh_data")
        sink.mesh_data.connect(source.mesh_data)

        # RPC round-trips prove coordinator->worker links; the counter rising
        # in the sink proves the worker->worker link carries pub/sub data.
        sink.start()
        deadline = time.monotonic() + 30.0
        sent = 0
        while time.monotonic() < deadline and sink.received < 3:
            assert source.emit(float(sent)) is True
            sent += 1
            time.sleep(0.1)

        assert sink.received >= 3
    finally:
        coordinator.stop()
