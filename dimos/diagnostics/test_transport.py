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

"""Real transport round trips, scoped to isolated local sessions."""

from contextlib import ExitStack
import socket
import threading

import pytest

from dimos.core.stream import TransportSpec
from dimos.core.transport import LCMTransport
from dimos.diagnostics.envelope import traced_channel
from dimos.diagnostics.model import context_scope, current_context
from dimos.diagnostics.runtime import Runtime
from dimos.diagnostics.traced_transport import TracedTransport
from dimos.msgs.std_msgs.String import String
from dimos.protocol.pubsub.impl.zenohpubsub import Topic, ZenohPubSubBase
from dimos.protocol.service.zenohservice import ZenohSessionPool


@pytest.fixture
def runtime(tmp_path, monkeypatch):
    rt = Runtime("run", tmp_path)
    monkeypatch.setattr("dimos.diagnostics.runtime._runtime", rt)
    monkeypatch.setattr("dimos.diagnostics.runtime._mode", "trace")
    yield rt
    rt.close()


def test_zenoh_native_attachment_survives_real_delivery(runtime):
    with ExitStack() as cleanup:
        pool = ZenohSessionPool()
        cleanup.callback(pool.close_all)
        ps = ZenohPubSubBase(
            session_pool=pool,
            mode="peer",
            connect=[],
            listen=[],
            scouting=False,
            multicast=False,
            gossip=False,
        )
        cleanup.callback(ps.stop)
        ps.start()
        received, ready = [], threading.Event()

        def receive(data, topic):
            received.append((data, current_context()))
            ready.set()

        cleanup.callback(ps.subscribe(Topic("dimos/trace_test"), receive))
        context = runtime.sample("hand")
        with context_scope(context):
            ps.publish(Topic("dimos/trace_test"), b"same")
        assert ready.wait(3)
        assert received == [(b"same", context)]


def test_legacy_zenoh_delivery_has_no_invented_parent(runtime):
    with ExitStack() as cleanup:
        pool = ZenohSessionPool()
        cleanup.callback(pool.close_all)
        ps = ZenohPubSubBase(
            session_pool=pool,
            mode="peer",
            connect=[],
            listen=[],
            scouting=False,
            multicast=False,
            gossip=False,
        )
        cleanup.callback(ps.stop)
        ps.start()
        received, ready = [], threading.Event()

        def receive(data, topic):
            received.append(current_context())
            ready.set()

        cleanup.callback(ps.subscribe(Topic("dimos/legacy_test"), receive))
        ps.publish(Topic("dimos/legacy_test"), b"legacy")
        assert ready.wait(3)
        assert received == [None]


def test_opt_in_lcm_carrier_round_trip_preserves_payload_and_context(runtime):
    channel = traced_channel("run", "arm")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    spec = TransportSpec(
        LCMTransport,
        kwargs={"topic": channel, "type": String, "url": f"udpm://239.255.76.67:{port}?ttl=0"},
    )
    transport = TracedTransport(spec, String, str, "run", "arm")
    peer = TracedTransport(spec, String, str, "run", "arm")
    received, ready = [], threading.Event()
    with ExitStack() as cleanup:
        cleanup.callback(transport.stop)
        cleanup.callback(peer.stop)
        peer.start()

        def receive(value):
            received.append((value, current_context()))
            ready.set()

        cleanup.callback(transport.subscribe(receive))
        transport.wait_ready()
        peer.wait_ready()
        context = runtime.sample("hand")
        with context_scope(context):
            transport.publish("hello")
        assert ready.wait(3)
        assert received == [("hello", context)]


def test_traced_link_refuses_legacy_channel():
    spec = TransportSpec(
        LCMTransport, kwargs={"topic": "actual_robot_command", "type": String, "url": "memq://"}
    )
    with pytest.raises(ValueError, match="dedicated"):
        TracedTransport(spec, String, str, "run", "arm")


def test_peer_mismatch_fails_during_setup_before_application_delivery():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    spec = TransportSpec(
        LCMTransport,
        kwargs={
            "topic": traced_channel("run", "mismatch"),
            "type": String,
            "url": f"udpm://239.255.76.67:{port}?ttl=0",
        },
    )
    first = TracedTransport(spec, String, str, "run", "mismatch")
    second = TracedTransport(spec, String, int, "run", "mismatch")
    received = []
    with ExitStack() as cleanup:
        cleanup.callback(first.stop)
        cleanup.callback(second.stop)
        first.subscribe(received.append)
        second.start()
        with pytest.raises(ValueError, match="incompatible"):
            first.wait_ready()
        with pytest.raises(RuntimeError, match="incompatible"):
            first.publish("should not arrive")
        assert received == []
