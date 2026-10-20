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

import pickle
import threading
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

pytest.importorskip("zmq")

from dimos.core.coordination.module_coordinator import _coerce_transport_to_backend
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import ZMQ_AVAILABLE, ZMQTransport, pZMQTransport
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.pubsub.registry import make_pubsub_transport
from dimos.protocol.service.zmqservice import close_proxies
from dimos.utils.testing.ports import free_zmq_endpoints
from dimos.utils.testing.waiting import wait_until


@pytest.fixture()
def endpoints():
    yield free_zmq_endpoints()
    close_proxies()


@pytest.fixture()
def collector():
    """Collect subscription messages; `event` fires when the first one arrives."""
    received = []
    event = threading.Event()

    def callback(msg):
        received.append(msg)
        event.set()

    return SimpleNamespace(received=received, event=event, callback=callback)


@pytest.fixture(params=["lcm", "zenoh"])
def any_backend(request, mocker):
    mocker.patch.object(global_config, "transport", request.param)
    return request.param


def test_zmq_available_reports_pyzmq() -> None:
    assert ZMQ_AVAILABLE is True


def test_zmq_transport_pickle_preserves_connection_settings(endpoints) -> None:
    transport = ZMQTransport("/roundtrip", Image, **endpoints, hwm=7, proxy="connect")
    restored = pickle.loads(pickle.dumps(transport))
    assert type(restored) is ZMQTransport
    assert restored.topic == transport.topic
    assert restored.zmq.config == transport.zmq.config
    assert restored.zmq.config.hwm == 7


def test_pzmq_transport_pickle_preserves_connection_settings(endpoints) -> None:
    transport = pZMQTransport("/roundtrip", **endpoints, settle_timeout=0.2)
    restored = pickle.loads(pickle.dumps(transport))
    assert type(restored) is pZMQTransport
    assert restored.topic == "/roundtrip"
    assert restored.zmq.config == transport.zmq.config


def test_zmq_transport_broadcast_and_subscribe(endpoints, collector) -> None:
    t = ZMQTransport("/test/transport", Image, **endpoints)
    t.start()
    try:
        t.subscribe(collector.callback)
        t.broadcast(None, Image(np.zeros((2, 2, 3), dtype=np.uint8)))
        assert collector.event.wait(timeout=2.0)
        assert isinstance(collector.received[0], Image)
    finally:
        t.stop()


def test_pzmq_transport_broadcast_and_subscribe(endpoints, collector) -> None:
    t = pZMQTransport("/test/pickle_transport", **endpoints)
    t.start()
    try:
        t.subscribe(collector.callback)
        t.broadcast(None, {"key": "value"})
        assert collector.event.wait(timeout=2.0)
        assert collector.received[0] == {"key": "value"}
    finally:
        t.stop()


def test_channel_is_the_typed_lcm_channel(endpoints) -> None:
    typed = ZMQTransport("/camera", Image, **endpoints)
    pickled = pZMQTransport("/anything", **endpoints)
    assert typed.channel == "/camera#sensor_msgs.Image"
    assert pickled.channel == "/anything"


def test_auto_start_on_broadcast(endpoints) -> None:
    t = pZMQTransport("/test/autostart", **endpoints)
    t.broadcast(None, "test")
    assert t._started
    t.stop()


def test_stop_and_restart(endpoints) -> None:
    t = pZMQTransport("/test/restart", **endpoints)
    t.start()
    assert t._started
    t.stop()
    assert not t._started
    t.start()
    assert t._started
    t.stop()


def test_coerce_leaves_deliberate_zmq_untouched(any_backend, endpoints) -> None:
    """The global lcm/zenoh switch never rebuilds an explicitly pinned ZMQ transport."""
    typed = ZMQTransport("/cmd_vel", Image, **endpoints)
    pickled = pZMQTransport("/human_input", **endpoints)
    assert _coerce_transport_to_backend(typed) is typed
    assert _coerce_transport_to_backend(pickled) is pickled


def test_registry_builds_zmq_transports() -> None:
    typed = make_pubsub_transport("zmq:/color_image#sensor_msgs.Image")
    pickled = make_pubsub_transport("pzmq:/anything", msg_type=Image)
    assert isinstance(typed, ZMQTransport)
    assert typed.topic.lcm_type is Image
    assert isinstance(pickled, pZMQTransport)
    assert pickled.channel == "/anything"


def test_registry_zmq_requires_a_message_type() -> None:
    with pytest.raises(ValueError, match="requires a message type"):
        make_pubsub_transport("zmq:/color_image")


class VecProducer(Module):
    vec: Out[Vector3]

    @rpc
    def emit(self, x: float) -> None:
        self.vec.publish(Vector3(x, 0.0, 0.0))


class VecConsumer(Module):
    vec: In[Vector3]

    def __init__(self, **kwargs: Any) -> None:
        self._got: list[float] = []
        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        super().start()
        self.vec.subscribe(lambda msg: self._got.append(msg.x))

    @rpc
    def received(self) -> list[float]:
        return list(self._got)


def test_modules_in_separate_workers_talk_over_zmq(dimos, endpoints) -> None:
    """The transport pickles into both workers; one hosts the proxy, the other joins."""
    producer = dimos.deploy(VecProducer)
    consumer = dimos.deploy(VecConsumer)
    producer.vec.transport = ZMQTransport("/vec", Vector3, **endpoints)
    consumer.vec.connect(producer.vec)
    dimos.start_all_modules()

    def emitted_and_received() -> bool:
        # The producer's worker learns of the consumer's subscription
        # asynchronously, so keep emitting until something lands.
        producer.emit(1.0)
        return 1.0 in consumer.received()

    wait_until(emitted_and_received, timeout=20.0, interval=0.1)
    producer.emit(2.0)
    wait_until(lambda: 2.0 in consumer.received(), timeout=5.0)
