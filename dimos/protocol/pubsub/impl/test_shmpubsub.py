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

from contextlib import ExitStack
from multiprocessing import get_context
import threading
from typing import Any
import uuid

from dimos_generated.geometry_msgs.msg import Vector3
import numpy as np
import pytest

from dimos.msgs.image import image_from_array, image_view
from dimos.protocol.pubsub.impl.jpeg_shm import JpegSharedMemory
from dimos.protocol.pubsub.impl.lcmpubsub import Topic
from dimos.protocol.pubsub.impl.shmpubsub import (
    BytesSharedMemory,
    CDRSharedMemory,
    PickleSharedMemory,
)
from dimos.protocol.pubsub.registry import make_pubsub_transport


@pytest.fixture
def buses():
    with ExitStack() as cleanup:

        def make(cls=BytesSharedMemory):
            bus = cls(prefer="cpu", default_capacity=4096)
            bus.start()
            cleanup.callback(bus.stop)
            return bus

        yield make


@pytest.fixture
def topic() -> str:
    return f"shm_test_{uuid.uuid4().hex}"


def test_new_subscriber_ignores_existing_frame_and_resubscribe_skips_gap(buses, topic):
    owner, reader = buses(), buses()
    owner.publish(topic, b"stale")
    got = []
    arrived = threading.Event()

    def receive(message, _topic):
        got.append(message)
        arrived.set()

    unsubscribe = reader.subscribe(topic, receive)
    assert not arrived.wait(0.05)
    owner.publish(topic, b"fresh")
    assert arrived.wait(2)
    assert got == [b"fresh"]
    unsubscribe()
    arrived.clear()
    owner.publish(topic, b"missed")
    reader.subscribe(topic, receive)
    assert not arrived.wait(0.05)
    owner.publish(topic, b"second")
    assert arrived.wait(2)
    assert got == [b"fresh", b"second"]


@pytest.mark.parametrize("payload", [b"hello", b""])
def test_bytes_between_instances(buses, topic, payload):
    owner, reader = buses(), buses()
    arrived = threading.Event()
    got = []

    def receive(message, channel):
        got.append((message, channel))
        arrived.set()

    reader.subscribe(topic, receive)
    owner.publish(topic, payload)
    assert arrived.wait(2)
    assert got == [(payload, topic)]


def test_generated_cdr_between_instances(buses, topic):
    owner, reader = buses(CDRSharedMemory), buses(CDRSharedMemory)
    typed_topic = Topic(topic, Vector3)
    arrived = threading.Event()
    got = []

    def receive(message, channel):
        got.append((message.x, message.y, message.z, channel.msg_type))
        arrived.set()

    reader.subscribe(typed_topic, receive)
    owner.publish(typed_topic, Vector3(x=1.25, y=-2.5, z=3.75))
    assert arrived.wait(2)
    assert got == [(1.25, -2.5, 3.75, Vector3)]


def test_jpeg_generated_image_between_instances(buses, topic):
    owner, reader = buses(JpegSharedMemory), buses(JpegSharedMemory)
    arrived = threading.Event()
    got = []

    def receive(message, _topic):
        got.append(message)
        arrived.set()

    reader.subscribe(topic, receive)
    message = image_from_array(np.full((4, 6, 3), 128, dtype=np.uint8), encoding="rgb8")
    owner.publish(topic, message)
    assert arrived.wait(2)
    assert got[0].encoding == "bgr8"
    np.testing.assert_array_equal(image_view(got[0]), np.full((4, 6, 3), 128, dtype=np.uint8))


def _publish_child(topic: str, pipe: Any) -> None:
    bus = PickleSharedMemory(prefer="cpu", default_capacity=4096)
    try:
        bus.start()
        bus.publish(topic, "stale")
        pipe.send("ready")
        assert pipe.recv() == "publish"
        bus.publish(topic, {"value": [1, 2, 3]})
        pipe.send("published")
        assert pipe.recv() == "stop"
    finally:
        bus.stop()
        pipe.close()


@pytest.fixture
def publisher_process(topic):
    context = get_context("fork")
    parent, child = context.Pipe()
    process = context.Process(target=_publish_child, args=(topic, child))
    process.start()
    child.close()
    try:
        assert parent.poll(5)
        assert parent.recv() == "ready"
        yield process, parent
    finally:
        if process.is_alive():
            parent.send("stop")
            process.join(5)
        if process.is_alive():
            process.terminate()
            process.join(5)
        parent.close()


def test_pickle_across_processes(buses, topic, publisher_process):
    process, pipe = publisher_process
    reader = buses(PickleSharedMemory)
    arrived = threading.Event()
    got = []

    def receive(message, _topic):
        got.append(message)
        arrived.set()

    reader.subscribe(topic, receive)
    pipe.send("publish")
    assert pipe.poll(5)
    assert pipe.recv() == "published"
    assert arrived.wait(2)
    assert got == [{"value": [1, 2, 3]}]
    pipe.send("stop")
    process.join(5)
    assert process.exitcode == 0


@pytest.fixture
def uri_transports(topic):
    with ExitStack() as cleanup:
        transports = [
            make_pubsub_transport(f"shm:{topic}#geometry_msgs/msg/Vector3") for _ in range(2)
        ]
        for transport in transports:
            transport.start()
            cleanup.callback(transport.stop)
        yield transports


def test_registry_cdr_roundtrip(uri_transports):
    publisher, subscriber = uri_transports
    arrived = threading.Event()
    got = []

    def receive(message):
        got.append((message.x, message.y, message.z))
        arrived.set()

    unsubscribe = subscriber.subscribe(receive)
    publisher.broadcast(None, Vector3(x=2.5, y=-3.5, z=4.5))
    assert arrived.wait(2)
    assert got == [(2.5, -3.5, 4.5)]
    unsubscribe()
