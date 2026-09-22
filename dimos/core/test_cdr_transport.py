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

"""Generated CDR crosses actual transport implementations, including large images."""

from contextlib import ExitStack
import threading
import uuid

from dimos_generated.geometry_msgs.msg import Point
from dimos_generated.sensor_msgs.msg import Image
import numpy as np
import pytest

from dimos.core.transport import LCMTransport, SHMTransport, ZenohTransport
from dimos.protocol.service.zenohservice import ZenohSessionPool


@pytest.fixture(params=["lcm", "zenoh", "shm"])
def transport_pair(request, lcm_url):
    with ExitStack() as stack:
        if request.param == "zenoh":
            pool = ZenohSessionPool()
            stack.callback(pool.close_all)

        def make(message_type):
            topic = f"dimos/cdr/{uuid.uuid4().hex[:8]}"
            transports = []
            for _ in range(2):
                if request.param == "lcm":
                    transport = LCMTransport(topic, message_type, url=lcm_url)
                elif request.param == "zenoh":
                    transport = ZenohTransport(
                        topic,
                        message_type,
                        session_pool=pool,
                        scouting=False,
                        multicast=False,
                        gossip=False,
                        connect=[],
                    )
                else:
                    transport = SHMTransport(topic, message_type, prefer="cpu")
                stack.callback(transport.stop)
                transport.start()
                transports.append(transport)
            return transports

        yield make


def test_generated_message_round_trip(transport_pair, retry_until):
    publisher, subscriber = transport_pair(Point)
    received = []
    ready = threading.Event()

    def on_message(message):
        received.append((message.x, message.y, message.z))
        ready.set()

    subscriber.subscribe(on_message)
    retry_until(ready, lambda: publisher.broadcast(None, Point(x=1.25, y=-2.5, z=3)))

    assert received[0] == (1.25, -2.5, 3.0)


def test_large_generated_image_preserves_pixels_and_header(transport_pair, retry_until):
    publisher, subscriber = transport_pair(Image)
    pixels = np.arange(640 * 480 * 3, dtype=np.uint8)
    message = Image(width=640, height=480, encoding="rgb8", step=1920, data=pixels)
    message.header.frame_id = "camera"
    message.header.stamp.sec = 1_700_000_000
    message.header.stamp.nanosec = 123_456_789
    received = []
    ready = threading.Event()

    def on_message(value):
        received.append(value)
        ready.set()

    subscriber.subscribe(on_message)
    retry_until(ready, lambda: publisher.broadcast(None, message), timeout=5)

    decoded = received[0]
    assert (decoded.width, decoded.height, decoded.step, decoded.encoding) == (
        640,
        480,
        1920,
        "rgb8",
    )
    assert decoded.header.frame_id == "camera"
    assert (decoded.header.stamp.sec, decoded.header.stamp.nanosec) == (1_700_000_000, 123_456_789)
    np.testing.assert_array_equal(decoded.data.view(), pixels)
