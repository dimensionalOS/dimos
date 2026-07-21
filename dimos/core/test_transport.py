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

from collections.abc import Callable, Iterator
import threading
from typing import Any, Literal

import numpy as np
import pytest

from dimos.core.transport import LCMTransport, PubSubTransport, ZenohTransport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.service.zenohservice import ZenohSessionPool

Backend = Literal["lcm", "zenoh"]


def _image_message() -> Image:
    return Image(np.array([[[1, 2, 3]]], dtype=np.uint8), frame_id="camera", ts=1.0)


def _pointcloud_message() -> PointCloud2:
    return PointCloud2.from_numpy(
        np.array([[1.0, 2.0, 3.0]], dtype=np.float32), frame_id="lidar", timestamp=1.0
    )


@pytest.fixture()
def heavy_transport_factory(
    lcm_url: str,
) -> Iterator[Callable[[Backend, type[Any]], PubSubTransport[Any]]]:
    """Create transports and release their single bus subscription after the test."""
    transports: list[PubSubTransport[Any]] = []
    session_pool = ZenohSessionPool()

    def create(backend: Backend, message_type: type[Any]) -> PubSubTransport[Any]:
        if backend == "lcm":
            transport = LCMTransport("/clause2/transport", message_type, url=lcm_url)
        else:
            transport = ZenohTransport(
                "dimos/clause2/transport", message_type, session_pool=session_pool
            )
        transports.append(transport)
        return transport

    yield create

    for transport in transports:
        transport.stop()
    session_pool.close_all()


@pytest.mark.parametrize("backend", ("lcm", "zenoh"))
@pytest.mark.parametrize("make_message", (_image_message, _pointcloud_message))
def test_clause2_heavy_transport_consumers_decode_after_typed_callback_entry(
    backend: Backend,
    heavy_transport_factory: Callable[[Backend, type[Any]], PubSubTransport[Any]],
    make_message: Callable[[], Image | PointCloud2],
    mocker,
    retry_until,
) -> None:
    """Clause 2 DIM-1125: heavy module consumers receive one decoded message after callback entry."""
    message = make_message()
    transport = heavy_transport_factory(backend, type(message))
    callback_entry_decode_counts: list[int] = []
    received_messages: list[Any] = []
    received = threading.Event()
    decode = mocker.spy(type(message), "lcm_decode")
    bus = transport.lcm if backend == "lcm" else transport.zenoh
    original_subscribe = bus.subscribe

    def subscribe(topic: Any, callback: Callable[[Any, Any], None]) -> Callable[[], None]:
        def observe_callback_entry(payload: Any, received_topic: Any) -> None:
            callback_entry_decode_counts.append(decode.call_count)
            callback(payload, received_topic)

        return original_subscribe(topic, observe_callback_entry)

    mocker.patch.object(bus, "subscribe", side_effect=subscribe)

    def receive(decoded_message: Any) -> None:
        received_messages.append(decoded_message)
        received.set()

    transport.subscribe(receive)
    retry_until(received, lambda: transport.publish(message))

    assert callback_entry_decode_counts == [0]
    assert decode.call_count == 1
    assert type(received_messages[0]) is type(message)
    assert received_messages[0].lcm_encode() == message.lcm_encode()
