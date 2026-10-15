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

from __future__ import annotations

from contextlib import ExitStack
import threading

import numpy as np
import pytest

from dimos.core.global_config import GlobalConfig
from dimos.core.transport import (
    JpegLcmTransport,
    LCMTransport,
    PubSubTransport,
    ZenohTransport,
    pLCMTransport,
    pZenohTransport,
)
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.protocol.pubsub.registry import (
    make_pubsub_transport,
    parse_pubsub_uri,
    subscribe_pubsub_uri,
    supported_protos,
)
import dimos.protocol.service.zenohservice as zenohservice
from dimos.protocol.service.zenohservice import ZenohSessionPool


@pytest.fixture
def session_pool(monkeypatch):
    pool = ZenohSessionPool()
    monkeypatch.setattr(zenohservice, "default_session_pool", pool)
    yield pool
    pool.close_all()


def test_supported_protos_includes_known_set() -> None:
    """Registry exposes the canonical proto names."""
    assert supported_protos() == ["jpeg_lcm", "lcm", "plcm", "pzenoh", "zenoh"]


@pytest.mark.parametrize(
    ("uri", "expected"),
    [
        ("lcm:/color_image", ("lcm", "/color_image", None)),
        ("jpeg_lcm:/color_image", ("jpeg_lcm", "/color_image", None)),
        ("zenoh:dimos/color_image", ("zenoh", "dimos/color_image", None)),
        ("pzenoh:foo/bar", ("pzenoh", "foo/bar", None)),
        (
            "lcm:/odom#nav_msgs.Odometry",
            ("lcm", "/odom", "nav_msgs.Odometry"),
        ),
        (
            "jpeg_lcm:/color_image#sensor_msgs.Image",
            ("jpeg_lcm", "/color_image", "sensor_msgs.Image"),
        ),
        (
            "zenoh:dimos/color_image#sensor_msgs.Image",
            ("zenoh", "dimos/color_image", "sensor_msgs.Image"),
        ),
    ],
)
def test_parse_pubsub_uri_happy_paths(uri: str, expected: tuple[str, str, str | None]) -> None:
    assert parse_pubsub_uri(uri) == expected


@pytest.mark.parametrize(
    "uri",
    [
        "",
        "no-colon-here",
        ":/topic-with-empty-proto",
        "lcm:",  # empty topic
        "zenoh:",
        "pzenoh:",
        "unknown_proto:/topic",
    ],
)
def test_parse_pubsub_uri_rejects_malformed(uri: str) -> None:
    with pytest.raises(ValueError):
        parse_pubsub_uri(uri)


@pytest.mark.parametrize("proto", ["shm", "pshm", "jpeg_shm"])
def test_make_pubsub_transport_rejects_removed_protocols(proto) -> None:
    with pytest.raises(ValueError, match=f"Unsupported proto '{proto}'"):
        make_pubsub_transport(f"{proto}:color_image", msg_type=Image)


def test_parse_pubsub_uri_error_lists_supported_protos() -> None:
    with pytest.raises(ValueError, match="supported:") as exc:
        parse_pubsub_uri("nope:/foo")
    msg = str(exc.value)
    for proto in ("lcm", "jpeg_lcm", "zenoh", "pzenoh"):
        assert proto in msg


def test_make_pubsub_transport_lcm_uses_LCMTransport() -> None:
    t = make_pubsub_transport("lcm:/color_image", msg_type=Image)
    assert isinstance(t, LCMTransport)


def test_make_pubsub_transport_jpeg_lcm_uses_JpegLcmTransport() -> None:
    t = make_pubsub_transport("jpeg_lcm:/color_image", msg_type=Image)
    assert isinstance(t, JpegLcmTransport)


def test_make_pubsub_transport_plcm_uses_pLCMTransport() -> None:
    t = make_pubsub_transport("plcm:/anything")
    assert isinstance(t, pLCMTransport)


def test_make_pubsub_transport_zenoh_matches_default_image_channel() -> None:
    t = make_pubsub_transport("zenoh:dimos/color_image", msg_type=Image)
    default = make_transport("color_image", Image, g=GlobalConfig(transport="zenoh"))

    assert isinstance(t, ZenohTransport)
    assert t.channel == default.channel == "dimos/color_image/sensor_msgs.Image"


def test_make_pubsub_transport_pzenoh_ignores_msg_type() -> None:
    t = make_pubsub_transport("pzenoh:dimos/anything", msg_type=Image)

    assert isinstance(t, pZenohTransport)
    assert t.channel == "dimos/anything"


@pytest.mark.parametrize("proto", ["lcm", "jpeg_lcm", "zenoh"])
def test_make_pubsub_transport_typed_proto_without_msg_type_raises(proto) -> None:
    with pytest.raises(ValueError, match="requires a message type"):
        make_pubsub_transport(f"{proto}:color_image")


@pytest.mark.parametrize("proto", ["lcm", "jpeg_lcm", "zenoh"])
@pytest.mark.parametrize("msg_type", [None, Vector3])
def test_make_pubsub_transport_uri_suffix_resolves_and_overrides_msg_type(proto, msg_type) -> None:
    t = make_pubsub_transport(f"{proto}:color_image#sensor_msgs.Image", msg_type=msg_type)

    assert t.topic.lcm_type is Image


@pytest.mark.parametrize("proto", ["lcm", "zenoh"])
def test_make_pubsub_transport_unknown_msg_type_raises(proto) -> None:
    with pytest.raises(ValueError, match="Could not resolve message type"):
        make_pubsub_transport(f"{proto}:x#not_a_module.NotAType")


def test_subscribe_pubsub_uri_returns_transport_and_unsubscribe(mocker) -> None:
    """``subscribe_pubsub_uri`` starts the transport and wires the callback."""
    transport = mocker.Mock(spec=PubSubTransport)
    factory = mocker.patch(
        "dimos.protocol.pubsub.registry.make_pubsub_transport",
        return_value=transport,
    )
    callback = mocker.Mock()

    result, unsub = subscribe_pubsub_uri("zenoh:dimos/color_image", callback, msg_type=Image)

    assert result is transport
    factory.assert_called_once_with("zenoh:dimos/color_image", msg_type=Image)
    transport.start.assert_called_once_with()
    transport.subscribe.assert_called_once_with(callback)
    unsub()
    transport.subscribe.return_value.assert_called_once_with()


@pytest.mark.parametrize(
    ("uri", "msg_type"),
    [("zenoh:dimos/registry_image", Image), ("pzenoh:dimos/registry_image", None)],
)
def test_zenoh_uri_receives_image_from_standard_publisher(uri, msg_type, session_pool) -> None:
    message = Image(
        data=np.array([[[10, 20, 30], [40, 50, 60]]], dtype=np.uint8),
        format=ImageFormat.RGB,
        ts=12.5,
        frame_id="camera",
    )
    received: list[Image] = []
    arrived = threading.Event()

    def on_image(image: Image) -> None:
        received.append(image)
        arrived.set()

    with ExitStack() as cleanup:
        publisher = make_transport("registry_image", msg_type, g=GlobalConfig(transport="zenoh"))
        cleanup.callback(publisher.stop)
        subscriber, unsub = subscribe_pubsub_uri(uri, on_image, msg_type=msg_type)
        cleanup.callback(subscriber.stop)
        cleanup.callback(unsub)

        publisher.broadcast(None, message)

        assert arrived.wait(timeout=5.0)
        assert len(received) == 1
        np.testing.assert_array_equal(received[0].data, message.data)
        assert received[0].format == ImageFormat.RGB
        assert received[0].ts == 12.5
        assert received[0].frame_id == "camera"
