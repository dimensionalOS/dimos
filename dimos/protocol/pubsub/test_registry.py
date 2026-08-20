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

import pickle
import threading
import uuid

import pytest

from dimos.core.transport import (
    JpegLcmTransport,
    JpegShmTransport,
    LCMTransport,
    SHMTransport,
    pLCMTransport,
    pSHMTransport,
)
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.pubsub.registry import (
    make_pubsub_transport,
    parse_pubsub_uri,
    subscribe_pubsub_uri,
    supported_protos,
)
from dimos.protocol.pubsub.shm.ipc_factory import CpuShmQueue


def test_supported_protos_includes_known_set() -> None:
    """Registry exposes the canonical proto names."""
    assert set(supported_protos()) >= {"lcm", "jpeg_lcm", "plcm", "pshm", "shm", "jpeg_shm"}


@pytest.mark.parametrize(
    ("uri", "expected"),
    [
        ("lcm:/color_image", ("lcm", "/color_image", None)),
        ("jpeg_lcm:/color_image", ("jpeg_lcm", "/color_image", None)),
        ("pshm:color_image", ("pshm", "color_image", None)),
        ("shm:foo/bar", ("shm", "foo/bar", None)),
        (
            "lcm:/odom#nav_msgs.Odometry",
            ("lcm", "/odom", "nav_msgs.Odometry"),
        ),
        (
            "jpeg_lcm:/color_image#sensor_msgs.Image",
            ("jpeg_lcm", "/color_image", "sensor_msgs.Image"),
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
        "unknown_proto:/topic",
    ],
)
def test_parse_pubsub_uri_rejects_malformed(uri: str) -> None:
    with pytest.raises(ValueError):
        parse_pubsub_uri(uri)


def test_parse_pubsub_uri_error_lists_supported_protos() -> None:
    with pytest.raises(ValueError, match="supported:") as exc:
        parse_pubsub_uri("nope:/foo")
    msg = str(exc.value)
    for proto in ("lcm", "jpeg_lcm", "pshm"):
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


def test_make_pubsub_transport_pshm_uses_pSHMTransport() -> None:
    t = make_pubsub_transport("pshm:color_image")
    assert isinstance(t, pSHMTransport)


def test_reliable_shm_transport_uses_configured_ring() -> None:
    transport = pSHMTransport("reliable-test", queue_size=32, default_capacity=1024)

    assert transport.shm._channel_class is CpuShmQueue
    assert transport.shm._channel_kwargs == {"slots": 32}


def test_pshm_transport_rejects_non_positive_queue_size() -> None:
    with pytest.raises(ValueError, match="queue_size must be positive"):
        pSHMTransport("invalid", queue_size=0)


def test_pshm_transport_preserves_queue_size_when_pickled() -> None:
    restored = pickle.loads(
        pickle.dumps(pSHMTransport("pickled", queue_size=17, default_capacity=1024))
    )

    assert restored.shm.queue_size == 17
    assert restored.shm.config.default_capacity == 1024


def test_pshm_transport_reports_sequence_gap_when_ring_overflows() -> None:
    topic = f"overflow-{uuid.uuid4().hex}"
    publisher = pSHMTransport[int](topic, queue_size=2, default_capacity=1024)
    subscriber = pSHMTransport[int](topic, queue_size=2, default_capacity=1024)
    callback_started = threading.Event()
    release_callback = threading.Event()
    overflow_reported = threading.Event()
    errors: list[str] = []

    def receive(value: int) -> None:
        if value == 0:
            callback_started.set()
            assert release_callback.wait(timeout=1.0)

    def receive_error(error: BaseException) -> None:
        errors.append(str(error))
        overflow_reported.set()

    unsubscribe = subscriber.subscribe(receive)
    unsubscribe_errors = subscriber.subscribe_errors(receive_error)
    try:
        publisher.broadcast(None, 0)
        assert callback_started.wait(timeout=1.0)
        for value in range(1, 5):
            publisher.broadcast(None, value)
        release_callback.set()
        assert overflow_reported.wait(timeout=1.0)
        assert any("lost 2 message(s)" in error for error in errors)
    finally:
        release_callback.set()
        unsubscribe()
        unsubscribe_errors()
        publisher.stop()
        subscriber.stop()


def test_make_pubsub_transport_shm_uses_SHMTransport() -> None:
    t = make_pubsub_transport("shm:bytes_topic")
    assert isinstance(t, SHMTransport)


def test_make_pubsub_transport_jpeg_shm_uses_JpegShmTransport() -> None:
    # The Python `turbojpeg` package is importable even when the native
    # libturbojpeg.so is missing; the RuntimeError only fires when TurboJPEG()
    # is actually constructed. Probe by trying to instantiate it.
    turbojpeg = pytest.importorskip("turbojpeg")
    try:
        turbojpeg.TurboJPEG()
    except RuntimeError as exc:
        pytest.skip(f"libturbojpeg not available: {exc}")
    t = make_pubsub_transport("jpeg_shm:color_image")
    assert isinstance(t, JpegShmTransport)


def test_make_pubsub_transport_typed_proto_without_msg_type_raises() -> None:
    with pytest.raises(ValueError, match="requires a message type"):
        make_pubsub_transport("lcm:/color_image")


def test_make_pubsub_transport_uri_suffix_resolves_msg_type() -> None:
    """The '#' suffix is resolved via resolve_msg_type and used for typed protos."""
    t = make_pubsub_transport("lcm:/color_image#sensor_msgs.Image")
    assert isinstance(t, LCMTransport)


def test_make_pubsub_transport_unknown_msg_type_raises() -> None:
    with pytest.raises(ValueError, match="Could not resolve message type"):
        make_pubsub_transport("lcm:/x#not_a_module.NotAType")


def test_subscribe_pubsub_uri_returns_transport_and_unsubscribe(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """``subscribe_pubsub_uri`` starts the transport and wires the callback."""
    events: dict[str, object] = {"started": False, "callback": None, "unsubscribed": False}

    class _FakeTransport:
        def start(self) -> None:
            events["started"] = True

        def stop(self) -> None:
            events["stopped"] = True

        def subscribe(self, cb: object) -> object:
            events["callback"] = cb

            def _unsub() -> None:
                events["unsubscribed"] = True

            return _unsub

    monkeypatch.setattr(
        "dimos.protocol.pubsub.registry.make_pubsub_transport",
        lambda uri, *, msg_type=None: _FakeTransport(),
    )

    cb_calls: list[object] = []

    def _record(msg: object) -> None:
        cb_calls.append(msg)

    transport, unsub = subscribe_pubsub_uri("lcm:/x", _record, msg_type=Image)

    assert isinstance(transport, _FakeTransport)
    assert events["started"] is True
    # The registry wires the user's callback to the transport's subscribe verbatim.
    assert events["callback"] is _record
    events["callback"]("hello")  # type: ignore[operator]
    assert cb_calls == ["hello"]

    unsub()
    assert events["unsubscribed"] is True
