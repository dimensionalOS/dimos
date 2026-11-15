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

import threading
from types import SimpleNamespace

import pytest

from dimos.core.transport import LCMTransport, ZenohTransport
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.encode.cbor import decode, encode
from dimos.protocol.pubsub.benchmarks.codec import CompressedCodec
from dimos.protocol.pubsub.benchmarks.image_roundtrip import make_image
from dimos.protocol.service.zenohservice import ZenohSessionPool

pytestmark = pytest.mark.skipif_no_turbojpeg


@pytest.fixture()
def collector():
    received = []
    event = threading.Event()

    def callback(msg):
        received.append(msg)
        event.set()

    return SimpleNamespace(received=received, event=event, callback=callback)


@pytest.fixture()
def session_pool():
    pool = ZenohSessionPool()
    yield pool
    pool.close_all()


def test_roundtrip_over_lcm(retry_until, collector) -> None:
    t = CompressedCodec(LCMTransport("dimos/test/codec_lcm", CompressedImage))
    with t:
        t.subscribe(collector.callback)
        src = make_image(320, 240)
        retry_until(collector.event, lambda: t.broadcast(None, src))
        img = collector.received[0]
        assert isinstance(img, Image)
        assert img.frame_id == "cam"
        assert abs(img.ts - src.ts) < 1e-6
        assert img.shape == src.shape


def test_roundtrip_over_zenoh(retry_until, collector, session_pool) -> None:
    t = CompressedCodec(
        ZenohTransport("dimos/test/codec_zenoh", CompressedImage, session_pool=session_pool)
    )
    with t:
        t.subscribe(collector.callback)
        src = make_image(320, 240)
        retry_until(collector.event, lambda: t.broadcast(None, src))
        img = collector.received[0]
        assert isinstance(img, Image)
        assert abs(img.ts - src.ts) < 1e-6


def test_cbor_roundtrip() -> None:
    """The bench pins the codec into blueprints — it must cross into forkserver workers."""
    t = CompressedCodec(LCMTransport("dimos/test/codec_pickle", CompressedImage), quality=50)
    t2 = decode(encode(t))
    assert isinstance(t2, CompressedCodec)
    assert t2.quality == 50
    assert t2.topic.topic == t.topic.topic
