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

"""Encode variants of one media route: one Zenoh subscriber, per-variant
writers and due times, one worker permit per encode."""

import asyncio
from collections.abc import AsyncIterator, Callable
from dataclasses import dataclass, field
import json
from pathlib import Path
import threading
import time
from typing import Any

from aioquic.quic.connection import QuicConnection
from dimos_lcm.sensor_msgs import Image
import pytest
from pytest_mock import MockerFixture

from dimos.utils.asyncio_timeout import timeout
from dimos.web.relay_bridge import topic_transform
from dimos.web.relay_bridge._wt_session import SessionProtocol, make_quic_configuration
from dimos.web.relay_bridge.protocol import DataFrame, decode_data_frame
from dimos.web.relay_bridge.topic_catalog import TopicCatalog
from dimos.web.relay_bridge.topic_mailbox import TopicIngressBudget, TopicSample
from dimos.web.relay_bridge.topic_profiles import normalize_settings
from dimos.web.relay_bridge.topic_protocol import TopicOpen
from dimos.web.relay_bridge.topic_service import _Route
from dimos.web.relay_bridge.topic_transform import TopicTransforms
from dimos.web.relay_bridge.topic_transport import LatestTopicWriter, TopicByteBudget

PROFILE = "image.jpeg.balanced.v1"
VECTOR = next(
    v
    for v in json.loads(
        (Path(__file__).parents[3] / "web/shared/fixtures/topic_media.json").read_text()
    )
    if v["name"] == "color_rgb_boxed"
)


class FakeSubscriber:
    undeclared = False

    def undeclare(self) -> None:
        self.undeclared = True


class FakeZenoh:
    def __init__(self) -> None:
        self.subscribers: list[FakeSubscriber] = []

    def declare_subscriber(self, expression: Any, callback: Any) -> FakeSubscriber:
        subscriber = FakeSubscriber()
        self.subscribers.append(subscriber)
        return subscriber


class FakeClient:
    """The two RelayClient methods a route uses, over a local QUIC session."""

    def __init__(self, session: SessionProtocol) -> None:
        self._session = session
        self.sent: list[Any] = []
        self.writers: list[LatestTopicWriter] = []

    def topic_writer(
        self, route: str, delivery: str, budget: TopicByteBudget, on_error: Callable[[str], None]
    ) -> LatestTopicWriter:
        assert delivery == "latest"
        writer = LatestTopicWriter(self._session, route, budget, on_error, stale_after=0.05)
        self.writers.append(writer)
        return writer

    def send_control_frame(self, message: Any) -> None:
        self.sent.append(message)


@dataclass
class Harness:
    zenoh: FakeZenoh
    client: FakeClient
    transforms: TopicTransforms
    frames: asyncio.Queue[DataFrame]
    make: Callable[[dict[str, Any] | None, float], _Route]
    routes: list[_Route] = field(default_factory=list)

    async def frame(self) -> DataFrame:
        async with timeout(5):
            return await self.frames.get()


def image(sequence: int) -> bytes:
    message = Image.lcm_decode(bytes.fromhex(VECTOR["lcm"]))
    message.header.seq = sequence
    return message.lcm_encode()


def sample(sequence: int, key: str = "sensor/cam") -> TopicSample:
    return TopicSample(key, "application/octet-stream", image(sequence), ingress_ts=time.time())


@pytest.fixture
async def harness(mocker: MockerFixture) -> AsyncIterator[Harness]:
    session = SessionProtocol(QuicConnection(configuration=make_quic_configuration(insecure=True)))
    mocker.patch.object(session, "transmit")
    session.open_session("relay.test", "/robot")
    frames: asyncio.Queue[DataFrame] = asyncio.Queue()
    real_send = session.send_frame_bytes

    def send(frame: bytes) -> int:
        frames.put_nowait(decode_data_frame(frame))
        return real_send(frame)

    mocker.patch.object(session, "send_frame_bytes", side_effect=send)
    client = FakeClient(session)
    zenoh = FakeZenoh()
    transforms = TopicTransforms()
    routes: list[_Route] = []

    def make(settings: dict[str, Any] | None, max_hz: float = 30) -> _Route:
        spec = TopicOpen(
            route="@t:p1",
            gen=1,
            keyExpr="sensor/cam",
            delivery="latest",
            profile=PROFILE,
            maxHz=max_hz,
        )
        route = _Route(
            zenoh,  # type: ignore[arg-type]
            client,  # type: ignore[arg-type]
            spec,
            settings,
            TopicIngressBudget(),
            TopicByteBudget(),
            TopicCatalog(),
            transforms,
            lambda _: None,
        )
        routes.append(route)
        return route

    try:
        yield Harness(zenoh, client, transforms, frames, make, routes)
    finally:
        for route in routes:
            await route.close()
        session.close()


async def test_variants_share_one_subscription_and_encode_once_each(
    harness: Harness, mocker: MockerFixture
) -> None:
    encode = mocker.spy(topic_transform, "_encode")
    route = harness.make(normalize_settings(PROFILE, {"maxHz": 30}))
    await route.start()
    assert route.set_variant("v2", {"maxHz": 30, "quality": 40}) == normalize_settings(
        PROFILE, {"quality": 40}
    )
    assert len(harness.zenoh.subscribers) == 1
    assert len(harness.client.writers) == 2
    route.activate()
    route._mailbox.offer(sample(1))
    frames = [await harness.frame() for _ in range(2)]
    by_variant = {frame.header.meta["variant"]: frame for frame in frames}
    assert sorted(by_variant) == ["v1", "v2"]
    assert by_variant["v1"].header.meta["quality"] == 75
    assert by_variant["v2"].header.meta["quality"] == 40
    assert {frame.header.seq for frame in frames} == {1, 2}
    assert encode.call_count == 2
    assert encode.call_args_list[0].args[2] != encode.call_args_list[1].args[2]


async def test_a_slow_variant_keeps_the_newest_sample_until_its_rate_makes_it_due(
    harness: Harness,
) -> None:
    route = harness.make(normalize_settings(PROFILE, {"maxHz": 30}), max_hz=30)
    await route.start()
    route.set_variant("v2", {"maxHz": 0.1, "quality": 40})
    assert route._mailbox._period == pytest.approx(1 / 30)
    route.activate()
    route._mailbox.offer(sample(1))
    first = [await harness.frame() for _ in range(2)]
    assert {frame.header.meta["variant"] for frame in first} == {"v1", "v2"}
    # Two more samples inside v2's ten second period: v1 encodes each as it
    # arrives, v2 holds the newest one.
    route._mailbox.offer(sample(2))
    second = await harness.frame()
    route._mailbox.offer(sample(3))
    third = await harness.frame()
    assert [(f.header.meta["variant"], f.header.meta["headerSeq"]) for f in (second, third)] == [
        ("v1", 2),
        ("v1", 3),
    ]
    assert harness.frames.empty()
    # Raising v2's rate keeps its last encode time, so the held sample is due
    # now rather than ten seconds after the first one.
    route.set_variant("v2", {"maxHz": 30, "quality": 40})
    held = await harness.frame()
    assert (held.header.meta["variant"], held.header.meta["headerSeq"]) == ("v2", 3)
    assert harness.frames.empty()


async def test_one_worker_permit_is_held_per_encode_not_per_route(
    harness: Harness, mocker: MockerFixture
) -> None:
    release = threading.Event()
    started = threading.Event()
    original = topic_transform._encode

    def slow(profile: str, data: bytes, settings: Any) -> Any:
        started.set()
        release.wait(5)
        return original(profile, data, settings)

    mocker.patch.object(topic_transform, "_encode", side_effect=slow)
    route = harness.make(normalize_settings(PROFILE, {"maxHz": 30}))
    await route.start()
    route.set_variant("v2", {"maxHz": 30, "quality": 40})
    route.activate()
    route._mailbox.offer(sample(1))
    await asyncio.to_thread(started.wait, 5)
    # One variant encodes; the other permit stays free for any other route.
    assert not harness.transforms.slots.locked()
    assert harness.transforms.slots._value == 1
    release.set()
    frames = [await harness.frame() for _ in range(2)]
    assert sorted(f.header.meta["variant"] for f in frames) == ["v1", "v2"]
    assert harness.transforms.slots._value == 2


async def test_closing_a_variant_frees_its_writer_and_the_route_drops_the_rest(
    harness: Harness,
) -> None:
    route = harness.make(normalize_settings(PROFILE, {"maxHz": 30}))
    await route.start()
    route.set_variant("v2", {"maxHz": 10, "quality": 40})
    first, second = harness.client.writers
    assert route.variants == {
        "v1": normalize_settings(PROFILE, {}),
        "v2": normalize_settings(PROFILE, {"maxHz": 10, "quality": 40}),
    }
    # A rate change keeps the writer; new encode settings on the slot swap it.
    route.set_variant("v2", {"maxHz": 20, "quality": 40})
    assert harness.client.writers == [first, second]
    route.set_variant("v2", {"maxHz": 20, "quality": 30})
    assert second._closed
    assert len(harness.client.writers) == 3
    await route.close_variant("v2")
    assert harness.client.writers[2]._closed
    assert list(route.variants) == ["v1"]
    assert not first._closed
    await route.close()
    assert first._closed
    assert harness.zenoh.subscribers[0].undeclared


async def test_an_older_relay_drives_the_only_variant_through_activation(
    harness: Harness,
) -> None:
    route = harness.make(None, max_hz=60)
    assert route.variants == {"v1": normalize_settings(PROFILE, {})}
    await route.start()
    route.activate(5)
    assert route.variants["v1"]["maxHz"] == 5
    assert route._mailbox._period == pytest.approx(0.2)
    route._mailbox.offer(sample(1))
    frame = await harness.frame()
    assert frame.header.meta["variant"] == "v1"
    assert frame.header.meta["quality"] == 75


async def test_a_variant_retired_while_encoding_drops_its_frame_and_keeps_the_route(
    harness: Harness, mocker: MockerFixture
) -> None:
    release = threading.Event()
    started = threading.Event()
    original = topic_transform._encode

    def slow(profile: str, data: bytes, settings: Any) -> Any:
        started.set()
        release.wait(5)
        return original(profile, data, settings)

    mocker.patch.object(topic_transform, "_encode", side_effect=slow)
    route = harness.make(normalize_settings(PROFILE, {"maxHz": 30}))
    await route.start()
    route.set_variant("v2", {"maxHz": 30, "quality": 40})
    route.activate()
    route._mailbox.offer(sample(1))
    await asyncio.to_thread(started.wait, 5)
    # v1 encodes first; its writer closes under it.
    await route.close_variant("v1")
    release.set()
    frame = await harness.frame()
    assert frame.header.meta["variant"] == "v2"
    assert route._error is None
    assert harness.client.sent == []
    assert harness.frames.empty()
